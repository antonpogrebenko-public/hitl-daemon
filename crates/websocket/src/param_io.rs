//! Parameter values flowing back from PX4.
//!
//! The daemon used to broadcast acks as a bare `(String, f32)` tuple, which was
//! enough while every use was "did the value I just wrote come back". Snapshot
//! and restore need more: PX4 silently drops a `PARAM_SET` whose `param_type`
//! does not match the parameter's declared type, so a snapshot that records
//! only name and value cannot be replayed onto the board. The type travels with
//! the value from the moment it arrives.

use crossbeam_channel::Sender;
use mavlink::ardupilotmega::{MavMessage, MavParamType, PARAM_REQUEST_READ_DATA, PARAM_VALUE_DATA};
use std::time::Duration;
use tokio::sync::broadcast;
use tracing::warn;

/// One `PARAM_VALUE` as reported by the autopilot.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamValue {
    pub name: String,
    pub value: f32,
    /// PX4's own declaration of the parameter's type. Written back verbatim on
    /// restore rather than inferred from the value, because an INT32 parameter
    /// holding 0 is indistinguishable from a REAL32 one by value alone.
    pub param_type: MavParamType,
    /// Index within the autopilot's parameter table. Carried for diagnostics
    /// and for detecting a truncated bulk read.
    pub index: u16,
}

impl ParamValue {
    /// Decode a `PARAM_VALUE`. Returns `None` when the name is empty, which is
    /// how a malformed or padding-only `param_id` presents.
    pub fn from_mavlink(pv: &PARAM_VALUE_DATA) -> Option<Self> {
        let name = decode_param_id(&pv.param_id);
        if name.is_empty() {
            return None;
        }
        Some(Self {
            name,
            value: pv.param_value,
            param_type: pv.param_type,
            index: pv.param_index,
        })
    }

    /// Whether PX4 declared this parameter as an integer type. Restore uses
    /// this to choose between the INT32 and REAL32 `PARAM_SET` encodings.
    pub fn is_integer(&self) -> bool {
        matches!(
            self.param_type,
            MavParamType::MAV_PARAM_TYPE_UINT8
                | MavParamType::MAV_PARAM_TYPE_INT8
                | MavParamType::MAV_PARAM_TYPE_UINT16
                | MavParamType::MAV_PARAM_TYPE_INT16
                | MavParamType::MAV_PARAM_TYPE_UINT32
                | MavParamType::MAV_PARAM_TYPE_INT32
                | MavParamType::MAV_PARAM_TYPE_UINT64
                | MavParamType::MAV_PARAM_TYPE_INT64
        )
    }
}

/// MAVLink pads `param_id` to 16 bytes with NULs; a name that fills the buffer
/// exactly has no terminator at all.
pub fn decode_param_id(param_id: &[u8; 16]) -> String {
    std::str::from_utf8(param_id)
        .unwrap_or("")
        .trim_end_matches('\0')
        .to_string()
}

/// How long to wait for a `PARAM_VALUE` reply to a single read request.
/// Matches the write path's ack timeout — the round trip is the same shape.
pub const PARAM_READ_TIMEOUT: Duration = Duration::from_millis(800);

/// Read attempts per parameter before giving up. PX4 drops requests under load
/// and the link is lossy, so a single unanswered request is not evidence the
/// parameter is absent.
pub const PARAM_READ_RETRY_COUNT: u8 = 3;

/// Default MAVLink routing for the connected PX4 autopilot.
const PX4_TARGET_SYSTEM: u8 = 1;
const PX4_TARGET_COMPONENT: u8 = 1;

/// Why a read did not produce a value.
#[derive(Debug, Clone, PartialEq)]
pub enum ParamReadError {
    /// No reply after every retry. The parameter may not exist on this
    /// firmware, or the link may have dropped the exchange.
    NoReply,
    /// The MAVLink writer is gone — the FC disconnected mid-read.
    LinkClosed,
}

/// Build a `PARAM_REQUEST_READ` addressed by name.
///
/// `param_index` is -1, which tells PX4 to resolve by `param_id` instead of by
/// table position. Indices shift between firmware builds; names do not.
pub fn make_param_request_read(name: &str) -> MavMessage {
    let mut param_id = [0u8; 16];
    let bytes = name.as_bytes();
    let copy_len = bytes.len().min(param_id.len());
    param_id[..copy_len].copy_from_slice(&bytes[..copy_len]);
    MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
        param_index: -1,
        target_system: PX4_TARGET_SYSTEM,
        target_component: PX4_TARGET_COMPONENT,
        param_id,
    })
}

/// Read one parameter by name, retrying on silence.
///
/// Subscribes before sending so a fast reply cannot land in the gap between
/// the two. Unrelated `PARAM_VALUE` traffic (a QGC parameter pull, another
/// read in flight) is drained rather than treated as a mismatch.
pub async fn read_param(
    mav_tx: &Sender<MavMessage>,
    param_value_tx: &broadcast::Sender<ParamValue>,
    name: &str,
) -> Result<ParamValue, ParamReadError> {
    read_param_with(mav_tx, param_value_tx, name, ParamReadPolicy::default()).await
}

/// Timeout and retry budget for a read. Injectable so tests can exercise the
/// failure paths without waiting out the production budget.
#[derive(Debug, Clone, Copy)]
pub struct ParamReadPolicy {
    pub timeout: Duration,
    pub retries: u8,
}

impl Default for ParamReadPolicy {
    fn default() -> Self {
        Self {
            timeout: PARAM_READ_TIMEOUT,
            retries: PARAM_READ_RETRY_COUNT,
        }
    }
}

pub async fn read_param_with(
    mav_tx: &Sender<MavMessage>,
    param_value_tx: &broadcast::Sender<ParamValue>,
    name: &str,
    policy: ParamReadPolicy,
) -> Result<ParamValue, ParamReadError> {
    for attempt in 1..=policy.retries {
        let mut rx = param_value_tx.subscribe();

        match mav_tx.try_send(make_param_request_read(name)) {
            Ok(()) => {}
            Err(crossbeam_channel::TrySendError::Full(_)) => {
                warn!(
                    param = name,
                    attempt, "MAVLink tx full — retrying param read"
                );
                tokio::time::sleep(Duration::from_millis(50)).await;
                continue;
            }
            Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                return Err(ParamReadError::LinkClosed);
            }
        }

        if let Some(value) = await_param(&mut rx, name, policy.timeout).await {
            return Ok(value);
        }
        warn!(
            param = name,
            attempt, "PARAM_VALUE read timed out — retrying"
        );
    }
    Err(ParamReadError::NoReply)
}

/// Read every named parameter, reporting which ones produced nothing.
///
/// Partial success is returned rather than aborting on the first miss: the
/// caller needs the full list of failures to tell the user which parameters
/// could not be captured, not just the first.
pub async fn read_params(
    mav_tx: &Sender<MavMessage>,
    param_value_tx: &broadcast::Sender<ParamValue>,
    names: &[&str],
) -> (Vec<ParamValue>, Vec<String>) {
    read_params_with(mav_tx, param_value_tx, names, ParamReadPolicy::default()).await
}

pub async fn read_params_with(
    mav_tx: &Sender<MavMessage>,
    param_value_tx: &broadcast::Sender<ParamValue>,
    names: &[&str],
    policy: ParamReadPolicy,
) -> (Vec<ParamValue>, Vec<String>) {
    let mut values = Vec::with_capacity(names.len());
    let mut failed = Vec::new();

    for name in names {
        match read_param_with(mav_tx, param_value_tx, name, policy).await {
            Ok(value) => values.push(value),
            Err(_) => failed.push((*name).to_string()),
        }
    }

    (values, failed)
}

/// Drain `rx` until a `PARAM_VALUE` for `name` arrives or the timeout elapses.
async fn await_param(
    rx: &mut broadcast::Receiver<ParamValue>,
    name: &str,
    timeout: Duration,
) -> Option<ParamValue> {
    let deadline = tokio::time::Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            return None;
        }
        match tokio::time::timeout(remaining, rx.recv()).await {
            Ok(Ok(pv)) if pv.name == name => return Some(pv),
            Ok(Ok(_)) => {}
            Ok(Err(broadcast::error::RecvError::Lagged(n))) => {
                warn!(
                    param = name,
                    lagged = n,
                    "PARAM_VALUE receiver lagged during read"
                );
            }
            Ok(Err(broadcast::error::RecvError::Closed)) => return None,
            Err(_) => return None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn param_id(name: &str) -> [u8; 16] {
        let mut buf = [0u8; 16];
        let bytes = name.as_bytes();
        let n = bytes.len().min(16);
        buf[..n].copy_from_slice(&bytes[..n]);
        buf
    }

    fn param_value(name: &str, value: f32, param_type: MavParamType) -> PARAM_VALUE_DATA {
        PARAM_VALUE_DATA {
            param_value: value,
            param_count: 900,
            param_index: 42,
            param_id: param_id(name),
            param_type,
        }
    }

    #[test]
    fn decodes_name_value_type_and_index() {
        let pv = param_value("SYS_HITL", 1.0, MavParamType::MAV_PARAM_TYPE_INT32);
        let decoded = ParamValue::from_mavlink(&pv).expect("well-formed PARAM_VALUE");
        assert_eq!(decoded.name, "SYS_HITL");
        assert_eq!(decoded.value, 1.0);
        assert_eq!(decoded.param_type, MavParamType::MAV_PARAM_TYPE_INT32);
        assert_eq!(decoded.index, 42);
    }

    #[test]
    fn decodes_a_name_that_fills_the_buffer_exactly() {
        // 16 chars, so there is no NUL terminator to trim.
        let name = "ABCDEFGHIJKLMNOP";
        let pv = param_value(name, 0.0, MavParamType::MAV_PARAM_TYPE_REAL32);
        let decoded = ParamValue::from_mavlink(&pv).expect("well-formed PARAM_VALUE");
        assert_eq!(decoded.name, name);
    }

    #[test]
    fn empty_name_is_rejected() {
        let pv = param_value("", 0.0, MavParamType::MAV_PARAM_TYPE_REAL32);
        assert!(ParamValue::from_mavlink(&pv).is_none());
    }

    #[test]
    fn integer_and_real_types_are_distinguished() {
        // The distinction PX4 enforces: an INT32 written as REAL32 is dropped
        // silently, with no PARAM_VALUE reply at all.
        let int_param = ParamValue::from_mavlink(&param_value(
            "SYS_AUTOSTART",
            4001.0,
            MavParamType::MAV_PARAM_TYPE_INT32,
        ))
        .unwrap();
        let real_param = ParamValue::from_mavlink(&param_value(
            "EKF2_REQ_HDRIFT",
            0.3,
            MavParamType::MAV_PARAM_TYPE_REAL32,
        ))
        .unwrap();
        assert!(int_param.is_integer());
        assert!(!real_param.is_integer());
    }

    #[test]
    fn zero_valued_int_is_not_mistaken_for_a_real() {
        // Value alone cannot tell these apart, which is the whole reason the
        // type is carried rather than inferred at restore time.
        let as_int =
            ParamValue::from_mavlink(&param_value("A", 0.0, MavParamType::MAV_PARAM_TYPE_INT32))
                .unwrap();
        let as_real =
            ParamValue::from_mavlink(&param_value("A", 0.0, MavParamType::MAV_PARAM_TYPE_REAL32))
                .unwrap();
        assert_eq!(as_int.value, as_real.value);
        assert_ne!(as_int.param_type, as_real.param_type);
    }
}

#[cfg(test)]
mod read_tests {
    use super::*;
    use crossbeam_channel::bounded;

    fn param_id_buf(name: &str) -> [u8; 16] {
        let mut buf = [0u8; 16];
        let bytes = name.as_bytes();
        let n = bytes.len().min(16);
        buf[..n].copy_from_slice(&bytes[..n]);
        buf
    }

    /// Answers PARAM_REQUEST_READ the way PX4 does: one PARAM_VALUE carrying
    /// the parameter's declared type. `answer_after` requests are ignored first,
    /// to model a lossy link.
    fn spawn_fake_px4(
        mav_rx: crossbeam_channel::Receiver<MavMessage>,
        param_value_tx: broadcast::Sender<ParamValue>,
        table: Vec<(&'static str, f32, MavParamType)>,
        ignore_first: usize,
    ) -> tokio::task::JoinHandle<()> {
        tokio::task::spawn_blocking(move || {
            let mut ignored = 0;
            while let Ok(msg) = mav_rx.recv() {
                let MavMessage::PARAM_REQUEST_READ(req) = msg else {
                    continue;
                };
                if ignored < ignore_first {
                    ignored += 1;
                    continue;
                }
                let requested = decode_param_id(&req.param_id);
                if let Some((name, value, ptype)) = table.iter().find(|(n, _, _)| *n == requested) {
                    let pv = PARAM_VALUE_DATA {
                        param_value: *value,
                        param_count: table.len() as u16,
                        param_index: 7,
                        param_id: param_id_buf(name),
                        param_type: *ptype,
                    };
                    let _ = param_value_tx.send(ParamValue::from_mavlink(&pv).unwrap());
                }
            }
        })
    }

    #[tokio::test]
    async fn reads_a_parameter_and_returns_its_declared_type() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
        let _px4 = spawn_fake_px4(
            mav_rx,
            pv_tx.clone(),
            vec![("SYS_HITL", 0.0, MavParamType::MAV_PARAM_TYPE_INT32)],
            0,
        );

        let value = read_param(&mav_tx, &pv_tx, "SYS_HITL").await.unwrap();
        assert_eq!(value.name, "SYS_HITL");
        assert_eq!(value.value, 0.0);
        // The point of the read: a zero-valued INT32 is indistinguishable from
        // a REAL32 by value, so the type has to come from PX4.
        assert_eq!(value.param_type, MavParamType::MAV_PARAM_TYPE_INT32);
        assert!(value.is_integer());
    }

    #[tokio::test]
    async fn request_addresses_by_name_not_index() {
        // Parameter indices shift between firmware builds; addressing by index
        // would read the wrong parameter after a firmware update.
        let MavMessage::PARAM_REQUEST_READ(req) = make_param_request_read("EKF2_REQ_HDRIFT") else {
            panic!("expected PARAM_REQUEST_READ");
        };
        assert_eq!(req.param_index, -1);
        assert_eq!(decode_param_id(&req.param_id), "EKF2_REQ_HDRIFT");
    }

    #[tokio::test]
    async fn retries_when_the_first_request_is_dropped() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
        let _px4 = spawn_fake_px4(
            mav_rx,
            pv_tx.clone(),
            vec![("COM_ARM_SDCARD", 1.0, MavParamType::MAV_PARAM_TYPE_INT32)],
            1,
        );

        let value = read_param(&mav_tx, &pv_tx, "COM_ARM_SDCARD").await.unwrap();
        assert_eq!(value.value, 1.0);
    }

    #[tokio::test]
    async fn unknown_parameter_reports_no_reply() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
        let _px4 = spawn_fake_px4(mav_rx, pv_tx.clone(), vec![], 0);

        let result = read_param(&mav_tx, &pv_tx, "NOT_A_PARAM").await;
        assert_eq!(result, Err(ParamReadError::NoReply));
    }

    #[tokio::test]
    async fn bulk_read_reports_which_names_failed() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
        let _px4 = spawn_fake_px4(
            mav_rx,
            pv_tx.clone(),
            vec![
                ("SYS_HITL", 0.0, MavParamType::MAV_PARAM_TYPE_INT32),
                ("EKF2_REQ_HDRIFT", 0.3, MavParamType::MAV_PARAM_TYPE_REAL32),
            ],
            0,
        );

        let (values, failed) = read_params(
            &mav_tx,
            &pv_tx,
            &["SYS_HITL", "GHOST_PARAM", "EKF2_REQ_HDRIFT"],
        )
        .await;

        assert_eq!(values.len(), 2);
        // The caller has to be able to tell the user every parameter that could
        // not be captured, not just the first.
        assert_eq!(failed, vec!["GHOST_PARAM".to_string()]);
    }

    #[tokio::test]
    async fn disconnected_link_is_reported_distinctly_from_silence() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<ParamValue>(64);
        drop(mav_rx);

        let result = read_param(&mav_tx, &pv_tx, "SYS_HITL").await;
        assert_eq!(result, Err(ParamReadError::LinkClosed));
    }
}
