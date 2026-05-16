use crossbeam_channel::Sender;
use hitl_physics::px4_pids::{compute_pids, fingerprint as pid_fingerprint, Px4Pids};
use hitl_physics::{
    estimate_flight_time_min, BaroChip, BatteryConfig, BuildSpec, FrameMaterial, ImuChip,
    MagChip, PhysicsConfig,
};
use mavlink::ardupilotmega::{MavMessage, MavParamType, PARAM_SET_DATA};
use std::sync::Mutex;
use std::time::Duration;
use crate::handler::ValidatedNshCommand;
use crate::protocol::{
    AppliedConfig, ConfigResult, ConfigState, ConfigureBuild, OutgoingMessage, Px4PidsView,
};
use tokio::sync::{broadcast, mpsc};
use tokio::time::timeout;
use tracing::{debug, error, info, warn};

/// Per-parameter ack budget. PX4 typically responds within 20-50 ms on a
/// healthy 921600-baud link; 800 ms covers serial backpressure and busy
/// EEPROM writes.
const PARAM_ACK_TIMEOUT: Duration = Duration::from_millis(800);

/// How many times we resend a single PARAM_SET before giving up and failing
/// the whole `ConfigureBuild`. Three covers a transient drop without making
/// the user wait minutes on a truly unreachable FC.
const PARAM_RETRY_COUNT: u8 = 3;

/// Float epsilon for matching PX4's `PARAM_VALUE` against the value we sent.
/// PX4 stores rate-controller gains as `float32`, so any round-trip drift is
/// well below this bound.
const PARAM_ACK_EPSILON: f32 = 1.0e-4;

const DEFAULT_API_URL: &str = "https://api.th3seus.net";

/// Default MAVLink routing for the connected PX4 autopilot.
const PX4_TARGET_SYSTEM: u8 = 1;
const PX4_TARGET_COMPONENT: u8 = 1;

pub struct BuildConfigHandler {
    api_url: String,
    http_client: reqwest::Client,
    config_tx: Sender<(PhysicsConfig, BatteryConfig)>,
    nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
    /// MAVLink output channel — same one the simulation loop uses for
    /// `HIL_SENSOR` / `HIL_GPS`. Phase 6 pushes `PARAM_SET` here.
    /// `None` in `--sim-only` mode (no PX4 attached, no point pushing).
    mav_tx: Option<Sender<MavMessage>>,
    /// Broadcast tap on incoming PARAM_VALUE messages — populated by the
    /// MAVLink receiver task. We subscribe before sending PARAM_SETs so we
    /// can verify each parameter was applied to PX4's running config.
    /// `None` mirrors `mav_tx == None` (sim-only mode skips verification).
    param_value_tx: Option<broadcast::Sender<(String, f32)>>,
    /// Fingerprint of the last set of PIDs successfully *verified*. When the
    /// next `ConfigureBuild` yields the same fingerprint, we skip the param
    /// push to avoid wearing PX4's EEPROM on rapid reconfigures. Only set
    /// after all PARAM_VALUE acks succeed.
    last_pid_fingerprint: Mutex<Option<u64>>,
}

impl BuildConfigHandler {
    pub fn new(
        config_tx: Sender<(PhysicsConfig, BatteryConfig)>,
        nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
        mav_tx: Option<Sender<MavMessage>>,
        param_value_tx: Option<broadcast::Sender<(String, f32)>>,
    ) -> Self {
        let api_url = std::env::var("RELEASE_API_URL")
            .unwrap_or_else(|_| DEFAULT_API_URL.to_string());

        Self {
            api_url,
            http_client: reqwest::Client::builder()
                .timeout(std::time::Duration::from_secs(10))
                .build()
                .expect("Failed to build HTTP client"),
            config_tx,
            nsh_tx,
            mav_tx,
            param_value_tx,
            last_pid_fingerprint: Mutex::new(None),
        }
    }

    /// Process a `ConfigureBuild` request through the two-stage lifecycle:
    ///
    /// 1. Early validation (fetch component specs from API). Failure here
    ///    returns a single `state: Error` ConfigResult and aborts.
    /// 2. Compute physics + PIDs, emit `state: Configuring` via `progress_tx`
    ///    so the UI can show a spinner. The simulation has NOT been
    ///    reconfigured at this point.
    /// 3. Push PARAM_SET to PX4 and `await` matching PARAM_VALUE acks (per-
    ///    param timeout + retry). Failure → `state: Error`, do NOT touch sim.
    /// 4. On full ack success: deliver new physics to sim loop, restart EKF2,
    ///    return `state: Ready`. The UI can now unlock "Continue to simulator".
    pub async fn handle(
        &self,
        request: ConfigureBuild,
        progress_tx: mpsc::Sender<OutgoingMessage>,
    ) -> ConfigResult {
        let motor_specs = match self.fetch_motor_specs(&request.motor_slug).await {
            Ok(specs) => specs,
            Err(e) => {
                error!(slug = %request.motor_slug, error = %e, "Failed to fetch motor specs");
                return ConfigResult {
                    state: ConfigState::Error,
                    success: false,
                    error: Some(format!("Failed to fetch motor: {e}")),
                    config: None,
                };
            }
        };

        let kv = match motor_specs.get("kvRating").and_then(|v| v.as_f64()) {
            Some(kv) => kv,
            None => {
                return ConfigResult {
                    state: ConfigState::Error,
                    success: false,
                    error: Some("Motor missing KV rating in specs".to_string()),
                    config: None,
                };
            }
        };

        let motor_weight_g = motor_specs
            .get("weightG")
            .and_then(|v| v.as_f64())
            .unwrap_or(30.0);

        // Fetch propeller specs if provided, otherwise use defaults
        let (prop_diameter, prop_pitch, blade_count) = if let Some(ref prop_slug) = request.prop_slug {
            match self.fetch_component_specs(prop_slug).await {
                Ok(specs) => {
                    let diameter = specs.get("diameterIn").and_then(|v| v.as_f64())
                        .unwrap_or(request.prop_diameter_inches);
                    let pitch = specs.get("pitchIn").and_then(|v| v.as_f64())
                        .unwrap_or(diameter * 0.9);
                    let blades = specs.get("bladeCount").and_then(|v| v.as_i64())
                        .unwrap_or(3) as i32;
                    info!(slug = %prop_slug, diameter, pitch, blades, "Loaded propeller specs");
                    (diameter, pitch, blades)
                }
                Err(e) => {
                    warn!(slug = %prop_slug, error = %e, "Failed to fetch propeller specs, using defaults");
                    (request.prop_diameter_inches, request.prop_diameter_inches * 0.9, 3)
                }
            }
        } else {
            (request.prop_diameter_inches, request.prop_diameter_inches * 0.9, 3)
        };

        // Build a typed BuildSpec from the request + fetched component specs.
        // Phase 0 keeps parity with the prior `from_build_specs` call; later phases
        // (frame/ESC/FC fetched as full components) populate more fields.
        let mut spec = BuildSpec::default();
        spec.frame.weight_g = request.frame_weight_g;
        spec.motors.kv = kv;
        spec.motors.weight_g = motor_weight_g;
        spec.propellers.diameter_in = prop_diameter;
        spec.propellers.pitch_in = prop_pitch;
        spec.propellers.blade_count = blade_count;
        spec.battery.cell_count = request.battery_cell_count;
        spec.battery.capacity_mah = request.battery_capacity_mah;

        // Fetch frame specs if frame_slug provided — overrides frame_weight_g
        if let Some(ref frame_slug) = request.frame_slug {
            match self.fetch_component_specs(frame_slug).await {
                Ok(specs) => {
                    if let Some(weight) = specs.get("weightG").and_then(|v| v.as_f64()) {
                        spec.frame.weight_g = weight;
                    }
                    if let Some(wheelbase) = specs.get("wheelbaseMm").and_then(|v| v.as_f64()) {
                        spec.frame.wheelbase_mm = wheelbase;
                    }
                    if let Some(material_str) = specs.get("material").and_then(|v| v.as_str()) {
                        spec.frame.material = Some(parse_frame_material(material_str));
                    }
                    info!(slug = %frame_slug, weight_g = spec.frame.weight_g, wheelbase_mm = spec.frame.wheelbase_mm, "Loaded frame specs");
                }
                Err(e) => {
                    warn!(slug = %frame_slug, error = %e, "Failed to fetch frame specs, using frame_weight_g fallback");
                }
            }
        }

        // Fetch ESC specs if esc_slug provided
        if let Some(ref esc_slug) = request.esc_slug {
            match self.fetch_component_specs(esc_slug).await {
                Ok(specs) => {
                    if let Some(continuous) = specs.get("continuousCurrentA").and_then(|v| v.as_f64()) {
                        spec.escs.continuous_amps = continuous;
                    }
                    if let Some(burst) = specs.get("burstCurrentA").and_then(|v| v.as_f64()) {
                        spec.escs.burst_amps = Some(burst);
                    }
                    if let Some(weight) = specs.get("weightG").and_then(|v| v.as_f64()) {
                        spec.escs.weight_g = Some(weight);
                    }
                    info!(slug = %esc_slug, continuous_amps = spec.escs.continuous_amps, "Loaded ESC specs");
                }
                Err(e) => {
                    warn!(slug = %esc_slug, error = %e, "Failed to fetch ESC specs, using defaults");
                }
            }
        }

        // Fetch flight controller specs if fc_slug provided
        if let Some(ref fc_slug) = request.fc_slug {
            match self.fetch_component_specs(fc_slug).await {
                Ok(specs) => {
                    if let Some(weight) = specs.get("weightG").and_then(|v| v.as_f64()) {
                        spec.flight_controller.weight_g = weight;
                    }
                    if let Some(gyro_str) = specs.get("gyro").and_then(|v| v.as_str()) {
                        spec.flight_controller.imu_chip = Some(parse_imu_chip(gyro_str));
                    }
                    if let Some(baro_str) = specs.get("barometer").and_then(|v| v.as_str()) {
                        spec.flight_controller.baro_chip = Some(parse_baro_chip(baro_str));
                    }
                    if let Some(mag_str) = specs.get("magnetometer").and_then(|v| v.as_str()) {
                        spec.flight_controller.mag_chip = Some(parse_mag_chip(mag_str));
                    }
                    info!(
                        slug = %fc_slug,
                        weight_g = spec.flight_controller.weight_g,
                        imu = ?spec.flight_controller.imu_chip,
                        "Loaded flight controller specs"
                    );
                }
                Err(e) => {
                    warn!(slug = %fc_slug, error = %e, "Failed to fetch FC specs, using defaults");
                }
            }
        }

        let mut physics = spec.to_physics_config();
        // The request may carry a non-nominal voltage (e.g. fully-charged 16.8 V on
        // a 4S pack). Preserve that — `to_physics_config()` derives nominal from
        // cell_count. Phase 2b's loop-side V_terminal will eventually subsume
        // this override.
        physics.battery_voltage = request.battery_voltage;

        // Phase 2b: BatteryConfig now carries internal_resistance_ohm. We don't
        // wire it into the discharge loop yet (deferred with brownout detection),
        // but the field is populated end-to-end so the loop can read it later.
        let battery = spec.to_battery_config();

        let max_omega = physics.max_motor_speed_from_voltage();
        let max_thrust_per_motor_g = (physics.kt * max_omega * max_omega) / 9.80665 * 1000.0;
        let thrust_to_weight_ratio = (4.0 * max_thrust_per_motor_g) / (physics.mass_kg * 1000.0);

        let max_motor_rpm = physics.motor_kv * physics.battery_voltage;
        let flight_time = estimate_flight_time_min(&battery, &physics);

        // Phase 6: derive per-build rate-controller PIDs. Without this, light
        // airframes (real inertia well below the legacy 0.012 floor) trigger
        // PX4 rate-controller oscillation because the stock PIDs are tuned
        // for I ≈ 0.005 kg·m².
        let pids = compute_pids(&physics);

        // Stage 1: emit "configuring" so the UI shows a spinner. applied_pids
        // is None and verified_params is 0 because acks haven't returned yet.
        // The sim loop has NOT been touched, so the user's current flight (if
        // any) keeps running on the previous config.
        let configuring_view = AppliedConfig {
            mass_kg: physics.mass_kg,
            kt: physics.kt,
            kq: physics.kq,
            arm_length_m: physics.arm_length_m,
            max_thrust_per_motor_g,
            thrust_to_weight_ratio,
            motor_kv: physics.motor_kv,
            battery_voltage: physics.battery_voltage,
            max_motor_rpm,
            estimated_flight_time_min: flight_time,
            applied_pids: None,
            verified_params: 0,
        };
        if progress_tx
            .send(OutgoingMessage::ConfigResult(ConfigResult {
                state: ConfigState::Configuring,
                success: true,
                error: None,
                config: Some(configuring_view.clone()),
            }))
            .await
            .is_err()
        {
            warn!("client disconnected before interim ConfigResult delivered");
        }

        // Stage 2: push PIDs and await per-param PARAM_VALUE acks. Fail-closed:
        // any verification failure aborts before touching the sim loop, so the
        // user can't accidentally fly with mismatched PIDs vs. physics.
        let (applied_pids, verified_params) = match self.push_pids_and_verify(&pids).await {
            Ok(view) => view,
            Err(e) => {
                error!(error = %e, "PID verification failed — aborting reconfigure");
                return ConfigResult {
                    state: ConfigState::Error,
                    success: false,
                    error: Some(format!("PID verification failed: {e}")),
                    config: None,
                };
            }
        };

        // Stage 3: hand new physics to the simulation loop. Only at this point
        // is the running drone state reconfigured. PX4 already has matching
        // PIDs; EKF2 is about to be reset to clear stale estimator state.
        if let Err(e) = self.config_tx.send((physics, battery)) {
            error!(error = %e, "Failed to send physics config to simulation");
            return ConfigResult {
                state: ConfigState::Error,
                success: false,
                error: Some("Simulation thread unavailable".to_string()),
                config: None,
            };
        }

        info!(
            mass_kg = configuring_view.mass_kg,
            kt = configuring_view.kt,
            twr = configuring_view.thrust_to_weight_ratio,
            verified_params,
            "Build configured + PIDs verified"
        );

        self.restart_ekf2().await;

        ConfigResult {
            state: ConfigState::Ready,
            success: true,
            error: None,
            config: Some(AppliedConfig {
                applied_pids,
                verified_params,
                ..configuring_view
            }),
        }
    }

    /// Push the per-build PIDs as `PARAM_SET` and verify each one with a
    /// matching `PARAM_VALUE` ack from PX4. Per-param retry up to
    /// `PARAM_RETRY_COUNT` times with `PARAM_ACK_TIMEOUT` per attempt.
    ///
    /// Returns `Ok((Some(view), N))` on full success (N = number of params
    /// verified), `Ok((None, 0))` when skipped (sim-only or fingerprint
    /// unchanged), or `Err(msg)` when any param could not be confirmed.
    ///
    /// Subscribes to the broadcast BEFORE sending so no acks are missed.
    /// Fingerprint cache is only updated on full success — partial pushes
    /// must retry the whole sequence on the next ConfigureBuild.
    pub(crate) async fn push_pids_and_verify(
        &self,
        pids: &Px4Pids,
    ) -> Result<(Option<Px4PidsView>, u32), String> {
        let (Some(mav_tx), Some(param_value_tx)) =
            (self.mav_tx.as_ref(), self.param_value_tx.as_ref())
        else {
            debug!("sim-only mode: skipping PARAM_SET push (no PX4 attached)");
            return Ok((None, 0));
        };

        let fp = pid_fingerprint(pids);
        {
            let cache = self.last_pid_fingerprint.lock().expect("PID cache poisoned");
            if *cache == Some(fp) {
                debug!(fingerprint = fp, "PID fingerprint unchanged — skipping PARAM_SET push");
                return Ok((None, 0));
            }
        }

        let params: [(&str, f32); 12] = [
            ("MC_ROLLRATE_P",   pids.roll_p),
            ("MC_ROLLRATE_I",   pids.roll_i),
            ("MC_ROLLRATE_D",   pids.roll_d),
            ("MC_ROLLRATE_FF",  pids.roll_ff),
            ("MC_PITCHRATE_P",  pids.pitch_p),
            ("MC_PITCHRATE_I",  pids.pitch_i),
            ("MC_PITCHRATE_D",  pids.pitch_d),
            ("MC_PITCHRATE_FF", pids.pitch_ff),
            ("MC_YAWRATE_P",    pids.yaw_p),
            ("MC_YAWRATE_I",    pids.yaw_i),
            ("MC_YAWRATE_D",    pids.yaw_d),
            ("MC_YAWRATE_FF",   pids.yaw_ff),
        ];

        let mut verified = 0u32;
        for (name, value) in params {
            let mut rx = param_value_tx.subscribe();
            let mut acked = false;

            for attempt in 1..=PARAM_RETRY_COUNT {
                // Resubscribe drains anything that arrived between attempts so
                // stale acks don't false-positive a later send.
                if attempt > 1 {
                    rx = param_value_tx.subscribe();
                }

                match mav_tx.try_send(make_param_set(name, value)) {
                    Ok(()) => {}
                    Err(crossbeam_channel::TrySendError::Full(_)) => {
                        warn!(
                            param = name,
                            attempt,
                            "MAVLink tx channel full — retrying PARAM_SET"
                        );
                        tokio::time::sleep(Duration::from_millis(50)).await;
                        continue;
                    }
                    Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                        return Err(format!(
                            "MAVLink tx disconnected while sending {name}"
                        ));
                    }
                }

                if let Some((got_name, got_value)) =
                    wait_for_param_ack(&mut rx, name, value).await
                {
                    debug!(
                        param = %got_name,
                        value = got_value,
                        attempt,
                        "PARAM_VALUE ack confirmed"
                    );
                    acked = true;
                    break;
                }
                warn!(
                    param = name,
                    attempt,
                    timeout_ms = PARAM_ACK_TIMEOUT.as_millis() as u64,
                    "PARAM_VALUE ack timed out — retrying"
                );
            }

            if !acked {
                return Err(format!(
                    "Failed to verify {name} after {PARAM_RETRY_COUNT} retries"
                ));
            }
            verified += 1;
        }

        // Cache only after full verification — partial pushes must retry.
        *self.last_pid_fingerprint.lock().expect("PID cache poisoned") = Some(fp);

        info!(verified, fingerprint = fp, "All per-build PIDs verified on PX4");

        Ok((
            Some(Px4PidsView {
                roll_p: pids.roll_p,
                roll_i: pids.roll_i,
                roll_d: pids.roll_d,
                roll_ff: pids.roll_ff,
                pitch_p: pids.pitch_p,
                pitch_i: pids.pitch_i,
                pitch_d: pids.pitch_d,
                pitch_ff: pids.pitch_ff,
                yaw_p: pids.yaw_p,
                yaw_i: pids.yaw_i,
                yaw_d: pids.yaw_d,
                yaw_ff: pids.yaw_ff,
            }),
            verified,
        ))
    }

    /// Restart EKF2 to clear stale estimator state after config change
    async fn restart_ekf2(&self) {
        let Some(ref nsh_tx) = self.nsh_tx else {
            warn!("NSH not available, skipping EKF2 restart");
            return;
        };

        // Send ekf2 stop command
        let stop_cmd = ValidatedNshCommand {
            request_id: 0xFFFF_FF01, // Special ID for internal commands
            command: "ekf2 stop".to_string(),
            timeout_ms: 2000,
            client_id: 0, // System client
        };

        if let Err(e) = nsh_tx.send(stop_cmd).await {
            warn!(error = %e, "Failed to send ekf2 stop command");
            return;
        }

        // Small delay to let EKF2 stop cleanly
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

        // Send ekf2 start command
        let start_cmd = ValidatedNshCommand {
            request_id: 0xFFFF_FF02,
            command: "ekf2 start".to_string(),
            timeout_ms: 2000,
            client_id: 0,
        };

        if let Err(e) = nsh_tx.send(start_cmd).await {
            warn!(error = %e, "Failed to send ekf2 start command");
            return;
        }

        info!("EKF2 restarted after config change");
    }

    async fn fetch_motor_specs(&self, slug: &str) -> Result<serde_json::Value, String> {
        self.fetch_component_specs(slug).await
    }

    async fn fetch_component_specs(&self, slug: &str) -> Result<serde_json::Value, String> {
        let url = format!("{}/api/components/{}", self.api_url, slug);
        let resp = self.http_client.get(&url).send().await
            .map_err(|e| format!("HTTP request failed: {e}"))?;

        if !resp.status().is_success() {
            return Err(format!("API returned {}", resp.status()));
        }

        let body: serde_json::Value = resp.json().await
            .map_err(|e| format!("Failed to parse response: {e}"))?;

        body.get("specs")
            .cloned()
            .ok_or_else(|| "Response missing 'specs' field".to_string())
    }
}

// ============================================================================
// Enum parsing helpers — case-insensitive string matching
// ============================================================================

/// Parse a material string to FrameMaterial (case-insensitive).
fn parse_frame_material(s: &str) -> FrameMaterial {
    let lower = s.to_lowercase();
    match lower.as_str() {
        "carbon" | "carbon fiber" | "cf" => FrameMaterial::Carbon,
        "polymer" | "plastic" | "abs" | "polycarbonate" => FrameMaterial::Polymer,
        "aluminum" | "aluminium" | "alu" | "al" => FrameMaterial::Aluminum,
        _ => FrameMaterial::Other,
    }
}

/// Parse a gyro/IMU chip string to ImuChip (case-insensitive).
fn parse_imu_chip(s: &str) -> ImuChip {
    let normalized = s
        .to_lowercase()
        .replace('-', "")
        .replace('_', "")
        .replace(' ', "");
    match normalized.as_str() {
        "mpu6000" | "mpu6000p" => ImuChip::Mpu6000,
        "mpu6500" | "mpu6500p" => ImuChip::Mpu6500,
        "icm20689" | "icm20689p" => ImuChip::Icm20689,
        "icm42688p" | "icm42688" => ImuChip::Icm42688p,
        "bmi270" => ImuChip::Bmi270,
        "lsm6dso" | "lsm6dso32" => ImuChip::Lsm6dso,
        _ => ImuChip::Other,
    }
}

/// Parse a barometer chip string to BaroChip (case-insensitive).
fn parse_baro_chip(s: &str) -> BaroChip {
    let normalized = s
        .to_lowercase()
        .replace('-', "")
        .replace('_', "")
        .replace(' ', "");
    match normalized.as_str() {
        "bmp280" => BaroChip::Bmp280,
        "bmp388" | "bmp390" => BaroChip::Bmp388,
        "dps310" => BaroChip::Dps310,
        "spl06" | "spl06001" => BaroChip::Spl06,
        "ms5611" | "ms561101ba03" => BaroChip::Ms5611,
        _ => BaroChip::Other,
    }
}

/// Build a `PARAM_SET` MAVLink message targeting the autopilot. Param IDs
/// are right-padded with NUL bytes inside the 16-byte buffer per the MAVLink
/// spec; names longer than 16 chars are truncated.
/// Drain `rx` until a `PARAM_VALUE` arrives whose name matches `name` and
/// whose value is within `PARAM_ACK_EPSILON` of `expected`. Returns the
/// matched (name, value) tuple, or `None` if `PARAM_ACK_TIMEOUT` elapses
/// without a match. Lagged/closed receivers are treated as "no ack".
async fn wait_for_param_ack(
    rx: &mut broadcast::Receiver<(String, f32)>,
    name: &str,
    expected: f32,
) -> Option<(String, f32)> {
    let deadline = tokio::time::Instant::now() + PARAM_ACK_TIMEOUT;
    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            return None;
        }
        match timeout(remaining, rx.recv()).await {
            Ok(Ok((got_name, got_value))) => {
                if got_name == name && (got_value - expected).abs() <= PARAM_ACK_EPSILON {
                    return Some((got_name, got_value));
                }
                // Unrelated PARAM_VALUE (QGC pull, other params) — keep draining.
            }
            Ok(Err(broadcast::error::RecvError::Lagged(n))) => {
                warn!(
                    param = name,
                    lagged = n,
                    "PARAM_VALUE receiver lagged — continuing to wait"
                );
            }
            Ok(Err(broadcast::error::RecvError::Closed)) => return None,
            Err(_) => return None,
        }
    }
}

fn make_param_set(name: &str, value: f32) -> MavMessage {
    let mut param_id = [0u8; 16];
    let bytes = name.as_bytes();
    let copy_len = bytes.len().min(param_id.len());
    param_id[..copy_len].copy_from_slice(&bytes[..copy_len]);
    MavMessage::PARAM_SET(PARAM_SET_DATA {
        param_value: value,
        target_system: PX4_TARGET_SYSTEM,
        target_component: PX4_TARGET_COMPONENT,
        param_id,
        param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
    })
}

/// Parse a magnetometer chip string to MagChip (case-insensitive).
fn parse_mag_chip(s: &str) -> MagChip {
    let normalized = s
        .to_lowercase()
        .replace('-', "")
        .replace('_', "")
        .replace(' ', "");
    match normalized.as_str() {
        "hmc5883" | "hmc5883l" => MagChip::Hmc5883,
        "qmc5883" | "qmc5883l" => MagChip::Qmc5883,
        "ist8310" => MagChip::Ist8310,
        "rm3100" => MagChip::Rm3100,
        "lis3mdl" => MagChip::Lis3mdl,
        "none" | "" => MagChip::None,
        _ => MagChip::Other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crossbeam_channel::bounded;
    use hitl_physics::px4_pids::REF_PIDS;
    use mavlink::ardupilotmega::MavMessage;

    /// Spawn a fake PX4 ack responder: every PARAM_SET pulled from `mav_rx`
    /// is mirrored back on `param_value_tx` as a successful ack. Optionally
    /// drop the first `drop_first` PARAM_SETs entirely (simulates lossy link).
    fn spawn_fake_px4(
        mav_rx: crossbeam_channel::Receiver<MavMessage>,
        param_value_tx: broadcast::Sender<(String, f32)>,
        drop_first: usize,
    ) -> tokio::task::JoinHandle<()> {
        tokio::task::spawn_blocking(move || {
            let mut dropped = 0;
            while let Ok(msg) = mav_rx.recv() {
                if let MavMessage::PARAM_SET(p) = msg {
                    let name = std::str::from_utf8(&p.param_id)
                        .unwrap_or("")
                        .trim_end_matches('\0')
                        .to_string();
                    if dropped < drop_first {
                        dropped += 1;
                        continue;
                    }
                    let _ = param_value_tx.send((name, p.param_value));
                }
            }
        })
    }

    fn make_handler(
        mav_tx: Option<Sender<MavMessage>>,
        param_value_tx: Option<broadcast::Sender<(String, f32)>>,
    ) -> BuildConfigHandler {
        let (config_tx, _config_rx) = bounded::<(PhysicsConfig, BatteryConfig)>(4);
        BuildConfigHandler::new(config_tx, None, mav_tx, param_value_tx)
    }

    #[tokio::test]
    async fn sim_only_mode_skips_verification() {
        let handler = make_handler(None, None);
        let result = handler.push_pids_and_verify(&REF_PIDS).await;
        assert!(matches!(result, Ok((None, 0))));
    }

    #[tokio::test]
    async fn happy_path_acks_all_twelve_params() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        let _px4 = spawn_fake_px4(mav_rx, pv_tx.clone(), 0);

        let handler = make_handler(Some(mav_tx), Some(pv_tx));
        let (view, verified) = handler.push_pids_and_verify(&REF_PIDS).await.unwrap();
        assert!(view.is_some());
        assert_eq!(verified, 12);
    }

    #[tokio::test]
    async fn one_dropped_ack_recovers_via_retry() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        // Drop only the first PARAM_SET — the retry should land an ack.
        let _px4 = spawn_fake_px4(mav_rx, pv_tx.clone(), 1);

        let handler = make_handler(Some(mav_tx), Some(pv_tx));
        let (view, verified) = handler.push_pids_and_verify(&REF_PIDS).await.unwrap();
        assert!(view.is_some());
        assert_eq!(verified, 12);
    }

    #[tokio::test]
    async fn persistent_silence_returns_error() {
        let (mav_tx, _mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        // No fake PX4 — every PARAM_SET sits in mav_rx forever and no ack ever
        // comes back. Each param exhausts PARAM_RETRY_COUNT attempts.
        let handler = make_handler(Some(mav_tx), Some(pv_tx));
        let err = handler.push_pids_and_verify(&REF_PIDS).await.unwrap_err();
        assert!(
            err.contains("MC_ROLLRATE_P"),
            "expected failure on the first param, got: {err}"
        );
    }

    #[tokio::test]
    async fn fingerprint_skips_second_call() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        let _px4 = spawn_fake_px4(mav_rx, pv_tx.clone(), 0);

        let handler = make_handler(Some(mav_tx), Some(pv_tx));
        let (_, verified_first) = handler.push_pids_and_verify(&REF_PIDS).await.unwrap();
        assert_eq!(verified_first, 12);

        // Identical PIDs → fingerprint matches → no push, no acks needed.
        let (view, verified_second) = handler.push_pids_and_verify(&REF_PIDS).await.unwrap();
        assert!(view.is_none());
        assert_eq!(verified_second, 0);
    }
}
