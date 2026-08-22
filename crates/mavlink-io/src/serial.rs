//! Serial port enumeration and Pixhawk detection

use serialport::{SerialPort, SerialPortType};
use std::time::Duration;
use thiserror::Error;
use tracing::{debug, info, warn};

/// Known PX4 vendor IDs
const VID_3DR: u16 = 0x26AC;
const VID_NXP: u16 = 0x1FC9;
const VID_HOLYBRO_1: u16 = 0x2DAE;
const VID_HOLYBRO_2: u16 = 0x3162;

/// Serial port configuration
#[derive(Debug, Clone)]
pub struct SerialConfig {
    pub baud_rate: u32,
    pub timeout: Duration,
}

impl Default for SerialConfig {
    fn default() -> Self {
        Self {
            baud_rate: 921600,
            timeout: Duration::from_millis(100),
        }
    }
}

impl SerialConfig {
    pub fn new(baud_rate: u32) -> Self {
        Self {
            baud_rate,
            ..Default::default()
        }
    }

    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }
}

#[derive(Debug, Error)]
pub enum SerialError {
    #[error("Failed to enumerate serial ports: {0}")]
    EnumerationFailed(#[from] serialport::Error),

    #[error("Failed to open serial port '{port}': {source}")]
    OpenFailed {
        port: String,
        source: serialport::Error,
    },

    #[error("No PX4 boards found")]
    NoFcFound,
}

/// On macOS, convert `/dev/cu.XXX` to `/dev/tty.XXX` if the tty variant exists.
/// The `cu` (call-up) device blocks reads until DTR is asserted, which
/// tokio-serial does not do. The `tty` device delivers data immediately.
fn maybe_prefer_tty(port: &str) -> String {
    #[cfg(target_os = "macos")]
    {
        if port.starts_with("/dev/cu.") {
            let tty_path = port.replace("/dev/cu.", "/dev/tty.");
            if std::path::Path::new(&tty_path).exists() {
                debug!(cu = %port, tty = %tty_path, "Preferring tty variant on macOS");
                return tty_path;
            }
        }
    }
    port.to_string()
}

/// Check if a USB vendor ID matches known PX4 board manufacturers
fn is_pixhawk_vid(vid: u16) -> bool {
    matches!(vid, VID_3DR | VID_NXP | VID_HOLYBRO_1 | VID_HOLYBRO_2)
}

/// Find all serial ports that appear to be PX4-compatible flight controllers
///
/// Detects by USB vendor ID:
/// - 0x26AC: 3DR
/// - 0x1FC9: NXP
/// - 0x2DAE, 0x3162: Holybro
pub fn find_pixhawk_ports() -> Vec<String> {
    let ports = match serialport::available_ports() {
        Ok(p) => p,
        Err(e) => {
            warn!(error = %e, "Failed to enumerate serial ports");
            return Vec::new();
        }
    };

    let mut pixhawk_ports = Vec::new();

    for port in ports {
        debug!(port = %port.port_name, "Checking serial port");

        if let SerialPortType::UsbPort(usb_info) = &port.port_type {
            if is_pixhawk_vid(usb_info.vid) {
                // On macOS, prefer /dev/tty.* over /dev/cu.* — the cu variant
                // doesn't deliver data until DTR is asserted, which tokio-serial
                // doesn't do by default. The tty variant works immediately.
                let port_name = maybe_prefer_tty(&port.port_name);

                info!(
                    port = %port_name,
                    vid = format!("0x{:04X}", usb_info.vid),
                    pid = format!("0x{:04X}", usb_info.pid),
                    manufacturer = usb_info.manufacturer.as_deref().unwrap_or("Unknown"),
                    product = usb_info.product.as_deref().unwrap_or("Unknown"),
                    "Found PX4 board"
                );
                if !pixhawk_ports.contains(&port_name) {
                    pixhawk_ports.push(port_name);
                }
            }
        }
    }

    pixhawk_ports
}

/// Per-port budget for a heartbeat probe.
///
/// PX4 broadcasts HEARTBEAT at 1 Hz, so anything under a second risks missing
/// a board that is genuinely there. 1.5s allows for one missed beat without
/// making a sweep of several ports feel stalled.
pub const PROBE_WINDOW: Duration = Duration::from_millis(1500);

/// Result of looking for a flight controller.
#[derive(Debug, Clone, PartialEq)]
pub struct DetectionOutcome {
    /// Ports that are, or appear to be, flight controllers.
    pub found: Vec<String>,
    /// Every port considered, whether by vendor ID or by probing. Reported to
    /// the user when nothing was found: "no FC detected" is not actionable,
    /// "examined these three ports" is.
    pub examined: Vec<String>,
    /// True when the port was adopted by probing rather than by vendor ID.
    pub adopted_by_probe: bool,
}

/// Locate a flight controller, falling back to probing when no vendor ID matches.
///
/// The allowlist runs first so recognised hardware never pays for probing.
pub fn detect_flight_controller() -> DetectionOutcome {
    let known = find_pixhawk_ports();
    let mut examined = known.clone();

    if !known.is_empty() {
        return DetectionOutcome {
            found: known,
            examined,
            adopted_by_probe: false,
        };
    }

    let candidates = candidate_probe_ports();
    if candidates.is_empty() {
        return DetectionOutcome {
            found: Vec::new(),
            examined,
            adopted_by_probe: false,
        };
    }

    info!(
        candidates = candidates.len(),
        "No board matched a known vendor ID — probing for a MAVLink heartbeat"
    );

    let config = SerialConfig::default();
    for port in candidates {
        examined.push(port.clone());
        match open_serial(&port, &config) {
            Ok(mut handle) => {
                let deadline = std::time::Instant::now() + PROBE_WINDOW;
                let found = scan_for_heartbeat(&mut handle, deadline);
                // Dropped either way: a port that stays quiet must be handed
                // straight back, since something else may need it.
                drop(handle);
                if found {
                    info!(port = %port, "Adopted flight controller found by probing");
                    return DetectionOutcome {
                        found: vec![port],
                        examined,
                        adopted_by_probe: true,
                    };
                }
                debug!(port = %port, "No heartbeat within the probe window");
            }
            Err(e) => {
                debug!(port = %port, error = %e, "Could not open port for probing");
            }
        }
    }

    DetectionOutcome {
        found: Vec::new(),
        examined,
        adopted_by_probe: false,
    }
}

/// Serial ports that did not match a known vendor ID but could still be a
/// flight controller.
///
/// The vendor allowlist covers four manufacturers; the market has many more.
/// Everything else USB-serial is a candidate, minus descriptors that are known
/// not to be flight controllers — probing a Bluetooth modem or a debug console
/// wastes the budget and risks disturbing something that is in use.
pub fn candidate_probe_ports() -> Vec<String> {
    let ports = match serialport::available_ports() {
        Ok(p) => p,
        Err(e) => {
            warn!(error = %e, "Failed to enumerate serial ports");
            return Vec::new();
        }
    };

    let mut candidates = Vec::new();
    for port in ports {
        let SerialPortType::UsbPort(usb_info) = &port.port_type else {
            // Non-USB serial (built-in UARTs, virtual ports) is not where a
            // USB-attached flight controller appears.
            continue;
        };
        if is_pixhawk_vid(usb_info.vid) {
            continue; // already found by the allowlist
        }
        if is_excluded_port(&port.port_name) {
            debug!(port = %port.port_name, "Skipping port excluded from probing");
            continue;
        }
        let port_name = maybe_prefer_tty(&port.port_name);
        if !candidates.contains(&port_name) {
            candidates.push(port_name);
        }
    }
    candidates
}

/// Port names that are never a flight controller.
///
/// Matched on the device name rather than the USB descriptor because the
/// offenders (Bluetooth bridges, debug consoles, wireless serial adapters)
/// are consistent in naming and inconsistent in how they describe themselves.
fn is_excluded_port(port_name: &str) -> bool {
    const EXCLUDED_FRAGMENTS: &[&str] = &[
        "Bluetooth",
        "bluetooth",
        "debug-console",
        "wlan-debug",
        "SOC",
        "MALS",
        "AirPods",
    ];
    EXCLUDED_FRAGMENTS
        .iter()
        .any(|fragment| port_name.contains(fragment))
}

/// Whether a byte stream contains something that looks like a MAVLink
/// HEARTBEAT.
///
/// Takes a reader and nothing else: a probe must never transmit, and the only
/// way to guarantee that is to give the probe no means of writing. A device on
/// the other end may be a 3D printer or a debug probe, and unsolicited MAVLink
/// bytes could put it into a state its owner did not ask for.
pub fn scan_for_heartbeat<R: std::io::Read>(reader: &mut R, deadline: std::time::Instant) -> bool {
    let mut window = Vec::with_capacity(512);
    let mut chunk = [0u8; 256];

    while std::time::Instant::now() < deadline {
        match reader.read(&mut chunk) {
            Ok(0) => continue,
            Ok(n) => {
                window.extend_from_slice(&chunk[..n]);
                if contains_heartbeat(&window) {
                    return true;
                }
                // Keep only enough tail to span a frame header split across
                // reads; unbounded growth over a chatty non-MAVLink device
                // would be a slow leak.
                if window.len() > 4096 {
                    window.drain(..window.len() - 512);
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(_) => return false,
        }
    }
    false
}

/// Look for a MAVLink v1 or v2 HEARTBEAT (message id 0) frame header.
///
/// Header-only: validating the CRC would need the message-specific CRC_EXTRA
/// table, and the question here is only "is a flight controller talking on
/// this port", not "is this frame intact".
pub fn contains_heartbeat(bytes: &[u8]) -> bool {
    const MAVLINK_V1_MAGIC: u8 = 0xFE;
    const MAVLINK_V2_MAGIC: u8 = 0xFD;

    for (i, byte) in bytes.iter().enumerate() {
        match *byte {
            // v1: magic, len, seq, sysid, compid, msgid
            MAVLINK_V1_MAGIC if bytes.len() > i + 5 => {
                if bytes[i + 5] == 0 {
                    return true;
                }
            }
            // v2: magic, len, incompat, compat, seq, sysid, compid, msgid(3 bytes LE)
            MAVLINK_V2_MAGIC if bytes.len() > i + 9 => {
                if bytes[i + 7] == 0 && bytes[i + 8] == 0 && bytes[i + 9] == 0 {
                    return true;
                }
            }
            _ => {}
        }
    }
    false
}

/// Open a serial port with the given configuration
pub fn open_serial(port: &str, config: &SerialConfig) -> Result<Box<dyn SerialPort>, SerialError> {
    info!(
        port = %port,
        baud_rate = config.baud_rate,
        timeout_ms = config.timeout.as_millis(),
        "Opening serial port"
    );

    serialport::new(port, config.baud_rate)
        .timeout(config.timeout)
        .open()
        .map_err(|e| SerialError::OpenFailed {
            port: port.to_string(),
            source: e,
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pixhawk_vid_detection() {
        assert!(is_pixhawk_vid(VID_3DR));
        assert!(is_pixhawk_vid(VID_NXP));
        assert!(is_pixhawk_vid(VID_HOLYBRO_1));
        assert!(is_pixhawk_vid(VID_HOLYBRO_2));
        assert!(!is_pixhawk_vid(0x1234));
    }

    /// A reader that also records whether anything was ever written to it.
    /// It cannot record a write, because `scan_for_heartbeat` is handed a
    /// `Read` and nothing else — the guarantee is structural, and this test
    /// documents that the signature is the mechanism.
    struct ReadOnlySource {
        data: std::io::Cursor<Vec<u8>>,
    }

    impl std::io::Read for ReadOnlySource {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            self.data.read(buf)
        }
    }

    fn heartbeat_v2_frame() -> Vec<u8> {
        // magic, len, incompat, compat, seq, sysid, compid, msgid(0,0,0)
        vec![0xFD, 9, 0, 0, 42, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    }

    fn heartbeat_v1_frame() -> Vec<u8> {
        // magic, len, seq, sysid, compid, msgid=0
        vec![0xFE, 9, 7, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    }

    #[test]
    fn detects_a_mavlink_v2_heartbeat() {
        assert!(contains_heartbeat(&heartbeat_v2_frame()));
    }

    #[test]
    fn detects_a_mavlink_v1_heartbeat() {
        assert!(contains_heartbeat(&heartbeat_v1_frame()));
    }

    #[test]
    fn detects_a_heartbeat_that_does_not_start_at_the_first_byte() {
        // A probe opens mid-stream, so the first bytes read are usually the
        // tail of some other frame.
        let mut stream = vec![0x11, 0x22, 0x33, 0x44];
        stream.extend_from_slice(&heartbeat_v2_frame());
        assert!(contains_heartbeat(&stream));
    }

    #[test]
    fn does_not_mistake_another_mavlink_message_for_a_heartbeat() {
        // Same framing, message id 30 (ATTITUDE). A port carrying MAVLink but
        // no heartbeat is not a flight controller announcing itself.
        let frame = vec![0xFD, 9, 0, 0, 42, 1, 1, 30, 0, 0, 0, 0, 0, 0];
        assert!(!contains_heartbeat(&frame));
    }

    #[test]
    fn does_not_find_a_heartbeat_in_unrelated_traffic() {
        // A 3D printer answering G-code, for instance.
        assert!(!contains_heartbeat(b"ok T:210.0 /210.0 B:60.0 /60.0\n"));
    }

    #[test]
    fn a_truncated_header_is_not_a_heartbeat() {
        // Magic byte at the very end of what was read so far: there is not
        // enough to decide, and guessing would adopt the wrong port.
        assert!(!contains_heartbeat(&[0x00, 0x00, 0xFD]));
        assert!(!contains_heartbeat(&[0xFE, 9, 7]));
    }

    #[test]
    fn scanning_adopts_a_port_that_emits_a_heartbeat() {
        let mut source = ReadOnlySource {
            data: std::io::Cursor::new(heartbeat_v2_frame()),
        };
        let deadline = std::time::Instant::now() + Duration::from_millis(200);
        assert!(scan_for_heartbeat(&mut source, deadline));
    }

    #[test]
    fn scanning_releases_a_port_that_stays_quiet() {
        let mut source = ReadOnlySource {
            data: std::io::Cursor::new(Vec::new()),
        };
        // Bounded: the probe must give the port back rather than holding it.
        let started = std::time::Instant::now();
        let deadline = started + Duration::from_millis(100);
        assert!(!scan_for_heartbeat(&mut source, deadline));
        assert!(started.elapsed() < Duration::from_secs(2), "probe must be bounded");
    }

    #[test]
    fn scanning_releases_a_port_carrying_unrelated_traffic() {
        let noise: Vec<u8> = std::iter::repeat(b"ok T:210.0\n")
            .take(50)
            .flat_map(|s| s.to_vec())
            .collect();
        let mut source = ReadOnlySource {
            data: std::io::Cursor::new(noise),
        };
        let deadline = std::time::Instant::now() + Duration::from_millis(100);
        assert!(!scan_for_heartbeat(&mut source, deadline));
    }

    #[test]
    fn known_non_flight_controller_ports_are_excluded() {
        assert!(is_excluded_port("/dev/tty.Bluetooth-Incoming-Port"));
        assert!(is_excluded_port("/dev/tty.debug-console"));
        assert!(!is_excluded_port("/dev/tty.usbmodem01"));
        assert!(!is_excluded_port("/dev/ttyACM0"));
    }

    #[test]
    fn test_serial_config_default() {
        let config = SerialConfig::default();
        assert_eq!(config.baud_rate, 921600);
        assert_eq!(config.timeout, Duration::from_millis(100));
    }

    #[test]
    fn test_serial_config_builder() {
        let config = SerialConfig::new(115200).with_timeout(Duration::from_secs(1));
        assert_eq!(config.baud_rate, 115200);
        assert_eq!(config.timeout, Duration::from_secs(1));
    }
}
