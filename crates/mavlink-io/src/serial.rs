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
