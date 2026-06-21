//! HIL_SENSOR and HIL_GPS message constructors

use mavlink::ardupilotmega::{HilSensorUpdatedFlags, MavMessage, HIL_GPS_DATA, HIL_SENSOR_DATA};

/// MAVLink system ID for the HITL simulator
pub const SYSTEM_ID: u8 = 1;

/// MAVLink component ID for the HITL simulator
pub const COMPONENT_ID: u8 = 1;

/// Sensor data for HIL_SENSOR message
#[derive(Debug, Clone, Default)]
pub struct HilSensorData {
    /// Timestamp in microseconds since boot
    pub time_usec: u64,
    /// X acceleration (m/s^2)
    pub xacc: f32,
    /// Y acceleration (m/s^2)
    pub yacc: f32,
    /// Z acceleration (m/s^2)
    pub zacc: f32,
    /// X angular speed (rad/s)
    pub xgyro: f32,
    /// Y angular speed (rad/s)
    pub ygyro: f32,
    /// Z angular speed (rad/s)
    pub zgyro: f32,
    /// X magnetic field (gauss)
    pub xmag: f32,
    /// Y magnetic field (gauss)
    pub ymag: f32,
    /// Z magnetic field (gauss)
    pub zmag: f32,
    /// Absolute pressure (hPa)
    pub abs_pressure: f32,
    /// Differential pressure (hPa)
    pub diff_pressure: f32,
    /// Altitude calculated from pressure (m)
    pub pressure_alt: f32,
    /// Temperature (degrees C)
    pub temperature: f32,
}

/// GPS data for HIL_GPS message
#[derive(Debug, Clone, Default)]
pub struct HilGpsData {
    /// Timestamp in microseconds since boot
    pub time_usec: u64,
    /// GPS fix type (0-5)
    pub fix_type: u8,
    /// Latitude in degrees * 1e7
    pub lat: i32,
    /// Longitude in degrees * 1e7
    pub lon: i32,
    /// Altitude MSL in millimeters
    pub alt: i32,
    /// GPS HDOP (horizontal dilution of position) * 100
    pub eph: u16,
    /// GPS VDOP (vertical dilution of position) * 100
    pub epv: u16,
    /// GPS ground speed (cm/s)
    pub vel: u16,
    /// GPS velocity north (cm/s)
    pub vn: i16,
    /// GPS velocity east (cm/s)
    pub ve: i16,
    /// GPS velocity down (cm/s)
    pub vd: i16,
    /// Course over ground (degrees * 100, 0.0..359.99)
    pub cog: u16,
    /// Number of satellites visible
    pub satellites_visible: u8,
}

/// Construct a HIL_SENSOR MAVLink message
pub fn make_hil_sensor(data: &HilSensorData) -> MavMessage {
    MavMessage::HIL_SENSOR(HIL_SENSOR_DATA {
        time_usec: data.time_usec,
        xacc: data.xacc,
        yacc: data.yacc,
        zacc: data.zacc,
        xgyro: data.xgyro,
        ygyro: data.ygyro,
        zgyro: data.zgyro,
        xmag: data.xmag,
        ymag: data.ymag,
        zmag: data.zmag,
        abs_pressure: data.abs_pressure,
        diff_pressure: data.diff_pressure,
        pressure_alt: data.pressure_alt,
        temperature: data.temperature,
        fields_updated: HilSensorUpdatedFlags::all(),
    })
}

/// Construct a HIL_GPS MAVLink message
pub fn make_hil_gps(data: &HilGpsData) -> MavMessage {
    MavMessage::HIL_GPS(HIL_GPS_DATA {
        time_usec: data.time_usec,
        fix_type: data.fix_type,
        lat: data.lat,
        lon: data.lon,
        alt: data.alt,
        eph: data.eph,
        epv: data.epv,
        vel: data.vel,
        vn: data.vn,
        ve: data.ve,
        vd: data.vd,
        cog: data.cog,
        satellites_visible: data.satellites_visible,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_make_hil_sensor() {
        let data = HilSensorData {
            time_usec: 1000000,
            xacc: 0.0,
            yacc: 0.0,
            zacc: -9.81,
            ..Default::default()
        };

        let msg = make_hil_sensor(&data);
        match msg {
            MavMessage::HIL_SENSOR(sensor) => {
                assert_eq!(sensor.time_usec, 1000000);
                assert!((sensor.zacc - (-9.81)).abs() < 0.01);
            }
            _ => panic!("Expected HIL_SENSOR message"),
        }
    }

    #[test]
    fn test_make_hil_gps() {
        let data = HilGpsData {
            time_usec: 1000000,
            fix_type: 3,
            lat: 473977418, // 47.3977418 degrees
            lon: 85455963,  // 8.5455963 degrees
            alt: 450000,    // 450m
            satellites_visible: 12,
            ..Default::default()
        };

        let msg = make_hil_gps(&data);
        match msg {
            MavMessage::HIL_GPS(gps) => {
                assert_eq!(gps.time_usec, 1000000);
                assert_eq!(gps.fix_type, 3);
                assert_eq!(gps.lat, 473977418);
                assert_eq!(gps.satellites_visible, 12);
            }
            _ => panic!("Expected HIL_GPS message"),
        }
    }
}
