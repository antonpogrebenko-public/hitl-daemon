use crossbeam_channel::Sender;
use hitl_physics::px4_pids::{compute_pids, fingerprint as pid_fingerprint, Px4Pids};
use hitl_physics::{
    estimate_flight_time_min, BaroChip, BatteryConfig, BuildSpec, FrameMaterial, ImuChip,
    MagChip, PhysicsConfig,
};
use mavlink::ardupilotmega::{MavMessage, MavParamType, PARAM_SET_DATA};
use std::sync::Mutex;
use crate::handler::ValidatedNshCommand;
use crate::protocol::{AppliedConfig, ConfigResult, ConfigureBuild, Px4PidsView};
use tokio::sync::mpsc;
use tracing::{debug, error, info, warn};

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
    /// Fingerprint of the last set of PIDs successfully queued. When the next
    /// `ConfigureBuild` yields the same fingerprint, we skip the param push
    /// to avoid wearing PX4's EEPROM on rapid reconfigures.
    last_pid_fingerprint: Mutex<Option<u64>>,
}

impl BuildConfigHandler {
    pub fn new(
        config_tx: Sender<(PhysicsConfig, BatteryConfig)>,
        nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
        mav_tx: Option<Sender<MavMessage>>,
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
            last_pid_fingerprint: Mutex::new(None),
        }
    }

    pub async fn handle(&self, request: ConfigureBuild) -> ConfigResult {
        let motor_specs = match self.fetch_motor_specs(&request.motor_slug).await {
            Ok(specs) => specs,
            Err(e) => {
                error!(slug = %request.motor_slug, error = %e, "Failed to fetch motor specs");
                return ConfigResult {
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

        // Phase 6: derive per-build rate-controller PIDs and push them to PX4
        // via MAVLink PARAM_SET so light airframes (whose real inertia is well
        // below the legacy 0.012 floor) get controller gains scaled for their
        // actual moments of inertia. Skipped if --sim-only (no PX4 attached)
        // or if the fingerprint matches the last applied set.
        let pids = compute_pids(&physics);
        let applied_pids = self.push_pids_if_changed(&pids);

        let applied = AppliedConfig {
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
            applied_pids,
        };

        if let Err(e) = self.config_tx.send((physics, battery)) {
            error!(error = %e, "Failed to send physics config to simulation");
            return ConfigResult {
                success: false,
                error: Some("Simulation thread unavailable".to_string()),
                config: None,
            };
        }

        info!(
            mass_kg = applied.mass_kg,
            kt = applied.kt,
            twr = applied.thrust_to_weight_ratio,
            "Build configured successfully"
        );

        // Restart EKF2 to clear stale state from previous config
        self.restart_ekf2().await;

        ConfigResult {
            success: true,
            error: None,
            config: Some(applied),
        }
    }

    /// Push the per-build PIDs as `PARAM_SET` MAVLink messages, but only if
    /// they differ from the last set we successfully queued. Returns the view
    /// to surface in `AppliedConfig.applied_pids`, or `None` when we skipped
    /// (sim-only, or fingerprint unchanged).
    ///
    /// This is "fire and forget" — we don't await `PARAM_VALUE` acks. PX4
    /// applies params synchronously on receipt; on the rare drop, the next
    /// `ConfigureBuild` re-sends. Ack-tracking with retry is a follow-up.
    fn push_pids_if_changed(&self, pids: &Px4Pids) -> Option<Px4PidsView> {
        let Some(mav_tx) = self.mav_tx.as_ref() else {
            debug!("sim-only mode: skipping PARAM_SET push for computed PIDs");
            return None;
        };

        let fp = pid_fingerprint(pids);
        {
            let mut cache = self.last_pid_fingerprint.lock().expect("PID cache poisoned");
            if *cache == Some(fp) {
                debug!(fingerprint = fp, "PID fingerprint unchanged — skipping PARAM_SET push");
                // Return None on a skip so the UI can tell the difference between
                // "freshly applied" and "no-op reconfigure".
                return None;
            }
            // Optimistically record the new fingerprint. If a send fails below
            // we'll wipe it so the next attempt retries the whole sequence.
            *cache = Some(fp);
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

        let mut queued = 0u32;
        for (name, value) in params {
            match mav_tx.try_send(make_param_set(name, value)) {
                Ok(()) => queued += 1,
                Err(crossbeam_channel::TrySendError::Full(_)) => {
                    warn!(param = name, "MAVLink tx channel full — dropping PARAM_SET");
                }
                Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                    error!(param = name, "MAVLink tx channel disconnected — aborting param push");
                    // Clear the cache so the next reconfigure retries.
                    *self.last_pid_fingerprint.lock().expect("PID cache poisoned") = None;
                    return None;
                }
            }
        }
        info!(queued, fingerprint = fp, "Queued PARAM_SET sequence for per-build PIDs");

        Some(Px4PidsView {
            roll_p: pids.roll_p,    roll_i: pids.roll_i,    roll_d: pids.roll_d,    roll_ff: pids.roll_ff,
            pitch_p: pids.pitch_p,  pitch_i: pids.pitch_i,  pitch_d: pids.pitch_d,  pitch_ff: pids.pitch_ff,
            yaw_p: pids.yaw_p,      yaw_i: pids.yaw_i,      yaw_d: pids.yaw_d,      yaw_ff: pids.yaw_ff,
        })
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
