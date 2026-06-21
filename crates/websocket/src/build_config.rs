use crate::handler::ValidatedNshCommand;
use crate::protocol::{
    AppliedConfig, ConfigResult, ConfigState, ConfigureBuild, OutgoingMessage, Px4PidsView,
};
use crossbeam_channel::Sender;
use hitl_physics::px4_pids::{compute_pids, fingerprint as pid_fingerprint, Px4Pids};
use hitl_physics::{
    estimate_flight_time_min, BaroChip, BatteryConfig, BuildSpec, FrameMaterial, ImuChip, MagChip,
    PhysicsConfig,
};
use mavlink::ardupilotmega::{MavCmd, MavMessage, MavParamType, COMMAND_LONG_DATA, PARAM_SET_DATA};
use std::sync::Mutex;
use std::time::Duration;
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
    config_tx: Sender<(PhysicsConfig, BatteryConfig, hitl_sensors::SensorsConfig)>,
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
    /// The last successfully verified PID set + thrust params. Used by
    /// `repush_if_configured` to re-push on FC reconnect without requiring
    /// a new `ConfigureBuild` from the browser.
    last_verified_params: Mutex<Option<LastVerifiedParams>>,
    /// Broadcast channel for system-initiated `ConfigResult` messages (e.g.
    /// those emitted by `repush_if_configured` on FC reconnect). The
    /// WebSocket server subscribes and forwards to all connected clients.
    system_config_tx: broadcast::Sender<OutgoingMessage>,
}

/// Snapshot of the parameters that were last successfully verified on PX4.
/// Stored so `repush_if_configured` can re-push them after a FC power cycle.
#[derive(Clone)]
struct LastVerifiedParams {
    pids: Px4Pids,
    hover_cmd: f32,
    thr_min: f32,
}

impl BuildConfigHandler {
    pub fn new(
        config_tx: Sender<(PhysicsConfig, BatteryConfig, hitl_sensors::SensorsConfig)>,
        nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
        mav_tx: Option<Sender<MavMessage>>,
        param_value_tx: Option<broadcast::Sender<(String, f32)>>,
    ) -> Self {
        let api_url =
            std::env::var("RELEASE_API_URL").unwrap_or_else(|_| DEFAULT_API_URL.to_string());

        let (system_config_tx, _) = broadcast::channel(16);

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
            last_verified_params: Mutex::new(None),
            system_config_tx,
        }
    }

    /// Subscribe to system-initiated `ConfigResult` messages.  The WebSocket
    /// server calls this once during setup so reconnect-triggered results are
    /// forwarded to all connected browsers.
    pub fn subscribe_system_config(&self) -> broadcast::Receiver<OutgoingMessage> {
        self.system_config_tx.subscribe()
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
        let (prop_diameter, prop_pitch, blade_count) = if let Some(ref prop_slug) =
            request.prop_slug
        {
            match self.fetch_component_specs(prop_slug).await {
                Ok(specs) => {
                    let diameter = specs
                        .get("diameterIn")
                        .and_then(|v| v.as_f64())
                        .unwrap_or(request.prop_diameter_inches);
                    let pitch = specs
                        .get("pitchIn")
                        .and_then(|v| v.as_f64())
                        .unwrap_or(diameter * 0.9);
                    let blades = specs
                        .get("bladeCount")
                        .and_then(|v| v.as_i64())
                        .unwrap_or(3) as i32;
                    info!(slug = %prop_slug, diameter, pitch, blades, "Loaded propeller specs");
                    (diameter, pitch, blades)
                }
                Err(e) => {
                    warn!(slug = %prop_slug, error = %e, "Failed to fetch propeller specs, using defaults");
                    (
                        request.prop_diameter_inches,
                        request.prop_diameter_inches * 0.9,
                        3,
                    )
                }
            }
        } else {
            (
                request.prop_diameter_inches,
                request.prop_diameter_inches * 0.9,
                3,
            )
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
        // Estimate battery weight from capacity when no battery_slug will provide
        // an exact value. Regression across FPV packs: ~7g per cell per 100mAh
        // (4S 1500mAh ≈ 210g, 4S 4500mAh ≈ 630g, 6S 1100mAh ≈ 230g).
        spec.battery.weight_g =
            request.battery_capacity_mah * request.battery_cell_count as f64 * 0.035;

        // Fetch frame specs if frame_slug provided — gets wheelbase/material
        // but frame_weight_g from the request always takes priority (user-tunable).
        if let Some(ref frame_slug) = request.frame_slug {
            match self.fetch_component_specs(frame_slug).await {
                Ok(specs) => {
                    if let Some(wheelbase) = specs.get("wheelbaseMm").and_then(|v| v.as_f64()) {
                        spec.frame.wheelbase_mm = wheelbase;
                    }
                    if let Some(material_str) = specs.get("material").and_then(|v| v.as_str()) {
                        spec.frame.material = Some(parse_frame_material(material_str));
                    }
                    info!(slug = %frame_slug, weight_g = spec.frame.weight_g, wheelbase_mm = spec.frame.wheelbase_mm, "Loaded frame specs (weight from user override)");
                }
                Err(e) => {
                    warn!(slug = %frame_slug, error = %e, "Failed to fetch frame specs, using defaults");
                }
            }
        }

        // Fetch ESC specs if esc_slug provided
        if let Some(ref esc_slug) = request.esc_slug {
            match self.fetch_component_specs(esc_slug).await {
                Ok(specs) => {
                    if let Some(continuous) =
                        specs.get("continuousCurrentA").and_then(|v| v.as_f64())
                    {
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

        // Fetch battery specs if battery_slug provided — sets weight for correct mass
        if let Some(ref battery_slug) = request.battery_slug {
            match self.fetch_component_specs(battery_slug).await {
                Ok(specs) => {
                    if let Some(weight) = specs.get("weightG").and_then(|v| v.as_f64()) {
                        spec.battery.weight_g = weight;
                    }
                    info!(slug = %battery_slug, weight_g = spec.battery.weight_g, "Loaded battery specs");
                }
                Err(e) => {
                    warn!(slug = %battery_slug, error = %e, "Failed to fetch battery specs, using default weight");
                }
            }
        }

        // Fetch GPS specs if gps_slug provided
        if let Some(ref gps_slug) = request.gps_slug {
            match self.fetch_component_specs(gps_slug).await {
                Ok(specs) => {
                    let chipset = specs
                        .get("chipset")
                        .and_then(|v| v.as_str())
                        .map(parse_gps_chipset)
                        .unwrap_or(hitl_physics::build::GpsChipset::Other);
                    let update_rate_hz = specs
                        .get("updateRateHz")
                        .and_then(|v| v.as_f64())
                        .unwrap_or(10.0)
                        .clamp(1.0, 100.0);
                    let has_compass = specs
                        .get("compass")
                        .or_else(|| specs.get("hasCompass"))
                        .and_then(|v| v.as_bool())
                        .unwrap_or(false);
                    let weight_g = specs
                        .get("weightG")
                        .and_then(|v| v.as_f64())
                        .unwrap_or(12.0);
                    spec.gps = Some(hitl_physics::build::GpsSpec {
                        chipset,
                        update_rate_hz,
                        has_compass,
                        weight_g,
                    });
                    info!(slug = %gps_slug, weight_g, update_rate_hz, "Loaded GPS specs");
                }
                Err(e) => {
                    warn!(slug = %gps_slug, error = %e, "Failed to fetch GPS specs, using defaults");
                }
            }
        }

        // ESC count: 4-in-1 (count=1) weighs once, individual (count=4) weight × 4
        if request.esc_count > 1 {
            if let Some(per_unit) = spec.escs.weight_g {
                spec.escs.weight_g = Some(per_unit * request.esc_count as f64);
            }
        }

        // Build sensor config from FC/GPS chip profiles.
        // For HITL: disable bias drift (causes EKF divergence) but use chip-specific
        // noise densities for realistic sensor behavior.
        let sensor_profiles = spec.to_sensor_profiles();
        // Override chipset default update_rate_hz with the API-reported value
        // from the specific GPS product listing.
        let gps_profile = spec.to_gps_profile().map(|mut p| {
            if let Some(gps) = &spec.gps {
                p.update_rate_hz = gps.update_rate_hz;
            }
            p
        });
        let sensors_config = build_sensors_config(&sensor_profiles, gps_profile.as_ref());

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

        // Hover cmd for MPC_THR_HOVER. The torque-balance model in
        // `max_motor_speed_from_voltage()` already accounts for prop loading,
        // so `hover_throttle_percent()` = hover_omega / max_omega gives the
        // correct motor command fraction at static (full-charge) voltage.
        //
        // No sag correction: the sim starts at full voltage and applies
        // voltage-sag scaling (`v_terminal / v_nominal`) in the physics loop
        // as the battery discharges. PX4's velocity-controller integral adapts
        // to the slowly changing plant gain — a static feedforward is correct.
        //
        // The previous 0.7225 sag factor was sized for the legacy inflated-thrust
        // model (TWR~8, hover~12%); with the recalibrated model (TWR~2, hover~45%)
        // it inflated MPC_THR_HOVER to 62% vs actual 44%, making the position
        // controller unable to take off after landing (sess113).
        let hover_cmd = physics.hover_throttle_percent().clamp(0.1, 0.8) as f32;

        // Phase 6: derive per-build rate-controller PIDs. Scales by inertia AND
        // by hover authority — high-TWR builds get attenuated gains to prevent
        // motor-saturation limit cycles.
        let pids = compute_pids(&physics, hover_cmd);

        // MPC_THR_MIN — minimum thr_desired PX4's altitude controller is
        // allowed to command. PX4's default 0.12 is sized for TWR≈2 builds
        // where it's well below hover (0.5). For high-TWR racers (TWR=8 →
        // hover=0.12), the default IS above hover, so the controller cannot
        // command less thrust than hover → drone physically cannot descend,
        // and position-mode lock-down devolves into a ~0.8 Hz limit cycle
        // (log100.ulg, 2026-05-17: pitch_act swings ±300°/s while pitch_sp
        // stays ±30°/s — that's the position loop fighting an unreachable
        // altitude setpoint).
        //
        // 30% of hover gives ≥0.5 g of authority for controlled descent,
        // clamped to PX4's accepted [0.05, 0.20] range.
        let thr_min = (hover_cmd * 0.3).clamp(0.05, 0.20);

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
            hover_cmd,
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
        let (applied_pids, verified_params) =
            match self.push_pids_and_verify(&pids, hover_cmd, thr_min).await {
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
        if let Err(e) = self.config_tx.send((physics, battery, sensors_config)) {
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

        if let Err(e) = self.restart_ekf2_with_retry(3).await {
            error!(error = %e, "EKF2 restart failed — PIDs are applied, EKF will converge on stale state");
        }

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
        hover_cmd: f32,
        thr_min: f32,
    ) -> Result<(Option<Px4PidsView>, u32), String> {
        let (Some(mav_tx), Some(param_value_tx)) =
            (self.mav_tx.as_ref(), self.param_value_tx.as_ref())
        else {
            debug!("sim-only mode: skipping PARAM_SET push (no PX4 attached)");
            return Ok((None, 0));
        };

        // Fingerprint mixes PID gains with hover_cmd in the high 32 bits and
        // thr_min in the middle so a TWR change forces a re-push even when
        // the rate PIDs are identical. (hover_cmd and thr_min both follow
        // 1/TWR, so a TWR change always flips both — but XOR-mixing both
        // keeps the fingerprint correct if either is ever pushed independently.)
        let fp = pid_fingerprint(pids)
            ^ ((hover_cmd.to_bits() as u64) << 32)
            ^ ((thr_min.to_bits() as u64) << 16);
        {
            let cache = self
                .last_pid_fingerprint
                .lock()
                .expect("PID cache poisoned");
            if *cache == Some(fp) {
                debug!(
                    fingerprint = fp,
                    "PID + hover_cmd fingerprint unchanged — skipping PARAM_SET push"
                );
                return Ok((None, 0));
            }
        }

        // Three thrust-curve params accompany the 12 rate PIDs:
        // - THR_MDL_FAC=1 tells PX4 to invert the quadratic actuator response
        //   by outputting `cmd = sqrt(thr_desired)`. This matches the sim's
        //   linear cmd→ω model (and real ESC behavior), so the round-trip
        //   PX4-controller→actuator→sim→thrust is linear in `thr_desired`.
        // - MPC_THR_HOVER tells the position controller what `thr_desired`
        //   produces hover. PX4's 0.5 default only matches TWR=2; high-TWR
        //   racers (TWR=8 → hover≈0.12) leave the altitude integrator fighting
        //   a 4× thrust overshoot on every position-hold cycle.
        // - MPC_THR_MIN sets the lowest thrust PX4's altitude controller can
        //   command. The 0.12 default is fine when hover≫0.12, but on a
        //   high-TWR build hover *is* 0.12 — the floor pins thrust at or
        //   above weight, the drone can't descend, position mode locks into
        //   a ~0.8 Hz limit cycle. Scaled to 30% of hover (≥0.5 g descent
        //   authority), clamped to PX4's [0.05, 0.20] range.
        let params: [(&str, f32); 21] = [
            ("MC_ROLLRATE_P", pids.roll_p),
            ("MC_ROLLRATE_I", pids.roll_i),
            ("MC_ROLLRATE_D", pids.roll_d),
            ("MC_ROLLRATE_FF", pids.roll_ff),
            ("MC_PITCHRATE_P", pids.pitch_p),
            ("MC_PITCHRATE_I", pids.pitch_i),
            ("MC_PITCHRATE_D", pids.pitch_d),
            ("MC_PITCHRATE_FF", pids.pitch_ff),
            ("MC_YAWRATE_P", pids.yaw_p),
            ("MC_YAWRATE_I", pids.yaw_i),
            ("MC_YAWRATE_D", pids.yaw_d),
            ("MC_YAWRATE_FF", pids.yaw_ff),
            ("THR_MDL_FAC", 1.0),
            ("MPC_THR_HOVER", hover_cmd),
            ("MPC_THR_MIN", thr_min),
            // Zero accel/gyro calibration offsets: the simulated IMU has no
            // physical mounting bias. Real-hardware offsets create a persistent
            // lateral accel bias (~0.05 m/s²) that the EKF integrates into
            // position drift.
            ("CAL_ACC0_XOFF", 0.0),
            ("CAL_ACC0_YOFF", 0.0),
            ("CAL_ACC0_ZOFF", 0.0),
            ("CAL_GYRO0_XOFF", 0.0),
            ("CAL_GYRO0_YOFF", 0.0),
            ("CAL_GYRO0_ZOFF", 0.0),
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
                            attempt, "MAVLink tx channel full — retrying PARAM_SET"
                        );
                        tokio::time::sleep(Duration::from_millis(50)).await;
                        continue;
                    }
                    Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                        return Err(format!("MAVLink tx disconnected while sending {name}"));
                    }
                }

                if let Some((got_name, got_value)) = wait_for_param_ack(&mut rx, name, value).await
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
        *self
            .last_pid_fingerprint
            .lock()
            .expect("PID cache poisoned") = Some(fp);
        // Persist the verified param snapshot so repush_if_configured can
        // re-push the same values after a FC power cycle without needing a
        // new ConfigureBuild from the browser.
        *self
            .last_verified_params
            .lock()
            .expect("PID params cache poisoned") = Some(LastVerifiedParams {
            pids: pids.clone(),
            hover_cmd,
            thr_min,
        });

        // Persist to PX4 flash. PARAM_SET only writes RAM, so a Pixhawk
        // reboot resets our PIDs / thrust-curve params back to PX4 defaults
        // and the user sees "trembling is back" without knowing why. PX4
        // doesn't ack the storage write reliably (EEPROM commit takes
        // ~100 ms and the COMMAND_ACK is best-effort) — we fire-and-forget,
        // because the worst case (silent failure) is identical to the
        // current behaviour and a subsequent ConfigureBuild re-pushes
        // everything anyway.
        match mav_tx.try_send(make_param_save()) {
            Ok(()) => debug!("Sent MAV_CMD_PREFLIGHT_STORAGE (write) — params will survive reboot"),
            Err(e) => warn!(error = ?e, "Failed to send PARAM_SAVE — params live in RAM only"),
        }

        info!(
            verified,
            fingerprint = fp,
            "All per-build PIDs verified on PX4"
        );

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

    /// Attempt a single EKF2 stop → start cycle via NSH.
    ///
    /// Returns `Ok(())` when both commands are queued, or `Err(msg)` if the
    /// NSH channel is unavailable or either send fails.
    async fn restart_ekf2(&self) -> Result<(), String> {
        let Some(ref nsh_tx) = self.nsh_tx else {
            return Err("NSH channel not available".to_string());
        };

        let stop_cmd = ValidatedNshCommand {
            request_id: 0xFFFF_FF01, // Special ID for internal commands
            command: "ekf2 stop".to_string(),
            timeout_ms: 2000,
            client_id: 0, // System client
        };

        nsh_tx
            .send(stop_cmd)
            .await
            .map_err(|e| format!("Failed to send ekf2 stop: {e}"))?;

        // Small delay to let EKF2 stop cleanly before issuing start.
        tokio::time::sleep(Duration::from_millis(100)).await;

        let start_cmd = ValidatedNshCommand {
            request_id: 0xFFFF_FF02,
            command: "ekf2 start".to_string(),
            timeout_ms: 2000,
            client_id: 0,
        };

        nsh_tx
            .send(start_cmd)
            .await
            .map_err(|e| format!("Failed to send ekf2 start: {e}"))?;

        Ok(())
    }

    /// Retry `restart_ekf2` up to `max_retries` times with 200 ms between
    /// attempts.  Returns `Ok(())` on the first success.  If every attempt
    /// fails, returns `Err` with the final error message; the caller is
    /// responsible for deciding whether to propagate or merely log it.
    async fn restart_ekf2_with_retry(&self, max_retries: u8) -> Result<(), String> {
        for attempt in 0..max_retries {
            match self.restart_ekf2().await {
                Ok(()) => {
                    info!(attempt = attempt + 1, "EKF2 restarted after config change");
                    return Ok(());
                }
                Err(e) => {
                    if attempt < max_retries - 1 {
                        warn!(
                            attempt = attempt + 1,
                            max_retries,
                            error = %e,
                            "EKF2 restart attempt failed, retrying in 200 ms"
                        );
                        tokio::time::sleep(Duration::from_millis(200)).await;
                    } else {
                        return Err(format!(
                            "EKF2 restart failed after {max_retries} attempts: {e}"
                        ));
                    }
                }
            }
        }
        // `max_retries == 0` edge case: nothing to do, treat as success.
        Ok(())
    }

    /// Re-push the last verified PID + thrust-curve parameters to PX4 after a
    /// FC power cycle / reconnect.
    ///
    /// Behaviour:
    /// - If no config was ever applied during this session, returns `Ok(())`
    ///   immediately — nothing to push.
    /// - Clears the PID fingerprint cache before pushing so the dedup check
    ///   inside `push_pids_and_verify` does not skip the re-push.
    /// - Broadcasts `ConfigState::Configuring` on `system_config_tx` so all
    ///   connected browser clients can show a spinner, then `ConfigState::Ready`
    ///   on success (or `ConfigState::Error` on failure).  Broadcasts silently
    ///   drop when no clients are subscribed.
    /// - Only re-pushes PIDs to PX4 — does NOT resend `PhysicsConfig` to the
    ///   simulation loop (physics are already correct in the running sim).
    /// - Returns `Err` only if the PARAM_SET sequence fails, so the caller
    ///   can log a warning; the sim continues with whatever PX4 has loaded.
    pub async fn repush_if_configured(&self) -> Result<(), String> {
        // Snapshot the last verified params under the lock, then release.
        let params = {
            self.last_verified_params
                .lock()
                .expect("PID params cache poisoned")
                .clone()
        };

        let Some(p) = params else {
            debug!("repush_if_configured: no prior config in this session — nothing to push");
            return Ok(());
        };

        info!(
            hover_cmd = p.hover_cmd,
            thr_min = p.thr_min,
            "FC reconnected — re-pushing last verified PIDs"
        );

        // Notify the frontend that re-configuration is in progress.
        let _ = self
            .system_config_tx
            .send(OutgoingMessage::ConfigResult(ConfigResult {
                state: ConfigState::Configuring,
                success: true,
                error: None,
                config: None,
            }));

        // Clear the fingerprint cache so push_pids_and_verify doesn't skip
        // the push — PX4 just reset its RAM on power cycle.
        *self
            .last_pid_fingerprint
            .lock()
            .expect("PID cache poisoned") = None;

        match self
            .push_pids_and_verify(&p.pids, p.hover_cmd, p.thr_min)
            .await
        {
            Ok(_) => {
                info!("PIDs re-verified on reconnected FC");
                let _ = self
                    .system_config_tx
                    .send(OutgoingMessage::ConfigResult(ConfigResult {
                        state: ConfigState::Ready,
                        success: true,
                        error: None,
                        config: None,
                    }));
                Ok(())
            }
            Err(e) => {
                error!(error = %e, "PID re-push failed after FC reconnect");
                let _ = self
                    .system_config_tx
                    .send(OutgoingMessage::ConfigResult(ConfigResult {
                        state: ConfigState::Error,
                        success: false,
                        error: Some(format!("PID re-push after reconnect failed: {e}")),
                        config: None,
                    }));
                Err(e)
            }
        }
    }

    async fn fetch_motor_specs(&self, slug: &str) -> Result<serde_json::Value, String> {
        self.fetch_component_specs(slug).await
    }

    async fn fetch_component_specs(&self, slug: &str) -> Result<serde_json::Value, String> {
        let url = format!("{}/api/components/{}", self.api_url, slug);
        let resp = self
            .http_client
            .get(&url)
            .send()
            .await
            .map_err(|e| format!("HTTP request failed: {e}"))?;

        if !resp.status().is_success() {
            return Err(format!("API returned {}", resp.status()));
        }

        let body: serde_json::Value = resp
            .json()
            .await
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

/// MAV_CMD_PREFLIGHT_STORAGE with `param1 = 1.0` (write parameter storage).
/// PX4 commits the in-RAM parameter table to flash so subsequent reboots
/// keep the per-build PIDs / `MPC_THR_HOVER` / `THR_MDL_FAC` we just pushed.
fn make_param_save() -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: PX4_TARGET_SYSTEM,
        target_component: PX4_TARGET_COMPONENT,
        confirmation: 0,
        command: MavCmd::MAV_CMD_PREFLIGHT_STORAGE,
        param1: 1.0, // 1 = write to storage; 0 = read; 2 = reset to defaults
        param2: 0.0, // mission storage: leave untouched
        param3: 0.0, // logging rate: leave untouched
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
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

fn parse_gps_chipset(s: &str) -> hitl_physics::build::GpsChipset {
    use hitl_physics::build::GpsChipset;
    let normalized = s.to_lowercase().replace(['-', '_', ' '], "");
    if normalized.contains("f9p") || normalized.contains("zedf9p") {
        return GpsChipset::UbloxF9P;
    }
    if normalized.contains("m10") {
        return GpsChipset::UbloxM10;
    }
    if normalized.contains("m9") {
        return GpsChipset::UbloxM9N;
    }
    if normalized.contains("m8") {
        return GpsChipset::UbloxM8N;
    }
    if normalized.contains("mosaic") || normalized.contains("septentrio") {
        return GpsChipset::SeptentrioMosaic;
    }
    if normalized.contains("here3") {
        return GpsChipset::Here3;
    }
    GpsChipset::Other
}

/// Construct a `SensorsConfig` from the build's FC sensor profiles and GPS profile.
///
/// For HITL: bias drift is always disabled (causes EKF divergence in sim) but
/// chip-specific noise densities are preserved for realistic sensor behavior.
fn build_sensors_config(
    profiles: &hitl_physics::build::SensorProfiles,
    gps_profile: Option<&hitl_physics::build::GpsProfile>,
) -> hitl_sensors::SensorsConfig {
    use hitl_sensors::{BaroConfig, GpsConfig, ImuConfig, MagConfig, SensorsConfig};

    let imu = ImuConfig {
        gyro_noise_density: profiles.imu.gyro_noise_density,
        accel_noise_density: profiles.imu.accel_noise_density,
        gyro_bias_sigma: 0.0, // CRITICAL: no drift in HITL
        gyro_bias_tau: 1000.0,
        accel_bias_sigma: 0.0, // CRITICAL: no drift in HITL
        accel_bias_tau: 1000.0,
    };

    let gps = if let Some(gp) = gps_profile {
        GpsConfig {
            horizontal_noise_sigma: gp.horizontal_noise_sigma_m,
            altitude_noise_sigma: gp.altitude_noise_sigma_m,
            velocity_noise_sigma: gp.velocity_noise_sigma_mps,
            position_drift_sigma: 0.0, // No drift in HITL
            position_drift_tau: 1000.0,
            update_rate_hz: gp.update_rate_hz,
            delay_ms: gp.delay_ms,
        }
    } else {
        // No GPS module selected — use tight defaults for HITL
        GpsConfig {
            horizontal_noise_sigma: 0.1,
            altitude_noise_sigma: 0.3,
            velocity_noise_sigma: 0.05,
            position_drift_sigma: 0.0,
            position_drift_tau: 1000.0,
            update_rate_hz: 10.0,
            delay_ms: 80.0,
        }
    };

    let baro = BaroConfig {
        noise_sigma: profiles.baro.noise_sigma_m,
        ..BaroConfig::default()
    };

    let mag = if let Some(mp) = profiles.mag {
        MagConfig {
            noise_sigma_gauss: mp.noise_sigma_gauss,
            ..MagConfig::default()
        }
    } else {
        MagConfig::default()
    };

    SensorsConfig {
        imu,
        gps,
        baro,
        mag,
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
        let (config_tx, _config_rx) =
            bounded::<(PhysicsConfig, BatteryConfig, hitl_sensors::SensorsConfig)>(4);
        BuildConfigHandler::new(config_tx, None, mav_tx, param_value_tx)
    }

    /// Hover throttle representative of a TWR≈2 build (1/2). Used by tests to
    /// exercise the thrust-curve params alongside the rate PIDs.
    const REF_HOVER_CMD: f32 = 0.5;
    /// Slightly different hover cmd, used to verify a TWR change re-pushes
    /// even when the rate PIDs are identical.
    const REF_HOVER_CMD_ALT: f32 = 0.3;
    /// MPC_THR_MIN representative of the same TWR≈2 build (30% of hover,
    /// clamped — matches the production formula).
    const REF_THR_MIN: f32 = 0.15;

    /// Captured outbound MAVLink message — either a PARAM_SET (`name`, `value`)
    /// or a COMMAND_LONG identified by command id. Tests use this to verify
    /// the daemon pushes the right param set *and* the trailing PARAM_SAVE.
    #[derive(Debug, Clone)]
    enum CapturedMsg {
        ParamSet(String, f32),
        CommandLong(u32),
    }

    fn spawn_fake_px4_with_capture(
        mav_rx: crossbeam_channel::Receiver<MavMessage>,
        param_value_tx: broadcast::Sender<(String, f32)>,
    ) -> (
        tokio::task::JoinHandle<()>,
        std::sync::Arc<Mutex<Vec<CapturedMsg>>>,
    ) {
        let captured = std::sync::Arc::new(Mutex::new(Vec::<CapturedMsg>::new()));
        let captured_clone = captured.clone();
        let handle = tokio::task::spawn_blocking(move || {
            while let Ok(msg) = mav_rx.recv() {
                match msg {
                    MavMessage::PARAM_SET(p) => {
                        let name = std::str::from_utf8(&p.param_id)
                            .unwrap_or("")
                            .trim_end_matches('\0')
                            .to_string();
                        captured_clone
                            .lock()
                            .expect("capture mutex poisoned")
                            .push(CapturedMsg::ParamSet(name.clone(), p.param_value));
                        let _ = param_value_tx.send((name, p.param_value));
                    }
                    MavMessage::COMMAND_LONG(c) => {
                        captured_clone
                            .lock()
                            .expect("capture mutex poisoned")
                            .push(CapturedMsg::CommandLong(c.command as u32));
                    }
                    _ => {}
                }
            }
        });
        (handle, captured)
    }

    #[tokio::test]
    async fn sim_only_mode_skips_verification() {
        let handler = make_handler(None, None);
        let result = handler
            .push_pids_and_verify(&REF_PIDS, REF_HOVER_CMD, REF_THR_MIN)
            .await;
        assert!(matches!(result, Ok((None, 0))));
    }

    #[tokio::test]
    async fn happy_path_acks_all_fifteen_params() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        let (_px4, captured) = spawn_fake_px4_with_capture(mav_rx, pv_tx.clone());

        let handler = make_handler(Some(mav_tx), Some(pv_tx));
        let (view, verified) = handler
            .push_pids_and_verify(&REF_PIDS, REF_HOVER_CMD, REF_THR_MIN)
            .await
            .unwrap();
        assert!(view.is_some());
        assert_eq!(verified, 21);

        let snapshot = captured.lock().unwrap().clone();
        let param_names: Vec<&str> = snapshot
            .iter()
            .filter_map(|m| match m {
                CapturedMsg::ParamSet(n, _) => Some(n.as_str()),
                _ => None,
            })
            .collect();
        assert!(param_names.contains(&"MPC_THR_HOVER"));
        assert!(param_names.contains(&"THR_MDL_FAC"));
        assert!(param_names.contains(&"MPC_THR_MIN"));

        let hover_value = snapshot
            .iter()
            .find_map(|m| match m {
                CapturedMsg::ParamSet(n, v) if n == "MPC_THR_HOVER" => Some(*v),
                _ => None,
            })
            .expect("MPC_THR_HOVER not pushed");
        assert!((hover_value - REF_HOVER_CMD).abs() < 1e-4);
    }

    #[tokio::test]
    async fn one_dropped_ack_recovers_via_retry() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        // Drop only the first PARAM_SET — the retry should land an ack.
        let _px4 = spawn_fake_px4(mav_rx, pv_tx.clone(), 1);

        let handler = make_handler(Some(mav_tx), Some(pv_tx));
        let (view, verified) = handler
            .push_pids_and_verify(&REF_PIDS, REF_HOVER_CMD, REF_THR_MIN)
            .await
            .unwrap();
        assert!(view.is_some());
        assert_eq!(verified, 21);
    }

    #[tokio::test]
    async fn persistent_silence_returns_error() {
        let (mav_tx, _mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        // No fake PX4 — every PARAM_SET sits in mav_rx forever and no ack ever
        // comes back. Each param exhausts PARAM_RETRY_COUNT attempts.
        let handler = make_handler(Some(mav_tx), Some(pv_tx));
        let err = handler
            .push_pids_and_verify(&REF_PIDS, REF_HOVER_CMD, REF_THR_MIN)
            .await
            .unwrap_err();
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
        let (_, verified_first) = handler
            .push_pids_and_verify(&REF_PIDS, REF_HOVER_CMD, REF_THR_MIN)
            .await
            .unwrap();
        assert_eq!(verified_first, 21);

        // Identical PIDs + hover_cmd → fingerprint matches → no push, no acks needed.
        let (view, verified_second) = handler
            .push_pids_and_verify(&REF_PIDS, REF_HOVER_CMD, REF_THR_MIN)
            .await
            .unwrap();
        assert!(view.is_none());
        assert_eq!(verified_second, 0);
    }

    #[tokio::test]
    async fn happy_path_sends_param_save_after_verification() {
        // MAV_CMD_PREFLIGHT_STORAGE — the command id we expect the daemon
        // to send right after the 14 PARAM_SETs verify.
        const MAV_CMD_PREFLIGHT_STORAGE: u32 = 245;

        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        let (_px4, captured) = spawn_fake_px4_with_capture(mav_rx, pv_tx.clone());

        let handler = make_handler(Some(mav_tx), Some(pv_tx));
        let (_, verified) = handler
            .push_pids_and_verify(&REF_PIDS, REF_HOVER_CMD, REF_THR_MIN)
            .await
            .unwrap();
        assert_eq!(verified, 21);

        // The PARAM_SAVE is `try_send`-d on the *same* mav channel that the
        // fake PX4 drains. push_pids_and_verify returns as soon as the 14th
        // PARAM_VALUE ack arrives; the PARAM_SAVE goes out right after but
        // the fake's `recv` loop runs on a separate thread, so give it a
        // tick to drain.
        tokio::time::sleep(Duration::from_millis(50)).await;

        let snapshot = captured.lock().unwrap().clone();
        let param_sets = snapshot
            .iter()
            .filter(|m| matches!(m, CapturedMsg::ParamSet(_, _)))
            .count();
        let saw_storage_write = snapshot
            .iter()
            .any(|m| matches!(m, CapturedMsg::CommandLong(c) if *c == MAV_CMD_PREFLIGHT_STORAGE));
        assert_eq!(
            param_sets, 21,
            "expected 21 PARAM_SETs (15 PID/thrust + 6 cal offsets)"
        );
        assert!(
            saw_storage_write,
            "expected MAV_CMD_PREFLIGHT_STORAGE (245) after the PARAM_SETs — \
             without this, a Pixhawk reboot drops our config to PX4 defaults"
        );
    }

    #[tokio::test]
    async fn hover_cmd_change_re_pushes_even_when_pids_unchanged() {
        let (mav_tx, mav_rx) = bounded::<MavMessage>(64);
        let (pv_tx, _) = broadcast::channel::<(String, f32)>(64);
        let _px4 = spawn_fake_px4(mav_rx, pv_tx.clone(), 0);

        let handler = make_handler(Some(mav_tx), Some(pv_tx));
        let (_, verified_first) = handler
            .push_pids_and_verify(&REF_PIDS, REF_HOVER_CMD, REF_THR_MIN)
            .await
            .unwrap();
        assert_eq!(verified_first, 21);

        // Same PIDs but a different TWR — must re-push so PX4 picks up the
        // new MPC_THR_HOVER. A TWR change without re-pushing was the original
        // cause of position-mode oscillation.
        let (view, verified_second) = handler
            .push_pids_and_verify(&REF_PIDS, REF_HOVER_CMD_ALT, REF_THR_MIN)
            .await
            .unwrap();
        assert!(view.is_some());
        assert_eq!(verified_second, 21);
    }
}
