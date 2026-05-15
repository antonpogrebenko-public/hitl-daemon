use crossbeam_channel::Sender;
use hitl_physics::{estimate_flight_time_min, BatteryConfig, PhysicsConfig};
use crate::handler::ValidatedNshCommand;
use crate::protocol::{AppliedConfig, ConfigResult, ConfigureBuild};
use tokio::sync::mpsc;
use tracing::{error, info, warn};

const DEFAULT_API_URL: &str = "https://api.th3seus.net";

pub struct BuildConfigHandler {
    api_url: String,
    http_client: reqwest::Client,
    config_tx: Sender<(PhysicsConfig, BatteryConfig)>,
    nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>,
}

impl BuildConfigHandler {
    pub fn new(config_tx: Sender<(PhysicsConfig, BatteryConfig)>, nsh_tx: Option<mpsc::Sender<ValidatedNshCommand>>) -> Self {
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

        let physics = PhysicsConfig::from_build_specs(
            kv,
            prop_diameter,
            prop_pitch,
            blade_count,
            request.frame_weight_g,
            motor_weight_g,
            request.battery_voltage,
        );

        let battery = BatteryConfig::new(
            request.battery_cell_count,
            request.battery_capacity_mah,
            75.0,
        );

        let max_omega = physics.max_motor_speed_from_voltage();
        let max_thrust_per_motor_g = (physics.kt * max_omega * max_omega) / 9.80665 * 1000.0;
        let thrust_to_weight_ratio = (4.0 * max_thrust_per_motor_g) / (physics.mass_kg * 1000.0);

        let max_motor_rpm = physics.motor_kv * physics.battery_voltage;
        let flight_time = estimate_flight_time_min(&battery, &physics);

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
