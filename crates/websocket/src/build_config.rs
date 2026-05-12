use crossbeam_channel::Sender;
use hitl_physics::PhysicsConfig;
use crate::protocol::{AppliedConfig, ConfigResult, ConfigureBuild};
use tracing::{error, info};

const DEFAULT_API_URL: &str = "https://api.th3seus.net";
const MAX_MOTOR_SPEED_SQUARED: f64 = 2500.0 * 2500.0;

pub struct BuildConfigHandler {
    api_url: String,
    http_client: reqwest::Client,
    config_tx: Sender<PhysicsConfig>,
}

impl BuildConfigHandler {
    pub fn new(config_tx: Sender<PhysicsConfig>) -> Self {
        let api_url = std::env::var("RELEASE_API_URL")
            .unwrap_or_else(|_| DEFAULT_API_URL.to_string());

        Self {
            api_url,
            http_client: reqwest::Client::builder()
                .timeout(std::time::Duration::from_secs(10))
                .build()
                .expect("Failed to build HTTP client"),
            config_tx,
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

        let kv = match motor_specs.get("kv").and_then(|v| v.as_f64()) {
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

        let physics = PhysicsConfig::from_motor_specs(
            kv,
            request.prop_diameter_inches,
            request.frame_weight_g,
            motor_weight_g,
        );

        let max_thrust_per_motor_g = (physics.kt * MAX_MOTOR_SPEED_SQUARED) / 9.80665 * 1000.0;
        let thrust_to_weight_ratio = (4.0 * max_thrust_per_motor_g) / (physics.mass_kg * 1000.0);

        let applied = AppliedConfig {
            mass_kg: physics.mass_kg,
            kt: physics.kt,
            kq: physics.kq,
            arm_length_m: physics.arm_length_m,
            max_thrust_per_motor_g,
            thrust_to_weight_ratio,
        };

        if let Err(e) = self.config_tx.send(physics) {
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

        ConfigResult {
            success: true,
            error: None,
            config: Some(applied),
        }
    }

    async fn fetch_motor_specs(&self, slug: &str) -> Result<serde_json::Value, String> {
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
