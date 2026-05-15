# hitl-daemon

Rust Hardware-in-the-Loop daemon for PX4 flight controller simulation.

## Overview

Connects to a Pixhawk via USB serial, runs 400 Hz physics simulation, and:
- Receives actuator commands from PX4 (HIL_ACTUATOR_CONTROLS)
- Sends simulated sensor data (HIL_SENSOR, HIL_GPS)
- Bridges to QGroundControl via UDP
- Serves WebSocket for browser UI and NSH access
- Accepts runtime build configuration (motor/prop/battery) from web UI
- Simulates battery discharge with motor cutoff on depletion

## Quick Start

```bash
# With Pixhawk connected
cargo run -- --port /dev/tty.usbmodem01

# Simulation-only mode (no hardware)
cargo run -- --sim-only

# Custom reference position
cargo run -- --port /dev/tty.usbmodem01 --lat 37.7749 --lon -122.4194 --alt 10
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      hitl-daemon                            │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐ │
│  │ mavlink- │   │ protocol │   │simulation│   │websocket │ │
│  │   io     │   │          │   │          │   │          │ │
│  └──────────┘   └──────────┘   └──────────┘   └──────────┘ │
├─────────────────────────────────────────────────────────────┤
│                External Dependencies                        │
│  ┌──────────────┐   ┌──────────────┐                       │
│  │ hitl-physics │   │ hitl-sensors │                       │
│  └──────────────┘   └──────────────┘                       │
└─────────────────────────────────────────────────────────────┘
```

## Crate Structure

```
hitl-daemon/
├── src/main.rs           # Main orchestrator, threading
├── crates/
│   ├── mavlink-io/       # Serial port, MAVLink codec, async I/O
│   ├── protocol/         # Shared types: ActuatorOutputs, FlightMode
│   ├── simulation/       # Physics loop, thread-safe state, battery
│   └── websocket/        # Axum server, binary protocol, NSH, build config handler
```

## Key Files

- `src/main.rs` — Thread spawning, channel plumbing, sensor config
- `crates/protocol/src/lib.rs` — PX4 motor mapping (PX4_TO_SIM_MOTOR_MAP)
- `crates/simulation/src/loop_runner.rs` — 400 Hz simulation loop, battery discharge
- `crates/simulation/src/state.rs` — Thread-safe state (QuadrotorState + BatteryState)
- `crates/websocket/src/handler.rs` — WebSocket message handling, NSH, recharge command
- `crates/websocket/src/build_config.rs` — ConfigureBuild handler, physics reconfiguration

## Runtime Configuration

The daemon accepts `ConfigureBuild` messages from the web UI via WebSocket:
```json
{
  "type": "configure_build",
  "motor_slug": "xing-2208-1800kv",
  "prop_slug": "gemfan-5030",
  "prop_diameter_inches": 5.0,
  "frame_weight_g": 350.0,
  "battery_voltage": 14.8,
  "battery_capacity_mah": 1000,
  "battery_cell_count": 4
}
```

This triggers:
1. New `PhysicsConfig` via `from_build_specs`
2. New `BatteryConfig`
3. Simulation state reset (position, battery recharged)
4. EKF2 restart via NSH (`ekf2 stop` → `ekf2 start`)
5. Response with applied config parameters and estimated flight time

### Recharge Command
The web UI can send a `Recharge` command to reset battery to 100% without resetting position/attitude. Handled in the websocket handler before forwarding to FC.

## Sensor Configuration

### Sensor Noise (in main.rs)
```rust
let clean_sensors = SensorsConfig {
    imu: ImuConfig {
        gyro_bias_sigma: 0.0,         // No drift for HITL
        ...
    },
    gps: GpsConfig {
        horizontal_noise_sigma: 0.1,  // Tight for HITL
        position_drift_sigma: 0.0,    // No drift
        update_rate_hz: 10.0,
        ...
    },
};
```

### CLI Arguments
| Flag | Default | Description |
|------|---------|-------------|
| --port | auto-detect | Serial port path |
| --baud | 921600 | Baud rate |
| --ws-port | 9876 | WebSocket port |
| --tick-rate | 400 | Simulation Hz |
| --gps-rate | 10 | GPS update Hz |
| --lat/--lon/--alt | Boulder, CO | Reference position |
| --sim-only | false | Run without Pixhawk |

## Motor Mapping

PX4 Standard Quad X — identity mapping (no remapping needed):

```
    Front
  3(CW)   1(CCW)
     \   /
       X
     /   \
  2(CCW)  4(CW)
    Back

PX4_TO_SIM_MOTOR_MAP = [0, 1, 2, 3]
ch0 → Motor 1 (FR, CCW)
ch1 → Motor 2 (BL, CCW)
ch2 → Motor 3 (FL, CW)
ch3 → Motor 4 (BR, CW)
```

## Simulation Loop Details

The 400 Hz loop in `loop_runner.rs`:
1. Check for new `(PhysicsConfig, BatteryConfig)` on config channel → reconfigure
2. Drain actuator commands from PX4 (use latest)
3. Convert throttle [0,1] → motor omega via `throttle_to_omega_with_config`
4. Discharge battery based on total motor current (`total_motor_current`)
5. If battery depleted → zero motor commands (drone falls)
6. RK4 integration step
7. Ground contact constraint (clamp Z, friction)
8. Sample sensors (IMU@400Hz, mag/baro@50Hz, GPS@10Hz)
9. Send HIL_SENSOR and HIL_GPS to PX4

## Building

```bash
cargo build --release
cargo test --workspace
```

## Gotchas

### macOS serial port naming
Use `/dev/tty.usbmodemXX` not `/dev/cu.usbmodemXX`. The `cu.*` device blocks reads until DTR is asserted.

### "High Gyro Bias" errors
Disable gyro bias drift in sensor config: `gyro_bias_sigma: 0.0`

### Position zig-zag in QGC
Reduce GPS noise: `horizontal_noise_sigma: 0.1` (was 1.0m)

### EKF2 startup delay
"ekf2 missing data" at startup is normal — clears in ~2 seconds.

### NSH when daemon running
Use `nsh --ws` mode. Direct serial mode outputs garbage when daemon holds the port.

### EKF2 restart on config change
When `ConfigureBuild` is received, the daemon automatically restarts EKF2 via NSH commands (`ekf2 stop` then `ekf2 start`) to clear stale estimator state. This causes a transient "ekf2 missing data" preflight warning that clears in ~2 seconds.

### Flight mode from HEARTBEAT
The daemon extracts flight mode from `HEARTBEAT.custom_mode` bits 16-23 (PX4 main mode) and updates the simulation state. The frontend displays this in real-time. Note: PX4 may reject mode changes when EKF2 has preflight failures.

### Battery depletion behavior
When battery SoC drops below 5%, motor commands are zeroed regardless of PX4 actuator outputs. The drone will fall. Recharge via WebSocket `Recharge` command or `ConfigureBuild` (which resets the full state).

### Config channel is non-blocking
The simulation loop uses `try_recv` on the config channel. If multiple configs are sent rapidly, only the last one applied matters (earlier ones are processed sequentially on next ticks).

## WebSocket Protocol

- Port: 9876
- Endpoint: `ws://localhost:9876/ws`
- Binary protocol for telemetry (30 Hz) — includes battery voltage and percent
- JSON for commands (NSH, ConfigureBuild, Recharge)

## Dependencies

```toml
[dependencies]
hitl-physics = { path = "../hitl-physics" }
hitl-sensors = { path = "../hitl-sensors" }
```
