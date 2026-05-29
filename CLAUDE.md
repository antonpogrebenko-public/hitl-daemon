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
4. EKF2 restart via NSH (`ekf2 stop` → `ekf2 start`) — up to 3 attempts with 200ms backoff
5. Response with applied config parameters and estimated flight time
6. Stores `LastVerifiedParams`; if `retry_count > 0`, `repush_if_configured()` re-pushes PID params 3s after reconnect

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
3. **Stale actuator timeout**: if `ACTUATOR_STALE_TIMEOUT` (100ms) elapses with no FC commands while motors are active → zero motor commands and disarm
4. Convert throttle [0,1] → motor omega via `throttle_to_omega_with_config`
5. **Battery voltage sag**: motor omegas scaled by `v_terminal / v_nominal` ratio for accurate thrust at low SoC
6. Discharge battery based on total motor current (`total_motor_current`)
7. If battery depleted → zero motor commands (drone falls); **sim-only**: auto-recharge after 3s
8. RK4 integration step
9. **Ground contact**: unified threshold (no dead zone), thrust-proportional friction (no sticky ground)
10. Sample sensors (IMU@400Hz, mag/baro@50Hz, GPS@10Hz)
11. Send HIL_SENSOR and HIL_GPS to PX4

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
When `ConfigureBuild` is received, the daemon automatically restarts EKF2 via NSH commands (`ekf2 stop` then `ekf2 start`) to clear stale estimator state. Up to 3 retry attempts with 200ms backoff. This causes a transient "ekf2 missing data" preflight warning that clears in ~2 seconds.

### Flight mode from HEARTBEAT
The daemon extracts flight mode from `HEARTBEAT.custom_mode` bits 16-23 (PX4 main mode) and updates the simulation state. The frontend displays this in real-time. Note: PX4 may reject mode changes when EKF2 has preflight failures.

### Battery depletion behavior
When battery SoC drops below 5%, motor commands are zeroed regardless of PX4 actuator outputs. The drone will fall. Recharge via WebSocket `Recharge` command or `ConfigureBuild` (which resets the full state). In `--sim-only` mode, the battery auto-recharges after 3s of depletion (no permanent crash state).

### Config channel is non-blocking
The simulation loop uses `try_recv` on the config channel. If multiple configs are sent rapidly, only the last one applied matters (earlier ones are processed sequentially on next ticks).

### Bootloader detection
If no HEARTBEAT is received within 5s of connecting, the daemon suspects the FC is stuck in bootloader mode. The `bootloader_suspected` atomic flag is shared between the receiver task and the connection loop. Connection is retried with a 10s backoff. The current status is broadcast to WebSocket clients via `ConnectionStatus { bootloader_suspected: bool }`.

### Stale actuator timeout
`ACTUATOR_STALE_TIMEOUT` (100ms) in `loop_runner.rs`: if the simulation loop has active motors but receives no actuator commands from PX4 for 100ms, motor outputs are zeroed and the sim is disarmed. Prevents runaway physics when the FC crashes or disconnects mid-flight.

### Auto PID re-push after reconnect
`BuildConfigHandler::repush_if_configured()` triggers 3s after reconnect when `retry_count > 0`. It re-sends the last `LastVerifiedParams` to PX4 so PIDs do not reset to firmware defaults on a USB reconnect.

### Serial link quality monitoring
`MavlinkIo` tracks `parse_successes` and `parse_failures`. If link quality drops below 95% over any 5s window, a warning is logged. Persistent parse failures indicate cable quality issues or baud rate mismatch.

### Sensor channel backpressure
If a sensor channel is full (producer outpacing consumer), the daemon emits a rate-limited warn at 5s and escalates to error at 2s continuous. Indicates the simulation loop is overloaded or a consumer task has stalled.

### Ground accelerometer must NOT use quaternion rotation
In `loop_runner.rs`, the on-ground accelerometer override must produce `[0, 0, -mg]` directly in body frame. Do NOT rotate the NED gravity vector by the quaternion — any residual quaternion tilt (even 0.25°) creates a persistent accel lateral bias that the EKF integrates into massive position drift (76m in 77s observed). This also prevents landing detection because the EKF altitude estimate diverges.

### ConfigureBuild zeroes accel/gyro calibration offsets
`push_pids_and_verify()` sends `CAL_ACC0_XOFF/YOFF/ZOFF=0` and `CAL_GYRO0_XOFF/YOFF/ZOFF=0` alongside PIDs. The real FC has calibration offsets from hardware mounting (e.g., +0.05 m/s² X-axis). PX4 subtracts these from raw readings. Since the simulated IMU has no physical bias, these offsets create a persistent ~0.05 m/s² lateral acceleration that the EKF integrates into 1.6m/8s of drift. Zeroing them eliminates the bias.

### Ground impact deceleration prevents EKF underground divergence
When the drone hits the ground at speed (>0.5 m/s), the simulation generates a 100ms deceleration impulse on the accelerometer (`ground_impact_accel`). Without this, the ground clamp instantly zeros velocity but reports only gravity — the EKF sees no force to explain the velocity change, rejects GPS, and dead-reckons underground (37m observed in a 6.4 m/s impact). The impulse gives the EKF a physical event it can reconcile with the sudden velocity change.

## WebSocket Protocol

- Port: 9876
- Endpoint: `ws://localhost:9876/ws`
- Binary protocol for telemetry (30 Hz) — includes battery voltage and percent
- JSON for commands (NSH, ConfigureBuild, Recharge)
- **Ping/pong**: 5s ping interval, 15s pong timeout — zombie connections are detected and dropped
- `ConnectionStatus` message includes `bootloader_suspected: bool` field

## Dependencies

```toml
[dependencies]
hitl-physics = { path = "../hitl-physics" }
hitl-sensors = { path = "../hitl-sensors" }
```
