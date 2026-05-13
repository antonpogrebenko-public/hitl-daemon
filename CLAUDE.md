# hitl-daemon

Rust Hardware-in-the-Loop daemon for PX4 flight controller simulation.

## Overview

Connects to a Pixhawk via USB serial, runs 400 Hz physics simulation, and:
- Receives actuator commands from PX4 (HIL_ACTUATOR_CONTROLS)
- Sends simulated sensor data (HIL_SENSOR, HIL_GPS)
- Bridges to QGroundControl via UDP
- Serves WebSocket for browser UI and NSH access

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
│   ├── simulation/       # Physics loop, thread-safe state
│   └── websocket/        # Axum server, binary protocol, NSH handler
```

## Key Files

- `src/main.rs` — Thread spawning, channel plumbing, sensor config
- `crates/protocol/src/lib.rs` — PX4 motor mapping (PX4_TO_SIM_MOTOR_MAP)
- `crates/simulation/src/loop_runner.rs` — 400 Hz simulation loop
- `crates/websocket/src/handler.rs` — WebSocket message handling, NSH

## Configuration

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

PX4 Quad X motor order differs from simulation:

```
PX4:                    Simulation:
    Front                   Front
  3(CW)   1(CCW)         1(CW)   2(CCW)
     \   /                  \   /
       X           →          X
     /   \                  /   \
  2(CCW)  4(CW)          4(CCW) 3(CW)
    Back                    Back

Mapping: PX4_TO_SIM_MOTOR_MAP = [2, 0, 3, 1]
```

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

## WebSocket Protocol

- Port: 9876
- Endpoint: `ws://localhost:9876/ws`
- Binary protocol for telemetry (30 Hz)
- JSON for NSH commands

## Dependencies

```toml
[dependencies]
hitl-physics = { git = "https://github.com/antonpogrebenko-public/hitl-physics" }
hitl-sensors = { git = "https://github.com/antonpogrebenko-public/hitl-sensors" }
```
