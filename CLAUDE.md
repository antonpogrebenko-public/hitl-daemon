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
- Resets the vehicle to the start on request (command byte 8) and reports when PX4 can arm again

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
- `crates/websocket/src/vehicle_reset.rs`: the byte-8 vehicle reset (vehicle back at its start, EKF2 restart, wait for PX4)
- `crates/websocket/src/fc_session.rs`: the FC session lock shared by configure, reset and the re-push; the confirmed EKF2 restart
- `crates/websocket/src/nsh_await.rs`: sends one internal NSH command and awaits its reply
- `crates/websocket/src/px4_ready.rs`: waits until PX4 reports it can arm after an EKF2 restart
- `crates/simulation/src/px4_health.rs`: PX4's pre-arm verdict from `SYS_STATUS` and its latest "Preflight Fail" text

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

It runs off the WebSocket receive loop and reports each stage as a `ConfigResult` with `state: configuring`. In order:
1. Refused, changing nothing, while PX4 reports armed ("Land and disarm first.", not in `--sim-only`) or while a reset, another apply or the reconnect re-push holds the FC session ("Another reset or configuration is in progress.")
2. `fetching_specs`: the component specs from the catalogue
3. `computing`: `PhysicsConfig` via `from_build_specs`, `BatteryConfig` and the PX4 PIDs
4. `pushing_params`: `PARAM_SET` for every PID and thrust-curve parameter, each verified by its `PARAM_VALUE` ack (skipped in `--sim-only`)
5. The flight origin, datum and terrain anchor move to the requested location, only now and only if PX4 is still disarmed
6. The physics go to the simulation loop, and the daemon waits (up to 1 s) until the loop has applied them, which resets the vehicle (position, battery recharged)
7. `restarting_ekf`: the confirmed EKF2 restart (see Gotchas)
8. `waiting_for_px4`: until PX4's pre-arm check passes (up to 30 s)
9. `state: ready` with the applied config, estimated flight time and `px4_health` (`healthy`, or `unknown` when PX4 sends no `SYS_STATUS`). In `--sim-only` steps 7 and 8 are skipped and `px4_health` is `not_applicable`.

Any failure ends in `state: error` and forgets the PID fingerprint, so the retry pushes every parameter again. PX4 has no rollback: a failed push leaves the parameters it already wrote. A failure before step 5 leaves the previous location, physics and vehicle in effect; the armed refusal at step 5 does too, but PX4 already holds (and has saved) the new parameters; a failure from step 6 on (the loop not taking the build within 1 s, a failed EKF2 restart, a pre-arm check that never passes) leaves the new location and parameters in effect, with the build applied or still queued, and the retry applies it again. A fully acknowledged push (step 4) stores `LastVerifiedParams`, even when a later step fails; if `retry_count > 0`, `repush_if_configured()` re-pushes PID params 3s after reconnect.

### Recharge Command
The web UI can send a `Recharge` command to reset battery to 100% without resetting position/attitude. Handled in the websocket handler before forwarding to FC.

### Vehicle Reset Command
Command byte 8 (`ResetVehicle`) returns the vehicle to its start: position, attitude, velocity, motors, a full battery and the stall flag are reset, while the flight location, the applied build and the simulation clock are kept. The PX4-mirrored armed flag, flight mode and landed state read 0 until PX4 next reports them (HEARTBEAT, EXTENDED_SYS_STATE), and the simulated sensors are re-created. EKF2 is then restarted and the daemon waits until PX4 can arm. The command is acknowledged (`CommandAck`) before anything else and runs off the receive loop. Progress and the outcome arrive as `ResetResult` (0x19, JSON): `state: resetting` with `stage` `resetting_vehicle`, `restarting_ekf` or `waiting_for_px4`; `state: ready` with `px4_health`; or `state: error` with `error`.
- Refused while PX4 reports armed: "Land and disarm first."; while a configure, another reset or the reconnect re-push runs: "Another reset or configuration is in progress."
- A failed restart reads "EKF2 restart failed: <reason>"; a pre-arm check that never passes carries PX4's "Preflight Fail" text, or "PX4 did not report ready to arm within 30 s".
- `--sim-only`: no EKF2 restart; `ready` with `px4_health: "not_applicable"`.
- `Rtl` (byte 4) is unchanged.

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
3. **Stale actuator timeout**: if `ACTUATOR_STALE_TIMEOUT` (100ms) elapses with no FC commands while motors are active → zero motor commands and raise `board_stall`; armed stays PX4's (a lost FC connection reports the disarm, via `mark_fc_lost`)
4. Convert throttle [0,1] → motor omega via `throttle_to_omega_with_config`
5. **Battery voltage sag**: motor omegas scaled by `v_terminal / v_nominal` ratio for accurate thrust at low SoC
6. Discharge battery based on total motor current (`total_motor_current`)
7. If battery depleted → zero motor commands (drone falls); **sim-only**: auto-recharge after 3s
8. RK4 integration step
9. **Ground contact**: unified threshold (no dead zone); attitude follows the sampled surface normal; Coulomb friction threshold decides park vs slide
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

### EKF2 restart is confirmed, then PX4 is awaited
ConfigureBuild (once the simulation has applied the build) and the vehicle reset restart EKF2 through `FcSession::restart_ekf2`: `ekf2 stop`, then `ekf2 start`, each reply awaited for up to 3 s. Internal NSH request ids are allocated from `0xFFFF_FF10` (the preflight reboot keeps its fixed `0xFFFF_FF04`). `ekf2 stop` passes on any reply, "not running" included; `ekf2 start` fails when its reply contains "error" or "failed"; no reply, "No FC connected" or a full NSH queue fails the step, and the error reads "EKF2 restart failed: <reason>". A failed `ekf2 start` after a successful stop leaves EKF2 stopped; a retry (configure or reset) stops and starts it again. The NSH task's own deadline for these commands (4 s) is past the 3 s reply window, because on that deadline the task answers with whatever it has (often only the echoed command) marked successful.

The wait for PX4 then follows three timings. A passing `SYS_STATUS` counts only if it arrives at least 1 s after `ekf2 start` was confirmed (one sent earlier still describes the old estimator). No `SYS_STATUS` at all for 3 s means `ready` with `px4_health: unknown`. After 30 s it fails with PX4's latest "Preflight Fail" text since the restart began, or "PX4 did not report ready to arm within 30 s". The transient "ekf2 missing data" after a restart is expected; it is what this wait sits out.

Log lines for the real recovery time: `EKF2 restart confirmed by PX4` (`stop_ms`, `start_ms`), `PX4 pre-arm check changed` (`can_arm`, `since_ekf2_start_ms`), then `PX4 reports ready to arm after the EKF2 restart` (`after_ms`), `PX4 did not report ready to arm` (`reason`) or `PX4 sent no SYS_STATUS after the EKF2 restart`.

### PX4 pre-arm health
`Px4HealthMonitor` (`crates/simulation/src/px4_health.rs`) keeps PX4's own verdict on whether it can arm: the `MAV_SYS_STATUS_PREARM_CHECK` bit of `SYS_STATUS.onboard_control_sensors_health`, which PX4 sets while the current mode's pre-arm checks pass (in the health field only, not in present or enabled), and the latest "Preflight Fail" `STATUSTEXT`, so a failed wait names PX4's reason. Every change of the bit is logged. Risk: on older PX4 firmware the bit may be missing or mean something else, and every reset and apply would then wait the full 30 s and fail; if QGroundControl says ready to arm while `can_arm` never turns true, switch to the estimator health bits.

### FC session lock
`FcSession` holds one lock shared by ConfigureBuild, the vehicle reset and the reconnect PID re-push, so their parameter writes, vehicle resets and EKF2 restarts never interleave. Configure and reset take it with `try_lock` and are refused as busy ("Another reset or configuration is in progress.") while it is held; the re-push waits for it instead, because it must not be lost. Restore and provisioning are outside the lock.

### Sim-only armed flag
In `--sim-only` the fake hover generator sends `FlightMode::HilArmed` and the loop copies it into `armed`, so `armed` reads true for the whole session. There is no PX4 to be armed, so `FcSession::px4_reports_armed` ignores the flag there: a sim-only configure or reset is never refused as armed.

### Flight mode from HEARTBEAT
The daemon extracts flight mode from `HEARTBEAT.custom_mode` bits 16-23 (PX4 main mode) and updates the simulation state. The frontend displays this in real-time. Note: PX4 may reject mode changes when EKF2 has preflight failures.

### Battery depletion behavior
When battery SoC drops below 5%, motor commands are zeroed regardless of PX4 actuator outputs. The drone will fall. Recharge via WebSocket `Recharge` command, the vehicle reset (command byte 8) or `ConfigureBuild` (both reset the full state). In `--sim-only` mode, the battery auto-recharges after 3s of depletion (no permanent crash state).

### Config channel is non-blocking
The simulation loop uses `try_recv` on the config channel. If multiple configs are sent rapidly, only the last one applied matters (earlier ones are processed sequentially on next ticks).

### Bootloader detection
If no HEARTBEAT is received within 5s of connecting, the daemon suspects the FC is stuck in bootloader mode. The `bootloader_suspected` atomic flag is shared between the receiver task and the connection loop. Connection is retried with a 10s backoff. The current status is broadcast to WebSocket clients via `ConnectionStatus { bootloader_suspected: bool }`. After the first HEARTBEAT, 5 s without one means the FC is gone with the port still open: the receiver drops the link (`HeartbeatVerdict::Silent`), so the connection loop reports it and `mark_fc_lost` disarms.

### Stale actuator timeout
`ACTUATOR_STALE_TIMEOUT` (100ms) in `loop_runner.rs`: if the simulation loop has active motors but receives no actuator commands from PX4 for 100ms, motor outputs are zeroed and `board_stall` is raised (state byte `[95]`) until the next command. Zeroing prevents runaway physics when the FC crashes or disconnects mid-flight.

Armed is deliberately **not** cleared: a stall is not a disarm PX4 made, and the browser's run screen read the old fake disarm as a landing and ended flights mid-air. The disarm for a vanished FC comes from the reconnect task's "FC connection lost" branch (`SimulationState::mark_fc_lost`), after the heartbeat timeout or a serial error.

### Auto PID re-push after reconnect
`BuildConfigHandler::repush_if_configured()` triggers 3s after reconnect when `retry_count > 0`. It re-sends the last `LastVerifiedParams` to PX4 so PIDs do not reset to firmware defaults on a USB reconnect. It waits for the FC session lock first, so it never interleaves with a configure or reset, and re-sends the last parameter set PX4 fully acknowledged, which can belong to an apply that failed in a later step.

### Serial link quality monitoring
`MavlinkIo` tracks `parse_successes` and `parse_failures`. If link quality drops below 95% over any 5s window, a warning is logged. Persistent parse failures indicate cable quality issues or baud rate mismatch.

### Sensor channel backpressure
If a sensor channel is full (producer outpacing consumer), the daemon emits a rate-limited warn at 5s and escalates to error at 2s continuous. Indicates the simulation loop is overloaded or a consumer task has stalled.

### Ground accelerometer: level ground must NOT use quaternion rotation
On **level** ground the on-ground accelerometer override must produce `[0, 0, -mg]` directly in body frame. Do NOT rotate the NED gravity vector by the simulated quaternion — any residual tilt (even 0.25°) creates a persistent lateral accel bias that the EKF integrates into massive position drift (76m in 77s observed), and it also prevents landing detection because the altitude estimate diverges.

Since slope support this is a *conditional*, not a blanket rule. `is_level()` gates it on the sampled surface normal. Level ground takes the exact `[0, 0, -g]` path, and `attitude_on_surface()` builds an exact euler attitude with no matrix round-trip, so there is no residual to integrate. On genuinely sloped ground the drone really is tilted and the reading is the gravity reaction rotated into the resting attitude — a physical measurement, not the artifact the flat-ground path guards against. Do not collapse the two branches back into one.

### Ground friction is a Coulomb threshold, not blanket damping
`GROUND_FRICTION_COEFF = 0.6` (about 31°). Below it a parked drone keeps `GROUND_STATIC_DAMPING` (0.9/tick) and stays put; above it, gravity along the surface wins and it slides with only `GROUND_SLIDING_DAMPING` (0.999/tick). A flat 0.9 everywhere is 1e-19 per second at 400 Hz — infinitely sticky ground that makes every slope behave like a level plane.

### Terrain coverage gaps mean "unknown", never "flat"
`ground_z` is `Option<f64>`. `None` means terrain is configured but the drone left the cached 3x3 tile block. Collapsing it to 0.0 clamps a drone flying below the origin datum straight up to it, velocity zeroed and attitude levelled — a discontinuity the EKF cannot reconcile. Unknown ground disables the clamp and logs a rate-limited warning. Flat Z=0 applies only when no terrain is configured at all.

### Landed state comes from PX4, never from the simulation
The daemon consumes `EXTENDED_SYS_STATE` and forwards `MAV_LANDED_STATE` in the state frame at byte `[86]`. The simulation deliberately does not infer it: the value is PX4's own land-detector verdict, so a disagreement with the simulated ground contact is the signal that synthesized sensors have misled the EKF. Computing it locally would destroy the only cross-check.

### Altitude datum comes from the DEM, not --alt
When terrain loads, `reference_alt` is replaced by `TerrainCache::origin_elevation_msl()`. Ground collision, the barometer, and HIL_GPS MSL then share one datum. Keeping the CLI `--alt` leaves baro and GPS self-consistent but both offset from true MSL (~1640 m with the default Boulder reference), so the vehicle plots below real terrain in QGC.

### ConfigureBuild zeroes accel/gyro calibration offsets
`push_pids_and_verify()` sends `CAL_ACC0_XOFF/YOFF/ZOFF=0` and `CAL_GYRO0_XOFF/YOFF/ZOFF=0` alongside PIDs. The real FC has calibration offsets from hardware mounting (e.g., +0.05 m/s² X-axis). PX4 subtracts these from raw readings. Since the simulated IMU has no physical bias, these offsets create a persistent ~0.05 m/s² lateral acceleration that the EKF integrates into 1.6m/8s of drift. Zeroing them eliminates the bias.

### Ground impact deceleration prevents EKF underground divergence
When the drone hits the ground at speed (>0.5 m/s), the simulation generates a 100ms deceleration impulse on the accelerometer (`ground_impact_accel`). Without this, the ground clamp instantly zeros velocity but reports only gravity — the EKF sees no force to explain the velocity change, rejects GPS, and dead-reckons underground (37m observed in a 6.4 m/s impact). The impulse gives the EKF a physical event it can reconcile with the sudden velocity change.

The impulse is derived from **NED** velocity and the accelerometer reads in the **body** frame, so the full `(a - g)` vector is rotated by the impact attitude before it is reported. Skipping that rotation puts the lateral component on the wrong body axis for any non-zero yaw — a 90° yaw sends a northward impact onto body Y — and the EKF integrates a phantom sideways acceleration.

## WebSocket Protocol

- Port: 9876
- Endpoint: `ws://localhost:9876/ws`
- Binary protocol for telemetry (30 Hz): includes battery open-circuit voltage and percent, plus loaded (terminal) voltage and total motor current at bytes 87-94 (always written, NaN when unknown) and a board-stall flag at byte 95 (96-byte frame; readers accept >= 86)
- JSON for commands (NSH, ConfigureBuild, Recharge)
- Command byte 8 (`ResetVehicle`): acknowledged at once, then `ResetResult` (0x19, JSON) progress and outcome (see Vehicle Reset Command)
- `ConfigResult` stages end with `waiting_for_px4`, and `ready` carries `px4_health`
- **Ping/pong**: 5s ping interval, 15s pong timeout — zombie connections are detected and dropped
- `ConnectionStatus` message includes `bootloader_suspected: bool` field

## Dependencies

```toml
[dependencies]
hitl-physics = { path = "../hitl-physics" }
hitl-sensors = { path = "../hitl-sensors" }
```

## Approach
- Read existing files before writing. Don't re-read unless changed.
- Thorough in reasoning, concise in output.
- No sycophantic openers or closing fluff.
- No emojis or em-dashes.
- Do not guess APIs, versions, flags, commit SHAs, or package names. Verify by reading code or docs before asserting.
