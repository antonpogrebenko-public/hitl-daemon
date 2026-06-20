# Changelog

All notable changes to the HITL daemon will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.9.1] - 2026-06-21

### Fixed
- **kt KV-floor clamp** — large low-KV builds (8"+, 500 KV) no longer inflate kt quadratically via unclamped `(2300/kv)²`. Effective KV floored at 1500 (matching TS physics-model `KT_KV_FLOOR`). Eliminates phantom ~20:1 TWR on big-prop configs.
- **Two-sided PID authority scaling** — `compute_pids()` now attenuates for both braking (down) and boost (up) headroom. Overloaded builds (high hover_cmd) previously got full P/D despite limited upward authority, causing oscillation.
- **Actuator-bandwidth PID derating** — P/D scaled by `REF_TAU_MOTOR / tau_motor` so slow large-prop / low-KV actuators don't outrun their motor pole. Prevents phase-lag limit cycles on 8"+ builds.

## [0.9.0] - 2026-05-30

### Changed
- **Breaking: WebSocket handshake protocol now includes version_patch byte.** HandshakeAck binary format is now `[0x02, major, minor, patch, fc_connected, ...serial_port, 0x00]`. The web UI can now display and enforce full semver (e.g., `0.9.0` instead of `0.8`). Older web clients will misinterpret the patch byte as `fc_connected` — update the web frontend alongside this daemon release.

## [0.8.5] - 2026-05-30

### Fixed
- **MPC_THR_HOVER now matches actual sim hover** (sess113 bug: position mode couldn't take off after landing). The old code computed hover from `(1/TWR) / 0.7225` — a sag correction calibrated for the legacy inflated-thrust model. With the recalibrated physics (TWR~2 instead of ~8), this pushed 62.5% while actual sim hover was 44%. PX4's position controller couldn't generate enough thrust to lift off. Now uses `physics.hover_throttle_percent()` directly — guaranteed to match the sim.
- **Battery weight estimated from capacity** when no `battery_slug` is provided in `ConfigureBuild`. Previously defaulted to 180g regardless of actual capacity — a 4S 4500mAh pack weighing ~630g was modeled as 180g, making the sim 40% too light and hover feedforward 21pp too high. Estimate: `capacity_mah × cell_count × 0.035` (overridden by exact API weight when `battery_slug` is present).

### Changed
- Rebuild against `hitl-physics` 0.10.0 (torque-balance loaded-RPM model, physical-CT recalibration).

## [0.8.4] - 2026-05-29

### Fixed
- **Armed-on-ground motor RPM "jump" fixed at its source; enables the ESC-idle limit-cycle fix.** The 400 Hz physics step was gated on `motors_active = any(cmd > 0.01)`, so while armed at idle the step toggled on/off as PX4's micro-corrections crossed 0.01 — snapping motor speeds between 0 and the idle floor. The step now runs continuously whenever the vehicle is **armed**, holding the idle steady. This lets `hitl-physics` 0.9.5 reintroduce a realistic armed ESC idle (removing the braking dead zone behind the ~14 Hz rate-loop limit cycle on high-TWR builds) without the cosmetic jump returning. Motor omegas are forced to zero when disarmed, so a disarmed or killed motor still produces no thrust.

## [0.7.7] - 2026-05-28

### Fixed
- **`MPC_THR_HOVER` ignored battery sag, causing altitude hunting** (log1.ulg, 2026-05-28: configured TWR=8.92 gave `MPC_THR_HOVER=0.112`, but a 25 s steady-state hover at 28-30 m showed observed hover thrust = 0.150 — effective TWR = 6.68, voltage ratio = 0.866). PX4's altitude controller ran a 34% feedforward deficit; the integrator made it up but left visible 18 cm altitude hunting at 0.12 Hz and 12 cm at 0.32 Hz. Also blocked the land detector from tripping in position mode near ground, because the controller never settled. The 0.7.6 `MPC_THR_MIN` fix was correct and necessary but the feedforward was still wrong.

### Changed
- `BuildConfigHandler::handle` now derates `hover_cmd` by `(0.85)² = 0.7225` to reflect typical mid-flight battery sag (loaded V ≈ 0.85 × unloaded; thrust ∝ V²). For TWR=8.92 → `hover_cmd` goes from 0.112 to 0.155 (matches observed 0.150 within 3%). `MPC_THR_MIN` is unchanged (still clamped to 0.05 floor for the same TWR). Param count stays at 15 — no new params pushed, since this PR deliberately keeps `MPC_Z_VEL_MAX_DN/UP` at PX4 defaults so position-mode descent behavior matches real-life PX4 (rate-limited to 1.5 m/s on full down stick — that's PX4 by design, not a HITL artifact).

## [0.7.6] - 2026-05-17

### Fixed
- **Position-mode 0.8 Hz limit cycle on high-TWR builds** (log100.ulg follow-up: pitch_act swings ±300°/s while pitch_sp stays ±30°/s, motors cycle 0.00→0.71). PX4's default `MPC_THR_MIN=0.12` is sized for typical TWR≈2 builds (where 0.12 ≪ 0.5 hover). For a TWR=8.92 racer, hover ≈ 0.112 ≤ MPC_THR_MIN, so the floor pins thrust at or above weight — the drone physically cannot descend, position control devolves into an altitude limit cycle that drives violent attitude oscillation. The earlier 0.7.3-0.7.5 fixes (params, attitude, auto-level) were all necessary but not sufficient; this is the last layer of the stack.

### Added
- `BuildConfigHandler::push_pids_and_verify` now pushes 15 params (was 14): adds `MPC_THR_MIN = (hover_cmd × 0.3).clamp(0.05, 0.20)` — 30% of hover gives ≥0.5 g of descent authority, clamped to PX4's accepted range. For TWR=8.92 → `MPC_THR_MIN = 0.05`. For TWR=2 → `0.15`, close to PX4 default so low-TWR builds aren't affected.

### Changed
- Fingerprint cache now mixes in `thr_min` (bits 16-47) alongside `hover_cmd` (bits 32-63) — a TWR change forces a re-push of all thrust-curve params even when rate PIDs happen to be unchanged.

## [0.7.5] - 2026-05-17

### Fixed
- **Pre-takeoff trembling caused by inverted sim quaternion** (log100.ulg: `accel_z=+9.80`, `attitude_roll≈+178.55°`, `rate_sp_roll=-220°/s` on the ground). The 0.7.1-0.7.4 controller/thrust-curve fixes were all treating the symptom — the rate loop was correctly fighting a real 178° attitude error because a previous crash/flip had left the simulator's quaternion non-trivial, and ground friction only damps angular *velocity*, never restores *orientation*. Auto-level fix lives in the sim loop (see below).
- **Params didn't survive PX4 reboots.** `PARAM_SET` only writes to RAM. A FC power-cycle silently dropped all 14 per-build params back to PX4 defaults, with no log indication.

### Added
- Sim loop auto-levels the quaternion when on-ground + disarmed via a 0.02-per-tick slerp toward `(0, 0, current_yaw)` (~190 ms time constant at 400 Hz). Invisible during normal touchdown dynamics, fast enough to clear a stuck-inverted state between flights.
- After the 14-param push verifies, the daemon sends `MAV_CMD_PREFLIGHT_STORAGE` (cmd 245, param1=1) to commit the in-RAM param table to PX4 flash. Fire-and-forget — PX4's storage ack is best-effort and a subsequent ConfigureBuild re-pushes everything if it didn't take.
- `SimulationStats.attitude_rpy_deg` exposes sim roll/pitch/yaw in degrees.
- TUI surfaces an `Att` row showing roll/pitch/yaw. When `|roll|>5°` or `|pitch|>5°` while disarmed, the row turns red and shows `⚠ inverted on ground — reconfigure` so this exact failure mode is visible at a glance instead of buried in a ULOG analysis.

## [0.7.4] - 2026-05-17

### Fixed
- Pre-takeoff motor trembling — the 0.7.3 fix paired with `hitl-physics` 0.9.0's ω²-space throttle interpolation amplified tiny rate-controller PID outputs (cmd ≈ 0.005-0.02) into massive motor RPM swings (2300-4300 RPM at idle vs the expected ~1000-1500). PX4's rate PIDs are tuned assuming linear cmd→ω (matching real ESCs); the ω² model gave a ~16× steeper `dω/dcmd` slope at idle, so the rate loop oscillated whenever the integrator nudged the motors. Confirmed via NSH `param show` that the 0.7.3 params landed; the trembling was downstream of the motor model itself, not the param push.

### Changed
- `THR_MDL_FAC` push flipped from `0.0` → `1.0`. With `hitl-physics` 0.9.1 reverted to linear cmd→ω, PX4 outputs `cmd = sqrt(thr_desired)` to compensate for the resulting quadratic cmd→thrust curve. End-to-end round-trip is still linear in `thr_desired`, but the actuator-side response is now stable at idle and matches how real drones behave.
- `MPC_THR_HOVER` semantics unchanged (still `1/TWR` clamped to [0.1, 0.8]) — PX4 stores it in pre-THR_MDL_FAC-inversion units, so it doesn't depend on which side of the contract owns the sqrt.

## [0.7.3] - 2026-05-17

### Fixed
- Position-mode trembling and slow descent on light racers (TWR > 2). PX4's default `MPC_THR_HOVER=0.5` only matches a TWR=2 build; with the new linear cmd→thrust motor model (hitl-physics 0.9.0), a TWR=5 racer needs `MPC_THR_HOVER=0.2`. The default left the altitude integrator fighting a 2.5× thrust overshoot on every position-hold cycle, which the position controller turned into visible "trembling".

### Added
- `BuildConfigHandler::push_pids_and_verify` now pushes 14 params instead of 12: the 12 rate PIDs plus `THR_MDL_FAC=0` (locks PX4's forward thrust model to linear, matching the sim's ω²-space throttle interpolation) and `MPC_THR_HOVER=1/TWR` (clamped to PX4's [0.1, 0.8] range).
- `AppliedConfig.hover_cmd` surfaces the actual pushed hover throttle so the UI can show it.

### Changed
- PID fingerprint cache now keys on `pid_fingerprint XOR (hover_cmd_bits << 32)`, so a TWR change re-pushes even when the rate PIDs themselves are unchanged.

## [0.7.2] - 2026-05-16

### Changed
- TUI header expanded from 2 lines to 7: now surfaces tick rate (color-coded by health), avg/max latency, sensor drops, armed/mode/sim-time/position, motor RPMs, HIL message counts, battery V/% (color-coded), build mass + TWR, uptime.
- Periodic `Simulation stats` `info!` log (fired every 5 s) demoted to `debug!`. Same data now lives in the TUI header and is updated at 2 Hz via a `tokio::sync::watch` channel.

### Added
- `protocol::SimulationStats` carries the live snapshot (loop perf, cumulative counts, drone state, applied build summary). `serde`-friendly so future web/HTTP status endpoints can consume the same shape.
- `SimulationLoop::with_stats_publisher(tx)` builder so the loop publishes a snapshot every 500 ms to anything that subscribes (TUI today, web status panel later).

## [0.7.1] - 2026-05-16

### Fixed
- Drone trembling / rate-controller oscillation on light builds: the Phase 6 PARAM_SET push to PX4 was commented out, so PX4 ran stock PIDs tuned for I_ref ≈ 0.005 against actual inertia of ~0.0037 — a ~34% over-gain that no manual tuning could stabilize.

### Added
- Two-stage `ConfigResult` lifecycle (`configuring` → `ready` | `error`). The simulation loop is no longer reconfigured until PX4 confirms every PID parameter.
- Per-build PID PARAM_SET push with `PARAM_VALUE` ack verification. Per-parameter 800 ms timeout, 3 retries, value-match within 1e-4 epsilon.
- `AppliedConfig.verified_params` and `AppliedConfig.applied_pids` surface what was actually written to PX4.
- Frontend banner on `/simulator/run` shows "Verifying PX4 PIDs…", then a green "Continue to simulator" CTA on ready, or red error + retry on ack failure.

### Changed
- `BuildConfigHandler::push_pids_and_verify` replaces the fire-and-forget `push_pids_if_changed`. Fingerprint cache only updates on full verification — partial pushes retry the whole sequence on the next `ConfigureBuild`.
- MAVLink receiver task taps `PARAM_VALUE` and broadcasts on a 256-deep tokio channel for the handler to subscribe to.

## [0.6.3] - 2026-05-15

### Fixed
- Motor RPM oscillation / drone trembling on lightweight builds (<500g): added Ixx/Iyy inertia floor of 0.012 kg·m² so PX4's rate PIDs don't overshoot
- Izz/Ixx ratio now always >= 1.7 to prevent unphysical gyroscopic coupling
- Battery depletion no longer allows infinite hover: motor commands zeroed when SoC < 5%
- Max speed unrealistically low (~4 m/s): drag coefficients now derived from frontal area (0.5×ρ×Cd×A) instead of hardcoded 0.25

### Changed
- Drag model uses physically-derived coefficients based on prop diameter (~0.016 for 5" lateral, ~0.022 vertical)
- `hitl-physics` bumped to 0.5.0 (breaking: drag and inertia behavior changes for all `from_build_specs` configs)

## [0.6.2] - 2026-05-15

### Added
- Battery simulation: LiPo discharge model consumes battery during flight based on motor current draw
- Estimated flight time reported in ConfigureBuild response
- Recharge command (type 7) resets battery to 100% without reconfiguring
- Battery recharges automatically on reconfiguration
- `battery_capacity_mah` and `battery_cell_count` fields in ConfigureBuild payload

### Changed
- State update packet reports live battery voltage/percent from simulation (no longer hardcoded)
- Zero-throttle mid-flight now applies gravity (freefall) instead of slow descent

## [0.6.1] - 2026-05-15

### Fixed
- Yaw oscillation after build config: inertia estimation in `from_build_specs` produced Izz ~3× too low for PX4 default PIDs, causing yaw hunting
- Simulation loop used legacy `throttle_to_omega` (fixed 2500 rad/s max) instead of voltage-aware `throttle_to_omega_with_config` after reconfiguration

### Changed
- Inertia model uses point-mass motor contribution with Izz floor of 0.020 kg·m²

## [0.6.0] - 2026-05-14

### Added
- Battery voltage parameter in ConfigureBuild command
- Battery voltage affects max motor RPM (KV × voltage × π/30)
- Propeller selection support (slug or diameter)
- Electrical parameters in AppliedConfig response (motor_kv, battery_voltage, max_motor_rpm)

### Changed
- Physics config now uses voltage-limited max motor speed instead of fixed constant

## [0.5.1] - 2026-05-13

### Added
- EKF2 auto-restart on config change to clear stale estimator state
- Flight mode telemetry from HEARTBEAT custom_mode bits

### Fixed
- Serial write timeouts to prevent stalls on port issues
- Parse buffer size limit with frame scanning to prevent OOM on corrupt streams
- Read timeouts so shutdown flag is checked periodically
- TUI always restores terminal on panic/error
- TUI auto-scroll logs to show latest output

## [0.5.0] - 2026-05-12

### Added
- Component-driven simulation: select motor, prop diameter, and frame weight to configure physics
- Runtime physics reconfiguration via WebSocket ConfigureBuild command (0x13)
- Daemon fetches motor specs from th3seus API and derives kt/kq/mass/inertia
- ConfigResult response (0x08) with applied config and thrust-to-weight ratio

## [0.4.0] - 2026-05-12

### Added
- Heartbeat watchdog: disconnects FC if no heartbeat received within 5 seconds (detects bootloader mode)
- Serial read timeout (1s): reader task no longer blocks forever on silent ports
- Serial write timeout (2s): writer task detects stalled USB/hub without hanging
- Parse buffer corruption recovery: scans forward to next MAVLink frame start (0xFD) on parse failure
- Parse buffer size cap (8KB): prevents OOM from sustained corrupt serial data
- Connection manager cooldown between disconnect and reconnect
- Retry count increments on all disconnect paths (including watchdog-triggered)
- TUI log auto-scroll: always shows most recent log lines
- TUI status panel shows real serial port path and reconnection state
- TUI panic hook restores terminal raw mode on crash
- Sensor message drop tracking with periodic warnings
- NSH queue backpressure: immediate rejection when queue is full ("NSH busy")
- WebSocket max incoming message size (1KB) to prevent memory abuse

### Changed
- Writer task poll interval increased from 100µs to 1ms (reduced CPU usage)
- Receiver task poll interval increased from 500µs to 2ms (reduced CPU usage)
- NSH command channel reduced from 32 to 4 slots (prevents pile-up)
- Connection manager awaits aborted task handles before reopening port (prevents FD race)

### Fixed
- Terminal stuck in raw mode after Ctrl+C (TUI restore moved to wrapper with error/panic coverage)
- Logs not visible after initial burst (Paragraph widget now scrolls to bottom)
- Status panel showing stale "Streaming" state after FC disconnect
- Serial port "none" in status panel (now reads from connection status broadcast)
- FC model not cleared on disconnect (stale model no longer shown during reconnect)

## [0.2.4] - 2026-05-07

### Added
- Initial release through the automated release system

### Fixed
- Motor mapping correction for X-frame configurations
- Serial port reconnection on macOS sleep/wake
