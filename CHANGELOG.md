# Changelog

All notable changes to the HITL daemon will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
