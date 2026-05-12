# Changelog

All notable changes to the HITL daemon will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
