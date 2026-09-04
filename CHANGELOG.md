# Changelog

All notable changes to the HITL daemon will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.23.3]

### Changed

- **The terrain tile decoder accepts a payload padded to a 4-byte boundary, as
  well as one packed immediately after the header.** This is the first half of a
  two-sided change and does nothing on its own; the browser still sends the
  unpadded form.

  It exists so the browser can eventually use `Float32Array.prototype.set` --
  one memcpy per tile -- where it currently writes `DataView.setFloat32` per
  sample, 589,824 calls for the nine tiles of a single push. That fast path is
  only available when the payload begins at a multiple of four, which the header
  does not naturally guarantee.

  It lands here first because a daemon is installed on someone's machine and
  updates on their schedule. A browser that started padding unconditionally
  would take terrain away from every daemon older than the change, so the reader
  has to tolerate both forms before the writer picks one.

  The forms are told apart by length rather than a flag: padding is 0-3 bytes,
  so at most one placement leaves exactly the expected payload behind, and when
  the header already ends on a boundary the two are the same offset. A wrong
  size is still rejected -- eight trailing bytes is not padding. The padding
  bytes are not inspected, since they carry nothing.

  Four tests, including that the unpadded form still decodes and that disabling
  the padded branch fails the padded one.

## [0.23.2]

### Changed

- **The serial reader memmoves a read chunk once instead of once per message.**
  Each parsed message did `parse_buffer.drain(..consumed)`, which moves every
  remaining byte down by the size of the message just taken -- so a chunk
  carrying k messages moved its tail k times. The reader now parses at an
  advancing offset and drains once per chunk.

  Four tests cover the loop, which previously had none: every message in a chunk
  parses, a trailing partial frame survives for the next read with its start byte
  intact, and the buffer left behind is byte-identical to what per-message
  draining left. A deliberate off-by-one in the cursor fails three of them.

  **The other half of the original finding was not taken.** Reusing one
  `PeekReader` across the messages in a chunk would depend on `read_v2_msg`
  always leaving the reader's internal cursor level with its read-ahead top --
  an invariant the crate documents nowhere, and getting it wrong desynchronises
  the stream rather than failing loudly. The re-zeroed 280-byte buffer that
  motivated it is allocated inside the mavlink crate's own `fetch`, not here, so
  reusing the reader would not have removed it either.

## [0.23.1]

### Fixed

- **The terrain ingress now performs the staleness check its own protocol
  documented.** `TerrainTilesHeader.origin` has always said "a frame anchored to
  a stale origin is dropped rather than mixed with the current one", and nothing
  read the field -- not `TerrainTiles::from_frame`, not the handler.

  The race is real rather than theoretical. `TerrainCache::set_origin` discards
  the resident tiles when the origin moves, precisely because tiles describe
  ground relative to an origin. A frame already in flight when that happened
  carries the origin the browser still believed was current, and accepting it
  refilled the cache with the tiles the re-anchor had just thrown away -- after
  which ground contact was computed against terrain anchored somewhere else,
  which is the divergence between drawn and simulated ground this path exists to
  prevent.

  The check lives in `TerrainCache::is_anchored_to`, next to the discard it has
  to agree with, and both now read one `SAME_ORIGIN_METERS`. Two thresholds that
  disagreed would leave a band in which the cache clears its tiles and then
  accepts replacements anchored to the origin it just left; a test asserts the
  two behaviours agree rather than asserting the constant.

  A stale frame is rejected with a message rather than dropped quietly. The
  browser owns the fetch and will re-send for the new origin, but a client that
  never learned its frames were being discarded would loop supplying terrain
  that is thrown away every time.

  An unanchored cache rejects everything: accepting tiles before a datum exists
  would have them sampled against an origin chosen afterwards.

## [0.23.0]

### Changed

- **Sensor sampling costs half what it did.** Picks up `hitl-sensors` 0.3.0.
  Measured against a same-session control arm: one second of simulated sampling
  at 400 Hz went from 223.79 us to 109.29 us, and a single IMU sample from
  401.45 ns to 206.42 ns. Box-Muller was generating two normal variates per call
  and discarding one, and `GaussMarkov` recomputed `alpha` and `noise_sigma`
  from a `dt` that does not change between ticks.

  The draw *order* changes as a result, so a seeded run no longer reproduces the
  previous sequence value-for-value. The distributions are unchanged.

- **Thin LTO on the release profile.** Disassembling the shipped binary showed
  `GaussMarkov::step` called out of line six times per IMU sample, and
  `max_motor_speed_from_voltage` likewise — the default `lto = false` with 16
  codegen units gave the optimiser nothing to inline across the crate boundary.

- **The simulation loop sleeps to a deadline instead of for a duration.**
  `sleep_until(next_tick)` rather than `sleep(remaining)`, so scheduling jitter
  on one tick no longer pushes the next one late.

  Spin accuracy was left at the default deliberately. The review claimed
  `spin_sleep` burned about 5 % of a core; measured, daemon total CPU is 1.4 %,
  a same-session A/B of 125 us against 60 us was identical, and the proposed
  tuning made `max_latency` worse — 95 us to 160 us. Reverted; only the deadline
  change was kept.

- **Phase timings are accumulated in nanoseconds.** They were in microseconds,
  which truncated the physics phase to 0 — a single RK4 step is 264 ns.

- **The outbound MAVLink writer awaits a channel** instead of polling with a
  1 ms sleep, and reuses one write buffer. A fourth blocking-send site was
  fixed at the same time. The inbound half is unchanged and still polls; moving
  it requires receiver ownership out of the shared `MavlinkIo` handle, which was
  left rather than attempted blind.

- **The terrain surface normal is only sampled when the vehicle is in ground
  contact.** It was sampled every tick and used on a small fraction of them.

### Fixed

- **`read_available` in the NSH client could not terminate.** It looped while
  `in_waiting > 0` with a 10 ms sleep inside the loop; at 57600 baud roughly 58
  bytes arrive during each sleep, so against a streaming port the condition
  never became false and the buffer grew at ~5.8 KB/s until the process was
  killed. It is the first thing every invocation does through `wake_console`,
  and `nsh/CLAUDE.md` documents the streaming-port state it hits. Now bounded by
  a deadline and a byte cap.

## [0.22.1]

### Fixed

- **The battery's C-rating was never read from the component record.** It stayed
  at the `BuildSpec` default of 75 while the browser read the catalogue's real
  figure, so the two disagreed on the current ceiling — `min(esc.burst_amps,
  C x Ah / 4)` — and therefore on thrust. Caught by running the real product
  rather than the model: for a 6S 1350 mAh 120C pack the configurator showed
  TWR 9.8 and the daemon simulated 6.11, a ratio of exactly 120/75. They now
  agree at 9.84.

  Same class as the `battery_slug` gap in 0.21.0: a specification the API
  serves and nothing read. Current limiting, added in 0.21.x, is what made it
  load-bearing.

## [0.22.0]

### Changed

- Picks up `hitl-physics` 0.13.0. Every simulated build's thrust changes again,
  from two corrections on top of 0.11.0's re-anchor:
  - **Blade count** is now `(blades/2)^0.7` rather than `0.85 + 0.05*blades`,
    fitted to UIUC's blade-count control pairs. A three-blade is 1.328x a
    two-blade, not 1.053x. `K_CT` is re-normalised so the six-inch three-blade
    lab anchor is unmoved, so **three-blade builds are unaffected** and
    two-blade builds read about 21% lower.
  - **All four reference builds** were corrected against manufacturer sources;
    two described a different aircraft than their name claimed.

  The reported regression build is a 6x4x3 — three blades — so its figures are
  unchanged from 0.21.x: mass 2.2033 kg, TWR 1.2400.

## [0.21.2]

### Changed

- Motor winding resistance and no-load current are read from the component
  record (`resistanceOhm`, `noLoadCurrentA`/`idleCurrentA`) when it carries
  them. Absent, the KV-only heuristic still applies and a DEBUG line names the
  fallback. The heuristic reads 0.070 ohm for a motor whose published figure is
  0.116, and under-reading resistance lets the torque balance settle above the
  speed the motor can reach — so it inflates thrust. The catalogue does not
  carry these fields yet; this makes the model improve the moment it does.

## [0.21.1]

### Changed

- Picks up `hitl-physics` 0.11.0: the thrust basis is re-anchored to lab bench
  data and the operating point is capped by available current. Every build's
  simulated thrust changes. The reported regression build (2212 1000KV, 6x4x3,
  4S 10 Ah) goes from TWR 0.6542 to **1.025** — it flies, which is what the user
  reported it used to do.

## [0.21.0]

### Fixed

- **A build that cannot lift its own weight is now reported instead of being
  silently handed an impossible hover throttle.** `hover_throttle_percent()` is
  `1/sqrt(TWR)`, so a thrust-to-weight ratio below 1.0 demands more than full
  throttle. The reported regression build (2212 1000KV, 6x4x3, 4S 10 Ah) needed
  123.6%; that was clamped to 0.8 and pushed to PX4 as `MPC_THR_HOVER`. The
  position controller then believed 80% throttle would hover, announced
  takeoff, and the airframe never moved. `MPC_THR_HOVER` is no longer pushed
  when the build cannot hover, the condition is logged at WARN, and
  `AppliedConfig` carries `can_hover` and `hover_required` so the browser can
  say so before launch. The decision is carried in `LastVerifiedParams`, so a
  re-push after an FC power cycle makes the same call.

- **Propeller mass was never read.** The propeller spec fetch pulled diameter,
  pitch and blade count but not `weightG`, so every propeller in every build
  used the 3 g `BuildSpec` default. A 6" tri-blade is nearer 8 g, counted four
  times.

- **The battery's measured mass was unreachable.** The daemon has always read
  `weightG` when `battery_slug` is present, but the browser never sent that
  field, so every build took the `capacity_mah * cells * 0.035` estimate. For a
  4S 10 Ah pack that is 1400 g against a real ~850 g — 55% of the reported
  build's all-up weight.

### Added

- `AppliedConfig.estimated_masses` lists the components whose mass was guessed
  rather than measured, so an estimate is never shown as a specification. The
  battery estimator's fit range (1100-4500 mAh, from its own three data points)
  is now named as `BATTERY_FIT_MIN_MAH`/`BATTERY_FIT_MAX_MAH`; applying it
  outside that span logs a warning and marks the entry `extrapolated`.

### Note

Correcting both mass defects takes the reported build from 2544.5 g to 2014.5 g
and its TWR from 0.6542 to 0.8263 — still below 1.0. The mass bugs were real
and are fixed, but they are not what stopped that build flying. See
`openspec/changes/fix-hitl-sim-flight-regression`.

## [0.20.1]

### Fixed

- **The vehicle fell through the terrain.** `TerrainCache::sample_ground_ned`
  needs a vertical datum and returns `None` without one, and the only
  production caller of `set_origin` — in `main` at startup — passed `None`.
  `ConfigureBuild` set the browser-supplied datum into `SharedOrigin`, which the
  barometer and `HIL_GPS` read, but never into the terrain cache, which is what
  ground contact reads. The datum stayed `None` for the whole session, so every
  ground sample returned "no ground" with a full set of tiles resident and the
  vehicle had nothing to land on. Both are now anchored together in
  `anchor_origin`, from one resolved origin, so they cannot disagree.
- The loop reported a missing datum as `Outside terrain coverage`, which is a
  different fault entirely — it sent a debugging session after tile coordinates
  that were resident and correct. The two cases now log distinctly, and
  `TerrainCache::describe_lookup` reports the coordinate wanted against those
  held.

## [0.20.0]

### Fixed

- **Terrain never reached the physics.** The WebSocket transport capped incoming
  messages at 64 KB while a single 256x256 f32 tile is 256 KB, so every terrain
  push the browser made was rejected with `Space limit exceeded` and the socket
  was closed. `TerrainTiles::MAX_FRAME_BYTES` (16 MiB) was unreachable — axum
  refused the frame before the parser saw it — and the protocol tests never
  crossed a socket, so nothing caught it. The physics silently ran every session
  on flat ground via its empty-cache fallback. The transport limit is now
  derived from the protocol's own bound rather than being a second independent
  number.
- **`SYS_HAS_BARO` is now pushed as 1, not 0.** The daemon simulates a
  barometer and ships it in every `HIL_SENSOR`; telling PX4 there was none meant
  `vehicle_air_data` was never published and EKF2 held height on GPS alone.
  Reboot-required, and the preflight flow already reboots.

### Changed

- Requires `hitl-sensors` 0.2.0, which stops applying a GPS module's datasheet
  accuracy as per-sample white noise. Builds that selected a GPS component
  could not arm: PX4 reported "vertical velocity unstable" and "height estimate
  not stable" because consecutive fixes jumped the full 3 m altitude sigma
  55 ms apart. The stated accuracy is preserved, but most of it now sits in a
  slowly-varying term, as it does in a real receiver.

## [0.19.1] - 2026-08-25

### Fixed

- **The vehicle would not arm with a GPS module selected.** PX4 reported
  "ekf2 missing data", "horizontal velocity unstable" and "height estimate not
  stable"; `sensor_gps` was publishing at 0 Hz while every other sensor ran
  normally.

  The cause was in `hitl-sensors`, not in this crate: the GPS delay buffer
  trimmed away every sample at or below its output target and then asked
  whether the remaining front was at or below that target, which after such a
  trim it could not be. A reading only escaped when the buffer happened to hold
  exactly one sample — true while the configured delay is shorter than one
  update period, so the built-in 10 Hz/80 ms shape worked and hid it. A
  component-database profile at 18 Hz/120 ms spans more than two periods, the
  buffer never drops below three, and GPS went silent for the rest of the
  session.

  Requires hitl-sensors 0.1.2.

## [0.19.0] - 2026-08-25

### Changed

- **BREAKING: the daemon no longer fetches terrain.** The browser is now the
  sole fetcher of elevation data: it resolves each tile, decodes it once, and
  pushes the decoded heights over the WebSocket. That is what makes "the physics
  collides against what the viewer draws" true by construction, rather than two
  systems independently resolving the same coordinate and happening to agree.
  Previously the browser fell back to a global elevation source for any
  coordinate on Earth while the daemon had no fallback at all, so off the one
  baked region the user saw hills and flew through them.

  `ConfigureBuild` loses `terrain_url`; the S3 host allowlist is gone; the
  `terrain` crate no longer depends on `reqwest` (or `serde`, `serde_json` or
  `tokio` — it is now a pure in-memory cache). `--terrain-url` is replaced by
  `--terrain-pack <dir>`, which reads `{z}/{x}/{y}.bin` from disk through the
  same validated ingress for headless and CI runs.

- **BREAKING: `ConfigureBuild` carries the flight location.** New
  `flight_location { lat, lon }` and `origin_elevation_msl`. A browser that
  sends neither still works and gets the documented default, so a stale tab
  degrades rather than breaking.

### Fixed

- **The vertical datum could differ between ground contact and the sensors.**
  The CLI path adopted the sampled terrain elevation into `reference_alt` while
  the WebSocket path did not, so ground collision sat on the terrain and the
  barometer and HIL_GPS sat on `--alt` — a standing altitude error the EKF had
  to absorb. The datum is now a single shared value set at configuration time
  and read by all three together. The terrain origin broadcast to the browser
  had the same fault, sending a datum snapshotted at startup; it now sends the
  live one.

### Added

- **Terrain follows the vehicle.** The fixed 3x3 ring loaded once at startup is
  replaced by a resident set bounded by tile count, evicting furthest-from-
  vehicle first and never the tile underfoot. The daemon asks the browser for
  what it is missing (`TerrainNeed`, `0x0F`) and the browser supplies it
  (`TerrainTiles`, `0x18`); unmet requests are simply re-stated, so the exchange
  recovers from dropped frames, tab reloads and restarts with no acknowledgement
  bookkeeping, and backs off when they go unanswered.

- **Every tile is validated at the boundary.** The WebSocket is now a data
  ingress, so submissions are checked for sample count, coordinate range,
  distance from the origin, and elevations that are finite and within Earth's
  real range, under a hard resident-memory bound. A rejected tile leaves
  previously accepted terrain untouched, and validation runs outside the write
  lock so a flood cannot stall the 400 Hz loop.

## [0.18.1] - 2026-08-23

### Fixed

- **Restoring a flight controller's original settings could not work at all.**
  The WebSocket server capped incoming messages at 1 KB, and a restore carries
  the whole parameter snapshot — 21 entries of name, value and type, about
  1.2 KB. The daemon rejected a message it had asked the browser to send, at
  the WebSocket layer, before any handler could log it, and closed the
  connection. The interface sat on "Writing your original settings" forever,
  over a board nothing had written to.

  The limit is now 64 KB: room for a snapshot several times larger than any
  board's parameter set, still bounded against a local client making the daemon
  buffer without limit. A test builds the real payload and fails if it no
  longer fits.

## [0.18.0] - 2026-08-23

### Added

- `PreflightStatus` now carries `board_identity` once the board has reported
  it. Identity previously reached the browser only with the parameter
  snapshot, which is captured *during* provisioning — after the point where
  approval for that provisioning is sought. Anything scoped to a board, such
  as asking the user to approve changing it, therefore could not name its
  subject and could not proceed. The field is optional, so an older client
  ignores it.

## [0.17.0] - 2026-08-23

### Added

- `ConfigResult` now carries the stage of the build apply that is running:
  `fetching_specs`, `computing`, `pushing_params`, `restarting_ekf`. Interim
  frames keep `state: configuring` and terminal results carry no stage, so a
  client that ignores the field behaves exactly as before.

  Applying a build takes several seconds of PX4 parameter acks plus an EKF2
  restart, and previously reported only "still working" for the whole of it —
  a slow step and a stuck one looked identical. Every stage is emitted when
  that work actually begins.

## [0.16.2] - 2026-08-23

### Fixed

- **Starting the daemon before plugging in the flight controller no longer
  hangs the daemon permanently.** A PX4 board spends ~5s in its bootloader on
  power-up under the same USB vendor ID it uses for application firmware, so
  detection — which matched on vendor ID alone — adopted a board that was still
  booting. The bootloader's CDC endpoint does not refuse the connection; the
  `open()` simply never returns, so the connection manager stopped for good:
  no retry, no log line, and unkillable by SIGTERM. Because the daemon scans
  once a second, it landed inside that window nearly every time, which is why
  the failure depended on plug order.

  Detection now separates a board in its bootloader from a board running
  firmware (`PX4 BL <board>` product string; the vendor ID is identical in both
  states) and refuses to open the former, reporting `SuspectedBootloader` so
  the interface can say the board is starting rather than missing.

  This also made the existing bootloader handling reachable for the first time.
  The 5s heartbeat watchdog and 10s port-release backoff sat downstream of that
  `open()` and had never once executed.

- Serial `open()` now has a 5s deadline. A device that accepts a connection and
  never completes it can no longer wedge the connection manager. This is a
  backstop, not the primary defence — an abandoned blocking open leaks its
  thread until the device is unplugged, so known-bad ports are still refused up
  front rather than relied on timing out.

## [0.16.1] - 2026-08-23

### Fixed
- **A dropped `PARAM_SAVE` no longer leads to a reboot.** Both provisioning and restore sent the save with `try_send` and ignored the outcome — restore silently, provisioning with a warning — then rebooted regardless. On a board already in HITL mode the MAVLink queue is saturated by the 400Hz sensor stream, so that save could be dropped; `PARAM_SET` only writes RAM, so the reboot would then silently discard every parameter just pushed. Worse, it reboots a board whose flash state is unknown, which is the condition that leaves one stuck in its bootloader. The save now uses the same backpressure-aware send as the parameter writes, and a failure aborts before the reboot with the values left in RAM rather than a reset of unknown consequence.

## [0.16.0] - 2026-08-23

### Added
- **Flash-settle cooldown between write cycles.** A second write-and-reboot cycle starting on the heels of the first can interrupt PX4's flash commit and leave the parameter store corrupted — the board then reports `PX4 BL FMU` indefinitely and needs a physical power cycle, which nothing in software can undo. Provisioning and restore now share a 15s window after any cycle that reached the writing stage, covering `PARAM_SAVE_SETTLE_DELAY` plus PX4's 3-5s bootloader dwell with margin. The refusal names the remaining wait rather than just declining. The pre-existing single-flight guard only covered *concurrent* operations; this covers sequential ones. Observed on real hardware.

## [0.15.1] - 2026-08-23

### Fixed
- **Provisioning refused every board on real hardware.** `main.rs` built `PreflightHandler` with `new()`, which creates its own empty board-identity cell, while the MAVLink receiver task populated a different one. The handler's identity was therefore permanently `None` and provisioning aborted with "reports no identifying serial" for every board. Every unit test passed because they all construct the handler with `with_identity` directly. Found by running against a real flight controller; regression tests now pin both the unwired default and the shared-cell behaviour.
- **Restore was unusable on a provisioned board.** Restore treated any `try_send` failure as a lost link, including a full queue. A board already in HITL mode is being streamed HIL sensors at 400Hz, so the MAVLink tx queue is saturated continuously — which is exactly when a restore is wanted. Backpressure now has its own retry budget, separate from the ack-retry budget it was previously exhausting, and only a genuinely disconnected writer is fatal.

## [0.15.0] - 2026-08-22

### Added
- **Capability frame on connect** — a JSON frame (0x0E) carrying daemon version, protocol revision and named feature flags is sent unprompted before any state update, so a client never has to interpret a frame before it knows what this daemon speaks. The binary handshake that predates it packs a version into fixed byte positions and already carries a legacy-layout heuristic to disambiguate two past encodings; named features let a mixed fleet degrade per-feature instead of per-version.
- **A ready line naming the URL to open**, printed only once the WebSocket socket is actually bound, so it is never a promise the daemon cannot keep. Overridable via `HITL_SIMULATOR_URL` for local and staging work.
- **Self-update via `--update`** — queries the release channel, downloads, checks the published SHA-256, swaps the binary atomically and keeps the previous one as `.previous`. Running the flag is the confirmation; nothing replaces the binary on its own. Signature verification is not possible: artifacts carry only an ad-hoc signature that authenticates no publisher, so integrity rests on the hash fetched over HTTPS.
- **A startup update check that cannot block startup** — detached and time-boxed, logged at debug on failure. A release channel that is down, slow, or behind a corporate proxy must never stop someone flying.

### Fixed
- **Port collision reported usefully.** Binding an occupied WebSocket port surfaced a bare "address in use"; it now names the port, states that another daemon is probably running, and exits non-zero instead of appearing to run.
- **Network errors no longer hide their cause.** reqwest's top-level message is "error sending request for url (...)", which does not say whether it was DNS, TLS, a timeout or a refused connection. The source chain is now flattened into the reported error.

## [0.14.0] - 2026-08-22

### Added
- **Heartbeat probing for unrecognised boards** — when no attached device matches one of the four known PX4 vendor IDs, candidate USB serial ports are opened read-only and listened to for a MAVLink HEARTBEAT. Boards outside those vendors no longer require `--port` by hand. The allowlist runs first, so recognised hardware never pays for probing, and probing also runs on reconnect so a board that comes back on a different port is still found.
- **A probe cannot transmit.** `scan_for_heartbeat` is handed a `Read` and nothing else, so the guarantee is structural rather than a convention. The device on the other end may be a 3D printer or a debug probe, and unsolicited MAVLink bytes could put it into a state its owner did not ask for. Ports whose names identify them as Bluetooth bridges, debug consoles or audio devices are excluded before any port is opened, and a port that stays quiet is released at the end of a bounded 1.5s window.
- **Examined ports are reported** when nothing is found. "No flight controller detected" is not actionable; the list of ports that were looked at is.
- **Explicit link state** (`searching` / `connected` / `reconnecting` / `suspected_bootloader`) alongside the existing booleans. `connected: false, reconnecting: true` could not distinguish a first scan from a reconnect, so a first-time user was told their board was "reconnecting" to something it had never been connected to. A present-but-silent board is likewise distinguishable from an absent one, because the remedy ("wait, do not unplug") is the opposite.

## [0.13.0] - 2026-08-22

### Added
- **Bounded automatic re-apply** — a provisioning cycle whose post-reboot verification fails is repeated once before the user sees an error. PX4 can report the old flags on its first HEARTBEAT after a reboot, and a parameter save that did not land is recoverable by pushing again. Bounded at 2 attempts: every cycle commits parameters to flash, so retrying a board that will never verify would wear it out. Only a verification failure retries — an unacked `PARAM_SET` or a board that never came back are not fixed by pushing the same values again.
- **Restore** — writes a stored snapshot back, saves, reboots, and reads every value back to confirm it took. Refuses a snapshot whose board identity does not match the connected board, a board with no identity, an empty snapshot, and sim-only mode. A value that does not read back is reported per-parameter with expected and actual, and the board is explicitly **not** reported as restored.
- **Provisioning progress is broadcast to every connected client** — a reloaded page or a second tab converges on the same state instead of being told an operation is "already in progress" with nothing to show. One fan-out path rather than a per-connection channel, so the tab that started the operation does not receive every frame twice.

### Fixed
- **Integer parameters were captured and restored as their raw bit pattern.** PX4 transports an INT32 parameter as the bits of the int32 reinterpreted inside `PARAM_VALUE`'s float field, so `SYS_HITL = 1` arrives as 1.4e-45. Capture stored that pattern verbatim and restore cast it back as a number, which would have written garbage to every integer parameter on the board — 18 of the 21 provisioning touches. `ParamValue::decoded_value()` now normalises on the way in, restore re-encodes per the recorded type, and acks for integer parameters are compared as bits rather than within a float epsilon. Caught by a restore test asserting the board holds what was asked for.

## [0.12.0] - 2026-08-22

### Added
- **Typed parameter values** — `PARAM_VALUE` is decoded into a `ParamValue` carrying PX4's declared `param_type` alongside name, value and table index, replacing the bare `(String, f32)` broadcast. PX4 silently drops a `PARAM_SET` whose type does not match the parameter's declared type, so a snapshot recording only name and value cannot be replayed onto the board. A zero-valued INT32 and a zero-valued REAL32 are indistinguishable by value alone.
- **Parameter reads** — `PARAM_REQUEST_READ` addressed by name with `param_index = -1`, since table indices shift between firmware builds. Retries on silence, subscribes before sending so a fast reply cannot land in the gap, and drains unrelated traffic (a QGC parameter pull) rather than treating it as a mismatch. `read_params` returns partial success plus the list of names that produced nothing.
- **Board identity** — derived from `AUTOPILOT_VERSION`'s hardware UID, with a composite fallback over vendor/product/board version/system id. Requested once the first HEARTBEAT proves the link, because requesting earlier races PX4's startup and the reply is lost. Firmware version is deliberately excluded from the composite so identity survives a reflash. `uid2` is not used: `mavlink` 0.13.1 does not generate that field.
- **Snapshot hand-off protocol** — `SnapshotCaptured` (0x0C) carries parameters read off the board to the browser; `SnapshotStored` (0x16) is the browser's confirmation that they are durably persisted. Provisioning will block on the acknowledgement, so a board is never modified before a restore point exists.
- **Session snapshot store** — the daemon holds one snapshot in memory for the session and refuses to hand it back for a different board. Deliberately never written to disk: the browser is the system of record, and a second persisted copy could disagree with it.

## [0.11.1] - 2026-07-29

### Fixed
- **Ground impact/sliding-friction desync** — the accelerometer impact impulse assumed the whole velocity vector was arrested in 100ms, which stopped being true once slopes could slide (tangential speed decays only ~4% in that window at the sliding damping rate). The slide decision is now made before the impulse, and on sliding ground only the into-surface component is arrested.
- **Surface normal coverage gap at tile-block edges** — central-difference normal sampling needs all four probes, so a 5m shell along the tile-block boundary had a resolvable height but no normal and silently defaulted to level. Boundary probes now fall back to one-sided differences, matching the height field's coverage exactly.
- **`ground_rest_accel_body` seeded from wrong config** — was read from `SimulationConfig::default()` instead of the constructor's config. Not exploitable today (gravity is identical in both paths and the field is refreshed before first sample), but correctness depended on an unenforced cross-crate invariant.

## [0.11.0] - 2026-07-29

### Added
- **Sloped ground contact** — `TerrainCache::sample_ground_normal_ned` derives a unit surface normal from central differences over the height field. Resting attitude now follows the normal with heading preserved instead of snapping to `(0, 0, yaw)` on every slope.
- **Coulomb friction threshold** — replaces the flat 0.9/tick damping (effectively infinite stiction at 400Hz) with a slope-angle threshold (`tan(theta) = 0.6`): below it the drone stays parked, above it gravity along the surface wins and it slides.

### Changed
- Resting accelerometer reading splits by branch: level ground keeps the exact `[0, 0, -g]` the EKF depends on; genuine slopes report the gravity reaction rotated into the tilted airframe's own frame.

## [0.10.1] - 2026-07-29

### Removed
- **Dead `TerrainProvider`** — a second, unused terrain implementation (192 lines) with a different vertical datum than the live `TerrainCache`; reviving it would have silently disagreed with physics ground.
- **Write-only `reference_alt`** on `TerrainCache` — never read after datum unification; removed from `load`/`load_from_tiles` and all call sites. Workspace now builds with zero warnings.

## [0.10.0] - 2026-07-29

### Fixed
- **Ground coverage gaps no longer clamp to flat ground** — `ground_z` is now `Option<f64>`; sampling outside cached terrain previously collapsed to `0.0` via `unwrap_or`, teleporting a drone flying below the origin datum straight up to it. Unknown ground now disables the clamp and logs a rate-limited warning.
- **Ground impact impulse rotated into body frame** — was computed from NED velocity and reported verbatim as body-frame specific force, putting the lateral component on the wrong body axis at any non-zero yaw. The full `(a - g)` vector is now rotated by the impact attitude.
- **Landing detection now visible** — daemon consumes `EXTENDED_SYS_STATE` and forwards PX4's `MAV_LANDED_STATE` to the browser (wire format grows to 87 bytes, byte `[86]`), so a disagreement with simulated ground contact is a diagnostic instead of invisible.
- **Altitude datum unified** — DEM elevation at origin becomes `reference_alt` on terrain load, so ground collision, baro, and HIL_GPS share one datum instead of collision using DEM while baro/GPS used `--alt`.

## [0.9.9] - 2026-06-22

### Added
- **Sensor profile logging** — `ConfigureBuild` now logs resolved sensor noise values and match type (exact/mcu_family/average/default) for diagnostics.
- **Sensor match type passthrough** — accepts optional `sensor_match_type` field in WebSocket config to track how the profile was resolved.

## [0.9.6] - 2026-06-21

### Changed
- **Origin terrain elevation as vertical reference** — `sample_ground_ned()` now returns NED-down relative to terrain at origin, not the absolute reference_alt. At (north=0, east=0), ground_z=0 — matching the frontend 3D mesh where Y=0 at origin. Origin elevation stored at load time for consistent vertical datum.

## [0.9.5] - 2026-06-21

### Added
- **WebSocket terrain URL** — frontend can now pass `terrain_url` in `ConfigureBuild` message. Daemon validates URL is from whitelisted S3 buckets (`th3seus-terrain`, `th3seus-terrain-playground`) and loads tiles dynamically. No CLI flag needed when using web UI.

### Changed
- **Shared terrain cache** — `TerrainCache` now shared between simulation and `BuildConfigHandler`. Terrain loaded via WebSocket updates the same cache used by physics loop.

## [0.9.4] - 2026-06-21

### Added
- **Terrain-aware ground collision** — new `--terrain-url` CLI flag loads XYZ terrain tiles from S3. Ground collision detection now uses real terrain elevation instead of flat Z=0. Terrain cache populated at startup for 3x3 tile grid around reference position.
- **New `terrain` crate** — XYZ/Slippy tile loader with sync `TerrainCache` for physics loop sampling.

## [0.9.3] - 2026-06-21

### Fixed
- **TERRAIN_ORIGIN altitude now uses simulation reference_alt** — the alt field in TERRAIN_ORIGIN message now contains the simulation's MSL ground level (1655m for Boulder CO) instead of the GPS_GLOBAL_ORIGIN altitude (often 0 due to ellipsoid datum). Terrain in the 3D viewer now renders at the correct height relative to the drone.

## [0.9.2] - 2026-06-21

### Added
- **TERRAIN_ORIGIN WebSocket message (0x09)** — broadcasts GPS origin to connected clients for terrain rendering. Extracts from MAVLink GPS_GLOBAL_ORIGIN > HOME_POSITION > GLOBAL_POSITION_INT with priority gating. 22-byte binary: tag + f64 lat + f64 lon + f32 alt + u8 source. Late-joining clients receive cached origin immediately.

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
