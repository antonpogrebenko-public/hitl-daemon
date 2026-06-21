//! Terminal UI for HITL daemon — status panel + scrolling log area

use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use protocol::{DaemonState, DaemonStatus, SimulationStats};
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Layout},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph, Wrap},
    Terminal,
};
use std::collections::VecDeque;
use std::io::stdout;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::watch;
use tracing::Subscriber;
use tracing_subscriber::Layer;

const MAX_LOG_LINES: usize = 200;
const TUI_REFRESH_MS: u64 = 500;

/// Run the TUI on the current thread (blocking). Call from a dedicated OS thread.
pub fn run_tui(
    status_rx: watch::Receiver<DaemonStatus>,
    sim_stats_rx: watch::Receiver<SimulationStats>,
    log_rx: std::sync::mpsc::Receiver<String>,
    shutdown: Arc<AtomicBool>,
) {
    // Install panic hook that restores terminal before printing panic info
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let _ = disable_raw_mode();
        let _ = stdout().execute(LeaveAlternateScreen);
        default_hook(info);
    }));

    let result = run_tui_inner(status_rx, sim_stats_rx, log_rx, shutdown);

    // Always restore terminal, even on error
    let _ = disable_raw_mode();
    let _ = stdout().execute(LeaveAlternateScreen);

    if let Err(e) = result {
        eprintln!("TUI error: {e}");
    }
}

fn run_tui_inner(
    status_rx: watch::Receiver<DaemonStatus>,
    sim_stats_rx: watch::Receiver<SimulationStats>,
    log_rx: std::sync::mpsc::Receiver<String>,
    shutdown: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error>> {
    enable_raw_mode()?;
    stdout().execute(EnterAlternateScreen)?;

    let backend = CrosstermBackend::new(stdout());
    let mut terminal = Terminal::new(backend)?;
    let mut log_buffer: VecDeque<String> = VecDeque::with_capacity(MAX_LOG_LINES);

    loop {
        if shutdown.load(Ordering::Relaxed) {
            break;
        }

        // Drain log messages
        while let Ok(line) = log_rx.try_recv() {
            if log_buffer.len() >= MAX_LOG_LINES {
                log_buffer.pop_front();
            }
            log_buffer.push_back(line);
        }

        let status = status_rx.borrow().clone();

        let sim = sim_stats_rx.borrow().clone();

        terminal.draw(|frame| {
            // 8 content rows + 2 borders.
            let chunks = Layout::vertical([
                Constraint::Length(10),
                Constraint::Min(5),
                Constraint::Length(1),
            ])
            .split(frame.area());

            // Each header row is laid out as two columns. Left column =
            // 8-char label + 30-char value. Right column = same. Total
            // width = 2 (margin) + 38 (col 1) + 2 (gap) + 38 (col 2) = 80
            // — fits a standard terminal and aligns labels across rows.
            //
            // Use `kv!` for label+value cells where the value is plain
            // text. Lines that need styled spans (the state/armed dot)
            // are built manually but use the same 8/30 widths.
            const LABEL_W: usize = 8;
            const VALUE_W: usize = 30;

            // Helper: render a cell as raw text. Returns a String exactly
            // (LABEL_W + VALUE_W) chars wide — pad short values, truncate
            // long ones so the right column always starts at the same col.
            fn cell(label: &str, value: impl AsRef<str>) -> String {
                let mut v = value.as_ref().to_string();
                if v.chars().count() > VALUE_W {
                    v.truncate(VALUE_W);
                }
                format!("{label:<LABEL_W$}{v:<VALUE_W$}")
            }

            // Helper: full row from two cells joined by a 2-char gutter.
            fn row(left: String, right: String) -> Line<'static> {
                Line::from(Span::raw(format!("  {left}  {right}")))
            }

            // Helper: row whose first cell carries a colored value span.
            // The label is plain, the value is styled, and we hand-pad the
            // styled value to VALUE_W so the right cell still aligns.
            fn row_styled(
                left_label: &str,
                left_value: Span<'static>,
                left_value_visible_len: usize,
                right: String,
            ) -> Line<'static> {
                let pad = VALUE_W.saturating_sub(left_value_visible_len);
                Line::from(vec![
                    Span::raw(format!("  {left_label:<LABEL_W$}")),
                    left_value,
                    Span::raw(format!("{:pad$}  {right}", "", pad = pad)),
                ])
            }

            // ── Connection / FC ───────────────────────────────────────
            let state_color = match status.state {
                DaemonState::Streaming => Color::Green,
                DaemonState::Connected => Color::Cyan,
                DaemonState::WaitingForFc | DaemonState::Reconnecting => Color::Yellow,
                DaemonState::FcLost => Color::Red,
                DaemonState::ShuttingDown => Color::Magenta,
                DaemonState::Starting => Color::White,
            };
            let state_text = format!("● {}", status.state);
            let state_text_len = state_text.chars().count();
            let state_span = Span::styled(
                state_text,
                Style::default()
                    .fg(state_color)
                    .add_modifier(Modifier::BOLD),
            );
            let line_state = row_styled(
                "State",
                state_span,
                state_text_len,
                cell("FC", status.fc_model.as_deref().unwrap_or("—")),
            );

            let line_port = row(
                cell("Port", status.serial_port.as_deref().unwrap_or("none")),
                cell(
                    "Clients",
                    format!(
                        "{}  pkts {}/s",
                        status.connected_clients, status.packets_per_sec
                    ),
                ),
            );

            // ── Loop performance ──────────────────────────────────────
            let tick_color = if sim.tick_rate_hz >= 380.0 {
                Color::Green
            } else if sim.tick_rate_hz >= 320.0 {
                Color::Yellow
            } else if sim.tick_rate_hz > 0.0 {
                Color::Red
            } else {
                Color::DarkGray
            };
            let tick_text = format!("{:.1} Hz", sim.tick_rate_hz);
            let tick_text_len = tick_text.chars().count();
            let tick_span = Span::styled(tick_text, Style::default().fg(tick_color));
            let line_loop = row_styled(
                "Loop",
                tick_span,
                tick_text_len,
                cell(
                    "Latency",
                    format!(
                        "avg {} µs / max {} µs",
                        sim.avg_latency_us, sim.max_latency_us
                    ),
                ),
            );

            let line_drops = row(
                cell("Drops", sim.sensor_drops.to_string()),
                cell("Uptime", format!("{} s", status.uptime_secs)),
            );

            // ── Drone state ───────────────────────────────────────────
            let (arm_label, arm_color) = if sim.armed {
                ("ARMED", Color::Red)
            } else {
                ("disarm", Color::DarkGray)
            };
            let arm_text_len = arm_label.chars().count();
            let arm_span = Span::styled(
                arm_label.to_string(),
                Style::default().fg(arm_color).add_modifier(Modifier::BOLD),
            );
            let line_arm = row_styled(
                "Armed",
                arm_span,
                arm_text_len,
                cell(
                    "Mode",
                    format!("{}    sim t  {:.1} s", sim.flight_mode, sim.sim_time_s),
                ),
            );

            let line_pos = row(
                cell(
                    "Pos NED",
                    format!(
                        "{:>6.2}  {:>6.2}  {:>6.2} m",
                        sim.position_ned[0], sim.position_ned[1], sim.position_ned[2]
                    ),
                ),
                cell(
                    "HIL",
                    format!(
                        "s {}  g {}  a {}",
                        sim.hil_sensor_count, sim.hil_gps_count, sim.actuator_count
                    ),
                ),
            );

            // ── Motors ────────────────────────────────────────────────
            let line_motors = row(
                cell(
                    "Motors",
                    format!(
                        "{:>5.0} {:>5.0} {:>5.0} {:>5.0} RPM",
                        sim.motor_rpms[0], sim.motor_rpms[1], sim.motor_rpms[2], sim.motor_rpms[3]
                    ),
                ),
                cell("", ""),
            );

            // ── Attitude ──────────────────────────────────────────────
            // Light up red when the drone is sitting non-level while disarmed
            // (phantom attitude → rate-loop trembling on arm). Threshold 5°
            // chosen to ignore the slerp-in-progress transient (which clears
            // in ~200 ms) but catch a real inverted-on-ground state.
            let roll = sim.attitude_rpy_deg[0];
            let pitch = sim.attitude_rpy_deg[1];
            let yaw = sim.attitude_rpy_deg[2];
            let phantom_tilt = !sim.armed && (roll.abs() > 5.0 || pitch.abs() > 5.0);
            let att_text = format!(
                "r {:>+6.1}°  p {:>+6.1}°  y {:>+6.1}°{}",
                roll,
                pitch,
                yaw,
                if phantom_tilt {
                    "   ⚠ inverted on ground — reconfigure"
                } else {
                    ""
                }
            );
            let att_color = if phantom_tilt {
                Color::Red
            } else {
                Color::DarkGray
            };
            let att_text_len = att_text.chars().count();
            let att_span = Span::styled(att_text, Style::default().fg(att_color));
            let line_att = row_styled("Att", att_span, att_text_len, cell("", ""));

            // ── Battery + Build ───────────────────────────────────────
            let batt_color = if sim.battery_percent >= 30.0 {
                Color::Green
            } else if sim.battery_percent >= 10.0 {
                Color::Yellow
            } else {
                Color::Red
            };
            let batt_text = format!(
                "{:>5.2} V  {:>3.0}%",
                sim.battery_voltage, sim.battery_percent
            );
            let batt_text_len = batt_text.chars().count();
            let batt_span = Span::styled(batt_text, Style::default().fg(batt_color));
            let build_value = if sim.build_configured {
                format!(
                    "{:>5.0} g    TWR {:>4.2}",
                    sim.mass_kg * 1000.0,
                    sim.thrust_to_weight
                )
            } else {
                "not configured".to_string()
            };
            let line_batt = row_styled(
                "Battery",
                batt_span,
                batt_text_len,
                cell("Build", build_value),
            );

            let version = env!("CARGO_PKG_VERSION");
            let status_block = Block::default()
                .borders(Borders::ALL)
                .title(format!(" HITL Daemon v{version} "))
                .border_style(Style::default().fg(Color::DarkGray));

            let status_widget = Paragraph::new(vec![
                line_state,
                line_port,
                line_loop,
                line_drops,
                line_arm,
                line_pos,
                line_motors,
                line_att,
                line_batt,
            ])
            .block(status_block);
            frame.render_widget(status_widget, chunks[0]);

            // Log area
            let log_lines: Vec<Line> = log_buffer
                .iter()
                .map(|l| {
                    let color = if l.contains("ERROR") || l.contains("error") {
                        Color::Red
                    } else if l.contains("WARN") || l.contains("warn") {
                        Color::Yellow
                    } else if l.contains("INFO") || l.contains("info") {
                        Color::Gray
                    } else {
                        Color::DarkGray
                    };
                    Line::from(Span::styled(l.as_str(), Style::default().fg(color)))
                })
                .collect();

            let log_area_height = chunks[1].height as usize;
            let total_lines = log_lines.len();
            let scroll_offset = total_lines.saturating_sub(log_area_height);

            let log_widget = Paragraph::new(log_lines)
                .wrap(Wrap { trim: false })
                .scroll((scroll_offset as u16, 0))
                .block(Block::default().borders(Borders::NONE));
            frame.render_widget(log_widget, chunks[1]);

            // Footer
            let footer = Paragraph::new(Line::from(Span::styled(
                " Ctrl+C to stop",
                Style::default().fg(Color::DarkGray),
            )));
            frame.render_widget(footer, chunks[2]);
        })?;

        // Poll for keyboard events (non-blocking with timeout)
        if event::poll(Duration::from_millis(TUI_REFRESH_MS))? {
            if let Event::Key(key) = event::read()? {
                if key.code == KeyCode::Char('c') && key.modifiers.contains(KeyModifiers::CONTROL) {
                    shutdown.store(true, Ordering::SeqCst);
                    break;
                }
            }
        }
    }

    Ok(())
}

// ─── Custom Tracing Layer ────────────────────────────────────────────────────

/// A tracing Layer that sends formatted log lines to the TUI via mpsc channel
pub struct TuiLayer {
    tx: std::sync::mpsc::SyncSender<String>,
}

impl TuiLayer {
    pub fn new(tx: std::sync::mpsc::SyncSender<String>) -> Self {
        Self { tx }
    }
}

impl<S: Subscriber> Layer<S> for TuiLayer {
    fn on_event(
        &self,
        event: &tracing::Event<'_>,
        _ctx: tracing_subscriber::layer::Context<'_, S>,
    ) {
        use tracing::Level;

        let meta = event.metadata();
        let level = match *meta.level() {
            Level::ERROR => "ERROR",
            Level::WARN => "WARN ",
            Level::INFO => "INFO ",
            Level::DEBUG => "DEBUG",
            Level::TRACE => "TRACE",
        };

        let now = chrono::Local::now().format("%H:%M:%S");

        let mut message = String::new();
        let mut visitor = MessageVisitor(&mut message);
        event.record(&mut visitor);

        let line = format!(" {now} {level} {message}");
        let _ = self.tx.try_send(line);
    }
}

struct MessageVisitor<'a>(&'a mut String);

impl tracing::field::Visit for MessageVisitor<'_> {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
        use std::fmt::Write;
        if field.name() == "message" {
            let _ = write!(self.0, "{:?}", value);
        } else {
            if !self.0.is_empty() {
                self.0.push(' ');
            }
            let _ = write!(self.0, "{}={:?}", field.name(), value);
        }
    }

    fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
        use std::fmt::Write;
        if field.name() == "message" {
            let _ = write!(self.0, "{}", value);
        } else {
            if !self.0.is_empty() {
                self.0.push(' ');
            }
            let _ = write!(self.0, "{}={}", field.name(), value);
        }
    }
}
