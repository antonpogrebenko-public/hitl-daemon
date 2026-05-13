//! Terminal UI for HITL daemon — status panel + scrolling log area

use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use protocol::{DaemonState, DaemonStatus};
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

    let result = run_tui_inner(status_rx, log_rx, shutdown);

    // Always restore terminal, even on error
    let _ = disable_raw_mode();
    let _ = stdout().execute(LeaveAlternateScreen);

    if let Err(e) = result {
        eprintln!("TUI error: {e}");
    }
}

fn run_tui_inner(
    status_rx: watch::Receiver<DaemonStatus>,
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

        terminal.draw(|frame| {
            let chunks = Layout::vertical([
                Constraint::Length(4),
                Constraint::Min(5),
                Constraint::Length(1),
            ])
            .split(frame.area());

            // Status panel
            let state_color = match status.state {
                DaemonState::Streaming => Color::Green,
                DaemonState::Connected => Color::Cyan,
                DaemonState::WaitingForFc | DaemonState::Reconnecting => Color::Yellow,
                DaemonState::FcLost => Color::Red,
                DaemonState::ShuttingDown => Color::Magenta,
                DaemonState::Starting => Color::White,
            };

            let state_indicator = Span::styled(
                format!("● {}", status.state),
                Style::default().fg(state_color).add_modifier(Modifier::BOLD),
            );

            let fc_info = status.fc_model.as_deref().unwrap_or("—");
            let port_info = status.serial_port.as_deref().unwrap_or("none");

            let line1 = Line::from(vec![
                Span::raw("  State: "),
                state_indicator,
                Span::raw(format!("          FC: {fc_info}")),
            ]);

            let line2 = Line::from(vec![Span::raw(format!(
                "  Port:  {port_info:<24} Packets: {}/s  Clients: {}",
                status.packets_per_sec, status.connected_clients
            ))]);

            let version = env!("CARGO_PKG_VERSION");
            let status_block = Block::default()
                .borders(Borders::ALL)
                .title(format!(" HITL Daemon v{version} "))
                .border_style(Style::default().fg(Color::DarkGray));

            let status_widget = Paragraph::new(vec![line1, line2]).block(status_block);
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
                if key.code == KeyCode::Char('c')
                    && key.modifiers.contains(KeyModifiers::CONTROL)
                {
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
