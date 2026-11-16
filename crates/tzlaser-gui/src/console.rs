use eframe::egui::{self, Color32, RichText};
use std::collections::VecDeque;
use std::time::Instant;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub enum ConsoleLevel {
    Info,
    Motion,
    Warn,
    Error,
}

#[derive(Debug, Clone)]
pub struct ConsoleEntry {
    pub elapsed_secs: f64,
    pub level: ConsoleLevel,
    pub message: String,
}

pub struct ConsoleState {
    pub expanded: bool,
    pub auto_scroll: bool,
    started_at: Instant,
    entries: VecDeque<ConsoleEntry>,
}

impl Default for ConsoleState {
    fn default() -> Self {
        let mut state = Self {
            expanded: false,
            auto_scroll: true,
            started_at: Instant::now(),
            entries: VecDeque::with_capacity(512),
        };
        state.log(ConsoleLevel::Info, "Tzlaser console initialized.");
        state
    }
}

impl ConsoleState {
    pub fn log(&mut self, level: ConsoleLevel, msg: impl Into<String>) {
        let message = msg.into();
        let elapsed_secs = self.started_at.elapsed().as_secs_f64();
        eprintln!("[{:.2}s] {}", elapsed_secs, message);
        self.entries.push_back(ConsoleEntry {
            elapsed_secs,
            level,
            message,
        });
        while self.entries.len() > 500 {
            self.entries.pop_front();
        }
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn clear(&mut self) {
        self.entries.clear();
    }

    pub fn format_all(&self) -> String {
        self.entries
            .iter()
            .map(|e| format!("[{:>7.2}s] {}", e.elapsed_secs, e.message))
            .collect::<Vec<_>>()
            .join("\n")
    }
}

pub fn render_console(ui: &mut egui::Ui, state: &mut ConsoleState) {
    ui.horizontal(|ui| {
        ui.label(
            RichText::new("Message Console")
                .size(12.0)
                .strong()
                .color(Color32::from_rgb(220, 228, 242)),
        );
        ui.label(
            RichText::new(format!("({} lines)", state.len()))
                .size(11.0)
                .weak(),
        );

        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.small_button("Clear").clicked() {
                state.clear();
            }
            if ui
                .small_button("Copy All")
                .on_hover_text("Copy all console messages to clipboard")
                .clicked()
            {
                ui.ctx().copy_text(state.format_all());
            }
            ui.checkbox(&mut state.auto_scroll, "Auto-scroll");
        });
    });

    ui.separator();

    egui::Frame::new()
        .fill(Color32::from_rgb(14, 17, 22))
        .inner_margin(egui::Margin::symmetric(6, 4))
        .show(ui, |ui| {
            let scroll = egui::ScrollArea::vertical()
                .id_salt("message_console_scroll")
                .auto_shrink([false, false])
                .stick_to_bottom(state.auto_scroll);

            scroll.show(ui, |ui| {
                ui.set_min_width(ui.available_width());
                if state.entries.is_empty() {
                    ui.label(
                        RichText::new("No messages yet.")
                            .monospace()
                            .size(11.5)
                            .weak(),
                    );
                    return;
                }
                for entry in &state.entries {
                    let color = match entry.level {
                        ConsoleLevel::Info => Color32::from_rgb(200, 210, 225),
                        ConsoleLevel::Motion => Color32::from_rgb(110, 220, 255),
                        ConsoleLevel::Warn => Color32::from_rgb(255, 200, 95),
                        ConsoleLevel::Error => Color32::from_rgb(255, 115, 115),
                    };
                    let line = format!("[{:>7.2}s] {}", entry.elapsed_secs, entry.message);
                    ui.label(RichText::new(line).monospace().size(11.5).color(color));
                }
            });
        });
}
