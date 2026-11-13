use eframe::egui::{self, Color32, Frame, Margin, RichText};
use kurbo::Point;

use tzlaser_core::{Document, Entity, HistoryManager, PulseConfig};
use tzlaser_protocol::{LaserController, MachineStatus};

mod canvas;
mod panels;
mod ribbon;
mod selection_hud;

use canvas::{render_canvas, CanvasState};
use panels::{render_right_panels, PanelState};
use ribbon::{render_ribbon, RibbonState};
use selection_hud::{render_selection_hud, SelectionHudState};

struct TzlaserApp {
    document: Document,
    selection: Vec<(usize, usize)>,
    history: HistoryManager,
    canvas_state: CanvasState,
    panel_state: PanelState,
    hud_state: SelectionHudState,
    ribbon_state: RibbonState,
    pulse_config: PulseConfig,
    controller: Option<LaserController>,
    status: MachineStatus,
    status_message: String,
}

impl Default for TzlaserApp {
    fn default() -> Self {
        let mut doc = Document::new();
        // Add sample demo cut shapes
        if let Some(cut_layer) = doc.layers.first_mut() {
            cut_layer.entities.push(Entity::Polyline {
                points: vec![
                    Point::new(50.0, 50.0),
                    Point::new(250.0, 50.0),
                    Point::new(250.0, 150.0),
                    Point::new(50.0, 150.0),
                ],
                closed: true,
            });
            cut_layer.entities.push(Entity::Circle {
                center: Point::new(150.0, 100.0),
                radius: 30.0,
            });
        }

        Self {
            document: doc,
            selection: Vec::new(),
            history: HistoryManager::default(),
            canvas_state: CanvasState::default(),
            panel_state: PanelState::default(),
            hud_state: SelectionHudState::default(),
            ribbon_state: RibbonState::default(),
            pulse_config: PulseConfig::default(),
            controller: None,
            status: MachineStatus::default(),
            status_message: "Ready. Click 'Open DXF' or select profiles to position on bed.".to_string(),
        }
    }
}

impl eframe::App for TzlaserApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Poll status from background thread if connected
        if let Some(ref c) = self.controller {
            if c.is_connected() {
                self.status = c.get_status();
            }
            ctx.request_repaint_after(std::time::Duration::from_millis(30));
        }

        // Advance active Start / Dry-Cut simulation along toolpaths
        if self.panel_state.run_mode != panels::RunMode::Idle && !self.panel_state.is_paused {
            if let Some(done_msg) = self.panel_state.tick_simulation(
                &self.controller,
                &mut self.status,
                &self.pulse_config,
            ) {
                self.status_message = done_msg;
            }
            ctx.request_repaint();
        }

        // Global Keyboard Shortcuts: Cmd+Z (Undo) and Cmd+Shift+Z / Cmd+Y (Redo)
        let cmd = ctx.input(|i| i.modifiers.command || i.modifiers.ctrl);
        let shift = ctx.input(|i| i.modifiers.shift);
        if cmd && !shift && ctx.input(|i| i.key_pressed(egui::Key::Z)) {
            if let Some(desc) = self.history.undo(&mut self.document, &mut self.selection) {
                self.status_message = format!("Undo: {}", desc);
            }
        }
        if (cmd && shift && ctx.input(|i| i.key_pressed(egui::Key::Z)))
            || (cmd && ctx.input(|i| i.key_pressed(egui::Key::Y)))
        {
            if let Some(desc) = self.history.redo(&mut self.document, &mut self.selection) {
                self.status_message = format!("Redo: {}", desc);
            }
        }

        // Synchronize active step between Canvas keyboard shortcuts and HUD
        self.canvas_state.active_step_mm = self.hud_state.step_mm;

        // 1. Industrial CAD/CAM Ribbon Menu
        egui::TopBottomPanel::top("top_panel")
            .frame(Frame::new().fill(Color32::from_rgb(22, 24, 28)).inner_margin(Margin::symmetric(10, 6)))
            .show(ctx, |ui| {
                render_ribbon(
                    ui,
                    &mut self.ribbon_state,
                    &mut self.document,
                    &mut self.selection,
                    &mut self.history,
                    &mut self.canvas_state,
                    &mut self.panel_state,
                    &mut self.controller,
                    &self.status,
                    &mut self.status_message,
                );
            });

        // 2. Bottom Status Bar
        egui::TopBottomPanel::bottom("status_bar")
            .frame(Frame::new().fill(Color32::from_rgb(20, 21, 24)).inner_margin(Margin::symmetric(10, 4)))
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(&self.status_message).size(12.0).color(Color32::from_rgb(180, 185, 195)));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(RichText::new(format!("Bed: {:.0} x {:.0} mm", self.canvas_state.bed_width_mm, self.canvas_state.bed_height_mm)).size(11.0).weak());
                        ui.separator();
                        ui.label(RichText::new(format!("Entities: {}", self.document.entity_count())).size(11.0).weak());
                        ui.separator();
                        if !self.selection.is_empty() {
                            ui.colored_label(
                                Color32::from_rgb(0, 220, 255),
                                format!("Selected: {} items", self.selection.len()),
                            );
                            ui.separator();
                        }
                        ui.label(RichText::new(format!("Zoom: {:.1}x", self.canvas_state.zoom)).size(11.0).weak());
                    });
                });
            });

        // 3. Right Control Panel
        egui::SidePanel::right("right_panel")
            .min_width(280.0)
            .default_width(320.0)
            .frame(Frame::new().fill(Color32::from_rgb(24, 25, 29)).inner_margin(Margin::same(10)))
            .show(ctx, |ui| {
                render_right_panels(
                    ui,
                    &mut self.panel_state,
                    &self.controller,
                    &mut self.status,
                    &self.pulse_config,
                    &mut self.document,
                    &mut self.status_message,
                );
            });

        // 4. Central 2D Vector CAD Canvas
        egui::CentralPanel::default().show(ctx, |ui| {
            let laser_pos = Point::new(
                self.pulse_config.pulses_to_mm_x(self.status.pos_x),
                self.pulse_config.pulses_to_mm_y(self.status.pos_y),
            );
            let paused_points_mm: Vec<Point> = self
                .panel_state
                .pause_buffer
                .iter()
                .map(|c| c.point_mm)
                .collect();
            let active_pause_mm = self.panel_state.paused_checkpoint.map(|c| c.point_mm);
            render_canvas(
                ui,
                &mut self.canvas_state,
                &mut self.document,
                &mut self.selection,
                &mut self.history,
                laser_pos,
                self.panel_state.run_mode == panels::RunMode::DryCut,
                self.status.laser_on,
                &paused_points_mm,
                active_pause_mm,
            );
        });

        // 5. Floating Selection Coordinates & D-Pad Nudge Pop-up Window (hidden during interactive manual sort)
        if !self.canvas_state.manual_sort_active {
            render_selection_hud(
                ctx,
                &mut self.hud_state,
                &mut self.document,
                &mut self.selection,
                &mut self.history,
                self.canvas_state.bed_width_mm,
                self.canvas_state.bed_height_mm,
            );
        }
    }
}

fn main() -> eframe::Result<()> {
    env_logger::init();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Ţlaser")
            .with_inner_size([1280.0, 800.0])
            .with_min_inner_size([960.0, 600.0]),
        ..Default::default()
    };

    eframe::run_native(
        "Ţlaser",
        native_options,
        Box::new(|_cc| Ok(Box::new(TzlaserApp::default()))),
    )
}
