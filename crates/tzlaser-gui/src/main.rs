use eframe::egui::{self, Color32, Frame, Margin, RichText};
use kurbo::Point;

use tzlaser_core::{Document, Entity, HistoryManager, PulseConfig};
use tzlaser_protocol::{LaserController, MachineStatus};

mod canvas;
mod layer_popup;
mod panels;
mod properties_window;
mod ribbon;
mod selection_hud;

use canvas::{render_canvas, CanvasState};
use panels::{render_right_panels, PanelState};
use properties_window::PropertiesWindowState;
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
    props_window_state: PropertiesWindowState,
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

        let pulse_config = PulseConfig::load_or_init(PulseConfig::default_config_path());

        let mut canvas_state = CanvasState::default();
        canvas_state.bed_width_mm = pulse_config.table_width_mm as f32;
        canvas_state.bed_height_mm = pulse_config.table_height_mm as f32;
        canvas_state.origin_corner = pulse_config.origin_corner.clone();

        let mut panel_state = PanelState::default();
        panel_state.material_db =
            tzlaser_core::MaterialLibrary::load_or_init(tzlaser_core::MaterialLibrary::default_db_path());
        if !pulse_config.controller_ip.trim().is_empty() {
            panel_state.target_ip = pulse_config.controller_ip.clone();
        }
        panel_state.jog_mode = panels::JogMode::Step(pulse_config.default_step_mm.clamp(0.01, 1000.0));

        let controller = if let Ok(addr) = panel_state.target_ip.parse::<std::net::SocketAddr>() {
            let c = LaserController::new(addr);
            let (min_speed, jog_speed, accel) = pulse_config.jog_pulses_params();
            c.update_jog_config(
                tzlaser_protocol::JogConfig {
                    min_speed,
                    jog_speed,
                    accel,
                },
                pulse_config.z_jog_speed_reg,
            );
            Some(c)
        } else {
            None
        };

        Self {
            document: doc,
            selection: Vec::new(),
            history: HistoryManager::default(),
            canvas_state,
            panel_state,
            hud_state: SelectionHudState::default(),
            ribbon_state: RibbonState::default(),
            props_window_state: PropertiesWindowState::default(),
            pulse_config,
            controller,
            status: MachineStatus::default(),
            status_message: "Ready. Auto-connecting to controller (10.1.1.168:502 & :999).".to_string(),
        }
    }
}

#[cfg(target_os = "windows")]
fn windows_dark_visuals() -> egui::Visuals {
    use egui::{epaint::Shadow, Stroke};

    let mut visuals = egui::Visuals::dark();
    visuals.dark_mode = true;
    visuals.override_text_color = None;

    // Distinct dark slate surfaces tuned for Windows gamma (2.2) contrast
    visuals.panel_fill = Color32::from_rgb(30, 34, 42);
    visuals.window_fill = Color32::from_rgb(34, 39, 49);
    visuals.window_stroke = Stroke::new(1.0_f32, Color32::from_rgb(78, 88, 110));
    visuals.window_shadow = Shadow {
        offset: [0, 4],
        blur: 12,
        spread: 0,
        color: Color32::from_black_alpha(120),
    };
    visuals.extreme_bg_color = Color32::from_rgb(18, 21, 27);
    visuals.faint_bg_color = Color32::from_rgb(38, 43, 54);
    visuals.code_bg_color = Color32::from_rgb(22, 26, 34);

    // Non-interactive widgets (labels, group frames, separators)
    // Note: `weak_bg_fill` on noninteractive is used by egui's `gray_out()` for `.weak()` text and disabled items
    visuals.widgets.noninteractive.bg_fill = Color32::from_rgb(34, 39, 49);
    visuals.widgets.noninteractive.weak_bg_fill = Color32::from_rgb(125, 136, 156);
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(68, 77, 96));
    visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(232, 238, 248));

    // Interactive widgets at rest (buttons, combo boxes, drag values, checkboxes)
    visuals.widgets.inactive.bg_fill = Color32::from_rgb(48, 55, 68);
    visuals.widgets.inactive.weak_bg_fill = Color32::from_rgb(48, 55, 68);
    visuals.widgets.inactive.bg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(82, 92, 114));
    visuals.widgets.inactive.fg_stroke = Stroke::new(1.2_f32, Color32::from_rgb(242, 246, 255));

    // Hovered widgets
    visuals.widgets.hovered.bg_fill = Color32::from_rgb(62, 72, 90);
    visuals.widgets.hovered.weak_bg_fill = Color32::from_rgb(62, 72, 90);
    visuals.widgets.hovered.bg_stroke = Stroke::new(1.2_f32, Color32::from_rgb(0, 210, 255));
    visuals.widgets.hovered.fg_stroke = Stroke::new(1.5_f32, Color32::WHITE);

    // Active / pressed widgets
    visuals.widgets.active.bg_fill = Color32::from_rgb(38, 100, 155);
    visuals.widgets.active.weak_bg_fill = Color32::from_rgb(38, 100, 155);
    visuals.widgets.active.bg_stroke = Stroke::new(1.2_f32, Color32::from_rgb(80, 230, 255));
    visuals.widgets.active.fg_stroke = Stroke::new(1.5_f32, Color32::WHITE);

    // Open combo-box / menu widgets
    visuals.widgets.open.bg_fill = Color32::from_rgb(44, 68, 102);
    visuals.widgets.open.weak_bg_fill = Color32::from_rgb(44, 68, 102);
    visuals.widgets.open.bg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(0, 210, 255));
    visuals.widgets.open.fg_stroke = Stroke::new(1.2_f32, Color32::WHITE);

    // Selected items (selectable_label, text selection)
    visuals.selection.bg_fill = Color32::from_rgb(28, 115, 185);
    visuals.selection.stroke = Stroke::new(1.0_f32, Color32::WHITE);

    visuals
}

#[cfg(target_os = "windows")]
fn configure_windows_theme_and_fonts(ctx: &egui::Context) {
    use std::sync::Arc;

    let mut fonts = egui::FontDefinitions::default();

    if let Ok(bytes) = std::fs::read("C:\\Windows\\Fonts\\segoeui.ttf") {
        fonts.font_data.insert(
            "segoeui".to_owned(),
            Arc::new(egui::FontData::from_owned(bytes)),
        );
        if let Some(vec) = fonts.families.get_mut(&egui::FontFamily::Proportional) {
            vec.insert(0, "segoeui".to_owned());
        }
    }

    if let Ok(bytes) = std::fs::read("C:\\Windows\\Fonts\\consola.ttf") {
        fonts.font_data.insert(
            "consola".to_owned(),
            Arc::new(egui::FontData::from_owned(bytes)),
        );
        if let Some(vec) = fonts.families.get_mut(&egui::FontFamily::Monospace) {
            vec.insert(0, "consola".to_owned());
        }
    }

    // Load Segoe UI Symbol so geometric arrows (▲ ▼ ◄ ►) and UI symbols render instead of □ boxes
    if let Ok(bytes) = std::fs::read("C:\\Windows\\Fonts\\seguisym.ttf") {
        fonts.font_data.insert(
            "seguisym".to_owned(),
            Arc::new(egui::FontData::from_owned(bytes)),
        );
        if let Some(vec) = fonts.families.get_mut(&egui::FontFamily::Proportional) {
            let insert_pos = if vec.first().map(|s| s.as_str()) == Some("segoeui") { 1 } else { 0 };
            vec.insert(insert_pos, "seguisym".to_owned());
        }
        if let Some(vec) = fonts.families.get_mut(&egui::FontFamily::Monospace) {
            vec.push("seguisym".to_owned());
        }
    }

    ctx.set_fonts(fonts);
    ctx.set_theme(egui::Theme::Dark);
    ctx.set_visuals(windows_dark_visuals());
}

impl eframe::App for TzlaserApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        #[cfg(target_os = "windows")]
        if !ctx.style().visuals.dark_mode {
            ctx.set_theme(egui::Theme::Dark);
            ctx.set_visuals(windows_dark_visuals());
        }

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
        let top_bar_fill = if cfg!(target_os = "windows") {
            Color32::from_rgb(26, 30, 38)
        } else {
            Color32::from_rgb(22, 24, 28)
        };
        egui::TopBottomPanel::top("top_panel")
            .frame(Frame::new().fill(top_bar_fill).inner_margin(Margin::symmetric(10, 6)))
            .show(ctx, |ui| {
                render_ribbon(
                    ui,
                    &mut self.ribbon_state,
                    &mut self.document,
                    &mut self.selection,
                    &mut self.history,
                    &mut self.canvas_state,
                    &mut self.panel_state,
                    &mut self.pulse_config,
                    &mut self.props_window_state,
                    &mut self.controller,
                    &self.status,
                    &mut self.status_message,
                );
            });

        // 2. Bottom Status Bar
        let status_bar_fill = if cfg!(target_os = "windows") {
            Color32::from_rgb(24, 28, 36)
        } else {
            Color32::from_rgb(20, 21, 24)
        };
        let status_text_color = if cfg!(target_os = "windows") {
            Color32::from_rgb(220, 228, 242)
        } else {
            Color32::from_rgb(180, 185, 195)
        };
        egui::TopBottomPanel::bottom("status_bar")
            .frame(Frame::new().fill(status_bar_fill).inner_margin(Margin::symmetric(10, 4)))
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(&self.status_message).size(12.0).color(status_text_color));
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
        let right_panel_fill = if cfg!(target_os = "windows") {
            Color32::from_rgb(30, 34, 42)
        } else {
            Color32::from_rgb(24, 25, 29)
        };
        egui::SidePanel::right("right_panel")
            .min_width(280.0)
            .default_width(320.0)
            .frame(Frame::new().fill(right_panel_fill).inner_margin(Margin::same(10)))
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
            let can_context_menu =
                self.panel_state.run_mode == panels::RunMode::Idle || self.panel_state.is_paused;
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
                can_context_menu,
            );
            if let Some(target_pt) = self.canvas_state.pending_continue_from_point.take() {
                if let Some(msg) = self.panel_state.set_pause_at_path_point(
                    &self.document,
                    target_pt,
                    &self.controller,
                    &mut self.status,
                    &self.pulse_config,
                ) {
                    self.status_message = msg;
                }
            }
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

        // 6. Floating Layer Properties & DB Pop-up Window (opened by clicking a layer in the right legend)
        layer_popup::render_layer_popup(
            ctx,
            &mut self.panel_state,
            &mut self.document,
            &mut self.status_message,
        );

        // 7. Table & Machine Properties Pop-up Window (opened from Ribbon → ⚙ Properties → 📐 Table Properties)
        properties_window::render_properties_window(
            ctx,
            &mut self.props_window_state,
            &mut self.pulse_config,
            &mut self.canvas_state,
            &mut self.panel_state,
            &self.controller,
            &mut self.status_message,
        );
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
        Box::new(|cc| {
            #[cfg(target_os = "windows")]
            configure_windows_theme_and_fonts(&cc.egui_ctx);
            #[cfg(not(target_os = "windows"))]
            let _ = cc;
            Ok(Box::new(TzlaserApp::default()))
        }),
    )
}
