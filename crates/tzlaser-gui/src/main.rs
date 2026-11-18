use eframe::egui::{self, Color32, Frame, Margin, RichText};
use kurbo::Point;

use tzlaser_core::{Document, Entity, HistoryManager, PulseConfig};
use tzlaser_protocol::{LaserController, MachineStatus};

mod canvas;
mod console;
mod layer_popup;
mod panels;
mod properties_window;
mod ribbon;
mod selection_hud;
mod signal_graph;

use canvas::{render_canvas, CanvasState};
use console::{render_console, ConsoleLevel, ConsoleState};
use panels::{render_right_panels, PanelState};
use properties_window::PropertiesWindowState;
use ribbon::{render_ribbon, RibbonState};
use selection_hud::{render_selection_hud, SelectionHudState};
use signal_graph::{render_signal_graph, SignalGraphState};

struct TzlaserApp {
    document: Document,
    selection: Vec<(usize, usize)>,
    history: HistoryManager,
    canvas_state: CanvasState,
    panel_state: PanelState,
    hud_state: SelectionHudState,
    ribbon_state: RibbonState,
    props_window_state: PropertiesWindowState,
    signal_graph: SignalGraphState,
    console: ConsoleState,
    pulse_config: PulseConfig,
    controller: Option<LaserController>,
    status: MachineStatus,
    status_message: String,
    last_logged_status: String,
    last_logged_connected: bool,
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
        canvas_state.bed_width_mm = pulse_config.cut_width_mm() as f32;
        canvas_state.bed_height_mm = pulse_config.cut_height_mm() as f32;
        canvas_state.origin_corner = pulse_config.origin_corner.clone();
        canvas_state.home_retract_x_mm = pulse_config.home_retract_x_mm.max(0.0);
        canvas_state.home_retract_y_mm = pulse_config.home_retract_y_mm.max(0.0);

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
            signal_graph: SignalGraphState::default(),
            console: ConsoleState::default(),
            pulse_config,
            controller,
            status: MachineStatus::default(),
            status_message: "Ready. Auto-connecting to controller (10.1.1.168:502 & :999).".to_string(),
            last_logged_status: String::new(),
            last_logged_connected: false,
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

#[cfg(not(target_os = "windows"))]
fn configure_unix_fonts(ctx: &egui::Context) {
    use std::sync::Arc;

    let mut fonts = egui::FontDefinitions::default();

    let symbol_candidates: &[(&str, &str)] = &[
        ("apple_symbols", "/System/Library/Fonts/Apple Symbols.ttf"),
        ("arial_unicode", "/System/Library/Fonts/Supplemental/Arial Unicode.ttf"),
        ("noto_symbols", "/usr/share/fonts/truetype/noto/NotoSansSymbols-Regular.ttf"),
        ("noto_symbols2", "/usr/share/fonts/truetype/noto/NotoSansSymbols2-Regular.ttf"),
        ("dejavu_sans", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    ];

    for (name, path) in symbol_candidates {
        if let Ok(bytes) = std::fs::read(path) {
            fonts.font_data.insert(
                (*name).to_owned(),
                Arc::new(egui::FontData::from_owned(bytes)),
            );
            if let Some(vec) = fonts.families.get_mut(&egui::FontFamily::Proportional) {
                vec.push((*name).to_owned());
            }
            if let Some(vec) = fonts.families.get_mut(&egui::FontFamily::Monospace) {
                vec.push((*name).to_owned());
            }
        }
    }

    ctx.set_fonts(fonts);
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
        let hw_connected = self.controller.as_ref().map_or(false, |c| c.is_connected());
        if self.panel_state.run_mode != panels::RunMode::Idle && !self.panel_state.is_paused {
            if let Some(done_msg) = self.panel_state.tick_simulation(
                &self.controller,
                &mut self.status,
                &self.pulse_config,
            ) {
                self.status_message = done_msg;
                if !hw_connected {
                    self.signal_graph.record_offline(0.0, 0.0, 0.0, 0.0, false, Some("Done"));
                }
            } else if !hw_connected {
                if let Some(seg) = self.panel_state.trajectory.get(self.panel_state.current_seg_idx).copied() {
                    let speed_mult = if self.panel_state.run_mode != panels::RunMode::DryCut {
                        1.0
                    } else {
                        self.panel_state.sim_speed_multiplier
                    };
                    let eff_speed = (seg.speed_mm_s * speed_mult).max(0.5);
                    let len = seg.length_mm.max(1e-9);
                    let target_vx = ((seg.end.x - seg.start.x) / len) * eff_speed * self.pulse_config.pulses_per_mm_x;
                    let target_vy = ((seg.end.y - seg.start.y) / len) * eff_speed * self.pulse_config.pulses_per_mm_y;
                    let max_accel_pls = (self.pulse_config.jog_accel_mm_s2.max(20.0)
                        * self.pulse_config.pulses_per_mm_x.max(self.pulse_config.pulses_per_mm_y)
                        * speed_mult) as f32;
                    let (vx_pls, vy_pls) = self.panel_state.step_filtered_velocity(
                        target_vx as f32,
                        target_vy as f32,
                        max_accel_pls,
                        0.016,
                    );
                    let tag = if self.panel_state.run_mode == panels::RunMode::DryCut {
                        "Dry-Cut"
                    } else if seg.is_rapid {
                        "Rapid"
                    } else {
                        "Cut"
                    };
                    self.signal_graph.record_offline(
                        vx_pls,
                        vy_pls,
                        0.0,
                        0.0,
                        self.status.laser_on,
                        Some(tag),
                    );
                }
            }
            ctx.request_repaint();
        }

        // Enforce cutting-area bounds on the head position every frame.
        // Skip while hardware is unhomed (unreferenced coordinates), actively homing,
        // or running a physical table-axis measurement.
        let measuring_axis = self
            .props_window_state
            .measure_axis
            .as_ref()
            .is_some_and(|m| m.is_running());
        let can_enforce_bounds = if hw_connected {
            self.status.xy_homed && !self.status.is_homing_xy && !self.status.is_homing_z && !measuring_axis
        } else {
            !measuring_axis
        };
        if can_enforce_bounds && (self.status.pos_x != 0 || self.status.pos_y != 0) {
            let head_world = panels::head_world_mm(&self.status, &self.pulse_config);
            if panels::point_within_cut_area(head_world, &self.pulse_config) {
                self.panel_state.oob_latched = false;
            } else if !self.panel_state.oob_latched && self.panel_state.oob_alert.is_none() {
                self.console.log(
                    ConsoleLevel::Warn,
                    format!(
                        "[OOB] pos=({},{}) world=({:.2},{:.2}) cut=({:.1},{:.1}) xy_homed={} moving={}",
                        self.status.pos_x,
                        self.status.pos_y,
                        head_world.x,
                        head_world.y,
                        self.pulse_config.cut_width_mm(),
                        self.pulse_config.cut_height_mm(),
                        self.status.xy_homed,
                        self.status.is_moving,
                    ),
                );
                let reason = format!(
                    "Head position jumped outside the effective cutting area ({:.1}, {:.1} mm vs {:.1} × {:.1} mm).",
                    head_world.x,
                    head_world.y,
                    self.pulse_config.cut_width_mm(),
                    self.pulse_config.cut_height_mm()
                );
                self.status_message = self.panel_state.trigger_out_of_bounds_recovery(
                    &reason,
                    &self.controller,
                    &mut self.status,
                );
                if !hw_connected {
                    self.signal_graph.record_offline(0.0, 0.0, 0.0, 0.0, false, Some("OOB Reset"));
                }
            }
        } else if self.status.pos_x == 0 && self.status.pos_y == 0 {
            self.panel_state.oob_latched = false;
        }

        // Prompt to home machine on first connection to controller board
        let is_connected = self.controller.as_ref().map_or(false, |c| c.is_connected());
        if !is_connected {
            self.panel_state.prompted_home_this_connection = false;
            self.panel_state.show_home_prompt = false;
        } else if !self.panel_state.prompted_home_this_connection {
            self.panel_state.prompted_home_this_connection = true;
            if !self.status.xy_homed || !self.status.z_homed {
                self.panel_state.show_home_prompt = true;
            }
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
                    let arrow = if self.signal_graph.expanded { "▼" } else { "▲" };
                    let toggle_btn = egui::Button::new(
                        RichText::new(format!("{} Signal Graph", arrow))
                            .size(11.0)
                            .color(if self.signal_graph.expanded {
                                Color32::from_rgb(0, 220, 255)
                            } else {
                                Color32::from_rgb(190, 200, 215)
                            }),
                    )
                    .fill(if self.signal_graph.expanded {
                        Color32::from_rgb(24, 52, 68)
                    } else {
                        Color32::from_rgb(42, 46, 56)
                    });
                    if ui
                        .add(toggle_btn)
                        .on_hover_text("Show/hide real-time axis command signal graph")
                        .clicked()
                    {
                        self.signal_graph.expanded = !self.signal_graph.expanded;
                    }
                    let console_arrow = if self.console.expanded { "▼" } else { "▲" };
                    let console_btn = egui::Button::new(
                        RichText::new(format!("{} Console", console_arrow))
                            .size(11.0)
                            .color(if self.console.expanded {
                                Color32::from_rgb(0, 220, 255)
                            } else {
                                Color32::from_rgb(190, 200, 215)
                            }),
                    )
                    .fill(if self.console.expanded {
                        Color32::from_rgb(24, 52, 68)
                    } else {
                        Color32::from_rgb(42, 46, 56)
                    });
                    if ui
                        .add(console_btn)
                        .on_hover_text("Show/hide message console")
                        .clicked()
                    {
                        self.console.expanded = !self.console.expanded;
                    }
                    ui.separator();
                    ui.label(RichText::new(&self.status_message).size(12.0).color(status_text_color));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(
                            RichText::new(format!(
                                "Cut Area: {:.0} x {:.0} mm (Table: {:.0} x {:.0} mm)",
                                self.canvas_state.bed_width_mm,
                                self.canvas_state.bed_height_mm,
                                self.pulse_config.table_width_mm,
                                self.pulse_config.table_height_mm
                            ))
                            .size(11.0)
                            .weak(),
                        );
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
                    &mut self.signal_graph,
                );
            });

        // 3b. Collapsible & Resizable Lower Panels (Message Console + Signal Graph, stacked)
        for msg in self.panel_state.pending_logs.drain(..) {
            self.console.log(ConsoleLevel::Motion, msg);
        }
        if is_connected != self.last_logged_connected {
            self.last_logged_connected = is_connected;
            if is_connected {
                self.console.log(
                    ConsoleLevel::Info,
                    format!("Controller connected at {}.", self.panel_state.target_ip),
                );
            } else {
                self.console.log(ConsoleLevel::Warn, "Controller disconnected.");
            }
        }
        if self.status_message != self.last_logged_status {
            self.last_logged_status = self.status_message.clone();
            let lvl = if self.status_message.contains("⚠️") {
                ConsoleLevel::Warn
            } else {
                ConsoleLevel::Info
            };
            self.console.log(lvl, self.status_message.clone());
        }

        if self.console.expanded {
            let console_fill = Color32::from_rgb(18, 21, 27);
            egui::TopBottomPanel::bottom("message_console_panel")
                .resizable(true)
                .min_height(80.0)
                .default_height(135.0)
                .max_height(360.0)
                .frame(Frame::new().fill(console_fill).inner_margin(Margin::symmetric(8, 6)))
                .show(ctx, |ui| {
                    render_console(ui, &mut self.console);
                });
        }

        if self.signal_graph.expanded {
            let graph_fill = Color32::from_rgb(20, 23, 29);
            egui::TopBottomPanel::bottom("motion_telemetry_panel")
                .resizable(true)
                .min_height(90.0)
                .default_height(165.0)
                .max_height(420.0)
                .frame(Frame::new().fill(graph_fill).inner_margin(Margin::symmetric(8, 6)))
                .show(ctx, |ui| {
                    let is_active_motion = (self.panel_state.run_mode != panels::RunMode::Idle
                        && !self.panel_state.is_paused)
                        || self.status.is_moving;
                    render_signal_graph(ui, &mut self.signal_graph, &self.controller, is_active_motion);
                });
        }

        // 4. Central 2D Vector CAD Canvas
        egui::CentralPanel::default().show(ctx, |ui| {
            let (origin_world, _, _) = self.canvas_state.origin_world_and_axes();
            let laser_pos = Point::new(
                origin_world.x + self.pulse_config.pulses_to_mm_x(self.status.pos_x),
                origin_world.y + self.pulse_config.pulses_to_mm_y(self.status.pos_y),
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
            let completed_before = self.panel_state.completed_before_path_1based();
            let initial_rapid = self.panel_state.initial_rapid_waypoints();
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
                completed_before,
                &initial_rapid,
                &self.panel_state.active_frame_polygon,
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
            if let Some((path_idx_1based, target_pt)) = self.canvas_state.pending_set_start_point.take() {
                if self.document.set_path_start_point(path_idx_1based, target_pt) {
                    self.history.record(&self.document, &self.selection, "Set Path Start Point");
                    self.status_message = format!(
                        "Starting point of Path #{} set to X: {:.2}, Y: {:.2} mm.",
                        path_idx_1based, target_pt.x, target_pt.y
                    );
                } else {
                    self.status_message = format!(
                        "Path #{} does not support re-anchoring its starting point.",
                        path_idx_1based
                    );
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
                &self.canvas_state.origin_corner,
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
            &self.status,
            &mut self.status_message,
        );

        // 8. Initial Connection Homing Prompt
        if self.panel_state.show_home_prompt {
            let mut close_prompt = false;
            egui::Window::new("🏠 Home Machine?")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
                .show(ctx, |ui| {
                    ui.set_min_width(360.0);
                    ui.label(
                        RichText::new("Controller board connected!")
                            .size(13.5)
                            .strong()
                            .color(Color32::from_rgb(120, 235, 180)),
                    );
                    ui.add_space(4.0);
                    ui.label(
                        RichText::new(format!(
                            "Would you like to home the cutting head now?\n\n• Z-axis will move UP until its upper sensor triggers.\n• X and Y axes will move toward {} until both origin sensors trigger.",
                            self.pulse_config.origin_corner
                        ))
                        .size(12.0),
                    );
                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        let home_all_btn = egui::Button::new(
                            RichText::new("🏠 Home All Axes").color(Color32::WHITE).strong(),
                        )
                        .fill(Color32::from_rgb(28, 125, 78))
                        .min_size(egui::Vec2::new(125.0, 28.0));
                        if ui.add(home_all_btn).clicked() {
                            if let Some(ref c) = self.controller {
                                c.home_all(panels::home_config_from_pulse(&self.pulse_config));
                                self.status_message = format!(
                                    "Homing Z (up) and X/Y toward {}...",
                                    self.pulse_config.origin_corner
                                );
                            }
                            close_prompt = true;
                        }

                        let home_z_btn = egui::Button::new(
                            RichText::new("⬆ Z-Home Only").color(Color32::WHITE).strong(),
                        )
                        .fill(Color32::from_rgb(32, 85, 130))
                        .min_size(egui::Vec2::new(110.0, 28.0));
                        if ui.add(home_z_btn).clicked() {
                            if let Some(ref c) = self.controller {
                                c.home_z();
                                self.status_message =
                                    "Homing Z-axis (moving Z-up to sensor)...".to_string();
                            }
                            close_prompt = true;
                        }

                        let skip_btn = egui::Button::new(RichText::new("Skip").size(12.0))
                            .fill(Color32::from_rgb(48, 52, 62))
                            .min_size(egui::Vec2::new(72.0, 28.0));
                        if ui.add(skip_btn).clicked() {
                            close_prompt = true;
                        }
                    });
                });
            if close_prompt {
                self.panel_state.show_home_prompt = false;
            }
        }

        // 9. Capacitive Height Sensor Calibration Dialog
        if self.ribbon_state.show_sensor_cal_dialog {
            let mut close_cal_dialog = false;
            let is_calibrating = self.status.is_calibrating_z || self.status.z_state_code == 7;
            if self.ribbon_state.sensor_cal_started && !is_calibrating && self.status.z_state_code == 0 {
                if !self.status.z_pos_sensor && !self.status.z_homed {
                    self.ribbon_state.sensor_cal_aborted = true;
                }
            }

            let (phase_text, phase_color): (String, Color32) = if let Some(ref err) = self.status.z_cal_error {
                let suffix = if self.status.is_homing_z || self.status.z_state_code == 2 {
                    " — retracting Z-axis to home..."
                } else if self.status.z_pos_sensor || self.status.z_homed {
                    " — Z-axis returned to home."
                } else {
                    ""
                };
                (format!("Calibration FAILED: {}{}", err, suffix), Color32::from_rgb(255, 95, 95))
            } else if is_calibrating {
                if self.status.z_velocity > 100 {
                    ("Phase 1/3: Fast Descent (Z-homing speed) — searching for sheet...".to_string(), Color32::from_rgb(100, 200, 255))
                } else if self.status.z_velocity > 0 {
                    ("Phase 2/3: Slow Touchdown (10% speed) — approaching sheet...".to_string(), Color32::from_rgb(255, 210, 80))
                } else if self.status.z_velocity == 0 && self.status.raw_capacitance == 0 {
                    ("Touchdown detected! Recording surface zero...".to_string(), Color32::from_rgb(120, 235, 180))
                } else if self.status.z_velocity < 0 && self.status.z_velocity > -100 {
                    ("Phase 3/3: Slow Retract off sheet...".to_string(), Color32::from_rgb(160, 220, 255))
                } else if self.status.z_velocity <= -100 {
                    ("Phase 3/3: Fast Z-Home Retract to upper limit...".to_string(), Color32::from_rgb(120, 200, 255))
                } else {
                    ("Calibrating capacitive height sensor...".to_string(), Color32::from_rgb(255, 215, 0))
                }
            } else if self.ribbon_state.sensor_cal_aborted {
                if self.status.is_homing_z || self.status.z_state_code == 2 {
                    ("Calibration STOPPED — returning Z-axis to home (max height)...".to_string(), Color32::from_rgb(255, 165, 80))
                } else if self.status.z_pos_sensor || self.status.z_homed {
                    ("Calibration STOPPED — Z-axis returned to home.".to_string(), Color32::from_rgb(255, 120, 120))
                } else {
                    ("Calibration STOPPED.".to_string(), Color32::from_rgb(255, 110, 110))
                }
            } else if self.ribbon_state.sensor_cal_started && (self.status.z_homed || self.status.z_pos_sensor) {
                ("Calibration COMPLETE — head returned to upper Z-home.".to_string(), Color32::from_rgb(120, 235, 180))
            } else {
                ("Ready. Descends at Z-homing speed, slows to 10% near sheet for touchdown, then Z-homes.".to_string(), Color32::from_rgb(200, 210, 225))
            };

            egui::Window::new("🎯 Capacitive Height Sensor Calibration")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
                .show(ctx, |ui| {
                    ui.set_min_width(470.0);
                    ui.label(RichText::new(phase_text).size(13.0).strong().color(phase_color));
                    ui.add_space(6.0);
                    ui.separator();
                    ui.add_space(6.0);

                    // Sync Z scale & material thickness to controller when changed or when dialog is open
                    let ppm_z = self.pulse_config.pulses_per_mm_z.max(1.0);
                    if (self.status.z_material_thickness_mm - self.ribbon_state.cal_material_thickness_mm).abs() > 1e-4
                        || (self.status.z_pulses_per_mm - ppm_z).abs() > 1e-3
                    {
                        if let Some(ref c) = self.controller {
                            c.set_z_material_config(self.ribbon_state.cal_material_thickness_mm, ppm_z);
                        }
                    }

                    ui.horizontal(|ui| {
                        ui.label(RichText::new("Material Thickness (mm):").strong())
                            .on_hover_text("Thickness of the metal sheet under the laser head.\nUsed with Z pulses/mm after calibration to compute the sheet top ('material line') and bed depth.");
                        let thick_resp = ui.add(
                            egui::DragValue::new(&mut self.ribbon_state.cal_material_thickness_mm)
                                .speed(0.05)
                                .range(0.0..=50.0)
                                .fixed_decimals(2)
                                .suffix(" mm"),
                        );
                        if thick_resp.changed() {
                            if let Some(ref c) = self.controller {
                                c.set_z_material_config(self.ribbon_state.cal_material_thickness_mm, ppm_z);
                            }
                        }
                        ui.add_space(12.0);
                        ui.label(
                            RichText::new(format!("Z Scale: {:.0} pls/mm", ppm_z))
                                .size(11.5)
                                .color(Color32::from_rgb(165, 180, 200)),
                        )
                        .on_hover_text("Z-axis displacement calibration from Machine Settings (pulses per mm).");
                    });
                    ui.add_space(6.0);
                    ui.separator();
                    ui.add_space(6.0);

                    let z_down_mm = -(self.status.pos_z as f64) / ppm_z;

                    egui::Grid::new("sensor_cal_telemetry_grid")
                        .num_columns(2)
                        .spacing([24.0, 6.0])
                        .striped(true)
                        .show(ui, |ui| {
                            ui.label(RichText::new("Sensor Height Above Sheet (D8)").color(Color32::from_rgb(175, 185, 200)))
                                .on_hover_text("Live calibrated capacitive distance from nozzle tip to the metal sheet (0.000 mm = touchdown, 25.000 mm = out of range).");
                            ui.label(RichText::new(format!("{:.3} mm", self.status.capacitive_height_mm)).strong().monospace());
                            ui.end_row();

                            ui.label(RichText::new("Raw Capacitance Reading (D7)").color(Color32::from_rgb(175, 185, 200)))
                                .on_hover_text("Raw capacitive sensor counts from the Z board (~383,200 in open air, drops toward 0 at sheet touchdown).");
                            ui.label(RichText::new(format!("{}  ({:.2} pF)", self.status.raw_capacitance, self.status.capacitance_pf)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Z-Axis Position From Home (D9)").color(Color32::from_rgb(175, 185, 200)))
                                .on_hover_text("Live Z-axis displacement downward from upper Z-home, converted using Z pulses/mm.");
                            ui.label(RichText::new(format!("{:.3} mm  ({} pls)", z_down_mm, self.status.pos_z)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Z-Axis Velocity Command (D12)").color(Color32::from_rgb(175, 185, 200)))
                                .on_hover_text("Live motor velocity command on Port 999:\n+300 = Fast descent\n+30 = 10% slow touchdown\n-30 = Slow retract\n-300 = Fast Z-home up");
                            ui.label(RichText::new(format!("{}", self.status.z_velocity)).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Z Controller Mode (D6)").color(Color32::from_rgb(175, 185, 200)))
                                .on_hover_text("Internal state code reported by the Port 999 Z-axis / capacitive follower controller.");
                            ui.label(RichText::new(format!("{} ({})", self.status.z_state_code, match self.status.z_state_code {
                                0 => "Idle",
                                2 => "Homing",
                                7 => "Calibrating",
                                9 => "Jogging",
                                _ => "Active",
                            })).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Upper Z+ Home Limit Switch (D5)").color(Color32::from_rgb(175, 185, 200)))
                                .on_hover_text("Upper Z-axis home sensor state (TRIGGERED when head is at max height).");
                            let (sensor_lbl, sensor_col) = if self.status.z_pos_sensor {
                                ("TRIGGERED", Color32::from_rgb(120, 235, 180))
                            } else {
                                ("Clear", Color32::from_rgb(180, 190, 205))
                            };
                            ui.label(RichText::new(sensor_lbl).color(sensor_col).monospace());
                            ui.end_row();

                            ui.label(RichText::new("Calibrated Material Line").color(Color32::from_rgb(175, 185, 200)))
                                .on_hover_text("Sheet top surface ('material line') and table bed depth below Z-home, computed from the last completed calibration touchdown + Material Thickness.");
                            if let (Some(mat_mm), Some(bed_mm)) = (
                                self.status.z_material_line_mm_from_home(),
                                self.status.z_bed_mm_from_home(),
                            ) {
                                ui.label(RichText::new(format!("{:.3} mm from home  (Bed: {:.3} mm)", mat_mm, bed_mm)).monospace());
                            } else {
                                ui.label(RichText::new("Not calibrated yet").color(Color32::from_rgb(150, 160, 175)).monospace());
                            }
                            ui.end_row();

                            ui.label(RichText::new("Material Line Clearance").color(Color32::from_rgb(175, 185, 200)))
                                .on_hover_text("Distance of the laser head above the calibrated material line.\nNegative values mean the head is BELOW the material line and pushing against the sheet!");
                            if let Some(clearance) = self.status.z_clearance_above_material_mm() {
                                if self.status.is_below_material_line() {
                                    ui.label(
                                        RichText::new(format!("⚠ BELOW MATERIAL LINE ({:.3} mm) — PUSHING SHEET!", clearance))
                                            .strong()
                                            .color(Color32::from_rgb(255, 85, 85))
                                            .monospace(),
                                    );
                                } else {
                                    ui.label(
                                        RichText::new(format!("{:+.3} mm above sheet", clearance))
                                            .color(Color32::from_rgb(120, 235, 180))
                                            .monospace(),
                                    );
                                }
                            } else {
                                ui.label(RichText::new("--").color(Color32::from_rgb(150, 160, 175)).monospace());
                            }
                            ui.end_row();
                        });

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(6.0);

                    ui.horizontal(|ui| {
                        let can_start = self.controller.is_some() && self.status.is_connected && !is_calibrating;
                        let start_btn = egui::Button::new(
                            RichText::new("▶ Start Calibration").color(Color32::WHITE).strong(),
                        )
                        .fill(if can_start { Color32::from_rgb(28, 125, 78) } else { Color32::from_rgb(45, 55, 50) })
                        .min_size(egui::Vec2::new(140.0, 30.0));
                        if ui.add_enabled(can_start, start_btn).clicked() {
                            if let Some(ref c) = self.controller {
                                c.set_z_material_config(self.ribbon_state.cal_material_thickness_mm, ppm_z);
                                c.calibrate_height_sensor();
                                self.ribbon_state.sensor_cal_started = true;
                                self.ribbon_state.sensor_cal_aborted = false;
                                self.status_message = "Height sensor calibration started (fast descent -> 10% slow touchdown -> Z-home)...".to_string();
                            }
                        }

                        let stop_btn = egui::Button::new(
                            RichText::new("⏹ STOP").color(Color32::WHITE).strong(),
                        )
                        .fill(Color32::from_rgb(185, 38, 38))
                        .min_size(egui::Vec2::new(96.0, 30.0));
                        if ui.add_enabled(self.controller.is_some(), stop_btn).clicked() {
                            if let Some(ref c) = self.controller {
                                c.stop();
                                self.ribbon_state.sensor_cal_aborted = true;
                                self.ribbon_state.sensor_cal_started = false;
                                self.status_message = "Height sensor calibration stopped.".to_string();
                            }
                        }

                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let close_btn = egui::Button::new(RichText::new("Close").size(12.0))
                                .fill(Color32::from_rgb(48, 52, 62))
                                .min_size(egui::Vec2::new(72.0, 30.0));
                            if ui.add(close_btn).clicked() {
                                close_cal_dialog = true;
                            }
                        });
                    });
                });

            if close_cal_dialog {
                self.ribbon_state.show_sensor_cal_dialog = false;
            }
        }

        // 9. Out-of-Bounds / Cutting Area Limit Alert
        if let Some((alert_msg, needs_rehome)) = self.panel_state.oob_alert.clone() {
            let mut dismiss = false;
            egui::Window::new("⚠ Cutting Area Limit Exceeded")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
                .show(ctx, |ui| {
                    ui.set_min_width(380.0);
                    ui.label(
                        RichText::new(alert_msg)
                            .size(12.5)
                            .color(Color32::from_rgb(255, 205, 110)),
                    );
                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        if needs_rehome {
                            let rehome_btn = egui::Button::new(
                                RichText::new("🏠 Re-Home All Axes").color(Color32::WHITE).strong(),
                            )
                            .fill(Color32::from_rgb(28, 125, 78))
                            .min_size(egui::Vec2::new(140.0, 28.0));
                            if ui.add(rehome_btn).clicked() {
                                if let Some(ref c) = self.controller {
                                    c.home_all(panels::home_config_from_pulse(&self.pulse_config));
                                    self.status_message = format!(
                                        "Re-homing Z (up) and X/Y toward {}...",
                                        self.pulse_config.origin_corner
                                    );
                                }
                                self.panel_state.show_home_prompt = false;
                                dismiss = true;
                            }
                        }
                        let ok_label = if needs_rehome { "Dismiss" } else { "OK" };
                        let ok_btn = egui::Button::new(RichText::new(ok_label).size(12.0))
                            .fill(Color32::from_rgb(55, 60, 72))
                            .min_size(egui::Vec2::new(80.0, 28.0));
                        if ui.add(ok_btn).clicked() {
                            dismiss = true;
                        }
                    });
                });
            if dismiss {
                self.panel_state.oob_alert = None;
            }
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
        Box::new(|cc| {
            #[cfg(target_os = "windows")]
            configure_windows_theme_and_fonts(&cc.egui_ctx);
            #[cfg(not(target_os = "windows"))]
            configure_unix_fonts(&cc.egui_ctx);
            Ok(Box::new(TzlaserApp::default()))
        }),
    )
}
