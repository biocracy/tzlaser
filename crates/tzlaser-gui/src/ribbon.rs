use crate::canvas::{catmull_rom_spline, make_ellipse_entity, make_rectangle_entity, DrawTool};
use eframe::egui::{
    self, Button, Color32, DragValue, Frame, Margin, RichText, Stroke, Ui, Vec2,
};
use kurbo::Point;
use std::net::SocketAddr;
use tzlaser_core::{
    apply_lead_lines, Color, Document, DxfImporter, Entity, HistoryManager, Layer, LeadConfig,
    LeadType, PostJobAction, PulseConfig, SortMode,
};
use tzlaser_protocol::{JogConfig, LaserController, MachineStatus};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RibbonTab {
    File,
    Draw,
    Profiles,
    View,
    Laser,
    Properties,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PendingSortAction {
    SaveOverwriteOrder,
    ChangeSortMode(SortMode),
    ApplySort(SortMode),
    EnterManualSort,
    RestoreSavedOrder,
}

pub struct RibbonState {
    pub active_tab: RibbonTab,
    pub lead_config: LeadConfig,
    pub pulse_duration_ms: u32,
    pub pulse_power_pct: f32,
    pub follower_enabled: bool,
    pub red_pointer_enabled: bool,
    pub gas_open: bool,
    pub pending_sort_action: Option<PendingSortAction>,
    // Dimensioned shape placement fields for the Draw tab
    pub draw_place_x: f64,
    pub draw_place_y: f64,
    pub draw_rect_w: f64,
    pub draw_rect_h: f64,
    pub draw_circle_r: f64,
    pub draw_line_len: f64,
    pub draw_line_deg: f64,
    pub draw_ellipse_rx: f64,
    pub draw_ellipse_ry: f64,
}

impl Default for RibbonState {
    fn default() -> Self {
        Self {
            active_tab: RibbonTab::Profiles,
            lead_config: LeadConfig::default(),
            pulse_duration_ms: 50,
            pulse_power_pct: 20.0,
            follower_enabled: false,
            red_pointer_enabled: false,
            gas_open: false,
            pending_sort_action: None,
            draw_place_x: 150.0,
            draw_place_y: 150.0,
            draw_rect_w: 100.0,
            draw_rect_h: 50.0,
            draw_circle_r: 25.0,
            draw_line_len: 100.0,
            draw_line_deg: 0.0,
            draw_ellipse_rx: 40.0,
            draw_ellipse_ry: 25.0,
        }
    }
}

pub fn render_ribbon(
    ui: &mut Ui,
    state: &mut RibbonState,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    canvas_state: &mut crate::canvas::CanvasState,
    panel_state: &mut crate::panels::PanelState,
    pulse_config: &mut PulseConfig,
    props_window_state: &mut crate::properties_window::PropertiesWindowState,
    controller: &mut Option<LaserController>,
    status: &MachineStatus,
    status_message: &mut String,
) {
    // 1. Top Ribbon Tab Bar & Quick Access
    ui.horizontal(|ui| {
        // App Logo: Ţlaser (Romanian letter Ţ with cedilla)
        ui.horizontal(|ui| {
            ui.label(RichText::new("⚡ Ţlaser").size(17.0).color(Color32::from_rgb(255, 185, 45)).strong());
            ui.label(RichText::new("CNC").size(10.0).color(Color32::from_rgb(130, 140, 160)));
        });

        ui.add_space(6.0);

        // Quick Access Undo / Redo
        let disabled_icon_fg = if cfg!(target_os = "windows") {
            Color32::from_rgb(155, 165, 182)
        } else {
            Color32::from_rgb(100, 100, 110)
        };
        let quick_btn_bg = if cfg!(target_os = "windows") {
            Color32::from_rgb(42, 48, 58)
        } else {
            Color32::from_rgb(32, 34, 40)
        };
        let can_undo = history.can_undo();
        let undo_desc = history.undo_description().unwrap_or("Undo");
        let undo_btn = Button::new(RichText::new("↩").size(13.0).color(if can_undo { Color32::WHITE } else { disabled_icon_fg }))
            .fill(quick_btn_bg);
        if ui.add_enabled(can_undo, undo_btn).on_hover_text(format!("Undo {} (Cmd+Z)", undo_desc)).clicked() {
            if let Some(desc) = history.undo(doc, selection) {
                *status_message = format!("Undo: {}", desc);
            }
        }

        let can_redo = history.can_redo();
        let redo_desc = history.redo_description().unwrap_or("Redo");
        let redo_btn = Button::new(RichText::new("↪").size(13.0).color(if can_redo { Color32::WHITE } else { disabled_icon_fg }))
            .fill(quick_btn_bg);
        if ui.add_enabled(can_redo, redo_btn).on_hover_text(format!("Redo {} (Cmd+Shift+Z / Cmd+Y)", redo_desc)).clicked() {
            if let Some(desc) = history.redo(doc, selection) {
                *status_message = format!("Redo: {}", desc);
            }
        }

        ui.add_space(8.0);

        // Ribbon Tab Buttons
        let tabs = [
            (RibbonTab::File, "📁 File & Project"),
            (RibbonTab::Draw, "✏ Draw"),
            (RibbonTab::Profiles, "📐 Profile CAM"),
            (RibbonTab::View, "👁 View & Overlays"),
            (RibbonTab::Laser, "⚡ Laser & Hardware"),
            (RibbonTab::Properties, "⚙ Machine & Table"),
        ];

        for (tab, label) in tabs {
            let is_active = state.active_tab == tab;
            let bg = if is_active {
                Color32::from_rgb(45, 52, 68)
            } else if cfg!(target_os = "windows") {
                Color32::from_rgb(36, 41, 52)
            } else {
                Color32::from_rgb(26, 28, 34)
            };
            let text_color = if is_active {
                Color32::from_rgb(0, 225, 255)
            } else if cfg!(target_os = "windows") {
                Color32::from_rgb(218, 226, 240)
            } else {
                Color32::from_rgb(180, 185, 195)
            };

            let btn = Button::new(RichText::new(label).size(12.5).color(text_color).strong())
                .fill(bg)
                .stroke(if is_active { Stroke::new(1.0_f32, Color32::from_rgb(0, 180, 240)) } else { Stroke::NONE });

            if ui.add(btn).clicked() {
                state.active_tab = tab;
            }
        }

        // Right Aligned: Hardware Connection Badge
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if let Some(ref c) = controller {
                let (dot, text, color) = if c.is_connected() {
                    ("●", "ONLINE (10.1.1.168:502)", Color32::GREEN)
                } else {
                    ("○", "CONNECTING...", Color32::YELLOW)
                };

                if ui.add(Button::new(RichText::new("Disconnect").size(11.0).color(Color32::LIGHT_RED)).fill(Color32::from_rgb(45, 25, 25))).clicked() {
                    *controller = None;
                    *status_message = "Disconnected from machine controller.".to_string();
                }

                ui.label(RichText::new(format!("{} {}", dot, text)).size(11.5).color(color).strong());
            } else {
                let connect_btn = Button::new(RichText::new("🔌 Connect").size(11.5).color(Color32::WHITE))
                    .fill(Color32::from_rgb(30, 85, 150));
                if ui.add(connect_btn).clicked() {
                    if let Ok(addr) = panel_state.target_ip.parse::<SocketAddr>() {
                        let c = LaserController::new(addr);
                        let (min_speed, jog_speed, accel) = pulse_config.jog_pulses_params();
                        c.update_jog_config(
                            JogConfig {
                                min_speed,
                                jog_speed,
                                accel,
                            },
                            pulse_config.z_jog_speed_reg,
                        );
                        *controller = Some(c);
                        *status_message = format!("Connecting to controller board at {}...", addr);
                    } else {
                        *status_message = "Invalid IP address format".to_string();
                    }
                }
            }
        });
    });

    ui.add_space(3.0);

    // 2. Ribbon Content Area (Rendered directly without horizontal scrollbar)
    let ribbon_body_fill = if cfg!(target_os = "windows") {
        Color32::from_rgb(34, 39, 49)
    } else {
        Color32::from_rgb(28, 30, 36)
    };
    let ribbon_body_stroke = if cfg!(target_os = "windows") {
        Color32::from_rgb(64, 73, 92)
    } else {
        Color32::from_rgb(42, 45, 54)
    };
    Frame::new()
        .fill(ribbon_body_fill)
        .inner_margin(Margin::symmetric(6, 4))
        .stroke(Stroke::new(1.0_f32, ribbon_body_stroke))
        .show(ui, |ui| {
            match state.active_tab {
                RibbonTab::File => render_file_tab(ui, doc, selection, history, status_message),
                RibbonTab::Draw => render_draw_tab(ui, state, doc, selection, history, canvas_state, status_message),
                RibbonTab::Profiles => render_profiles_tab(ui, state, doc, selection, history, canvas_state, status_message),
                RibbonTab::View => render_view_tab(ui, doc, canvas_state),
                RibbonTab::Laser => render_laser_tab(ui, state, pulse_config, controller, status, status_message),
                RibbonTab::Properties => render_properties_tab(
                    ui,
                    pulse_config,
                    props_window_state,
                    canvas_state,
                    panel_state,
                    controller,
                    status_message,
                ),
            }
        });

    render_sort_overwrite_modal(ui.ctx(), state, doc, selection, history, canvas_state, status_message);
}

fn place_drawn_entity(
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    entity: Entity,
    label: &str,
    status_message: &mut String,
) {
    history.record(doc, selection, &format!("Draw {}", label));
    let layer_idx = doc
        .layers
        .iter()
        .position(|l| !l.is_travel && !l.is_obstacle)
        .unwrap_or_else(|| {
            doc.layers.push(Layer::new("Cut", Color { r: 255, g: 255, b: 255 }));
            doc.layers.len() - 1
        });
    let entity_idx = doc.layers[layer_idx].entities.len();
    doc.layers[layer_idx].entities.push(entity);
    selection.clear();
    selection.push((layer_idx, entity_idx));
    *status_message = format!("Placed {} on '{}'", label, doc.layers[layer_idx].name);
}

fn render_draw_tab(
    ui: &mut Ui,
    state: &mut RibbonState,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    canvas_state: &mut crate::canvas::CanvasState,
    status_message: &mut String,
) {
    let active_bg = Color32::from_rgb(35, 95, 130);
    let inactive_bg = if cfg!(target_os = "windows") {
        Color32::from_rgb(44, 50, 62)
    } else {
        Color32::from_rgb(36, 40, 48)
    };

    ribbon_group(ui, "Shapes", |ui| {
        let tools = [
            (DrawTool::None, "🖱 Select", "Return to selection / pan mode (Esc)"),
            (DrawTool::Rectangle, "⬛ Rectangle", "Draw rectangle by 2 corners on canvas or place by dimensions"),
            (DrawTool::Circle, "⚪ Circle", "Draw circle (center + radius point) or place by radius"),
            (DrawTool::Line, "📏 Line", "Draw line segment (2 points) or place by length & angle"),
            (DrawTool::Point, "• Point", "Click canvas or Place to insert a point marker"),
            (DrawTool::Ellipse, "⬭ Ellipse", "Draw ellipse (center + corner) or place by Rx / Ry"),
            (DrawTool::Spline, "〰 Spline", "Click control points on canvas; double-click, right-click, or Enter to finish"),
        ];

        for (tool, label, tip) in tools {
            let is_active = canvas_state.draw_tool == tool;
            let btn = Button::new(RichText::new(label).size(11.5).color(if is_active {
                Color32::WHITE
            } else {
                Color32::from_rgb(215, 222, 235)
            }))
            .fill(if is_active { active_bg } else { inactive_bg })
            .min_size(Vec2::new(72.0, 26.0));
            if ui.add(btn).on_hover_text(tip).clicked() {
                canvas_state.draw_tool = tool;
                canvas_state.draw_anchor = None;
                canvas_state.draw_spline_pts.clear();
                *status_message = match tool {
                    DrawTool::None => "Selection tool active".to_string(),
                    DrawTool::Point => "Click on canvas to place a Point (or use Place)".to_string(),
                    DrawTool::Spline => "Click control points on canvas; Enter / double-click / right-click to finish".to_string(),
                    _ => format!("{:?} tool active: click/drag on canvas or enter dimensions and click Place", tool),
                };
            }
        }
    });

    ui.add_space(4.0);

    ribbon_group(ui, "Dimensioned Place", |ui| {
        ui.label(RichText::new("X:").size(11.0));
        ui.add(DragValue::new(&mut state.draw_place_x).speed(1.0).max_decimals(2).suffix(" mm"));
        ui.label(RichText::new("Y:").size(11.0));
        ui.add(DragValue::new(&mut state.draw_place_y).speed(1.0).max_decimals(2).suffix(" mm"));
        ui.separator();

        match canvas_state.draw_tool {
            DrawTool::None => {
                ui.label(RichText::new("Select a shape tool to draw or place by exact dimensions.").size(11.0).weak());
            }
            DrawTool::Rectangle => {
                ui.label(RichText::new("W:").size(11.0));
                ui.add(DragValue::new(&mut state.draw_rect_w).range(0.1..=10000.0).speed(1.0).max_decimals(2).suffix(" mm"));
                ui.label(RichText::new("H:").size(11.0));
                ui.add(DragValue::new(&mut state.draw_rect_h).range(0.1..=10000.0).speed(1.0).max_decimals(2).suffix(" mm"));
                if ui.button(RichText::new("📍 Place").strong()).clicked() {
                    let a = Point { x: state.draw_place_x, y: state.draw_place_y };
                    let b = Point { x: state.draw_place_x + state.draw_rect_w, y: state.draw_place_y + state.draw_rect_h };
                    place_drawn_entity(doc, selection, history, make_rectangle_entity(a, b), "Rectangle", status_message);
                }
            }
            DrawTool::Circle => {
                ui.label(RichText::new("R:").size(11.0));
                ui.add(DragValue::new(&mut state.draw_circle_r).range(0.1..=5000.0).speed(0.5).max_decimals(2).suffix(" mm"));
                if ui.button(RichText::new("📍 Place").strong()).clicked() {
                    let ent = Entity::Circle {
                        center: Point { x: state.draw_place_x, y: state.draw_place_y },
                        radius: state.draw_circle_r.max(0.05),
                    };
                    place_drawn_entity(doc, selection, history, ent, "Circle", status_message);
                }
            }
            DrawTool::Line => {
                ui.label(RichText::new("L:").size(11.0));
                ui.add(DragValue::new(&mut state.draw_line_len).range(0.1..=10000.0).speed(1.0).max_decimals(2).suffix(" mm"));
                ui.label(RichText::new("∠:").size(11.0));
                ui.add(DragValue::new(&mut state.draw_line_deg).range(-360.0..=360.0).speed(1.0).max_decimals(1).suffix("°"));
                if ui.button(RichText::new("📍 Place").strong()).clicked() {
                    let rad = state.draw_line_deg.to_radians();
                    let start = Point { x: state.draw_place_x, y: state.draw_place_y };
                    let end = Point {
                        x: state.draw_place_x + state.draw_line_len * rad.cos(),
                        y: state.draw_place_y + state.draw_line_len * rad.sin(),
                    };
                    place_drawn_entity(doc, selection, history, Entity::Line { start, end }, "Line", status_message);
                }
            }
            DrawTool::Point => {
                if ui.button(RichText::new("📍 Place Point").strong()).clicked() {
                    let p = Point { x: state.draw_place_x, y: state.draw_place_y };
                    place_drawn_entity(doc, selection, history, Entity::Line { start: p, end: p }, "Point", status_message);
                }
            }
            DrawTool::Ellipse => {
                ui.label(RichText::new("Rx:").size(11.0));
                ui.add(DragValue::new(&mut state.draw_ellipse_rx).range(0.1..=5000.0).speed(0.5).max_decimals(2).suffix(" mm"));
                ui.label(RichText::new("Ry:").size(11.0));
                ui.add(DragValue::new(&mut state.draw_ellipse_ry).range(0.1..=5000.0).speed(0.5).max_decimals(2).suffix(" mm"));
                if ui.button(RichText::new("📍 Place").strong()).clicked() {
                    let ent = make_ellipse_entity(
                        Point { x: state.draw_place_x, y: state.draw_place_y },
                        state.draw_ellipse_rx.max(0.05),
                        state.draw_ellipse_ry.max(0.05),
                    );
                    place_drawn_entity(doc, selection, history, ent, "Ellipse", status_message);
                }
            }
            DrawTool::Spline => {
                ui.checkbox(&mut canvas_state.draw_spline_closed, "Closed");
                let can_finish = canvas_state.draw_spline_pts.len() >= 2;
                if ui.add_enabled(can_finish, Button::new(RichText::new("✓ Finish").strong())).clicked() {
                    let pts = catmull_rom_spline(&canvas_state.draw_spline_pts, canvas_state.draw_spline_closed);
                    let ent = Entity::Polyline { points: pts, closed: canvas_state.draw_spline_closed };
                    canvas_state.draw_spline_pts.clear();
                    place_drawn_entity(doc, selection, history, ent, "Spline", status_message);
                }
                if ui.add_enabled(!canvas_state.draw_spline_pts.is_empty(), Button::new("✕ Clear")).clicked() {
                    canvas_state.draw_spline_pts.clear();
                }
            }
        }
    });
}

fn execute_sort_action(
    action: PendingSortAction,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    canvas_state: &mut crate::canvas::CanvasState,
    status_message: &mut String,
) {
    canvas_state.push_sort_undo(doc);
    match action {
        PendingSortAction::SaveOverwriteOrder => {
            let label = if doc.has_saved_cut_order() {
                "Overwrite Saved Cut Order"
            } else {
                "Save Cut Order"
            };
            history.record(doc, selection, label);
            doc.save_current_cut_order();
            *status_message = format!("Saved cutting order ({} profiles).", doc.saved_cut_order.len());
        }
        PendingSortAction::ChangeSortMode(new_mode) => {
            let had_custom = doc.use_saved_cut_order || !doc.manual_sequence.is_empty();
            if had_custom {
                history.record(doc, selection, format!("Sort Method ({})", new_mode.label()));
                doc.use_saved_cut_order = false;
                doc.manual_sequence.clear();
            }
            doc.sort_mode = new_mode;
            if new_mode != SortMode::Manual {
                canvas_state.manual_sort_active = false;
            }
            *status_message = format!("Sort method set to: {}", new_mode.label());
        }
        PendingSortAction::ApplySort(mode) => {
            history.record(doc, selection, format!("Sort ({})", mode.label()));
            doc.apply_sort_order(mode);
            let seq = doc.compute_cut_sequence();
            *status_message = format!("Sorted {} cut paths using '{}'.", seq.len(), mode.label());
        }
        PendingSortAction::EnterManualSort => {
            if doc.use_saved_cut_order || !doc.manual_sequence.is_empty() {
                history.record(doc, selection, "Enter Manual Sort");
                doc.use_saved_cut_order = false;
                doc.manual_sequence.clear();
            }
            canvas_state.manual_sort_active = true;
            canvas_state.manual_sort_next_index = 1;
            canvas_state.show_cut_order = true;
            selection.clear();
            *status_message = "Manual Sort active: click paths on canvas in desired cut order.".to_string();
        }
        PendingSortAction::RestoreSavedOrder => {
            history.record(doc, selection, "Restore Saved Cut Order");
            doc.restore_saved_cut_order();
            canvas_state.manual_sort_active = false;
            canvas_state.show_cut_order = true;
            *status_message = format!("Restored saved cutting order ({} profiles).", doc.saved_cut_order.len());
        }
    }
}

fn render_sort_overwrite_modal(
    ctx: &egui::Context,
    state: &mut RibbonState,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    canvas_state: &mut crate::canvas::CanvasState,
    status_message: &mut String,
) {
    let Some(pending) = state.pending_sort_action.clone() else {
        return;
    };

    let prompt_body = match &pending {
        PendingSortAction::SaveOverwriteOrder => {
            "A saved profile cutting order already exists for this table layout.\n\nDo you want to overwrite the saved cutting order with the current sequence?".to_string()
        }
        PendingSortAction::ChangeSortMode(mode) => format!(
            "Changing sort method to '{}' will overwrite the active custom cutting order.\n\nDo you want to continue?",
            mode.label()
        ),
        PendingSortAction::ApplySort(mode) => format!(
            "Applying '{}' will overwrite the existing custom/saved profile cutting order.\n\nDo you want to continue?",
            mode.label()
        ),
        PendingSortAction::EnterManualSort => {
            "Starting interactive Manual Sort will overwrite the current custom/saved profile cutting order.\n\nDo you want to continue?".to_string()
        }
        PendingSortAction::RestoreSavedOrder => {
            "Restoring the saved cutting order will overwrite the current profile sorting on the table.\n\nDo you want to continue?".to_string()
        }
    };

    egui::Window::new("⚠ Overwrite Profile Cutting Order?")
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO)
        .show(ctx, |ui| {
            ui.set_min_width(340.0);
            ui.label(RichText::new(prompt_body).size(12.5));
            ui.add_space(10.0);
            ui.separator();
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                let confirm_btn = Button::new(RichText::new("✓ Overwrite").color(Color32::WHITE).strong())
                    .fill(Color32::from_rgb(175, 55, 45))
                    .min_size(Vec2::new(110.0, 26.0));
                if ui.add(confirm_btn).clicked() {
                    execute_sort_action(pending, doc, selection, history, canvas_state, status_message);
                    state.pending_sort_action = None;
                }

                ui.add_space(8.0);

                let cancel_btn = Button::new(RichText::new("✕ Cancel").size(12.0))
                    .fill(Color32::from_rgb(48, 52, 62))
                    .min_size(Vec2::new(90.0, 26.0));
                if ui.add(cancel_btn).clicked() {
                    state.pending_sort_action = None;
                }
            });
        });
}

fn ribbon_group(ui: &mut Ui, title: &str, content: impl FnOnce(&mut Ui)) {
    let title_color = if cfg!(target_os = "windows") {
        Color32::from_rgb(182, 194, 214)
    } else {
        Color32::from_rgb(130, 135, 150)
    };
    ui.group(|ui| {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                content(ui);
            });
            ui.add_space(2.0);
            ui.label(RichText::new(title).size(9.0).color(title_color).strong());
        });
    });
}

fn render_file_tab(
    ui: &mut Ui,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    status_message: &mut String,
) {
    ui.horizontal(|ui| {
        ribbon_group(ui, "Project & Files", |ui| {
            let open_btn = Button::new(RichText::new("📂 Open DXF").size(12.0).strong())
                .fill(Color32::from_rgb(35, 65, 95));
            if ui.add(open_btn).on_hover_text("Open AutoCAD DXF file (2D lines, arcs, circles, polylines, splines)").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("CAD Files", &["dxf", "DXF"])
                    .pick_file()
                {
                    match DxfImporter::load_file(&path) {
                        Ok(imported_doc) => {
                            history.record(doc, selection, "Open DXF");
                            *status_message = format!(
                                "Loaded DXF: {} ({} profiles across {} layers)",
                                path.file_name().unwrap_or_default().to_string_lossy(),
                                imported_doc.entity_count(),
                                imported_doc.layers.len()
                            );
                            *doc = imported_doc;
                            selection.clear();
                        }
                        Err(e) => {
                            *status_message = format!("DXF Load error: {}", e);
                        }
                    }
                }
            }

            let save_btn = Button::new(RichText::new("💾 Save Job").size(12.0))
                .fill(Color32::from_rgb(40, 45, 55));
            if ui.add(save_btn).on_hover_text("Save job layout (*.tzjob)").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("Ţlaser Job", &["tzjob", "json"])
                    .set_file_name("job.tzjob")
                    .save_file()
                {
                    match serde_json::to_string_pretty(&doc) {
                        Ok(json) => {
                            if std::fs::write(&path, json).is_ok() {
                                *status_message = format!("Saved job to {}", path.display());
                            }
                        }
                        Err(e) => *status_message = format!("Save error: {}", e),
                    }
                }
            }

            let clear_btn = Button::new(RichText::new("📄 New Job").size(12.0).color(Color32::from_rgb(255, 140, 140)))
                .fill(Color32::from_rgb(48, 35, 38));
            if ui.add(clear_btn).on_hover_text("Clear all profiles and reset table").clicked() {
                history.record(doc, selection, "New Job");
                *doc = Document::new();
                selection.clear();
                *status_message = "New job created. Workspace cleared.".to_string();
            }
        });

        ui.add_space(4.0);

        ribbon_group(ui, "History", |ui| {
            let can_undo = history.can_undo();
            let undo_label = format!("↩ Undo\n{}", history.undo_description().unwrap_or(""));
            let undo_btn = Button::new(RichText::new(undo_label).size(11.0))
                .fill(Color32::from_rgb(40, 45, 55));
            if ui.add_enabled(can_undo, undo_btn).on_hover_text("Undo last modification (Cmd+Z)").clicked() {
                if let Some(desc) = history.undo(doc, selection) {
                    *status_message = format!("Undo: {}", desc);
                }
            }

            let can_redo = history.can_redo();
            let redo_label = format!("↪ Redo\n{}", history.redo_description().unwrap_or(""));
            let redo_btn = Button::new(RichText::new(redo_label).size(11.0))
                .fill(Color32::from_rgb(40, 45, 55));
            if ui.add_enabled(can_redo, redo_btn).on_hover_text("Redo action (Cmd+Shift+Z / Cmd+Y)").clicked() {
                if let Some(desc) = history.redo(doc, selection) {
                    *status_message = format!("Redo: {}", desc);
                }
            }
        });
    });
}

fn render_view_tab(
    ui: &mut Ui,
    doc: &mut Document,
    canvas_state: &mut crate::canvas::CanvasState,
) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing = Vec2::new(6.0, 2.0);

        ribbon_group(ui, "Viewport", |ui| {
            let fit_btn = Button::new(RichText::new("🔍 Fit View").size(12.0).strong())
                .fill(Color32::from_rgb(35, 65, 95))
                .min_size(Vec2::new(88.0, 36.0));
            if ui.add(fit_btn).on_hover_text("Frame all objects on the cutting bed").clicked() {
                canvas_state.fit_view_to_doc(doc);
            }

            let reset_zoom_btn = Button::new(RichText::new("1:1 Zoom").size(12.0))
                .fill(Color32::from_rgb(40, 45, 55))
                .min_size(Vec2::new(72.0, 36.0));
            if ui.add(reset_zoom_btn).on_hover_text("Reset viewport zoom to 1:1 (1 px/mm)").clicked() {
                canvas_state.zoom = 1.0;
            }
        });

        ribbon_group(ui, "Toolpath Overlays", |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    let order_active = canvas_state.show_cut_order;
                    let (order_text, order_color, order_bg) = if order_active {
                        ("🔢 Index: ON", Color32::from_rgb(0, 230, 255), Color32::from_rgb(20, 65, 95))
                    } else {
                        ("🔢 Index: OFF", Color32::from_rgb(160, 165, 175), Color32::from_rgb(36, 40, 48))
                    };
                    let btn_order = Button::new(RichText::new(order_text).size(11.0).color(order_color).strong())
                        .fill(order_bg)
                        .min_size(Vec2::new(102.0, 19.0));
                    if ui.add(btn_order).on_hover_text("Toggle cut sequence numbers (1, 2, 3...) and green pierce start markers").clicked() {
                        canvas_state.show_cut_order = !canvas_state.show_cut_order;
                    }

                    let travel_active = doc.layers.iter().find(|l| l.is_travel).map_or(true, |l| l.visible);
                    let (travel_text, travel_color, travel_bg) = if travel_active {
                        ("〰 Travel: ON", Color32::from_rgb(255, 175, 85), Color32::from_rgb(80, 52, 22))
                    } else {
                        ("〰 Travel: OFF", Color32::from_rgb(160, 165, 175), Color32::from_rgb(36, 40, 48))
                    };
                    let btn_travel = Button::new(RichText::new(travel_text).size(11.0).color(travel_color).strong())
                        .fill(travel_bg)
                        .min_size(Vec2::new(102.0, 19.0));
                    if ui
                        .add(btn_travel)
                        .on_hover_text("Toggle rapid travel paths (synced with the Travel layer visibility checkbox)")
                        .clicked()
                    {
                        let new_vis = !travel_active;
                        for layer in &mut doc.layers {
                            if layer.is_travel {
                                layer.visible = new_vis;
                            }
                        }
                    }
                });

                let dir_active = canvas_state.show_cut_direction;
                let (dir_text, dir_color, dir_bg) = if dir_active {
                    ("🧭 Direction: ON", Color32::from_rgb(0, 255, 120), Color32::from_rgb(20, 65, 45))
                } else {
                    ("🧭 Direction: OFF", Color32::from_rgb(160, 165, 175), Color32::from_rgb(36, 40, 48))
                };
                let btn_dir = Button::new(RichText::new(dir_text).size(11.0).color(dir_color).strong())
                    .fill(dir_bg)
                    .min_size(Vec2::new(102.0, 19.0));
                if ui.add(btn_dir).on_hover_text("Toggle cut direction arrowheads along profile toolpaths").clicked() {
                    canvas_state.show_cut_direction = !canvas_state.show_cut_direction;
                }
            });
        });

        ribbon_group(ui, "Bed & Guides", |ui| {
            ui.vertical(|ui| {
                let (grid_text, grid_color, grid_bg) = if canvas_state.show_grid {
                    ("⊞ Grid: ON", Color32::from_rgb(120, 210, 255), Color32::from_rgb(24, 58, 82))
                } else {
                    ("⊞ Grid: OFF", Color32::from_rgb(160, 165, 175), Color32::from_rgb(36, 40, 48))
                };
                if ui
                    .add(Button::new(RichText::new(grid_text).size(11.0).color(grid_color).strong()).fill(grid_bg).min_size(Vec2::new(102.0, 19.0)))
                    .on_hover_text("Toggle 100 mm machine bed grid")
                    .clicked()
                {
                    canvas_state.show_grid = !canvas_state.show_grid;
                }

                let (ruler_text, ruler_color, ruler_bg) = if canvas_state.show_rulers {
                    ("📏 Ruler: ON", Color32::from_rgb(255, 220, 110), Color32::from_rgb(72, 58, 22))
                } else {
                    ("📏 Ruler: OFF", Color32::from_rgb(160, 165, 175), Color32::from_rgb(36, 40, 48))
                };
                if ui
                    .add(Button::new(RichText::new(ruler_text).size(11.0).color(ruler_color).strong()).fill(ruler_bg).min_size(Vec2::new(102.0, 19.0)))
                    .on_hover_text("Toggle millimeter coordinate rulers along the top and left canvas edges")
                    .clicked()
                {
                    canvas_state.show_rulers = !canvas_state.show_rulers;
                }

                let (ret_text, ret_color, ret_bg) = if canvas_state.show_retraction_zone {
                    ("🛡 Retract: ON", Color32::from_rgb(245, 130, 120), Color32::from_rgb(78, 28, 28))
                } else {
                    ("🛡 Retract: OFF", Color32::from_rgb(160, 165, 175), Color32::from_rgb(36, 40, 48))
                };
                if ui
                    .add(Button::new(RichText::new(ret_text).size(11.0).color(ret_color).strong()).fill(ret_bg).min_size(Vec2::new(102.0, 19.0)))
                    .on_hover_text("Toggle homing retraction exclusion zone overlay in negative coordinates")
                    .clicked()
                {
                    canvas_state.show_retraction_zone = !canvas_state.show_retraction_zone;
                }
            });
        });

        ribbon_group(ui, "Cut Order Coloring", |ui| {
            ui.vertical(|ui| {
                let (col_text, col_fg, col_bg) = if canvas_state.color_by_cut_index {
                    ("🎨 Color: ON", Color32::from_rgb(0, 255, 180), Color32::from_rgb(20, 68, 60))
                } else {
                    ("🎨 Color: OFF", Color32::from_rgb(170, 175, 185), Color32::from_rgb(38, 42, 50))
                };
                if ui.add(Button::new(RichText::new(col_text).size(11.0).color(col_fg).strong()).fill(col_bg).min_size(Vec2::new(105.0, 19.0)))
                    .on_hover_text("Color toolpaths according to their cut index using the selected palette")
                    .clicked()
                {
                    canvas_state.color_by_cut_index = !canvas_state.color_by_cut_index;
                }

                egui::ComboBox::from_id_salt("cut_color_palette_combo")
                    .selected_text(RichText::new(canvas_state.color_palette.label()).size(11.0))
                    .width(105.0)
                    .show_ui(ui, |ui| {
                        use crate::canvas::ColorPalette;
                        ui.selectable_value(&mut canvas_state.color_palette, ColorPalette::Rainbow, "🌈 Rainbow");
                        ui.selectable_value(&mut canvas_state.color_palette, ColorPalette::Thermal, "🔥 Thermal");
                        ui.selectable_value(&mut canvas_state.color_palette, ColorPalette::HighContrast, "🎯 High Contrast");
                        ui.selectable_value(&mut canvas_state.color_palette, ColorPalette::OceanDepth, "🌊 Ocean");
                        ui.selectable_value(&mut canvas_state.color_palette, ColorPalette::MonochromeFade, "💎 Cyan Fade");
                    });
            });
        });
    });
}

fn render_profiles_tab(
    ui: &mut Ui,
    state: &mut RibbonState,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    canvas_state: &mut crate::canvas::CanvasState,
    status_message: &mut String,
) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing = Vec2::new(4.0, 2.0);

        // 1. Selection Group (Compact 2x2 grid)
        ribbon_group(ui, "Selection", |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    if ui.add(Button::new(RichText::new("⬚ All").size(11.0)).fill(Color32::from_rgb(40, 45, 55)).min_size(Vec2::new(48.0, 19.0))).on_hover_text("Select all profiles on visible layers").clicked() {
                        selection.clear();
                        for (l_idx, layer) in doc.layers.iter().enumerate() {
                            if !layer.visible { continue; }
                            for e_idx in 0..layer.entities.len() {
                                selection.push((l_idx, e_idx));
                            }
                        }
                    }

                    if ui.add(Button::new(RichText::new("🔄 Inv").size(11.0)).fill(Color32::from_rgb(40, 45, 55)).min_size(Vec2::new(48.0, 19.0))).on_hover_text("Invert selection").clicked() {
                        let mut new_sel = Vec::new();
                        for (l_idx, layer) in doc.layers.iter().enumerate() {
                            if !layer.visible { continue; }
                            for e_idx in 0..layer.entities.len() {
                                if !selection.contains(&(l_idx, e_idx)) {
                                    new_sel.push((l_idx, e_idx));
                                }
                            }
                        }
                        *selection = new_sel;
                    }
                });

                ui.horizontal(|ui| {
                    let has_sel = !selection.is_empty();
                    if ui.add_enabled(has_sel, Button::new(RichText::new("✕ Clear").size(11.0)).fill(Color32::from_rgb(45, 40, 42)).min_size(Vec2::new(48.0, 19.0))).clicked() {
                        selection.clear();
                    }
                    if ui.add_enabled(has_sel, Button::new(RichText::new("🗑 Del").size(11.0).color(if has_sel { Color32::from_rgb(255, 90, 90) } else { Color32::GRAY })).fill(Color32::from_rgb(52, 30, 32)).min_size(Vec2::new(48.0, 19.0))).clicked() {
                        history.record(doc, selection, "Delete Selection");
                        doc.delete_selection(selection);
                        selection.clear();
                    }
                });
            });
        });

        ui.add_space(2.0);

        // 2. Transforms (Mirror)
        ribbon_group(ui, "Transform", |ui| {
            let has_sel = !selection.is_empty();
            ui.vertical(|ui| {
                if ui.add_enabled(has_sel, Button::new(RichText::new("🪞 Flip X").size(11.0)).fill(Color32::from_rgb(38, 42, 50)).min_size(Vec2::new(70.0, 19.0))).on_hover_text("Mirror selection horizontally").clicked() {
                    history.record(doc, selection, "Mirror X");
                    doc.mirror_selection_horizontal(selection);
                }
                if ui.add_enabled(has_sel, Button::new(RichText::new("🪞 Flip Y").size(11.0)).fill(Color32::from_rgb(38, 42, 50)).min_size(Vec2::new(70.0, 19.0))).on_hover_text("Mirror selection vertically").clicked() {
                    history.record(doc, selection, "Mirror Y");
                    doc.mirror_selection_vertical(selection);
                }
            });
        });

        ui.add_space(2.0);

        // 3. Layer Auto-Classification (Large prominent button)
        ribbon_group(ui, "Classification", |ui| {
            let classify_btn = Button::new(
                RichText::new("⚡ Auto-Classify\nOuter / Inner / Bend")
                    .size(11.5)
                    .color(Color32::from_rgb(100, 220, 255))
                    .strong(),
            )
            .fill(Color32::from_rgb(25, 48, 70))
            .min_size(Vec2::new(115.0, 41.0));
            if ui.add(classify_btn).on_hover_text("Auto-segregate Outer Profiles, Interior Holes, and Bend Lines into dedicated cutting layers").clicked() {
                history.record(doc, selection, "Auto-Classify");
                doc.auto_classify_profiles();
                selection.clear();
                *status_message = format!("Auto-classified {} layers: {}", doc.layers.len(), doc.layers.iter().map(|l| l.name.as_str()).collect::<Vec<_>>().join(", "));
            }
        });

        ui.add_space(2.0);

        // 4. Lead Lines Group (2 stacked rows)
        ribbon_group(ui, "Lead Lines", |ui| {
            ui.vertical(|ui| {
                let lead_btn = Button::new(RichText::new("📍 Add Leads").size(11.0).color(Color32::from_rgb(255, 215, 0)))
                    .fill(Color32::from_rgb(45, 40, 25))
                    .min_size(Vec2::new(95.0, 19.0));
                if ui.add(lead_btn).on_hover_text(format!("Attach pierce lead-in lines ({:.1} mm at {:.0}°) to contours", state.lead_config.length_mm, state.lead_config.angle_deg)).clicked() {
                    history.record(doc, selection, "Add Lead Lines");
                    let count = apply_lead_lines(doc, selection, &state.lead_config);
                    *status_message = format!("Added lead-in lines to {} contours.", count);
                }

                ui.menu_button(RichText::new(format!("⚙ {:.1}mm / {:.0}°", state.lead_config.length_mm, state.lead_config.angle_deg)).size(10.0), |ui| {
                    ui.label(RichText::new("Lead-In Parameters").strong());
                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.label("Length:");
                        ui.add(egui::DragValue::new(&mut state.lead_config.length_mm).speed(0.2).range(0.5..=20.0).suffix(" mm"));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Angle:");
                        ui.add(egui::DragValue::new(&mut state.lead_config.angle_deg).speed(5.0).range(15.0..=90.0).suffix("°"));
                    });
                    ui.horizontal(|ui| {
                        ui.radio_value(&mut state.lead_config.lead_type, LeadType::Line, "Straight");
                        ui.radio_value(&mut state.lead_config.lead_type, LeadType::Arc, "Arc");
                    });
                });
            });
        });

        ui.add_space(2.0);

        // 5. Toolpath Sorting (Sort Method + Interactive Manual Sort + Save/Restore)
        ribbon_group(ui, "Toolpath Sorting", |ui| {
            ui.vertical(|ui| {
                // Row 1: Sort Method
                ui.horizontal(|ui| {
                    let mut current_mode = doc.sort_mode;
                    egui::ComboBox::from_id_salt("cut_sort_mode_combo")
                        .selected_text(RichText::new(current_mode.label()).size(11.0).strong().color(Color32::from_rgb(255, 220, 100)))
                        .width(165.0)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut current_mode, SortMode::OpenFirst, "⚡ Open / Bend First");
                            ui.selectable_value(&mut current_mode, SortMode::InnerFirst, "🔘 Inner Holes First");
                            ui.selectable_value(&mut current_mode, SortMode::MaxDistanceInterior, "❄ Max Dist Interior (Cool)");
                            ui.selectable_value(&mut current_mode, SortMode::ShortestPath, "🚀 Shortest Travel (TSP)");
                            ui.selectable_value(&mut current_mode, SortMode::LeftToRight, "➡️ Left to Right (X)");
                            ui.selectable_value(&mut current_mode, SortMode::TopToBottom, "⬇️ Top to Bottom (Y)");
                            ui.selectable_value(&mut current_mode, SortMode::Manual, "📝 Manual / Document Order");
                        });

                    if current_mode != doc.sort_mode {
                        if doc.use_saved_cut_order || !doc.manual_sequence.is_empty() {
                            state.pending_sort_action = Some(PendingSortAction::ChangeSortMode(current_mode));
                        } else {
                            execute_sort_action(
                                PendingSortAction::ChangeSortMode(current_mode),
                                doc,
                                selection,
                                history,
                                canvas_state,
                                status_message,
                            );
                        }
                    }
                });

                // Row 2: Sort Action / Interactive Manual Sort + Save/Restore Order
                ui.horizontal(|ui| {
                    if doc.sort_mode == SortMode::Manual {
                        if canvas_state.manual_sort_active {
                            let exit_btn = Button::new(RichText::new("✓ Exit Manual Sort").size(11.0).color(Color32::from_rgb(100, 255, 150)).strong())
                                .fill(Color32::from_rgb(25, 70, 42))
                                .min_size(Vec2::new(112.0, 19.0));
                            if ui.add(exit_btn).on_hover_text("Exit interactive manual sorting and return to normal selection").clicked() {
                                canvas_state.manual_sort_active = false;
                                *status_message = "Exited manual sort mode.".to_string();
                            }

                            ui.label(RichText::new("Next Index:").size(11.0).color(Color32::from_rgb(255, 220, 100)).strong());
                            let total_paths = doc.compute_cut_sequence().len().max(1);
                            ui.add(
                                egui::DragValue::new(&mut canvas_state.manual_sort_next_index)
                                    .range(1..=total_paths)
                                    .speed(0.2),
                            )
                            .on_hover_text("Cut index to assign to the next clicked path (editable; auto-increments on click)");
                        } else {
                            let sort_btn = Button::new(RichText::new("⚡ Sort").size(11.0).color(Color32::from_rgb(255, 215, 0)).strong())
                                .fill(Color32::from_rgb(45, 40, 22))
                                .min_size(Vec2::new(60.0, 19.0));
                            if ui.add(sort_btn).on_hover_text("Enter interactive Manual Sort mode: click paths in order to assign cut indices").clicked() {
                                if doc.use_saved_cut_order || !doc.manual_sequence.is_empty() {
                                    state.pending_sort_action = Some(PendingSortAction::EnterManualSort);
                                } else {
                                    execute_sort_action(
                                        PendingSortAction::EnterManualSort,
                                        doc,
                                        selection,
                                        history,
                                        canvas_state,
                                        status_message,
                                    );
                                }
                            }
                        }
                    } else {
                        let sort_btn = Button::new(RichText::new("⚡ Sort").size(11.0).color(Color32::from_rgb(255, 215, 0)).strong())
                            .fill(Color32::from_rgb(45, 40, 22))
                            .min_size(Vec2::new(60.0, 19.0));
                        if ui.add(sort_btn).on_hover_text(format!("Apply '{}' sequence to document toolpaths: {}", doc.sort_mode.label(), doc.sort_mode.description())).clicked() {
                            if doc.has_saved_cut_order() || !doc.manual_sequence.is_empty() {
                                state.pending_sort_action = Some(PendingSortAction::ApplySort(doc.sort_mode));
                            } else {
                                execute_sort_action(
                                    PendingSortAction::ApplySort(doc.sort_mode),
                                    doc,
                                    selection,
                                    history,
                                    canvas_state,
                                    status_message,
                                );
                            }
                        }
                    }

                    // Save / Restore Profile Cutting Order
                    let save_order_btn = Button::new(RichText::new("💾 Save Order").size(11.0).color(Color32::from_rgb(130, 215, 255)).strong())
                        .fill(Color32::from_rgb(28, 48, 68))
                        .min_size(Vec2::new(86.0, 19.0));
                    if ui.add(save_order_btn).on_hover_text("Save the current profile cutting order for this table layout").clicked() {
                        if doc.has_saved_cut_order() {
                            state.pending_sort_action = Some(PendingSortAction::SaveOverwriteOrder);
                        } else {
                            execute_sort_action(
                                PendingSortAction::SaveOverwriteOrder,
                                doc,
                                selection,
                                history,
                                canvas_state,
                                status_message,
                            );
                        }
                    }

                    if doc.has_saved_cut_order() {
                        if doc.use_saved_cut_order {
                            ui.label(RichText::new("🔒 Saved").size(11.0).color(Color32::from_rgb(100, 235, 150)).strong())
                                .on_hover_text("Saved cutting order is currently active and locked");
                        } else {
                            let restore_btn = Button::new(RichText::new("📂 Restore").size(11.0).color(Color32::from_rgb(180, 235, 180)))
                                .fill(Color32::from_rgb(30, 58, 42))
                                .min_size(Vec2::new(68.0, 19.0));
                            if ui.add(restore_btn).on_hover_text("Restore previously saved cutting order for this table layout").clicked() {
                                state.pending_sort_action = Some(PendingSortAction::RestoreSavedOrder);
                            }
                        }
                    }

                    let can_undo_sort = !canvas_state.sort_undo_stack.is_empty();
                    let undo_sort_btn = Button::new(
                        RichText::new("↩ Undo Index")
                            .size(11.0)
                            .color(if can_undo_sort {
                                Color32::from_rgb(255, 210, 120)
                            } else {
                                Color32::from_rgb(115, 120, 130)
                            })
                            .strong(),
                    )
                    .fill(Color32::from_rgb(42, 38, 30))
                    .min_size(Vec2::new(84.0, 19.0));
                    if ui
                        .add_enabled(can_undo_sort, undo_sort_btn)
                        .on_hover_text("Undo only the last cut-index / sorting change (does not affect geometry or positions)")
                        .clicked()
                    {
                        if canvas_state.pop_sort_undo(doc) {
                            *status_message = "Undid last cut-index change.".to_string();
                        }
                    }
                });
            });
        });
    });
}

fn render_laser_tab(
    ui: &mut Ui,
    state: &mut RibbonState,
    pulse_config: &mut PulseConfig,
    controller: &mut Option<LaserController>,
    status: &MachineStatus,
    status_message: &mut String,
) {
    ui.horizontal(|ui| {
        // 1. Head Calibration (Capacitive Height Sensor)
        ribbon_group(ui, "Head Calibration (Capacitive)", |ui| {
            let cal_btn = Button::new(RichText::new("🎯 Calibrate Sensor").size(12.0).color(Color32::from_rgb(255, 215, 0)).strong())
                .fill(Color32::from_rgb(50, 42, 20));
            if ui.add(cal_btn).on_hover_text("Calibrate capacitive height sensor (measures capacitance vs distance curve on metal sheet)").clicked() {
                if let Some(ref c) = controller {
                    c.calibrate_height_sensor();
                    *status_message = "Height sensor calibration started. Head searching surface...".to_string();
                } else {
                    *status_message = "Controller offline. Connect controller before calibrating sensor.".to_string();
                }
            }

            let probe_btn = Button::new(RichText::new("⬇ Probe Surface (Z=0)").size(12.0).color(Color32::from_rgb(0, 220, 255)))
                .fill(Color32::from_rgb(24, 45, 60));
            if ui.add(probe_btn).on_hover_text("Lower head until nozzle touches sheet metal to zero Z coordinate").clicked() {
                if let Some(ref c) = controller {
                    c.probe_surface_z();
                    *status_message = "Probing surface... Z zero set on contact.".to_string();
                } else {
                    *status_message = "Controller offline.".to_string();
                }
            }

            let servo_bg = if state.follower_enabled {
                Color32::from_rgb(20, 70, 40)
            } else {
                Color32::from_rgb(38, 42, 50)
            };
            let servo_text = if state.follower_enabled {
                RichText::new("🛡 Follower ON").color(Color32::from_rgb(0, 255, 120)).size(12.0)
            } else {
                RichText::new("🛡 Follower OFF").color(Color32::WHITE).size(12.0)
            };
            if ui.add(Button::new(servo_text).fill(servo_bg)).on_hover_text("Toggle capacitive height tracking follower servo").clicked() {
                state.follower_enabled = !state.follower_enabled;
                if let Some(ref c) = controller {
                    c.set_follower_servo(state.follower_enabled);
                }
                *status_message = format!("Height follower servo: {}", if state.follower_enabled { "ENABLED" } else { "DISABLED" });
            }
        });

        ui.add_space(4.0);

        // 2. Optics & Test Pulse
        ribbon_group(ui, "Laser & Optics", |ui| {
            let red_bg = if state.red_pointer_enabled {
                Color32::from_rgb(80, 25, 25)
            } else {
                Color32::from_rgb(38, 42, 50)
            };
            let red_text = if state.red_pointer_enabled {
                RichText::new("🔴 Red Guide ON").color(Color32::from_rgb(255, 100, 100)).size(12.0)
            } else {
                RichText::new("🔴 Red Guide OFF").size(12.0)
            };
            if ui.add(Button::new(red_text).fill(red_bg)).on_hover_text("Toggle 650nm visible red laser guide pointer").clicked() {
                state.red_pointer_enabled = !state.red_pointer_enabled;
                if let Some(ref c) = controller {
                    c.set_red_pointer(state.red_pointer_enabled);
                }
                *status_message = format!("Red pointer: {}", if state.red_pointer_enabled { "ON" } else { "OFF" });
            }

            let pulse_btn = Button::new(RichText::new(format!("⚡ Pulse {}ms", state.pulse_duration_ms)).size(12.0).color(Color32::from_rgb(255, 230, 80)))
                .fill(Color32::from_rgb(48, 42, 24));
            if ui.add(pulse_btn).on_hover_text(format!("Fire laser test burst: {}ms at {:.0}% power", state.pulse_duration_ms, state.pulse_power_pct)).clicked() {
                if let Some(ref c) = controller {
                    c.laser_pulse(state.pulse_duration_ms, state.pulse_power_pct);
                    *status_message = format!("Test pulse fired: {}ms @ {:.0}% power.", state.pulse_duration_ms, state.pulse_power_pct);
                } else {
                    *status_message = "Controller offline.".to_string();
                }
            }

            let gas_bg = if state.gas_open {
                Color32::from_rgb(25, 60, 85)
            } else {
                Color32::from_rgb(38, 42, 50)
            };
            let gas_text = if state.gas_open {
                RichText::new("💨 Gas ON").color(Color32::from_rgb(0, 220, 255)).size(12.0)
            } else {
                RichText::new("💨 Gas Purge").size(12.0)
            };
            if ui.add(Button::new(gas_text).fill(gas_bg)).on_hover_text("Open assist gas valve to purge line").clicked() {
                state.gas_open = !state.gas_open;
                if let Some(ref c) = controller {
                    c.set_gas(3, state.gas_open); // 3 = Air
                }
                *status_message = format!("Assist gas valve: {}", if state.gas_open { "OPEN (Purging)" } else { "CLOSED" });
            }
        });

        ui.add_space(4.0);

        // 3. Work Origin & Homing
        ribbon_group(ui, "Origin & Homing", |ui| {
            let is_conn = controller.as_ref().map_or(false, |c| c.is_connected());
            if ui.add_enabled(is_conn, Button::new(RichText::new("🏠 Home All").size(12.0).color(Color32::from_rgb(130, 235, 180))).fill(Color32::from_rgb(24, 65, 50))).on_hover_text(format!("Home all 3 axes (Z up, then X/Y toward {})", pulse_config.origin_corner)).clicked() {
                if let Some(ref c) = controller {
                    c.home_all(crate::panels::home_config_from_pulse(pulse_config));
                    *status_message = format!("Homing Z (up) and X/Y toward {}...", pulse_config.origin_corner);
                }
            }

            if ui.add_enabled(is_conn, Button::new(RichText::new("⬆ Z-Home").size(12.0).color(Color32::from_rgb(130, 210, 255))).fill(Color32::from_rgb(24, 52, 78))).on_hover_text("Home Z-axis only (Z-up until sensor triggers)").clicked() {
                if let Some(ref c) = controller {
                    c.home_z();
                    *status_message = "Homing Z-axis (moving Z-up to sensor)...".to_string();
                }
            }

            if ui.add(Button::new(RichText::new("📍 Set Origin").size(12.0).color(Color32::from_rgb(0, 230, 230))).fill(Color32::from_rgb(20, 48, 55))).on_hover_text("Zero Workpiece Origin (G54 X0 Y0) at current head position").clicked() {
                *status_message = format!("Workpiece Origin set at current position (X: {}, Y: {})", status.pos_x, status.pos_y);
            }
        });

        ui.add_space(4.0);

        // 3b. After Job Behavior
        ribbon_group(ui, "After Job", |ui| {
            let mut post_changed = false;
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("Return:").size(11.5));
                    egui::ComboBox::from_id_salt("post_job_action_combo")
                        .width(138.0)
                        .selected_text(pulse_config.post_job_action.label())
                        .show_ui(ui, |ui| {
                            for opt in [
                                PostJobAction::ToStartPosition,
                                PostJobAction::ToEndPosition,
                                PostJobAction::ToOrigin,
                                PostJobAction::ToCoordinates,
                            ] {
                                if ui
                                    .selectable_value(
                                        &mut pulse_config.post_job_action,
                                        opt,
                                        opt.label(),
                                    )
                                    .changed()
                                {
                                    post_changed = true;
                                }
                            }
                        });

                    if ui
                        .checkbox(&mut pulse_config.post_job_z_home, "⬆ Z-Home")
                        .on_hover_text("Lift head to Z=0 (upper sensor touch off) after job finishes")
                        .changed()
                    {
                        post_changed = true;
                    }
                });

                if pulse_config.post_job_action == PostJobAction::ToCoordinates {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new("X:").size(11.0));
                        if ui
                            .add(
                                DragValue::new(&mut pulse_config.post_job_x_mm)
                                    .range(0.0..=10_000.0)
                                    .speed(1.0)
                                    .suffix(" mm"),
                            )
                            .changed()
                        {
                            post_changed = true;
                        }
                        ui.label(RichText::new("Y:").size(11.0));
                        if ui
                            .add(
                                DragValue::new(&mut pulse_config.post_job_y_mm)
                                    .range(0.0..=10_000.0)
                                    .speed(1.0)
                                    .suffix(" mm"),
                            )
                            .changed()
                        {
                            post_changed = true;
                        }
                    });
                }
            });
            if post_changed {
                let _ = pulse_config.save();
            }
        });

        ui.add_space(4.0);

        // 4. Safety
        ribbon_group(ui, "Safety", |ui| {
            let estop_btn = Button::new(RichText::new("🛑 E-STOP").size(12.0).color(Color32::WHITE).strong())
                .fill(Color32::from_rgb(180, 30, 30));
            if ui.add(estop_btn).on_hover_text("IMMEDIATE EMERGENCY STOP - Halts all motion and turns laser OFF").clicked() {
                if let Some(ref c) = controller {
                    c.stop();
                }
                *status_message = "EMERGENCY STOP TRIGGERED. All motion halted.".to_string();
            }
        });
    });
}

fn render_properties_tab(
    ui: &mut Ui,
    pulse_config: &mut PulseConfig,
    props_window_state: &mut crate::properties_window::PropertiesWindowState,
    canvas_state: &mut crate::canvas::CanvasState,
    panel_state: &mut crate::panels::PanelState,
    controller: &Option<LaserController>,
    status_message: &mut String,
) {
    let mut changed = false;

    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing = Vec2::new(6.0, 2.0);

        // 1. Table Properties Modal Launcher
        ribbon_group(ui, "Table & Machine Setup", |ui| {
            let is_open = props_window_state.open;
            let btn_fill = if is_open {
                Color32::from_rgb(28, 95, 145)
            } else {
                Color32::from_rgb(32, 65, 105)
            };
            let btn_stroke = if is_open {
                Stroke::new(1.2_f32, Color32::from_rgb(0, 220, 255))
            } else {
                Stroke::new(1.0_f32, Color32::from_rgb(70, 120, 175))
            };
            let table_props_btn = Button::new(
                RichText::new("📐 Table Properties")
                    .size(12.5)
                    .color(Color32::from_rgb(140, 235, 255))
                    .strong(),
            )
            .fill(btn_fill)
            .stroke(btn_stroke)
            .min_size(Vec2::new(142.0, 36.0));

            if ui
                .add(table_props_btn)
                .on_hover_text(
                    "Open Table & Machine Properties window (bed size, pulses/mm, axis direction, jog speeds, and step calibration)",
                )
                .clicked()
            {
                props_window_state.open = !props_window_state.open;
            }

            ui.vertical(|ui| {
                ui.label(
                    RichText::new(format!(
                        "Table: {:.0} × {:.0} mm",
                        pulse_config.table_width_mm, pulse_config.table_height_mm
                    ))
                    .size(11.0)
                    .strong(),
                );
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(format!("Z: {:.0} mm", pulse_config.z_max_travel_mm))
                            .size(10.5)
                            .weak(),
                    );
                    ui.menu_button("📐 Bed Size", |ui| {
                        if ui.button("1300 × 900 mm (Standard)").clicked() {
                            pulse_config.table_width_mm = 1300.0;
                            pulse_config.table_height_mm = 900.0;
                            changed = true;
                            ui.close_menu();
                        }
                        if ui.button("3000 × 1500 mm (Sheet Metal)").clicked() {
                            pulse_config.table_width_mm = 3000.0;
                            pulse_config.table_height_mm = 1500.0;
                            changed = true;
                            ui.close_menu();
                        }
                        if ui.button("4000 × 2000 mm (Heavy Plate)").clicked() {
                            pulse_config.table_width_mm = 4000.0;
                            pulse_config.table_height_mm = 2000.0;
                            changed = true;
                            ui.close_menu();
                        }
                    });
                });
            });
        });

        ui.add_space(2.0);

        // 2. Quick Jog Speed & Presets
        ribbon_group(ui, "Jog Speed & Presets (Auto-Saved)", |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("XY Speed:").size(11.0));
                    if ui
                        .add(
                            egui::DragValue::new(&mut pulse_config.jog_speed_mm_s)
                                .range(0.5..=500.0)
                                .speed(0.5)
                                .suffix(" mm/s"),
                        )
                        .on_hover_text("Conservative XY jog speed in mm/s")
                        .changed()
                    {
                        changed = true;
                    }

                    ui.label(RichText::new("Z Reg:").size(11.0));
                    if ui
                        .add(
                            egui::DragValue::new(&mut pulse_config.z_jog_speed_reg)
                                .range(1..=1000)
                                .speed(1.0),
                        )
                        .on_hover_text("Z-axis Modbus speed register (conservative: 25, SC2000 capture: 100)")
                        .changed()
                    {
                        changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    let safe_btn = Button::new(
                        RichText::new("🛡 Conservative Safe")
                            .size(11.0)
                            .color(Color32::from_rgb(130, 240, 175))
                            .strong(),
                    )
                    .fill(Color32::from_rgb(24, 52, 38))
                    .min_size(Vec2::new(125.0, 19.0));
                    if ui
                        .add(safe_btn)
                        .on_hover_text("Reset to slow, conservative safe defaults (200 pls/mm, 10 mm/s jog, 1 mm step)")
                        .clicked()
                    {
                        let ip = pulse_config.controller_ip.clone();
                        let w = pulse_config.table_width_mm;
                        let h = pulse_config.table_height_mm;
                        *pulse_config = PulseConfig::default();
                        pulse_config.controller_ip = ip;
                        pulse_config.table_width_mm = w;
                        pulse_config.table_height_mm = h;
                        panel_state.jog_mode = crate::panels::JogMode::Step(pulse_config.default_step_mm);
                        panel_state.step_index = 1;
                        changed = true;
                    }

                    let sc2000_btn = Button::new(
                        RichText::new("📋 SC2000 (590 / 1000)")
                            .size(11.0)
                            .color(Color32::from_rgb(255, 220, 110))
                            .strong(),
                    )
                    .fill(Color32::from_rgb(48, 42, 22))
                    .min_size(Vec2::new(135.0, 19.0));
                    if ui
                        .add(sc2000_btn)
                        .on_hover_text("Apply SC2000 BkHardPara.xml scaling (2000×3000 mm, 590 pulses/mm XY, 1,000 pulses/mm Z)")
                        .clicked()
                    {
                        pulse_config.table_width_mm = 2000.0;
                        pulse_config.table_height_mm = 3000.0;
                        pulse_config.pulses_per_mm_x = 590.0;
                        pulse_config.pulses_per_mm_y = 590.0;
                        pulse_config.pulses_per_mm_z = 1_000.0;
                        pulse_config.jog_start_speed_mm_s = 10.0;
                        pulse_config.jog_speed_mm_s = 10.0;
                        pulse_config.jog_accel_mm_s2 = 100.0;
                        pulse_config.z_jog_speed_reg = 50;
                        changed = true;
                    }
                });
            });
        });
    });

    if changed {
        crate::properties_window::sync_and_save_properties(
            pulse_config,
            canvas_state,
            panel_state,
            controller,
            status_message,
        );
    }
}
