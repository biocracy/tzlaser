use egui::{Color32, Key, PointerButton, Pos2, Rect, Response, Sense, Stroke, StrokeKind, Ui, Vec2};
use kurbo::Point;
use std::collections::HashMap;
use tzlaser_core::{Document, Entity, HistoryManager};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorPalette {
    Rainbow,
    Thermal,
    HighContrast,
    OceanDepth,
    MonochromeFade,
}

impl ColorPalette {
    pub fn label(&self) -> &'static str {
        match self {
            ColorPalette::Rainbow => "🌈 Rainbow",
            ColorPalette::Thermal => "🔥 Thermal",
            ColorPalette::HighContrast => "🎯 High Contrast",
            ColorPalette::OceanDepth => "🌊 Ocean",
            ColorPalette::MonochromeFade => "💎 Cyan Fade",
        }
    }

    pub fn color_for_index(&self, idx: usize, total: usize) -> Color32 {
        let t = if total <= 1 {
            0.0
        } else {
            (idx as f32 / (total - 1) as f32).clamp(0.0, 1.0)
        };

        match self {
            ColorPalette::Rainbow => {
                // Smooth HSV spectrum from Red (0°) through Yellow/Green/Cyan/Blue to Violet (290°)
                let hue = t * 290.0;
                hsv_to_color32(hue, 0.92, 1.0)
            }
            ColorPalette::Thermal => {
                // Cool Blue -> Cyan -> Green -> Yellow -> Hot Red
                let stops = [
                    (40.0, 130.0, 255.0),
                    (0.0, 230.0, 255.0),
                    (60.0, 240.0, 110.0),
                    (255.0, 220.0, 40.0),
                    (255.0, 55.0, 55.0),
                ];
                lerp_gradient(&stops, t)
            }
            ColorPalette::HighContrast => {
                const PALETTE: [Color32; 12] = [
                    Color32::from_rgb(0, 230, 255),
                    Color32::from_rgb(255, 215, 0),
                    Color32::from_rgb(255, 70, 160),
                    Color32::from_rgb(80, 255, 110),
                    Color32::from_rgb(255, 130, 40),
                    Color32::from_rgb(150, 110, 255),
                    Color32::from_rgb(0, 255, 200),
                    Color32::from_rgb(255, 80, 80),
                    Color32::from_rgb(190, 255, 60),
                    Color32::from_rgb(100, 170, 255),
                    Color32::from_rgb(255, 160, 220),
                    Color32::from_rgb(255, 255, 255),
                ];
                PALETTE[idx % PALETTE.len()]
            }
            ColorPalette::OceanDepth => {
                let stops = [
                    (120.0, 255.0, 235.0),
                    (0.0, 200.0, 255.0),
                    (45.0, 115.0, 255.0),
                    (155.0, 75.0, 255.0),
                ];
                lerp_gradient(&stops, t)
            }
            ColorPalette::MonochromeFade => {
                let stops = [
                    (235.0, 250.0, 255.0),
                    (60.0, 210.0, 255.0),
                    (65.0, 110.0, 180.0),
                ];
                lerp_gradient(&stops, t)
            }
        }
    }
}

fn lerp_gradient(stops: &[(f32, f32, f32)], t: f32) -> Color32 {
    if stops.is_empty() {
        return Color32::WHITE;
    }
    if stops.len() == 1 {
        return Color32::from_rgb(stops[0].0 as u8, stops[0].1 as u8, stops[0].2 as u8);
    }
    let scaled = t * (stops.len() - 1) as f32;
    let i = (scaled.floor() as usize).min(stops.len() - 2);
    let frac = scaled - i as f32;
    let (r0, g0, b0) = stops[i];
    let (r1, g1, b1) = stops[i + 1];
    Color32::from_rgb(
        (r0 + (r1 - r0) * frac) as u8,
        (g0 + (g1 - g0) * frac) as u8,
        (b0 + (b1 - b0) * frac) as u8,
    )
}

fn hsv_to_color32(h_deg: f32, s: f32, v: f32) -> Color32 {
    let c = v * s;
    let x = c * (1.0 - ((h_deg / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;
    let (r, g, b) = if h_deg < 60.0 {
        (c, x, 0.0)
    } else if h_deg < 120.0 {
        (x, c, 0.0)
    } else if h_deg < 180.0 {
        (0.0, c, x)
    } else if h_deg < 240.0 {
        (0.0, x, c)
    } else if h_deg < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };
    Color32::from_rgb(
        ((r + m) * 255.0) as u8,
        ((g + m) * 255.0) as u8,
        ((b + m) * 255.0) as u8,
    )
}

pub struct CanvasState {
    pub pan: Vec2,          // Pan offset in screen pixels
    pub zoom: f32,          // Pixels per millimeter
    pub bed_width_mm: f32,  // Machine bed width
    pub bed_height_mm: f32, // Machine bed height
    pub active_step_mm: f64,// Step distance for nudge buttons
    pub is_dragging_selection: bool,
    pub box_select_start: Option<Point>,
    pub box_select_current: Option<Point>,
    pub show_cut_order: bool,
    pub show_cut_direction: bool,
    pub color_by_cut_index: bool,
    pub color_palette: ColorPalette,
    pub manual_sort_active: bool,
    pub manual_sort_next_index: usize,
    pub last_viewport_size: Vec2,
}

impl Default for CanvasState {
    fn default() -> Self {
        Self {
            pan: Vec2::new(120.0, 120.0),
            zoom: 0.8,
            bed_width_mm: 1300.0,
            bed_height_mm: 900.0,
            active_step_mm: 10.0,
            is_dragging_selection: false,
            box_select_start: None,
            box_select_current: None,
            show_cut_order: true,
            show_cut_direction: true,
            color_by_cut_index: false,
            color_palette: ColorPalette::Rainbow,
            manual_sort_active: false,
            manual_sort_next_index: 1,
            last_viewport_size: Vec2::new(800.0, 600.0),
        }
    }
}

impl CanvasState {
    pub fn fit_view_to_doc(&mut self, doc: &Document) {
        let bounds = doc.bounding_box().unwrap_or_else(|| {
            kurbo::Rect::new(0.0, 0.0, self.bed_width_mm as f64, self.bed_height_mm as f64)
        });

        let obj_w = (bounds.width() as f32).max(10.0);
        let obj_h = (bounds.height() as f32).max(10.0);
        let vp_w = self.last_viewport_size.x.max(100.0);
        let vp_h = self.last_viewport_size.y.max(100.0);

        let zoom_x = (vp_w * 0.85) / obj_w;
        let zoom_y = (vp_h * 0.85) / obj_h;
        self.zoom = zoom_x.min(zoom_y).clamp(0.05, 50.0);

        let cx = ((bounds.min_x() + bounds.max_x()) / 2.0) as f32;
        let cy = ((bounds.min_y() + bounds.max_y()) / 2.0) as f32;
        self.pan = Vec2::new(
            vp_w / 2.0 - cx * self.zoom,
            vp_h / 2.0 + cy * self.zoom,
        );
    }

    pub fn world_to_screen(&self, pt: Point, origin_screen: Pos2) -> Pos2 {
        Pos2::new(
            origin_screen.x + self.pan.x + (pt.x as f32) * self.zoom,
            origin_screen.y + self.pan.y - (pt.y as f32) * self.zoom,
        )
    }

    pub fn screen_to_world(&self, pt: Pos2, origin_screen: Pos2) -> Point {
        Point::new(
            ((pt.x - origin_screen.x - self.pan.x) / self.zoom) as f64,
            ((origin_screen.y + self.pan.y - pt.y) / self.zoom) as f64,
        )
    }
}

fn draw_arrowhead(painter: &egui::Painter, pos: Pos2, angle_rad: f32, color: Color32, size: f32) {
    let half_angle = 0.45; // ~26 degrees
    let p1 = pos;
    let p2 = Pos2::new(
        pos.x - size * (angle_rad - half_angle).cos(),
        pos.y - size * (angle_rad - half_angle).sin(),
    );
    let p3 = Pos2::new(
        pos.x - size * (angle_rad + half_angle).cos(),
        pos.y - size * (angle_rad + half_angle).sin(),
    );
    painter.add(egui::Shape::convex_polygon(vec![p1, p2, p3], color, Stroke::NONE));
}

fn render_entity(
    painter: &egui::Painter,
    entity: &Entity,
    is_selected: bool,
    layer_color: Color32,
    base_stroke: Stroke,
    sel_glow_stroke: Stroke,
    sel_core_stroke: Stroke,
    state: &CanvasState,
    origin_screen: Pos2,
) {
    match entity {
        Entity::Group { children } => {
            for child in children {
                render_entity(
                    painter,
                    child,
                    is_selected,
                    layer_color,
                    base_stroke,
                    sel_glow_stroke,
                    sel_core_stroke,
                    state,
                    origin_screen,
                );
            }
        }
        Entity::Line { start, end } => {
            let p1 = state.world_to_screen(*start, origin_screen);
            let p2 = state.world_to_screen(*end, origin_screen);
            if is_selected {
                painter.line_segment([p1, p2], sel_glow_stroke);
                painter.line_segment([p1, p2], sel_core_stroke);
            } else {
                painter.line_segment([p1, p2], base_stroke);
            }

            if state.show_cut_direction && p1.distance(p2) > 12.0 {
                let angle = (p2.y - p1.y).atan2(p2.x - p1.x);
                let mid = Pos2::new((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
                let arrow_color = if is_selected {
                    Color32::from_rgb(255, 235, 10)
                } else {
                    layer_color
                };
                draw_arrowhead(painter, mid, angle, arrow_color, 6.5);
            }
        }
        Entity::Polyline { points, closed } => {
            if points.len() >= 2 {
                let screen_pts: Vec<Pos2> = points
                    .iter()
                    .map(|p| state.world_to_screen(*p, origin_screen))
                    .collect();

                for pass in if is_selected { 0..2 } else { 0..1 } {
                    let stroke = if is_selected {
                        if pass == 0 { sel_glow_stroke } else { sel_core_stroke }
                    } else {
                        base_stroke
                    };

                    for i in 0..screen_pts.len() - 1 {
                        painter.line_segment([screen_pts[i], screen_pts[i + 1]], stroke);
                    }
                    if *closed {
                        painter.line_segment([*screen_pts.last().unwrap(), screen_pts[0]], stroke);
                    }
                }

                if state.show_cut_direction {
                    let arrow_color = if is_selected {
                        Color32::from_rgb(255, 235, 10)
                    } else {
                        layer_color
                    };
                    let total_segs = screen_pts.len() - if *closed { 0 } else { 1 };
                    for i in 0..total_segs {
                        let p_a = screen_pts[i];
                        let p_b = screen_pts[(i + 1) % screen_pts.len()];
                        if p_a.distance(p_b) > 12.0 {
                            let mid = Pos2::new((p_a.x + p_b.x) / 2.0, (p_a.y + p_b.y) / 2.0);
                            let angle = (p_b.y - p_a.y).atan2(p_b.x - p_a.x);
                            draw_arrowhead(painter, mid, angle, arrow_color, 6.0);
                        }
                    }
                }
            }
        }
        Entity::Circle { center, radius } => {
            let sc = state.world_to_screen(*center, origin_screen);
            let sr = (*radius as f32) * state.zoom;
            if is_selected {
                painter.circle_stroke(sc, sr, sel_glow_stroke);
                painter.circle_stroke(sc, sr, sel_core_stroke);
            } else {
                painter.circle_stroke(sc, sr, base_stroke);
            }

            if state.show_cut_direction && sr > 6.0 {
                let arrow_color = if is_selected {
                    Color32::from_rgb(255, 235, 10)
                } else {
                    layer_color
                };
                for &ang_deg in &[0.0f32, 90.0, 180.0, 270.0] {
                    let ang_rad = ang_deg.to_radians();
                    let pt = Pos2::new(
                        sc.x + ang_rad.cos() * sr,
                        sc.y - ang_rad.sin() * sr,
                    );
                    let tangent_screen_angle = (-ang_rad.cos()).atan2(-ang_rad.sin());
                    draw_arrowhead(painter, pt, tangent_screen_angle, arrow_color, 5.5);
                }
            }
        }
        Entity::Arc {
            center,
            radius,
            start_angle,
            end_angle,
        } => {
            let steps = 36;
            let sc_radius = (*radius as f32) * state.zoom;
            let sc_center = state.world_to_screen(*center, origin_screen);
            let s = start_angle.rem_euclid(360.0);
            let mut e = end_angle.rem_euclid(360.0);
            if e <= s {
                e += 360.0;
            }
            let d_angle = (e - s) / (steps as f64);
            let arc_points: Vec<Pos2> = (0..=steps)
                .map(|step| {
                    let ang = (s + (step as f64) * d_angle).to_radians();
                    Pos2::new(
                        sc_center.x + (ang.cos() as f32) * sc_radius,
                        sc_center.y - (ang.sin() as f32) * sc_radius,
                    )
                })
                .collect();

            for pass in if is_selected { 0..2 } else { 0..1 } {
                let stroke = if is_selected {
                    if pass == 0 { sel_glow_stroke } else { sel_core_stroke }
                } else {
                    base_stroke
                };
                for i in 0..arc_points.len() - 1 {
                    painter.line_segment([arc_points[i], arc_points[i + 1]], stroke);
                }
            }

            if state.show_cut_direction && arc_points.len() >= 2 {
                let arrow_color = if is_selected {
                    Color32::from_rgb(255, 235, 10)
                } else {
                    layer_color
                };
                let mid_idx = arc_points.len() / 2;
                let p_a = arc_points[mid_idx.saturating_sub(1)];
                let p_b = arc_points[mid_idx];
                if p_a.distance(p_b) > 2.0 {
                    let mid = Pos2::new((p_a.x + p_b.x) / 2.0, (p_a.y + p_b.y) / 2.0);
                    let angle = (p_b.y - p_a.y).atan2(p_b.x - p_a.x);
                    draw_arrowhead(painter, mid, angle, arrow_color, 6.0);
                }
            }
        }
    }
}

pub fn render_canvas(
    ui: &mut Ui,
    state: &mut CanvasState,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    laser_pos_mm: Point,
    is_dry_cut: bool,
    laser_on: bool,
    paused_points_mm: &[Point],
    active_pause_mm: Option<Point>,
) -> Response {
    let (response, painter) = ui.allocate_painter(ui.available_size_before_wrap(), Sense::click_and_drag());
    state.last_viewport_size = response.rect.size();
    let origin_screen = response.rect.min;
    let tolerance_mm = (15.0 / (state.zoom as f64)).clamp(0.5, 30.0);

    // 1. Pan with Middle or Right Mouse Button
    if response.dragged_by(PointerButton::Middle) || response.dragged_by(PointerButton::Secondary) {
        state.pan += response.drag_delta();
    }

    // 2. Zoom with Mouse Wheel centered on cursor
    let scroll_delta = ui.input(|i| i.raw_scroll_delta.y);
    if scroll_delta != 0.0 && response.hovered() {
        if let Some(hover_pos) = response.hover_pos() {
            let world_before = state.screen_to_world(hover_pos, origin_screen);
            let zoom_factor = if scroll_delta > 0.0 { 1.15 } else { 0.85 };
            state.zoom = (state.zoom * zoom_factor).clamp(0.05, 50.0);
            let screen_after = state.world_to_screen(world_before, origin_screen);
            state.pan += hover_pos - screen_after;
        }
    }

    // 3. Selection & Dragging with Primary (Left) Mouse Button
    let primary_pressed = ui.input(|i| i.pointer.button_pressed(PointerButton::Primary));
    let primary_down = ui.input(|i| i.pointer.button_down(PointerButton::Primary));
    let is_shift = ui.input(|i| i.modifiers.shift);

    if response.hovered() && primary_pressed {
        if let Some(press_pos) = response.hover_pos() {
            let press_world = state.screen_to_world(press_pos, origin_screen);

            if state.manual_sort_active {
                // Interactive Manual Sort Mode: single-path selection & assign next cut index
                if let Some((l_idx, e_idx)) = doc.hit_test(press_world, tolerance_mm) {
                    history.record(doc, selection, "Manual Cut Order");
                    doc.assign_manual_cut_index(l_idx, e_idx, state.manual_sort_next_index);
                    state.manual_sort_next_index += 1;
                    selection.clear();
                    selection.push((l_idx, e_idx));
                    state.is_dragging_selection = false;
                    state.box_select_start = None;
                } else {
                    selection.clear();
                    state.is_dragging_selection = false;
                    state.box_select_start = Some(press_world);
                    state.box_select_current = Some(press_world);
                }
            } else if let Some(hit) = doc.hit_test(press_world, tolerance_mm) {
                if is_shift {
                    if let Some(pos) = selection.iter().position(|&x| x == hit) {
                        selection.remove(pos);
                    } else {
                        selection.push(hit);
                    }
                } else if !selection.contains(&hit) {
                    selection.clear();
                    selection.push(hit);
                }
                state.is_dragging_selection = true;
                state.box_select_start = None;
            } else {
                let clicked_inside_bounds = if let Some(bounds) = doc.selection_bounding_box(selection) {
                    bounds.contains(press_world)
                } else {
                    false
                };

                if clicked_inside_bounds {
                    state.is_dragging_selection = true;
                    state.box_select_start = None;
                } else {
                    if !is_shift {
                        selection.clear();
                    }
                    state.is_dragging_selection = false;
                    state.box_select_start = Some(press_world);
                    state.box_select_current = Some(press_world);
                }
            }
        }
    }

    // Process drag motion
    if !state.manual_sort_active {
        if response.drag_started_by(PointerButton::Primary) && state.is_dragging_selection && !selection.is_empty() {
            history.record(doc, selection, "Move Selection");
        }
    }

    if response.dragged_by(PointerButton::Primary) {
        if !state.manual_sort_active && state.is_dragging_selection && !selection.is_empty() {
            let drag_screen = response.drag_delta();
            let dx_mm = (drag_screen.x as f64) / (state.zoom as f64);
            let dy_mm = -(drag_screen.y as f64) / (state.zoom as f64);
            doc.translate_selection(selection, dx_mm, dy_mm);
        } else if state.box_select_start.is_some() {
            if let Some(curr_pos) = response.hover_pos() {
                state.box_select_current = Some(state.screen_to_world(curr_pos, origin_screen));
            }
        }
    }

    // Finish marquee selection on release
    if !primary_down {
        if let (Some(p1), Some(p2)) = (state.box_select_start, state.box_select_current) {
            let sel_rect = kurbo::Rect::from_points(p1, p2);
            if sel_rect.width() > tolerance_mm || sel_rect.height() > tolerance_mm {
                'outer_marquee: for (l_idx, layer) in doc.layers.iter().enumerate() {
                    if !layer.visible { continue; }
                    for (e_idx, ent) in layer.entities.iter().enumerate() {
                        let eb = ent.bounding_box();
                        if sel_rect.intersect(eb).area() > 0.0 || sel_rect.contains(eb.origin()) {
                            if state.manual_sort_active {
                                // In manual sort mode, marquee selection allows only single path selection
                                history.record(doc, selection, "Manual Cut Order");
                                doc.assign_manual_cut_index(l_idx, e_idx, state.manual_sort_next_index);
                                state.manual_sort_next_index += 1;
                                selection.clear();
                                selection.push((l_idx, e_idx));
                                break 'outer_marquee;
                            } else if !selection.contains(&(l_idx, e_idx)) {
                                selection.push((l_idx, e_idx));
                            }
                        }
                    }
                }
            }
        }
        state.is_dragging_selection = false;
        state.box_select_start = None;
        state.box_select_current = None;
    }

    // 4. Keyboard Arrow Nudge & Delete
    if !selection.is_empty() && response.hovered() {
        let step = state.active_step_mm;
        let mut key_dx = 0.0;
        let mut key_dy = 0.0;
        if ui.input(|i| i.key_pressed(Key::ArrowLeft)) {
            key_dx -= step;
        }
        if ui.input(|i| i.key_pressed(Key::ArrowRight)) {
            key_dx += step;
        }
        if ui.input(|i| i.key_pressed(Key::ArrowDown)) {
            key_dy -= step;
        }
        if ui.input(|i| i.key_pressed(Key::ArrowUp)) {
            key_dy += step;
        }
        if key_dx != 0.0 || key_dy != 0.0 {
            history.record(doc, selection, "Nudge Selection");
            doc.translate_selection(selection, key_dx, key_dy);
        }

        if ui.input(|i| i.key_pressed(Key::Delete) || i.key_pressed(Key::Backspace)) {
            history.record(doc, selection, "Delete Selection");
            doc.delete_selection(selection);
            selection.clear();
        }

        // Grouping shortcuts: Ctrl+G (Cmd+G) to Group, Ctrl+Shift+G or Ctrl+U to Ungroup
        let cmd = ui.input(|i| i.modifiers.command || i.modifiers.ctrl);
        let shift = ui.input(|i| i.modifiers.shift);
        if cmd && !shift && ui.input(|i| i.key_pressed(Key::G)) {
            if selection.len() > 1 {
                history.record(doc, selection, "Group Profiles");
                doc.group_selection(selection);
            }
        }
        if (cmd && shift && ui.input(|i| i.key_pressed(Key::G))) || (cmd && ui.input(|i| i.key_pressed(Key::U))) {
            history.record(doc, selection, "Ungroup Profiles");
            doc.ungroup_selection(selection);
        }
    }

    // 5. Render Machine Bed Outline & Grid
    let bed_top_left = state.world_to_screen(Point::new(0.0, state.bed_height_mm as f64), origin_screen);
    let bed_bottom_right = state.world_to_screen(Point::new(state.bed_width_mm as f64, 0.0), origin_screen);
    let bed_rect = Rect::from_two_pos(bed_top_left, bed_bottom_right);

    painter.rect_filled(bed_rect, 0.0, Color32::from_rgb(18, 19, 22));
    painter.rect_stroke(bed_rect, 0.0, Stroke::new(1.5, Color32::from_rgb(60, 65, 75)), StrokeKind::Outside);

    let mut x = 0.0;
    while x <= state.bed_width_mm {
        let p1 = state.world_to_screen(Point::new(x as f64, 0.0), origin_screen);
        let p2 = state.world_to_screen(Point::new(x as f64, state.bed_height_mm as f64), origin_screen);
        painter.line_segment([p1, p2], Stroke::new(0.5, Color32::from_rgb(32, 34, 40)));
        x += 100.0;
    }
    let mut y = 0.0;
    while y <= state.bed_height_mm {
        let p1 = state.world_to_screen(Point::new(0.0, y as f64), origin_screen);
        let p2 = state.world_to_screen(Point::new(state.bed_width_mm as f64, y as f64), origin_screen);
        painter.line_segment([p1, p2], Stroke::new(0.5, Color32::from_rgb(32, 34, 40)));
        y += 100.0;
    }

    let origin_pt = state.world_to_screen(Point::new(0.0, 0.0), origin_screen);
    painter.circle_filled(origin_pt, 4.0, Color32::from_rgb(255, 70, 70));

    // 6. Render Document Layers & Entities with High-Visibility Glow
    let has_selection = !selection.is_empty();
    let index_color_map: HashMap<(usize, usize, Option<usize>), Color32> = if state.color_by_cut_index {
        let seq = doc.compute_cut_sequence();
        let total = seq.len();
        let mut map = HashMap::new();
        for (order_idx, info) in seq.iter().enumerate() {
            map.entry((info.layer_idx, info.entity_idx, info.child_idx))
                .or_insert_with(|| state.color_palette.color_for_index(order_idx, total));
        }
        map
    } else {
        HashMap::new()
    };

    for (l_idx, layer) in doc.layers.iter().enumerate() {
        if !layer.visible {
            continue;
        }

        // Unselected stroke (dimmed slightly if there is an active selection so selection pops out)
        let unsel_alpha = if has_selection { 180 } else { 255 };

        // Selection highlight styles: electric bright gold-yellow with prominent glow
        let sel_glow_stroke = Stroke::new(6.0, Color32::from_rgba_unmultiplied(255, 230, 0, 90));
        let sel_core_stroke = Stroke::new(2.8, Color32::from_rgb(255, 235, 10));

        let layer_color = Color32::from_rgb(layer.color.r, layer.color.g, layer.color.b);
        for (e_idx, entity) in layer.entities.iter().enumerate() {
            let is_selected = selection.contains(&(l_idx, e_idx));
            match entity {
                Entity::Group { children } => {
                    for (c_idx, child) in children.iter().enumerate() {
                        let ent_color = index_color_map
                            .get(&(l_idx, e_idx, Some(c_idx)))
                            .copied()
                            .unwrap_or(layer_color);
                        let base_stroke = Stroke::new(
                            if state.color_by_cut_index { 2.0 } else { 1.5 },
                            Color32::from_rgba_unmultiplied(ent_color.r(), ent_color.g(), ent_color.b(), unsel_alpha),
                        );
                        render_entity(
                            &painter,
                            child,
                            is_selected,
                            ent_color,
                            base_stroke,
                            sel_glow_stroke,
                            sel_core_stroke,
                            state,
                            origin_screen,
                        );
                    }
                }
                _ => {
                    let ent_color = index_color_map
                        .get(&(l_idx, e_idx, None))
                        .copied()
                        .unwrap_or(layer_color);
                    let base_stroke = Stroke::new(
                        if state.color_by_cut_index { 2.0 } else { 1.5 },
                        Color32::from_rgba_unmultiplied(ent_color.r(), ent_color.g(), ent_color.b(), unsel_alpha),
                    );
                    render_entity(
                        &painter,
                        entity,
                        is_selected,
                        ent_color,
                        base_stroke,
                        sel_glow_stroke,
                        sel_core_stroke,
                        state,
                        origin_screen,
                    );
                }
            }
        }
    }

    // 7. Render Selection Bounding Box with Clear Handles (hidden in manual sort mode)
    if !state.manual_sort_active {
        if let Some(bounds) = doc.selection_bounding_box(selection) {
        let p_top_left = state.world_to_screen(Point::new(bounds.min_x(), bounds.max_y()), origin_screen);
        let p_bottom_right = state.world_to_screen(Point::new(bounds.max_x(), bounds.min_y()), origin_screen);
        let sel_rect = Rect::from_two_pos(p_top_left, p_bottom_right);

        // Selection perimeter box
        painter.rect_filled(sel_rect, 2.0, Color32::from_rgba_unmultiplied(0, 210, 255, 18));
        painter.rect_stroke(sel_rect, 2.0, Stroke::new(1.8, Color32::from_rgb(0, 225, 255)), StrokeKind::Outside);

        // 8 Sturdy Transform Grab Handles
        let handle_size = 7.0;
        let corners = [
            sel_rect.left_top(),
            sel_rect.right_top(),
            sel_rect.left_bottom(),
            sel_rect.right_bottom(),
            Pos2::new(sel_rect.center().x, sel_rect.top()),
            Pos2::new(sel_rect.center().x, sel_rect.bottom()),
            Pos2::new(sel_rect.left(), sel_rect.center().y),
            Pos2::new(sel_rect.right(), sel_rect.center().y),
        ];

        for c in corners {
            let h_rect = Rect::from_center_size(c, Vec2::splat(handle_size));
            painter.rect_filled(h_rect, 1.0, Color32::WHITE);
            painter.rect_stroke(h_rect, 1.0, Stroke::new(1.2, Color32::from_rgb(20, 20, 25)), StrokeKind::Outside);
            // Center indicator dot
            painter.circle_filled(c, 1.5, Color32::from_rgb(0, 180, 255));
        }
        }
    }

    // 8. Render Marquee Box Selection Preview
    if let (Some(p1), Some(p2)) = (state.box_select_start, state.box_select_current) {
        let sp1 = state.world_to_screen(p1, origin_screen);
        let sp2 = state.world_to_screen(p2, origin_screen);
        let m_rect = Rect::from_two_pos(sp1, sp2);
        painter.rect_filled(m_rect, 1.0, Color32::from_rgba_unmultiplied(0, 180, 255, 30));
        painter.rect_stroke(m_rect, 1.0, Stroke::new(1.5, Color32::from_rgb(0, 220, 255)), StrokeKind::Outside);
    }

    // 9. Render Cut Order Sequence Badges and Starting Pierce Points
    if state.show_cut_order {
        let cut_sequence = doc.compute_cut_sequence();
        for (order_idx, info) in cut_sequence.iter().enumerate() {
            let sc_start = state.world_to_screen(info.start_point, origin_screen);

            // Subtle rapid travel dashed line from previous cut end
            if order_idx > 0 {
                let prev_end = state.world_to_screen(cut_sequence[order_idx - 1].end_point, origin_screen);
                if prev_end.distance(sc_start) > 8.0 {
                    painter.line_segment(
                        [prev_end, sc_start],
                        Stroke::new(1.0, Color32::from_rgba_unmultiplied(0, 200, 255, 45)),
                    );
                }
            }

            // Pierce Starting Point Marker: bright green target dot with outer ring
            painter.circle_stroke(
                sc_start,
                4.5,
                Stroke::new(1.5, Color32::from_rgb(0, 255, 100)),
            );
            painter.circle_filled(sc_start, 2.0, Color32::from_rgb(0, 255, 100));

            // Cut Sequence Number Pill Badge
            let num = order_idx + 1;
            let text = format!("{}", num);
            let badge_pos = Pos2::new(sc_start.x + 10.0, sc_start.y - 10.0);

            let border_color = if info.is_interior {
                Color32::from_rgb(255, 215, 0) // Gold for interior holes
            } else {
                Color32::from_rgb(0, 229, 255) // Cyan for outer boundaries
            };

            let badge_w = (text.len() as f32 * 7.5 + 8.0).max(18.0);
            let badge_rect = Rect::from_center_size(badge_pos, Vec2::new(badge_w, 16.0));

            painter.rect_filled(badge_rect, 3.0, Color32::from_rgb(15, 23, 42));
            painter.rect_stroke(
                badge_rect,
                3.0,
                Stroke::new(1.2, border_color),
                StrokeKind::Outside,
            );

            painter.text(
                badge_pos,
                egui::Align2::CENTER_CENTER,
                text,
                egui::FontId::monospace(10.5),
                Color32::WHITE,
            );
        }
    }

    // 10. Render Pause Buffer Markers & Physical Laser Head Position Crosshair
    let head_pos = state.world_to_screen(laser_pos_mm, origin_screen);

    for &pt in paused_points_mm {
        if active_pause_mm.map_or(false, |ap| ap.distance(pt) < 1e-3) {
            continue;
        }
        let p = state.world_to_screen(pt, origin_screen);
        let r = 5.5;
        let diamond = vec![
            Pos2::new(p.x, p.y - r),
            Pos2::new(p.x + r, p.y),
            Pos2::new(p.x, p.y + r),
            Pos2::new(p.x - r, p.y),
        ];
        painter.add(egui::Shape::convex_polygon(
            diamond,
            Color32::from_rgba_unmultiplied(245, 165, 35, 130),
            Stroke::new(1.2, Color32::from_rgb(255, 200, 70)),
        ));
    }

    if let Some(pause_mm) = active_pause_mm {
        let pause_screen = state.world_to_screen(pause_mm, origin_screen);

        if laser_pos_mm.distance(pause_mm) > 0.5 {
            let delta = pause_screen - head_pos;
            let dist = delta.length();
            if dist > 1.0 {
                let dir = delta / dist;
                let mut t = 0.0;
                let dash = 6.0;
                let gap = 4.0;
                while t < dist {
                    let t_end = (t + dash).min(dist);
                    painter.line_segment(
                        [head_pos + dir * t, head_pos + dir * t_end],
                        Stroke::new(1.8, Color32::from_rgba_unmultiplied(255, 185, 50, 215)),
                    );
                    t += dash + gap;
                }
            }
        }

        painter.circle_stroke(
            pause_screen,
            13.0,
            Stroke::new(2.0, Color32::from_rgb(255, 185, 40)),
        );
        let r = 7.0;
        let diamond = vec![
            Pos2::new(pause_screen.x, pause_screen.y - r),
            Pos2::new(pause_screen.x + r, pause_screen.y),
            Pos2::new(pause_screen.x, pause_screen.y + r),
            Pos2::new(pause_screen.x - r, pause_screen.y),
        ];
        painter.add(egui::Shape::convex_polygon(
            diamond,
            Color32::from_rgb(255, 175, 30),
            Stroke::new(1.5, Color32::from_rgb(255, 240, 170)),
        ));

        let badge_rect = egui::Rect::from_min_size(
            Pos2::new(pause_screen.x + 14.0, pause_screen.y - 18.0),
            egui::Vec2::new(66.0, 17.0),
        );
        painter.rect_filled(
            badge_rect,
            3.0,
            Color32::from_rgba_unmultiplied(45, 32, 12, 230),
        );
        painter.rect_stroke(
            badge_rect,
            3.0,
            Stroke::new(1.0, Color32::from_rgb(255, 185, 40)),
            egui::StrokeKind::Outside,
        );
        painter.text(
            badge_rect.center(),
            egui::Align2::CENTER_CENTER,
            "⏸ PAUSE",
            egui::FontId::proportional(10.5),
            Color32::from_rgb(255, 220, 110),
        );
    }

    let cross_len = 13.0;
    let cross_color = if laser_on {
        Color32::from_rgb(255, 85, 30)
    } else if is_dry_cut {
        Color32::from_rgb(0, 235, 255)
    } else {
        Color32::from_rgb(255, 50, 50)
    };

    if laser_on {
        painter.circle_filled(head_pos, 5.5, Color32::from_rgba_unmultiplied(255, 190, 60, 140));
        painter.circle_stroke(head_pos, 12.0, Stroke::new(2.0, Color32::from_rgb(255, 110, 40)));
    } else if is_dry_cut {
        painter.circle_stroke(head_pos, 11.0, Stroke::new(2.0, Color32::from_rgb(0, 235, 255)));
        painter.circle_filled(head_pos, 3.0, Color32::from_rgb(0, 255, 160));
    }

    painter.line_segment(
        [Pos2::new(head_pos.x - cross_len, head_pos.y), Pos2::new(head_pos.x + cross_len, head_pos.y)],
        Stroke::new(2.0, cross_color),
    );
    painter.line_segment(
        [Pos2::new(head_pos.x, head_pos.y - cross_len), Pos2::new(head_pos.x, head_pos.y + cross_len)],
        Stroke::new(2.0, cross_color),
    );
    painter.circle_stroke(
        head_pos,
        8.0,
        Stroke::new(1.2, cross_color.linear_multiply(0.75)),
    );

    response
}
