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

#[derive(Debug, Clone)]
pub struct PathContextMenu {
    pub screen_pos: Pos2,
    pub point_mm: Point,
    pub path_index_1based: usize,
}

#[derive(Debug, Clone)]
pub struct SortSnapshot {
    pub sort_mode: tzlaser_core::SortMode,
    pub manual_sequence: Vec<(usize, usize, Option<usize>)>,
    pub saved_cut_order: Vec<(usize, usize, Option<usize>)>,
    pub use_saved_cut_order: bool,
    pub manual_sort_next_index: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrawTool {
    None,
    Rectangle,
    Circle,
    Line,
    Point,
    Ellipse,
    Spline,
}

pub struct CanvasState {
    pub pan: Vec2,          // Pan offset in screen pixels
    pub zoom: f32,          // Pixels per millimeter
    pub bed_width_mm: f32,  // Machine bed width
    pub bed_height_mm: f32, // Machine bed height
    pub origin_corner: String,
    pub active_step_mm: f64,// Step distance for nudge buttons
    pub is_dragging_selection: bool,
    pub box_select_start: Option<Point>,
    pub box_select_current: Option<Point>,
    pub show_cut_order: bool,
    pub show_cut_direction: bool,
    pub show_grid: bool,
    pub show_rulers: bool,
    pub show_retraction_zone: bool,
    pub home_retract_x_mm: f64,
    pub home_retract_y_mm: f64,
    pub color_by_cut_index: bool,
    pub color_palette: ColorPalette,
    pub manual_sort_active: bool,
    pub manual_sort_next_index: usize,
    pub sort_undo_stack: Vec<SortSnapshot>,
    pub last_viewport_size: Vec2,
    pub context_menu: Option<PathContextMenu>,
    pub pending_continue_from_point: Option<Point>,
    pub pending_set_start_point: Option<(usize, Point)>,
    pub draw_tool: DrawTool,
    pub draw_anchor: Option<Point>,
    pub draw_spline_pts: Vec<Point>,
    pub draw_spline_closed: bool,
    /// Cached obstacle-avoiding rapid travel polylines keyed by scene geometry + obstacle hash.
    pub cached_rapid_routes: (Option<u64>, Vec<Vec<Point>>),
}

impl Default for CanvasState {
    fn default() -> Self {
        Self {
            pan: Vec2::new(120.0, 120.0),
            zoom: 0.8,
            bed_width_mm: 1300.0,
            bed_height_mm: 900.0,
            origin_corner: "Bottom-Left (X0, Y0)".to_string(),
            active_step_mm: 10.0,
            is_dragging_selection: false,
            box_select_start: None,
            box_select_current: None,
            show_cut_order: true,
            show_cut_direction: true,
            show_grid: true,
            show_rulers: true,
            show_retraction_zone: true,
            home_retract_x_mm: 10.0,
            home_retract_y_mm: 10.0,
            color_by_cut_index: false,
            color_palette: ColorPalette::Rainbow,
            manual_sort_active: false,
            manual_sort_next_index: 1,
            sort_undo_stack: Vec::new(),
            last_viewport_size: Vec2::new(800.0, 600.0),
            context_menu: None,
            pending_continue_from_point: None,
            pending_set_start_point: None,
            draw_tool: DrawTool::None,
            draw_anchor: None,
            draw_spline_pts: Vec::new(),
            draw_spline_closed: false,
            cached_rapid_routes: (None, Vec::new()),
        }
    }
}

impl CanvasState {
    pub fn push_sort_undo(&mut self, doc: &Document) {
        self.sort_undo_stack.push(SortSnapshot {
            sort_mode: doc.sort_mode,
            manual_sequence: doc.manual_sequence.clone(),
            saved_cut_order: doc.saved_cut_order.clone(),
            use_saved_cut_order: doc.use_saved_cut_order,
            manual_sort_next_index: self.manual_sort_next_index,
        });
        if self.sort_undo_stack.len() > 100 {
            self.sort_undo_stack.remove(0);
        }
    }

    pub fn pop_sort_undo(&mut self, doc: &mut Document) -> bool {
        if let Some(snap) = self.sort_undo_stack.pop() {
            doc.sort_mode = snap.sort_mode;
            doc.manual_sequence = snap.manual_sequence;
            doc.saved_cut_order = snap.saved_cut_order;
            doc.use_saved_cut_order = snap.use_saved_cut_order;
            self.manual_sort_next_index = snap.manual_sort_next_index;
            true
        } else {
            false
        }
    }

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

    /// Return the selected bed origin in world coordinates (mm) and unit screen-space directions
    /// for the X and Y axes pointing inward across the cutting bed.
    pub fn origin_world_and_axes(&self) -> (Point, Vec2, Vec2) {
        let w = self.bed_width_mm as f64;
        let h = self.bed_height_mm as f64;
        if self.origin_corner.contains("Top-Left") {
            // World (0, H): +X right (+1, 0), Y down into bed (+1 screen Y)
            (Point::new(0.0, h), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0))
        } else if self.origin_corner.contains("Bottom-Right") {
            // World (W, 0): X left into bed (-1, 0), +Y up (-1 screen Y)
            (Point::new(w, 0.0), Vec2::new(-1.0, 0.0), Vec2::new(0.0, -1.0))
        } else if self.origin_corner.contains("Top-Right") {
            // World (W, H): X left into bed (-1, 0), Y down into bed (+1 screen Y)
            (Point::new(w, h), Vec2::new(-1.0, 0.0), Vec2::new(0.0, 1.0))
        } else {
            // Default: Bottom-Left (0, 0): +X right (+1, 0), +Y up (-1 screen Y)
            (Point::new(0.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, -1.0))
        }
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

pub fn make_rectangle_entity(a: Point, b: Point) -> Entity {
    let min_x = a.x.min(b.x);
    let max_x = a.x.max(b.x);
    let min_y = a.y.min(b.y);
    let max_y = a.y.max(b.y);
    Entity::Polyline {
        points: vec![
            Point::new(min_x, min_y),
            Point::new(max_x, min_y),
            Point::new(max_x, max_y),
            Point::new(min_x, max_y),
        ],
        closed: true,
    }
}

pub fn make_ellipse_entity(center: Point, rx: f64, ry: f64) -> Entity {
    let segs = 64usize;
    let points = (0..segs)
        .map(|i| {
            let ang = std::f64::consts::TAU * (i as f64) / (segs as f64);
            Point::new(center.x + rx * ang.cos(), center.y + ry * ang.sin())
        })
        .collect();
    Entity::Polyline {
        points,
        closed: true,
    }
}

pub fn catmull_rom_spline(ctrl: &[Point], closed: bool) -> Vec<Point> {
    if ctrl.len() < 2 {
        return ctrl.to_vec();
    }
    if ctrl.len() == 2 {
        return ctrl.to_vec();
    }
    let samples_per_seg = 12usize;
    let mut out = Vec::new();
    let n = ctrl.len();
    let seg_count = if closed { n } else { n - 1 };
    for i in 0..seg_count {
        let p0 = if closed {
            ctrl[(i + n - 1) % n]
        } else {
            ctrl[i.saturating_sub(1)]
        };
        let p1 = ctrl[i];
        let p2 = ctrl[(i + 1) % n];
        let p3 = if closed {
            ctrl[(i + 2) % n]
        } else {
            ctrl[(i + 2).min(n - 1)]
        };

        for s in 0..samples_per_seg {
            let t = (s as f64) / (samples_per_seg as f64);
            let t2 = t * t;
            let t3 = t2 * t;
            let x = 0.5
                * ((2.0 * p1.x)
                    + (-p0.x + p2.x) * t
                    + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2
                    + (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);
            let y = 0.5
                * ((2.0 * p1.y)
                    + (-p0.y + p2.y) * t
                    + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2
                    + (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);
            out.push(Point::new(x, y));
        }
    }
    if !closed {
        out.push(*ctrl.last().unwrap());
    }
    out
}

fn first_cut_layer_idx(doc: &Document) -> usize {
    doc.layers
        .iter()
        .position(|l| !l.is_travel && !l.is_obstacle)
        .unwrap_or(0)
}

fn commit_drawn_entity(
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    label: &str,
    entity: Entity,
) {
    history.record(doc, selection, label);
    let l_idx = first_cut_layer_idx(doc);
    let e_idx = doc.layers[l_idx].entities.len();
    doc.layers[l_idx].entities.push(entity);
    selection.clear();
    selection.push((l_idx, e_idx));
}

fn draw_filled_polygon(painter: &egui::Painter, pts: &[Pos2], fill: Color32) {
    if pts.len() < 3 {
        return;
    }
    // Ear-clipping triangulation so both convex and concave polygons fill cleanly
    let mut indices: Vec<usize> = (0..pts.len()).collect();
    let area2: f32 = (0..pts.len())
        .map(|i| {
            let j = (i + 1) % pts.len();
            pts[i].x * pts[j].y - pts[j].x * pts[i].y
        })
        .sum();
    if area2 < 0.0 {
        indices.reverse();
    }

    let mut mesh = egui::epaint::Mesh::default();
    for &p in pts {
        mesh.colored_vertex(p, fill);
    }

    let cross = |a: Pos2, b: Pos2, c: Pos2| -> f32 {
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    };
    let pt_in_tri = |p: Pos2, a: Pos2, b: Pos2, c: Pos2| -> bool {
        let c1 = cross(a, b, p);
        let c2 = cross(b, c, p);
        let c3 = cross(c, a, p);
        c1 >= 0.0 && c2 >= 0.0 && c3 >= 0.0
    };

    let mut guard = pts.len() * pts.len();
    while indices.len() >= 3 && guard > 0 {
        guard -= 1;
        let m = indices.len();
        let mut clipped = false;
        for i in 0..m {
            let i0 = indices[(i + m - 1) % m];
            let i1 = indices[i];
            let i2 = indices[(i + 1) % m];
            if cross(pts[i0], pts[i1], pts[i2]) <= 1e-4 {
                continue;
            }
            let any_inside = indices
                .iter()
                .any(|&k| k != i0 && k != i1 && k != i2 && pt_in_tri(pts[k], pts[i0], pts[i1], pts[i2]));
            if !any_inside {
                mesh.add_triangle(i0 as u32, i1 as u32, i2 as u32);
                indices.remove(i);
                clipped = true;
                break;
            }
        }
        if !clipped {
            break;
        }
    }
    if !mesh.indices.is_empty() {
        painter.add(egui::Shape::mesh(mesh));
    }
}

fn draw_dashed_closed_poly(painter: &egui::Painter, pts: &[Pos2], stroke: Stroke) {
    if pts.len() < 2 {
        return;
    }
    let dash_len = 6.0_f32;
    let gap_len = 4.0_f32;
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let len = dx.hypot(dy);
        if len < 0.5 {
            continue;
        }
        let ux = dx / len;
        let uy = dy / len;
        let mut d = 0.0_f32;
        while d < len {
            let d_end = (d + dash_len).min(len);
            let p1 = Pos2::new(a.x + ux * d, a.y + uy * d);
            let p2 = Pos2::new(a.x + ux * d_end, a.y + uy * d_end);
            painter.line_segment([p1, p2], stroke);
            d += dash_len + gap_len;
        }
    }
}

fn desaturate_and_alpha(c: Color32, sat_factor: f32, alpha: u8) -> Color32 {
    let r = c.r() as f32;
    let g = c.g() as f32;
    let b = c.b() as f32;
    let lum = 0.299 * r + 0.587 * g + 0.114 * b;
    let nr = (lum + (r - lum) * sat_factor).round().clamp(0.0, 255.0) as u8;
    let ng = (lum + (g - lum) * sat_factor).round().clamp(0.0, 255.0) as u8;
    let nb = (lum + (b - lum) * sat_factor).round().clamp(0.0, 255.0) as u8;
    Color32::from_rgba_unmultiplied(nr, ng, nb, alpha)
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
    can_context_menu: bool,
    completed_before_path_1based: Option<usize>,
) -> Response {
    let (response, painter) = ui.allocate_painter(ui.available_size_before_wrap(), Sense::click_and_drag());
    state.last_viewport_size = response.rect.size();
    let origin_screen = response.rect.min;
    let tolerance_mm = (15.0 / (state.zoom as f64)).clamp(0.5, 30.0);

    // 1. Pan with Middle or Right Mouse Button
    if response.dragged_by(PointerButton::Middle) || response.dragged_by(PointerButton::Secondary) {
        state.pan += response.drag_delta();
        state.context_menu = None;
    }

    // 1B. Right-click Path Context Menu (only when paused or stopped)
    let mut context_menu_just_opened = false;
    if !can_context_menu {
        state.context_menu = None;
    } else if response.secondary_clicked() {
        let click_pos = response
            .interact_pointer_pos()
            .or_else(|| ui.input(|i| i.pointer.interact_pos()))
            .or(response.hover_pos());
        if let Some(screen_pos) = click_pos {
            let click_world = state.screen_to_world(screen_pos, origin_screen);
            if let Some((snapped_pt, path_idx, _)) = doc.closest_point_on_cut_paths(click_world, tolerance_mm) {
                state.context_menu = Some(PathContextMenu {
                    screen_pos,
                    point_mm: snapped_pt,
                    path_index_1based: path_idx,
                });
                context_menu_just_opened = true;
            } else {
                state.context_menu = None;
            }
        }
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

    // 3. Interactive Shape Drawing OR Selection & Dragging with Primary (Left) Mouse Button
    let primary_pressed = ui.input(|i| i.pointer.button_pressed(PointerButton::Primary));
    let primary_down = ui.input(|i| i.pointer.button_down(PointerButton::Primary));
    let primary_released = ui.input(|i| i.pointer.button_released(PointerButton::Primary));
    let is_shift = ui.input(|i| i.modifiers.shift);

    if state.draw_tool != DrawTool::None {
        if ui.input(|i| i.key_pressed(Key::Escape)) {
            if state.draw_anchor.is_some() || !state.draw_spline_pts.is_empty() {
                state.draw_anchor = None;
                state.draw_spline_pts.clear();
            } else {
                state.draw_tool = DrawTool::None;
            }
        }

        if state.draw_tool == DrawTool::Spline
            && !state.draw_spline_pts.is_empty()
            && (ui.input(|i| i.key_pressed(Key::Enter))
                || response.double_clicked()
                || response.secondary_clicked())
        {
            if state.draw_spline_pts.len() >= 2 {
                let pts = catmull_rom_spline(&state.draw_spline_pts, state.draw_spline_closed);
                commit_drawn_entity(
                    doc,
                    selection,
                    history,
                    "Draw Spline",
                    Entity::Polyline {
                        points: pts,
                        closed: state.draw_spline_closed,
                    },
                );
            }
            state.draw_spline_pts.clear();
        } else if response.hovered() && primary_pressed {
            if let Some(press_pos) = response.hover_pos() {
                let pt = state.screen_to_world(press_pos, origin_screen);
                match state.draw_tool {
                    DrawTool::Point => {
                        commit_drawn_entity(
                            doc,
                            selection,
                            history,
                            "Draw Point",
                            Entity::Line { start: pt, end: pt },
                        );
                    }
                    DrawTool::Spline => {
                        state.draw_spline_pts.push(pt);
                    }
                    DrawTool::Rectangle | DrawTool::Circle | DrawTool::Line | DrawTool::Ellipse => {
                        if let Some(anchor) = state.draw_anchor {
                            let dx = pt.x - anchor.x;
                            let dy = pt.y - anchor.y;
                            let dist = dx.hypot(dy);
                            if dist > 0.2 {
                                match state.draw_tool {
                                    DrawTool::Rectangle => commit_drawn_entity(
                                        doc,
                                        selection,
                                        history,
                                        "Draw Rectangle",
                                        make_rectangle_entity(anchor, pt),
                                    ),
                                    DrawTool::Circle => commit_drawn_entity(
                                        doc,
                                        selection,
                                        history,
                                        "Draw Circle",
                                        Entity::Circle {
                                            center: anchor,
                                            radius: dist,
                                        },
                                    ),
                                    DrawTool::Line => commit_drawn_entity(
                                        doc,
                                        selection,
                                        history,
                                        "Draw Line",
                                        Entity::Line {
                                            start: anchor,
                                            end: pt,
                                        },
                                    ),
                                    DrawTool::Ellipse => {
                                        if dx.abs() > 0.2 && dy.abs() > 0.2 {
                                            commit_drawn_entity(
                                                doc,
                                                selection,
                                                history,
                                                "Draw Ellipse",
                                                make_ellipse_entity(anchor, dx.abs(), dy.abs()),
                                            );
                                        }
                                    }
                                    _ => {}
                                }
                            }
                            state.draw_anchor = None;
                        } else {
                            state.draw_anchor = Some(pt);
                        }
                    }
                    DrawTool::None => {}
                }
            }
        } else if primary_released {
            if let (Some(anchor), Some(rel_pos)) = (state.draw_anchor, response.hover_pos()) {
                let pt = state.screen_to_world(rel_pos, origin_screen);
                let dx = pt.x - anchor.x;
                let dy = pt.y - anchor.y;
                let dist = dx.hypot(dy);
                // If user click-dragged > 2 mm, commit on release
                if dist > 2.0 {
                    match state.draw_tool {
                        DrawTool::Rectangle => {
                            commit_drawn_entity(
                                doc,
                                selection,
                                history,
                                "Draw Rectangle",
                                make_rectangle_entity(anchor, pt),
                            );
                            state.draw_anchor = None;
                        }
                        DrawTool::Circle => {
                            commit_drawn_entity(
                                doc,
                                selection,
                                history,
                                "Draw Circle",
                                Entity::Circle {
                                    center: anchor,
                                    radius: dist,
                                },
                            );
                            state.draw_anchor = None;
                        }
                        DrawTool::Line => {
                            commit_drawn_entity(
                                doc,
                                selection,
                                history,
                                "Draw Line",
                                Entity::Line {
                                    start: anchor,
                                    end: pt,
                                },
                            );
                            state.draw_anchor = None;
                        }
                        DrawTool::Ellipse => {
                            if dx.abs() > 0.5 && dy.abs() > 0.5 {
                                commit_drawn_entity(
                                    doc,
                                    selection,
                                    history,
                                    "Draw Ellipse",
                                    make_ellipse_entity(anchor, dx.abs(), dy.abs()),
                                );
                                state.draw_anchor = None;
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    } else if response.hovered() && primary_pressed {
        if let Some(press_pos) = response.hover_pos() {
            let press_world = state.screen_to_world(press_pos, origin_screen);

            if state.manual_sort_active {
                // Interactive Manual Sort Mode: single-path selection & assign next cut index
                if let Some((l_idx, e_idx, c_idx)) = doc.hit_test_path(press_world, tolerance_mm) {
                    history.record(doc, selection, "Manual Cut Order");
                    state.push_sort_undo(doc);
                    doc.assign_manual_cut_index(l_idx, e_idx, c_idx, state.manual_sort_next_index);
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
                doc.expand_selection_with_groups(selection);
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
    if !state.manual_sort_active && !doc.is_selection_locked(selection) {
        if response.drag_started_by(PointerButton::Primary) && state.is_dragging_selection && !selection.is_empty() {
            history.record(doc, selection, "Move Selection");
        }
    }

    if response.dragged_by(PointerButton::Primary) {
        if !state.manual_sort_active
            && state.is_dragging_selection
            && !selection.is_empty()
            && !doc.is_selection_locked(selection)
        {
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
                                let c_idx = match ent {
                                    Entity::Group { children } => children.iter().position(|c| {
                                        let cb = c.bounding_box();
                                        sel_rect.intersect(cb).area() > 0.0 || sel_rect.contains(cb.origin())
                                    }),
                                    _ => None,
                                };
                                history.record(doc, selection, "Manual Cut Order");
                                state.push_sort_undo(doc);
                                doc.assign_manual_cut_index(l_idx, e_idx, c_idx, state.manual_sort_next_index);
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
                if !state.manual_sort_active {
                    doc.expand_selection_with_groups(selection);
                }
            }
        }
        state.is_dragging_selection = false;
        state.box_select_start = None;
        state.box_select_current = None;
    }

    // 4. Keyboard Arrow Nudge & Delete
    if !selection.is_empty() && response.hovered() {
        let is_locked = doc.is_selection_locked(selection);
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
        if !is_locked && (key_dx != 0.0 || key_dy != 0.0) {
            history.record(doc, selection, "Nudge Selection");
            doc.translate_selection(selection, key_dx, key_dy);
        }

        if !is_locked && ui.input(|i| i.key_pressed(Key::Delete) || i.key_pressed(Key::Backspace)) {
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
    #[cfg(target_os = "windows")]
    painter.rect_filled(response.rect, 0.0, Color32::from_rgb(34, 38, 48));

    let bed_top_left = state.world_to_screen(Point::new(0.0, state.bed_height_mm as f64), origin_screen);
    let bed_bottom_right = state.world_to_screen(Point::new(state.bed_width_mm as f64, 0.0), origin_screen);
    let bed_rect = Rect::from_two_pos(bed_top_left, bed_bottom_right);

    let bed_fill = if cfg!(target_os = "windows") {
        Color32::from_rgb(16, 18, 23)
    } else {
        Color32::from_rgb(18, 19, 22)
    };
    let bed_border = if cfg!(target_os = "windows") {
        Color32::from_rgb(95, 108, 132)
    } else {
        Color32::from_rgb(60, 65, 75)
    };
    let grid_color = if cfg!(target_os = "windows") {
        Color32::from_rgb(56, 64, 78)
    } else {
        Color32::from_rgb(50, 56, 68)
    };

    painter.rect_filled(bed_rect, 0.0, bed_fill);
    painter.rect_stroke(bed_rect, 0.0, Stroke::new(1.5_f32, bed_border), StrokeKind::Outside);

    // Origin marker & X/Y axes stemming from the selected table origin corner
    let (origin_world, x_screen_dir, y_screen_dir) = state.origin_world_and_axes();

    if state.show_grid {
        let minor_grid_color = if cfg!(target_os = "windows") {
            Color32::from_rgb(34, 39, 49)
        } else {
            Color32::from_rgb(30, 34, 42)
        };
        let target_mm = (90.0_f32 / state.zoom.max(0.01)) as f64;
        let candidates = [1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0];
        let major_step = candidates
            .iter()
            .copied()
            .find(|&c| c >= target_mm)
            .unwrap_or(1000.0);
        let minor_step = major_step / 5.0;
        let draw_minor = (minor_step as f32) * state.zoom >= 8.0;
        let step = if draw_minor { minor_step } else { major_step };

        let x_sign = x_screen_dir.x as f64;
        let y_world_sign = -y_screen_dir.y as f64;
        let w = state.bed_width_mm as f64;
        let h = state.bed_height_mm as f64;

        let mx_a = (0.0 - origin_world.x) * x_sign;
        let mx_b = (w - origin_world.x) * x_sign;
        let (mx_min, mx_max) = (mx_a.min(mx_b), mx_a.max(mx_b));
        let kx_start = (mx_min / step).ceil() as i64;
        let kx_end = (mx_max / step).floor() as i64;
        for k in kx_start..=kx_end {
            let wx = origin_world.x + (k as f64) * step * x_sign;
            let is_major = !draw_minor || k.rem_euclid(5) == 0;
            let (stroke_w, col) = if is_major {
                (1.0_f32, grid_color)
            } else {
                (0.6_f32, minor_grid_color)
            };
            let p1 = state.world_to_screen(Point::new(wx, 0.0), origin_screen);
            let p2 = state.world_to_screen(Point::new(wx, h), origin_screen);
            painter.line_segment([p1, p2], Stroke::new(stroke_w, col));
        }

        let my_a = (0.0 - origin_world.y) * y_world_sign;
        let my_b = (h - origin_world.y) * y_world_sign;
        let (my_min, my_max) = (my_a.min(my_b), my_a.max(my_b));
        let ky_start = (my_min / step).ceil() as i64;
        let ky_end = (my_max / step).floor() as i64;
        for k in ky_start..=ky_end {
            let wy = origin_world.y + (k as f64) * step * y_world_sign;
            let is_major = !draw_minor || k.rem_euclid(5) == 0;
            let (stroke_w, col) = if is_major {
                (1.0_f32, grid_color)
            } else {
                (0.6_f32, minor_grid_color)
            };
            let p1 = state.world_to_screen(Point::new(0.0, wy), origin_screen);
            let p2 = state.world_to_screen(Point::new(w, wy), origin_screen);
            painter.line_segment([p1, p2], Stroke::new(stroke_w, col));
        }
    }

    // 5b. Render Homing Retraction Exclusion Zone in the negative machine-coordinate realm
    // (0,0 origin remains 0,0; the retraction zone sits behind the origin toward the sensors)
    let rx = state.home_retract_x_mm.max(0.0);
    let ry = state.home_retract_y_mm.max(0.0);
    if state.show_retraction_zone && (rx > 0.01 || ry > 0.01) {
        let x_sign = x_screen_dir.x as f64;
        let y_world_sign = -y_screen_dir.y as f64;
        let w = state.bed_width_mm as f64;
        let h = state.bed_height_mm as f64;
        let to_world = |mx: f64, my: f64| -> Point {
            Point::new(origin_world.x + mx * x_sign, origin_world.y + my * y_world_sign)
        };

        let fill_col = Color32::from_rgba_unmultiplied(220, 58, 58, 42);
        let stroke_col = Color32::from_rgba_unmultiplied(240, 88, 78, 175);

        // X-negative strip: mx in [-rx, 0], my in [-ry, H]
        if rx > 0.01 {
            let x_strip = Rect::from_two_pos(
                state.world_to_screen(to_world(-rx, -ry), origin_screen),
                state.world_to_screen(to_world(0.0, h), origin_screen),
            );
            painter.rect_filled(x_strip, 0.0, fill_col);
        }
        // Y-negative strip: mx in [0, W], my in [-ry, 0]
        if ry > 0.01 {
            let y_strip = Rect::from_two_pos(
                state.world_to_screen(to_world(0.0, -ry), origin_screen),
                state.world_to_screen(to_world(w, 0.0), origin_screen),
            );
            painter.rect_filled(y_strip, 0.0, fill_col);
        }

        let l_outline: Vec<Pos2> = [
            to_world(-rx, -ry),
            to_world(w, -ry),
            to_world(w, 0.0),
            to_world(0.0, 0.0),
            to_world(0.0, h),
            to_world(-rx, h),
        ]
        .into_iter()
        .map(|p| state.world_to_screen(p, origin_screen))
        .collect();
        draw_dashed_closed_poly(&painter, &l_outline, Stroke::new(1.3_f32, stroke_col));

        let max_r = rx.max(ry);
        if (max_r as f32) * state.zoom >= 7.0 {
            let lbl_pos = if ry > 0.01 {
                state.world_to_screen(to_world(w * 0.5, -ry * 0.5), origin_screen)
            } else {
                state.world_to_screen(to_world(-rx * 0.5, h * 0.5), origin_screen)
            };
            painter.text(
                lbl_pos,
                egui::Align2::CENTER_CENTER,
                format!("Homing Retraction Exclusion (X: -{:.0} mm, Y: -{:.0} mm)", rx, ry),
                egui::FontId::proportional(10.0),
                Color32::from_rgba_unmultiplied(245, 130, 120, 210),
            );
        }
    }

    let origin_pt = state.world_to_screen(origin_world, origin_screen);
    let axis_len = 64.0_f32;

    let x_color = Color32::from_rgb(240, 80, 80);
    let y_color = Color32::from_rgb(75, 225, 115);
    let x_end = origin_pt + x_screen_dir * axis_len;
    let y_end = origin_pt + y_screen_dir * axis_len;

    painter.line_segment([origin_pt, x_end], Stroke::new(2.2, x_color));
    draw_arrowhead(&painter, x_end, x_screen_dir.y.atan2(x_screen_dir.x), x_color, 8.5);
    let x_label_pos = x_end + x_screen_dir * 10.0 + Vec2::new(0.0, y_screen_dir.y * 8.0);
    painter.text(
        x_label_pos,
        egui::Align2::CENTER_CENTER,
        "X",
        egui::FontId::proportional(13.0),
        x_color,
    );

    painter.line_segment([origin_pt, y_end], Stroke::new(2.2, y_color));
    draw_arrowhead(&painter, y_end, y_screen_dir.y.atan2(y_screen_dir.x), y_color, 8.5);
    let y_label_pos = y_end + y_screen_dir * 10.0 + Vec2::new(x_screen_dir.x * 10.0, 0.0);
    painter.text(
        y_label_pos,
        egui::Align2::CENTER_CENTER,
        "Y",
        egui::FontId::proportional(13.0),
        y_color,
    );

    // Origin target ring & center dot + "O" label
    painter.circle_stroke(origin_pt, 6.5, Stroke::new(1.5, Color32::from_rgb(255, 215, 70)));
    painter.circle_filled(origin_pt, 3.5, Color32::from_rgb(255, 80, 70));
    let origin_label_pos = origin_pt - x_screen_dir * 14.0 - y_screen_dir * 12.0;
    painter.text(
        origin_label_pos,
        egui::Align2::CENTER_CENTER,
        "Origin",
        egui::FontId::proportional(10.5),
        Color32::from_rgb(255, 215, 90),
    );

    // 6. Render Document Layers & Entities with High-Visibility Glow
    let has_selection = !selection.is_empty();
    let (index_color_map, path_order_map): (
        HashMap<(usize, usize, Option<usize>), Color32>,
        HashMap<(usize, usize, Option<usize>), usize>,
    ) = if state.color_by_cut_index || completed_before_path_1based.is_some() {
        let seq = doc.compute_cut_sequence();
        let total = seq.len();
        let mut cmap = HashMap::new();
        let mut omap = HashMap::new();
        for (order_idx, info) in seq.iter().enumerate() {
            let key = (info.layer_idx, info.entity_idx, info.child_idx);
            if state.color_by_cut_index {
                cmap.entry(key)
                    .or_insert_with(|| state.color_palette.color_for_index(order_idx, total));
            }
            omap.entry(key).or_insert(order_idx + 1);
        }
        (cmap, omap)
    } else {
        (HashMap::new(), HashMap::new())
    };
    let completed_alpha: u8 = (255.0_f32 * 0.30).round() as u8;

    for (l_idx, layer) in doc.layers.iter().enumerate() {
        if !layer.visible {
            continue;
        }

        // Unselected stroke (dimmed slightly if there is an active selection so selection pops out)
        let unsel_alpha = if has_selection { 180 } else { 255 };

        // Selection highlight styles: electric bright gold-yellow with prominent glow
        let sel_glow_stroke = Stroke::new(6.0_f32, Color32::from_rgba_unmultiplied(255, 230, 0, 90));
        let sel_core_stroke = Stroke::new(2.8_f32, Color32::from_rgb(255, 235, 10));

        let layer_color = Color32::from_rgb(layer.color.r, layer.color.g, layer.color.b);
        for (e_idx, entity) in layer.entities.iter().enumerate() {
            let is_selected = selection.contains(&(l_idx, e_idx));
            match entity {
                Entity::Group { children } => {
                    for (c_idx, child) in children.iter().enumerate() {
                        let key = (l_idx, e_idx, Some(c_idx));
                        let ent_color = index_color_map
                            .get(&key)
                            .copied()
                            .unwrap_or(layer_color);
                        let is_completed = completed_before_path_1based.is_some_and(|cutoff| {
                            path_order_map.get(&key).is_some_and(|&idx1| idx1 < cutoff)
                        });
                        let draw_color = if is_completed {
                            desaturate_and_alpha(ent_color, 0.20, completed_alpha)
                        } else {
                            Color32::from_rgba_unmultiplied(
                                ent_color.r(),
                                ent_color.g(),
                                ent_color.b(),
                                unsel_alpha,
                            )
                        };
                        let base_stroke = Stroke::new(
                            if state.color_by_cut_index { 2.0 } else { 1.5_f32 },
                            draw_color,
                        );
                        render_entity(
                            &painter,
                            child,
                            is_selected,
                            draw_color,
                            base_stroke,
                            sel_glow_stroke,
                            sel_core_stroke,
                            state,
                            origin_screen,
                        );
                    }
                }
                _ => {
                    let key = (l_idx, e_idx, None);
                    let ent_color = index_color_map
                        .get(&key)
                        .copied()
                        .unwrap_or(layer_color);
                    let is_completed = completed_before_path_1based.is_some_and(|cutoff| {
                        path_order_map.get(&key).is_some_and(|&idx1| idx1 < cutoff)
                    });
                    let draw_color = if is_completed {
                        desaturate_and_alpha(ent_color, 0.20, completed_alpha)
                    } else {
                        Color32::from_rgba_unmultiplied(
                            ent_color.r(),
                            ent_color.g(),
                            ent_color.b(),
                            unsel_alpha,
                        )
                    };
                    let base_stroke = Stroke::new(
                        if state.color_by_cut_index { 2.0 } else { 1.5_f32 },
                        draw_color,
                    );
                    render_entity(
                        &painter,
                        entity,
                        is_selected,
                        draw_color,
                        base_stroke,
                        sel_glow_stroke,
                        sel_core_stroke,
                        state,
                        origin_screen,
                    );
                }
            }

            // Fill closed Obstacle shapes & draw outward exclusion boundary
            if layer.is_obstacle {
                let fill_color = Color32::from_rgba_unmultiplied(235, 95, 55, 65);
                match entity {
                    Entity::Circle { center, radius } => {
                        let sc = state.world_to_screen(*center, origin_screen);
                        painter.circle_filled(sc, (*radius as f32) * state.zoom, fill_color);
                    }
                    Entity::Polyline { points, closed: true } => {
                        let screen_pts: Vec<Pos2> = points
                            .iter()
                            .map(|p| state.world_to_screen(*p, origin_screen))
                            .collect();
                        draw_filled_polygon(&painter, &screen_pts, fill_color);
                    }
                    Entity::Group { children } => {
                        for child in children {
                            match child {
                                Entity::Circle { center, radius } => {
                                    let sc = state.world_to_screen(*center, origin_screen);
                                    painter.circle_filled(sc, (*radius as f32) * state.zoom, fill_color);
                                }
                                Entity::Polyline { points, closed: true } => {
                                    let screen_pts: Vec<Pos2> = points
                                        .iter()
                                        .map(|p| state.world_to_screen(*p, origin_screen))
                                        .collect();
                                    draw_filled_polygon(&painter, &screen_pts, fill_color);
                                }
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }

                // Draw outward exclusion boundary at exact `exclusion_radius` distance
                let excl_polys = doc.obstacle_exclusion_polygons(l_idx, e_idx);
                let excl_stroke = Stroke::new(
                    1.6_f32,
                    Color32::from_rgba_unmultiplied(255, 175, 55, 200),
                );
                for poly in excl_polys {
                    let screen_pts: Vec<Pos2> = poly
                        .iter()
                        .map(|p| state.world_to_screen(*p, origin_screen))
                        .collect();
                    draw_dashed_closed_poly(&painter, &screen_pts, excl_stroke);
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
        painter.rect_stroke(sel_rect, 2.0, Stroke::new(1.8_f32, Color32::from_rgb(0, 225, 255)), StrokeKind::Outside);

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
            painter.rect_stroke(h_rect, 1.0, Stroke::new(1.2_f32, Color32::from_rgb(20, 20, 25)), StrokeKind::Outside);
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
        painter.rect_stroke(m_rect, 1.0, Stroke::new(1.5_f32, Color32::from_rgb(0, 220, 255)), StrokeKind::Outside);
    }

    // 9. Render Rapid Travel Paths (Travel layer visibility) and Cut Order Badges (show_cut_order)
    let show_travel_paths = doc.layers.iter().find(|l| l.is_travel).map_or(true, |l| l.visible);
    if state.show_cut_order || show_travel_paths {
        let cut_sequence = doc.compute_cut_sequence();

        if show_travel_paths {
            let exclusion_polys = doc.all_obstacle_exclusion_polygons();

            // Retrigger obstacle-avoidance routing whenever any cut endpoint or obstacle position/radius changes
            use std::hash::{Hash, Hasher};
            let mut hasher = std::collections::hash_map::DefaultHasher::new();
            cut_sequence.len().hash(&mut hasher);
            for info in &cut_sequence {
                info.start_point.x.to_bits().hash(&mut hasher);
                info.start_point.y.to_bits().hash(&mut hasher);
                info.end_point.x.to_bits().hash(&mut hasher);
                info.end_point.y.to_bits().hash(&mut hasher);
            }
            exclusion_polys.len().hash(&mut hasher);
            for poly in &exclusion_polys {
                poly.len().hash(&mut hasher);
                for pt in poly {
                    pt.x.to_bits().hash(&mut hasher);
                    pt.y.to_bits().hash(&mut hasher);
                }
            }
            let scene_hash = hasher.finish();

            if state.cached_rapid_routes.0 != Some(scene_hash) {
                let mut routes = Vec::with_capacity(cut_sequence.len().saturating_sub(1));
                for order_idx in 1..cut_sequence.len() {
                    let from = cut_sequence[order_idx - 1].end_point;
                    let to = cut_sequence[order_idx].start_point;
                    routes.push(Document::route_rapid_between(from, to, &exclusion_polys));
                }
                state.cached_rapid_routes = (Some(scene_hash), routes);
            }

            for order_idx in 1..cut_sequence.len() {
                if let Some(waypoints) = state.cached_rapid_routes.1.get(order_idx - 1) {
                    let rapid_color = if waypoints.len() > 2 {
                        Color32::from_rgba_unmultiplied(255, 165, 70, 140)
                    } else {
                        Color32::from_rgba_unmultiplied(0, 200, 255, 45)
                    };
                    for pair in waypoints.windows(2) {
                        let p1 = state.world_to_screen(pair[0], origin_screen);
                        let p2 = state.world_to_screen(pair[1], origin_screen);
                        if p1.distance(p2) > 2.0 {
                            painter.line_segment([p1, p2], Stroke::new(1.2_f32, rapid_color));
                        }
                    }
                }
            }
        }

        if state.show_cut_order {
            for (order_idx, info) in cut_sequence.iter().enumerate() {
                let sc_start = state.world_to_screen(info.start_point, origin_screen);

                // Pierce Starting Point Marker: bright green target dot with outer ring
                painter.circle_stroke(
                    sc_start,
                    4.5,
                    Stroke::new(1.5_f32, Color32::from_rgb(0, 255, 100)),
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
                    Stroke::new(1.2_f32, border_color),
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
            Stroke::new(1.2_f32, Color32::from_rgb(255, 200, 70)),
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
                        Stroke::new(1.8_f32, Color32::from_rgba_unmultiplied(255, 185, 50, 215)),
                    );
                    t += dash + gap;
                }
            }
        }

        painter.circle_stroke(
            pause_screen,
            13.0,
            Stroke::new(2.0_f32, Color32::from_rgb(255, 185, 40)),
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
            Stroke::new(1.5_f32, Color32::from_rgb(255, 240, 170)),
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
            Stroke::new(1.0_f32, Color32::from_rgb(255, 185, 40)),
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

    // 7B. Live Rubberband Preview & Dimension Callout while Drawing
    if state.draw_tool != DrawTool::None {
        if let Some(hover_pos) = response.hover_pos() {
            let curr = state.screen_to_world(hover_pos, origin_screen);
            let preview_stroke = Stroke::new(1.8_f32, Color32::from_rgb(0, 225, 255));
            let mut dim_label: Option<String> = None;

            match state.draw_tool {
                DrawTool::Point => {
                    painter.circle_filled(hover_pos, 4.0, Color32::from_rgb(0, 225, 255));
                    dim_label = Some(format!("Point ({:.1}, {:.1})", curr.x, curr.y));
                }
                DrawTool::Spline => {
                    let mut pts = state.draw_spline_pts.clone();
                    pts.push(curr);
                    if pts.len() >= 2 {
                        let smooth = catmull_rom_spline(&pts, state.draw_spline_closed);
                        let sc: Vec<Pos2> = smooth
                            .iter()
                            .map(|p| state.world_to_screen(*p, origin_screen))
                            .collect();
                        if sc.len() >= 2 {
                            painter.add(egui::Shape::line(sc, preview_stroke));
                        }
                    }
                    for p in &state.draw_spline_pts {
                        painter.circle_filled(
                            state.world_to_screen(*p, origin_screen),
                            3.5,
                            Color32::from_rgb(255, 215, 60),
                        );
                    }
                    dim_label = Some(format!(
                        "Spline ({} pts) · Double-click / Enter to finish",
                        state.draw_spline_pts.len()
                    ));
                }
                _ => {
                    if let Some(anchor) = state.draw_anchor {
                        let sa = state.world_to_screen(anchor, origin_screen);
                        match state.draw_tool {
                            DrawTool::Line => {
                                painter.line_segment([sa, hover_pos], preview_stroke);
                                let len = (curr.x - anchor.x).hypot(curr.y - anchor.y);
                                let ang = (curr.y - anchor.y).atan2(curr.x - anchor.x).to_degrees();
                                dim_label = Some(format!("L: {:.2} mm · {:.1}°", len, ang));
                            }
                            DrawTool::Rectangle => {
                                let rect = Rect::from_two_pos(sa, hover_pos);
                                painter.rect_stroke(rect, 0.0, preview_stroke, StrokeKind::Outside);
                                let w = (curr.x - anchor.x).abs();
                                let h = (curr.y - anchor.y).abs();
                                dim_label = Some(format!("{:.2} × {:.2} mm", w, h));
                            }
                            DrawTool::Circle => {
                                let r_mm = (curr.x - anchor.x).hypot(curr.y - anchor.y);
                                painter.circle_stroke(sa, (r_mm as f32) * state.zoom, preview_stroke);
                                dim_label = Some(format!("R: {:.2} mm (Ø {:.2})", r_mm, r_mm * 2.0));
                            }
                            DrawTool::Ellipse => {
                                let rx = (curr.x - anchor.x).abs();
                                let ry = (curr.y - anchor.y).abs();
                                let ent = make_ellipse_entity(anchor, rx, ry);
                                if let Entity::Polyline { points, .. } = ent {
                                    let mut sc: Vec<Pos2> = points
                                        .iter()
                                        .map(|p| state.world_to_screen(*p, origin_screen))
                                        .collect();
                                    if let Some(&first) = sc.first() {
                                        sc.push(first);
                                    }
                                    painter.add(egui::Shape::line(sc, preview_stroke));
                                }
                                dim_label = Some(format!("Rx: {:.2} × Ry: {:.2} mm", rx, ry));
                            }
                            _ => {}
                        }
                    } else {
                        dim_label = Some(format!("Click start ({:.1}, {:.1})", curr.x, curr.y));
                    }
                }
            }

            if let Some(lbl) = dim_label {
                let text_pos = hover_pos + Vec2::new(14.0, -14.0);
                painter.text(
                    text_pos,
                    egui::Align2::LEFT_BOTTOM,
                    lbl,
                    egui::FontId::proportional(11.5),
                    Color32::from_rgb(0, 235, 255),
                );
            }
        }
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
        painter.circle_stroke(head_pos, 12.0, Stroke::new(2.0_f32, Color32::from_rgb(255, 110, 40)));
    } else if is_dry_cut {
        painter.circle_stroke(head_pos, 11.0, Stroke::new(2.0_f32, Color32::from_rgb(0, 235, 255)));
        painter.circle_filled(head_pos, 3.0, Color32::from_rgb(0, 255, 160));
    }

    painter.line_segment(
        [Pos2::new(head_pos.x - cross_len, head_pos.y), Pos2::new(head_pos.x + cross_len, head_pos.y)],
        Stroke::new(2.0_f32, cross_color),
    );
    painter.line_segment(
        [Pos2::new(head_pos.x, head_pos.y - cross_len), Pos2::new(head_pos.x, head_pos.y + cross_len)],
        Stroke::new(2.0_f32, cross_color),
    );
    painter.circle_stroke(
        head_pos,
        8.0,
        Stroke::new(1.2_f32, cross_color.linear_multiply(0.75)),
    );

    // 10b. Render Millimeter Coordinate Rulers along Top & Left edges
    if state.show_rulers {
        let r_thick = 22.0_f32;
        let vr = response.rect;
        let ruler_bg = Color32::from_rgba_unmultiplied(18, 22, 30, 230);
        let ruler_border = Color32::from_rgb(55, 65, 82);
        let tick_color = Color32::from_rgb(130, 145, 168);
        let label_color = Color32::from_rgb(190, 204, 224);

        let top_rect = Rect::from_min_max(vr.min, Pos2::new(vr.max.x, vr.min.y + r_thick));
        let left_rect = Rect::from_min_max(Pos2::new(vr.min.x, vr.min.y + r_thick), Pos2::new(vr.min.x + r_thick, vr.max.y));
        let corner_rect = Rect::from_min_max(vr.min, Pos2::new(vr.min.x + r_thick, vr.min.y + r_thick));

        painter.rect_filled(top_rect, 0.0, ruler_bg);
        painter.rect_filled(left_rect, 0.0, ruler_bg);
        painter.line_segment(
            [Pos2::new(vr.min.x + r_thick, vr.min.y + r_thick), Pos2::new(vr.max.x, vr.min.y + r_thick)],
            Stroke::new(1.0_f32, ruler_border),
        );
        painter.line_segment(
            [Pos2::new(vr.min.x + r_thick, vr.min.y + r_thick), Pos2::new(vr.min.x + r_thick, vr.max.y)],
            Stroke::new(1.0_f32, ruler_border),
        );

        // Choose adaptive major tick step in mm so labels are ~65-120 px apart
        let target_mm = (80.0_f32 / state.zoom.max(0.01)) as f64;
        let candidates = [1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0];
        let major_step = candidates
            .iter()
            .copied()
            .find(|&c| c >= target_mm)
            .unwrap_or(1000.0);
        let minor_step = major_step / 5.0;

        // Horizontal (X) ruler in machine coordinates (0 at origin_world.x, increasing along x_screen_dir)
        let x_sign = x_screen_dir.x as f64;
        let w_left = state.screen_to_world(Pos2::new(vr.min.x + r_thick, vr.min.y), origin_screen).x;
        let w_right = state.screen_to_world(Pos2::new(vr.max.x, vr.min.y), origin_screen).x;
        let m_left = (w_left - origin_world.x) * x_sign;
        let m_right = (w_right - origin_world.x) * x_sign;
        let (mx_min, mx_max) = (m_left.min(m_right), m_left.max(m_right));
        let k_start = (mx_min / minor_step).floor() as i64;
        let k_end = (mx_max / minor_step).ceil() as i64;
        for k in k_start..=k_end {
            let mx = (k as f64) * minor_step;
            let wx = origin_world.x + mx * x_sign;
            let sx = state.world_to_screen(Point::new(wx, 0.0), origin_screen).x;
            if sx < vr.min.x + r_thick || sx > vr.max.x {
                continue;
            }
            let is_major = k.rem_euclid(5) == 0;
            let tick_h = if is_major { 9.0 } else { 4.5 };
            painter.line_segment(
                [Pos2::new(sx, vr.min.y + r_thick - tick_h), Pos2::new(sx, vr.min.y + r_thick)],
                Stroke::new(1.0_f32, tick_color),
            );
            if is_major {
                painter.text(
                    Pos2::new(sx + 3.0, vr.min.y + 2.0),
                    egui::Align2::LEFT_TOP,
                    format!("{:.0}", mx),
                    egui::FontId::monospace(9.5),
                    label_color,
                );
            }
        }

        // Vertical (Y) ruler in machine coordinates (0 at origin_world.y, increasing along y_screen_dir)
        let y_sign = y_screen_dir.y as f64;
        let w_top = state.screen_to_world(Pos2::new(vr.min.x, vr.min.y + r_thick), origin_screen).y;
        let w_bot = state.screen_to_world(Pos2::new(vr.min.x, vr.max.y), origin_screen).y;
        let m_top = (origin_world.y - w_top) * y_sign;
        let m_bot = (origin_world.y - w_bot) * y_sign;
        let (my_min, my_max) = (m_top.min(m_bot), m_top.max(m_bot));
        let ky_start = (my_min / minor_step).floor() as i64;
        let ky_end = (my_max / minor_step).ceil() as i64;
        for k in ky_start..=ky_end {
            let my = (k as f64) * minor_step;
            let wy = origin_world.y - my * y_sign;
            let sy = state.world_to_screen(Point::new(0.0, wy), origin_screen).y;
            if sy < vr.min.y + r_thick || sy > vr.max.y {
                continue;
            }
            let is_major = k.rem_euclid(5) == 0;
            let tick_w = if is_major { 9.0 } else { 4.5 };
            painter.line_segment(
                [Pos2::new(vr.min.x + r_thick - tick_w, sy), Pos2::new(vr.min.x + r_thick, sy)],
                Stroke::new(1.0_f32, tick_color),
            );
            if is_major {
                painter.text(
                    Pos2::new(vr.min.x + 2.0, sy + 2.0),
                    egui::Align2::LEFT_TOP,
                    format!("{:.0}", my),
                    egui::FontId::monospace(9.0),
                    label_color,
                );
            }
        }

        painter.rect_filled(corner_rect, 0.0, Color32::from_rgb(24, 30, 40));
        painter.text(
            corner_rect.center(),
            egui::Align2::CENTER_CENTER,
            "mm",
            egui::FontId::monospace(8.5),
            Color32::from_rgb(150, 165, 190),
        );
    }

    // 11. Render Right-Click Path Context Menu (when paused or stopped)
    if let Some(menu) = state.context_menu.clone() {
        let menu_screen = state.world_to_screen(menu.point_mm, origin_screen);
        painter.circle_stroke(
            menu_screen,
            9.0,
            Stroke::new(2.0_f32, Color32::from_rgb(90, 220, 255)),
        );
        painter.circle_filled(menu_screen, 3.5, Color32::from_rgb(90, 220, 255));

        let mut close_menu = false;
        let area_resp = egui::Area::new(egui::Id::new("path_context_menu"))
            .order(egui::Order::Foreground)
            .fixed_pos(menu.screen_pos)
            .show(ui.ctx(), |ui| {
                egui::Frame::popup(ui.style())
                    .fill(Color32::from_rgb(28, 31, 38))
                    .stroke(Stroke::new(1.0_f32, Color32::from_rgb(85, 150, 210)))
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(format!(
                                "Path #{} ({:.1}, {:.1} mm)",
                                menu.path_index_1based, menu.point_mm.x, menu.point_mm.y
                            ))
                            .size(11.0)
                            .color(Color32::from_rgb(160, 215, 245)),
                        );
                        ui.add_space(2.0);
                        let btn = egui::Button::new(
                            egui::RichText::new("From this point continue")
                                .size(12.5)
                                .color(Color32::WHITE)
                                .strong(),
                        )
                        .fill(Color32::from_rgb(30, 120, 98));
                        if ui.add(btn).clicked() {
                            state.pending_continue_from_point = Some(menu.point_mm);
                            close_menu = true;
                        }
                        ui.add_space(2.0);
                        let start_btn = egui::Button::new(
                            egui::RichText::new("Set starting point here")
                                .size(12.5)
                                .color(Color32::WHITE)
                                .strong(),
                        )
                        .fill(Color32::from_rgb(45, 92, 145));
                        if ui.add(start_btn).clicked() {
                            state.pending_set_start_point = Some((menu.path_index_1based, menu.point_mm));
                            close_menu = true;
                        }
                    });
            });

        if close_menu || ui.input(|i| i.key_pressed(Key::Escape)) {
            state.context_menu = None;
        } else if !context_menu_just_opened && ui.input(|i| i.pointer.any_pressed()) {
            let press_inside = ui
                .input(|i| i.pointer.interact_pos())
                .map_or(false, |p| area_resp.response.rect.contains(p));
            if !press_inside {
                state.context_menu = None;
            }
        }
    }

    response
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_origin_world_and_axes_corners() {
        let mut cs = CanvasState::default();
        cs.bed_width_mm = 2000.0;
        cs.bed_height_mm = 1000.0;

        cs.origin_corner = "Bottom-Left (X0, Y0)".to_string();
        let (o, x, y) = cs.origin_world_and_axes();
        assert_eq!(o, Point::new(0.0, 0.0));
        assert_eq!(x, Vec2::new(1.0, 0.0));
        assert_eq!(y, Vec2::new(0.0, -1.0));

        cs.origin_corner = "Top-Left (X0, Ymax)".to_string();
        let (o, x, y) = cs.origin_world_and_axes();
        assert_eq!(o, Point::new(0.0, 1000.0));
        assert_eq!(x, Vec2::new(1.0, 0.0));
        assert_eq!(y, Vec2::new(0.0, 1.0));

        cs.origin_corner = "Bottom-Right (Xmax, Y0)".to_string();
        let (o, x, y) = cs.origin_world_and_axes();
        assert_eq!(o, Point::new(2000.0, 0.0));
        assert_eq!(x, Vec2::new(-1.0, 0.0));
        assert_eq!(y, Vec2::new(0.0, -1.0));

        cs.origin_corner = "Top-Right (Xmax, Ymax)".to_string();
        let (o, x, y) = cs.origin_world_and_axes();
        assert_eq!(o, Point::new(2000.0, 1000.0));
        assert_eq!(x, Vec2::new(-1.0, 0.0));
        assert_eq!(y, Vec2::new(0.0, 1.0));
    }
}
