use kurbo::{Affine, Point, Rect};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color {
    pub const RED: Self = Self { r: 255, g: 60, b: 60 };
    pub const GREEN: Self = Self { r: 60, g: 255, b: 60 };
    pub const BLUE: Self = Self { r: 60, g: 140, b: 255 };
    pub const YELLOW: Self = Self { r: 255, g: 220, b: 20 };
    pub const CYAN: Self = Self { r: 0, g: 230, b: 230 };
    pub const MAGENTA: Self = Self { r: 240, g: 60, b: 240 };
    pub const WHITE: Self = Self { r: 255, g: 255, b: 255 };
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CuttingParams {
    pub speed_mm_s: f64,
    pub power_percent: f64,
    pub pierce_time_ms: u32,
    pub gas_type: String,
}

impl Default for CuttingParams {
    fn default() -> Self {
        Self {
            speed_mm_s: 100.0,
            power_percent: 80.0,
            pierce_time_ms: 200,
            gas_type: "Air".to_string(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Entity {
    Line { start: Point, end: Point },
    Polyline { points: Vec<Point>, closed: bool },
    Circle { center: Point, radius: f64 },
    Arc { center: Point, radius: f64, start_angle: f64, end_angle: f64 },
    Group { children: Vec<Entity> },
}

impl Entity {
    pub fn bounding_box(&self) -> Rect {
        match self {
            Entity::Line { start, end } => Rect::from_points(*start, *end),
            Entity::Polyline { points, .. } => {
                if points.is_empty() {
                    return Rect::ZERO;
                }
                let mut min_x = points[0].x;
                let mut max_x = points[0].x;
                let mut min_y = points[0].y;
                let mut max_y = points[0].y;
                for p in points.iter().skip(1) {
                    min_x = min_x.min(p.x);
                    max_x = max_x.max(p.x);
                    min_y = min_y.min(p.y);
                    max_y = max_y.max(p.y);
                }
                Rect::new(min_x, min_y, max_x, max_y)
            }
            Entity::Circle { center, radius } => {
                Rect::new(center.x - radius, center.y - radius, center.x + radius, center.y + radius)
            }
            Entity::Arc { center, radius, .. } => {
                Rect::new(center.x - radius, center.y - radius, center.x + radius, center.y + radius)
            }
            Entity::Group { children } => {
                let mut total: Option<Rect> = None;
                for c in children {
                    let b = c.bounding_box();
                    total = Some(total.map_or(b, |t| t.union(b)));
                }
                total.unwrap_or(Rect::ZERO)
            }
        }
    }

    pub fn transform(&mut self, affine: Affine) {
        match self {
            Entity::Line { start, end } => {
                *start = affine * *start;
                *end = affine * *end;
            }
            Entity::Polyline { points, .. } => {
                for p in points.iter_mut() {
                    *p = affine * *p;
                }
            }
            Entity::Circle { center, .. } => {
                *center = affine * *center;
            }
            Entity::Arc { center, .. } => {
                *center = affine * *center;
            }
            Entity::Group { children } => {
                for c in children.iter_mut() {
                    c.transform(affine);
                }
            }
        }
    }

    pub fn translate(&mut self, dx: f64, dy: f64) {
        self.transform(Affine::translate((dx, dy)));
    }

    /// Starting pierce point for cutting path
    pub fn start_point(&self) -> Point {
        match self {
            Entity::Line { start, .. } => *start,
            Entity::Polyline { points, .. } => points.first().copied().unwrap_or(Point::ORIGIN),
            Entity::Circle { center, radius } => Point::new(center.x + radius, center.y),
            Entity::Arc { center, radius, start_angle, .. } => {
                let rad = start_angle.to_radians();
                Point::new(center.x + radius * rad.cos(), center.y + radius * rad.sin())
            }
            Entity::Group { children } => {
                children.first().map_or(Point::ORIGIN, |c| c.start_point())
            }
        }
    }

    /// End point for cutting path
    pub fn end_point(&self) -> Point {
        match self {
            Entity::Line { end, .. } => *end,
            Entity::Polyline { points, closed } => {
                if *closed {
                    points.first().copied().unwrap_or(Point::ORIGIN)
                } else {
                    points.last().copied().unwrap_or(Point::ORIGIN)
                }
            }
            Entity::Circle { center, radius } => Point::new(center.x + radius, center.y),
            Entity::Arc { center, radius, end_angle, .. } => {
                let rad = end_angle.to_radians();
                Point::new(center.x + radius * rad.cos(), center.y + radius * rad.sin())
            }
            Entity::Group { children } => {
                children.last().map_or(Point::ORIGIN, |c| c.end_point())
            }
        }
    }

    pub fn is_closed(&self) -> bool {
        match self {
            Entity::Line { .. } => false,
            Entity::Polyline { closed, .. } => *closed,
            Entity::Circle { .. } => true,
            Entity::Arc { .. } => false,
            Entity::Group { children } => !children.is_empty() && children.iter().all(|c| c.is_closed()),
        }
    }

    /// Approximate 2D area (positive for closed shapes, 0 for open shapes)
    pub fn area(&self) -> f64 {
        match self {
            Entity::Circle { radius, .. } => std::f64::consts::PI * radius * radius,
            Entity::Polyline { points, closed: true } => {
                if points.len() < 3 {
                    return 0.0;
                }
                let mut sum = 0.0;
                let n = points.len();
                for i in 0..n {
                    let j = (i + 1) % n;
                    sum += points[i].x * points[j].y - points[j].x * points[i].y;
                }
                sum.abs() / 2.0
            }
            Entity::Group { children } => children.iter().map(|c| c.area()).sum(),
            _ => 0.0,
        }
    }

    /// Check if point is inside entity (only for closed shapes)
    pub fn contains_point(&self, pt: Point) -> bool {
        match self {
            Entity::Circle { center, radius } => {
                (pt.x - center.x).hypot(pt.y - center.y) <= *radius
            }
            Entity::Polyline { points, closed: true } => {
                point_in_polygon(pt, points)
            }
            Entity::Group { children } => {
                children.iter().any(|c| c.contains_point(pt))
            }
            _ => false,
        }
    }

    /// Mirror horizontally across a given vertical axis X = center_x
    pub fn mirror_x(&mut self, center_x: f64) {
        match self {
            Entity::Line { start, end } => {
                start.x = 2.0 * center_x - start.x;
                end.x = 2.0 * center_x - end.x;
            }
            Entity::Polyline { points, .. } => {
                for p in points.iter_mut() {
                    p.x = 2.0 * center_x - p.x;
                }
            }
            Entity::Circle { center, .. } => {
                center.x = 2.0 * center_x - center.x;
            }
            Entity::Arc { center, start_angle, end_angle, .. } => {
                center.x = 2.0 * center_x - center.x;
                let s = (180.0 - *end_angle).rem_euclid(360.0);
                let e = (180.0 - *start_angle).rem_euclid(360.0);
                *start_angle = s;
                *end_angle = e;
            }
            Entity::Group { children } => {
                for c in children.iter_mut() {
                    c.mirror_x(center_x);
                }
            }
        }
    }

    /// Mirror vertically across a given horizontal axis Y = center_y
    pub fn mirror_y(&mut self, center_y: f64) {
        match self {
            Entity::Line { start, end } => {
                start.y = 2.0 * center_y - start.y;
                end.y = 2.0 * center_y - end.y;
            }
            Entity::Polyline { points, .. } => {
                for p in points.iter_mut() {
                    p.y = 2.0 * center_y - p.y;
                }
            }
            Entity::Circle { center, .. } => {
                center.y = 2.0 * center_y - center.y;
            }
            Entity::Arc { center, start_angle, end_angle, .. } => {
                center.y = 2.0 * center_y - center.y;
                let s = (360.0 - *end_angle).rem_euclid(360.0);
                let e = (360.0 - *start_angle).rem_euclid(360.0);
                *start_angle = s;
                *end_angle = e;
            }
            Entity::Group { children } => {
                for c in children.iter_mut() {
                    c.mirror_y(center_y);
                }
            }
        }
    }

    /// Calculate distance from point to the entity's path/perimeter itself.
    pub fn distance_to_point(&self, p: Point) -> f64 {
        match self {
            Entity::Line { start, end } => dist_point_to_segment(p, *start, *end),
            Entity::Polyline { points, closed } => {
                if points.is_empty() {
                    return f64::MAX;
                }
                if points.len() == 1 {
                    return (p.x - points[0].x).hypot(p.y - points[0].y);
                }
                let mut min_d = f64::MAX;
                for i in 0..points.len() - 1 {
                    min_d = min_d.min(dist_point_to_segment(p, points[i], points[i + 1]));
                }
                if *closed && points.len() > 2 {
                    min_d = min_d.min(dist_point_to_segment(p, *points.last().unwrap(), points[0]));
                }
                min_d
            }
            Entity::Circle { center, radius } => {
                let d_center = (p.x - center.x).hypot(p.y - center.y);
                (d_center - *radius).abs()
            }
            Entity::Arc {
                center,
                radius,
                start_angle,
                end_angle,
            } => {
                let d_center = (p.x - center.x).hypot(p.y - center.y);
                let angle_deg = (p.y - center.y).atan2(p.x - center.x).to_degrees().rem_euclid(360.0);
                let s = start_angle.rem_euclid(360.0);
                let mut e = end_angle.rem_euclid(360.0);
                if e <= s {
                    e += 360.0;
                }
                let in_arc = (angle_deg >= s && angle_deg <= e) || (angle_deg + 360.0 >= s && angle_deg + 360.0 <= e);

                if in_arc {
                    (d_center - *radius).abs()
                } else {
                    let s_rad = start_angle.to_radians();
                    let e_rad = end_angle.to_radians();
                    let p_start = Point::new(center.x + radius * s_rad.cos(), center.y + radius * s_rad.sin());
                    let p_end = Point::new(center.x + radius * e_rad.cos(), center.y + radius * e_rad.sin());
                    (p.x - p_start.x).hypot(p.y - p_start.y).min((p.x - p_end.x).hypot(p.y - p_end.y))
                }
            }
            Entity::Group { children } => {
                children.iter().map(|c| c.distance_to_point(p)).fold(f64::MAX, f64::min)
            }
        }
    }
}

/// Ray-casting point-in-polygon test
fn point_in_polygon(p: Point, poly: &[Point]) -> bool {
    let mut inside = false;
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let pi = poly[i];
        let pj = poly[j];
        if ((pi.y > p.y) != (pj.y > p.y))
            && (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y + 1e-12) + pi.x)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

fn dist_point_to_segment(p: Point, a: Point, b: Point) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-9 {
        return (p.x - a.x).hypot(p.y - a.y);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq).clamp(0.0, 1.0);
    let proj_x = a.x + t * dx;
    let proj_y = a.y + t * dy;
    (p.x - proj_x).hypot(p.y - proj_y)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Layer {
    pub name: String,
    pub color: Color,
    pub visible: bool,
    pub params: CuttingParams,
    pub entities: Vec<Entity>,
}

impl Layer {
    pub fn new(name: impl Into<String>, color: Color) -> Self {
        Self {
            name: name.into(),
            color,
            visible: true,
            params: CuttingParams::default(),
            entities: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SortMode {
    /// 1. Bend / Open lines -> 2. Interior holes -> 3. Outer contours (Greedy TSP within each group)
    OpenFirst,
    /// 1. Interior holes -> 2. Outer contours -> 3. Open lines (Greedy TSP within each group)
    InnerFirst,
    /// Pure shortest rapid travel distance (Greedy TSP across all paths)
    ShortestPath,
    /// Spatial sort from Left to Right (X ascending)
    LeftToRight,
    /// Spatial sort from Top to Bottom (Y descending)
    TopToBottom,
    /// Manual / Document order (preserve entity list order without automatic optimization)
    Manual,
}

impl Default for SortMode {
    fn default() -> Self {
        SortMode::OpenFirst
    }
}

impl SortMode {
    pub fn label(&self) -> &'static str {
        match self {
            SortMode::OpenFirst => "Open / Bend First",
            SortMode::InnerFirst => "Inner Holes First",
            SortMode::ShortestPath => "Shortest Travel (TSP)",
            SortMode::LeftToRight => "Left to Right (X)",
            SortMode::TopToBottom => "Top to Bottom (Y)",
            SortMode::Manual => "Manual / Document Order",
        }
    }

    pub fn description(&self) -> &'static str {
        match self {
            SortMode::OpenFirst => "Cut open/bend lines first, then interior cutouts, outer contours last",
            SortMode::InnerFirst => "Cut interior holes first to prevent part movement, then outer contours",
            SortMode::ShortestPath => "Minimize rapid transit travel distance between pierces",
            SortMode::LeftToRight => "Sort cut paths from left to right across table",
            SortMode::TopToBottom => "Sort cut paths from top to bottom across table",
            SortMode::Manual => "Preserve exact entity list order without automatic optimization",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Document {
    pub layers: Vec<Layer>,
    #[serde(default)]
    pub sort_mode: SortMode,
    #[serde(default)]
    pub manual_sequence: Vec<(usize, usize)>,
}

impl Default for Document {
    fn default() -> Self {
        Self::new()
    }
}

impl Document {
    pub fn new() -> Self {
        Self {
            layers: vec![
                Layer::new("Cut 1", Color::RED),
                Layer::new("Mark / Engrave", Color::BLUE),
            ],
            sort_mode: SortMode::default(),
            manual_sequence: Vec::new(),
        }
    }

    pub fn bounding_box(&self) -> Option<Rect> {
        let mut total: Option<Rect> = None;
        for layer in &self.layers {
            if !layer.visible { continue; }
            for ent in &layer.entities {
                let b = ent.bounding_box();
                total = Some(total.map_or(b, |t| t.union(b)));
            }
        }
        total
    }

    pub fn entity_count(&self) -> usize {
        self.layers.iter().map(|l| l.entities.len()).sum()
    }

    /// Hit-test across all visible layers. Returns (layer_idx, entity_idx) of closest entity within tolerance.
    pub fn hit_test(&self, pt: Point, tolerance_mm: f64) -> Option<(usize, usize)> {
        let mut closest: Option<((usize, usize), f64)> = None;

        for (l_idx, layer) in self.layers.iter().enumerate() {
            if !layer.visible { continue; }
            for (e_idx, entity) in layer.entities.iter().enumerate() {
                let d = entity.distance_to_point(pt);
                if d <= tolerance_mm {
                    match closest {
                        Some((_, cur_min)) if d < cur_min => {
                            closest = Some(((l_idx, e_idx), d));
                        }
                        None => {
                            closest = Some(((l_idx, e_idx), d));
                        }
                        _ => {}
                    }
                }
            }
        }

        closest.map(|(idx, _)| idx)
    }

    /// Compute composite bounding box of a selection set.
    pub fn selection_bounding_box(&self, selection: &[(usize, usize)]) -> Option<Rect> {
        let mut total: Option<Rect> = None;
        for &(l_idx, e_idx) in selection {
            if let Some(layer) = self.layers.get(l_idx) {
                if let Some(entity) = layer.entities.get(e_idx) {
                    let b = entity.bounding_box();
                    total = Some(total.map_or(b, |t| t.union(b)));
                }
            }
        }
        total
    }

    /// Translate all selected entities by (dx, dy) in millimeters.
    pub fn translate_selection(&mut self, selection: &[(usize, usize)], dx: f64, dy: f64) {
        if dx.abs() < 1e-9 && dy.abs() < 1e-9 {
            return;
        }
        for &(l_idx, e_idx) in selection {
            if let Some(layer) = self.layers.get_mut(l_idx) {
                if let Some(entity) = layer.entities.get_mut(e_idx) {
                    entity.translate(dx, dy);
                }
            }
        }
    }

    /// Mirror selected entities horizontally across their bounding box center.
    pub fn mirror_selection_horizontal(&mut self, selection: &[(usize, usize)]) {
        let Some(bounds) = self.selection_bounding_box(selection) else { return; };
        let center_x = (bounds.min_x() + bounds.max_x()) / 2.0;
        for &(l_idx, e_idx) in selection {
            if let Some(layer) = self.layers.get_mut(l_idx) {
                if let Some(entity) = layer.entities.get_mut(e_idx) {
                    entity.mirror_x(center_x);
                }
            }
        }
    }

    /// Mirror selected entities vertically across their bounding box center.
    pub fn mirror_selection_vertical(&mut self, selection: &[(usize, usize)]) {
        let Some(bounds) = self.selection_bounding_box(selection) else { return; };
        let center_y = (bounds.min_y() + bounds.max_y()) / 2.0;
        for &(l_idx, e_idx) in selection {
            if let Some(layer) = self.layers.get_mut(l_idx) {
                if let Some(entity) = layer.entities.get_mut(e_idx) {
                    entity.mirror_y(center_y);
                }
            }
        }
    }

    /// Duplicate selected entities translated by (dx, dy). Returns the indices of the new entities.
    pub fn duplicate_selection(&mut self, selection: &[(usize, usize)], dx: f64, dy: f64) -> Vec<(usize, usize)> {
        let mut new_selection = Vec::new();

        for &(l_idx, e_idx) in selection {
            if let Some(layer) = self.layers.get(l_idx) {
                if let Some(entity) = layer.entities.get(e_idx) {
                    let mut cloned = entity.clone();
                    cloned.translate(dx, dy);
                    let target_layer = &mut self.layers[l_idx];
                    let new_idx = target_layer.entities.len();
                    target_layer.entities.push(cloned);
                    new_selection.push((l_idx, new_idx));
                }
            }
        }

        new_selection
    }

    /// Create an Nx by Ny matrix array copy with specified spacing pitch (gap + size).
    pub fn array_copy(
        &mut self,
        selection: &[(usize, usize)],
        cols: usize,
        rows: usize,
        col_pitch: f64,
        row_pitch: f64,
    ) -> Vec<(usize, usize)> {
        let mut all_indices = selection.to_vec();

        for r in 0..rows {
            for c in 0..cols {
                if r == 0 && c == 0 {
                    continue; // Base item is already at (0, 0)
                }
                let dx = (c as f64) * col_pitch;
                let dy = (r as f64) * row_pitch;
                let copies = self.duplicate_selection(selection, dx, dy);
                all_indices.extend(copies);
            }
        }

        all_indices
    }

    /// Delete selected entities from the document.
    pub fn delete_selection(&mut self, selection: &[(usize, usize)]) {
        self.manual_sequence.clear();
        let mut to_remove: HashMap<usize, Vec<usize>> = HashMap::new();
        for &(l, e) in selection {
            to_remove.entry(l).or_default().push(e);
        }
        for (l_idx, mut e_indices) in to_remove {
            if let Some(layer) = self.layers.get_mut(l_idx) {
                e_indices.sort_unstable_by(|a, b| b.cmp(a)); // remove in reverse order
                for e in e_indices {
                    if e < layer.entities.len() {
                        layer.entities.remove(e);
                    }
                }
            }
        }
    }

    /// Group selected entities into a single compound object.
    /// All relative positions of the profiles stay strictly fixed.
    /// Updates selection to point to the new single grouped entity.
    pub fn group_selection(&mut self, selection: &mut Vec<(usize, usize)>) -> Option<(usize, usize)> {
        if selection.len() <= 1 {
            return selection.first().copied();
        }

        let target_layer_idx = selection[0].0;

        let mut to_remove: HashMap<usize, Vec<usize>> = HashMap::new();
        for &(l, e) in selection.iter() {
            to_remove.entry(l).or_default().push(e);
        }

        let mut grouped_children = Vec::new();

        for (l_idx, mut e_indices) in to_remove {
            e_indices.sort_unstable_by(|a, b| b.cmp(a));
            if let Some(layer) = self.layers.get_mut(l_idx) {
                for e_idx in e_indices {
                    if e_idx < layer.entities.len() {
                        let removed = layer.entities.remove(e_idx);
                        if let Entity::Group { children } = removed {
                            grouped_children.extend(children);
                        } else {
                            grouped_children.push(removed);
                        }
                    }
                }
            }
        }

        if grouped_children.is_empty() {
            return None;
        }
        grouped_children.reverse();

        let new_group = Entity::Group { children: grouped_children };
        let new_idx = self.layers[target_layer_idx].entities.len();
        self.layers[target_layer_idx].entities.push(new_group);

        *selection = vec![(target_layer_idx, new_idx)];
        Some((target_layer_idx, new_idx))
    }

    /// Ungroup any selected Group entities back into individual entities.
    pub fn ungroup_selection(&mut self, selection: &mut Vec<(usize, usize)>) {
        let mut new_selection = Vec::new();

        let mut to_unpack: HashMap<usize, Vec<usize>> = HashMap::new();
        for &(l, e) in selection.iter() {
            to_unpack.entry(l).or_default().push(e);
        }

        for (l_idx, mut e_indices) in to_unpack {
            e_indices.sort_unstable_by(|a, b| b.cmp(a));
            if let Some(layer) = self.layers.get_mut(l_idx) {
                for e_idx in e_indices {
                    if e_idx < layer.entities.len() {
                        if matches!(layer.entities[e_idx], Entity::Group { .. }) {
                            if let Entity::Group { children } = layer.entities.remove(e_idx) {
                                for child in children {
                                    let new_idx = layer.entities.len();
                                    layer.entities.push(child);
                                    new_selection.push((l_idx, new_idx));
                                }
                            }
                        } else {
                            new_selection.push((l_idx, e_idx));
                        }
                    }
                }
            }
        }

        if !new_selection.is_empty() {
            *selection = new_selection;
        }
    }

    /// Check if the selection contains any Group entity
    pub fn selection_has_group(&self, selection: &[(usize, usize)]) -> bool {
        for &(l, e) in selection {
            if let Some(layer) = self.layers.get(l) {
                if let Some(Entity::Group { .. }) = layer.entities.get(e) {
                    return true;
                }
            }
        }
        false
    }

    /// Auto-detects and classifies profiles into Outer Profiles, Interior Profiles, and Bend / Open Lines.
    pub fn auto_classify_profiles(&mut self) {
        self.manual_sequence.clear();
        let mut all_entities = Vec::new();
        for layer in self.layers.drain(..) {
            all_entities.extend(layer.entities);
        }

        let chained = chain_connected_entities(all_entities, 0.1);

        let mut closed_shapes: Vec<Entity> = Vec::new();
        let mut open_paths: Vec<Entity> = Vec::new();

        for ent in chained {
            if ent.is_closed() {
                closed_shapes.push(ent);
            } else {
                open_paths.push(ent);
            }
        }

        // Sort closed shapes by area descending (outer perimeters have larger area)
        closed_shapes.sort_by(|a, b| b.area().partial_cmp(&a.area()).unwrap_or(std::cmp::Ordering::Equal));

        let mut outer_shapes: Vec<Entity> = Vec::new();
        let mut interior_shapes: Vec<Entity> = Vec::new();

        for shape in closed_shapes {
            let test_pt = match &shape {
                Entity::Circle { center, .. } => *center,
                Entity::Polyline { points, .. } => {
                    if points.len() >= 3 {
                        Point::new(
                            (points[0].x + points[1].x + points[2].x) / 3.0,
                            (points[0].y + points[1].y + points[2].y) / 3.0,
                        )
                    } else if !points.is_empty() {
                        points[0]
                    } else {
                        Point::ORIGIN
                    }
                }
                _ => Point::ORIGIN,
            };

            let is_inside_outer = outer_shapes.iter().any(|outer| outer.contains_point(test_pt));

            if is_inside_outer {
                interior_shapes.push(shape);
            } else {
                outer_shapes.push(shape);
            }
        }

        let mut new_layers = Vec::new();

        if !outer_shapes.is_empty() {
            let mut outer_layer = Layer::new("Outer Profiles", Color::GREEN);
            outer_layer.entities = outer_shapes;
            new_layers.push(outer_layer);
        }

        if !interior_shapes.is_empty() {
            let mut inner_layer = Layer::new("Interior Profiles", Color::YELLOW);
            inner_layer.entities = interior_shapes;
            new_layers.push(inner_layer);
        }

        if !open_paths.is_empty() {
            let mut bend_layer = Layer::new("Bend / Open Lines", Color::MAGENTA);
            bend_layer.entities = open_paths;
            new_layers.push(bend_layer);
        }

        if new_layers.is_empty() {
            new_layers.push(Layer::new("Outer Profiles", Color::GREEN));
        }

        self.layers = new_layers;
    }

    /// Computes the toolpath cutting sequence using the document's currently selected SortMode.
    pub fn compute_cut_sequence(&self) -> Vec<CutPathInfo> {
        self.compute_cut_sequence_with_mode(self.sort_mode)
    }

    /// Computes the toolpath cutting sequence under a specific SortMode.
    /// Supports: OpenFirst, InnerFirst, ShortestPath (TSP), LeftToRight, TopToBottom, and Manual order.
    pub fn compute_cut_sequence_with_mode(&self, mode: SortMode) -> Vec<CutPathInfo> {
        let mut raw_paths = Vec::new();

        for (l_idx, layer) in self.layers.iter().enumerate() {
            if !layer.visible {
                continue;
            }
            let name_lower = layer.name.to_lowercase();
            let is_bend_layer = name_lower.contains("bend")
                || name_lower.contains("mark")
                || name_lower.contains("engrave");
            let is_interior_layer = name_lower.contains("interior")
                || name_lower.contains("inner")
                || name_lower.contains("hole");

            for (e_idx, entity) in layer.entities.iter().enumerate() {
                match entity {
                    Entity::Group { children } => {
                        for child in children {
                            let open = !child.is_closed();
                            let is_bend = is_bend_layer || open;
                            let is_interior = is_interior_layer;
                            raw_paths.push(CutPathInfo {
                                layer_idx: l_idx,
                                entity_idx: e_idx,
                                start_point: child.start_point(),
                                end_point: child.end_point(),
                                is_interior,
                                is_open: is_bend,
                            });
                        }
                    }
                    _ => {
                        let open = !entity.is_closed();
                        let is_bend = is_bend_layer || open;
                        let is_interior = is_interior_layer;
                        raw_paths.push(CutPathInfo {
                            layer_idx: l_idx,
                            entity_idx: e_idx,
                            start_point: entity.start_point(),
                            end_point: entity.end_point(),
                            is_interior,
                            is_open: is_bend,
                        });
                    }
                }
            }
        }

        // Geometric containment detection:
        // Even if layers haven't been segregated into "Interior Profiles", detect if a closed shape
        // is nested inside another closed shape on any visible layer.
        for item in &mut raw_paths {
            if !item.is_open && !item.is_interior {
                let test_pt = item.start_point;
                for (other_l_idx, other_layer) in self.layers.iter().enumerate() {
                    if !other_layer.visible { continue; }
                    for (other_e_idx, other_ent) in other_layer.entities.iter().enumerate() {
                        if item.layer_idx == other_l_idx && item.entity_idx == other_e_idx {
                            continue;
                        }
                        if other_ent.is_closed() && other_ent.contains_point(test_pt) {
                            item.is_interior = true;
                            break;
                        }
                    }
                    if item.is_interior { break; }
                }
            }
        }

        match mode {
            SortMode::Manual => {
                if self.manual_sequence.is_empty() {
                    raw_paths
                } else {
                    let mut result = Vec::with_capacity(raw_paths.len());
                    let mut used = vec![false; raw_paths.len()];
                    for &(l_idx, e_idx) in &self.manual_sequence {
                        for (i, item) in raw_paths.iter().enumerate() {
                            if !used[i] && item.layer_idx == l_idx && item.entity_idx == e_idx {
                                result.push(*item);
                                used[i] = true;
                            }
                        }
                    }
                    for (i, item) in raw_paths.into_iter().enumerate() {
                        if !used[i] {
                            result.push(item);
                        }
                    }
                    result
                }
            }
            SortMode::LeftToRight => {
                raw_paths.sort_by(|a, b| a.start_point.x.partial_cmp(&b.start_point.x).unwrap_or(std::cmp::Ordering::Equal));
                raw_paths
            }
            SortMode::TopToBottom => {
                raw_paths.sort_by(|a, b| b.start_point.y.partial_cmp(&a.start_point.y).unwrap_or(std::cmp::Ordering::Equal));
                raw_paths
            }
            SortMode::ShortestPath => {
                let mut result = Vec::with_capacity(raw_paths.len());
                let mut current_pos = Point::ORIGIN;
                Self::sequence_group_nearest_neighbor(&mut raw_paths, &mut current_pos, &mut result);
                result
            }
            SortMode::OpenFirst => {
                let mut bend_paths = Vec::new();
                let mut interior_paths = Vec::new();
                let mut outer_paths = Vec::new();

                for item in raw_paths {
                    if item.is_open {
                        bend_paths.push(item);
                    } else if item.is_interior {
                        interior_paths.push(item);
                    } else {
                        outer_paths.push(item);
                    }
                }

                let mut result = Vec::with_capacity(bend_paths.len() + interior_paths.len() + outer_paths.len());
                let mut current_pos = Point::ORIGIN;
                Self::sequence_group_nearest_neighbor(&mut bend_paths, &mut current_pos, &mut result);
                Self::sequence_group_nearest_neighbor(&mut interior_paths, &mut current_pos, &mut result);
                Self::sequence_group_nearest_neighbor(&mut outer_paths, &mut current_pos, &mut result);
                result
            }
            SortMode::InnerFirst => {
                let mut bend_paths = Vec::new();
                let mut interior_paths = Vec::new();
                let mut outer_paths = Vec::new();

                for item in raw_paths {
                    if item.is_open {
                        bend_paths.push(item);
                    } else if item.is_interior {
                        interior_paths.push(item);
                    } else {
                        outer_paths.push(item);
                    }
                }

                let mut result = Vec::with_capacity(bend_paths.len() + interior_paths.len() + outer_paths.len());
                let mut current_pos = Point::ORIGIN;
                Self::sequence_group_nearest_neighbor(&mut interior_paths, &mut current_pos, &mut result);
                Self::sequence_group_nearest_neighbor(&mut outer_paths, &mut current_pos, &mut result);
                Self::sequence_group_nearest_neighbor(&mut bend_paths, &mut current_pos, &mut result);
                result
            }
        }
    }

    /// Reorders the entities within layers to permanently match the computed sort sequence.
    pub fn apply_sort_order(&mut self, mode: SortMode) {
        self.sort_mode = mode;
        let seq = self.compute_cut_sequence_with_mode(mode);
        self.manual_sequence.clear();

        for (l_idx, layer) in self.layers.iter_mut().enumerate() {
            let mut ordered_entities = Vec::with_capacity(layer.entities.len());
            let mut used = vec![false; layer.entities.len()];

            for info in &seq {
                if info.layer_idx == l_idx && info.entity_idx < layer.entities.len() && !used[info.entity_idx] {
                    ordered_entities.push(layer.entities[info.entity_idx].clone());
                    used[info.entity_idx] = true;
                }
            }

            // Append any unvisited entities (e.g. from hidden layers)
            for (i, ent) in layer.entities.drain(..).enumerate() {
                if !used[i] {
                    ordered_entities.push(ent);
                }
            }
            layer.entities = ordered_entities;
        }
    }

    /// Assigns a specific 1-based cut order index to `(layer_idx, entity_idx)` in manual sort mode.
    pub fn assign_manual_cut_index(&mut self, layer_idx: usize, entity_idx: usize, target_1based: usize) {
        self.sort_mode = SortMode::Manual;
        if self.manual_sequence.is_empty() {
            let current = self.compute_cut_sequence_with_mode(SortMode::Manual);
            for info in current {
                let pair = (info.layer_idx, info.entity_idx);
                if !self.manual_sequence.contains(&pair) {
                    self.manual_sequence.push(pair);
                }
            }
        }

        self.manual_sequence.retain(|&p| p != (layer_idx, entity_idx));
        let insert_pos = target_1based.saturating_sub(1).min(self.manual_sequence.len());
        self.manual_sequence.insert(insert_pos, (layer_idx, entity_idx));
    }

    /// Move selected entities earlier in their layer entity sequence (for manual order adjustments).
    pub fn move_selection_earlier(&mut self, selection: &mut [(usize, usize)]) -> bool {
        let mut changed = false;
        for (l_idx, e_idx) in selection.iter_mut() {
            if *e_idx > 0 {
                if let Some(layer) = self.layers.get_mut(*l_idx) {
                    if *e_idx < layer.entities.len() {
                        layer.entities.swap(*e_idx, *e_idx - 1);
                        *e_idx -= 1;
                        changed = true;
                    }
                }
            }
        }
        changed
    }

    /// Move selected entities later in their layer entity sequence (for manual order adjustments).
    pub fn move_selection_later(&mut self, selection: &mut [(usize, usize)]) -> bool {
        let mut changed = false;
        for (l_idx, e_idx) in selection.iter_mut().rev() {
            if let Some(layer) = self.layers.get_mut(*l_idx) {
                if *e_idx + 1 < layer.entities.len() {
                    layer.entities.swap(*e_idx, *e_idx + 1);
                    *e_idx += 1;
                    changed = true;
                }
            }
        }
        changed
    }

    fn sequence_group_nearest_neighbor(
        group: &mut Vec<CutPathInfo>,
        current_pos: &mut Point,
        result: &mut Vec<CutPathInfo>,
    ) {
        while !group.is_empty() {
            let mut best_idx = 0;
            let mut best_dist = f64::MAX;

            for (i, item) in group.iter().enumerate() {
                let d = (item.start_point.x - current_pos.x).hypot(item.start_point.y - current_pos.y);
                if d < best_dist {
                    best_dist = d;
                    best_idx = i;
                }
            }

            let next_cut = group.swap_remove(best_idx);
            *current_pos = next_cut.end_point;
            result.push(next_cut);
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CutPathInfo {
    pub layer_idx: usize,
    pub entity_idx: usize,
    pub start_point: Point,
    pub end_point: Point,
    pub is_interior: bool,
    pub is_open: bool,
}

/// Chaining algorithm to connect contiguous lines and arcs into polylines.
pub fn chain_connected_entities(entities: Vec<Entity>, tolerance_mm: f64) -> Vec<Entity> {
    let mut closed_result = Vec::new();
    let mut chains: Vec<Vec<Point>> = Vec::new();

    for ent in entities {
        match ent {
            Entity::Group { children } => {
                closed_result.push(Entity::Group { children });
            }
            Entity::Circle { .. } => {
                closed_result.push(ent);
            }
            Entity::Polyline { points, closed } => {
                if closed {
                    closed_result.push(Entity::Polyline { points, closed });
                } else if points.len() >= 2 {
                    chains.push(points);
                }
            }
            Entity::Line { start, end } => {
                chains.push(vec![start, end]);
            }
            Entity::Arc {
                center,
                radius,
                start_angle,
                end_angle,
            } => {
                let steps = 16;
                let s = start_angle;
                let mut e = end_angle;
                if e < s {
                    e += 360.0;
                }
                let da = (e - s) / (steps as f64);
                let mut pts = Vec::with_capacity(steps + 1);
                for step in 0..=steps {
                    let ang = (s + (step as f64) * da).to_radians();
                    pts.push(Point::new(
                        center.x + radius * ang.cos(),
                        center.y + radius * ang.sin(),
                    ));
                }
                chains.push(pts);
            }
        }
    }

    let tol_sq = tolerance_mm * tolerance_mm;
    let dist_sq = |p1: Point, p2: Point| {
        let dx = p1.x - p2.x;
        let dy = p1.y - p2.y;
        dx * dx + dy * dy
    };

    let mut merged = true;
    while merged {
        merged = false;
        let n = chains.len();
        'outer: for i in 0..n {
            if chains[i].is_empty() {
                continue;
            }
            for j in 0..n {
                if i == j || chains[j].is_empty() {
                    continue;
                }

                let c1_start = chains[i][0];
                let c1_end = *chains[i].last().unwrap();
                let c2_start = chains[j][0];
                let c2_end = *chains[j].last().unwrap();

                // 1. c1 end -> c2 start
                if dist_sq(c1_end, c2_start) <= tol_sq {
                    let mut c2 = std::mem::take(&mut chains[j]);
                    chains[i].extend(c2.drain(1..));
                    merged = true;
                    break 'outer;
                }
                // 2. c1 end -> c2 end (reverse c2)
                else if dist_sq(c1_end, c2_end) <= tol_sq {
                    let mut c2 = std::mem::take(&mut chains[j]);
                    c2.reverse();
                    chains[i].extend(c2.drain(1..));
                    merged = true;
                    break 'outer;
                }
                // 3. c1 start -> c2 end (prepend c2)
                else if dist_sq(c1_start, c2_end) <= tol_sq {
                    let mut c2 = std::mem::take(&mut chains[j]);
                    c2.extend(chains[i].drain(1..));
                    chains[i] = c2;
                    merged = true;
                    break 'outer;
                }
                // 4. c1 start -> c2 start (reverse c2, prepend)
                else if dist_sq(c1_start, c2_start) <= tol_sq {
                    let mut c2 = std::mem::take(&mut chains[j]);
                    c2.reverse();
                    c2.extend(chains[i].drain(1..));
                    chains[i] = c2;
                    merged = true;
                    break 'outer;
                }
            }
        }
    }

    for mut chain in chains {
        if chain.len() < 2 {
            continue;
        }

        let is_closed = dist_sq(chain[0], *chain.last().unwrap()) <= tol_sq && chain.len() >= 3;
        if is_closed {
            chain.pop();
            closed_result.push(Entity::Polyline {
                points: chain,
                closed: true,
            });
        } else if chain.len() == 2 {
            closed_result.push(Entity::Line {
                start: chain[0],
                end: chain[1],
            });
        } else {
            closed_result.push(Entity::Polyline {
                points: chain,
                closed: false,
            });
        }
    }

    closed_result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cut_sequence_interior_before_outer() {
        let mut doc = Document::default();
        let mut outer_layer = Layer::new("Outer Profiles", Color::GREEN);
        outer_layer.entities.push(Entity::Polyline {
            points: vec![
                Point::new(0.0, 0.0),
                Point::new(100.0, 0.0),
                Point::new(100.0, 100.0),
                Point::new(0.0, 100.0),
            ],
            closed: true,
        });

        let mut inner_layer = Layer::new("Interior Profiles", Color::YELLOW);
        inner_layer.entities.push(Entity::Circle {
            center: Point::new(50.0, 50.0),
            radius: 10.0,
        });

        doc.layers = vec![outer_layer, inner_layer];

        let seq = doc.compute_cut_sequence();
        assert_eq!(seq.len(), 2);
        // The interior hole must be cut first!
        assert!(seq[0].is_interior);
        assert_eq!(seq[0].layer_idx, 1);
        // The outer perimeter must be cut second!
        assert!(!seq[1].is_interior);
        assert_eq!(seq[1].layer_idx, 0);
    }

    #[test]
    fn test_group_and_ungroup_selection() {
        let mut doc = Document::default();
        let mut layer = Layer::new("Test", Color::RED);
        layer.entities.push(Entity::Circle {
            center: Point::new(10.0, 10.0),
            radius: 5.0,
        });
        layer.entities.push(Entity::Circle {
            center: Point::new(50.0, 50.0),
            radius: 5.0,
        });
        doc.layers = vec![layer];

        let mut selection = vec![(0, 0), (0, 1)];
        assert_eq!(doc.selection_has_group(&selection), false);

        // Group the selection
        let group_res = doc.group_selection(&mut selection);
        assert!(group_res.is_some());
        assert_eq!(selection.len(), 1);
        assert_eq!(doc.layers[0].entities.len(), 1);
        assert_eq!(doc.selection_has_group(&selection), true);

        // Check group bounding box
        let bb = doc.layers[0].entities[0].bounding_box();
        assert!((bb.x0 - 5.0).abs() < 1e-4);
        assert!((bb.x1 - 55.0).abs() < 1e-4);

        // Translate the group by (20, -10)
        doc.translate_selection(&selection, 20.0, -10.0);

        // Ungroup
        doc.ungroup_selection(&mut selection);
        assert_eq!(doc.layers[0].entities.len(), 2);
        assert_eq!(selection.len(), 2);
        assert_eq!(doc.selection_has_group(&selection), false);

        // Verify the two circles maintained relative offset and translated correctly
        if let Entity::Circle { center, radius } = &doc.layers[0].entities[0] {
            assert!((center.x - 30.0).abs() < 1e-4);
            assert!((center.y - 0.0).abs() < 1e-4);
            assert_eq!(*radius, 5.0);
        } else {
            panic!("Expected circle");
        }

        if let Entity::Circle { center, radius } = &doc.layers[0].entities[1] {
            assert!((center.x - 70.0).abs() < 1e-4);
            assert!((center.y - 40.0).abs() < 1e-4);
            assert_eq!(*radius, 5.0);
        } else {
            panic!("Expected circle");
        }
    }

    #[test]
    fn test_sorting_modes_open_first_and_inner_first_and_manual() {
        let mut doc = Document::default();
        let mut layer = Layer::new("All", Color::WHITE);
        // Outer box 0..100
        layer.entities.push(Entity::Polyline {
            points: vec![
                Point::new(0.0, 0.0),
                Point::new(100.0, 0.0),
                Point::new(100.0, 100.0),
                Point::new(0.0, 100.0),
            ],
            closed: true,
        });
        // Open line (e.g. bend/score line)
        layer.entities.push(Entity::Line {
            start: Point::new(10.0, 20.0),
            end: Point::new(90.0, 20.0),
        });
        // Inner hole circle at 50, 50
        layer.entities.push(Entity::Circle {
            center: Point::new(50.0, 50.0),
            radius: 8.0,
        });
        doc.layers = vec![layer];

        // 1. OpenFirst: Open line -> Inner circle -> Outer contour
        let seq_open = doc.compute_cut_sequence_with_mode(SortMode::OpenFirst);
        assert_eq!(seq_open.len(), 3);
        assert!(seq_open[0].is_open); // Open line first
        assert!(seq_open[1].is_interior); // Inner hole next
        assert!(!seq_open[2].is_interior && !seq_open[2].is_open); // Outer last

        // 2. InnerFirst: Inner circle -> Outer contour -> Open line
        let seq_inner = doc.compute_cut_sequence_with_mode(SortMode::InnerFirst);
        assert_eq!(seq_inner.len(), 3);
        assert!(seq_inner[0].is_interior); // Inner hole first
        assert!(!seq_inner[1].is_interior && !seq_inner[1].is_open); // Outer next
        assert!(seq_inner[2].is_open); // Open line last

        // 3. Manual: Exact insertion order: 0 (Outer), 1 (Open), 2 (Inner)
        let seq_manual = doc.compute_cut_sequence_with_mode(SortMode::Manual);
        assert_eq!(seq_manual.len(), 3);
        assert_eq!(seq_manual[0].entity_idx, 0);
        assert_eq!(seq_manual[1].entity_idx, 1);
        assert_eq!(seq_manual[2].entity_idx, 2);

        // 4. Apply sort and reorder
        doc.apply_sort_order(SortMode::OpenFirst);
        // Entity at index 0 in layer must now be the open line
        assert!(!doc.layers[0].entities[0].is_closed());

        // 5. Interactive manual cut index assignment
        doc.assign_manual_cut_index(0, 2, 1); // Put 3rd entity at index #1
        doc.assign_manual_cut_index(0, 1, 2); // Put 2nd entity at index #2
        let manual_seq = doc.compute_cut_sequence_with_mode(SortMode::Manual);
        assert_eq!(manual_seq[0].entity_idx, 2);
        assert_eq!(manual_seq[1].entity_idx, 1);
        assert_eq!(manual_seq[2].entity_idx, 0);
    }
}
