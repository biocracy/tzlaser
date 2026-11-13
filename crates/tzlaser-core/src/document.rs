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

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
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

    /// Sample ordered polyline waypoints along the entity for motion execution and dry-cut simulation.
    pub fn sample_points(&self) -> Vec<Point> {
        match self {
            Entity::Line { start, end } => vec![*start, *end],
            Entity::Polyline { points, closed } => {
                let mut pts = points.clone();
                if *closed && pts.len() >= 2 {
                    pts.push(pts[0]);
                }
                pts
            }
            Entity::Circle { center, radius } => {
                let segments = 64usize;
                let mut pts = Vec::with_capacity(segments + 1);
                for i in 0..=segments {
                    let ang = if i == segments {
                        0.0
                    } else {
                        std::f64::consts::TAU * (i as f64) / (segments as f64)
                    };
                    pts.push(Point::new(
                        center.x + radius * ang.cos(),
                        center.y + radius * ang.sin(),
                    ));
                }
                pts
            }
            Entity::Arc {
                center,
                radius,
                start_angle,
                end_angle,
            } => {
                let segments = 32usize;
                let s = *start_angle;
                let mut e = *end_angle;
                if e < s {
                    e += 360.0;
                }
                let mut pts = Vec::with_capacity(segments + 1);
                for i in 0..=segments {
                    let ang = (s + (e - s) * (i as f64) / (segments as f64)).to_radians();
                    pts.push(Point::new(
                        center.x + radius * ang.cos(),
                        center.y + radius * ang.sin(),
                    ));
                }
                pts
            }
            Entity::Group { children } => {
                let mut pts = Vec::new();
                for child in children {
                    pts.extend(child.sample_points());
                }
                pts
            }
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

/// Project point `p` onto segment `a -> b`, returning `(projected_point, distance, t_clamped)`.
pub fn closest_point_on_segment(p: Point, a: Point, b: Point) -> (Point, f64, f64) {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-9 {
        return (a, (p.x - a.x).hypot(p.y - a.y), 0.0);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq).clamp(0.0, 1.0);
    let proj = Point::new(a.x + t * dx, a.y + t * dy);
    let dist = (p.x - proj.x).hypot(p.y - proj.y);
    (proj, dist, t)
}

fn dist_point_to_segment(p: Point, a: Point, b: Point) -> f64 {
    closest_point_on_segment(p, a, b).1
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
    /// 1. Open/Bend lines -> 2. Interior holes maximizing distance between cuts (heat dissipation) -> 3. Outer contours
    MaxDistanceInterior,
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
            SortMode::MaxDistanceInterior => "Max Dist Interior (Cool)",
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
            SortMode::MaxDistanceInterior => "Maximize distance between consecutive interior cuts to prevent overheating and allow travel cooling",
            SortMode::ShortestPath => "Minimize rapid transit travel distance between pierces",
            SortMode::LeftToRight => "Sort cut paths from left to right across table",
            SortMode::TopToBottom => "Sort cut paths from top to bottom across table",
            SortMode::Manual => "Preserve exact entity list order without automatic optimization",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PastedOrigin {
    pub instance_id: usize,
    pub parent_layer: usize,
    pub parent_entity: usize,
}

mod tuple_map_serde {
    use super::PastedOrigin;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::collections::HashMap;

    pub fn serialize<S>(
        map: &HashMap<(usize, usize), PastedOrigin>,
        serializer: S,
    ) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let entries: Vec<((usize, usize), PastedOrigin)> = map.iter().map(|(&k, &v)| (k, v)).collect();
        entries.serialize(serializer)
    }

    pub fn deserialize<'de, D>(
        deserializer: D,
    ) -> Result<HashMap<(usize, usize), PastedOrigin>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let entries: Vec<((usize, usize), PastedOrigin)> = Vec::deserialize(deserializer)?;
        Ok(entries.into_iter().collect())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Document {
    pub layers: Vec<Layer>,
    #[serde(default)]
    pub sort_mode: SortMode,
    #[serde(default)]
    pub manual_sequence: Vec<(usize, usize)>,
    #[serde(default, with = "tuple_map_serde")]
    pub pasted_origins: HashMap<(usize, usize), PastedOrigin>,
    #[serde(default)]
    pub next_instance_id: usize,
    #[serde(default)]
    pub saved_cut_order: Vec<(usize, usize, Option<usize>)>,
    #[serde(default)]
    pub use_saved_cut_order: bool,
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
            pasted_origins: HashMap::new(),
            next_instance_id: 0,
            saved_cut_order: Vec::new(),
            use_saved_cut_order: false,
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
        let mut seen = std::collections::HashSet::new();
        let mut unique_sel = Vec::new();
        for &(l_idx, e_idx) in selection {
            if seen.insert((l_idx, e_idx)) {
                unique_sel.push((l_idx, e_idx));
            }
        }
        if unique_sel.is_empty() {
            return new_selection;
        }

        let current_seq = self.compute_cut_sequence();
        let mut seq_rank: HashMap<(usize, usize), usize> = HashMap::new();
        for (idx, info) in current_seq.iter().enumerate() {
            seq_rank.entry((info.layer_idx, info.entity_idx)).or_insert(idx);
        }
        unique_sel.sort_by_key(|pair| seq_rank.get(pair).copied().unwrap_or(usize::MAX));

        self.next_instance_id += 1;
        let instance_id = self.next_instance_id;

        for &(l_idx, e_idx) in &unique_sel {
            let root_parent = self
                .pasted_origins
                .get(&(l_idx, e_idx))
                .map(|o| (o.parent_layer, o.parent_entity))
                .unwrap_or((l_idx, e_idx));

            if let Some(layer) = self.layers.get(l_idx) {
                if let Some(entity) = layer.entities.get(e_idx) {
                    let mut cloned = entity.clone();
                    cloned.translate(dx, dy);
                    let target_layer = &mut self.layers[l_idx];
                    let new_idx = target_layer.entities.len();
                    target_layer.entities.push(cloned);
                    self.pasted_origins.insert(
                        (l_idx, new_idx),
                        PastedOrigin {
                            instance_id,
                            parent_layer: root_parent.0,
                            parent_entity: root_parent.1,
                        },
                    );
                    if !self.manual_sequence.is_empty() {
                        self.manual_sequence.push((l_idx, new_idx));
                    }
                    if self.use_saved_cut_order && !self.saved_cut_order.is_empty() {
                        match &self.layers[l_idx].entities[new_idx] {
                            Entity::Group { children } => {
                                let mut added = vec![false; children.len()];
                                let parent_children: Vec<usize> = self
                                    .saved_cut_order
                                    .iter()
                                    .filter_map(|&(pl, pe, pc)| {
                                        if pl == l_idx && pe == e_idx { pc } else { None }
                                    })
                                    .collect();
                                for c_idx in parent_children {
                                    if c_idx < children.len() && !added[c_idx] {
                                        self.saved_cut_order.push((l_idx, new_idx, Some(c_idx)));
                                        added[c_idx] = true;
                                    }
                                }
                                for (c_idx, &was_added) in added.iter().enumerate() {
                                    if !was_added {
                                        self.saved_cut_order.push((l_idx, new_idx, Some(c_idx)));
                                    }
                                }
                            }
                            _ => {
                                self.saved_cut_order.push((l_idx, new_idx, None));
                            }
                        }
                    }
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
        self.pasted_origins.clear();
        self.saved_cut_order.clear();
        self.use_saved_cut_order = false;
        let mut to_remove: HashMap<usize, Vec<usize>> = HashMap::new();
        for &(l, e) in selection {
            to_remove.entry(l).or_default().push(e);
        }
        for (l_idx, mut e_indices) in to_remove {
            if let Some(layer) = self.layers.get_mut(l_idx) {
                e_indices.sort_unstable_by(|a, b| b.cmp(a)); // remove in reverse order
                e_indices.dedup();
                for e in e_indices {
                    if e < layer.entities.len() {
                        layer.entities.remove(e);
                    }
                }
            }
        }
    }

    /// Group selected entities into a single compound object.
    /// All relative positions and cut sequence order of the profiles stay strictly fixed.
    /// Updates selection to point to the new single grouped entity.
    pub fn group_selection(&mut self, selection: &mut Vec<(usize, usize)>) -> Option<(usize, usize)> {
        if selection.len() <= 1 {
            return selection.first().copied();
        }

        let target_layer_idx = selection[0].0;
        let mut seen_sel = std::collections::HashSet::new();
        let mut selected_list = Vec::new();
        for &(l, e) in selection.iter() {
            if seen_sel.insert((l, e)) {
                selected_list.push((l, e));
            }
        }

        let current_seq = self.compute_cut_sequence();
        let mut grouped_children = Vec::new();
        let mut consumed_subpaths = std::collections::HashSet::new();
        let mut consumed_entities = std::collections::HashSet::new();

        for info in &current_seq {
            if seen_sel.contains(&(info.layer_idx, info.entity_idx)) {
                if let Some(layer) = self.layers.get(info.layer_idx) {
                    if let Some(entity) = layer.entities.get(info.entity_idx) {
                        match (entity, info.child_idx) {
                            (Entity::Group { children }, Some(c_idx)) => {
                                if let Some(child) = children.get(c_idx) {
                                    grouped_children.push(child.clone());
                                    consumed_subpaths.insert((info.layer_idx, info.entity_idx, Some(c_idx)));
                                    consumed_entities.insert((info.layer_idx, info.entity_idx));
                                }
                            }
                            _ => {
                                if consumed_entities.insert((info.layer_idx, info.entity_idx)) {
                                    grouped_children.push(entity.clone());
                                    consumed_subpaths.insert((info.layer_idx, info.entity_idx, None));
                                }
                            }
                        }
                    }
                }
            }
        }

        // Append any selected entities or children absent from current_seq (e.g. hidden layers)
        for &(l_idx, e_idx) in &selected_list {
            if let Some(layer) = self.layers.get(l_idx) {
                if let Some(entity) = layer.entities.get(e_idx) {
                    match entity {
                        Entity::Group { children } => {
                            for (c_idx, child) in children.iter().enumerate() {
                                if !consumed_subpaths.contains(&(l_idx, e_idx, Some(c_idx))) {
                                    grouped_children.push(child.clone());
                                }
                            }
                        }
                        _ => {
                            if !consumed_entities.contains(&(l_idx, e_idx)) {
                                grouped_children.push(entity.clone());
                            }
                        }
                    }
                }
            }
        }

        if grouped_children.is_empty() {
            return None;
        }

        let mut to_remove: HashMap<usize, Vec<usize>> = HashMap::new();
        for &(l, e) in &selected_list {
            to_remove.entry(l).or_default().push(e);
        }
        for (l_idx, mut e_indices) in to_remove {
            e_indices.sort_unstable_by(|a, b| b.cmp(a));
            e_indices.dedup();
            if let Some(layer) = self.layers.get_mut(l_idx) {
                for e_idx in e_indices {
                    if e_idx < layer.entities.len() {
                        layer.entities.remove(e_idx);
                    }
                }
            }
        }

        self.pasted_origins.clear();
        self.manual_sequence.clear();
        self.saved_cut_order.clear();
        self.use_saved_cut_order = false;

        let new_group = Entity::Group { children: grouped_children };
        let new_idx = self.layers[target_layer_idx].entities.len();
        self.layers[target_layer_idx].entities.push(new_group);

        *selection = vec![(target_layer_idx, new_idx)];
        Some((target_layer_idx, new_idx))
    }

    /// Ungroup any selected Group entities back into individual entities.
    pub fn ungroup_selection(&mut self, selection: &mut Vec<(usize, usize)>) {
        self.pasted_origins.clear();
        self.saved_cut_order.clear();
        self.use_saved_cut_order = false;
        let mut new_selection = Vec::new();

        let mut to_unpack: HashMap<usize, Vec<usize>> = HashMap::new();
        for &(l, e) in selection.iter() {
            to_unpack.entry(l).or_default().push(e);
        }

        for (l_idx, mut e_indices) in to_unpack {
            e_indices.sort_unstable_by(|a, b| b.cmp(a));
            e_indices.dedup();
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
        self.pasted_origins.clear();
        self.saved_cut_order.clear();
        self.use_saved_cut_order = false;
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

    /// Returns true if the document has a saved profile cutting order.
    pub fn has_saved_cut_order(&self) -> bool {
        !self.saved_cut_order.is_empty()
    }

    /// Saves the current cutting order of profiles and activates it.
    pub fn save_current_cut_order(&mut self) {
        let seq = self.compute_cut_sequence();
        self.saved_cut_order = seq
            .into_iter()
            .map(|i| (i.layer_idx, i.entity_idx, i.child_idx))
            .collect();
        self.use_saved_cut_order = !self.saved_cut_order.is_empty();
    }

    /// Restores the previously saved profile cutting order as active.
    pub fn restore_saved_cut_order(&mut self) {
        if !self.saved_cut_order.is_empty() {
            self.use_saved_cut_order = true;
        }
    }

    /// Computes the toolpath cutting sequence using the active saved order (if enabled)
    /// or the document's currently selected SortMode.
    pub fn compute_cut_sequence(&self) -> Vec<CutPathInfo> {
        let mut paths = self.compute_cut_sequence_with_mode(self.sort_mode);
        if self.use_saved_cut_order && !self.saved_cut_order.is_empty() {
            let mut pos_by_key: HashMap<(usize, usize, Option<usize>), usize> =
                HashMap::with_capacity(paths.len());
            for (idx, item) in paths.iter().enumerate() {
                pos_by_key.insert((item.layer_idx, item.entity_idx, item.child_idx), idx);
            }

            let mut result = Vec::with_capacity(paths.len());
            let mut used = vec![false; paths.len()];
            for &key in &self.saved_cut_order {
                if let Some(&pos) = pos_by_key.get(&key) {
                    if !used[pos] {
                        result.push(paths[pos]);
                        used[pos] = true;
                    }
                }
            }
            for (i, item) in paths.drain(..).enumerate() {
                if !used[i] {
                    result.push(item);
                }
            }
            return result;
        }
        paths
    }

    /// Computes the toolpath cutting sequence under a specific SortMode.
    /// Supports: OpenFirst, InnerFirst, MaxDistanceInterior, ShortestPath (TSP), LeftToRight, TopToBottom, and Manual order.
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
                        for (c_idx, child) in children.iter().enumerate() {
                            let open = !child.is_closed();
                            let is_bend = is_bend_layer || open;
                            let is_interior = is_interior_layer;
                            raw_paths.push(CutPathInfo {
                                layer_idx: l_idx,
                                entity_idx: e_idx,
                                child_idx: Some(c_idx),
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
                            child_idx: None,
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
        // Detect if a closed shape is nested inside another closed shape on any visible layer
        // or inside a sibling entity in the same Group.
        for item in &mut raw_paths {
            if !item.is_open && !item.is_interior {
                let test_pt = item.start_point;
                for (other_l_idx, other_layer) in self.layers.iter().enumerate() {
                    if !other_layer.visible { continue; }
                    for (other_e_idx, other_ent) in other_layer.entities.iter().enumerate() {
                        match other_ent {
                            Entity::Group { children } => {
                                for (other_c_idx, other_child) in children.iter().enumerate() {
                                    if item.layer_idx == other_l_idx
                                        && item.entity_idx == other_e_idx
                                        && item.child_idx == Some(other_c_idx)
                                    {
                                        continue;
                                    }
                                    if other_child.is_closed() && other_child.contains_point(test_pt) {
                                        item.is_interior = true;
                                        break;
                                    }
                                }
                            }
                            _ => {
                                if item.layer_idx == other_l_idx && item.entity_idx == other_e_idx {
                                    continue;
                                }
                                if other_ent.is_closed() && other_ent.contains_point(test_pt) {
                                    item.is_interior = true;
                                    break;
                                }
                            }
                        }
                        if item.is_interior { break; }
                    }
                    if item.is_interior { break; }
                }
            }
        }

        if mode == SortMode::Manual && !self.manual_sequence.is_empty() {
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
            return result;
        }

        if self.pasted_origins.is_empty() {
            return self.sequence_base_paths(raw_paths, mode);
        }

        let mut base_paths = Vec::with_capacity(raw_paths.len());
        let mut pasted_by_instance: std::collections::BTreeMap<usize, Vec<(PastedOrigin, CutPathInfo)>> =
            std::collections::BTreeMap::new();

        for item in raw_paths {
            if let Some(&orig) = self.pasted_origins.get(&(item.layer_idx, item.entity_idx)) {
                pasted_by_instance.entry(orig.instance_id).or_default().push((orig, item));
            } else {
                base_paths.push(item);
            }
        }

        let total_len = base_paths.len() + pasted_by_instance.values().map(|v| v.len()).sum::<usize>();
        let mut result = self.sequence_base_paths(base_paths, mode);
        result.reserve(total_len.saturating_sub(result.len()));

        let mut parent_rank: HashMap<(usize, usize, Option<usize>), usize> = HashMap::with_capacity(result.len());
        for (rank, item) in result.iter().enumerate() {
            parent_rank.insert((item.layer_idx, item.entity_idx, item.child_idx), rank);
        }

        for (_instance_id, mut items) in pasted_by_instance {
            items.sort_by_key(|(orig, item)| {
                let rank = parent_rank
                    .get(&(orig.parent_layer, orig.parent_entity, item.child_idx))
                    .copied()
                    .unwrap_or(usize::MAX);
                (rank, item.child_idx.unwrap_or(0))
            });
            for (_orig, item) in items {
                result.push(item);
            }
        }

        result
    }

    fn order_group_children_for_mode(children: Vec<CutPathInfo>, mode: SortMode) -> Vec<CutPathInfo> {
        match mode {
            SortMode::OpenFirst => {
                let mut open = Vec::new();
                let mut interior = Vec::new();
                let mut outer = Vec::new();
                for item in children {
                    if item.is_open {
                        open.push(item);
                    } else if item.is_interior {
                        interior.push(item);
                    } else {
                        outer.push(item);
                    }
                }
                open.extend(interior);
                open.extend(outer);
                open
            }
            SortMode::InnerFirst => {
                let mut interior = Vec::new();
                let mut outer = Vec::new();
                let mut open = Vec::new();
                for item in children {
                    if item.is_open {
                        open.push(item);
                    } else if item.is_interior {
                        interior.push(item);
                    } else {
                        outer.push(item);
                    }
                }
                interior.extend(outer);
                interior.extend(open);
                interior
            }
            SortMode::MaxDistanceInterior => {
                let mut open = Vec::new();
                let mut interior = Vec::new();
                let mut outer = Vec::new();
                for item in children {
                    if item.is_open {
                        open.push(item);
                    } else if item.is_interior {
                        interior.push(item);
                    } else {
                        outer.push(item);
                    }
                }
                let mut current_pos = open.last().map(|p| p.end_point).unwrap_or(Point::ORIGIN);
                let mut ordered_interior = Vec::with_capacity(interior.len());
                Self::sequence_group_farthest_neighbor(&mut interior, &mut current_pos, &mut ordered_interior);
                open.extend(ordered_interior);
                open.extend(outer);
                open
            }
            _ => children,
        }
    }

    fn sequence_base_paths(&self, mut raw_paths: Vec<CutPathInfo>, mode: SortMode) -> Vec<CutPathInfo> {
        match mode {
            SortMode::Manual => raw_paths,
            SortMode::LeftToRight => {
                raw_paths.sort_by(|a, b| a.start_point.x.partial_cmp(&b.start_point.x).unwrap_or(std::cmp::Ordering::Equal));
                raw_paths
            }
            SortMode::TopToBottom => {
                raw_paths.sort_by(|a, b| b.start_point.y.partial_cmp(&a.start_point.y).unwrap_or(std::cmp::Ordering::Equal));
                raw_paths
            }
            SortMode::ShortestPath | SortMode::OpenFirst | SortMode::InnerFirst | SortMode::MaxDistanceInterior => {
                let total_len = raw_paths.len();
                let mut standalone = Vec::new();
                let mut groups: std::collections::BTreeMap<(usize, usize), Vec<CutPathInfo>> =
                    std::collections::BTreeMap::new();

                for item in raw_paths {
                    if item.child_idx.is_some() {
                        groups.entry((item.layer_idx, item.entity_idx)).or_default().push(item);
                    } else {
                        standalone.push(item);
                    }
                }

                let mut group_units: Vec<Vec<CutPathInfo>> = groups
                    .into_values()
                    .map(|g| Self::order_group_children_for_mode(g, mode))
                    .filter(|u| !u.is_empty())
                    .collect();

                let mut result = Vec::with_capacity(total_len);
                let mut current_pos = Point::ORIGIN;

                if mode == SortMode::ShortestPath {
                    let mut units: Vec<Vec<CutPathInfo>> = standalone.into_iter().map(|s| vec![s]).collect();
                    units.append(&mut group_units);
                    Self::sequence_units_nearest_neighbor(&mut units, &mut current_pos, &mut result);
                    return result;
                }

                let mut bend_paths = Vec::new();
                let mut interior_paths = Vec::new();
                let mut outer_paths = Vec::new();

                for item in standalone {
                    if item.is_open {
                        bend_paths.push(item);
                    } else if item.is_interior {
                        interior_paths.push(item);
                    } else {
                        outer_paths.push(item);
                    }
                }

                match mode {
                    SortMode::OpenFirst => {
                        Self::sequence_group_nearest_neighbor(&mut bend_paths, &mut current_pos, &mut result);
                        Self::sequence_group_nearest_neighbor(&mut interior_paths, &mut current_pos, &mut result);
                        Self::sequence_group_nearest_neighbor(&mut outer_paths, &mut current_pos, &mut result);
                    }
                    SortMode::InnerFirst => {
                        Self::sequence_group_nearest_neighbor(&mut interior_paths, &mut current_pos, &mut result);
                        Self::sequence_group_nearest_neighbor(&mut outer_paths, &mut current_pos, &mut result);
                        Self::sequence_group_nearest_neighbor(&mut bend_paths, &mut current_pos, &mut result);
                    }
                    SortMode::MaxDistanceInterior => {
                        Self::sequence_group_nearest_neighbor(&mut bend_paths, &mut current_pos, &mut result);
                        Self::sequence_group_farthest_neighbor(&mut interior_paths, &mut current_pos, &mut result);
                        Self::sequence_group_nearest_neighbor(&mut outer_paths, &mut current_pos, &mut result);
                    }
                    _ => unreachable!(),
                }

                Self::sequence_units_nearest_neighbor(&mut group_units, &mut current_pos, &mut result);
                result
            }
        }
    }

    fn sequence_units_nearest_neighbor(
        units: &mut Vec<Vec<CutPathInfo>>,
        current_pos: &mut Point,
        result: &mut Vec<CutPathInfo>,
    ) {
        while !units.is_empty() {
            let mut best_idx = 0;
            let mut best_dist = f64::MAX;

            for (i, unit) in units.iter().enumerate() {
                if let Some(first) = unit.first() {
                    let d = (first.start_point.x - current_pos.x).hypot(first.start_point.y - current_pos.y);
                    if d < best_dist {
                        best_dist = d;
                        best_idx = i;
                    }
                }
            }

            let next_unit = units.swap_remove(best_idx);
            if let Some(last) = next_unit.last() {
                *current_pos = last.end_point;
            }
            result.extend(next_unit);
        }
    }

    /// Reorders the entities within layers to permanently match the computed sort sequence.
    pub fn apply_sort_order(&mut self, mode: SortMode) {
        self.sort_mode = mode;
        self.use_saved_cut_order = false;
        let seq = self.compute_cut_sequence_with_mode(mode);
        self.manual_sequence.clear();

        let mut old_to_new: HashMap<(usize, usize), usize> = HashMap::new();
        let mut old_child_to_new: HashMap<(usize, usize, usize), usize> = HashMap::new();

        for (l_idx, layer) in self.layers.iter_mut().enumerate() {
            let mut ordered_entities = Vec::with_capacity(layer.entities.len());
            let mut used = vec![false; layer.entities.len()];

            for info in &seq {
                if info.layer_idx == l_idx && info.entity_idx < layer.entities.len() && !used[info.entity_idx] {
                    let new_idx = ordered_entities.len();
                    let ent = match &layer.entities[info.entity_idx] {
                        Entity::Group { children } => {
                            let mut reordered = Vec::with_capacity(children.len());
                            let mut c_used = vec![false; children.len()];
                            for s in &seq {
                                if s.layer_idx == l_idx && s.entity_idx == info.entity_idx {
                                    if let Some(old_c) = s.child_idx {
                                        if old_c < children.len() && !c_used[old_c] {
                                            let new_c = reordered.len();
                                            reordered.push(children[old_c].clone());
                                            c_used[old_c] = true;
                                            old_child_to_new.insert((l_idx, info.entity_idx, old_c), new_c);
                                        }
                                    }
                                }
                            }
                            for (old_c, child) in children.iter().enumerate() {
                                if !c_used[old_c] {
                                    let new_c = reordered.len();
                                    reordered.push(child.clone());
                                    old_child_to_new.insert((l_idx, info.entity_idx, old_c), new_c);
                                }
                            }
                            Entity::Group { children: reordered }
                        }
                        other => other.clone(),
                    };
                    ordered_entities.push(ent);
                    used[info.entity_idx] = true;
                    old_to_new.insert((l_idx, info.entity_idx), new_idx);
                }
            }

            // Append any unvisited entities (e.g. from hidden layers)
            for (i, ent) in layer.entities.drain(..).enumerate() {
                if !used[i] {
                    let new_idx = ordered_entities.len();
                    ordered_entities.push(ent);
                    old_to_new.insert((l_idx, i), new_idx);
                }
            }
            layer.entities = ordered_entities;
        }

        if !self.pasted_origins.is_empty() {
            let mut remapped = HashMap::with_capacity(self.pasted_origins.len());
            for ((old_l, old_e), mut origin) in self.pasted_origins.drain() {
                let new_e = old_to_new.get(&(old_l, old_e)).copied().unwrap_or(old_e);
                let new_parent_e = old_to_new
                    .get(&(origin.parent_layer, origin.parent_entity))
                    .copied()
                    .unwrap_or(origin.parent_entity);
                origin.parent_entity = new_parent_e;
                remapped.insert((old_l, new_e), origin);
            }
            self.pasted_origins = remapped;
        }

        if !self.saved_cut_order.is_empty() {
            for (l_idx, e_idx, child_idx) in &mut self.saved_cut_order {
                let old_e = *e_idx;
                let new_e = old_to_new.get(&(*l_idx, old_e)).copied().unwrap_or(old_e);
                if let Some(old_c) = child_idx {
                    *old_c = old_child_to_new.get(&(*l_idx, old_e, *old_c)).copied().unwrap_or(*old_c);
                }
                *e_idx = new_e;
            }
        }
    }

    /// Assigns a specific 1-based cut order index to `(layer_idx, entity_idx)` in manual sort mode.
    pub fn assign_manual_cut_index(&mut self, layer_idx: usize, entity_idx: usize, target_1based: usize) {
        self.sort_mode = SortMode::Manual;
        self.use_saved_cut_order = false;
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

    fn sequence_group_farthest_neighbor(
        group: &mut Vec<CutPathInfo>,
        current_pos: &mut Point,
        result: &mut Vec<CutPathInfo>,
    ) {
        while !group.is_empty() {
            let mut best_idx = 0;
            let mut best_dist = -1.0_f64;

            for (i, item) in group.iter().enumerate() {
                let d = (item.start_point.x - current_pos.x).hypot(item.start_point.y - current_pos.y);
                if d > best_dist {
                    best_dist = d;
                    best_idx = i;
                }
            }

            let next_cut = group.remove(best_idx);
            *current_pos = next_cut.end_point;
            result.push(next_cut);
        }
    }

    /// Build ordered rapid and cutting motion segments across the document's active cut sequence,
    /// starting from `start_origin` and returning to `start_origin` at the end.
    pub fn build_motion_trajectory(&self, start_origin: Point, rapid_speed_mm_s: f64) -> Vec<MotionSegment> {
        let seq = self.compute_cut_sequence();
        let rapid_speed = rapid_speed_mm_s.max(10.0);
        let mut segments = Vec::new();
        let mut current_pos = start_origin;

        for (idx, info) in seq.iter().enumerate() {
            let Some(layer) = self.layers.get(info.layer_idx) else { continue; };
            let Some(entity) = layer.entities.get(info.entity_idx) else { continue; };
            let target = match (entity, info.child_idx) {
                (Entity::Group { children }, Some(c_idx)) => match children.get(c_idx) {
                    Some(c) => c,
                    None => continue,
                },
                _ => entity,
            };

            let cut_speed = layer.params.speed_mm_s.max(0.5);
            let pts = target.sample_points();
            if pts.len() < 2 {
                continue;
            }

            let path_index_1based = idx + 1;
            let rapid_len = (pts[0].x - current_pos.x).hypot(pts[0].y - current_pos.y);
            if rapid_len > 1e-4 {
                segments.push(MotionSegment {
                    start: current_pos,
                    end: pts[0],
                    length_mm: rapid_len,
                    speed_mm_s: rapid_speed,
                    is_rapid: true,
                    path_index_1based,
                });
            }

            for pair in pts.windows(2) {
                let seg_len = (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y);
                if seg_len > 1e-6 {
                    segments.push(MotionSegment {
                        start: pair[0],
                        end: pair[1],
                        length_mm: seg_len,
                        speed_mm_s: cut_speed,
                        is_rapid: false,
                        path_index_1based,
                    });
                }
            }

            if let Some(&end_pt) = pts.last() {
                current_pos = end_pt;
            }
        }

        if !segments.is_empty() {
            let return_len = (start_origin.x - current_pos.x).hypot(start_origin.y - current_pos.y);
            if return_len > 1e-4 {
                segments.push(MotionSegment {
                    start: current_pos,
                    end: start_origin,
                    length_mm: return_len,
                    speed_mm_s: rapid_speed,
                    is_rapid: true,
                    path_index_1based: seq.len(),
                });
            }
        }

        segments
    }

    /// Find the closest point along any active cutting path in the document within `tolerance_mm`.
    /// Returns `Some((projected_point_mm, path_index_1based, distance_mm))`.
    pub fn closest_point_on_cut_paths(
        &self,
        pt: Point,
        tolerance_mm: f64,
    ) -> Option<(Point, usize, f64)> {
        let seq = self.compute_cut_sequence();
        let mut best: Option<(Point, usize, f64)> = None;

        for (idx, info) in seq.iter().enumerate() {
            let Some(layer) = self.layers.get(info.layer_idx) else {
                continue;
            };
            let Some(entity) = layer.entities.get(info.entity_idx) else {
                continue;
            };
            let target = match (entity, info.child_idx) {
                (Entity::Group { children }, Some(c_idx)) => match children.get(c_idx) {
                    Some(c) => c,
                    None => continue,
                },
                _ => entity,
            };

            let pts = target.sample_points();
            if pts.len() < 2 {
                continue;
            }
            let path_index_1based = idx + 1;

            for pair in pts.windows(2) {
                let (proj, dist, _) = closest_point_on_segment(pt, pair[0], pair[1]);
                if dist <= tolerance_mm && best.map_or(true, |(_, _, b_dist)| dist < b_dist) {
                    best = Some((proj, path_index_1based, dist));
                }
            }
        }

        best
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionSegment {
    pub start: Point,
    pub end: Point,
    pub length_mm: f64,
    pub speed_mm_s: f64,
    pub is_rapid: bool,
    pub path_index_1based: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CutPathInfo {
    pub layer_idx: usize,
    pub entity_idx: usize,
    pub child_idx: Option<usize>,
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

    #[test]
    fn test_grouped_copy_preserves_unique_indices_and_containment() {
        let mut doc = Document::default();
        let mut layer = Layer::new("Outer Profiles", Color::GREEN);
        layer.entities.push(Entity::Polyline {
            points: vec![
                Point::new(0.0, 0.0),
                Point::new(100.0, 0.0),
                Point::new(100.0, 100.0),
                Point::new(0.0, 100.0),
            ],
            closed: true,
        });
        layer.entities.push(Entity::Circle {
            center: Point::new(50.0, 50.0),
            radius: 15.0,
        });
        doc.layers = vec![layer];

        let mut sel = vec![(0, 0), (0, 1)];
        doc.group_selection(&mut sel);
        assert_eq!(sel.len(), 1);

        // Copy group to the right (+Copy X)
        let copies = doc.duplicate_selection(&sel, 120.0, 0.0);
        assert_eq!(copies.len(), 1);

        let seq = doc.compute_cut_sequence();
        assert_eq!(seq.len(), 4);

        // Part 1 cuts its interior hole (index 0) before its outer contour (index 1),
        // and Copy 2 follows the exact same relative order: interior hole (index 2) before outer contour (index 3).
        assert!(seq[0].is_interior);
        assert!(!seq[1].is_interior);
        assert!(seq[2].is_interior);
        assert!(!seq[3].is_interior);

        // Every CutPathInfo must have a unique (layer_idx, entity_idx, child_idx) key
        let unique_keys: std::collections::HashSet<_> = seq
            .iter()
            .map(|i| (i.layer_idx, i.entity_idx, i.child_idx))
            .collect();
        assert_eq!(unique_keys.len(), 4);
    }

    #[test]
    fn test_pasted_objects_preserve_parent_relative_index_order() {
        // Build a part with 1 outer box, 3 asymmetric interior holes (at different Y/X so TSP
        // would normally reverse/scramble when entering from another corner), and 1 open bend line.
        let make_part_doc = || {
            let mut doc = Document::default();
            let mut outer = Layer::new("Outer Profiles", Color::GREEN);
            outer.entities.push(Entity::Polyline {
                points: vec![
                    Point::new(0.0, 0.0),
                    Point::new(100.0, 0.0),
                    Point::new(100.0, 100.0),
                    Point::new(0.0, 100.0),
                ],
                closed: true,
            });
            let mut inner = Layer::new("Interior Profiles", Color::YELLOW);
            inner.entities.push(Entity::Circle {
                center: Point::new(85.0, 80.0),
                radius: 5.0,
            });
            inner.entities.push(Entity::Circle {
                center: Point::new(15.0, 20.0),
                radius: 5.0,
            });
            inner.entities.push(Entity::Circle {
                center: Point::new(85.0, 20.0),
                radius: 5.0,
            });
            let mut bend = Layer::new("Bend / Open Lines", Color::MAGENTA);
            bend.entities.push(Entity::Line {
                start: Point::new(10.0, 50.0),
                end: Point::new(90.0, 50.0),
            });
            doc.layers = vec![outer, inner, bend];
            doc
        };

        // 1. Ungrouped multi-path part: duplicate_selection and array_copy
        {
            let mut doc = make_part_doc();
            let sel = vec![(0, 0), (1, 0), (1, 1), (1, 2), (2, 0)];
            let all = doc.array_copy(&sel, 2, 2, 130.0, 130.0);
            assert_eq!(all.len(), 20);

            let seq = doc.compute_cut_sequence();
            assert_eq!(seq.len(), 20);

            // Verify every pasted copy instance (copies 1..4) has the exact same relative path order
            // as parent copy 0, shifted by (dx, dy) = (c * 130, r * 130).
            let offsets = [(130.0, 0.0), (0.0, 130.0), (130.0, 130.0)];
            for (copy_i, &(dx, dy)) in offsets.iter().enumerate() {
                let base_idx = (copy_i + 1) * 5;
                for k in 0..5 {
                    let parent = &seq[k];
                    let copy = &seq[base_idx + k];
                    assert!((copy.start_point.x - parent.start_point.x - dx).abs() < 1e-6);
                    assert!((copy.start_point.y - parent.start_point.y - dy).abs() < 1e-6);
                }
            }
        }

        // 2. Grouped Entity::Group part: duplicate_selection and array_copy
        {
            let mut doc = make_part_doc();
            let mut sel = vec![(0, 0), (1, 0), (1, 1), (1, 2), (2, 0)];
            doc.group_selection(&mut sel);
            assert_eq!(sel.len(), 1);

            let _copy1 = doc.duplicate_selection(&sel, 130.0, 0.0);
            let _all = doc.array_copy(&sel, 2, 1, 260.0, 0.0);

            let seq = doc.compute_cut_sequence();
            assert_eq!(seq.len(), 15);

            // Parent group + 2 copies => 3 instances of 5 paths each
            let dxs = [130.0, 260.0];
            for (copy_i, &dx) in dxs.iter().enumerate() {
                let base_idx = (copy_i + 1) * 5;
                for k in 0..5 {
                    let parent = &seq[k];
                    let copy = &seq[base_idx + k];
                    assert!((copy.start_point.x - parent.start_point.x - dx).abs() < 1e-6);
                    assert!((copy.start_point.y - parent.start_point.y).abs() < 1e-6);
                }
            }
        }
    }

    #[test]
    fn test_max_distance_interior_sorting() {
        let mut doc = Document::default();
        let mut outer = Layer::new("Outer Profiles", Color::GREEN);
        outer.entities.push(Entity::Polyline {
            points: vec![
                Point::new(0.0, 0.0),
                Point::new(200.0, 0.0),
                Point::new(200.0, 100.0),
                Point::new(0.0, 100.0),
            ],
            closed: true,
        });

        let mut inner = Layer::new("Interior Profiles", Color::YELLOW);
        for x in [10.0, 20.0, 30.0, 180.0] {
            inner.entities.push(Entity::Circle {
                center: Point::new(x, 50.0),
                radius: 2.0,
            });
        }
        doc.layers = vec![outer, inner];

        let max_seq = doc.compute_cut_sequence_with_mode(SortMode::MaxDistanceInterior);
        assert_eq!(max_seq.len(), 5);
        // All 4 interior holes first, maximizing travel distance between consecutive holes:
        // from (0,0) -> 180 -> 10 -> 30 -> 20, then outer contour last
        assert!(max_seq[0..4].iter().all(|p| p.is_interior));
        assert!(!max_seq[4].is_interior);

        let xs_max: Vec<f64> = max_seq[0..4].iter().map(|p| p.start_point.x - 2.0).collect();
        assert_eq!(xs_max, vec![180.0, 10.0, 30.0, 20.0]);

        let inner_seq = doc.compute_cut_sequence_with_mode(SortMode::InnerFirst);
        let travel_dist = |seq: &[CutPathInfo]| -> f64 {
            seq.windows(2)
                .map(|w| (w[1].start_point.x - w[0].end_point.x).hypot(w[1].start_point.y - w[0].end_point.y))
                .sum()
        };
        assert!(travel_dist(&max_seq[0..4]) > travel_dist(&inner_seq[0..4]));
    }

    #[test]
    fn test_save_and_restore_cut_order() {
        let mut doc = Document::default();
        let mut layer = Layer::new("All", Color::GREEN);
        layer.entities.push(Entity::Polyline {
            points: vec![
                Point::new(0.0, 0.0),
                Point::new(200.0, 0.0),
                Point::new(200.0, 100.0),
                Point::new(0.0, 100.0),
            ],
            closed: true,
        });
        layer.entities.push(Entity::Circle {
            center: Point::new(20.0, 50.0),
            radius: 5.0,
        });
        layer.entities.push(Entity::Circle {
            center: Point::new(150.0, 50.0),
            radius: 5.0,
        });
        doc.layers = vec![layer];

        assert!(!doc.has_saved_cut_order());
        doc.assign_manual_cut_index(0, 2, 1);
        doc.assign_manual_cut_index(0, 1, 2);
        doc.assign_manual_cut_index(0, 0, 3);
        doc.save_current_cut_order();
        assert!(doc.has_saved_cut_order());
        assert!(doc.use_saved_cut_order);

        let saved_order_keys: Vec<(usize, usize, Option<usize>)> = doc
            .compute_cut_sequence()
            .iter()
            .map(|p| (p.layer_idx, p.entity_idx, p.child_idx))
            .collect();
        assert_eq!(saved_order_keys, vec![(0, 2, None), (0, 1, None), (0, 0, None)]);

        // Changing sort_mode while use_saved_cut_order is true preserves the active saved order
        doc.sort_mode = SortMode::LeftToRight;
        let still_saved: Vec<(usize, usize, Option<usize>)> = doc
            .compute_cut_sequence()
            .iter()
            .map(|p| (p.layer_idx, p.entity_idx, p.child_idx))
            .collect();
        assert_eq!(still_saved, saved_order_keys);

        // Applying LeftToRight disables use_saved_cut_order, but remaps saved_cut_order
        doc.apply_sort_order(SortMode::LeftToRight);
        assert!(!doc.use_saved_cut_order);
        assert!(doc.has_saved_cut_order());

        // After LeftToRight: entity 0 is Polyline (x=0), entity 1 is Circle(20), entity 2 is Circle(150)
        // Restoring saved order puts Circle(150) -> Circle(20) -> Polyline back in exact saved sequence!
        doc.restore_saved_cut_order();
        assert!(doc.use_saved_cut_order);
        let restored_starts: Vec<f64> = doc.compute_cut_sequence().iter().map(|p| p.start_point.x).collect();
        assert_eq!(restored_starts, vec![155.0, 25.0, 0.0]);
    }

    #[test]
    fn test_build_motion_trajectory_for_dry_cut() {
        let mut doc = Document::default();
        let mut outer = Layer::new("Outer Profiles", Color::GREEN);
        outer.entities.push(Entity::Polyline {
            points: vec![
                Point::new(10.0, 10.0),
                Point::new(110.0, 10.0),
                Point::new(110.0, 60.0),
                Point::new(10.0, 60.0),
            ],
            closed: true,
        });
        let mut inner = Layer::new("Interior Profiles", Color::YELLOW);
        inner.entities.push(Entity::Circle {
            center: Point::new(50.0, 35.0),
            radius: 10.0,
        });
        doc.layers = vec![outer, inner];

        let origin = Point::ORIGIN;
        let traj = doc.build_motion_trajectory(origin, 200.0);
        assert!(!traj.is_empty());

        // Starts at origin and returns to origin
        assert_eq!(traj.first().unwrap().start, origin);
        assert!(traj.first().unwrap().is_rapid);
        assert_eq!(traj.last().unwrap().end, origin);
        assert!(traj.last().unwrap().is_rapid);

        // Path 1 is the interior circle (start at x=60, y=35), Path 2 is outer box (4 cutting segments)
        let cut_segs: Vec<&MotionSegment> = traj.iter().filter(|s| !s.is_rapid).collect();
        assert_eq!(cut_segs.iter().filter(|s| s.path_index_1based == 1).count(), 64);
        assert_eq!(cut_segs.iter().filter(|s| s.path_index_1based == 2).count(), 4);
    }
}
