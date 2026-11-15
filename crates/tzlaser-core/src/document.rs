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

fn default_cut_height_mm() -> f64 {
    1.0
}

fn default_frequency_hz() -> u32 {
    5000
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CuttingParams {
    pub speed_mm_s: f64,
    pub power_percent: f64,
    pub pierce_time_ms: u32,
    pub gas_type: String,
    #[serde(default = "default_cut_height_mm")]
    pub cut_height_mm: f64,
    #[serde(default = "default_frequency_hz")]
    pub frequency_hz: u32,
}

impl Default for CuttingParams {
    fn default() -> Self {
        Self {
            speed_mm_s: 15.0,
            power_percent: 80.0,
            pierce_time_ms: 200,
            gas_type: "Air".to_string(),
            cut_height_mm: default_cut_height_mm(),
            frequency_hz: default_frequency_hz(),
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

/// Compute smooth adaptive segment count for a circular arc of `radius` (mm) and `sweep_rad` (rad),
/// keeping chord sagitta error <= 0.002 mm and arc step <= 0.20 mm so motion commands glide smoothly.
fn adaptive_arc_segments(radius: f64, sweep_rad: f64, min_segs: usize) -> usize {
    let r = radius.abs().max(0.1);
    let sweep = sweep_rad.abs().max(1e-6);
    let max_sagitta = 0.002_f64;
    let max_step_mm = 0.20_f64;
    let dtheta_sagitta = 2.0 * (1.0 - (max_sagitta / r).min(0.5)).acos().max(1e-4);
    let by_sagitta = (sweep / dtheta_sagitta).ceil() as usize;
    let by_step = ((r * sweep) / max_step_mm).ceil() as usize;
    by_sagitta.max(by_step).clamp(min_segs, 1440)
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

    /// Scale entity around `center` by `(sx, sy)`.
    /// Non-uniform scaling of a `Circle` converts it into a 64-segment closed ellipse polyline.
    pub fn scale_around(&mut self, center: Point, sx: f64, sy: f64) {
        let sx = if sx.abs() < 1e-6 { 1e-6_f64.copysign(sx) } else { sx };
        let sy = if sy.abs() < 1e-6 { 1e-6_f64.copysign(sy) } else { sy };
        let scale_pt = |p: Point| -> Point {
            Point::new(
                center.x + (p.x - center.x) * sx,
                center.y + (p.y - center.y) * sy,
            )
        };
        match self {
            Entity::Line { start, end } => {
                *start = scale_pt(*start);
                *end = scale_pt(*end);
            }
            Entity::Polyline { points, .. } => {
                for p in points.iter_mut() {
                    *p = scale_pt(*p);
                }
            }
            Entity::Circle { center: c, radius } => {
                let new_center = scale_pt(*c);
                if (sx.abs() - sy.abs()).abs() < 1e-6 {
                    *c = new_center;
                    *radius = (*radius * sx.abs()).max(1e-4);
                } else {
                    let segs = 64usize;
                    let rx = (*radius * sx.abs()).max(1e-4);
                    let ry = (*radius * sy.abs()).max(1e-4);
                    let pts = (0..segs)
                        .map(|i| {
                            let ang = std::f64::consts::TAU * (i as f64) / (segs as f64);
                            Point::new(new_center.x + rx * ang.cos(), new_center.y + ry * ang.sin())
                        })
                        .collect();
                    *self = Entity::Polyline { points: pts, closed: true };
                }
            }
            Entity::Arc { center: c, radius, .. } => {
                *c = scale_pt(*c);
                *radius = (*radius * ((sx.abs() + sy.abs()) * 0.5)).max(1e-4);
            }
            Entity::Group { children } => {
                for c in children.iter_mut() {
                    c.scale_around(center, sx, sy);
                }
            }
        }
    }

    /// Rotate entity counter-clockwise around `center` by `angle_rad` radians.
    pub fn rotate_around(&mut self, center: Point, angle_rad: f64) {
        let (sin_a, cos_a) = angle_rad.sin_cos();
        let rot_pt = |p: Point| -> Point {
            let dx = p.x - center.x;
            let dy = p.y - center.y;
            Point::new(
                center.x + dx * cos_a - dy * sin_a,
                center.y + dx * sin_a + dy * cos_a,
            )
        };
        match self {
            Entity::Line { start, end } => {
                *start = rot_pt(*start);
                *end = rot_pt(*end);
            }
            Entity::Polyline { points, .. } => {
                for p in points.iter_mut() {
                    *p = rot_pt(*p);
                }
            }
            Entity::Circle { center: c, .. } => {
                *c = rot_pt(*c);
            }
            Entity::Arc { center: c, start_angle, .. } => {
                *c = rot_pt(*c);
                *start_angle += angle_rad;
            }
            Entity::Group { children } => {
                for c in children.iter_mut() {
                    c.rotate_around(center, angle_rad);
                }
            }
        }
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

    /// Re-anchor a closed path's starting vertex to the point on its contour closest to `target_pt`,
    /// or reverse an open path if `target_pt` is closer to its end than its start.
    pub fn set_start_point_near(&mut self, target_pt: Point) -> bool {
        match self {
            Entity::Polyline { points, closed } => {
                if points.len() < 2 {
                    return false;
                }
                if *closed {
                    let n = points.len();
                    let mut best: Option<(usize, Point, f64)> = None;
                    for i in 0..n {
                        let a = points[i];
                        let b = points[(i + 1) % n];
                        let dx = b.x - a.x;
                        let dy = b.y - a.y;
                        let len2 = dx * dx + dy * dy;
                        let t = if len2 > 1e-12 {
                            (((target_pt.x - a.x) * dx + (target_pt.y - a.y) * dy) / len2)
                                .clamp(0.0, 1.0)
                        } else {
                            0.0
                        };
                        let proj = Point::new(a.x + dx * t, a.y + dy * t);
                        let dist = proj.distance(target_pt);
                        if best.map_or(true, |(_, _, bd)| dist < bd) {
                            best = Some((i, proj, dist));
                        }
                    }
                    let Some((seg_i, proj, _)) = best else {
                        return false;
                    };
                    let next_i = (seg_i + 1) % n;
                    if proj.distance(points[seg_i]) < 1e-4 {
                        points.rotate_left(seg_i);
                    } else if proj.distance(points[next_i]) < 1e-4 {
                        points.rotate_left(next_i);
                    } else {
                        let mut rotated = Vec::with_capacity(n + 1);
                        rotated.push(proj);
                        rotated.extend_from_slice(&points[seg_i + 1..]);
                        rotated.extend_from_slice(&points[..=seg_i]);
                        *points = rotated;
                    }
                    true
                } else {
                    if target_pt.distance(*points.last().unwrap()) < target_pt.distance(points[0]) {
                        points.reverse();
                        true
                    } else {
                        false
                    }
                }
            }
            Entity::Circle { center, radius } => {
                let start_ang = (target_pt.y - center.y).atan2(target_pt.x - center.x);
                let segments = adaptive_arc_segments(*radius, std::f64::consts::TAU, 180);
                let mut points = Vec::with_capacity(segments);
                for i in 0..segments {
                    let ang = start_ang + std::f64::consts::TAU * (i as f64) / (segments as f64);
                    points.push(Point::new(
                        center.x + *radius * ang.cos(),
                        center.y + *radius * ang.sin(),
                    ));
                }
                *self = Entity::Polyline {
                    points,
                    closed: true,
                };
                true
            }
            Entity::Line { start, end } => {
                if target_pt.distance(*end) < target_pt.distance(*start) {
                    std::mem::swap(start, end);
                    true
                } else {
                    false
                }
            }
            Entity::Arc { .. } | Entity::Group { .. } => false,
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
                let segments = adaptive_arc_segments(*radius, std::f64::consts::TAU, 180);
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
                let s = *start_angle;
                let mut e = *end_angle;
                if e < s {
                    e += 360.0;
                }
                let sweep_rad = (e - s).to_radians().abs();
                let min_segs = ((sweep_rad / std::f64::consts::TAU) * 180.0).ceil().max(48.0) as usize;
                let segments = adaptive_arc_segments(*radius, sweep_rad, min_segs);
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

    /// Number of individual cuttable paths represented by this entity (counting group children).
    pub fn path_count(&self) -> usize {
        match self {
            Entity::Group { children } => children.iter().map(|c| c.path_count()).sum(),
            _ => 1,
        }
    }

    /// Total path length in mm (recursing into groups without counting inter-child travel jumps).
    pub fn path_length_mm(&self) -> f64 {
        match self {
            Entity::Group { children } => children.iter().map(|c| c.path_length_mm()).sum(),
            _ => {
                let pts = self.sample_points();
                pts.windows(2)
                    .map(|pair| (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y))
                    .sum()
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

    /// True if this entity is a closed shape with non-zero area (eligible to be an Obstacle).
    /// Points and open lines/paths return `false`.
    pub fn can_be_obstacle(&self) -> bool {
        match self {
            Entity::Group { children } => {
                !children.is_empty() && children.iter().all(|c| c.can_be_obstacle())
            }
            _ => self.is_closed() && self.area() > 1e-3,
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
    #[serde(default)]
    pub is_custom: bool,
    #[serde(default)]
    pub is_travel: bool,
    #[serde(default)]
    pub is_obstacle: bool,
}

impl Layer {
    pub fn new(name: impl Into<String>, color: Color) -> Self {
        Self {
            name: name.into(),
            color,
            visible: true,
            params: CuttingParams::default(),
            entities: Vec::new(),
            is_custom: false,
            is_travel: false,
            is_obstacle: false,
        }
    }

    pub fn new_travel() -> Self {
        Self {
            name: "Travel".to_string(),
            color: Color { r: 130, g: 165, b: 205 },
            visible: true,
            params: CuttingParams {
                speed_mm_s: 20.0,
                power_percent: 0.0,
                pierce_time_ms: 0,
                gas_type: "None".to_string(),
                cut_height_mm: 5.0,
                frequency_hz: 0,
            },
            entities: Vec::new(),
            is_custom: false,
            is_travel: true,
            is_obstacle: false,
        }
    }

    pub fn new_obstacle() -> Self {
        Self {
            name: "Obstacle".to_string(),
            color: Color { r: 235, g: 95, b: 55 },
            visible: true,
            params: CuttingParams {
                speed_mm_s: 0.0,
                power_percent: 0.0,
                pierce_time_ms: 0,
                gas_type: "None".to_string(),
                cut_height_mm: 0.0,
                frequency_hz: 0,
            },
            entities: Vec::new(),
            is_custom: false,
            is_travel: false,
            is_obstacle: true,
        }
    }

    /// Total number of individual paths in this layer (counting children inside groups).
    pub fn total_path_count(&self) -> usize {
        self.entities.iter().map(|e| e.path_count()).sum()
    }

    /// Total path length in meters across all entities in this layer.
    pub fn total_path_length_m(&self) -> f64 {
        self.entities.iter().map(|e| e.path_length_mm()).sum::<f64>() / 1000.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JobTimeEstimate {
    pub total_s: f64,
    pub cut_s: f64,
    pub rapid_s: f64,
    pub pierce_s: f64,
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
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::collections::HashMap;

    pub fn serialize<S, V>(
        map: &HashMap<(usize, usize), V>,
        serializer: S,
    ) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
        V: Serialize + Copy,
    {
        let entries: Vec<((usize, usize), V)> = map.iter().map(|(&k, &v)| (k, v)).collect();
        entries.serialize(serializer)
    }

    pub fn deserialize<'de, D, V>(
        deserializer: D,
    ) -> Result<HashMap<(usize, usize), V>, D::Error>
    where
        D: Deserializer<'de>,
        V: Deserialize<'de>,
    {
        let entries: Vec<((usize, usize), V)> = Vec::deserialize(deserializer)?;
        Ok(entries.into_iter().collect())
    }
}

mod tuple_set_serde {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::collections::HashSet;

    pub fn serialize<S>(set: &HashSet<(usize, usize)>, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let entries: Vec<(usize, usize)> = set.iter().copied().collect();
        entries.serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<HashSet<(usize, usize)>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let entries: Vec<(usize, usize)> = Vec::deserialize(deserializer)?;
        Ok(entries.into_iter().collect())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Document {
    pub layers: Vec<Layer>,
    #[serde(default)]
    pub sort_mode: SortMode,
    #[serde(default)]
    pub manual_sequence: Vec<(usize, usize, Option<usize>)>,
    #[serde(default, with = "tuple_map_serde")]
    pub pasted_origins: HashMap<(usize, usize), PastedOrigin>,
    #[serde(default)]
    pub next_instance_id: usize,
    #[serde(default)]
    pub saved_cut_order: Vec<(usize, usize, Option<usize>)>,
    #[serde(default)]
    pub use_saved_cut_order: bool,
    #[serde(default, with = "tuple_map_serde")]
    pub obstacle_exclusion_mm: HashMap<(usize, usize), f64>,
    #[serde(default, with = "tuple_map_serde")]
    pub entity_groups: HashMap<(usize, usize), u64>,
    #[serde(default)]
    pub next_group_id: u64,
    #[serde(default, with = "tuple_set_serde")]
    pub locked_entities: std::collections::HashSet<(usize, usize)>,
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
                Layer::new_travel(),
            ],
            sort_mode: SortMode::default(),
            manual_sequence: Vec::new(),
            pasted_origins: HashMap::new(),
            next_instance_id: 0,
            saved_cut_order: Vec::new(),
            use_saved_cut_order: false,
            obstacle_exclusion_mm: HashMap::new(),
            entity_groups: HashMap::new(),
            next_group_id: 0,
            locked_entities: std::collections::HashSet::new(),
        }
    }

    /// Ensure the dedicated Obstacle layer exists and return its layer index.
    pub fn ensure_obstacle_layer(&mut self) -> usize {
        if let Some(idx) = self.layers.iter().position(|l| l.is_obstacle) {
            return idx;
        }
        let insert_at = self
            .layers
            .iter()
            .position(|l| l.is_travel)
            .unwrap_or(self.layers.len());
        self.layers.insert(insert_at, Layer::new_obstacle());
        insert_at
    }

    /// Exclusion radius (mm) for an obstacle entity (defaults to `1.0` mm).
    pub fn obstacle_exclusion_radius(&self, layer_idx: usize, entity_idx: usize) -> f64 {
        self.obstacle_exclusion_mm
            .get(&(layer_idx, entity_idx))
            .copied()
            .unwrap_or(1.0)
            .max(0.0)
    }

    /// Set exclusion radius (mm, clamped >= 0.0) for an obstacle entity.
    pub fn set_obstacle_exclusion_radius(&mut self, layer_idx: usize, entity_idx: usize, radius_mm: f64) {
        self.obstacle_exclusion_mm
            .insert((layer_idx, entity_idx), radius_mm.max(0.0));
    }

    /// Return the outward-offset exclusion polygons (world mm) for a given obstacle entity.
    pub fn obstacle_exclusion_polygons(&self, layer_idx: usize, entity_idx: usize) -> Vec<Vec<Point>> {
        let Some(layer) = self.layers.get(layer_idx) else {
            return Vec::new();
        };
        let Some(ent) = layer.entities.get(entity_idx) else {
            return Vec::new();
        };
        let r = self.obstacle_exclusion_radius(layer_idx, entity_idx);
        offset_entity_outwards(ent, r)
    }

    /// Ensure the dedicated Empty Move / Travel layer exists in `self.layers`.
    pub fn ensure_travel_layer(&mut self) {
        if !self.layers.iter().any(|l| l.is_travel) {
            self.layers.push(Layer::new_travel());
        }
    }

    /// Effective rapid/empty-move travel speed from the Empty Move / Travel layer (or fallback).
    pub fn travel_speed_mm_s(&self, fallback_mm_s: f64) -> f64 {
        self.layers
            .iter()
            .find(|l| l.is_travel)
            .map(|l| l.params.speed_mm_s.max(1.0))
            .unwrap_or_else(|| fallback_mm_s.max(10.0))
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

    /// Hit-test individual cuttable paths across visible non-obstacle layers.
    /// For `Entity::Group`, returns `(layer_idx, entity_idx, Some(child_idx))` for the specific child path clicked.
    pub fn hit_test_path(&self, pt: Point, tolerance_mm: f64) -> Option<(usize, usize, Option<usize>)> {
        let mut closest: Option<((usize, usize, Option<usize>), f64)> = None;

        for (l_idx, layer) in self.layers.iter().enumerate() {
            if !layer.visible || layer.is_travel || layer.is_obstacle {
                continue;
            }
            for (e_idx, entity) in layer.entities.iter().enumerate() {
                match entity {
                    Entity::Group { children } => {
                        for (c_idx, child) in children.iter().enumerate() {
                            let d = child.distance_to_point(pt);
                            if d <= tolerance_mm && closest.map_or(true, |(_, cur)| d < cur) {
                                closest = Some(((l_idx, e_idx, Some(c_idx)), d));
                            }
                        }
                    }
                    _ => {
                        let d = entity.distance_to_point(pt);
                        if d <= tolerance_mm && closest.map_or(true, |(_, cur)| d < cur) {
                            closest = Some(((l_idx, e_idx, None), d));
                        }
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

    /// True if `(l_idx, e_idx)` is locked against accidental displacement/transforms.
    pub fn is_entity_locked(&self, l_idx: usize, e_idx: usize) -> bool {
        self.locked_entities.contains(&(l_idx, e_idx))
    }

    /// True if any entity in `selection` is locked.
    pub fn is_selection_locked(&self, selection: &[(usize, usize)]) -> bool {
        selection
            .iter()
            .any(|&(l, e)| self.locked_entities.contains(&(l, e)))
    }

    /// Lock or unlock all entities in `selection`.
    pub fn set_selection_locked(&mut self, selection: &[(usize, usize)], locked: bool) {
        for &pair in selection {
            if locked {
                self.locked_entities.insert(pair);
            } else {
                self.locked_entities.remove(&pair);
            }
        }
    }

    /// Translate all selected entities by (dx, dy) in millimeters (no-op if any selected entity is locked).
    pub fn translate_selection(&mut self, selection: &[(usize, usize)], dx: f64, dy: f64) {
        if self.is_selection_locked(selection) || (dx.abs() < 1e-9 && dy.abs() < 1e-9) {
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

    /// Scale selected entities around `pivot` (no-op if locked).
    pub fn scale_selection_around(
        &mut self,
        selection: &[(usize, usize)],
        pivot: Point,
        sx: f64,
        sy: f64,
    ) {
        if self.is_selection_locked(selection) {
            return;
        }
        for &(l_idx, e_idx) in selection {
            if let Some(layer) = self.layers.get_mut(l_idx) {
                if let Some(entity) = layer.entities.get_mut(e_idx) {
                    entity.scale_around(pivot, sx, sy);
                }
            }
        }
    }

    /// Scale selected entities around their combined bounding-box center (no-op if locked).
    pub fn scale_selection(&mut self, selection: &[(usize, usize)], sx: f64, sy: f64) {
        if self.is_selection_locked(selection) {
            return;
        }
        let Some(bounds) = self.selection_bounding_box(selection) else { return; };
        let center = Point::new(
            (bounds.min_x() + bounds.max_x()) * 0.5,
            (bounds.min_y() + bounds.max_y()) * 0.5,
        );
        self.scale_selection_around(selection, center, sx, sy);
    }

    /// Rotate selected entities counter-clockwise around their combined bounding-box center by `angle_deg` degrees (no-op if locked).
    pub fn rotate_selection(&mut self, selection: &[(usize, usize)], angle_deg: f64) {
        if self.is_selection_locked(selection) || angle_deg.abs() < 1e-9 {
            return;
        }
        let Some(bounds) = self.selection_bounding_box(selection) else { return; };
        let center = Point::new(
            (bounds.min_x() + bounds.max_x()) * 0.5,
            (bounds.min_y() + bounds.max_y()) * 0.5,
        );
        let rad = angle_deg.to_radians();
        for &(l_idx, e_idx) in selection {
            if let Some(layer) = self.layers.get_mut(l_idx) {
                if let Some(entity) = layer.entities.get_mut(e_idx) {
                    entity.rotate_around(center, rad);
                }
            }
        }
    }

    /// Mirror selected entities horizontally across their bounding box center (no-op if locked).
    pub fn mirror_selection_horizontal(&mut self, selection: &[(usize, usize)]) {
        if self.is_selection_locked(selection) {
            return;
        }
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

    /// Mirror selected entities vertically across their bounding box center (no-op if locked).
    pub fn mirror_selection_vertical(&mut self, selection: &[(usize, usize)]) {
        if self.is_selection_locked(selection) {
            return;
        }
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
                        match &self.layers[l_idx].entities[new_idx] {
                            Entity::Group { children } => {
                                for c_idx in 0..children.len() {
                                    self.manual_sequence.push((l_idx, new_idx, Some(c_idx)));
                                }
                            }
                            _ => self.manual_sequence.push((l_idx, new_idx, None)),
                        }
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
                for e in &e_indices {
                    if *e < layer.entities.len() {
                        layer.entities.remove(*e);
                    }
                }
                // Remap entity_groups and obstacle_exclusion_mm after removals in l_idx
                let remap_index = |old_e: usize| -> Option<usize> {
                    if e_indices.contains(&old_e) {
                        None
                    } else {
                        let removed_below = e_indices.iter().filter(|&&r| r < old_e).count();
                        Some(old_e - removed_below)
                    }
                };
                if !self.entity_groups.is_empty() {
                    let mut next = HashMap::with_capacity(self.entity_groups.len());
                    for ((gl, ge), gid) in self.entity_groups.drain() {
                        if gl == l_idx {
                            if let Some(ne) = remap_index(ge) {
                                next.insert((gl, ne), gid);
                            }
                        } else {
                            next.insert((gl, ge), gid);
                        }
                    }
                    self.entity_groups = next;
                }
                if !self.obstacle_exclusion_mm.is_empty() {
                    let mut next = HashMap::with_capacity(self.obstacle_exclusion_mm.len());
                    for ((ol, oe), r) in self.obstacle_exclusion_mm.drain() {
                        if ol == l_idx {
                            if let Some(ne) = remap_index(oe) {
                                next.insert((ol, ne), r);
                            }
                        } else {
                            next.insert((ol, oe), r);
                        }
                    }
                    self.obstacle_exclusion_mm = next;
                }
                if !self.locked_entities.is_empty() {
                    let mut next = std::collections::HashSet::with_capacity(self.locked_entities.len());
                    for (ll, le) in self.locked_entities.drain() {
                        if ll == l_idx {
                            if let Some(ne) = remap_index(le) {
                                next.insert((ll, ne));
                            }
                        } else {
                            next.insert((ll, le));
                        }
                    }
                    self.locked_entities = next;
                }
            }
        }
    }

    /// Expand `selection` to include all members of any cross-layer logical group
    /// (`entity_groups`) that overlaps `selection`.
    pub fn expand_selection_with_groups(&self, selection: &mut Vec<(usize, usize)>) {
        if self.entity_groups.is_empty() || selection.is_empty() {
            return;
        }
        let gids: std::collections::HashSet<u64> = selection
            .iter()
            .filter_map(|pair| self.entity_groups.get(pair).copied())
            .collect();
        if gids.is_empty() {
            return;
        }
        for (&pair, &gid) in &self.entity_groups {
            if gids.contains(&gid) && !selection.contains(&pair) {
                if self
                    .layers
                    .get(pair.0)
                    .and_then(|l| l.entities.get(pair.1))
                    .is_some()
                {
                    selection.push(pair);
                }
            }
        }
    }

    /// Group selected entities into a single compound object.
    /// When entities span multiple layers (e.g. cutting paths + obstacles), each entity
    /// retains its own layer and identity via a cross-layer logical group ID.
    /// Single-layer selections are combined into `Entity::Group` with cut order preserved.
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

        let has_obstacle = selected_list
            .iter()
            .any(|&(l, _)| self.layers.get(l).map_or(false, |ly| ly.is_obstacle));
        let has_non_obstacle = selected_list
            .iter()
            .any(|&(l, _)| self.layers.get(l).map_or(false, |ly| !ly.is_obstacle));
        if has_obstacle && has_non_obstacle {
            self.expand_selection_with_groups(selection);
            self.next_group_id += 1;
            let gid = self.next_group_id;
            for &pair in selection.iter() {
                self.entity_groups.insert(pair, gid);
            }
            return selection.first().copied();
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

    /// Ungroup any selected Group entities or cross-layer logical groups back into individual entities.
    pub fn ungroup_selection(&mut self, selection: &mut Vec<(usize, usize)>) {
        self.pasted_origins.clear();
        self.saved_cut_order.clear();
        self.use_saved_cut_order = false;

        let gids_to_dissolve: std::collections::HashSet<u64> = selection
            .iter()
            .filter_map(|pair| self.entity_groups.get(pair).copied())
            .collect();
        if !gids_to_dissolve.is_empty() {
            self.entity_groups
                .retain(|_, gid| !gids_to_dissolve.contains(gid));
        }

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

    /// Check if the selection contains any Group entity or cross-layer logical group member.
    pub fn selection_has_group(&self, selection: &[(usize, usize)]) -> bool {
        for &(l, e) in selection {
            if self.entity_groups.contains_key(&(l, e)) {
                return true;
            }
            if let Some(layer) = self.layers.get(l) {
                if let Some(Entity::Group { .. }) = layer.entities.get(e) {
                    return true;
                }
            }
        }
        false
    }

    /// Add a new custom layer below existing layers (preserved across Auto-Classify).
    pub fn add_custom_layer(&mut self) -> usize {
        const PALETTE: [Color; 7] = [
            Color::CYAN,
            Color { r: 255, g: 165, b: 40 },
            Color::BLUE,
            Color::YELLOW,
            Color::MAGENTA,
            Color::GREEN,
            Color::RED,
        ];
        let mut n = 1;
        let name = loop {
            let candidate = format!("Custom {}", n);
            if !self
                .layers
                .iter()
                .any(|l| l.name.eq_ignore_ascii_case(&candidate))
            {
                break candidate;
            }
            n += 1;
        };
        let color = PALETTE[self.layers.len() % PALETTE.len()];
        let mut layer = Layer::new(name, color);
        layer.is_custom = true;
        if let Some(travel_pos) = self.layers.iter().position(|l| l.is_travel) {
            self.layers.insert(travel_pos, layer);
            travel_pos
        } else {
            self.layers.push(layer);
            self.layers.len() - 1
        }
    }

    /// Move all entities in `selection` to `target_layer_idx`, updating `selection` and
    /// remapping cut-order / manual-sequence / pasted-origin indices so cut ordering is preserved.
    pub fn move_selection_to_layer(
        &mut self,
        selection: &mut Vec<(usize, usize)>,
        target_layer_idx: usize,
    ) -> bool {
        if target_layer_idx >= self.layers.len() || selection.is_empty() {
            return false;
        }

        let selected_set: std::collections::HashSet<(usize, usize)> = selection
            .iter()
            .copied()
            .filter(|&(l, e)| {
                l != target_layer_idx
                    && self
                        .layers
                        .get(l)
                        .map_or(false, |layer| e < layer.entities.len())
            })
            .collect();

        if selected_set.is_empty() {
            return false;
        }

        let pre_seq = self.compute_cut_sequence();
        if self.sort_mode == SortMode::Manual && self.manual_sequence.is_empty() {
            for info in &pre_seq {
                let entry = (info.layer_idx, info.entity_idx, info.child_idx);
                if !self.manual_sequence.contains(&entry) {
                    self.manual_sequence.push(entry);
                }
            }
        }

        let mut old_to_new: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
        let mut moved_entities: Vec<Entity> = Vec::new();
        let target_base = self.layers[target_layer_idx].entities.len();

        for (e_idx, _) in self.layers[target_layer_idx].entities.iter().enumerate() {
            old_to_new.insert((target_layer_idx, e_idx), (target_layer_idx, e_idx));
        }

        for (l_idx, layer) in self.layers.iter_mut().enumerate() {
            if l_idx == target_layer_idx {
                continue;
            }
            let mut kept = Vec::with_capacity(layer.entities.len());
            for (e_idx, ent) in layer.entities.drain(..).enumerate() {
                if selected_set.contains(&(l_idx, e_idx)) {
                    let new_e = target_base + moved_entities.len();
                    old_to_new.insert((l_idx, e_idx), (target_layer_idx, new_e));
                    moved_entities.push(ent);
                } else {
                    let new_e = kept.len();
                    old_to_new.insert((l_idx, e_idx), (l_idx, new_e));
                    kept.push(ent);
                }
            }
            layer.entities = kept;
        }

        self.layers[target_layer_idx].entities.extend(moved_entities);

        for pair in selection.iter_mut() {
            if let Some(&new_pair) = old_to_new.get(pair) {
                *pair = new_pair;
            }
        }

        for (l_idx, e_idx, _) in &mut self.manual_sequence {
            if let Some(&(nl, ne)) = old_to_new.get(&(*l_idx, *e_idx)) {
                *l_idx = nl;
                *e_idx = ne;
            }
        }

        for (l_idx, e_idx, _) in &mut self.saved_cut_order {
            if let Some(&(nl, ne)) = old_to_new.get(&(*l_idx, *e_idx)) {
                *l_idx = nl;
                *e_idx = ne;
            }
        }

        if !self.pasted_origins.is_empty() {
            let mut remapped = HashMap::with_capacity(self.pasted_origins.len());
            for ((old_l, old_e), mut origin) in self.pasted_origins.drain() {
                let (nl, ne) = old_to_new.get(&(old_l, old_e)).copied().unwrap_or((old_l, old_e));
                let (pl, pe) = old_to_new
                    .get(&(origin.parent_layer, origin.parent_entity))
                    .copied()
                    .unwrap_or((origin.parent_layer, origin.parent_entity));
                origin.parent_layer = pl;
                origin.parent_entity = pe;
                remapped.insert((nl, ne), origin);
            }
            self.pasted_origins = remapped;
        }

        if !self.obstacle_exclusion_mm.is_empty() {
            let mut remapped = HashMap::with_capacity(self.obstacle_exclusion_mm.len());
            for ((old_l, old_e), r) in self.obstacle_exclusion_mm.drain() {
                let (nl, ne) = old_to_new.get(&(old_l, old_e)).copied().unwrap_or((old_l, old_e));
                remapped.insert((nl, ne), r);
            }
            self.obstacle_exclusion_mm = remapped;
        }

        if !self.entity_groups.is_empty() {
            let mut remapped = HashMap::with_capacity(self.entity_groups.len());
            for ((old_l, old_e), gid) in self.entity_groups.drain() {
                let (nl, ne) = old_to_new.get(&(old_l, old_e)).copied().unwrap_or((old_l, old_e));
                remapped.insert((nl, ne), gid);
            }
            self.entity_groups = remapped;
        }

        if !self.locked_entities.is_empty() {
            let mut remapped = std::collections::HashSet::with_capacity(self.locked_entities.len());
            for (old_l, old_e) in self.locked_entities.drain() {
                let (nl, ne) = old_to_new.get(&(old_l, old_e)).copied().unwrap_or((old_l, old_e));
                remapped.insert((nl, ne));
            }
            self.locked_entities = remapped;
        }

        true
    }

    /// Auto-detects and classifies profiles into Outer Profiles, Interior Profiles, and Bend / Open Lines.
    /// Unpacks grouped entities so nested holes/lines inside groups are classified accurately,
    /// while preserving any user-created custom layers (`is_custom == true`).
    pub fn auto_classify_profiles(&mut self) {
        self.manual_sequence.clear();
        self.pasted_origins.clear();
        self.saved_cut_order.clear();
        self.use_saved_cut_order = false;

        fn flatten_entities(entities: Vec<Entity>, out: &mut Vec<Entity>) {
            for ent in entities {
                match ent {
                    Entity::Group { children } => flatten_entities(children, out),
                    other => out.push(other),
                }
            }
        }

        let mut all_entities = Vec::new();
        let mut custom_layers = Vec::new();
        let mut travel_layer: Option<Layer> = None;
        let mut preserved_settings: HashMap<String, (Color, bool, CuttingParams)> = HashMap::new();
        for layer in self.layers.drain(..) {
            if layer.is_travel {
                travel_layer = Some(layer);
            } else if layer.is_custom || layer.is_obstacle {
                custom_layers.push(layer);
            } else {
                preserved_settings.insert(
                    layer.name.to_lowercase(),
                    (layer.color, layer.visible, layer.params.clone()),
                );
                flatten_entities(layer.entities, &mut all_entities);
            }
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
        let make_classified_layer = |name: &str, default_color: Color, entities: Vec<Entity>| -> Layer {
            let mut layer = Layer::new(name, default_color);
            if let Some((color, visible, params)) = preserved_settings.get(&name.to_lowercase()) {
                layer.color = *color;
                layer.visible = *visible;
                layer.params = params.clone();
            }
            layer.entities = entities;
            layer
        };

        if !outer_shapes.is_empty() {
            new_layers.push(make_classified_layer("Outer Profiles", Color::GREEN, outer_shapes));
        }

        if !interior_shapes.is_empty() {
            new_layers.push(make_classified_layer("Interior Profiles", Color::YELLOW, interior_shapes));
        }

        if !open_paths.is_empty() {
            new_layers.push(make_classified_layer("Bend / Open Lines", Color::MAGENTA, open_paths));
        }

        if new_layers.is_empty() && custom_layers.is_empty() {
            new_layers.push(make_classified_layer("Outer Profiles", Color::GREEN, Vec::new()));
        }

        new_layers.extend(custom_layers);
        new_layers.push(travel_layer.unwrap_or_else(Layer::new_travel));
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
        let effective_mode = if self.sort_mode == SortMode::MaxDistanceInterior {
            SortMode::Manual
        } else {
            self.sort_mode
        };
        let mut paths = self.compute_cut_sequence_with_mode(effective_mode);
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
            if !layer.visible || layer.is_travel || layer.is_obstacle {
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
            for &(l_idx, e_idx, c_idx) in &self.manual_sequence {
                for (i, item) in raw_paths.iter().enumerate() {
                    if !used[i]
                        && item.layer_idx == l_idx
                        && item.entity_idx == e_idx
                        && item.child_idx == c_idx
                    {
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
        self.apply_cut_sequence(&seq);
    }

    /// Reorders the entities within layers to permanently match an explicit `CutPathInfo` sequence.
    pub fn apply_cut_sequence(&mut self, seq: &[CutPathInfo]) {
        self.use_saved_cut_order = false;
        self.manual_sequence.clear();

        let mut old_to_new: HashMap<(usize, usize), usize> = HashMap::new();
        let mut old_child_to_new: HashMap<(usize, usize, usize), usize> = HashMap::new();

        for (l_idx, layer) in self.layers.iter_mut().enumerate() {
            let mut ordered_entities = Vec::with_capacity(layer.entities.len());
            let mut used = vec![false; layer.entities.len()];

            for info in seq {
                if info.layer_idx == l_idx && info.entity_idx < layer.entities.len() && !used[info.entity_idx] {
                    let new_idx = ordered_entities.len();
                    let ent = match &layer.entities[info.entity_idx] {
                        Entity::Group { children } => {
                            let mut reordered = Vec::with_capacity(children.len());
                            let mut c_used = vec![false; children.len()];
                            for s in seq {
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

    /// Assigns a specific 1-based cut order index to `(layer_idx, entity_idx, child_idx)` in manual sort mode.
    pub fn assign_manual_cut_index(
        &mut self,
        layer_idx: usize,
        entity_idx: usize,
        child_idx: Option<usize>,
        target_1based: usize,
    ) {
        self.sort_mode = SortMode::Manual;
        self.use_saved_cut_order = false;
        if self.manual_sequence.is_empty() {
            let current = self.compute_cut_sequence_with_mode(SortMode::Manual);
            for info in current {
                let entry = (info.layer_idx, info.entity_idx, info.child_idx);
                if !self.manual_sequence.contains(&entry) {
                    self.manual_sequence.push(entry);
                }
            }
        }

        let target = (layer_idx, entity_idx, child_idx);
        self.manual_sequence.retain(|&p| p != target);
        let insert_pos = target_1based.saturating_sub(1).min(self.manual_sequence.len());
        self.manual_sequence.insert(insert_pos, target);
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

    /// Collect all outward-offset exclusion polygons for every entity on any `Obstacle` layer.
    pub fn all_obstacle_exclusion_polygons(&self) -> Vec<Vec<Point>> {
        let mut exclusion_polys: Vec<Vec<Point>> = Vec::new();
        for (l_idx, layer) in self.layers.iter().enumerate() {
            if !layer.is_obstacle {
                continue;
            }
            for e_idx in 0..layer.entities.len() {
                exclusion_polys.extend(self.obstacle_exclusion_polygons(l_idx, e_idx));
            }
        }
        exclusion_polys
    }

    /// Route a rapid travel move from `start` to `goal` avoiding all `exclusion_polys`.
    pub fn route_rapid_between(
        start: Point,
        goal: Point,
        exclusion_polys: &[Vec<Point>],
    ) -> Vec<Point> {
        let direct_len = (goal.x - start.x).hypot(goal.y - start.y);
        if direct_len <= 1e-4 || exclusion_polys.is_empty() {
            return vec![start, goal];
        }
        route_around_obstacles(start, goal, exclusion_polys)
    }

    /// Build ordered rapid and cutting motion segments across the document's active cut sequence,
    /// starting from `start_origin` and returning to `start_origin` at the end.
    /// Rapid travel moves automatically route around any `Obstacle` layer exclusion zones.
    /// Re-anchor the starting point of the 1-based cut-sequence path at `target_pt`.
    pub fn set_path_start_point(&mut self, path_index_1based: usize, target_pt: Point) -> bool {
        let seq = self.compute_cut_sequence();
        let Some(info) = path_index_1based.checked_sub(1).and_then(|i| seq.get(i)) else {
            return false;
        };
        let Some(layer) = self.layers.get_mut(info.layer_idx) else {
            return false;
        };
        let Some(entity) = layer.entities.get_mut(info.entity_idx) else {
            return false;
        };
        let target = match (entity, info.child_idx) {
            (Entity::Group { children }, Some(c_idx)) => match children.get_mut(c_idx) {
                Some(c) => c,
                None => return false,
            },
            (ent, None) => ent,
            _ => return false,
        };
        target.set_start_point_near(target_pt)
    }

    pub fn build_motion_trajectory(&self, start_origin: Point, rapid_speed_mm_s: f64) -> Vec<MotionSegment> {
        let seq = self.compute_cut_sequence();
        let rapid_speed = self.travel_speed_mm_s(rapid_speed_mm_s);
        let mut segments = Vec::new();
        let mut current_pos = start_origin;

        let exclusion_polys = self.all_obstacle_exclusion_polygons();

        let push_rapid_routed = |from: Point,
                                 to: Point,
                                 path_index_1based: usize,
                                 segs: &mut Vec<MotionSegment>| {
            let direct_len = (to.x - from.x).hypot(to.y - from.y);
            if direct_len <= 1e-4 {
                return;
            }
            let waypoints = Self::route_rapid_between(from, to, &exclusion_polys);
            for pair in waypoints.windows(2) {
                let len = (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y);
                if len > 1e-4 {
                    segs.push(MotionSegment {
                        start: pair[0],
                        end: pair[1],
                        length_mm: len,
                        speed_mm_s: rapid_speed,
                        is_rapid: true,
                        path_index_1based,
                    });
                }
            }
        };

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
            push_rapid_routed(current_pos, pts[0], path_index_1based, &mut segments);

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
            push_rapid_routed(current_pos, start_origin, seq.len(), &mut segments);
        }

        segments
    }

    /// Estimate total job duration (in seconds) from `start_origin`:
    /// cutting length / layer speed + rapid travel length / travel speed + piercing time per path.
    pub fn estimate_job_time(&self, start_origin: Point) -> JobTimeEstimate {
        let traj = self.build_motion_trajectory(start_origin, 200.0);
        let mut cut_s = 0.0;
        let mut rapid_s = 0.0;
        for seg in &traj {
            if seg.is_rapid {
                rapid_s += seg.length_mm / seg.speed_mm_s.max(1.0);
            } else {
                cut_s += seg.length_mm / seg.speed_mm_s.max(0.5);
            }
        }

        let seq = self.compute_cut_sequence();
        let pierce_s: f64 = seq
            .iter()
            .filter_map(|info| self.layers.get(info.layer_idx))
            .map(|layer| (layer.params.pierce_time_ms as f64) / 1000.0)
            .sum();

        JobTimeEstimate {
            total_s: cut_s + rapid_s + pierce_s,
            cut_s,
            rapid_s,
            pierce_s,
        }
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

/// Anytime bottleneck-maximization heuristic for `SortMode::MaxDistanceInterior` ("Cool" mode).
/// Maximizes the minimal distance between any two consecutive cuts while keeping bend/open paths
/// first and outer contours last (or optimizing all paths when fewer than 2 interior paths exist).
#[derive(Debug, Clone)]
pub struct CoolHeuristicSolver {
    prefix: Vec<CutPathInfo>,
    suffix: Vec<CutPathInfo>,
    current: Vec<CutPathInfo>,
    best: Vec<CutPathInfo>,
    best_score: (f64, f64, f64),
    pub best_min_dist_mm: f64,
    pub iterations: usize,
    stagnant_iters: usize,
    rng_state: u64,
}

impl CoolHeuristicSolver {
    fn pair_dist(a: &CutPathInfo, b: &CutPathInfo) -> f64 {
        let dx = b.start_point.x - a.end_point.x;
        let dy = b.start_point.y - a.end_point.y;
        dx.hypot(dy)
    }

    /// Score tuple `(min_dist, second_min_dist, sum_dist)` — larger is better.
    fn eval_score(paths: &[CutPathInfo]) -> (f64, f64, f64) {
        if paths.len() < 2 {
            return (0.0, 0.0, 0.0);
        }
        let mut min1 = f64::INFINITY;
        let mut min2 = f64::INFINITY;
        let mut sum = 0.0;
        for w in paths.windows(2) {
            let d = Self::pair_dist(&w[0], &w[1]);
            sum += d;
            if d < min1 {
                min2 = min1;
                min1 = d;
            } else if d < min2 {
                min2 = d;
            }
        }
        if !min2.is_finite() {
            min2 = min1;
        }
        (min1, min2, sum)
    }

    fn better_score(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        if (a.0 - b.0).abs() > 1e-6 {
            a.0 > b.0
        } else if (a.1 - b.1).abs() > 1e-6 {
            a.1 > b.1
        } else {
            a.2 > b.2 + 1e-6
        }
    }

    fn next_rand(&mut self) -> usize {
        self.rng_state = self
            .rng_state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.rng_state >> 32) as usize
    }

    pub fn new(doc: &Document) -> Self {
        let current_seq = doc.compute_cut_sequence();
        let mut bend = Vec::new();
        let mut interior = Vec::new();
        let mut outer = Vec::new();
        for p in &current_seq {
            if p.is_open {
                bend.push(*p);
            } else if p.is_interior {
                interior.push(*p);
            } else {
                outer.push(*p);
            }
        }

        let (prefix, mut active, suffix) = if interior.len() >= 2 {
            (bend, interior, outer)
        } else {
            (Vec::new(), current_seq, Vec::new())
        };

        let mut best = active.clone();
        let mut best_score = Self::eval_score(&best);

        if active.len() >= 2 {
            // Seed 1: farthest-neighbor order
            let mut fn_pool = active.clone();
            let mut fn_pos = prefix.last().map_or(Point::ORIGIN, |p| p.end_point);
            let mut fn_order = Vec::with_capacity(active.len());
            Document::sequence_group_farthest_neighbor(&mut fn_pool, &mut fn_pos, &mut fn_order);
            let fn_score = Self::eval_score(&fn_order);
            if Self::better_score(fn_score, best_score) {
                best = fn_order;
                best_score = fn_score;
            }

            // Seed 2: 1D-projected interleaving (0, n/2, 1, n/2+1, ...)
            active.sort_by(|a, b| {
                let pa = a.start_point.x + a.start_point.y;
                let pb = b.start_point.x + b.start_point.y;
                pa.partial_cmp(&pb).unwrap_or(std::cmp::Ordering::Equal)
            });
            let half = (active.len() + 1) / 2;
            let mut interleaved = Vec::with_capacity(active.len());
            for i in 0..half {
                interleaved.push(active[i]);
                if i + half < active.len() {
                    interleaved.push(active[i + half]);
                }
            }
            let il_score = Self::eval_score(&interleaved);
            if Self::better_score(il_score, best_score) {
                best = interleaved;
                best_score = il_score;
            }
        }

        Self {
            prefix,
            suffix,
            current: best.clone(),
            best,
            best_score,
            best_min_dist_mm: best_score.0,
            iterations: 0,
            stagnant_iters: 0,
            rng_state: 0x9E37_79B9_7F4A_7C15,
        }
    }

    pub fn step(&mut self, max_iters: usize) {
        let n = self.current.len();
        if n < 3 {
            self.iterations += 1;
            return;
        }

        for _ in 0..max_iters {
            self.iterations += 1;

            // Find the current bottleneck edge (minimum consecutive distance)
            let mut bottleneck_i = 0usize;
            let mut min_d = f64::INFINITY;
            for i in 0..(n - 1) {
                let d = Self::pair_dist(&self.current[i], &self.current[i + 1]);
                if d < min_d {
                    min_d = d;
                    bottleneck_i = i;
                }
            }

            let mut cur_score = Self::eval_score(&self.current);
            let mut improved = false;

            // Target bottleneck endpoints (bottleneck_i or bottleneck_i + 1)
            let pivot = if (self.next_rand() & 1) == 0 {
                bottleneck_i
            } else {
                bottleneck_i + 1
            };

            let sample_count = n.min(24);
            for s in 0..sample_count {
                let j = if n <= 24 { s } else { self.next_rand() % n };
                if j == pivot {
                    continue;
                }
                self.current.swap(pivot, j);
                let cand = Self::eval_score(&self.current);
                if Self::better_score(cand, cur_score) {
                    cur_score = cand;
                    improved = true;
                    break;
                }
                self.current.swap(pivot, j);
            }

            // Also try random relocate / swap to escape bottleneck ties
            if !improved {
                let a = self.next_rand() % n;
                let b = self.next_rand() % n;
                if a != b {
                    self.current.swap(a, b);
                    let cand = Self::eval_score(&self.current);
                    if Self::better_score(cand, cur_score) {
                        cur_score = cand;
                        improved = true;
                    } else {
                        self.current.swap(a, b);
                    }
                }
            }

            if Self::better_score(cur_score, self.best_score) {
                self.best_score = cur_score;
                self.best = self.current.clone();
                self.best_min_dist_mm = cur_score.0;
                self.stagnant_iters = 0;
            } else if improved {
                self.stagnant_iters = 0;
            } else {
                self.stagnant_iters += 1;
                if self.stagnant_iters > 40 {
                    // Perturb from best to explore a fresh basin
                    self.current = self.best.clone();
                    let a = self.next_rand() % n;
                    let b = self.next_rand() % n;
                    let c = self.next_rand() % n;
                    self.current.swap(a, b);
                    self.current.swap(b, c);
                    self.stagnant_iters = 0;
                }
            }
        }
    }

    pub fn full_sequence(&self) -> Vec<CutPathInfo> {
        let mut out = Vec::with_capacity(self.prefix.len() + self.best.len() + self.suffix.len());
        out.extend_from_slice(&self.prefix);
        out.extend_from_slice(&self.best);
        out.extend_from_slice(&self.suffix);
        out
    }
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
        doc.assign_manual_cut_index(0, 2, None, 1); // Put 3rd entity at index #1
        doc.assign_manual_cut_index(0, 1, None, 2); // Put 2nd entity at index #2
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
        doc.assign_manual_cut_index(0, 2, None, 1);
        doc.assign_manual_cut_index(0, 1, None, 2);
        doc.assign_manual_cut_index(0, 0, None, 3);
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
        assert!(cut_segs.iter().filter(|s| s.path_index_1based == 1).count() >= 180);
        assert_eq!(cut_segs.iter().filter(|s| s.path_index_1based == 2).count(), 4);
    }

    #[test]
    fn test_auto_classify_grouped_paths_and_preserve_custom_layers() {
        let mut doc = Document::new();
        // Put a grouped outer rectangle + inner hole + open bend line in Layer 0
        doc.layers[0].entities.push(Entity::Group {
            children: vec![
                Entity::Polyline {
                    points: vec![
                        Point::new(0.0, 0.0),
                        Point::new(200.0, 0.0),
                        Point::new(200.0, 100.0),
                        Point::new(0.0, 100.0),
                    ],
                    closed: true,
                },
                Entity::Circle {
                    center: Point::new(50.0, 50.0),
                    radius: 15.0,
                },
                Entity::Line {
                    start: Point::new(100.0, 10.0),
                    end: Point::new(100.0, 90.0),
                },
            ],
        });

        // Add a custom layer with a custom entity before auto-classifying
        let custom_idx = doc.add_custom_layer();
        assert!(doc.layers[custom_idx].is_custom);
        let custom_name = doc.layers[custom_idx].name.clone();
        doc.layers[custom_idx].entities.push(Entity::Circle {
            center: Point::new(300.0, 300.0),
            radius: 10.0,
        });

        doc.auto_classify_profiles();

        let outer = doc.layers.iter().find(|l| l.name == "Outer Profiles").expect("Outer layer");
        let inner = doc.layers.iter().find(|l| l.name == "Interior Profiles").expect("Inner layer");
        let bend = doc.layers.iter().find(|l| l.name == "Bend / Open Lines").expect("Bend layer");
        let custom = doc.layers.iter().find(|l| l.name == custom_name).expect("Custom layer must be preserved");

        assert_eq!(outer.entities.len(), 1);
        assert_eq!(inner.entities.len(), 1);
        assert_eq!(bend.entities.len(), 1);
        assert!(custom.is_custom);
        assert_eq!(custom.entities.len(), 1);

        // User edits default Outer Profiles and Empty Move / Travel parameters, then re-runs auto_classify
        let outer_idx = doc.layers.iter().position(|l| l.name == "Outer Profiles").unwrap();
        doc.layers[outer_idx].params.speed_mm_s = 37.5;
        doc.layers[outer_idx].params.cut_height_mm = 1.8;
        doc.layers[outer_idx].params.frequency_hz = 3200;
        let travel_idx = doc.layers.iter().position(|l| l.is_travel).expect("Travel layer must exist");
        doc.layers[travel_idx].params.speed_mm_s = 350.0;

        doc.auto_classify_profiles();
        let outer_after = doc.layers.iter().find(|l| l.name == "Outer Profiles").unwrap();
        assert!((outer_after.params.speed_mm_s - 37.5).abs() < 1e-6);
        assert!((outer_after.params.cut_height_mm - 1.8).abs() < 1e-6);
        assert_eq!(outer_after.params.frequency_hz, 3200);
        let travel_after = doc.layers.iter().find(|l| l.is_travel).unwrap();
        assert!((travel_after.params.speed_mm_s - 350.0).abs() < 1e-6);

        // Verify travel layer speed drives rapid segments in build_motion_trajectory
        let traj = doc.build_motion_trajectory(Point::ORIGIN, 200.0);
        assert!((traj[0].speed_mm_s - 350.0).abs() < 1e-6);
    }

    #[test]
    fn test_move_selection_to_layer_preserves_cut_order_and_selection() {
        let mut doc = Document::new();
        let rect = Entity::Polyline {
            points: vec![
                Point::new(0.0, 0.0),
                Point::new(200.0, 0.0),
                Point::new(200.0, 100.0),
                Point::new(0.0, 100.0),
            ],
            closed: true,
        };
        let circle = Entity::Circle {
            center: Point::new(50.0, 50.0),
            radius: 10.0,
        };
        doc.layers[0].entities.push(rect);
        doc.layers[0].entities.push(circle);
        let custom_idx = doc.add_custom_layer();

        // Save custom cut order: [rect (0,0), circle (0,1)]
        doc.sort_mode = SortMode::Manual;
        doc.assign_manual_cut_index(0, 0, None, 1);
        doc.assign_manual_cut_index(0, 1, None, 2);
        doc.save_current_cut_order();

        let before_starts: Vec<Point> = doc
            .compute_cut_sequence()
            .into_iter()
            .map(|c| c.start_point)
            .collect();
        assert_eq!(before_starts.len(), 2);

        // Move rect (0, 0) to custom layer
        let mut selection = vec![(0, 0)];
        assert!(doc.move_selection_to_layer(&mut selection, custom_idx));
        assert_eq!(selection, vec![(custom_idx, 0)]);
        assert_eq!(doc.layers[0].entities.len(), 1);
        assert_eq!(doc.layers[custom_idx].entities.len(), 1);

        // Cut order sequence must remain identical!
        let after_starts: Vec<Point> = doc
            .compute_cut_sequence()
            .into_iter()
            .map(|c| c.start_point)
            .collect();
        assert_eq!(before_starts, after_starts);
    }

    #[test]
    fn test_layer_path_stats_and_job_time_estimate() {
        let mut doc = Document::new();
        // Set travel layer speed to 100 mm/s
        if let Some(travel) = doc.layers.iter_mut().find(|l| l.is_travel) {
            assert_eq!(travel.name, "Travel");
            travel.params.speed_mm_s = 100.0;
        }
        // Layer 0: speed 50 mm/s, pierce 500 ms
        doc.layers[0].params.speed_mm_s = 50.0;
        doc.layers[0].params.pierce_time_ms = 500;

        // Grouped two lines of 500 mm each = 1000 mm = 1.00 m, 2 paths
        let line1 = Entity::Line {
            start: Point::new(100.0, 0.0),
            end: Point::new(600.0, 0.0),
        };
        let line2 = Entity::Line {
            start: Point::new(600.0, 0.0),
            end: Point::new(100.0, 0.0),
        };
        doc.layers[0].entities.push(Entity::Group {
            children: vec![line1, line2],
        });

        assert_eq!(doc.layers[0].total_path_count(), 2);
        assert!((doc.layers[0].total_path_length_m() - 1.0).abs() < 1e-6);

        let est = doc.estimate_job_time(Point::new(0.0, 0.0));
        // Cut: 1000 mm / 50 mm/s = 20.0s
        // Rapid: 100 mm to start + 100 mm return = 200 mm / 100 mm/s = 2.0s
        // Pierce: 2 paths * 500 ms = 1.0s
        // Total = 23.0s
        assert!((est.cut_s - 20.0).abs() < 1e-4);
        assert!((est.rapid_s - 2.0).abs() < 1e-4);
        assert!((est.pierce_s - 1.0).abs() < 1e-4);
        assert!((est.total_s - 23.0).abs() < 1e-4);
    }

    #[test]
    fn test_obstacle_validation_offset_and_rapid_avoidance() {
        let point = Entity::Line {
            start: Point::new(10.0, 10.0),
            end: Point::new(10.0, 10.0),
        };
        let open_line = Entity::Line {
            start: Point::new(0.0, 0.0),
            end: Point::new(50.0, 0.0),
        };
        let rect = Entity::Polyline {
            points: vec![
                Point::new(40.0, -10.0),
                Point::new(60.0, -10.0),
                Point::new(60.0, 10.0),
                Point::new(40.0, 10.0),
            ],
            closed: true,
        };
        let circle = Entity::Circle {
            center: Point::new(50.0, 50.0),
            radius: 10.0,
        };

        assert!(!point.can_be_obstacle());
        assert!(!open_line.can_be_obstacle());
        assert!(rect.can_be_obstacle());
        assert!(circle.can_be_obstacle());

        // Offset rectangle by 2.0 mm -> min_x=38, max_x=62, min_y=-12, max_y=12
        let offset = offset_entity_outwards(&rect, 2.0);
        assert_eq!(offset.len(), 1);
        let min_x = offset[0].iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
        let max_x = offset[0].iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
        let min_y = offset[0].iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
        let max_y = offset[0].iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);
        assert!((min_x - 38.0).abs() < 1e-4);
        assert!((max_x - 62.0).abs() < 1e-4);
        assert!((min_y - (-12.0)).abs() < 1e-4);
        assert!((max_y - 12.0).abs() < 1e-4);

        // Document with two cut lines separated by an obstacle blocking y=0 between x=40..60
        let mut doc = Document::new();
        doc.layers[0].entities.push(Entity::Line {
            start: Point::new(0.0, 0.0),
            end: Point::new(10.0, 0.0),
        });
        doc.layers[0].entities.push(Entity::Line {
            start: Point::new(90.0, 0.0),
            end: Point::new(100.0, 0.0),
        });
        let obs_layer = doc.ensure_obstacle_layer();
        doc.layers[obs_layer].entities.push(rect);
        doc.set_obstacle_exclusion_radius(obs_layer, 0, 1.0);

        // Cut sequence must exclude the obstacle (only 2 cut lines)
        assert_eq!(doc.compute_cut_sequence().len(), 2);

        // Rapid travel from (10,0) to (90,0) must route around the 1mm exclusion zone (x=39..61, y=-11..11)
        let traj = doc.build_motion_trajectory(Point::new(0.0, 0.0), 200.0);
        let rapid_segs: Vec<_> = traj.iter().filter(|s| s.is_rapid).collect();
        assert!(rapid_segs.len() > 2); // Must detour around obstacle!
        let inner_exclusion = &offset_entity_outwards(&doc.layers[obs_layer].entities[0], 0.95)[0];
        for seg in rapid_segs {
            assert!(
                !segment_intersects_polygon(seg.start, seg.end, inner_exclusion),
                "Rapid segment {:?} -> {:?} traversed obstacle exclusion zone!",
                seg.start,
                seg.end
            );
        }
    }

    #[test]
    fn test_obstacle_positional_change_retriggers_routing() {
        let mut doc = Document::new();
        doc.layers[0].entities.push(Entity::Line {
            start: Point::new(0.0, 0.0),
            end: Point::new(10.0, 0.0),
        });
        doc.layers[0].entities.push(Entity::Line {
            start: Point::new(90.0, 0.0),
            end: Point::new(100.0, 0.0),
        });
        let obs_layer = doc.ensure_obstacle_layer();
        // Start obstacle well above the rapid line (y = 50..70) -> direct path
        doc.layers[obs_layer].entities.push(Entity::Polyline {
            points: vec![
                Point::new(40.0, 50.0),
                Point::new(60.0, 50.0),
                Point::new(60.0, 70.0),
                Point::new(40.0, 70.0),
            ],
            closed: true,
        });
        doc.set_obstacle_exclusion_radius(obs_layer, 0, 2.0);

        let polys_clear = doc.all_obstacle_exclusion_polygons();
        let direct = Document::route_rapid_between(Point::new(10.0, 0.0), Point::new(90.0, 0.0), &polys_clear);
        assert_eq!(direct.len(), 2);

        // Move obstacle down by -60mm so it blocks y = -10..10 (exclusion y = -12..12)
        doc.translate_selection(&[(obs_layer, 0)], 0.0, -60.0);
        let polys_blocked = doc.all_obstacle_exclusion_polygons();
        let detoured = Document::route_rapid_between(Point::new(10.0, 0.0), Point::new(90.0, 0.0), &polys_blocked);
        assert!(detoured.len() > 2, "Moving obstacle into path must re-route around exclusion zone");

        let inner_excl = &offset_entity_outwards(&doc.layers[obs_layer].entities[0], 1.9)[0];
        for pair in detoured.windows(2) {
            assert!(!segment_intersects_polygon(pair[0], pair[1], inner_excl));
        }
    }

    #[test]
    fn test_concave_zigzag_obstacle_routing() {
        // Thin M-shaped / zigzag closed polygon similar to a hand-drawn spline obstacle
        let zigzag = Entity::Polyline {
            points: vec![
                Point::new(100.0, 200.0),
                Point::new(140.0, 200.0),
                Point::new(160.0, 140.0),
                Point::new(190.0, 200.0),
                Point::new(220.0, 140.0),
                Point::new(250.0, 200.0),
                Point::new(250.0, 205.0),
                Point::new(215.0, 150.0),
                Point::new(190.0, 208.0),
                Point::new(160.0, 150.0),
                Point::new(135.0, 205.0),
                Point::new(100.0, 205.0),
            ],
            closed: true,
        };
        let excl = offset_entity_outwards(&zigzag, 1.0);
        let inner_excl = &offset_entity_outwards(&zigzag, 0.9)[0];

        let test_pairs = [
            (Point::new(50.0, 175.0), Point::new(300.0, 175.0)),
            (Point::new(120.0, 100.0), Point::new(230.0, 260.0)),
            (Point::new(80.0, 250.0), Point::new(270.0, 120.0)),
            (Point::new(175.0, 100.0), Point::new(175.0, 260.0)),
        ];
        for (a, b) in test_pairs {
            let route = Document::route_rapid_between(a, b, &excl);
            assert!(route.len() > 2, "Expected detour around zigzag for {:?} -> {:?}", a, b);
            for pair in route.windows(2) {
                assert!(
                    !segment_intersects_polygon(pair[0], pair[1], inner_excl),
                    "Route segment {:?} -> {:?} crossed zigzag obstacle!",
                    pair[0],
                    pair[1]
                );
            }
        }
    }

    #[test]
    fn test_group_mixed_paths_and_obstacles_preserves_identity() {
        let mut doc = Document::new();
        doc.layers[0].entities.push(Entity::Line {
            start: Point::new(0.0, 0.0),
            end: Point::new(10.0, 0.0),
        });
        doc.layers[0].entities.push(Entity::Line {
            start: Point::new(90.0, 0.0),
            end: Point::new(100.0, 0.0),
        });
        let obs_layer = doc.ensure_obstacle_layer();
        doc.layers[obs_layer].entities.push(Entity::Polyline {
            points: vec![
                Point::new(40.0, -10.0),
                Point::new(60.0, -10.0),
                Point::new(60.0, 10.0),
                Point::new(40.0, 10.0),
            ],
            closed: true,
        });
        doc.set_obstacle_exclusion_radius(obs_layer, 0, 2.0);

        // Group path 0 (layer 0) and the obstacle (obs_layer)
        let mut sel = vec![(0, 0), (obs_layer, 0)];
        doc.group_selection(&mut sel);

        // Obstacle must remain on its obstacle layer and NOT be cut
        assert!(doc.layers[obs_layer].is_obstacle);
        assert_eq!(doc.layers[obs_layer].entities.len(), 1);
        assert_eq!(doc.compute_cut_sequence().len(), 2);
        assert!(doc.selection_has_group(&sel));
        assert_eq!(doc.obstacle_exclusion_radius(obs_layer, 0), 2.0);

        // Rapid routing must still detour around the obstacle
        let polys = doc.all_obstacle_exclusion_polygons();
        let detoured = Document::route_rapid_between(Point::new(10.0, 0.0), Point::new(90.0, 0.0), &polys);
        assert!(detoured.len() > 2);

        // Clicking just the path expands selection to include the grouped obstacle
        let mut click_sel = vec![(0, 0)];
        doc.expand_selection_with_groups(&mut click_sel);
        assert!(click_sel.contains(&(0, 0)));
        assert!(click_sel.contains(&(obs_layer, 0)));

        // Ungrouping dissolves the cross-layer group while keeping both in their layers
        doc.ungroup_selection(&mut click_sel);
        assert!(!doc.selection_has_group(&click_sel));
        assert_eq!(doc.layers[obs_layer].entities.len(), 1);
    }

    #[test]
    fn test_scale_and_rotate_selection() {
        let mut doc = Document::new();
        // 40x20 rectangle centered at (50, 50)
        doc.layers[0].entities.push(Entity::Polyline {
            points: vec![
                Point::new(30.0, 40.0),
                Point::new(70.0, 40.0),
                Point::new(70.0, 60.0),
                Point::new(30.0, 60.0),
            ],
            closed: true,
        });
        // Circle of radius 10 at (50, 50)
        doc.layers[0].entities.push(Entity::Circle {
            center: Point::new(50.0, 50.0),
            radius: 10.0,
        });

        let sel_rect = vec![(0, 0)];
        doc.scale_selection(&sel_rect, 2.0, 1.5);
        let b1 = doc.selection_bounding_box(&sel_rect).unwrap();
        assert!((b1.width() - 80.0).abs() < 1e-6);
        assert!((b1.height() - 30.0).abs() < 1e-6);

        // Rotate 90 degrees -> width and height swap
        doc.rotate_selection(&sel_rect, 90.0);
        let b2 = doc.selection_bounding_box(&sel_rect).unwrap();
        assert!((b2.width() - 30.0).abs() < 1e-6);
        assert!((b2.height() - 80.0).abs() < 1e-6);

        // Uniform scale on Circle scales radius; non-uniform converts to closed polyline
        let sel_circ = vec![(0, 1)];
        doc.scale_selection(&sel_circ, 2.0, 2.0);
        match doc.layers[0].entities[1] {
            Entity::Circle { radius, .. } => assert!((radius - 20.0).abs() < 1e-6),
            _ => panic!("Uniform scale should keep Circle"),
        }
        doc.scale_selection(&sel_circ, 1.0, 0.5);
        let bc = doc.selection_bounding_box(&sel_circ).unwrap();
        assert!((bc.width() - 40.0).abs() < 0.2);
        assert!((bc.height() - 20.0).abs() < 0.2);
    }

    #[test]
    fn test_locked_entities_prevent_displacement() {
        let mut doc = Document::new();
        doc.layers[0].entities.push(Entity::Line {
            start: Point::new(10.0, 10.0),
            end: Point::new(30.0, 10.0),
        });
        let sel = vec![(0, 0)];
        doc.set_selection_locked(&sel, true);
        assert!(doc.is_selection_locked(&sel));

        doc.translate_selection(&sel, 50.0, 50.0);
        doc.scale_selection(&sel, 2.0, 2.0);
        doc.rotate_selection(&sel, 90.0);
        doc.mirror_selection_horizontal(&sel);
        let b = doc.selection_bounding_box(&sel).unwrap();
        assert!((b.min_x() - 10.0).abs() < 1e-6);
        assert!((b.max_x() - 30.0).abs() < 1e-6);

        doc.set_selection_locked(&sel, false);
        doc.translate_selection(&sel, 50.0, 0.0);
        let b2 = doc.selection_bounding_box(&sel).unwrap();
        assert!((b2.min_x() - 60.0).abs() < 1e-6);
    }

    #[test]
    fn test_manual_sort_individual_paths_inside_group() {
        let mut doc = Document::new();
        doc.layers[0].entities.push(Entity::Group {
            children: vec![
                Entity::Line {
                    start: Point::new(0.0, 0.0),
                    end: Point::new(10.0, 0.0),
                },
                Entity::Line {
                    start: Point::new(0.0, 20.0),
                    end: Point::new(10.0, 20.0),
                },
                Entity::Line {
                    start: Point::new(0.0, 40.0),
                    end: Point::new(10.0, 40.0),
                },
            ],
        });

        // Clicking on the 3rd child path inside the group hits (0, 0, Some(2))
        let hit = doc.hit_test_path(Point::new(5.0, 40.0), 2.0);
        assert_eq!(hit, Some((0, 0, Some(2))));

        // Assigning child 2 to index 1, child 0 to index 2, child 1 to index 3 reorders only those individual paths
        doc.assign_manual_cut_index(0, 0, Some(2), 1);
        doc.assign_manual_cut_index(0, 0, Some(0), 2);
        doc.assign_manual_cut_index(0, 0, Some(1), 3);

        let seq = doc.compute_cut_sequence();
        assert_eq!(seq.len(), 3);
        assert_eq!(seq[0].child_idx, Some(2));
        assert_eq!(seq[1].child_idx, Some(0));
        assert_eq!(seq[2].child_idx, Some(1));
    }
}

/// Compute outward-offset closed polygons for an entity at constant perpendicular distance `radius_mm`.
/// For closed polylines, convex corners are rounded with circular arcs of radius `radius_mm` so the
/// distance between the obstacle and the expanded boundary is everywhere `radius_mm`.
pub fn offset_entity_outwards(entity: &Entity, radius_mm: f64) -> Vec<Vec<Point>> {
    let r = radius_mm.max(0.0);
    match entity {
        Entity::Circle { center, radius } => {
            let new_r = (radius + r).max(0.0);
            let segs = 64usize;
            let poly = (0..segs)
                .map(|i| {
                    let ang = std::f64::consts::TAU * (i as f64) / (segs as f64);
                    Point::new(center.x + new_r * ang.cos(), center.y + new_r * ang.sin())
                })
                .collect();
            vec![poly]
        }
        Entity::Polyline { points, closed: true } => {
            let poly = offset_polygon_outwards(points, r);
            if poly.len() >= 3 {
                vec![poly]
            } else {
                Vec::new()
            }
        }
        Entity::Group { children } => children
            .iter()
            .flat_map(|c| offset_entity_outwards(c, r))
            .collect(),
        _ => Vec::new(),
    }
}

fn offset_polygon_outwards(points: &[Point], r: f64) -> Vec<Point> {
    // Deduplicate consecutive identical points
    let mut clean: Vec<Point> = Vec::with_capacity(points.len());
    for &p in points {
        if clean
            .last()
            .map_or(true, |prev| (p.x - prev.x).hypot(p.y - prev.y) > 1e-6)
        {
            clean.push(p);
        }
    }
    if clean.len() >= 2 {
        let first = clean[0];
        let last = *clean.last().unwrap();
        if (first.x - last.x).hypot(first.y - last.y) <= 1e-6 {
            clean.pop();
        }
    }
    if clean.len() < 3 {
        return Vec::new();
    }
    if r <= 1e-6 {
        return clean;
    }

    // Ensure CCW winding so outward normal of edge (a -> b) is (dy/len, -dx/len)
    let mut signed_area = 0.0;
    let n = clean.len();
    for i in 0..n {
        let j = (i + 1) % n;
        signed_area += clean[i].x * clean[j].y - clean[j].x * clean[i].y;
    }
    if signed_area < 0.0 {
        clean.reverse();
    }

    let mut out = Vec::with_capacity(n * 4);
    for i in 0..n {
        let prev = clean[(i + n - 1) % n];
        let curr = clean[i];
        let next = clean[(i + 1) % n];

        let e1x = curr.x - prev.x;
        let e1y = curr.y - prev.y;
        let l1 = e1x.hypot(e1y);
        let e2x = next.x - curr.x;
        let e2y = next.y - curr.y;
        let l2 = e2x.hypot(e2y);
        if l1 < 1e-9 || l2 < 1e-9 {
            continue;
        }

        // Outward unit normals for CCW polygon
        let n1x = e1y / l1;
        let n1y = -e1x / l1;
        let n2x = e2y / l2;
        let n2y = -e2x / l2;

        let cross = (e1x / l1) * (e2y / l2) - (e1y / l1) * (e2x / l2);
        if cross >= 0.0 {
            // Convex corner: round with arc of radius r from angle(n1) to angle(n2)
            let a1 = n1y.atan2(n1x);
            let mut a2 = n2y.atan2(n2x);
            while a2 < a1 {
                a2 += std::f64::consts::TAU;
            }
            let sweep = a2 - a1;
            let steps = ((sweep / 0.35).ceil() as usize).clamp(2, 8);
            for s in 0..=steps {
                let ang = a1 + sweep * (s as f64) / (steps as f64);
                out.push(Point::new(curr.x + r * ang.cos(), curr.y + r * ang.sin()));
            }
        } else {
            // Concave corner: intersect the two offset lines
            let p1 = Point::new(curr.x + n1x * r, curr.y + n1y * r);
            let p2 = Point::new(curr.x + n2x * r, curr.y + n2y * r);
            let denom = (e1x / l1) * (e2y / l2) - (e1y / l1) * (e2x / l2);
            if denom.abs() > 1e-6 {
                let dx = p2.x - p1.x;
                let dy = p2.y - p1.y;
                let t = (dx * (e2y / l2) - dy * (e2x / l2)) / denom;
                out.push(Point::new(p1.x + (e1x / l1) * t, p1.y + (e1y / l1) * t));
            } else {
                out.push(p1);
            }
        }
    }
    out
}

fn segments_intersect_strict(a: Point, b: Point, c: Point, d: Point) -> bool {
    let orient = |p: Point, q: Point, r: Point| -> f64 {
        (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)
    };
    let o1 = orient(a, b, c);
    let o2 = orient(a, b, d);
    let o3 = orient(c, d, a);
    let o4 = orient(c, d, b);
    if (o1 > 1e-7 && o2 < -1e-7 || o1 < -1e-7 && o2 > 1e-7)
        && (o3 > 1e-7 && o4 < -1e-7 || o3 < -1e-7 && o4 > 1e-7)
    {
        return true;
    }
    false
}

/// True if segment `a -> b` enters or crosses `poly`.
pub fn segment_intersects_polygon(a: Point, b: Point, poly: &[Point]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    for &t in &[0.25, 0.5, 0.75] {
        let pt = Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
        if point_in_polygon(pt, poly) {
            return true;
        }
    }
    let n = poly.len();
    for i in 0..n {
        let p1 = poly[i];
        let p2 = poly[(i + 1) % n];
        if segments_intersect_strict(a, b, p1, p2) {
            return true;
        }
    }
    false
}

fn convex_hull(pts: &[Point]) -> Vec<Point> {
    if pts.len() <= 3 {
        return pts.to_vec();
    }
    let mut sorted = pts.to_vec();
    sorted.sort_by(|a, b| {
        a.x.partial_cmp(&b.x)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.y.partial_cmp(&b.y).unwrap_or(std::cmp::Ordering::Equal))
    });
    let cross = |o: Point, a: Point, b: Point| -> f64 {
        (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
    };
    let mut lower = Vec::with_capacity(sorted.len());
    for &p in &sorted {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 1e-9 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper = Vec::with_capacity(sorted.len());
    for &p in sorted.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 1e-9 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn push_nudged_vertices(poly: &[Point], nudge: f64, obstacles: &[Vec<Point>], nodes: &mut Vec<Point>) {
    let m = poly.len();
    if m < 3 {
        return;
    }
    let mut signed_area = 0.0;
    for i in 0..m {
        let j = (i + 1) % m;
        signed_area += poly[i].x * poly[j].y - poly[j].x * poly[i].y;
    }
    let ccw_sign = if signed_area >= 0.0 { 1.0 } else { -1.0 };
    let step = (m / 64).max(1);

    for idx in 0..m {
        let prev = poly[(idx + m - 1) % m];
        let v = poly[idx];
        let next = poly[(idx + 1) % m];

        let e1x = v.x - prev.x;
        let e1y = v.y - prev.y;
        let l1 = e1x.hypot(e1y);
        let e2x = next.x - v.x;
        let e2y = next.y - v.y;
        let l2 = e2x.hypot(e2y);
        if l1 < 1e-9 || l2 < 1e-9 {
            continue;
        }

        // Outward unit normals of adjacent edges
        let n1x = ccw_sign * (e1y / l1);
        let n1y = ccw_sign * (-e1x / l1);
        let n2x = ccw_sign * (e2y / l2);
        let n2y = ccw_sign * (-e2x / l2);

        // Preserve sharp corners even on dense polygons; only subsample smooth arcs
        let dot = n1x * n2x + n1y * n2y;
        let is_corner = dot < 0.96;
        if !is_corner && idx % step != 0 {
            continue;
        }

        let bx = n1x + n2x;
        let by = n1y + n2y;
        let blen = bx.hypot(by);
        let dirs: [(f64, f64); 4] = if blen > 1e-6 {
            [
                (bx / blen, by / blen),
                (-bx / blen, -by / blen),
                (n1x, n1y),
                (n2x, n2y),
            ]
        } else {
            [(n1x, n1y), (n2x, n2y), (-n1x, -n1y), (-n2x, -n2y)]
        };

        for (dx, dy) in dirs {
            let candidate = Point::new(v.x + dx * nudge, v.y + dy * nudge);
            if !obstacles.iter().any(|p| point_in_polygon(candidate, p)) {
                nodes.push(candidate);
                break;
            }
        }
    }
}

fn route_around_obstacles(start: Point, goal: Point, obstacles: &[Vec<Point>]) -> Vec<Point> {
    let blocks_any = |a: Point, b: Point| -> bool {
        obstacles
            .iter()
            .any(|poly| segment_intersects_polygon(a, b, poly))
    };

    if !blocks_any(start, goal) {
        return vec![start, goal];
    }

    // Build visibility graph nodes: start (0), goal (1), outward-nudged polygon corners,
    // and outward-nudged convex hull vertices of each obstacle (guarantees outer connectivity
    // even for thin concave/zigzag shapes).
    let mut nodes: Vec<Point> = Vec::new();
    nodes.push(start);
    nodes.push(goal);

    for poly in obstacles {
        if poly.len() < 3 {
            continue;
        }
        push_nudged_vertices(poly, 0.08, obstacles, &mut nodes);
        let hull = convex_hull(poly);
        push_nudged_vertices(&hull, 0.15, obstacles, &mut nodes);
    }

    let n = nodes.len();
    let mut dist = vec![f64::INFINITY; n];
    let mut prev = vec![usize::MAX; n];
    let mut visited = vec![false; n];
    dist[0] = 0.0;

    for _ in 0..n {
        let mut u = usize::MAX;
        let mut best_d = f64::INFINITY;
        for i in 0..n {
            if !visited[i] && dist[i] < best_d {
                best_d = dist[i];
                u = i;
            }
        }
        if u == usize::MAX || u == 1 {
            break;
        }
        visited[u] = true;

        for v in 0..n {
            if visited[v] || u == v {
                continue;
            }
            if !blocks_any(nodes[u], nodes[v]) {
                let d = dist[u] + (nodes[v].x - nodes[u].x).hypot(nodes[v].y - nodes[u].y);
                if d < dist[v] {
                    dist[v] = d;
                    prev[v] = u;
                }
            }
        }
    }

    if !dist[1].is_finite() {
        return vec![start, goal];
    }

    let mut path = Vec::new();
    let mut cur = 1;
    while cur != usize::MAX {
        path.push(nodes[cur]);
        if cur == 0 {
            break;
        }
        cur = prev[cur];
    }
    path.reverse();
    if path.len() >= 2 {
        path
    } else {
        vec![start, goal]
    }
}
