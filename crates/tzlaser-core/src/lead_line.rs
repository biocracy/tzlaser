use kurbo::Point;
use crate::{Document, Entity};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeadType {
    Line,
    Arc,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LeadConfig {
    pub lead_type: LeadType,
    pub length_mm: f64,
    pub angle_deg: f64,
    pub arc_radius_mm: f64,
    pub arc_length_mm: f64,
    pub overcut_mm: f64,
}

impl Default for LeadConfig {
    fn default() -> Self {
        Self {
            lead_type: LeadType::Line,
            length_mm: 3.5,
            angle_deg: 90.0,
            arc_radius_mm: 3.0,
            arc_length_mm: 3.5,
            overcut_mm: 0.5,
        }
    }
}

/// Signed area of a polygon (`> 0` = CCW, `< 0` = CW).
fn polygon_signed_area(pts: &[Point]) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        area += a.x * b.y - b.x * a.y;
    }
    0.5 * area
}

/// Standard ray-casting point-in-polygon test.
pub fn point_in_polygon(p: Point, poly: &[Point]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let pi = poly[i];
        let pj = poly[j];
        let intersect = ((pi.y > p.y) != (pj.y > p.y))
            && (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y + 1e-18) + pi.x);
        if intersect {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Split a `Polyline { closed: false }` that has an attached lead-in into `(lead_with_entry, closed_contour)`.
/// Returns `None` if the polyline is genuinely open or has no attached lead-in.
pub fn split_lead_contour(points: &[Point]) -> Option<(&[Point], &[Point])> {
    if points.len() < 5 {
        return None;
    }
    for k in 1..(points.len() - 2) {
        for m in ((k + 3)..points.len()).rev() {
            if points[k].distance(points[m]) < 1e-5 {
                let contour = &points[k..m];
                if contour.len() >= 3 {
                    return Some((&points[..=k], contour));
                }
            }
        }
    }
    None
}

/// Convert a closed entity (or one that already has an attached lead-in) into its base closed polygon.
/// Returns `(contour_points, existing_lead_pierce_point)` where `existing_lead_pierce_point` is `Some(p0)`
/// if a lead-in was already attached and stripped.
fn extract_closed_contour(entity: &Entity) -> Option<(Vec<Point>, Option<Point>)> {
    match entity {
        Entity::Circle { center, radius } => {
            if *radius <= 1e-6 {
                return None;
            }
            let steps = 48;
            let mut pts = Vec::with_capacity(steps);
            for i in 0..steps {
                let angle = (i as f64) * std::f64::consts::TAU / (steps as f64);
                pts.push(Point::new(
                    center.x + *radius * angle.cos(),
                    center.y + *radius * angle.sin(),
                ));
            }
            Some((pts, None))
        }
        Entity::Polyline { points, closed } => {
            if *closed && points.len() >= 3 {
                let mut pts = points.clone();
                if pts.len() >= 4 && pts.first().unwrap().distance(*pts.last().unwrap()) < 1e-5 {
                    pts.pop();
                }
                if pts.len() >= 3 {
                    return Some((pts, None));
                }
            } else if !*closed && points.len() >= 5 {
                if let Some((lead, contour)) = split_lead_contour(points) {
                    return Some((contour.to_vec(), Some(lead[0])));
                }
                // Also handle a polyline that is geometrically closed (first == last) without `closed = true`
                let last = *points.last().unwrap();
                if points.first().unwrap().distance(last) < 1e-5 {
                    let contour = points[..points.len() - 1].to_vec();
                    if contour.len() >= 3 {
                        return Some((contour, None));
                    }
                }
            }
            None
        }
        _ => None,
    }
}

/// Test whether polygon `inner` is completely enclosed by polygon `outer`.
pub(crate) fn polygon_enclosed_by(inner: &[Point], outer: &[Point]) -> bool {
    if inner.len() < 3 || outer.len() < 3 {
        return false;
    }
    let mut in_min_x = f64::INFINITY;
    let mut in_max_x = f64::NEG_INFINITY;
    let mut in_min_y = f64::INFINITY;
    let mut in_max_y = f64::NEG_INFINITY;
    for p in inner {
        in_min_x = in_min_x.min(p.x);
        in_max_x = in_max_x.max(p.x);
        in_min_y = in_min_y.min(p.y);
        in_max_y = in_max_y.max(p.y);
    }

    let mut out_min_x = f64::INFINITY;
    let mut out_max_x = f64::NEG_INFINITY;
    let mut out_min_y = f64::INFINITY;
    let mut out_max_y = f64::NEG_INFINITY;
    for p in outer {
        out_min_x = out_min_x.min(p.x);
        out_max_x = out_max_x.max(p.x);
        out_min_y = out_min_y.min(p.y);
        out_max_y = out_max_y.max(p.y);
    }

    if in_min_x < out_min_x - 1e-5
        || in_max_x > out_max_x + 1e-5
        || in_min_y < out_min_y - 1e-5
        || in_max_y > out_max_y + 1e-5
    {
        return false;
    }

    // Check that all vertices (or edge midpoints if touching) are inside `outer`
    let mut inside_count = 0usize;
    for i in 0..inner.len() {
        let a = inner[i];
        let b = inner[(i + 1) % inner.len()];
        let mid = Point::new(0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
        if point_in_polygon(mid, outer) {
            inside_count += 1;
        } else if !point_in_polygon(a, outer) {
            return false;
        }
    }
    inside_count > 0
}

/// Determine whether a closed profile at `(l_idx, e_idx)` should have its lead-in placed **inside**.
/// Automatically detects if another closed profile in any visible layer completely encloses it
/// (odd nesting depth => inside; outermost / even depth => outside).
fn should_place_inside(
    doc: &Document,
    target_l: usize,
    target_e: usize,
    target_poly: &[Point],
) -> bool {
    let mut enclosing_count = 0usize;
    for (l_idx, layer) in doc.layers.iter().enumerate() {
        if !layer.visible {
            continue;
        }
        for (e_idx, ent) in layer.entities.iter().enumerate() {
            if l_idx == target_l && e_idx == target_e {
                continue;
            }
            if let Some((other_poly, _)) = extract_closed_contour(ent) {
                if polygon_enclosed_by(target_poly, &other_poly) {
                    enclosing_count += 1;
                }
            }
        }
    }
    enclosing_count % 2 == 1
}

/// Generate lead-in points (excluding the entry point `entry` itself) for a given entry point,
/// unit cut tangent `(tx, ty)`, and unit normal `(nx, ny)` pointing toward the desired side
/// (inward or outward).
fn generate_lead_points(
    entry: Point,
    tx: f64,
    ty: f64,
    nx: f64,
    ny: f64,
    config: &LeadConfig,
) -> Vec<Point> {
    match config.lead_type {
        LeadType::Line => {
            let len = config.length_mm.max(0.1);
            let angle_rad = config.angle_deg.clamp(1.0, 179.0).to_radians();
            let cos_a = angle_rad.cos();
            let sin_a = angle_rad.sin();
            // At 90 deg: cos=0, sin=1 => strictly along normal (nx, ny)
            let ldir_x = nx * sin_a - tx * cos_a;
            let ldir_y = ny * sin_a - ty * cos_a;
            vec![Point::new(entry.x + len * ldir_x, entry.y + len * ldir_y)]
        }
        LeadType::Arc => {
            let r = config.arc_radius_mm.max(0.1);
            let arc_len = config.arc_length_mm.max(0.1);
            let theta = (arc_len / r).clamp(0.05, std::f64::consts::PI);
            let steps = ((arc_len / 0.25).ceil() as usize).clamp(8, 64);

            // Center offset along desired normal `n` so the arc arrives at `entry` tangent to `-t`
            // (180 deg opposite to the outgoing cut direction `t`)
            let cx = entry.x + r * nx;
            let cy = entry.y + r * ny;

            let mut pts = Vec::with_capacity(steps);
            for i in 0..steps {
                let u = (i as f64) / (steps as f64);
                let phi = (1.0 - u) * theta;
                let px = cx - r * phi.cos() * nx + r * phi.sin() * tx;
                let py = cy - r * phi.cos() * ny + r * phi.sin() * ty;
                pts.push(Point::new(px, py));
            }
            pts
        }
    }
}

/// Build a polyline with lead-in attached right at `contour[0]` (preserving the user's chosen start point),
/// using edge normal on flat edges or bisector normal at corners so it never overlaps adjacent edges.
pub fn build_poly_with_lead_at_entry(
    contour: &[Point],
    place_inside: bool,
    config: &LeadConfig,
) -> Vec<Point> {
    let n = contour.len();
    let entry = contour[0];
    let next_pt = contour[1 % n];
    let prev_pt = contour[n - 1];

    let out_dx = next_pt.x - entry.x;
    let out_dy = next_pt.y - entry.y;
    let out_len = out_dx.hypot(out_dy).max(1e-9);
    let tx = out_dx / out_len;
    let ty = out_dy / out_len;

    let in_dx = entry.x - prev_pt.x;
    let in_dy = entry.y - prev_pt.y;
    let in_len = in_dx.hypot(in_dy).max(1e-9);
    let in_tx = in_dx / in_len;
    let in_ty = in_dy / in_len;

    let area = polygon_signed_area(contour);
    let edge_normal = |ux: f64, uy: f64| -> (f64, f64) {
        let (mut nx, mut ny) = if area >= 0.0 { (uy, -ux) } else { (-uy, ux) };
        if place_inside {
            nx = -nx;
            ny = -ny;
        }
        (nx, ny)
    };

    let (nx_out, ny_out) = edge_normal(tx, ty);
    let (mut nx, mut ny) = if (in_tx * tx + in_ty * ty) < 0.999 {
        // Corner vertex: average incoming and outgoing normals along the bisector
        let (nx_in, ny_in) = edge_normal(in_tx, in_ty);
        let bx = nx_in + nx_out;
        let by = ny_in + ny_out;
        let blen = bx.hypot(by);
        if blen > 1e-6 {
            (bx / blen, by / blen)
        } else {
            (nx_out, ny_out)
        }
    } else {
        (nx_out, ny_out)
    };

    // Verify with point-in-polygon test so winding anomalies never cause overlap
    let probe_dist = (out_len.min(in_len) * 0.01).clamp(1e-4, 0.05);
    let probe = Point::new(entry.x + probe_dist * nx, entry.y + probe_dist * ny);
    if point_in_polygon(probe, contour) != place_inside {
        nx = -nx;
        ny = -ny;
    }

    let lead_pts = generate_lead_points(entry, tx, ty, nx, ny, config);

    let mut out = Vec::with_capacity(lead_pts.len() + n + 4);
    out.extend(lead_pts);
    out.extend_from_slice(contour);
    out.push(entry);
    append_overcut_points(contour, config.overcut_mm, &mut out);
    out
}

/// Append `overcut_mm` of travel past `contour[0]` along the closed `contour`
/// so the kerf at the end of the cut overlaps the lead-in entry and leaves no tab.
fn append_overcut_points(contour: &[Point], overcut_mm: f64, out: &mut Vec<Point>) {
    if overcut_mm <= 1e-4 || contour.len() < 2 {
        return;
    }
    let mut rem = overcut_mm;
    let n = contour.len();
    for i in 0..n {
        let a = contour[i];
        let b = contour[(i + 1) % n];
        let seg_len = a.distance(b);
        if seg_len <= 1e-9 {
            continue;
        }
        if seg_len >= rem {
            let t = rem / seg_len;
            out.push(Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
            return;
        }
        out.push(b);
        rem -= seg_len;
    }
}

/// Rebuild a lead-in on `new_contour` (anchored at `new_contour[0]`) inferring straight vs. arc
/// dimensions from `old_lead`. Used when re-anchoring the start point of a profile that already has a lead.
pub fn rebuild_lead_from_existing(
    old_lead: &[Point],
    new_contour: &[Point],
    place_inside: bool,
) -> Vec<Point> {
    let mut cfg = LeadConfig::default();
    if old_lead.len() == 2 {
        cfg.lead_type = LeadType::Line;
        cfg.length_mm = old_lead[0].distance(old_lead[1]).max(0.5);
        cfg.angle_deg = 90.0;
    } else if old_lead.len() > 2 {
        cfg.lead_type = LeadType::Arc;
        let arc_len: f64 = old_lead.windows(2).map(|w| w[0].distance(w[1])).sum();
        cfg.arc_length_mm = arc_len.max(0.5);
        // Estimate radius from total turning angle across lead segments
        let mut total_turn = 0.0f64;
        for w in old_lead.windows(3) {
            let a = (w[1].x - w[0].x, w[1].y - w[0].y);
            let b = (w[2].x - w[1].x, w[2].y - w[1].y);
            let la = a.0.hypot(a.1);
            let lb = b.0.hypot(b.1);
            if la > 1e-9 && lb > 1e-9 {
                let cos_t = ((a.0 * b.0 + a.1 * b.1) / (la * lb)).clamp(-1.0, 1.0);
                total_turn += cos_t.acos();
            }
        }
        if total_turn > 0.05 {
            cfg.arc_radius_mm = (arc_len / total_turn).clamp(0.5, 50.0);
        }
    }
    build_poly_with_lead_at_entry(new_contour, place_inside, &cfg)
}

/// Build a polyline with lead-in attached at the midpoint of the longest edge of `contour`,
/// guaranteeing the lead-in sits strictly on the `place_inside` side and never overlaps an edge.
fn build_poly_with_lead(contour: &[Point], place_inside: bool, config: &LeadConfig) -> Vec<Point> {
    let n = contour.len();
    // Pick the longest edge so the entry point is away from corners
    let mut best_edge = 0usize;
    let mut best_len = -1.0f64;
    for i in 0..n {
        let a = contour[i];
        let b = contour[(i + 1) % n];
        let len = a.distance(b);
        if len > best_len {
            best_len = len;
            best_edge = i;
        }
    }

    let a = contour[best_edge];
    let b = contour[(best_edge + 1) % n];
    let entry = Point::new(0.5 * (a.x + b.x), 0.5 * (a.y + b.y));

    let mut reordered = Vec::with_capacity(n + 1);
    reordered.push(entry);
    for k in 1..=n {
        reordered.push(contour[(best_edge + k) % n]);
    }

    build_poly_with_lead_at_entry(&reordered, place_inside, config)
}

fn collect_target_entities(doc: &Document, selection: &[(usize, usize)]) -> Vec<(usize, usize)> {
    if selection.is_empty() {
        let mut list = Vec::new();
        for (l_idx, layer) in doc.layers.iter().enumerate() {
            if !layer.visible {
                continue;
            }
            for (e_idx, ent) in layer.entities.iter().enumerate() {
                if extract_closed_contour(ent).is_some() {
                    list.push((l_idx, e_idx));
                }
            }
        }
        list
    } else {
        selection.to_vec()
    }
}

/// Apply lead-in lines to the document (all closed entities, or specifically selected ones).
/// Automatically detects enclosing profiles to place lead-ins inside holes and outside outer parts.
/// Preserves existing entry point if a lead-in was already attached.
pub fn apply_lead_lines(
    doc: &mut Document,
    selection: &[(usize, usize)],
    config: &LeadConfig,
) -> usize {
    let target_entities = collect_target_entities(doc, selection);

    // Precompute containment decisions before mutating entities
    let mut plans: Vec<(usize, usize, Vec<Point>, bool, bool)> = Vec::new();
    for (l_idx, e_idx) in target_entities {
        if l_idx >= doc.layers.len() || e_idx >= doc.layers[l_idx].entities.len() {
            continue;
        }
        if let Some((contour, existing_pierce)) =
            extract_closed_contour(&doc.layers[l_idx].entities[e_idx])
        {
            let place_inside = should_place_inside(doc, l_idx, e_idx, &contour);
            plans.push((
                l_idx,
                e_idx,
                contour,
                place_inside,
                existing_pierce.is_some(),
            ));
        }
    }

    let mut count = 0;
    for (l_idx, e_idx, contour, place_inside, has_existing_lead) in plans {
        let new_points = if has_existing_lead {
            build_poly_with_lead_at_entry(&contour, place_inside, config)
        } else {
            build_poly_with_lead(&contour, place_inside, config)
        };
        doc.layers[l_idx].entities[e_idx] = Entity::Polyline {
            points: new_points,
            closed: false,
        };
        count += 1;
    }

    count
}

/// Manually flip lead-in lines between inside and outside for the selected (or all) closed profiles.
/// Preserves the current starting point on the profile.
pub fn flip_lead_lines(
    doc: &mut Document,
    selection: &[(usize, usize)],
    config: &LeadConfig,
) -> usize {
    let target_entities = collect_target_entities(doc, selection);

    let mut plans: Vec<(usize, usize, Vec<Point>, bool, bool)> = Vec::new();
    for (l_idx, e_idx) in target_entities {
        if l_idx >= doc.layers.len() || e_idx >= doc.layers[l_idx].entities.len() {
            continue;
        }
        if let Some((contour, existing_pierce)) =
            extract_closed_contour(&doc.layers[l_idx].entities[e_idx])
        {
            let currently_inside = match existing_pierce {
                Some(p0) => point_in_polygon(p0, &contour),
                None => should_place_inside(doc, l_idx, e_idx, &contour),
            };
            plans.push((
                l_idx,
                e_idx,
                contour,
                !currently_inside,
                existing_pierce.is_some(),
            ));
        }
    }

    let mut count = 0;
    for (l_idx, e_idx, contour, place_inside, has_existing_lead) in plans {
        let new_points = if has_existing_lead {
            build_poly_with_lead_at_entry(&contour, place_inside, config)
        } else {
            build_poly_with_lead(&contour, place_inside, config)
        };
        doc.layers[l_idx].entities[e_idx] = Entity::Polyline {
            points: new_points,
            closed: false,
        };
        count += 1;
    }

    count
}

/// Remove attached lead-in lines from selected (or all) closed profiles, restoring `closed = true`.
pub fn remove_lead_lines(doc: &mut Document, selection: &[(usize, usize)]) -> usize {
    let target_entities = collect_target_entities(doc, selection);
    let mut count = 0;
    for (l_idx, e_idx) in target_entities {
        if l_idx >= doc.layers.len() || e_idx >= doc.layers[l_idx].entities.len() {
            continue;
        }
        if let Entity::Polyline { points, closed } = &mut doc.layers[l_idx].entities[e_idx] {
            if !*closed {
                if let Some((_, contour)) = split_lead_contour(points) {
                    *points = contour.to_vec();
                    *closed = true;
                    count += 1;
                }
            }
        }
    }
    count
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Color, Layer};

    #[test]
    fn test_apply_lead_lines_to_circle_and_poly() {
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

        let mut inner_layer = Layer::new("Layer 0", Color::YELLOW);
        inner_layer.entities.push(Entity::Circle {
            center: Point::new(50.0, 50.0),
            radius: 10.0,
        });

        doc.layers = vec![outer_layer, inner_layer];

        let config = LeadConfig {
            lead_type: LeadType::Line,
            length_mm: 5.0,
            angle_deg: 90.0,
            arc_radius_mm: 3.0,
            arc_length_mm: 3.5,
            overcut_mm: 0.5,
        };

        let applied = apply_lead_lines(&mut doc, &[], &config);
        assert_eq!(applied, 2);

        // Outer square: pierce point must be strictly OUTSIDE [0..100, 0..100]
        if let Entity::Polyline { points, closed } = &doc.layers[0].entities[0] {
            assert!(!*closed);
            let p0 = points[0];
            assert!(
                p0.x < -1.0 || p0.x > 101.0 || p0.y < -1.0 || p0.y > 101.0,
                "Outer lead-in should be outside rectangle, got {:?}",
                p0
            );
        } else {
            panic!("Expected polyline");
        }

        // Inner circle enclosed by outer square: pierce point must be INSIDE circle (dist < 10)
        if let Entity::Polyline { points, closed } = &doc.layers[1].entities[0] {
            assert!(!*closed);
            let p0 = points[0];
            let dist = p0.distance(Point::new(50.0, 50.0));
            assert!(
                (dist - 5.0).abs() < 0.2,
                "Inner circle lead-in should be 5mm inside radius 10 (dist=5), got dist={}",
                dist
            );
        } else {
            panic!("Expected circle converted to toolpath with lead");
        }
    }

    #[test]
    fn test_cw_rectangle_never_overlaps_and_flip_works() {
        let mut doc = Document::default();
        let mut layer = Layer::new("Cut", Color::GREEN);
        // Clockwise axis-aligned rectangle
        let rect = vec![
            Point::new(0.0, 0.0),
            Point::new(0.0, 80.0),
            Point::new(120.0, 80.0),
            Point::new(120.0, 0.0),
        ];
        layer.entities.push(Entity::Polyline {
            points: rect.clone(),
            closed: true,
        });
        doc.layers = vec![layer];

        let config = LeadConfig {
            lead_type: LeadType::Line,
            length_mm: 4.0,
            angle_deg: 45.0,
            ..Default::default()
        };

        assert_eq!(apply_lead_lines(&mut doc, &[], &config), 1);
        if let Entity::Polyline { points, .. } = &doc.layers[0].entities[0] {
            assert!(!point_in_polygon(points[0], &rect), "Should start outside CW rect");
        }

        // Flip manually -> should now pierce INSIDE
        assert_eq!(flip_lead_lines(&mut doc, &[], &config), 1);
        if let Entity::Polyline { points, .. } = &doc.layers[0].entities[0] {
            assert!(point_in_polygon(points[0], &rect), "Flipped lead should start inside CW rect");
        }
    }

    #[test]
    fn test_arc_lead_in_geometry() {
        let mut doc = Document::default();
        let mut layer = Layer::new("Cut", Color::GREEN);
        let rect = vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0),
            Point::new(100.0, 100.0),
            Point::new(0.0, 100.0),
        ];
        layer.entities.push(Entity::Polyline {
            points: rect.clone(),
            closed: true,
        });
        doc.layers = vec![layer];

        let config = LeadConfig {
            lead_type: LeadType::Arc,
            arc_radius_mm: 4.0,
            arc_length_mm: 6.2831853, // 90 deg arc
            ..Default::default()
        };

        assert_eq!(apply_lead_lines(&mut doc, &[], &config), 1);
        if let Entity::Polyline { points, .. } = &doc.layers[0].entities[0] {
            assert!(points.len() > 12, "Arc lead should generate smooth arc segments");
            assert!(!point_in_polygon(points[0], &rect), "Arc pierce should be outside");
            let (lead, contour) = split_lead_contour(points).expect("lead+contour");
            let n = lead.len();
            let lead_end_dir = (lead[n - 1].x - lead[n - 2].x, lead[n - 1].y - lead[n - 2].y);
            let cut_dir = (contour[1].x - contour[0].x, contour[1].y - contour[0].y);
            let l_len = lead_end_dir.0.hypot(lead_end_dir.1);
            let c_len = cut_dir.0.hypot(cut_dir.1);
            let dot = (lead_end_dir.0 * cut_dir.0 + lead_end_dir.1 * cut_dir.1) / (l_len * c_len);
            assert!(dot < -0.95, "Arc lead should arrive 180 deg opposite to cut direction, got dot={}", dot);
        }
    }

    #[test]
    fn test_reanchor_after_lead_flip_preserves_start_and_cut_order() {
        let mut doc = Document::default();
        let mut layer = Layer::new("Cut", Color::GREEN);
        layer.entities.push(Entity::Polyline {
            points: vec![
                Point::new(0.0, 0.0),
                Point::new(100.0, 0.0),
                Point::new(100.0, 100.0),
                Point::new(0.0, 100.0),
            ],
            closed: true,
        });
        layer.entities.push(Entity::Polyline {
            points: vec![
                Point::new(200.0, 0.0),
                Point::new(300.0, 0.0),
                Point::new(300.0, 100.0),
                Point::new(200.0, 100.0),
            ],
            closed: true,
        });
        doc.layers = vec![layer];

        let order_before: Vec<(usize, usize)> = doc
            .compute_cut_sequence()
            .iter()
            .map(|c| (c.layer_idx, c.entity_idx))
            .collect();

        let config = LeadConfig::default();
        assert_eq!(apply_lead_lines(&mut doc, &[], &config), 2);

        // Cut order must be unchanged after adding leads
        let order_after_add: Vec<(usize, usize)> = doc
            .compute_cut_sequence()
            .iter()
            .map(|c| (c.layer_idx, c.entity_idx))
            .collect();
        assert_eq!(order_before, order_after_add);

        // Re-anchoring Path #2 after leads were added must succeed and move entry to (250, 100)
        assert!(doc.set_path_start_point(2, Point::new(250.0, 100.0)));
        let entry_before_flip = match &doc.layers[0].entities[1] {
            Entity::Polyline { points, .. } => {
                let (_, contour) = split_lead_contour(points).expect("lead still present");
                assert!(contour[0].distance(Point::new(250.0, 100.0)) < 1e-4);
                contour[0]
            }
            _ => panic!("expected polyline"),
        };

        // Flip only Path #2 -> entry point must NOT jump, and cut order must remain unchanged
        assert_eq!(flip_lead_lines(&mut doc, &[(0, 1)], &config), 1);
        let entry_after_flip = match &doc.layers[0].entities[1] {
            Entity::Polyline { points, .. } => {
                let (_, contour) = split_lead_contour(points).expect("lead still present");
                contour[0]
            }
            _ => panic!("expected polyline"),
        };
        assert!(entry_before_flip.distance(entry_after_flip) < 1e-6);

        let order_after_flip: Vec<(usize, usize)> = doc
            .compute_cut_sequence()
            .iter()
            .map(|c| (c.layer_idx, c.entity_idx))
            .collect();
        assert_eq!(order_before, order_after_flip);
    }

    #[test]
    fn test_lead_overcut_preserves_split_and_extends_past_entry() {
        let mut doc = Document::default();
        let mut layer = Layer::new("Cut", Color::GREEN);
        layer.entities.push(Entity::Polyline {
            points: vec![
                Point::new(0.0, 0.0),
                Point::new(40.0, 0.0),
                Point::new(40.0, 40.0),
                Point::new(0.0, 40.0),
            ],
            closed: true,
        });
        doc.layers = vec![layer];

        let cfg = LeadConfig {
            lead_type: LeadType::Arc,
            arc_radius_mm: 3.0,
            arc_length_mm: 3.5,
            overcut_mm: 0.6,
            ..Default::default()
        };
        assert_eq!(apply_lead_lines(&mut doc, &[], &cfg), 1);
        if let Entity::Polyline { points, .. } = &doc.layers[0].entities[0] {
            let (lead, contour) = split_lead_contour(points).expect("lead+contour splits cleanly");
            let entry = *lead.last().unwrap();
            let last = *points.last().unwrap();
            assert!((entry.distance(last) - 0.6).abs() < 1e-4, "expected 0.6mm overcut past entry");
            assert_eq!(contour.len(), 5);
        } else {
            panic!("expected polyline");
        }
    }
}
