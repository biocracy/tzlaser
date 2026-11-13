use std::collections::HashMap;
use std::f64::consts::PI;
use std::path::Path;

use dxf::entities::EntityType;
use dxf::{Drawing, Point as DxfPoint, Vector as DxfVector};
use kurbo::{Affine, Point};
use thiserror::Error;

use crate::document::{Color, Document, Entity, Layer};

#[derive(Error, Debug)]
pub enum DxfError {
    #[error("DXF parse error: {0}")]
    Dxf(#[from] dxf::DxfError),
    #[error("File IO error: {0}")]
    Io(#[from] std::io::Error),
}

/// AutoCAD Arbitrary Axis Algorithm to convert Object Coordinate System (OCS) to World Coordinate System (WCS).
///
/// In DXF, 2D entities with extrusion vectors (CIRCLE, ARC, LWPOLYLINE, TEXT) are defined in OCS.
/// When Nz < 0 (e.g. 0, 0, -1), AutoCAD reflects the local X-axis. Without this transformation,
/// coordinates are inverted.
///
/// Note: 3D entities like LINE and POINT are ALREADY in WCS per the AutoCAD DXF specification,
/// and must NOT be converted through OCS.
pub fn ocs_to_wcs(x: f64, y: f64, z: f64, normal: &DxfVector) -> Point {
    let nx = normal.x;
    let ny = normal.y;
    let nz = normal.z;

    let len = (nx * nx + ny * ny + nz * nz).sqrt();
    if len < 1e-9 {
        return Point::new(x, y);
    }
    let (nx, ny, nz) = (nx / len, ny / len, nz / len);

    // Arbitrary Axis Algorithm:
    // If |Nx| < 1/64 and |Ny| < 1/64:
    //   Ax = Wy x N = (0, 1, 0) x (nx, ny, nz) = (nz, 0, -nx)
    // Else:
    //   Ax = Wz x N = (0, 0, 1) x (nx, ny, nz) = (-ny, nx, 0)
    let (ax_x, ax_y, ax_z) = if nx.abs() < (1.0 / 64.0) && ny.abs() < (1.0 / 64.0) {
        let ax_x = nz;
        let _ax_y = 0.0;
        let ax_z = -nx;
        let ax_len = (ax_x * ax_x + ax_z * ax_z).sqrt();
        if ax_len > 1e-9 {
            (ax_x / ax_len, 0.0, ax_z / ax_len)
        } else {
            (1.0, 0.0, 0.0)
        }
    } else {
        let ax_x = -ny;
        let ax_y = nx;
        let _ax_z = 0.0;
        let ax_len = (ax_x * ax_x + ax_y * ax_y).sqrt();
        if ax_len > 1e-9 {
            (ax_x / ax_len, ax_y / ax_len, 0.0)
        } else {
            (1.0, 0.0, 0.0)
        }
    };

    // Ay = N x Ax = (ny * ax_z - nz * ax_y, nz * ax_x - nx * ax_z, nx * ax_y - ny * ax_x)
    let ay_x = ny * ax_z - nz * ax_y;
    let ay_y = nz * ax_x - nx * ax_z;

    // Project onto 2D plane (wx, wy)
    let wx = x * ax_x + y * ay_x + z * nx;
    let wy = x * ax_y + y * ay_y + z * ny;

    Point::new(wx, wy)
}

/// Interpolates an arc segment between two polyline vertices when bulge != 0.
/// Bulge is tan(theta / 4), where theta is the included angle.
fn interpolate_bulge(p1: Point, p2: Point, bulge: f64) -> Vec<Point> {
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let chord_len = (dx * dx + dy * dy).sqrt();

    if chord_len < 1e-9 || bulge.abs() < 1e-6 {
        return vec![p1, p2];
    }

    let theta = 4.0 * bulge.atan();
    let dist_to_center = chord_len * (1.0 - bulge * bulge) / (4.0 * bulge);

    let mid_x = (p1.x + p2.x) / 2.0;
    let mid_y = (p1.y + p2.y) / 2.0;

    // Perpendicular unit vector to chord (pointing to the left of p1->p2)
    let norm_x = -dy / chord_len;
    let norm_y = dx / chord_len;

    let center_x = mid_x + norm_x * dist_to_center;
    let center_y = mid_y + norm_y * dist_to_center;

    let radius = ((p1.x - center_x).powi(2) + (p1.y - center_y).powi(2)).sqrt();
    let start_angle = (p1.y - center_y).atan2(p1.x - center_x);

    let steps = (theta.abs() / (PI / 16.0)).ceil().clamp(4.0, 32.0) as usize;
    let mut arc_pts = Vec::with_capacity(steps + 1);
    arc_pts.push(p1);

    for s in 1..=steps {
        let angle = start_angle + theta * (s as f64 / steps as f64);
        arc_pts.push(Point::new(
            center_x + radius * angle.cos(),
            center_y + radius * angle.sin(),
        ));
    }

    arc_pts
}

/// Evaluate a B-spline / NURBS into a sampled polyline via De Boor's algorithm.
fn evaluate_bspline(
    degree: usize,
    knots: &[f64],
    control_points: &[DxfPoint],
    weights: &[f64],
    num_samples: usize,
) -> Vec<Point> {
    let n = control_points.len();
    if n == 0 {
        return Vec::new();
    }
    if n <= degree || knots.len() < n + degree + 1 {
        // Fallback: connect control points directly
        return control_points.iter().map(|p| Point::new(p.x, p.y)).collect();
    }

    let p = degree;
    let t_min = knots[p];
    let t_max = knots[knots.len() - 1 - p];
    if t_max <= t_min {
        return control_points.iter().map(|p| Point::new(p.x, p.y)).collect();
    }

    let has_weights = !weights.is_empty() && weights.len() == n;
    let mut c_homo = Vec::with_capacity(n);
    for (i, pt) in control_points.iter().enumerate() {
        let w = if has_weights { weights[i] } else { 1.0 };
        c_homo.push([pt.x * w, pt.y * w, w]);
    }

    let samples = num_samples.max(24);
    let mut result = Vec::with_capacity(samples);
    let dt = (t_max - t_min) / ((samples - 1) as f64);

    for step in 0..samples {
        let mut t = t_min + (step as f64) * dt;
        if step == samples - 1 {
            t = (t_max - 1e-9).max(t_min);
        }

        let mut k = p;
        while k < knots.len() - 1 - p && knots[k + 1] <= t {
            k += 1;
        }

        let mut d = Vec::with_capacity(p + 1);
        for j in (k - p)..=k {
            d.push(c_homo[j]);
        }

        for r in 1..=p {
            for j in (r..=p).rev() {
                let i = k - p + j;
                let denom = knots[i + p + 1 - r] - knots[i];
                let alpha = if denom.abs() < 1e-9 { 0.0 } else { (t - knots[i]) / denom };
                for coord in 0..3 {
                    d[j][coord] = (1.0 - alpha) * d[j - 1][coord] + alpha * d[j][coord];
                }
            }
        }

        let w = d[p][2];
        let x = if w.abs() > 1e-9 { d[p][0] / w } else { d[p][0] };
        let y = if w.abs() > 1e-9 { d[p][1] / w } else { d[p][1] };
        result.push(Point::new(x, y));
    }

    result
}

/// Discretize an ellipse arc into polyline points.
fn evaluate_ellipse(
    center: Point,
    major_axis: &DxfVector,
    minor_axis_ratio: f64,
    start_param: f64,
    end_param: f64,
    num_samples: usize,
) -> Vec<Point> {
    let ax = major_axis.x;
    let ay = major_axis.y;
    let maj_len = (ax * ax + ay * ay).sqrt();
    if maj_len < 1e-9 {
        return vec![center];
    }
    let u_x = ax / maj_len;
    let u_y = ay / maj_len;
    let v_x = -u_y;
    let v_y = u_x;
    let min_len = maj_len * minor_axis_ratio;

    let mut end = end_param;
    if end <= start_param {
        end += 2.0 * PI;
    }

    let samples = num_samples.max(24);
    let dt = (end - start_param) / ((samples - 1) as f64);
    let mut pts = Vec::with_capacity(samples);

    for s in 0..samples {
        let t = start_param + (s as f64) * dt;
        let c = t.cos();
        let s_val = t.sin();
        let x = center.x + (c * maj_len) * u_x + (s_val * min_len) * v_x;
        let y = center.y + (c * maj_len) * u_y + (s_val * min_len) * v_y;
        pts.push(Point::new(x, y));
    }

    pts
}

pub struct DxfImporter;

impl DxfImporter {
    pub fn load_file(path: impl AsRef<Path>) -> Result<Document, DxfError> {
        let drawing = Drawing::load_file(path)?;
        let mut layers_map: HashMap<String, Layer> = HashMap::new();

        // Standard palette for imported layers
        let default_colors = [
            Color::RED,
            Color::BLUE,
            Color::GREEN,
            Color::YELLOW,
            Color::CYAN,
            Color::MAGENTA,
        ];
        let mut color_idx = 0;

        // Build a lookup map of blocks for resolving INSERT references
        let blocks_map: HashMap<String, Vec<dxf::entities::Entity>> = drawing
            .blocks()
            .map(|b| (b.name.clone(), b.entities.clone()))
            .collect();

        // Process top-level entities in drawing
        for entity in drawing.entities() {
            Self::import_entity(
                entity,
                &Affine::IDENTITY,
                &mut layers_map,
                &blocks_map,
                &default_colors,
                &mut color_idx,
            );
        }

        let mut doc = Document {
            layers: Vec::new(),
            ..Default::default()
        };
        if layers_map.is_empty() {
            doc.layers.push(Layer::new("Default", Color::RED));
        } else {
            for (_, layer) in layers_map {
                doc.layers.push(layer);
            }
        }

        // Always chain contiguous lines, arcs, and open polylines within each layer
        // so closed and open polypaths are treated as single continuous toolpaths.
        for layer in &mut doc.layers {
            let raw = std::mem::take(&mut layer.entities);
            layer.entities = crate::document::chain_connected_entities(raw, 0.1);
        }

        // If the imported document only has a single generic layer (e.g. "Visible", "0", "Default"),
        // automatically classify profiles into Outer Profiles, Interior Profiles, and Bend lines.
        if doc.layers.len() <= 1 {
            doc.auto_classify_profiles();
        }

        Ok(doc)
    }

    fn get_or_create_layer<'a>(
        layers_map: &'a mut HashMap<String, Layer>,
        layer_name: &str,
        default_colors: &[Color],
        color_idx: &mut usize,
    ) -> &'a mut Layer {
        let name = if layer_name.is_empty() { "0" } else { layer_name };
        layers_map.entry(name.to_string()).or_insert_with(|| {
            let color = default_colors[*color_idx % default_colors.len()];
            *color_idx += 1;
            Layer::new(name, color)
        })
    }

    fn import_entity(
        entity: &dxf::entities::Entity,
        transform: &Affine,
        layers_map: &mut HashMap<String, Layer>,
        blocks_map: &HashMap<String, Vec<dxf::entities::Entity>>,
        default_colors: &[Color],
        color_idx: &mut usize,
    ) {
        let layer_name = &entity.common.layer;

        match &entity.specific {
            EntityType::Line(line) => {
                // In AutoCAD DXF, LINE coordinates (start & end) are ALWAYS in World Coordinates (WCS).
                // They do NOT undergo OCS transformation.
                let mut p1 = Point::new(line.p1.x, line.p1.y);
                let mut p2 = Point::new(line.p2.x, line.p2.y);
                p1 = *transform * p1;
                p2 = *transform * p2;

                let layer = Self::get_or_create_layer(layers_map, layer_name, default_colors, color_idx);
                layer.entities.push(Entity::Line { start: p1, end: p2 });
            }

            EntityType::LwPolyline(poly) => {
                let norm = &poly.extrusion_direction;
                let vertices = &poly.vertices;
                if vertices.len() < 2 {
                    return;
                }

                let mut path_points = Vec::new();
                for i in 0..vertices.len() {
                    let v1 = &vertices[i];
                    let next_idx = if i + 1 < vertices.len() {
                        i + 1
                    } else if poly.is_closed() {
                        0
                    } else {
                        break;
                    };
                    let v2 = &vertices[next_idx];

                    let p1 = ocs_to_wcs(v1.x, v1.y, 0.0, norm);
                    let p2 = ocs_to_wcs(v2.x, v2.y, 0.0, norm);

                    if v1.bulge.abs() > 1e-5 {
                        let arc = interpolate_bulge(p1, p2, v1.bulge);
                        for pt in arc.into_iter().take(if next_idx == 0 { usize::MAX } else { 16 }) {
                            path_points.push(*transform * pt);
                        }
                    } else {
                        path_points.push(*transform * p1);
                        if next_idx != 0 && i + 1 == vertices.len() - 1 {
                            path_points.push(*transform * p2);
                        }
                    }
                }

                if path_points.len() >= 2 {
                    let layer = Self::get_or_create_layer(layers_map, layer_name, default_colors, color_idx);
                    layer.entities.push(Entity::Polyline {
                        points: path_points,
                        closed: poly.is_closed(),
                    });
                }
            }

            EntityType::Polyline(poly) => {
                let norm = &poly.normal;
                let vertices: Vec<_> = poly.vertices().collect();
                if vertices.len() < 2 {
                    return;
                }

                let is_3d = poly.is_3d_polyline();
                let mut path_points = Vec::new();
                for i in 0..vertices.len() {
                    let v1 = vertices[i];
                    let next_idx = if i + 1 < vertices.len() {
                        i + 1
                    } else if poly.is_closed() {
                        0
                    } else {
                        break;
                    };
                    let v2 = vertices[next_idx];

                    let p1 = if is_3d {
                        Point::new(v1.location.x, v1.location.y)
                    } else {
                        ocs_to_wcs(v1.location.x, v1.location.y, v1.location.z, norm)
                    };
                    let p2 = if is_3d {
                        Point::new(v2.location.x, v2.location.y)
                    } else {
                        ocs_to_wcs(v2.location.x, v2.location.y, v2.location.z, norm)
                    };

                    if v1.bulge.abs() > 1e-5 {
                        let arc = interpolate_bulge(p1, p2, v1.bulge);
                        for pt in arc.into_iter().take(if next_idx == 0 { usize::MAX } else { 16 }) {
                            path_points.push(*transform * pt);
                        }
                    } else {
                        path_points.push(*transform * p1);
                        if next_idx != 0 && i + 1 == vertices.len() - 1 {
                            path_points.push(*transform * p2);
                        }
                    }
                }

                if path_points.len() >= 2 {
                    let layer = Self::get_or_create_layer(layers_map, layer_name, default_colors, color_idx);
                    layer.entities.push(Entity::Polyline {
                        points: path_points,
                        closed: poly.is_closed(),
                    });
                }
            }

            EntityType::Circle(circle) => {
                let norm = &circle.normal;
                let mut center = ocs_to_wcs(circle.center.x, circle.center.y, circle.center.z, norm);
                center = *transform * center;

                let coeffs = transform.as_coeffs();
                let scale = ((coeffs[0].powi(2) + coeffs[1].powi(2)).sqrt()
                    + (coeffs[2].powi(2) + coeffs[3].powi(2)).sqrt()) / 2.0;

                let layer = Self::get_or_create_layer(layers_map, layer_name, default_colors, color_idx);
                layer.entities.push(Entity::Circle {
                    center,
                    radius: circle.radius * scale,
                });
            }

            EntityType::Arc(arc) => {
                let norm = &arc.normal;
                let mut center = ocs_to_wcs(arc.center.x, arc.center.y, arc.center.z, norm);
                center = *transform * center;

                let coeffs = transform.as_coeffs();
                let scale = ((coeffs[0].powi(2) + coeffs[1].powi(2)).sqrt()
                    + (coeffs[2].powi(2) + coeffs[3].powi(2)).sqrt()) / 2.0;
                let rot_deg = coeffs[1].atan2(coeffs[0]).to_degrees();

                // Adjust angles if normal is inverted (mirrored local X axis)
                let (mut start_angle, mut end_angle) = if norm.z < 0.0 {
                    // Mirrored: start and end swap across 180 degrees
                    let s = (180.0 - arc.end_angle).rem_euclid(360.0);
                    let e = (180.0 - arc.start_angle).rem_euclid(360.0);
                    (s, e)
                } else {
                    (arc.start_angle, arc.end_angle)
                };

                start_angle = (start_angle + rot_deg).rem_euclid(360.0);
                end_angle = (end_angle + rot_deg).rem_euclid(360.0);

                let layer = Self::get_or_create_layer(layers_map, layer_name, default_colors, color_idx);
                layer.entities.push(Entity::Arc {
                    center,
                    radius: arc.radius * scale,
                    start_angle,
                    end_angle,
                });
            }

            EntityType::Spline(spline) => {
                let degree = spline.degree_of_curve.max(1) as usize;
                let num_samples = (spline.control_points.len() * 8).clamp(32, 256);
                let evaluated = evaluate_bspline(
                    degree,
                    &spline.knot_values,
                    &spline.control_points,
                    &spline.weight_values,
                    num_samples,
                );

                if evaluated.len() >= 2 {
                    let pts: Vec<Point> = evaluated.into_iter().map(|p| *transform * p).collect();
                    let layer = Self::get_or_create_layer(layers_map, layer_name, default_colors, color_idx);
                    layer.entities.push(Entity::Polyline {
                        points: pts,
                        closed: spline.is_closed(),
                    });
                }
            }

            EntityType::Ellipse(ellipse) => {
                let center = Point::new(ellipse.center.x, ellipse.center.y);
                let pts_raw = evaluate_ellipse(
                    center,
                    &ellipse.major_axis,
                    ellipse.minor_axis_ratio,
                    ellipse.start_parameter,
                    ellipse.end_parameter,
                    48,
                );
                if pts_raw.len() >= 2 {
                    let pts: Vec<Point> = pts_raw.into_iter().map(|p| *transform * p).collect();
                    let is_closed = (ellipse.end_parameter - ellipse.start_parameter - 2.0 * PI).abs() < 1e-4;
                    let layer = Self::get_or_create_layer(layers_map, layer_name, default_colors, color_idx);
                    layer.entities.push(Entity::Polyline {
                        points: pts,
                        closed: is_closed,
                    });
                }
            }

            EntityType::Solid(solid) => {
                // In AutoCAD DXF, SOLID 4-point ordering is 1 -> 2 -> 4 -> 3 (bowtie fix)
                let p1 = *transform * ocs_to_wcs(solid.first_corner.x, solid.first_corner.y, solid.first_corner.z, &solid.extrusion_direction);
                let p2 = *transform * ocs_to_wcs(solid.second_corner.x, solid.second_corner.y, solid.second_corner.z, &solid.extrusion_direction);
                let p3 = *transform * ocs_to_wcs(solid.fourth_corner.x, solid.fourth_corner.y, solid.fourth_corner.z, &solid.extrusion_direction);
                let p4 = *transform * ocs_to_wcs(solid.third_corner.x, solid.third_corner.y, solid.third_corner.z, &solid.extrusion_direction);
                let layer = Self::get_or_create_layer(layers_map, layer_name, default_colors, color_idx);
                layer.entities.push(Entity::Polyline {
                    points: vec![p1, p2, p3, p4],
                    closed: true,
                });
            }

            EntityType::Insert(insert) => {
                // Resolve nested blocks (symbols / sub-parts)
                if let Some(block_entities) = blocks_map.get(&insert.name) {
                    let loc_wcs = ocs_to_wcs(
                        insert.location.x,
                        insert.location.y,
                        insert.location.z,
                        &insert.extrusion_direction,
                    );
                    let rot_rad = insert.rotation.to_radians();

                    let insert_affine = Affine::translate((loc_wcs.x, loc_wcs.y))
                        * Affine::rotate(rot_rad)
                        * Affine::scale_non_uniform(insert.x_scale_factor, insert.y_scale_factor);

                    let combined = *transform * insert_affine;

                    for sub_ent in block_entities {
                        Self::import_entity(
                            sub_ent,
                            &combined,
                            layers_map,
                            blocks_map,
                            default_colors,
                            color_idx,
                        );
                    }
                }
            }

            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_arbitrary_axis_ocs_to_wcs() {
        let norm_standard = DxfVector::new(0.0, 0.0, 1.0);
        let p_standard = ocs_to_wcs(150.0, 75.0, 0.0, &norm_standard);
        assert!((p_standard.x - 150.0).abs() < 1e-6);
        assert!((p_standard.y - 75.0).abs() < 1e-6);

        // Inverted normal (0, 0, -1): Local X axis points in -X direction
        let norm_inverted = DxfVector::new(0.0, 0.0, -1.0);
        let p_inverted = ocs_to_wcs(-157.5, 32.0, 0.0, &norm_inverted);
        // An OCS x of -157.5 with inverted normal corresponds to +157.5 in WCS
        assert!((p_inverted.x - 157.5).abs() < 1e-6);
        assert!((p_inverted.y - 32.0).abs() < 1e-6);
    }

    #[test]
    fn test_cutie_danutz_panouri_import() {
        let path = Path::new("/Users/dinu/Projects/various geometries/cutieDanutz_panouri.dxf");
        if path.exists() {
            let doc = DxfImporter::load_file(path).expect("Should load cutieDanutz_panouri.dxf cleanly");
            let bbox = doc.bounding_box().expect("Should have valid bounding box");
            
            // Expected bounds: X in [58.32, 1112.59], Y in [198.48, 937.35]
            assert!((bbox.min_x() - 58.32).abs() < 1.0, "Expected min_x ~58.32, got {}", bbox.min_x());
            assert!((bbox.max_x() - 1112.59).abs() < 1.0, "Expected max_x ~1112.59, got {}", bbox.max_x());
            assert!((bbox.min_y() - 198.48).abs() < 1.0, "Expected min_y ~198.48, got {}", bbox.min_y());
            assert!((bbox.max_y() - 937.35).abs() < 1.0, "Expected max_y ~937.35, got {}", bbox.max_y());
            
            // Single-layer DXF is automatically chained and classified into 10 outer panels + 174 interior holes
            assert_eq!(doc.entity_count(), 184);
            assert_eq!(doc.layers.len(), 2);
            assert_eq!(doc.layers[0].name, "Outer Profiles");
            assert_eq!(doc.layers[0].entities.len(), 10);
            assert_eq!(doc.layers[1].name, "Interior Profiles");
            assert_eq!(doc.layers[1].entities.len(), 174);
        }
    }

    #[test]
    fn test_load_all_user_dxfs() {
        let dir = Path::new("/Users/dinu/Projects/various geometries");
        if let Ok(entries) = std::fs::read_dir(dir) {
            for entry in entries.flatten() {
                let p = entry.path();
                if p.extension().map_or(false, |ext| ext.eq_ignore_ascii_case("dxf")) {
                    let doc = DxfImporter::load_file(&p);
                    assert!(doc.is_ok(), "Failed to import {:?}", p);
                    let doc = doc.unwrap();
                    println!("Successfully loaded {:?} with {} entities", p.file_name().unwrap(), doc.entity_count());
                }
            }
        }
    }
}
