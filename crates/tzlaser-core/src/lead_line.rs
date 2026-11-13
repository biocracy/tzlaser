use kurbo::Point;
use crate::{Document, Entity};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LeadType {
    Line,
    Arc,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LeadConfig {
    pub lead_type: LeadType,
    pub length_mm: f64,
    pub angle_deg: f64,
    pub overcut_mm: f64,
}

impl Default for LeadConfig {
    fn default() -> Self {
        Self {
            lead_type: LeadType::Line,
            length_mm: 3.5,
            angle_deg: 90.0,
            overcut_mm: 0.0,
        }
    }
}

/// Apply lead-in lines to the document (all closed entities, or specifically selected ones).
pub fn apply_lead_lines(
    doc: &mut Document,
    selection: &[(usize, usize)],
    config: &LeadConfig,
) -> usize {
    let mut count = 0;

    let target_entities: Vec<(usize, usize)> = if selection.is_empty() {
        // Apply to all closed entities across all visible layers
        let mut list = Vec::new();
        for (l_idx, layer) in doc.layers.iter().enumerate() {
            if !layer.visible {
                continue;
            }
            for (e_idx, ent) in layer.entities.iter().enumerate() {
                if ent.is_closed() {
                    list.push((l_idx, e_idx));
                }
            }
        }
        list
    } else {
        selection.to_vec()
    };

    for (l_idx, e_idx) in target_entities {
        if l_idx >= doc.layers.len() || e_idx >= doc.layers[l_idx].entities.len() {
            continue;
        }

        let is_interior_layer = doc.layers[l_idx].name.to_lowercase().contains("interior")
            || doc.layers[l_idx].name.to_lowercase().contains("hole");

        let entity = &mut doc.layers[l_idx].entities[e_idx];
        match entity {
            Entity::Circle { center, radius } => {
                // For circle: lead in along radial line.
                // If interior hole: pierce inside (radius - length).
                // If outer profile: pierce outside (radius + length).
                let sign = if is_interior_layer { -1.0 } else { 1.0 };
                let lead_start = Point::new(center.x + *radius + sign * config.length_mm, center.y);

                // Convert circle to a closed path with lead-in
                let steps = 48;
                let mut points = vec![lead_start];
                for i in 0..=steps {
                    let angle = (i as f64) * std::f64::consts::TAU / (steps as f64);
                    points.push(Point::new(
                        center.x + *radius * angle.cos(),
                        center.y + *radius * angle.sin(),
                    ));
                }
                *entity = Entity::Polyline {
                    points,
                    closed: false,
                };
                count += 1;
            }
            Entity::Polyline { points, closed } if *closed && points.len() >= 3 => {
                let p0 = points[0];
                let p1 = points[1];
                let dx = p1.x - p0.x;
                let dy = p1.y - p0.y;
                let len = dx.hypot(dy);
                if len > 1e-6 {
                    let tx = dx / len;
                    let ty = dy / len;

                    // Perpendicular normal (pointing left of direction)
                    let nx = -ty;
                    let ny = tx;

                    // Determine inward vs outward by sign
                    let sign = if is_interior_layer { -1.0 } else { 1.0 };

                    let angle_rad = config.angle_deg.to_radians();
                    let cos_a = angle_rad.cos();
                    let sin_a = angle_rad.sin();

                    // Lead direction vector
                    let ldir_x = sign * (nx * sin_a - tx * cos_a);
                    let ldir_y = sign * (ny * sin_a - ty * cos_a);

                    let lead_start = Point::new(
                        p0.x + config.length_mm * ldir_x,
                        p0.y + config.length_mm * ldir_y,
                    );

                    let mut new_points = Vec::with_capacity(points.len() + 2);
                    new_points.push(lead_start);
                    new_points.extend(points.iter().copied());
                    new_points.push(p0); // Return to p0 to close the loop

                    *points = new_points;
                    *closed = false;
                    count += 1;
                }
            }
            _ => {}
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

        let mut inner_layer = Layer::new("Interior Profiles", Color::YELLOW);
        inner_layer.entities.push(Entity::Circle {
            center: Point::new(50.0, 50.0),
            radius: 10.0,
        });

        doc.layers = vec![outer_layer, inner_layer];

        let config = LeadConfig {
            lead_type: LeadType::Line,
            length_mm: 5.0,
            angle_deg: 90.0,
            overcut_mm: 0.0,
        };

        let applied = apply_lead_lines(&mut doc, &[], &config);
        assert_eq!(applied, 2);

        // Check outer polyline lead-in
        if let Entity::Polyline { points, closed } = &doc.layers[0].entities[0] {
            assert!(!*closed);
            assert!(points.len() > 4);
        } else {
            panic!("Expected polyline");
        }

        // Check inner circle lead-in (interior hole pierces inside: 50 + 10 - 5 = 55)
        if let Entity::Polyline { points, closed } = &doc.layers[1].entities[0] {
            assert!(!*closed);
            let p_start = points[0];
            assert!((p_start.x - 55.0).abs() < 1e-3);
            assert!((p_start.y - 50.0).abs() < 1e-3);
        } else {
            panic!("Expected circle converted to toolpath with lead");
        }
    }
}
