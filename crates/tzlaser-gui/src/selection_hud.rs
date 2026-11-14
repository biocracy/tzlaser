use egui::{Button, Color32, ComboBox, Context, DragValue, Grid, RichText, Vec2, Window};
use tzlaser_core::{Document, HistoryManager};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnchorPoint {
    BottomLeft,
    Center,
}

pub struct SelectionHudState {
    pub anchor: AnchorPoint,
    pub step_mm: f64,
    pub array_cols: usize,
    pub array_rows: usize,
    pub array_gap_x: f64,
    pub array_gap_y: f64,
}

impl Default for SelectionHudState {
    fn default() -> Self {
        Self {
            anchor: AnchorPoint::BottomLeft,
            step_mm: 10.0,
            array_cols: 2,
            array_rows: 2,
            array_gap_x: 5.0,
            array_gap_y: 5.0,
        }
    }
}

pub fn render_selection_hud(
    ctx: &Context,
    state: &mut SelectionHudState,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    bed_width: f32,
    bed_height: f32,
) {
    if selection.is_empty() {
        return;
    }

    let Some(bounds) = doc.selection_bounding_box(selection) else {
        return;
    };

    let width = bounds.width();
    let height = bounds.height();

    // Determine current position based on selected anchor
    let (current_x, current_y) = match state.anchor {
        AnchorPoint::BottomLeft => (bounds.min_x(), bounds.min_y()),
        AnchorPoint::Center => (
            (bounds.min_x() + bounds.max_x()) / 2.0,
            (bounds.min_y() + bounds.max_y()) / 2.0,
        ),
    };

    let mut edit_x = current_x;
    let mut edit_y = current_y;

    Window::new(RichText::new("🎯 Selection Transform").color(Color32::from_rgb(0, 225, 255)).strong())
        .default_pos([260.0, 75.0])
        .default_width(280.0)
        .resizable(false)
        .collapsible(true)
        .show(ctx, |ui| {
            // Anchor & selection count
            let has_group = doc.selection_has_group(selection);
            ui.horizontal(|ui| {
                let label_text = if selection.len() == 1 && has_group {
                    "1 Group selected".to_string()
                } else {
                    format!("{} selected", selection.len())
                };
                ui.label(RichText::new(label_text).italics().color(Color32::from_rgb(255, 220, 50)));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.selectable_value(&mut state.anchor, AnchorPoint::Center, "Center");
                    ui.selectable_value(&mut state.anchor, AnchorPoint::BottomLeft, "Origin (BL)");
                });
            });

            // Group / Ungroup action bar
            let can_group = selection.len() > 1;
            if can_group || has_group {
                ui.horizontal(|ui| {
                    if can_group {
                        let group_btn = Button::new(RichText::new("🔗 Group Profiles").strong().color(Color32::from_rgb(0, 230, 255)))
                            .fill(Color32::from_rgb(24, 45, 65));
                        if ui.add(group_btn).on_hover_text("Lock selected profiles into a single rigid object with fixed relative positions").clicked() {
                            history.record(doc, selection, "Group Profiles");
                            doc.group_selection(selection);
                        }
                    }

                    if has_group {
                        let ungroup_btn = Button::new(RichText::new("🔓 Ungroup").color(Color32::from_rgb(255, 190, 70)))
                            .fill(Color32::from_rgb(45, 38, 25));
                        if ui.add(ungroup_btn).on_hover_text("Break grouped object back into individual profiles").clicked() {
                            history.record(doc, selection, "Ungroup Profiles");
                            doc.ungroup_selection(selection);
                        }
                    }
                });
            }

            ui.separator();

            // 0. Selection Layer Assignment Dropdown (preserves cut order indexing)
            ui.horizontal(|ui| {
                ui.label(RichText::new("Layer:").strong());
                let first_layer = selection.first().map(|&(l, _)| l);
                let all_same_layer = first_layer
                    .map(|fl| selection.iter().all(|&(l, _)| l == fl))
                    .unwrap_or(false);
                let current_layer_text = if all_same_layer {
                    first_layer
                        .and_then(|l| doc.layers.get(l))
                        .map(|layer| layer.name.clone())
                        .unwrap_or_else(|| "Unknown".to_string())
                } else {
                    "Mixed Layers".to_string()
                };

                let mut target_layer: Option<usize> = None;
                ComboBox::from_id_salt("selection_layer_combo")
                    .selected_text(RichText::new(current_layer_text).strong())
                    .width(175.0)
                    .show_ui(ui, |ui| {
                        for (l_idx, layer) in doc.layers.iter().enumerate() {
                            if layer.is_travel {
                                continue;
                            }
                            let swatch = Color32::from_rgb(layer.color.r, layer.color.g, layer.color.b);
                            let selected = all_same_layer && first_layer == Some(l_idx);
                            let label = RichText::new(format!("■ {}", layer.name)).color(swatch);
                            if ui.selectable_label(selected, label).clicked() {
                                target_layer = Some(l_idx);
                            }
                        }
                    });

                if let Some(t_idx) = target_layer {
                    if !(all_same_layer && first_layer == Some(t_idx)) {
                        history.record(doc, selection, "Change Selection Layer");
                        doc.move_selection_to_layer(selection, t_idx);
                    }
                }
            });

            ui.add_space(4.0);

            // 1. Position & Dimensions Numeric Grid
            Grid::new("selection_coords_grid")
                .num_columns(4)
                .spacing([12.0, 6.0])
                .show(ui, |ui| {
                    ui.label(RichText::new("X:").strong());
                    let x_resp = ui.add(DragValue::new(&mut edit_x).speed(0.5).suffix(" mm"));
                    if x_resp.drag_started() {
                        history.record(doc, selection, "Edit X Position");
                    }
                    if x_resp.changed() {
                        let dx = edit_x - current_x;
                        doc.translate_selection(selection, dx, 0.0);
                    }

                    ui.label(RichText::new("W:").weak());
                    ui.monospace(format!("{:.2} mm", width));
                    ui.end_row();

                    ui.label(RichText::new("Y:").strong());
                    let y_resp = ui.add(DragValue::new(&mut edit_y).speed(0.5).suffix(" mm"));
                    if y_resp.drag_started() {
                        history.record(doc, selection, "Edit Y Position");
                    }
                    if y_resp.changed() {
                        let dy = edit_y - current_y;
                        doc.translate_selection(selection, 0.0, dy);
                    }

                    ui.label(RichText::new("H:").weak());
                    ui.monospace(format!("{:.2} mm", height));
                    ui.end_row();
                });

            ui.add_space(6.0);
            ui.separator();

            // 2. Dropdown List for Increment Choices
            ui.horizontal(|ui| {
                ui.label(RichText::new("Nudge Step:").strong());
                let step_choices = [0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0];
                let current_label = if state.step_mm < 1.0 {
                    format!("{} mm", state.step_mm)
                } else {
                    format!("{:.0} mm", state.step_mm)
                };

                ComboBox::from_id_salt("nudge_step_combo")
                    .selected_text(RichText::new(current_label).strong())
                    .width(110.0)
                    .show_ui(ui, |ui| {
                        for &choice in &step_choices {
                            let label = if choice < 1.0 {
                                format!("{} mm", choice)
                            } else {
                                format!("{:.0} mm", choice)
                            };
                            let selected = (state.step_mm - choice).abs() < 1e-4;
                            if ui.selectable_label(selected, label).clicked() {
                                state.step_mm = choice;
                            }
                        }
                    });
            });

            ui.add_space(6.0);

            // 3. Directional Cross-Wise (D-Pad) Nudge Buttons with Center Option
            let step = state.step_mm;
            let btn_size = Vec2::new(72.0, 30.0);

            ui.vertical_centered(|ui| {
                // Row 1: Y+
                if ui.add_sized(btn_size, Button::new(RichText::new("▲ Y+").size(14.0))).clicked() {
                    history.record(doc, selection, "Nudge Y+");
                    doc.translate_selection(selection, 0.0, step);
                }

                // Row 2: X- | Center on Bed | X+
                ui.horizontal(|ui| {
                    ui.add_space((ui.available_width() - (btn_size.x * 3.0 + 12.0)).max(0.0) / 2.0);
                    if ui.add_sized(btn_size, Button::new(RichText::new("◄ X-").size(14.0))).clicked() {
                        history.record(doc, selection, "Nudge X-");
                        doc.translate_selection(selection, -step, 0.0);
                    }

                    // Center Option
                    let center_btn = Button::new(RichText::new("Center").color(Color32::from_rgb(0, 225, 255)))
                        .fill(Color32::from_rgb(34, 44, 56));
                    if ui.add_sized(btn_size, center_btn).clicked() {
                        history.record(doc, selection, "Center Selection");
                        let target_cx = (bed_width as f64) / 2.0;
                        let target_cy = (bed_height as f64) / 2.0;
                        let cur_cx = (bounds.min_x() + bounds.max_x()) / 2.0;
                        let cur_cy = (bounds.min_y() + bounds.max_y()) / 2.0;
                        doc.translate_selection(selection, target_cx - cur_cx, target_cy - cur_cy);
                    }

                    if ui.add_sized(btn_size, Button::new(RichText::new("X+ ►").size(14.0))).clicked() {
                        history.record(doc, selection, "Nudge X+");
                        doc.translate_selection(selection, step, 0.0);
                    }
                });

                // Row 3: Y-
                if ui.add_sized(btn_size, Button::new(RichText::new("▼ Y-").size(14.0))).clicked() {
                    history.record(doc, selection, "Nudge Y-");
                    doc.translate_selection(selection, 0.0, -step);
                }
            });

            ui.add_space(8.0);
            ui.separator();

            // 4. Mirroring Controls
            ui.label(RichText::new("Mirror / Symmetry:").strong());
            ui.horizontal(|ui| {
                let flip_h_btn = Button::new(RichText::new("⇋ Mirror X (Horiz)").size(12.0))
                    .fill(Color32::from_rgb(38, 42, 50));
                if ui.add_sized([126.0, 28.0], flip_h_btn).clicked() {
                    history.record(doc, selection, "Mirror X");
                    doc.mirror_selection_horizontal(selection);
                }

                let flip_v_btn = Button::new(RichText::new("⇵ Mirror Y (Vert)").size(12.0))
                    .fill(Color32::from_rgb(38, 42, 50));
                if ui.add_sized([126.0, 28.0], flip_v_btn).clicked() {
                    history.record(doc, selection, "Mirror Y");
                    doc.mirror_selection_vertical(selection);
                }
            });

            ui.add_space(8.0);
            ui.separator();

            // 5. Array / Matrix Duplication
            ui.collapsing(RichText::new("▦ Array Matrix Copy (X/Y)").strong(), |ui| {
                Grid::new("array_grid").num_columns(4).spacing([8.0, 4.0]).show(ui, |ui| {
                    ui.label("Cols (X):");
                    ui.add(DragValue::new(&mut state.array_cols).range(1..=50));

                    ui.label("Gap X:");
                    ui.add(DragValue::new(&mut state.array_gap_x).speed(0.5).suffix(" mm"));
                    ui.end_row();

                    ui.label("Rows (Y):");
                    ui.add(DragValue::new(&mut state.array_rows).range(1..=50));

                    ui.label("Gap Y:");
                    ui.add(DragValue::new(&mut state.array_gap_y).speed(0.5).suffix(" mm"));
                    ui.end_row();
                });

                ui.add_space(6.0);

                ui.horizontal(|ui| {
                    // Quick duplicate buttons
                    if ui.button("+ Copy X").on_hover_text("Quick copy 1 step to the right").clicked() {
                        history.record(doc, selection, "Copy X");
                        let pitch_x = width + state.array_gap_x;
                        let copies = doc.duplicate_selection(selection, pitch_x, 0.0);
                        *selection = copies;
                    }
                    if ui.button("+ Copy Y").on_hover_text("Quick copy 1 step upward").clicked() {
                        history.record(doc, selection, "Copy Y");
                        let pitch_y = height + state.array_gap_y;
                        let copies = doc.duplicate_selection(selection, 0.0, pitch_y);
                        *selection = copies;
                    }

                    let array_btn = Button::new(RichText::new("Generate Matrix").color(Color32::WHITE).strong())
                        .fill(Color32::from_rgb(25, 110, 190));
                    if ui.add(array_btn).clicked() {
                        history.record(doc, selection, "Generate Matrix");
                        let pitch_x = width + state.array_gap_x;
                        let pitch_y = height + state.array_gap_y;
                        let all = doc.array_copy(
                            selection,
                            state.array_cols,
                            state.array_rows,
                            pitch_x,
                            pitch_y,
                        );
                        *selection = all;
                    }
                });
            });

            ui.add_space(4.0);
            ui.separator();

            // Quick Snap Origin
            if ui.add(Button::new("Snap Origin to (0,0)").small()).clicked() {
                history.record(doc, selection, "Snap Origin");
                let dx = 0.0 - bounds.min_x();
                let dy = 0.0 - bounds.min_y();
                doc.translate_selection(selection, dx, dy);
            }
        });
}
