use egui::{Button, Color32, ComboBox, Context, DragValue, Grid, RichText, TextEdit, Window};
use tzlaser_core::{Document, MaterialLibrary, MaterialPreset};

use crate::panels::PanelState;

pub fn render_layer_popup(
    ctx: &Context,
    panel_state: &mut PanelState,
    doc: &mut Document,
    status_message: &mut String,
) {
    let Some(layer_idx) = panel_state.active_layer_popup else {
        return;
    };
    if layer_idx >= doc.layers.len() {
        panel_state.active_layer_popup = None;
        return;
    }

    let mut is_open = true;
    let mut speed_changed = false;

    let layer_title = doc.layers[layer_idx].name.clone();
    let layer_color = doc.layers[layer_idx].color;
    let profile_count = doc.layers[layer_idx].entities.len();

    Window::new(
        RichText::new(format!("🎨 Layer Properties: {}", layer_title))
            .color(Color32::from_rgb(layer_color.r, layer_color.g, layer_color.b))
            .strong(),
    )
    .id(egui::Id::new("layer_properties_popup"))
    .open(&mut is_open)
    .default_pos([260.0, 260.0])
    .default_width(305.0)
    .resizable(false)
    .collapsible(true)
    .show(ctx, |ui| {
        // Header: color swatch, profile count, visibility toggle
        ui.horizontal(|ui| {
            ui.colored_label(
                Color32::from_rgb(layer_color.r, layer_color.g, layer_color.b),
                "■",
            );
            ui.label(RichText::new(&layer_title).strong());
            let count_text = if doc.layers[layer_idx].is_travel {
                "(Travel Layer)".to_string()
            } else {
                format!("({} profiles)", profile_count)
            };
            ui.label(RichText::new(count_text).size(11.5).weak());
            if !doc.layers[layer_idx].is_travel {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.checkbox(&mut doc.layers[layer_idx].visible, "Visible");
                });
            }
        });

        ui.separator();

        // 1. Load Profile from Internal DB Dropdown (shown whenever DB has entries)
        if !panel_state.material_db.materials.is_empty() {
            let current_params = &doc.layers[layer_idx].params;
            let matched_idx = panel_state
                .material_db
                .materials
                .iter()
                .position(|m| &m.params == current_params);
            let selected_label = matched_idx
                .and_then(|i| panel_state.material_db.materials.get(i))
                .map(|m| m.name.as_str())
                .unwrap_or("Select DB profile...");

            let mut load_idx: Option<usize> = None;
            let mut delete_idx: Option<usize> = None;

            ui.horizontal(|ui| {
                ui.label(RichText::new("DB Profile:").strong());
                ComboBox::from_id_salt("layer_popup_db_select")
                    .selected_text(selected_label)
                    .width(175.0)
                    .show_ui(ui, |ui| {
                        for (i, preset) in panel_state.material_db.materials.iter().enumerate() {
                            let prefix = if preset.is_builtin { "" } else { "★ " };
                            let label = format!(
                                "{}{} ({:.0} mm/s, {:.0}%)",
                                prefix,
                                preset.name,
                                preset.params.speed_mm_s,
                                preset.params.power_percent
                            );
                            let is_sel = matched_idx == Some(i);
                            if ui.selectable_label(is_sel, label).clicked() {
                                load_idx = Some(i);
                            }
                        }
                    });

                if let Some(m_idx) = matched_idx {
                    if ui
                        .small_button("🗑")
                        .on_hover_text("Delete this profile from DB")
                        .clicked()
                    {
                        delete_idx = Some(m_idx);
                    }
                }
            });

            if let Some(idx) = load_idx {
                if let Some(preset) = panel_state.material_db.materials.get(idx) {
                    doc.layers[layer_idx].params = preset.params.clone();
                    panel_state.new_mat_name = preset.name.clone();
                    panel_state.new_mat_material = preset.material.clone();
                    panel_state.new_mat_thickness = preset.thickness_mm;
                    speed_changed = true;
                    *status_message = format!(
                        "Loaded DB profile '{}' onto layer '{}'.",
                        preset.name, layer_title
                    );
                }
            }

            if let Some(idx) = delete_idx {
                let removed_name = panel_state.material_db.materials[idx].name.clone();
                match panel_state.material_db.remove_material(idx) {
                    Ok(()) => {
                        *status_message = format!("Deleted '{}' from material DB.", removed_name);
                    }
                    Err(e) => *status_message = e,
                }
            }

            ui.add_space(4.0);
        }

        // 2. Layer Cutting Parameters Grid
        Grid::new("layer_popup_params_grid")
            .num_columns(2)
            .spacing([14.0, 6.0])
            .show(ui, |ui| {
                let params = &mut doc.layers[layer_idx].params;

                ui.label(RichText::new("Speed:").strong());
                if ui
                    .add(
                        DragValue::new(&mut params.speed_mm_s)
                            .range(0.5..=2000.0)
                            .speed(1.0)
                            .suffix(" mm/s"),
                    )
                    .changed()
                {
                    speed_changed = true;
                }
                ui.end_row();

                ui.label(RichText::new("Power:").strong());
                ui.add(
                    DragValue::new(&mut params.power_percent)
                        .range(0.0..=100.0)
                        .speed(1.0)
                        .suffix(" %"),
                );
                ui.end_row();

                ui.label(RichText::new("Pierce:").strong());
                ui.add(
                    DragValue::new(&mut params.pierce_time_ms)
                        .range(0..=10000)
                        .speed(10.0)
                        .suffix(" ms"),
                );
                ui.end_row();

                ui.label(RichText::new("Cut Height:").strong());
                ui.add(
                    DragValue::new(&mut params.cut_height_mm)
                        .range(0.1..=50.0)
                        .speed(0.1)
                        .suffix(" mm"),
                );
                ui.end_row();

                ui.label(RichText::new("Frequency:").strong());
                ui.add(
                    DragValue::new(&mut params.frequency_hz)
                        .range(0..=50000)
                        .speed(50.0)
                        .suffix(" Hz"),
                );
                ui.end_row();

                ui.label(RichText::new("Assist Gas:").strong());
                ComboBox::from_id_salt("layer_popup_gas")
                    .selected_text(&params.gas_type)
                    .width(95.0)
                    .show_ui(ui, |ui| {
                        for gas in ["Air", "O2", "N2", "None"] {
                            if ui.selectable_label(params.gas_type == gas, gas).clicked() {
                                params.gas_type = gas.to_string();
                            }
                        }
                    });
                ui.end_row();
            });

        ui.separator();

        // 3. Save Individual Profile to Internal DB
        ui.label(RichText::new("Save Profile to DB:").strong().size(12.0));
        Grid::new("layer_popup_save_db_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("Name:");
                ui.add(
                    TextEdit::singleline(&mut panel_state.new_mat_name)
                        .hint_text("e.g. Steel 2mm (O2)")
                        .desired_width(185.0),
                );
                ui.end_row();

                ui.label("Material:");
                ui.horizontal(|ui| {
                    ui.add(
                        TextEdit::singleline(&mut panel_state.new_mat_material)
                            .hint_text("Mild Steel")
                            .desired_width(105.0),
                    );
                    ui.add(
                        DragValue::new(&mut panel_state.new_mat_thickness)
                            .range(0.0..=100.0)
                            .speed(0.5)
                            .suffix(" mm"),
                    );
                });
                ui.end_row();
            });

        ui.add_space(4.0);
        ui.horizontal(|ui| {
            let save_db_btn = Button::new(
                RichText::new("💾 Save to DB")
                    .color(Color32::WHITE)
                    .strong(),
            )
            .fill(Color32::from_rgb(28, 115, 92));
            if ui.add(save_db_btn).clicked() {
                let name = panel_state.new_mat_name.trim().to_string();
                if name.is_empty() {
                    *status_message = "Enter a profile name before saving to DB.".to_string();
                } else {
                    let preset = MaterialPreset {
                        name: name.clone(),
                        material: panel_state.new_mat_material.trim().to_string(),
                        thickness_mm: panel_state.new_mat_thickness,
                        params: doc.layers[layer_idx].params.clone(),
                        notes: String::new(),
                        is_builtin: false,
                    };
                    match panel_state.material_db.upsert_material(preset) {
                        Ok(updated) => {
                            *status_message = if updated {
                                format!("Updated profile '{}' in DB.", name)
                            } else {
                                format!("Saved profile '{}' to DB.", name)
                            };
                        }
                        Err(e) => *status_message = e,
                    }
                }
            }

            if ui
                .button("💾 Save File")
                .on_hover_text("Export layer parameters to JSON file")
                .clicked()
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("Layer Config", &["json"])
                    .set_file_name("layers_config.json")
                    .save_file()
                {
                    match MaterialLibrary::export_layers_to_file(doc, &path) {
                        Ok(()) => {
                            *status_message =
                                format!("Layer configuration saved to {}.", path.display());
                        }
                        Err(e) => *status_message = e,
                    }
                }
            }

            if ui
                .button("📂 Load File")
                .on_hover_text("Load layer parameters from JSON file")
                .clicked()
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("Layer Config", &["json"])
                    .pick_file()
                {
                    match MaterialLibrary::import_layers_from_file(doc, &path) {
                        Ok(count) => {
                            speed_changed = true;
                            *status_message = format!(
                                "Loaded {} layer config(s) from {}.",
                                count,
                                path.display()
                            );
                        }
                        Err(e) => *status_message = e,
                    }
                }
            }
        });
    });

    if !is_open {
        panel_state.active_layer_popup = None;
    }
    if speed_changed {
        panel_state.sync_layer_speeds(doc);
    }
}

