use eframe::egui::{
    self, Button, Color32, Frame, Margin, RichText, ScrollArea, Stroke, Ui, Vec2,
};
use std::net::SocketAddr;
use tzlaser_core::{apply_lead_lines, Document, DxfImporter, HistoryManager, LeadConfig, LeadType};
use tzlaser_protocol::{LaserController, MachineStatus};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RibbonTab {
    File,
    Profiles,
    Laser,
}

pub struct RibbonState {
    pub active_tab: RibbonTab,
    pub lead_config: LeadConfig,
    pub pulse_duration_ms: u32,
    pub pulse_power_pct: f32,
    pub follower_enabled: bool,
    pub red_pointer_enabled: bool,
    pub gas_open: bool,
}

impl Default for RibbonState {
    fn default() -> Self {
        Self {
            active_tab: RibbonTab::Profiles,
            lead_config: LeadConfig::default(),
            pulse_duration_ms: 50,
            pulse_power_pct: 20.0,
            follower_enabled: false,
            red_pointer_enabled: false,
            gas_open: false,
        }
    }
}

pub fn render_ribbon(
    ui: &mut Ui,
    state: &mut RibbonState,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    canvas_state: &mut crate::canvas::CanvasState,
    panel_state: &mut crate::panels::PanelState,
    controller: &mut Option<LaserController>,
    status: &MachineStatus,
    status_message: &mut String,
) {
    // 1. Top Ribbon Tab Bar & Quick Access
    ui.horizontal(|ui| {
        // App Logo: Ţlaser (Romanian letter Ţ with cedilla)
        ui.horizontal(|ui| {
            ui.label(RichText::new("⚡ Ţlaser").size(17.0).color(Color32::from_rgb(255, 185, 45)).strong());
            ui.label(RichText::new("CNC").size(10.0).color(Color32::from_rgb(130, 140, 160)));
        });

        ui.add_space(6.0);

        // Quick Access Undo / Redo
        let can_undo = history.can_undo();
        let undo_desc = history.undo_description().unwrap_or("Undo");
        let undo_btn = Button::new(RichText::new("↩").size(13.0).color(if can_undo { Color32::WHITE } else { Color32::from_rgb(100, 100, 110) }))
            .fill(Color32::from_rgb(32, 34, 40));
        if ui.add_enabled(can_undo, undo_btn).on_hover_text(format!("Undo {} (Cmd+Z)", undo_desc)).clicked() {
            if let Some(desc) = history.undo(doc, selection) {
                *status_message = format!("Undo: {}", desc);
            }
        }

        let can_redo = history.can_redo();
        let redo_desc = history.redo_description().unwrap_or("Redo");
        let redo_btn = Button::new(RichText::new("↪").size(13.0).color(if can_redo { Color32::WHITE } else { Color32::from_rgb(100, 100, 110) }))
            .fill(Color32::from_rgb(32, 34, 40));
        if ui.add_enabled(can_redo, redo_btn).on_hover_text(format!("Redo {} (Cmd+Shift+Z / Cmd+Y)", redo_desc)).clicked() {
            if let Some(desc) = history.redo(doc, selection) {
                *status_message = format!("Redo: {}", desc);
            }
        }

        ui.add_space(8.0);

        // Ribbon Tab Buttons
        let tabs = [
            (RibbonTab::File, "📁 File & Project"),
            (RibbonTab::Profiles, "📐 Profile CAM"),
            (RibbonTab::Laser, "⚡ Laser & Hardware"),
        ];

        for (tab, label) in tabs {
            let is_active = state.active_tab == tab;
            let bg = if is_active {
                Color32::from_rgb(45, 52, 68)
            } else {
                Color32::from_rgb(26, 28, 34)
            };
            let text_color = if is_active {
                Color32::from_rgb(0, 225, 255)
            } else {
                Color32::from_rgb(180, 185, 195)
            };

            let btn = Button::new(RichText::new(label).size(12.5).color(text_color).strong())
                .fill(bg)
                .stroke(if is_active { Stroke::new(1.0, Color32::from_rgb(0, 180, 240)) } else { Stroke::NONE });

            if ui.add(btn).clicked() {
                state.active_tab = tab;
            }
        }

        // Right Aligned: Hardware Connection Badge
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if let Some(ref c) = controller {
                let (dot, text, color) = if c.is_connected() {
                    ("●", "ONLINE (10.1.1.168:502)", Color32::GREEN)
                } else {
                    ("○", "CONNECTING...", Color32::YELLOW)
                };

                if ui.add(Button::new(RichText::new("Disconnect").size(11.0).color(Color32::LIGHT_RED)).fill(Color32::from_rgb(45, 25, 25))).clicked() {
                    *controller = None;
                    *status_message = "Disconnected from machine controller.".to_string();
                }

                ui.label(RichText::new(format!("{} {}", dot, text)).size(11.5).color(color).strong());
            } else {
                let connect_btn = Button::new(RichText::new("🔌 Connect").size(11.5).color(Color32::WHITE))
                    .fill(Color32::from_rgb(30, 85, 150));
                if ui.add(connect_btn).clicked() {
                    if let Ok(addr) = panel_state.target_ip.parse::<SocketAddr>() {
                        *controller = Some(LaserController::new(addr));
                        *status_message = format!("Connecting to controller board at {}...", addr);
                    } else {
                        *status_message = "Invalid IP address format".to_string();
                    }
                }
            }
        });
    });

    ui.add_space(3.0);

    // 2. Ribbon Content Area with Horizontal Scrolling (Never clips on smaller screens)
    Frame::new()
        .fill(Color32::from_rgb(28, 30, 36))
        .inner_margin(Margin::symmetric(6, 5))
        .stroke(Stroke::new(1.0, Color32::from_rgb(42, 45, 54)))
        .show(ui, |ui| {
            ScrollArea::horizontal()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    match state.active_tab {
                        RibbonTab::File => render_file_tab(ui, doc, selection, history, canvas_state, status_message),
                        RibbonTab::Profiles => render_profiles_tab(ui, state, doc, selection, history, canvas_state, status_message),
                        RibbonTab::Laser => render_laser_tab(ui, state, controller, status, status_message),
                    }
                });
        });
}

fn ribbon_group(ui: &mut Ui, title: &str, content: impl FnOnce(&mut Ui)) {
    ui.group(|ui| {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                content(ui);
            });
            ui.add_space(2.0);
            ui.centered_and_justified(|ui| {
                ui.label(RichText::new(title).size(9.5).color(Color32::from_rgb(130, 135, 150)).strong());
            });
        });
    });
}

fn render_file_tab(
    ui: &mut Ui,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    canvas_state: &mut crate::canvas::CanvasState,
    status_message: &mut String,
) {
    ui.horizontal(|ui| {
        ribbon_group(ui, "DXF & Projects", |ui| {
            let open_btn = Button::new(RichText::new("📂 Open DXF").size(12.0).strong())
                .fill(Color32::from_rgb(35, 65, 95));
            if ui.add(open_btn).on_hover_text("Open AutoCAD DXF file (2D lines, arcs, circles, polylines, splines)").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("CAD Files", &["dxf", "DXF"])
                    .pick_file()
                {
                    match DxfImporter::load_file(&path) {
                        Ok(imported_doc) => {
                            history.record(doc, selection, "Open DXF");
                            *status_message = format!(
                                "Loaded DXF: {} ({} profiles across {} layers)",
                                path.file_name().unwrap_or_default().to_string_lossy(),
                                imported_doc.entity_count(),
                                imported_doc.layers.len()
                            );
                            *doc = imported_doc;
                            selection.clear();
                        }
                        Err(e) => {
                            *status_message = format!("DXF Load error: {}", e);
                        }
                    }
                }
            }

            let save_btn = Button::new(RichText::new("💾 Save Job").size(12.0))
                .fill(Color32::from_rgb(40, 45, 55));
            if ui.add(save_btn).on_hover_text("Save job layout (*.tzjob)").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("Ţlaser Job", &["tzjob", "json"])
                    .set_file_name("job.tzjob")
                    .save_file()
                {
                    match serde_json::to_string_pretty(&doc) {
                        Ok(json) => {
                            if std::fs::write(&path, json).is_ok() {
                                *status_message = format!("Saved job to {}", path.display());
                            }
                        }
                        Err(e) => *status_message = format!("Save error: {}", e),
                    }
                }
            }

            let clear_btn = Button::new(RichText::new("📄 New Job").size(12.0).color(Color32::from_rgb(255, 140, 140)))
                .fill(Color32::from_rgb(48, 35, 38));
            if ui.add(clear_btn).on_hover_text("Clear all profiles and reset table").clicked() {
                history.record(doc, selection, "New Job");
                *doc = Document::new();
                selection.clear();
                *status_message = "New job created. Workspace cleared.".to_string();
            }
        });

        ui.add_space(4.0);

        ribbon_group(ui, "History & Undo", |ui| {
            let can_undo = history.can_undo();
            let undo_label = format!("↩ Undo\n{}", history.undo_description().unwrap_or(""));
            let undo_btn = Button::new(RichText::new(undo_label).size(11.0))
                .fill(Color32::from_rgb(40, 45, 55));
            if ui.add_enabled(can_undo, undo_btn).on_hover_text("Undo last modification (Cmd+Z)").clicked() {
                if let Some(desc) = history.undo(doc, selection) {
                    *status_message = format!("Undo: {}", desc);
                }
            }

            let can_redo = history.can_redo();
            let redo_label = format!("↪ Redo\n{}", history.redo_description().unwrap_or(""));
            let redo_btn = Button::new(RichText::new(redo_label).size(11.0))
                .fill(Color32::from_rgb(40, 45, 55));
            if ui.add_enabled(can_redo, redo_btn).on_hover_text("Redo action (Cmd+Shift+Z / Cmd+Y)").clicked() {
                if let Some(desc) = history.redo(doc, selection) {
                    *status_message = format!("Redo: {}", desc);
                }
            }
        });

        ui.add_space(4.0);

        ribbon_group(ui, "Bed & Viewport", |ui| {
            if ui.add(Button::new(RichText::new("🔍 Fit View").size(12.0)).fill(Color32::from_rgb(40, 45, 55))).clicked() {
                if let Some(bounds) = doc.bounding_box() {
                    let center_x = (bounds.min_x() + bounds.max_x()) / 2.0;
                    let center_y = (bounds.min_y() + bounds.max_y()) / 2.0;
                    canvas_state.pan = Vec2::new(
                        420.0 - (center_x as f32) * canvas_state.zoom,
                        340.0 + (center_y as f32) * canvas_state.zoom,
                    );
                }
            }

            if ui.add(Button::new(RichText::new("1:1").size(12.0)).fill(Color32::from_rgb(40, 45, 55))).clicked() {
                canvas_state.zoom = 1.0;
            }

            ui.menu_button("📐 Bed Size", |ui| {
                if ui.button("1300 x 900 mm (Standard)").clicked() {
                    canvas_state.bed_width_mm = 1300.0;
                    canvas_state.bed_height_mm = 900.0;
                    ui.close_menu();
                }
                if ui.button("1500 x 3000 mm (Sheet Metal)").clicked() {
                    canvas_state.bed_width_mm = 3000.0;
                    canvas_state.bed_height_mm = 1500.0;
                    ui.close_menu();
                }
                if ui.button("2000 x 4000 mm (Heavy Plate)").clicked() {
                    canvas_state.bed_width_mm = 4000.0;
                    canvas_state.bed_height_mm = 2000.0;
                    ui.close_menu();
                }
            });
        });
    });
}

fn render_profiles_tab(
    ui: &mut Ui,
    state: &mut RibbonState,
    doc: &mut Document,
    selection: &mut Vec<(usize, usize)>,
    history: &mut HistoryManager,
    canvas_state: &mut crate::canvas::CanvasState,
    status_message: &mut String,
) {
    ui.horizontal(|ui| {
        // 1. Structure & Classification (Most critical for DXF CAM!)
        ribbon_group(ui, "Structure & Group", |ui| {
            let classify_btn = Button::new(RichText::new("⚡ Auto-Classify").size(12.0).color(Color32::from_rgb(100, 220, 255)).strong())
                .fill(Color32::from_rgb(25, 48, 70));
            if ui.add(classify_btn).on_hover_text("Auto-segregate Outer Profiles, Interior Holes, and Bend Lines").clicked() {
                history.record(doc, selection, "Auto-Classify");
                doc.auto_classify_profiles();
                selection.clear();
                *status_message = format!("Auto-classified {} layers: {}", doc.layers.len(), doc.layers.iter().map(|l| l.name.as_str()).collect::<Vec<_>>().join(", "));
            }

            let group_enabled = selection.len() > 1;
            let group_btn = Button::new(RichText::new("🔗 Group").size(12.0).color(if group_enabled { Color32::from_rgb(100, 220, 255) } else { Color32::GRAY }))
                .fill(Color32::from_rgb(24, 45, 65));
            if ui.add_enabled(group_enabled, group_btn).on_hover_text("Lock selected profiles into a single compound object (Cmd+G)").clicked() {
                history.record(doc, selection, "Group Profiles");
                doc.group_selection(selection);
            }

            let has_group = doc.selection_has_group(selection);
            let ungroup_btn = Button::new(RichText::new("🔓 Ungroup").size(12.0).color(if has_group { Color32::from_rgb(255, 190, 70) } else { Color32::GRAY }))
                .fill(Color32::from_rgb(45, 38, 25));
            if ui.add_enabled(has_group, ungroup_btn).on_hover_text("Break compound group back into individual profiles (Cmd+Shift+G)").clicked() {
                history.record(doc, selection, "Ungroup Profiles");
                doc.ungroup_selection(selection);
            }
        });

        ui.add_space(4.0);

        // 2. Cut Toolpath & Ordering
        ribbon_group(ui, "Cut Toolpath", |ui| {
            let order_bg = if canvas_state.show_cut_order {
                Color32::from_rgb(20, 65, 95)
            } else {
                Color32::from_rgb(38, 42, 50)
            };
            let order_color = if canvas_state.show_cut_order {
                Color32::from_rgb(0, 225, 255)
            } else {
                Color32::WHITE
            };
            if ui.add(Button::new(RichText::new("🔢 Cut Order").size(12.0).color(order_color)).fill(order_bg)).on_hover_text("Toggle cut sequence number badges and pierce start markers").clicked() {
                canvas_state.show_cut_order = !canvas_state.show_cut_order;
            }

            let dir_bg = if canvas_state.show_cut_direction {
                Color32::from_rgb(20, 65, 45)
            } else {
                Color32::from_rgb(38, 42, 50)
            };
            let dir_color = if canvas_state.show_cut_direction {
                Color32::from_rgb(0, 255, 120)
            } else {
                Color32::WHITE
            };
            if ui.add(Button::new(RichText::new("🧭 Direction").size(12.0).color(dir_color)).fill(dir_bg)).on_hover_text("Toggle cut direction arrowheads along toolpath").clicked() {
                canvas_state.show_cut_direction = !canvas_state.show_cut_direction;
            }

            if ui.add(Button::new(RichText::new("⚡ Optimize").size(12.0).color(Color32::from_rgb(255, 215, 0))).fill(Color32::from_rgb(45, 40, 22))).on_hover_text("Greedy nearest-neighbor TSP optimizer ensuring bend lines → interior holes → outer contours").clicked() {
                let seq = doc.compute_cut_sequence();
                *status_message = format!("Toolpath sequence optimized: {} pierce cuts planned (bends → holes → perimeters).", seq.len());
            }
        });

        ui.add_space(4.0);

        // 3. Selection Group
        ribbon_group(ui, "Select", |ui| {
            if ui.add(Button::new(RichText::new("⬚ All").size(12.0)).fill(Color32::from_rgb(40, 45, 55))).on_hover_text("Select all profiles on visible layers").clicked() {
                selection.clear();
                for (l_idx, layer) in doc.layers.iter().enumerate() {
                    if !layer.visible { continue; }
                    for e_idx in 0..layer.entities.len() {
                        selection.push((l_idx, e_idx));
                    }
                }
            }

            if ui.add(Button::new(RichText::new("🔄 Invert").size(12.0)).fill(Color32::from_rgb(40, 45, 55))).on_hover_text("Invert selection").clicked() {
                let mut new_sel = Vec::new();
                for (l_idx, layer) in doc.layers.iter().enumerate() {
                    if !layer.visible { continue; }
                    for e_idx in 0..layer.entities.len() {
                        if !selection.contains(&(l_idx, e_idx)) {
                            new_sel.push((l_idx, e_idx));
                        }
                    }
                }
                *selection = new_sel;
            }

            if !selection.is_empty() {
                if ui.add(Button::new(RichText::new("✕ Clear").size(12.0)).fill(Color32::from_rgb(45, 40, 42))).clicked() {
                    selection.clear();
                }
                if ui.add(Button::new(RichText::new("🗑 Del").size(12.0).color(Color32::from_rgb(255, 90, 90))).fill(Color32::from_rgb(52, 30, 32))).clicked() {
                    history.record(doc, selection, "Delete Selection");
                    doc.delete_selection(selection);
                    selection.clear();
                }
            }
        });

        ui.add_space(4.0);

        // 4. Lead Lines Group
        ribbon_group(ui, "Lead Lines", |ui| {
            let lead_btn = Button::new(RichText::new("📍 Add Leads").size(12.0).color(Color32::from_rgb(255, 215, 0)))
                .fill(Color32::from_rgb(45, 40, 25));
            if ui.add(lead_btn).on_hover_text(format!("Attach pierce lead-in lines ({:.1} mm at {:.0}°) to contours (interior holes pierce inside, outer pierce outside)", state.lead_config.length_mm, state.lead_config.angle_deg)).clicked() {
                history.record(doc, selection, "Add Lead Lines");
                let count = apply_lead_lines(doc, selection, &state.lead_config);
                *status_message = format!("Added lead-in lines to {} contours.", count);
            }

            ui.menu_button(format!("⚙ {:.1}mm", state.lead_config.length_mm), |ui| {
                ui.label(RichText::new("Lead-In Parameters").strong());
                ui.separator();
                ui.horizontal(|ui| {
                    ui.label("Length:");
                    ui.add(egui::DragValue::new(&mut state.lead_config.length_mm).speed(0.2).range(0.5..=20.0).suffix(" mm"));
                });
                ui.horizontal(|ui| {
                    ui.label("Angle:");
                    ui.add(egui::DragValue::new(&mut state.lead_config.angle_deg).speed(5.0).range(15.0..=90.0).suffix("°"));
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut state.lead_config.lead_type, LeadType::Line, "Straight");
                    ui.radio_value(&mut state.lead_config.lead_type, LeadType::Arc, "Arc");
                });
            });
        });

        ui.add_space(4.0);

        // 5. Transforms
        if !selection.is_empty() {
            ribbon_group(ui, "Transforms", |ui| {
                if ui.add(Button::new(RichText::new("🪞 Flip X").size(12.0)).fill(Color32::from_rgb(38, 42, 50))).on_hover_text("Mirror selection horizontally").clicked() {
                    history.record(doc, selection, "Mirror X");
                    doc.mirror_selection_horizontal(selection);
                }
                if ui.add(Button::new(RichText::new("🪞 Flip Y").size(12.0)).fill(Color32::from_rgb(38, 42, 50))).on_hover_text("Mirror selection vertically").clicked() {
                    history.record(doc, selection, "Mirror Y");
                    doc.mirror_selection_vertical(selection);
                }
            });
        }
    });
}

fn render_laser_tab(
    ui: &mut Ui,
    state: &mut RibbonState,
    controller: &mut Option<LaserController>,
    status: &MachineStatus,
    status_message: &mut String,
) {
    ui.horizontal(|ui| {
        // 1. Head Calibration (Capacitive Height Sensor)
        ribbon_group(ui, "Head Calibration (Capacitive)", |ui| {
            let cal_btn = Button::new(RichText::new("🎯 Calibrate Sensor").size(12.0).color(Color32::from_rgb(255, 215, 0)).strong())
                .fill(Color32::from_rgb(50, 42, 20));
            if ui.add(cal_btn).on_hover_text("Calibrate capacitive height sensor (measures capacitance vs distance curve on metal sheet)").clicked() {
                if let Some(ref c) = controller {
                    c.calibrate_height_sensor();
                    *status_message = "Height sensor calibration started. Head searching surface...".to_string();
                } else {
                    *status_message = "Controller offline. Connect controller before calibrating sensor.".to_string();
                }
            }

            let probe_btn = Button::new(RichText::new("⬇ Probe Surface (Z=0)").size(12.0).color(Color32::from_rgb(0, 220, 255)))
                .fill(Color32::from_rgb(24, 45, 60));
            if ui.add(probe_btn).on_hover_text("Lower head until nozzle touches sheet metal to zero Z coordinate").clicked() {
                if let Some(ref c) = controller {
                    c.probe_surface_z();
                    *status_message = "Probing surface... Z zero set on contact.".to_string();
                } else {
                    *status_message = "Controller offline.".to_string();
                }
            }

            let servo_bg = if state.follower_enabled {
                Color32::from_rgb(20, 70, 40)
            } else {
                Color32::from_rgb(38, 42, 50)
            };
            let servo_text = if state.follower_enabled {
                RichText::new("🛡 Follower ON").color(Color32::from_rgb(0, 255, 120)).size(12.0)
            } else {
                RichText::new("🛡 Follower OFF").color(Color32::WHITE).size(12.0)
            };
            if ui.add(Button::new(servo_text).fill(servo_bg)).on_hover_text("Toggle capacitive height tracking follower servo").clicked() {
                state.follower_enabled = !state.follower_enabled;
                if let Some(ref c) = controller {
                    c.set_follower_servo(state.follower_enabled);
                }
                *status_message = format!("Height follower servo: {}", if state.follower_enabled { "ENABLED" } else { "DISABLED" });
            }
        });

        ui.add_space(4.0);

        // 2. Optics & Test Pulse
        ribbon_group(ui, "Laser & Optics", |ui| {
            let red_bg = if state.red_pointer_enabled {
                Color32::from_rgb(80, 25, 25)
            } else {
                Color32::from_rgb(38, 42, 50)
            };
            let red_text = if state.red_pointer_enabled {
                RichText::new("🔴 Red Guide ON").color(Color32::from_rgb(255, 100, 100)).size(12.0)
            } else {
                RichText::new("🔴 Red Guide OFF").size(12.0)
            };
            if ui.add(Button::new(red_text).fill(red_bg)).on_hover_text("Toggle 650nm visible red laser guide pointer").clicked() {
                state.red_pointer_enabled = !state.red_pointer_enabled;
                if let Some(ref c) = controller {
                    c.set_red_pointer(state.red_pointer_enabled);
                }
                *status_message = format!("Red pointer: {}", if state.red_pointer_enabled { "ON" } else { "OFF" });
            }

            let pulse_btn = Button::new(RichText::new(format!("⚡ Pulse {}ms", state.pulse_duration_ms)).size(12.0).color(Color32::from_rgb(255, 230, 80)))
                .fill(Color32::from_rgb(48, 42, 24));
            if ui.add(pulse_btn).on_hover_text(format!("Fire laser test burst: {}ms at {:.0}% power", state.pulse_duration_ms, state.pulse_power_pct)).clicked() {
                if let Some(ref c) = controller {
                    c.laser_pulse(state.pulse_duration_ms, state.pulse_power_pct);
                    *status_message = format!("Test pulse fired: {}ms @ {:.0}% power.", state.pulse_duration_ms, state.pulse_power_pct);
                } else {
                    *status_message = "Controller offline.".to_string();
                }
            }

            let gas_bg = if state.gas_open {
                Color32::from_rgb(25, 60, 85)
            } else {
                Color32::from_rgb(38, 42, 50)
            };
            let gas_text = if state.gas_open {
                RichText::new("💨 Gas ON").color(Color32::from_rgb(0, 220, 255)).size(12.0)
            } else {
                RichText::new("💨 Gas Purge").size(12.0)
            };
            if ui.add(Button::new(gas_text).fill(gas_bg)).on_hover_text("Open assist gas valve to purge line").clicked() {
                state.gas_open = !state.gas_open;
                if let Some(ref c) = controller {
                    c.set_gas(3, state.gas_open); // 3 = Air
                }
                *status_message = format!("Assist gas valve: {}", if state.gas_open { "OPEN (Purging)" } else { "CLOSED" });
            }
        });

        ui.add_space(4.0);

        // 3. Work Origin & Framing
        ribbon_group(ui, "Origin & Framing", |ui| {
            if ui.add(Button::new(RichText::new("📍 Set Origin").size(12.0).color(Color32::from_rgb(0, 230, 230))).fill(Color32::from_rgb(20, 48, 55))).on_hover_text("Zero Workpiece Origin (G54 X0 Y0) at current head position").clicked() {
                *status_message = format!("Workpiece Origin set at current position (X: {}, Y: {})", status.pos_x, status.pos_y);
            }

            let frame_btn = Button::new(RichText::new("🔲 Frame Job").size(12.0).color(Color32::from_rgb(255, 200, 50)))
                .fill(Color32::from_rgb(45, 40, 20));
            if ui.add(frame_btn).on_hover_text("Dry-run trace the bounding box with red pointer (laser off) to verify sheet positioning").clicked() {
                *status_message = "Framing boundary rectangle with red guide pointer...".to_string();
            }

            let estop_btn = Button::new(RichText::new("🛑 E-STOP").size(12.0).color(Color32::WHITE).strong())
                .fill(Color32::from_rgb(180, 30, 30));
            if ui.add(estop_btn).on_hover_text("IMMEDIATE EMERGENCY STOP - Halts all motion and turns laser OFF").clicked() {
                if let Some(ref c) = controller {
                    c.stop();
                }
                *status_message = "EMERGENCY STOP TRIGGERED. All motion halted.".to_string();
            }
        });
    });
}
