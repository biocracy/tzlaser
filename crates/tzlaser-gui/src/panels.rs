use egui::{Button, Color32, Frame, Grid, RichText, ScrollArea, Ui};
use tzlaser_core::{Document, PulseConfig};
use tzlaser_protocol::{Axis, LaserController, MachineStatus};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JogMode {
    Step(f64), // mm
    Continuous,
}

pub struct PanelState {
    pub jog_mode: JogMode,
    pub step_index: usize,
    pub target_ip: String,
}

impl Default for PanelState {
    fn default() -> Self {
        Self {
            jog_mode: JogMode::Step(10.0),
            step_index: 2, // 10mm
            target_ip: "10.1.1.168:502".to_string(),
        }
    }
}

pub fn render_right_panels(
    ui: &mut Ui,
    panel_state: &mut PanelState,
    controller: &Option<LaserController>,
    status: &MachineStatus,
    pulses_cfg: &PulseConfig,
    doc: &mut Document,
) {
    ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
        // ==========================================
        // 1. MONITOR PANEL (Head Position, Laser, Gas Valve, Capacitive Sensor)
        // ==========================================
        ui.heading("📊 Monitor");
        ui.separator();

        // 1A. Head Position (DRO)
        let pos_x_mm = pulses_cfg.pulses_to_mm_x(status.pos_x);
        let pos_y_mm = pulses_cfg.pulses_to_mm_y(status.pos_y);
        let pos_z_mm = pulses_cfg.pulses_to_mm_x(status.pos_z);

        Frame::group(ui.style()).fill(Color32::from_rgb(18, 20, 24)).show(ui, |ui| {
            ui.label(RichText::new("Head Coordinates (WCS)").size(11.0).color(Color32::from_rgb(140, 150, 170)).strong());
            ui.add_space(3.0);
            Grid::new("dro_grid").num_columns(3).spacing([14.0, 4.0]).show(ui, |ui| {
                ui.label(RichText::new("X:").color(Color32::from_rgb(255, 90, 90)).strong().size(14.0));
                ui.monospace(RichText::new(format!("{:>9.3} mm", pos_x_mm)).size(13.5).strong().color(Color32::WHITE));
                ui.label(RichText::new("WCS X").size(10.0).weak());
                ui.end_row();

                ui.label(RichText::new("Y:").color(Color32::from_rgb(90, 220, 90)).strong().size(14.0));
                ui.monospace(RichText::new(format!("{:>9.3} mm", pos_y_mm)).size(13.5).strong().color(Color32::WHITE));
                ui.label(RichText::new("WCS Y").size(10.0).weak());
                ui.end_row();

                ui.label(RichText::new("Z:").color(Color32::from_rgb(90, 160, 255)).strong().size(14.0));
                ui.monospace(RichText::new(format!("{:>9.3} mm", pos_z_mm)).size(13.5).strong().color(Color32::WHITE));
                ui.label(RichText::new("Height").size(10.0).weak());
                ui.end_row();
            });
        });

        ui.add_space(6.0);

        // 1B. Laser Source Status
        Frame::group(ui.style()).fill(Color32::from_rgb(18, 20, 24)).show(ui, |ui| {
            ui.label(RichText::new("Laser Source").size(11.0).color(Color32::from_rgb(140, 150, 170)).strong());
            ui.add_space(3.0);
            Grid::new("laser_monitor_grid").num_columns(2).spacing([16.0, 3.0]).show(ui, |ui| {
                ui.label("Emission:");
                if status.laser_on {
                    ui.colored_label(Color32::from_rgb(255, 60, 60), "● ACTIVE / FIRING");
                } else {
                    ui.colored_label(Color32::from_rgb(140, 145, 155), "○ READY / STANDBY");
                }
                ui.end_row();

                ui.label("Power / Temp:");
                ui.monospace(format!("{:.1}%  |  {:.1} °C", status.laser_power_pct, status.laser_temp_c));
                ui.end_row();

                ui.label("Red Guide Pointer:");
                if status.red_pointer_on {
                    ui.colored_label(Color32::from_rgb(255, 90, 90), "● 650nm ON");
                } else {
                    ui.label("OFF");
                }
                ui.end_row();
            });
        });

        ui.add_space(6.0);

        // 1C. Assist Gas Solenoid Valve
        Frame::group(ui.style()).fill(Color32::from_rgb(18, 20, 24)).show(ui, |ui| {
            ui.label(RichText::new("Assist Gas Solenoid").size(11.0).color(Color32::from_rgb(140, 150, 170)).strong());
            ui.add_space(3.0);
            Grid::new("gas_monitor_grid").num_columns(2).spacing([16.0, 3.0]).show(ui, |ui| {
                ui.label("Valve Status:");
                if status.gas_valve_open {
                    ui.colored_label(Color32::from_rgb(0, 220, 255), "● OPEN (Purging)");
                } else {
                    ui.colored_label(Color32::from_rgb(140, 145, 155), "○ CLOSED");
                }
                ui.end_row();

                ui.label("Gas Feed:");
                ui.label(RichText::new(&status.gas_type).strong());
                ui.end_row();

                ui.label("Line Pressure:");
                ui.monospace(format!("{:.1} bar", status.gas_pressure_bar));
                ui.end_row();
            });
        });

        ui.add_space(6.0);

        // 1D. Capacitive Height Sensor Readout (BCS100 Follower)
        Frame::group(ui.style()).fill(Color32::from_rgb(18, 20, 24)).show(ui, |ui| {
            ui.label(RichText::new("Capacitive Height Sensor").size(11.0).color(Color32::from_rgb(140, 150, 170)).strong());
            ui.add_space(3.0);
            Grid::new("height_sensor_grid").num_columns(2).spacing([16.0, 3.0]).show(ui, |ui| {
                ui.label("Nozzle Height:");
                ui.monospace(RichText::new(format!("{:.2} mm", status.capacitive_height_mm)).strong().color(Color32::from_rgb(255, 215, 0)));
                ui.end_row();

                ui.label("Capacitance:");
                ui.monospace(format!("{:.2} pF", status.capacitance_pf));
                ui.end_row();

                ui.label("Follower Servo:");
                if status.follower_active {
                    ui.colored_label(Color32::from_rgb(0, 255, 120), "● TRACKING (Locked)");
                } else {
                    ui.colored_label(Color32::from_rgb(140, 145, 155), "○ STANDBY");
                }
                ui.end_row();
            });
        });

        ui.add_space(14.0);
        ui.separator();

        // ==========================================
        // 2. MACHINE CONTROLS (Jogging, Connection, Motion Pad)
        // ==========================================
        ui.heading("🎮 Machine Controls");
        ui.separator();

        // Connection Status Badge
        let is_connected = controller.as_ref().map_or(false, |c| c.is_connected());
        ui.horizontal(|ui| {
            let (dot_color, label) = if is_connected {
                (Color32::GREEN, "ONLINE")
            } else {
                (Color32::RED, "OFFLINE")
            };
            ui.colored_label(dot_color, "●");
            ui.label(RichText::new(label).strong());
            ui.label(format!("({})", panel_state.target_ip));
        });

        ui.add_space(8.0);

        // Jog Mode & Step Selection
        ui.horizontal(|ui| {
            ui.label(RichText::new("Jog Mode:").strong());
            let is_step = matches!(panel_state.jog_mode, JogMode::Step(_));
            if ui.selectable_label(is_step, "Step").clicked() {
                let step_options = [0.1, 1.0, 10.0, 50.0, 100.0];
                panel_state.jog_mode = JogMode::Step(step_options[panel_state.step_index.min(step_options.len() - 1)]);
            }
            let is_cont = matches!(panel_state.jog_mode, JogMode::Continuous);
            if ui.selectable_label(is_cont, "Continuous").clicked() {
                panel_state.jog_mode = JogMode::Continuous;
            }
        });

        if let JogMode::Step(_) = panel_state.jog_mode {
            let step_options = [0.1, 1.0, 10.0, 50.0, 100.0];
            ui.horizontal_wrapped(|ui| {
                for (i, &step) in step_options.iter().enumerate() {
                    let selected = panel_state.step_index == i;
                    if ui.selectable_label(selected, format!("{}mm", step)).clicked() {
                        panel_state.step_index = i;
                        panel_state.jog_mode = JogMode::Step(step);
                    }
                }
            });
        }

        ui.add_space(8.0);

        // Jog Control Pad (X-, X+, Y-, Y+, STOP)
        let step_dist = match panel_state.jog_mode {
            JogMode::Step(d) => d,
            JogMode::Continuous => 10.0,
        };
        let step_pulses_x = pulses_cfg.mm_to_pulses_x(step_dist);
        let step_pulses_y = pulses_cfg.mm_to_pulses_y(step_dist);

        ui.vertical_centered(|ui| {
            // Row 1: Y+
            if ui.add_sized([70.0, 36.0], Button::new(RichText::new("Y+ ▲").size(14.0))).clicked() {
                if let Some(c) = controller {
                    c.jog(Axis::Y, step_pulses_y);
                }
            }

            // Row 2: X- | STOP | X+
            ui.horizontal(|ui| {
                ui.add_space((ui.available_width() - (70.0 * 3.0 + 16.0)).max(0.0) / 2.0);
                if ui.add_sized([70.0, 36.0], Button::new(RichText::new("◄ X-").size(14.0))).clicked() {
                    if let Some(c) = controller {
                        c.jog(Axis::X, -step_pulses_x);
                    }
                }
                if ui.add_sized([70.0, 36.0], Button::new(RichText::new("STOP").color(Color32::WHITE).strong()).fill(Color32::from_rgb(180, 40, 40))).clicked() {
                    if let Some(c) = controller {
                        c.stop();
                    }
                }
                if ui.add_sized([70.0, 36.0], Button::new(RichText::new("X+ ►").size(14.0))).clicked() {
                    if let Some(c) = controller {
                        c.jog(Axis::X, step_pulses_x);
                    }
                }
            });

            // Row 3: Y-
            if ui.add_sized([70.0, 36.0], Button::new(RichText::new("Y- ▼").size(14.0))).clicked() {
                if let Some(c) = controller {
                    c.jog(Axis::Y, -step_pulses_y);
                }
            }
        });

        ui.add_space(14.0);
        ui.separator();

        // ==========================================
        // 3. CUTTING LAYERS (Layer parameters, speed, power, pierce)
        // ==========================================
        ui.heading("🎨 Cutting Layers");
        ui.separator();

        for layer in &mut doc.layers {
            ui.group(|ui| {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut layer.visible, "");
                    ui.colored_label(
                        Color32::from_rgb(layer.color.r, layer.color.g, layer.color.b),
                        "■",
                    );
                    ui.label(RichText::new(&layer.name).strong());
                    ui.label(format!("({} profiles)", layer.entities.len()));
                });

                Grid::new(format!("layer_grid_{}", layer.name)).num_columns(2).spacing([12.0, 3.0]).show(ui, |ui| {
                    ui.label("Speed:");
                    ui.add(egui::DragValue::new(&mut layer.params.speed_mm_s).speed(5.0).suffix(" mm/s"));
                    ui.end_row();

                    ui.label("Power:");
                    ui.add(egui::DragValue::new(&mut layer.params.power_percent).range(0.0..=100.0).suffix(" %"));
                    ui.end_row();

                    ui.label("Pierce:");
                    ui.add(egui::DragValue::new(&mut layer.params.pierce_time_ms).suffix(" ms"));
                    ui.end_row();
                });
            });
        }
    });
}
