use egui::{Button, Color32, Grid, RichText, Ui};
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
    ui.heading("Machine Control");
    ui.separator();

    // 1. Connection Status Badge
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

    // 2. Position Readouts
    let pos_x_mm = pulses_cfg.pulses_to_mm_x(status.pos_x);
    let pos_y_mm = pulses_cfg.pulses_to_mm_y(status.pos_y);
    let pos_z_mm = pulses_cfg.pulses_to_mm_x(status.pos_z);

    egui::Frame::group(ui.style()).show(ui, |ui| {
        Grid::new("dro_grid").num_columns(2).spacing([20.0, 4.0]).show(ui, |ui| {
            ui.label(RichText::new("X:").strong());
            ui.monospace(format!("{:>9.3} mm", pos_x_mm));
            ui.end_row();

            ui.label(RichText::new("Y:").strong());
            ui.monospace(format!("{:>9.3} mm", pos_y_mm));
            ui.end_row();

            ui.label(RichText::new("Z:").strong());
            ui.monospace(format!("{:>9.3} mm", pos_z_mm));
            ui.end_row();
        });
    });

    ui.add_space(12.0);

    // 3. Jog Mode & Step Selection
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

    // 4. Jog Control Pad (X-, X+, Y-, Y+, Z-, Z+)
    let step_dist = match panel_state.jog_mode {
        JogMode::Step(d) => d,
        JogMode::Continuous => 10.0,
    };
    let step_pulses_x = pulses_cfg.mm_to_pulses_x(step_dist);
    let step_pulses_y = pulses_cfg.mm_to_pulses_y(step_dist);

    ui.vertical_centered(|ui| {
        // Row 1: Y+
        if ui.add_sized([70.0, 40.0], Button::new(RichText::new("Y+ ▲").size(15.0))).clicked() {
            if let Some(c) = controller {
                c.jog(Axis::Y, step_pulses_y);
            }
        }

        // Row 2: X- | STOP | X+
        ui.horizontal(|ui| {
            ui.add_space((ui.available_width() - (70.0 * 3.0 + 16.0)).max(0.0) / 2.0);
            if ui.add_sized([70.0, 40.0], Button::new(RichText::new("◄ X-").size(15.0))).clicked() {
                if let Some(c) = controller {
                    c.jog(Axis::X, -step_pulses_x);
                }
            }
            if ui.add_sized([70.0, 40.0], Button::new(RichText::new("STOP").color(Color32::WHITE)).fill(Color32::from_rgb(180, 40, 40))).clicked() {
                if let Some(c) = controller {
                    c.stop();
                }
            }
            if ui.add_sized([70.0, 40.0], Button::new(RichText::new("X+ ►").size(15.0))).clicked() {
                if let Some(c) = controller {
                    c.jog(Axis::X, step_pulses_x);
                }
            }
        });

        // Row 3: Y-
        if ui.add_sized([70.0, 40.0], Button::new(RichText::new("Y- ▼").size(15.0))).clicked() {
            if let Some(c) = controller {
                c.jog(Axis::Y, -step_pulses_y);
            }
        }
    });

    ui.add_space(16.0);
    ui.separator();

    // 5. Layer Cutting Parameters
    ui.horizontal(|ui| {
        ui.heading("Cutting Layers");
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.button("⚡ Auto-Classify").on_hover_text("Auto-detect Outer Profiles, Interior Holes, and Bend lines").clicked() {
                doc.auto_classify_profiles();
            }
        });
    });
    for layer in &mut doc.layers {
        ui.group(|ui| {
            ui.horizontal(|ui| {
                ui.checkbox(&mut layer.visible, "");
                ui.colored_label(
                    Color32::from_rgb(layer.color.r, layer.color.g, layer.color.b),
                    "■",
                );
                ui.label(RichText::new(&layer.name).strong());
                ui.label(format!("({} shapes)", layer.entities.len()));
            });

            Grid::new(format!("layer_grid_{}", layer.name)).num_columns(2).show(ui, |ui| {
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
}

