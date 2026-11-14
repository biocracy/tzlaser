use eframe::egui::{
    self, Button, Color32, ComboBox, Context, DragValue, Frame, Grid, RichText, TextEdit, Vec2,
    Window,
};
use tzlaser_core::PulseConfig;
use tzlaser_protocol::{JogConfig, LaserController};

use crate::canvas::CanvasState;
use crate::panels::{JogMode, PanelState};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibAxisTarget {
    XY,
    X,
    Y,
    Z,
}

pub struct PropertiesWindowState {
    pub open: bool,
    pub calib_target: CalibAxisTarget,
    pub calib_commanded_mm: f64,
    pub calib_measured_mm: f64,
}

impl Default for PropertiesWindowState {
    fn default() -> Self {
        Self {
            open: false,
            calib_target: CalibAxisTarget::XY,
            calib_commanded_mm: 10.0,
            calib_measured_mm: 10.0,
        }
    }
}

/// Apply updated `PulseConfig` values to the live canvas, panel state, controller, and persist to disk.
pub fn sync_and_save_properties(
    pulse_config: &PulseConfig,
    canvas_state: &mut CanvasState,
    panel_state: &mut PanelState,
    controller: &Option<LaserController>,
    status_message: &mut String,
) {
    canvas_state.bed_width_mm = pulse_config.table_width_mm.max(10.0) as f32;
    canvas_state.bed_height_mm = pulse_config.table_height_mm.max(10.0) as f32;
    panel_state.target_ip = pulse_config.controller_ip.clone();

    if let Some(ref c) = controller {
        let (min_speed, jog_speed, accel) = pulse_config.jog_pulses_params();
        c.update_jog_config(
            JogConfig {
                min_speed,
                jog_speed,
                accel,
            },
            pulse_config.z_jog_speed_reg,
        );
    }

    match pulse_config.save() {
        Ok(()) => {
            *status_message = format!(
                "Saved Table & Machine Properties (Table: {:.0}×{:.0} mm, X: {:.1} / Y: {:.1} / Z: {:.1} pls/mm).",
                pulse_config.table_width_mm,
                pulse_config.table_height_mm,
                pulse_config.pulses_per_mm_x,
                pulse_config.pulses_per_mm_y,
                pulse_config.pulses_per_mm_z,
            );
        }
        Err(e) => {
            *status_message = format!("Failed to save machine properties: {}", e);
        }
    }
}

pub fn render_properties_window(
    ctx: &Context,
    state: &mut PropertiesWindowState,
    pulse_config: &mut PulseConfig,
    canvas_state: &mut CanvasState,
    panel_state: &mut PanelState,
    controller: &Option<LaserController>,
    status_message: &mut String,
) {
    if !state.open {
        return;
    }

    let mut is_open = state.open;
    let mut changed = false;
    let mut close_clicked = false;

    let section_fill = if cfg!(target_os = "windows") {
        Color32::from_rgb(24, 28, 36)
    } else {
        Color32::from_rgb(20, 22, 27)
    };
    let section_header_color = Color32::from_rgb(0, 220, 255);

    Window::new(
        RichText::new("📐 Table & Machine Properties (SC2000 Setup)")
            .color(Color32::from_rgb(0, 225, 255))
            .strong(),
    )
    .open(&mut is_open)
    .default_pos([300.0, 90.0])
    .default_width(470.0)
    .resizable(false)
    .collapsible(false)
    .show(ctx, |ui| {
        ui.set_min_width(450.0);

        ui.horizontal(|ui| {
            ui.label(
                RichText::new("● Auto-Saved on Edit")
                    .size(11.0)
                    .color(Color32::from_rgb(90, 235, 135))
                    .strong(),
            );
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .add(
                        Button::new(
                            RichText::new("📋 SC2000 Config (590 / 1000 pls/mm)")
                                .size(11.0)
                                .color(Color32::from_rgb(255, 220, 100)),
                        )
                        .fill(Color32::from_rgb(48, 42, 24)),
                    )
                    .on_hover_text(
                        "Apply values from SC2000 BkHardPara.xml & capture (2000×3000 mm, 590 pls/mm XY, 1000 pls/mm Z, 10 mm/s jog)",
                    )
                    .clicked()
                {
                    pulse_config.table_width_mm = 2000.0;
                    pulse_config.table_height_mm = 3000.0;
                    pulse_config.pulses_per_mm_x = 590.0;
                    pulse_config.pulses_per_mm_y = 590.0;
                    pulse_config.pulses_per_mm_z = 1_000.0;
                    pulse_config.jog_start_speed_mm_s = 10.0;
                    pulse_config.jog_speed_mm_s = 10.0;
                    pulse_config.jog_accel_mm_s2 = 100.0;
                    pulse_config.z_jog_speed_reg = 50;
                    changed = true;
                }

                if ui
                    .add(
                        Button::new(
                            RichText::new("🛡 Conservative Safe")
                                .size(11.0)
                                .color(Color32::from_rgb(130, 235, 175)),
                        )
                        .fill(Color32::from_rgb(24, 52, 38)),
                    )
                    .on_hover_text(
                        "Apply conservative low-displacement, low-speed settings for safe initial testing (200 pls/mm XY, 1000 pls/mm Z)",
                    )
                    .clicked()
                {
                    pulse_config.pulses_per_mm_x = 200.0;
                    pulse_config.pulses_per_mm_y = 200.0;
                    pulse_config.pulses_per_mm_z = 1_000.0;
                    pulse_config.jog_start_speed_mm_s = 5.0;
                    pulse_config.jog_speed_mm_s = 10.0;
                    pulse_config.jog_accel_mm_s2 = 80.0;
                    pulse_config.z_jog_speed_reg = 50;
                    pulse_config.default_step_mm = 1.0;
                    panel_state.jog_mode = JogMode::Step(1.0);
                    panel_state.step_index = 2;
                    changed = true;
                }
            });
        });

        ui.add_space(4.0);

        // =====================================================================
        // 1. TABLE & WORKING ENVELOPE
        // =====================================================================
        Frame::group(ui.style()).fill(section_fill).show(ui, |ui| {
            ui.label(
                RichText::new("1. Table & Working Area Dimensions")
                    .size(12.0)
                    .color(section_header_color)
                    .strong(),
            );
            ui.add_space(4.0);

            Grid::new("props_table_grid")
                .num_columns(4)
                .spacing([12.0, 6.0])
                .show(ui, |ui| {
                    ui.label("Table Width (X):");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.table_width_mm)
                                .range(50.0..=10_000.0)
                                .speed(5.0)
                                .suffix(" mm"),
                        )
                        .changed()
                    {
                        changed = true;
                    }

                    ui.label("Table Length (Y):");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.table_height_mm)
                                .range(50.0..=10_000.0)
                                .speed(5.0)
                                .suffix(" mm"),
                        )
                        .changed()
                    {
                        changed = true;
                    }
                    ui.end_row();

                    ui.label("Z Max Travel:");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.z_max_travel_mm)
                                .range(10.0..=1000.0)
                                .speed(1.0)
                                .suffix(" mm"),
                        )
                        .changed()
                    {
                        changed = true;
                    }

                    ui.label("Origin Corner:");
                    let origins = [
                        "Bottom-Left (X0, Y0)",
                        "Top-Left (X0, Ymax)",
                        "Bottom-Right (Xmax, Y0)",
                        "Top-Right (Xmax, Ymax)",
                    ];
                    ComboBox::from_id_salt("props_origin_corner_combo")
                        .selected_text(&pulse_config.origin_corner)
                        .width(145.0)
                        .show_ui(ui, |ui| {
                            for orig in origins {
                                if ui
                                    .selectable_value(
                                        &mut pulse_config.origin_corner,
                                        orig.to_string(),
                                        orig,
                                    )
                                    .changed()
                                {
                                    changed = true;
                                }
                            }
                        });
                    ui.end_row();
                });

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui
                    .checkbox(
                        &mut pulse_config.soft_limits_enabled,
                        "Enable Machine Soft Limits",
                    )
                    .changed()
                {
                    changed = true;
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    for (w, h, lbl) in [
                        (4000.0, 2000.0, "4000×2000"),
                        (3000.0, 1500.0, "3000×1500"),
                        (2000.0, 3000.0, "2000×3000"),
                        (1300.0, 900.0, "1300×900"),
                    ] {
                        if ui.small_button(lbl).clicked() {
                            pulse_config.table_width_mm = w;
                            pulse_config.table_height_mm = h;
                            changed = true;
                        }
                    }
                    ui.label(RichText::new("Presets:").size(10.5).weak());
                });
            });
        });

        ui.add_space(6.0);

        // =====================================================================
        // 2. AXIS PULSE CALIBRATION (PULSES PER MM)
        // =====================================================================
        Frame::group(ui.style()).fill(section_fill).show(ui, |ui| {
            ui.label(
                RichText::new("2. Axis Pulse Calibration (SC2000 Pulses / mm)")
                    .size(12.0)
                    .color(section_header_color)
                    .strong(),
            );
            ui.add_space(4.0);

            Grid::new("props_pulses_grid")
                .num_columns(4)
                .spacing([14.0, 6.0])
                .show(ui, |ui| {
                    ui.label(
                        RichText::new("X-Axis:")
                            .color(Color32::from_rgb(255, 100, 100))
                            .strong(),
                    );
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.pulses_per_mm_x)
                                .range(0.1..=100_000.0)
                                .speed(1.0)
                                .max_decimals(3)
                                .suffix(" pls/mm"),
                        )
                        .changed()
                    {
                        changed = true;
                    }
                    if ui
                        .checkbox(&mut pulse_config.invert_x, "Invert X Dir")
                        .changed()
                    {
                        changed = true;
                    }
                    ui.label(
                        RichText::new(format!(
                            "1 mm = {:.0} pls",
                            pulse_config.pulses_per_mm_x
                        ))
                        .size(10.5)
                        .weak(),
                    );
                    ui.end_row();

                    ui.label(
                        RichText::new("Y-Axis:")
                            .color(Color32::from_rgb(100, 230, 100))
                            .strong(),
                    );
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.pulses_per_mm_y)
                                .range(0.1..=100_000.0)
                                .speed(1.0)
                                .max_decimals(3)
                                .suffix(" pls/mm"),
                        )
                        .changed()
                    {
                        changed = true;
                    }
                    if ui
                        .checkbox(&mut pulse_config.invert_y, "Invert Y Dir")
                        .changed()
                    {
                        changed = true;
                    }
                    ui.label(
                        RichText::new(format!(
                            "1 mm = {:.0} pls",
                            pulse_config.pulses_per_mm_y
                        ))
                        .size(10.5)
                        .weak(),
                    );
                    ui.end_row();

                    ui.label(
                        RichText::new("Z-Axis:")
                            .color(Color32::from_rgb(110, 175, 255))
                            .strong(),
                    );
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.pulses_per_mm_z)
                                .range(0.1..=100_000.0)
                                .speed(1.0)
                                .max_decimals(3)
                                .suffix(" pls/mm"),
                        )
                        .changed()
                    {
                        changed = true;
                    }
                    if ui
                        .checkbox(&mut pulse_config.invert_z, "Invert Z Dir")
                        .changed()
                    {
                        changed = true;
                    }
                    ui.label(
                        RichText::new(format!(
                            "1 mm = {:.0} pls",
                            pulse_config.pulses_per_mm_z
                        ))
                        .size(10.5)
                        .weak(),
                    );
                    ui.end_row();

                    ui.label(RichText::new("W-Rotary:").strong());
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.pulses_per_deg_w)
                                .range(0.1..=100_000.0)
                                .speed(1.0)
                                .max_decimals(3)
                                .suffix(" pls/°"),
                        )
                        .changed()
                    {
                        changed = true;
                    }
                    ui.label("");
                    ui.label("");
                    ui.end_row();
                });

            ui.add_space(4.0);
            ui.collapsing(
                RichText::new("📏 Step Displacement Calibration Calculator").size(11.5),
                |ui| {
                    ui.label(
                        RichText::new(
                            "Jog a known step in Ţlaser, measure real head travel (or read SC2000), and compute exact pulses/mm:",
                        )
                        .size(10.5)
                        .weak(),
                    );
                    ui.horizontal(|ui| {
                        ui.label("Axis:");
                        ui.selectable_value(&mut state.calib_target, CalibAxisTarget::XY, "X & Y");
                        ui.selectable_value(&mut state.calib_target, CalibAxisTarget::X, "X");
                        ui.selectable_value(&mut state.calib_target, CalibAxisTarget::Y, "Y");
                        ui.selectable_value(&mut state.calib_target, CalibAxisTarget::Z, "Z");
                    });
                    ui.horizontal(|ui| {
                        ui.label("Window Step:");
                        ui.add(
                            DragValue::new(&mut state.calib_commanded_mm)
                                .range(0.01..=1000.0)
                                .speed(0.5)
                                .suffix(" mm"),
                        );
                        ui.label("Actual Head Move:");
                        ui.add(
                            DragValue::new(&mut state.calib_measured_mm)
                                .range(0.01..=1000.0)
                                .speed(0.5)
                                .suffix(" mm"),
                        );

                        let ratio =
                            state.calib_commanded_mm.max(0.001) / state.calib_measured_mm.max(0.001);
                        if ui
                            .add(
                                Button::new(
                                    RichText::new("✓ Apply Scale")
                                        .size(11.0)
                                        .color(Color32::WHITE)
                                        .strong(),
                                )
                                .fill(Color32::from_rgb(28, 105, 165)),
                            )
                            .clicked()
                        {
                            match state.calib_target {
                                CalibAxisTarget::XY => {
                                    pulse_config.pulses_per_mm_x =
                                        (pulse_config.pulses_per_mm_x * ratio).clamp(0.1, 100_000.0);
                                    pulse_config.pulses_per_mm_y =
                                        (pulse_config.pulses_per_mm_y * ratio).clamp(0.1, 100_000.0);
                                }
                                CalibAxisTarget::X => {
                                    pulse_config.pulses_per_mm_x =
                                        (pulse_config.pulses_per_mm_x * ratio).clamp(0.1, 100_000.0);
                                }
                                CalibAxisTarget::Y => {
                                    pulse_config.pulses_per_mm_y =
                                        (pulse_config.pulses_per_mm_y * ratio).clamp(0.1, 100_000.0);
                                }
                                CalibAxisTarget::Z => {
                                    pulse_config.pulses_per_mm_z =
                                        (pulse_config.pulses_per_mm_z * ratio).clamp(0.1, 100_000.0);
                                }
                            }
                            changed = true;
                        }
                    });
                },
            );
        });

        ui.add_space(6.0);

        // =====================================================================
        // 3. JOGGING & MOTION DYNAMICS
        // =====================================================================
        Frame::group(ui.style()).fill(section_fill).show(ui, |ui| {
            ui.label(
                RichText::new("3. Jogging Speeds & Motion Dynamics")
                    .size(12.0)
                    .color(section_header_color)
                    .strong(),
            );
            ui.add_space(4.0);

            Grid::new("props_jog_grid")
                .num_columns(4)
                .spacing([12.0, 6.0])
                .show(ui, |ui| {
                    ui.label("XY Start Speed:");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.jog_start_speed_mm_s)
                                .range(0.5..=500.0)
                                .speed(0.5)
                                .suffix(" mm/s"),
                        )
                        .changed()
                    {
                        changed = true;
                    }

                    ui.label("XY Jog Speed:");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.jog_speed_mm_s)
                                .range(0.5..=500.0)
                                .speed(0.5)
                                .suffix(" mm/s"),
                        )
                        .changed()
                    {
                        changed = true;
                    }
                    ui.end_row();

                    ui.label("XY Acceleration:");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.jog_accel_mm_s2)
                                .range(5.0..=5000.0)
                                .speed(5.0)
                                .suffix(" mm/s²"),
                        )
                        .changed()
                    {
                        changed = true;
                    }

                    ui.label("Z Speed (Port 999):");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.z_jog_speed_reg)
                                .range(1..=300)
                                .speed(1.0),
                        )
                        .changed()
                    {
                        changed = true;
                    }
                    ui.end_row();

                    ui.label("Default Step:");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.default_step_mm)
                                .range(0.05..=500.0)
                                .speed(0.2)
                                .suffix(" mm"),
                        )
                        .changed()
                    {
                        panel_state.jog_mode = JogMode::Step(pulse_config.default_step_mm);
                        changed = true;
                    }
                    ui.label("");
                    ui.label("");
                    ui.end_row();
                });

            let (vmin_p, vjog_p, acc_p) = pulse_config.jog_pulses_params();
            ui.add_space(2.0);
            ui.label(
                RichText::new(format!(
                    "Board Registers (Reg 116): vmin = {} pls/s  ·  vjog = {} pls/s  ·  accel = {} pls/s²",
                    vmin_p, vjog_p, acc_p
                ))
                .size(10.5)
                .weak(),
            );
        });

        ui.add_space(6.0);

        // =====================================================================
        // 4. NETWORK & CONTROLLER ENDPOINTS
        // =====================================================================
        Frame::group(ui.style()).fill(section_fill).show(ui, |ui| {
            ui.label(
                RichText::new("4. Controller Network Endpoints")
                    .size(12.0)
                    .color(section_header_color)
                    .strong(),
            );
            ui.add_space(4.0);

            ui.horizontal(|ui| {
                ui.label("XY Motion IP:Port:");
                if ui
                    .add(TextEdit::singleline(&mut pulse_config.controller_ip).desired_width(135.0))
                    .changed()
                {
                    changed = true;
                }
                ui.add_space(8.0);
                ui.label("Z-Follower Port:");
                if ui
                    .add(DragValue::new(&mut pulse_config.z_follower_port).range(1..=65535))
                    .changed()
                {
                    changed = true;
                }
            });
        });

        ui.add_space(6.0);
        ui.separator();

        // Footer with persisted file location & Close button
        ui.horizontal(|ui| {
            let path_str = pulse_config
                .config_path
                .as_ref()
                .map(|p| p.display().to_string())
                .unwrap_or_else(|| "machine_properties.json".to_string());
            ui.label(RichText::new(path_str).size(10.0).weak());

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let close_btn = Button::new(
                    RichText::new("✓ Done")
                        .size(12.0)
                        .color(Color32::WHITE)
                        .strong(),
                )
                .fill(Color32::from_rgb(30, 95, 160))
                .min_size(Vec2::new(80.0, 26.0));
                if ui.add(close_btn).clicked() {
                    close_clicked = true;
                }
            });
        });
    });

    if changed {
        sync_and_save_properties(
            pulse_config,
            canvas_state,
            panel_state,
            controller,
            status_message,
        );
    }

    if close_clicked || !is_open {
        state.open = false;
    }
}
