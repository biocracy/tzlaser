use std::time::{Duration, Instant};

use eframe::egui::{
    self, Button, Color32, ComboBox, Context, DragValue, Frame, Grid, RichText, TextEdit, Vec2,
    Window,
};
use tzlaser_core::PulseConfig;
use tzlaser_protocol::{Axis, JogConfig, LaserController, MachineStatus};

use crate::canvas::CanvasState;
use crate::panels::{JogMode, PanelState};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibAxisTarget {
    XY,
    X,
    Y,
    Z,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasureApplyChoice {
    Average,
    OriginToMax,
    MaxToOrigin,
    Custom,
}

impl MeasureApplyChoice {
    pub fn label(self) -> &'static str {
        match self {
            Self::Average => "AVG",
            Self::OriginToMax => "0->MX",
            Self::MaxToOrigin => "MX->0",
            Self::Custom => "Custom",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasurePhase {
    Confirm,
    /// If already on origin sensor at start, back off 5mm first before touching off.
    PreClearOrigin,
    SeekOrigin,
    SeekMax,
    RetractFromMax,
    RetouchMax,
    ReturnFast,
    ReturnSlow,
    Done,
    Stopped,
}

pub struct MeasureAxisState {
    pub axis: Axis,
    pub phase: MeasurePhase,
    pub slow_speed_mm_s: f64,
    pub fast_return_speed_mm_s: f64,
    pub phase_entered_at: Instant,
    pub motion_seen: bool,
    pub phase_start_pulses: i32,
    pub expected_move_mm: f64,
    pub origin_pos_1: i32,
    pub max_pos_1: i32,
    pub max_pos_2: i32,
    pub origin_pos_2: i32,
    pub meas_origin_to_max_mm: Option<f64>,
    pub meas_max_to_origin_mm: Option<f64>,
    pub apply_choice: MeasureApplyChoice,
    pub custom_mm: f64,
    pub status_note: String,
}

impl MeasureAxisState {
    pub fn new(axis: Axis, initial_table_mm: f64) -> Self {
        Self {
            axis,
            phase: MeasurePhase::Confirm,
            slow_speed_mm_s: 5.0,
            fast_return_speed_mm_s: 10.0,
            phase_entered_at: Instant::now(),
            motion_seen: false,
            phase_start_pulses: 0,
            expected_move_mm: 0.0,
            origin_pos_1: 0,
            max_pos_1: 0,
            max_pos_2: 0,
            origin_pos_2: 0,
            meas_origin_to_max_mm: None,
            meas_max_to_origin_mm: None,
            apply_choice: MeasureApplyChoice::Average,
            custom_mm: initial_table_mm,
            status_note: "Ready to measure physical table travel.".to_string(),
        }
    }

    pub fn is_running(&self) -> bool {
        matches!(
            self.phase,
            MeasurePhase::PreClearOrigin
                | MeasurePhase::SeekOrigin
                | MeasurePhase::SeekMax
                | MeasurePhase::RetractFromMax
                | MeasurePhase::RetouchMax
                | MeasurePhase::ReturnFast
                | MeasurePhase::ReturnSlow
        )
    }

    pub fn avg_mm(&self) -> Option<f64> {
        match (self.meas_origin_to_max_mm, self.meas_max_to_origin_mm) {
            (Some(a), Some(b)) => Some(0.5 * (a + b)),
            _ => None,
        }
    }

    pub fn selected_apply_mm(&self) -> Option<f64> {
        match self.apply_choice {
            MeasureApplyChoice::Average => self.avg_mm(),
            MeasureApplyChoice::OriginToMax => self.meas_origin_to_max_mm,
            MeasureApplyChoice::MaxToOrigin => self.meas_max_to_origin_mm,
            MeasureApplyChoice::Custom => Some(self.custom_mm),
        }
    }
}

pub struct AxisPulseCalibState {
    pub axis: Axis,
    pub step_mm: f64,
    pub commanded_pulses_total: i32,
    pub commanded_signed_mm: f64,
    pub micrometer_reading_mm: f64,
}

impl AxisPulseCalibState {
    pub fn new(axis: Axis) -> Self {
        Self {
            axis,
            step_mm: 1.0,
            commanded_pulses_total: 0,
            commanded_signed_mm: 0.0,
            micrometer_reading_mm: 1.0,
        }
    }

    pub fn computed_pulses_per_unit(&self) -> Option<f64> {
        if self.commanded_pulses_total == 0 || self.micrometer_reading_mm.abs() <= 1e-4 {
            return None;
        }
        Some((self.commanded_pulses_total.abs() as f64) / self.micrometer_reading_mm.abs())
    }
}

pub struct PropertiesWindowState {
    pub open: bool,
    pub calib_target: CalibAxisTarget,
    pub calib_commanded_mm: f64,
    pub calib_measured_mm: f64,
    pub measure_axis: Option<MeasureAxisState>,
    pub calib_dialog: Option<AxisPulseCalibState>,
}

impl Default for PropertiesWindowState {
    fn default() -> Self {
        Self {
            open: false,
            calib_target: CalibAxisTarget::XY,
            calib_commanded_mm: 10.0,
            calib_measured_mm: 10.0,
            measure_axis: None,
            calib_dialog: None,
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
    canvas_state.bed_width_mm = pulse_config.cut_width_mm() as f32;
    canvas_state.bed_height_mm = pulse_config.cut_height_mm() as f32;
    canvas_state.origin_corner = pulse_config.origin_corner.clone();
    canvas_state.home_retract_x_mm = pulse_config.home_retract_x_mm.max(0.0);
    canvas_state.home_retract_y_mm = pulse_config.home_retract_y_mm.max(0.0);
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
    status: &MachineStatus,
    status_message: &mut String,
) {
    canvas_state.bed_width_mm = pulse_config.cut_width_mm() as f32;
    canvas_state.bed_height_mm = pulse_config.cut_height_mm() as f32;
    canvas_state.origin_corner = pulse_config.origin_corner.clone();
    canvas_state.home_retract_x_mm = pulse_config.home_retract_x_mm.max(0.0);
    canvas_state.home_retract_y_mm = pulse_config.home_retract_y_mm.max(0.0);

    let mut measure_applied = false;
    if state.measure_axis.is_some() {
        measure_applied = render_measure_axis_dialog(
            ctx,
            state,
            pulse_config,
            controller,
            status,
            status_message,
        );
    }
    let mut calib_applied = false;
    if state.calib_dialog.is_some() {
        calib_applied = render_axis_calib_dialog(
            ctx,
            state,
            pulse_config,
            controller,
            status_message,
        );
    }
    if measure_applied || calib_applied {
        sync_and_save_properties(
            pulse_config,
            canvas_state,
            panel_state,
            controller,
            status_message,
        );
    }

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
                    ui.horizontal(|ui| {
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
                        if ui
                            .add(
                                Button::new(RichText::new("📏 Measure X").size(11.0).strong())
                                    .fill(Color32::from_rgb(30, 75, 115)),
                            )
                            .on_hover_text("Measure physical X-axis table length between origin and max limit sensors")
                            .clicked()
                        {
                            state.measure_axis = Some(MeasureAxisState::new(
                                Axis::X,
                                pulse_config.table_width_mm,
                            ));
                        }
                    });

                    ui.label("Table Length (Y):");
                    ui.horizontal(|ui| {
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
                        if ui
                            .add(
                                Button::new(RichText::new("📏 Measure Y").size(11.0).strong())
                                    .fill(Color32::from_rgb(30, 75, 115)),
                            )
                            .on_hover_text("Measure physical Y-axis table length between origin and max limit sensors")
                            .clicked()
                        {
                            state.measure_axis = Some(MeasureAxisState::new(
                                Axis::Y,
                                pulse_config.table_height_mm,
                            ));
                        }
                    });
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
                    ui.end_row();

                    ui.label("X Home Retract:");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.home_retract_x_mm)
                                .range(0.0..=500.0)
                                .speed(0.5)
                                .suffix(" mm"),
                        )
                        .on_hover_text("Homing retraction distance on X axis after sensor trigger (exclusion zone in negative realm)")
                        .changed()
                    {
                        changed = true;
                    }

                    ui.label("Y Home Retract:");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.home_retract_y_mm)
                                .range(0.0..=500.0)
                                .speed(0.5)
                                .suffix(" mm"),
                        )
                        .on_hover_text("Homing retraction distance on Y axis after sensor trigger (exclusion zone in negative realm)")
                        .changed()
                    {
                        changed = true;
                    }
                    ui.end_row();

                    ui.label("Z Home Retract:");
                    if ui
                        .add(
                            DragValue::new(&mut pulse_config.home_retract_z_mm)
                                .range(0.0..=500.0)
                                .speed(0.5)
                                .suffix(" mm"),
                        )
                        .on_hover_text("Homing retraction distance on Z axis after upper sensor trigger")
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
            ui.label(
                RichText::new(format!(
                    "Effective Cut Area (Table − Retract): {:.1} × {:.1} mm",
                    pulse_config.cut_width_mm(),
                    pulse_config.cut_height_mm()
                ))
                .size(11.5)
                .color(Color32::from_rgb(130, 225, 175)),
            );
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
                .num_columns(5)
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
                    if ui
                        .add(
                            Button::new(RichText::new("🎯 Calibrate X").size(11.0).strong())
                                .fill(Color32::from_rgb(42, 72, 108)),
                        )
                        .on_hover_text("Open X-axis micrometer pulse calibration dialog")
                        .clicked()
                    {
                        state.calib_dialog = Some(AxisPulseCalibState::new(Axis::X));
                    }
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
                    if ui
                        .add(
                            Button::new(RichText::new("🎯 Calibrate Y").size(11.0).strong())
                                .fill(Color32::from_rgb(42, 72, 108)),
                        )
                        .on_hover_text("Open Y-axis micrometer pulse calibration dialog")
                        .clicked()
                    {
                        state.calib_dialog = Some(AxisPulseCalibState::new(Axis::Y));
                    }
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
                    if ui
                        .add(
                            Button::new(RichText::new("🎯 Calibrate Z").size(11.0).strong())
                                .fill(Color32::from_rgb(42, 72, 108)),
                        )
                        .on_hover_text("Open Z-axis micrometer pulse calibration dialog")
                        .clicked()
                    {
                        state.calib_dialog = Some(AxisPulseCalibState::new(Axis::Z));
                    }
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
                    if ui
                        .add(
                            Button::new(RichText::new("🎯 Calibrate W").size(11.0).strong())
                                .fill(Color32::from_rgb(42, 72, 108)),
                        )
                        .on_hover_text("Open W-rotary pulse calibration dialog")
                        .clicked()
                    {
                        state.calib_dialog = Some(AxisPulseCalibState::new(Axis::W));
                    }
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

fn axis_origin_dir(pulse_config: &PulseConfig, axis: Axis) -> i32 {
    let oc = pulse_config.origin_corner.as_str();
    let x_dir = if oc.starts_with("Bottom-Right") || oc.starts_with("Top-Right") {
        1
    } else {
        -1
    };
    let y_dir = if oc.starts_with("Top-Left") || oc.starts_with("Top-Right") {
        1
    } else {
        -1
    };
    match axis {
        Axis::X => x_dir,
        Axis::Y => y_dir,
        _ => -1,
    }
}

fn axis_pos_pulses(status: &MachineStatus, axis: Axis) -> i32 {
    match axis {
        Axis::X => status.pos_x,
        Axis::Y => status.pos_y,
        Axis::Z => status.pos_z,
        Axis::W => status.pos_w,
    }
}

fn axis_ppm(pulse_config: &PulseConfig, axis: Axis) -> f64 {
    match axis {
        Axis::X => pulse_config.pulses_per_mm_x.max(0.001),
        Axis::Y => pulse_config.pulses_per_mm_y.max(0.001),
        Axis::Z => pulse_config.pulses_per_mm_z.max(0.001),
        Axis::W => pulse_config.pulses_per_deg_w.max(0.001),
    }
}

fn axis_mm_to_pulses(pulse_config: &PulseConfig, axis: Axis, mm: f64) -> i32 {
    match axis {
        Axis::X => pulse_config.mm_to_pulses_x(mm),
        Axis::Y => pulse_config.mm_to_pulses_y(mm),
        Axis::Z => pulse_config.mm_to_pulses_z(mm),
        Axis::W => (mm * pulse_config.pulses_per_deg_w.max(0.001)).round() as i32,
    }
}

fn jog_cfg_for_speed(pulse_config: &PulseConfig, axis: Axis, speed_mm_s: f64) -> JogConfig {
    let ppm = axis_ppm(pulse_config, axis);
    let spd = (speed_mm_s.max(0.5) * ppm).round() as u32;
    let vmin = (spd / 2).max(50);
    let accel = ((pulse_config.jog_accel_mm_s2.max(10.0) * ppm).round() as u32).max(500);
    JogConfig {
        min_speed: vmin,
        jog_speed: spd.max(50),
        accel,
    }
}

fn command_axis_move(
    controller: &LaserController,
    pulse_config: &PulseConfig,
    axis: Axis,
    dir: i32,
    dist_mm: f64,
    speed_mm_s: f64,
    auto_stop: bool,
) {
    let pulses = axis_mm_to_pulses(pulse_config, axis, dist_mm * (dir as f64));
    let cfg = jog_cfg_for_speed(pulse_config, axis, speed_mm_s);
    controller.jog_with_config(axis, pulses, cfg, auto_stop);
}

fn tick_measure_axis(
    m: &mut MeasureAxisState,
    pulse_config: &PulseConfig,
    controller: &LaserController,
    status: &MachineStatus,
) {
    if !m.is_running() {
        return;
    }

    if status.is_moving {
        m.motion_seen = true;
    }

    let elapsed = m.phase_entered_at.elapsed();
    if elapsed < Duration::from_millis(180) {
        return;
    }

    let axis = m.axis;
    let origin_dir = axis_origin_dir(pulse_config, axis);
    let max_dir = -origin_dir;
    let ppm = axis_ppm(pulse_config, axis);
    let cur_pos = axis_pos_pulses(status, axis);
    let traveled_mm = ((cur_pos - m.phase_start_pulses).abs() as f64) / ppm;
    let expected_table_mm = match axis {
        Axis::X => pulse_config.table_width_mm,
        Axis::Y => pulse_config.table_height_mm,
        _ => 1500.0,
    };
    let max_search_mm = (expected_table_mm + 500.0).max(2000.0);
    let sensor_triggered =
        (status.axis_limit_active(axis) && !status.is_moving) || (m.motion_seen && !status.is_moving);

    match m.phase {
        MeasurePhase::PreClearOrigin => {
            if (m.motion_seen && !status.is_moving)
                || (traveled_mm >= m.expected_move_mm - 0.5 && !status.is_moving)
            {
                m.phase = MeasurePhase::SeekOrigin;
                m.phase_entered_at = Instant::now();
                m.motion_seen = false;
                m.phase_start_pulses = cur_pos;
                m.expected_move_mm = max_search_mm;
                m.status_note = format!(
                    "Phase 1/5: Seeking {:?} origin sensor at {:.1} mm/s...",
                    axis, m.slow_speed_mm_s
                );
                command_axis_move(
                    controller,
                    pulse_config,
                    axis,
                    origin_dir,
                    max_search_mm,
                    m.slow_speed_mm_s,
                    false,
                );
            }
        }
        MeasurePhase::SeekOrigin => {
            if sensor_triggered {
                m.origin_pos_1 = cur_pos;
                m.phase = MeasurePhase::SeekMax;
                m.phase_entered_at = Instant::now();
                m.motion_seen = false;
                m.phase_start_pulses = cur_pos;
                m.expected_move_mm = max_search_mm;
                m.status_note = format!(
                    "Phase 2/5: Measuring 0 → MX at {:.1} mm/s until max sensor...",
                    m.slow_speed_mm_s
                );
                command_axis_move(
                    controller,
                    pulse_config,
                    axis,
                    max_dir,
                    max_search_mm,
                    m.slow_speed_mm_s,
                    false,
                );
            }
        }
        MeasurePhase::SeekMax => {
            if sensor_triggered && traveled_mm > 1.0 {
                m.max_pos_1 = cur_pos;
                m.meas_origin_to_max_mm = Some(((m.max_pos_1 - m.origin_pos_1).abs() as f64) / ppm);
                m.phase = MeasurePhase::RetractFromMax;
                m.phase_entered_at = Instant::now();
                m.motion_seen = false;
                m.phase_start_pulses = cur_pos;
                m.expected_move_mm = 10.0;
                m.status_note = "Phase 3/5: Retracting 10.0 mm from max sensor...".to_string();
                command_axis_move(
                    controller,
                    pulse_config,
                    axis,
                    origin_dir,
                    10.0,
                    m.slow_speed_mm_s,
                    true,
                );
            }
        }
        MeasurePhase::RetractFromMax => {
            if (m.motion_seen && !status.is_moving)
                || (traveled_mm >= 9.0 && !status.is_moving)
            {
                m.phase = MeasurePhase::RetouchMax;
                m.phase_entered_at = Instant::now();
                m.motion_seen = false;
                m.phase_start_pulses = cur_pos;
                m.expected_move_mm = 30.0;
                m.status_note = format!(
                    "Phase 4/5: Re-touching max sensor at {:.1} mm/s...",
                    m.slow_speed_mm_s
                );
                command_axis_move(
                    controller,
                    pulse_config,
                    axis,
                    max_dir,
                    30.0,
                    m.slow_speed_mm_s,
                    false,
                );
            }
        }
        MeasurePhase::RetouchMax => {
            if sensor_triggered && traveled_mm > 0.5 {
                m.max_pos_2 = cur_pos;
                let fast_dist = (expected_table_mm - 100.0).max(0.0);
                if fast_dist > 1.0 {
                    m.phase = MeasurePhase::ReturnFast;
                    m.phase_entered_at = Instant::now();
                    m.motion_seen = false;
                    m.phase_start_pulses = cur_pos;
                    m.expected_move_mm = fast_dist;
                    m.status_note = format!(
                        "Phase 5/5: Measuring MX → 0 at {:.1} mm/s (slowdown 100 mm before origin)...",
                        m.fast_return_speed_mm_s
                    );
                    command_axis_move(
                        controller,
                        pulse_config,
                        axis,
                        origin_dir,
                        fast_dist,
                        m.fast_return_speed_mm_s,
                        true,
                    );
                } else {
                    m.phase = MeasurePhase::ReturnSlow;
                    m.phase_entered_at = Instant::now();
                    m.motion_seen = false;
                    m.phase_start_pulses = cur_pos;
                    m.expected_move_mm = max_search_mm;
                    m.status_note = format!(
                        "Phase 5/5: Approaching origin sensor at {:.1} mm/s...",
                        m.slow_speed_mm_s
                    );
                    command_axis_move(
                        controller,
                        pulse_config,
                        axis,
                        origin_dir,
                        max_search_mm,
                        m.slow_speed_mm_s,
                        false,
                    );
                }
            }
        }
        MeasurePhase::ReturnFast => {
            if status.axis_limit_active(axis) && !status.is_moving && traveled_mm > 5.0 {
                // Origin sensor reached earlier than expected
                m.origin_pos_2 = cur_pos;
                let d2 = ((m.max_pos_2 - m.origin_pos_2).abs() as f64) / ppm;
                m.meas_max_to_origin_mm = Some(d2);
                if let Some(avg) = m.avg_mm() {
                    m.custom_mm = avg;
                }
                m.phase = MeasurePhase::Done;
                m.status_note = "Measurement complete!".to_string();
            } else if (m.motion_seen && !status.is_moving)
                || (traveled_mm >= m.expected_move_mm - 1.0 && !status.is_moving)
            {
                m.phase = MeasurePhase::ReturnSlow;
                m.phase_entered_at = Instant::now();
                m.motion_seen = false;
                m.phase_start_pulses = cur_pos;
                m.expected_move_mm = max_search_mm;
                m.status_note = format!(
                    "Phase 5/5: Final 100 mm slowdown at {:.1} mm/s until origin sensor...",
                    m.slow_speed_mm_s
                );
                command_axis_move(
                    controller,
                    pulse_config,
                    axis,
                    origin_dir,
                    max_search_mm,
                    m.slow_speed_mm_s,
                    false,
                );
            }
        }
        MeasurePhase::ReturnSlow => {
            if sensor_triggered && traveled_mm > 0.5 {
                m.origin_pos_2 = cur_pos;
                let d2 = ((m.max_pos_2 - m.origin_pos_2).abs() as f64) / ppm;
                m.meas_max_to_origin_mm = Some(d2);
                if let Some(avg) = m.avg_mm() {
                    m.custom_mm = avg;
                }
                m.phase = MeasurePhase::Done;
                m.status_note = "Measurement complete!".to_string();
            }
        }
        _ => {}
    }
}

fn render_measure_axis_dialog(
    ctx: &Context,
    state: &mut PropertiesWindowState,
    pulse_config: &mut PulseConfig,
    controller: &Option<LaserController>,
    status: &MachineStatus,
    status_message: &mut String,
) -> bool {
    let Some(mut m) = state.measure_axis.take() else {
        return false;
    };

    if m.is_running() {
        if let Some(ref c) = controller {
            if c.is_connected() {
                tick_measure_axis(&mut m, pulse_config, c, status);
                ctx.request_repaint_after(Duration::from_millis(30));
            } else {
                m.phase = MeasurePhase::Stopped;
                m.status_note = "Controller disconnected during measurement.".to_string();
            }
        } else {
            m.phase = MeasurePhase::Stopped;
            m.status_note = "No controller available.".to_string();
        }
    }

    let axis_name = match m.axis {
        Axis::X => "X",
        Axis::Y => "Y",
        _ => "Axis",
    };
    let is_connected = controller.as_ref().is_some_and(|c| c.is_connected());
    let mut keep_open = true;
    let mut applied = false;

    render_modal_backdrop(ctx, "measure_axis_modal_backdrop");

    Window::new(format!("📏 Measure Table {} Axis", axis_name))
        .order(egui::Order::Foreground)
        .collapsible(false)
        .resizable(false)
        .default_width(430.0)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .show(ctx, |ui| {
            ui.set_min_width(410.0);

            match m.phase {
                MeasurePhase::Confirm => {
                    ui.label(
                        RichText::new(format!(
                            "Perform physical {}-axis table length measurement?",
                            axis_name
                        ))
                        .size(13.0)
                        .strong(),
                    );
                    ui.add_space(6.0);
                    ui.label(
                        RichText::new(
                            "Sequence:\n\
                             1. Touch off origin sensor at slow speed\n\
                             2. Advance to opposite (MX) sensor at slow speed (0 → MX)\n\
                             3. Retract 10 mm and re-touch MX sensor\n\
                             4. Return at 10 mm/s, slowing to 5 mm/s 100 mm before origin (MX → 0)",
                        )
                        .size(11.5)
                        .weak(),
                    );
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        ui.label("Slow Touch Speed:");
                        ui.add(
                            DragValue::new(&mut m.slow_speed_mm_s)
                                .range(1.0..=50.0)
                                .speed(0.5)
                                .suffix(" mm/s"),
                        );
                    });
                }
                _ => {
                    ui.label(RichText::new(&m.status_note).size(12.5).strong());
                    ui.add_space(6.0);
                    let ppm = axis_ppm(pulse_config, m.axis);
                    let live_mm = (axis_pos_pulses(status, m.axis) as f64) / ppm;
                    ui.label(
                        RichText::new(format!(
                            "Live {} Position: {:.2} mm  |  Moving: {}  |  Sensor: {}",
                            axis_name,
                            live_mm,
                            if status.is_moving { "Yes" } else { "No" },
                            if status.axis_limit_active(m.axis) {
                                "TRIGGERED"
                            } else {
                                "Clear"
                            }
                        ))
                        .size(11.0)
                        .weak(),
                    );
                    ui.add_space(8.0);

                    Frame::group(ui.style()).show(ui, |ui| {
                        Grid::new("measure_results_grid")
                            .num_columns(2)
                            .spacing([16.0, 4.0])
                            .show(ui, |ui| {
                                ui.label("1st Measurement (0 → MX):");
                                ui.label(match m.meas_origin_to_max_mm {
                                    Some(v) => RichText::new(format!("{:.2} mm", v))
                                        .strong()
                                        .color(Color32::from_rgb(120, 220, 255)),
                                    None => RichText::new("Pending...").weak(),
                                });
                                ui.end_row();

                                ui.label("2nd Measurement (MX → 0):");
                                ui.label(match m.meas_max_to_origin_mm {
                                    Some(v) => RichText::new(format!("{:.2} mm", v))
                                        .strong()
                                        .color(Color32::from_rgb(120, 240, 180)),
                                    None => RichText::new("Pending...").weak(),
                                });
                                ui.end_row();

                                ui.label("Average (AVG):");
                                ui.label(match m.avg_mm() {
                                    Some(v) => RichText::new(format!("{:.2} mm", v))
                                        .strong()
                                        .color(Color32::from_rgb(255, 220, 120)),
                                    None => RichText::new("—").weak(),
                                });
                                ui.end_row();
                            });
                    });
                }
            }

            if m.phase == MeasurePhase::Done {
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    let apply_enabled = m.selected_apply_mm().is_some();
                    if ui
                        .add_enabled(
                            apply_enabled,
                            Button::new(
                                RichText::new("✓ Apply to Table")
                                    .color(Color32::WHITE)
                                    .strong(),
                            )
                            .fill(Color32::from_rgb(28, 125, 78))
                            .min_size(Vec2::new(120.0, 26.0)),
                        )
                        .clicked()
                    {
                        if let Some(val) = m.selected_apply_mm() {
                            let clamped = val.clamp(50.0, 10_000.0);
                            match m.axis {
                                Axis::X => pulse_config.table_width_mm = clamped,
                                Axis::Y => pulse_config.table_height_mm = clamped,
                                _ => {}
                            }
                            *status_message = format!(
                                "Applied measured {} table dimension: {:.2} mm ({}).",
                                axis_name,
                                clamped,
                                m.apply_choice.label()
                            );
                            applied = true;
                            keep_open = false;
                        }
                    }

                    ComboBox::from_id_salt("measure_apply_choice_combo")
                        .width(90.0)
                        .selected_text(m.apply_choice.label())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut m.apply_choice,
                                MeasureApplyChoice::Average,
                                "AVG",
                            );
                            ui.selectable_value(
                                &mut m.apply_choice,
                                MeasureApplyChoice::OriginToMax,
                                "0->MX",
                            );
                            ui.selectable_value(
                                &mut m.apply_choice,
                                MeasureApplyChoice::MaxToOrigin,
                                "MX->0",
                            );
                            ui.selectable_value(
                                &mut m.apply_choice,
                                MeasureApplyChoice::Custom,
                                "Custom",
                            );
                        });

                    if m.apply_choice == MeasureApplyChoice::Custom {
                        ui.add(
                            DragValue::new(&mut m.custom_mm)
                                .range(50.0..=10_000.0)
                                .speed(1.0)
                                .suffix(" mm"),
                        );
                    }
                });
            }

            ui.add_space(10.0);
            ui.separator();
            ui.add_space(4.0);

            ui.horizontal(|ui| {
                if m.phase == MeasurePhase::Confirm {
                    if ui
                        .add_enabled(
                            is_connected,
                            Button::new(
                                RichText::new("▶ Start Measurement")
                                    .color(Color32::WHITE)
                                    .strong(),
                            )
                            .fill(Color32::from_rgb(28, 125, 78))
                            .min_size(Vec2::new(140.0, 26.0)),
                        )
                        .on_disabled_hover_text("Connect to controller board first")
                        .clicked()
                    {
                        if let Some(ref c) = controller {
                            let origin_dir = axis_origin_dir(pulse_config, m.axis);
                            let max_dir = -origin_dir;
                            let cur_pos = axis_pos_pulses(status, m.axis);
                            let expected_table_mm = match m.axis {
                                Axis::X => pulse_config.table_width_mm,
                                Axis::Y => pulse_config.table_height_mm,
                                _ => 1500.0,
                            };
                            let max_search_mm = (expected_table_mm + 500.0).max(2000.0);
                            m.phase_entered_at = Instant::now();
                            m.motion_seen = false;
                            m.phase_start_pulses = cur_pos;
                            if status.axis_limit_active(m.axis) {
                                m.phase = MeasurePhase::PreClearOrigin;
                                m.expected_move_mm = 5.0;
                                m.status_note =
                                    "Backing off 5 mm from active sensor before touch-off..."
                                        .to_string();
                                command_axis_move(
                                    c,
                                    pulse_config,
                                    m.axis,
                                    max_dir,
                                    5.0,
                                    m.slow_speed_mm_s,
                                    true,
                                );
                            } else {
                                m.phase = MeasurePhase::SeekOrigin;
                                m.expected_move_mm = max_search_mm;
                                m.status_note = format!(
                                    "Phase 1/5: Seeking {:?} origin sensor at {:.1} mm/s...",
                                    m.axis, m.slow_speed_mm_s
                                );
                                command_axis_move(
                                    c,
                                    pulse_config,
                                    m.axis,
                                    origin_dir,
                                    max_search_mm,
                                    m.slow_speed_mm_s,
                                    false,
                                );
                            }
                        }
                    }
                }

                if ui
                    .add_enabled(
                        m.is_running(),
                        Button::new(RichText::new("⏹ Stop").color(Color32::WHITE).strong())
                            .fill(Color32::from_rgb(175, 42, 42))
                            .min_size(Vec2::new(78.0, 26.0)),
                    )
                    .clicked()
                {
                    if let Some(ref c) = controller {
                        c.stop();
                    }
                    m.phase = MeasurePhase::Stopped;
                    m.status_note = "Measurement stopped by user.".to_string();
                    *status_message = format!("{} axis measurement stopped.", axis_name);
                }

                if ui
                    .add(
                        Button::new(RichText::new("Cancel").size(12.0))
                            .min_size(Vec2::new(78.0, 26.0)),
                    )
                    .clicked()
                {
                    if m.is_running() {
                        if let Some(ref c) = controller {
                            c.stop();
                        }
                    }
                    keep_open = false;
                }
            });
        });

    if keep_open {
        state.measure_axis = Some(m);
    }
    applied
}

fn render_modal_backdrop(ctx: &Context, id: &str) {
    let screen_rect = ctx.screen_rect();
    egui::Area::new(egui::Id::new(id))
        .order(egui::Order::Middle)
        .fixed_pos(screen_rect.min)
        .interactable(true)
        .show(ctx, |ui| {
            let (rect, _) = ui.allocate_exact_size(screen_rect.size(), egui::Sense::click_and_drag());
            ui.painter()
                .rect_filled(rect, 0.0, Color32::from_rgba_unmultiplied(8, 10, 14, 155));
        });
}

fn render_axis_calib_dialog(
    ctx: &Context,
    state: &mut PropertiesWindowState,
    pulse_config: &mut PulseConfig,
    controller: &Option<LaserController>,
    status_message: &mut String,
) -> bool {
    let Some(mut d) = state.calib_dialog.take() else {
        return false;
    };

    render_modal_backdrop(ctx, "axis_calib_modal_backdrop");

    let is_connected = controller.as_ref().is_some_and(|c| c.is_connected());
    let (axis_name, unit, cur_ppm) = match d.axis {
        Axis::X => ("X", "mm", pulse_config.pulses_per_mm_x),
        Axis::Y => ("Y", "mm", pulse_config.pulses_per_mm_y),
        Axis::Z => ("Z", "mm", pulse_config.pulses_per_mm_z),
        Axis::W => ("W", "deg", pulse_config.pulses_per_deg_w),
    };
    let (neg_arrow, pos_arrow) = match d.axis {
        Axis::X => ("◀", "▶"),
        Axis::Y | Axis::Z => ("▼", "▲"),
        Axis::W => ("⟲", "⟳"),
    };

    let mut keep_open = true;
    let mut applied = false;

    Window::new(format!("🎯 Calibrate {}-Axis Pulses / {}", axis_name, unit))
        .order(egui::Order::Foreground)
        .collapsible(false)
        .resizable(false)
        .default_width(430.0)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .show(ctx, |ui| {
            ui.set_min_width(410.0);

            ui.label(
                RichText::new(format!(
                    "Micrometer Calibration for {}-Axis (current: {:.3} pls/{})",
                    axis_name, cur_ppm, unit
                ))
                .size(12.5)
                .strong(),
            );
            ui.add_space(4.0);
            ui.label(
                RichText::new(format!(
                    "1. Place a micrometer in line with the {}-axis touching the cutting head and zero it.\n\
                     2. Use the arrow buttons below to displace the head (e.g. ±1.0 {}).\n\
                     3. Enter the actual micrometer reading and click Apply Calibration.",
                    axis_name, unit
                ))
                .size(11.0)
                .weak(),
            );
            ui.add_space(8.0);

            ui.horizontal(|ui| {
                ui.label("Step Size:");
                ui.add(
                    DragValue::new(&mut d.step_mm)
                        .range(0.05..=100.0)
                        .speed(0.1)
                        .max_decimals(2)
                        .suffix(format!(" {}", unit)),
                );
                for preset in [0.5, 1.0, 5.0, 10.0] {
                    if ui.small_button(format!("{:.1}", preset)).clicked() {
                        d.step_mm = preset;
                    }
                }
            });

            ui.add_space(6.0);
            ui.horizontal(|ui| {
                let neg_lbl = format!("{} -{:.2} {}", neg_arrow, d.step_mm, unit);
                let pos_lbl = format!("+{:.2} {} {}", d.step_mm, unit, pos_arrow);

                if ui
                    .add_enabled(
                        is_connected,
                        Button::new(RichText::new(neg_lbl).size(12.0).strong())
                            .fill(Color32::from_rgb(44, 58, 78))
                            .min_size(Vec2::new(128.0, 30.0)),
                    )
                    .on_disabled_hover_text("Connect to controller board first")
                    .clicked()
                {
                    if let Some(ref c) = controller {
                        let pulses = axis_mm_to_pulses(pulse_config, d.axis, -d.step_mm);
                        c.jog(d.axis, pulses);
                        d.commanded_pulses_total += pulses;
                        d.commanded_signed_mm -= d.step_mm;
                    }
                }

                if ui
                    .add_enabled(
                        is_connected,
                        Button::new(RichText::new(pos_lbl).size(12.0).strong())
                            .fill(Color32::from_rgb(44, 58, 78))
                            .min_size(Vec2::new(128.0, 30.0)),
                    )
                    .on_disabled_hover_text("Connect to controller board first")
                    .clicked()
                {
                    if let Some(ref c) = controller {
                        let pulses = axis_mm_to_pulses(pulse_config, d.axis, d.step_mm);
                        c.jog(d.axis, pulses);
                        d.commanded_pulses_total += pulses;
                        d.commanded_signed_mm += d.step_mm;
                    }
                }

                if ui
                    .add(
                        Button::new(RichText::new("Zero / Reset").size(11.0))
                            .min_size(Vec2::new(88.0, 30.0)),
                    )
                    .on_hover_text("Reset commanded displacement counter after re-zeroing the micrometer")
                    .clicked()
                {
                    d.commanded_pulses_total = 0;
                    d.commanded_signed_mm = 0.0;
                }
            });

            ui.add_space(8.0);
            Frame::group(ui.style()).show(ui, |ui| {
                Grid::new("axis_calib_readout_grid")
                    .num_columns(2)
                    .spacing([14.0, 6.0])
                    .show(ui, |ui| {
                        ui.label("Commanded Net Displacement:");
                        ui.label(
                            RichText::new(format!(
                                "{:+.3} {} ({} pulses)",
                                d.commanded_signed_mm, unit, d.commanded_pulses_total
                            ))
                            .strong()
                            .color(Color32::from_rgb(130, 215, 255)),
                        );
                        ui.end_row();

                        ui.label(format!("Micrometer Reading ({}):", unit));
                        ui.add(
                            DragValue::new(&mut d.micrometer_reading_mm)
                                .range(-1000.0..=1000.0)
                                .speed(0.005)
                                .max_decimals(4)
                                .suffix(format!(" {}", unit)),
                        );
                        ui.end_row();

                        ui.label("Calibrated Value:");
                        match d.computed_pulses_per_unit() {
                            Some(new_ppm) => {
                                ui.label(
                                    RichText::new(format!(
                                        "{:.4} pls/{} (was {:.4})",
                                        new_ppm, unit, cur_ppm
                                    ))
                                    .strong()
                                    .color(Color32::from_rgb(120, 240, 175)),
                                );
                            }
                            None => {
                                ui.label(
                                    RichText::new("Move head with arrow buttons first")
                                        .size(11.0)
                                        .weak(),
                                );
                            }
                        }
                        ui.end_row();
                    });
            });

            ui.add_space(10.0);
            ui.separator();
            ui.add_space(4.0);

            ui.horizontal(|ui| {
                let can_apply = d.computed_pulses_per_unit().is_some();
                if ui
                    .add_enabled(
                        can_apply,
                        Button::new(
                            RichText::new("✓ Apply Calibration")
                                .color(Color32::WHITE)
                                .strong(),
                        )
                        .fill(Color32::from_rgb(28, 125, 78))
                        .min_size(Vec2::new(140.0, 26.0)),
                    )
                    .clicked()
                {
                    if let Some(new_ppm) = d.computed_pulses_per_unit() {
                        let clamped = new_ppm.clamp(0.1, 100_000.0);
                        match d.axis {
                            Axis::X => pulse_config.pulses_per_mm_x = clamped,
                            Axis::Y => pulse_config.pulses_per_mm_y = clamped,
                            Axis::Z => pulse_config.pulses_per_mm_z = clamped,
                            Axis::W => pulse_config.pulses_per_deg_w = clamped,
                        }
                        *status_message = format!(
                            "Calibrated {}-axis to {:.4} pls/{} (micrometer: {:+.4} {} for {} pulses).",
                            axis_name,
                            clamped,
                            unit,
                            d.micrometer_reading_mm,
                            unit,
                            d.commanded_pulses_total
                        );
                        applied = true;
                        keep_open = false;
                    }
                }

                if ui
                    .add_enabled(
                        is_connected,
                        Button::new(RichText::new("⏹ Stop").color(Color32::WHITE).strong())
                            .fill(Color32::from_rgb(175, 42, 42))
                            .min_size(Vec2::new(76.0, 26.0)),
                    )
                    .clicked()
                {
                    if let Some(ref c) = controller {
                        c.stop();
                    }
                }

                if ui
                    .add(
                        Button::new(RichText::new("Cancel").size(12.0))
                            .min_size(Vec2::new(76.0, 26.0)),
                    )
                    .clicked()
                {
                    keep_open = false;
                }
            });
        });

    if keep_open {
        state.calib_dialog = Some(d);
    }
    applied
}
