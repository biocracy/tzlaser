use egui::{Button, CollapsingHeader, Color32, Frame, Grid, ProgressBar, RichText, ScrollArea, Ui};
use tzlaser_core::{Document, MotionSegment, PulseConfig};
use tzlaser_protocol::{Axis, LaserController, MachineStatus};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JogMode {
    Step(f64), // mm
    Continuous,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunMode {
    Idle,
    DryCut,
    Cutting,
}

#[derive(Debug, Clone, Copy)]
pub struct PauseCheckpoint {
    pub point_mm: kurbo::Point,
    pub seg_idx: usize,
    #[allow(dead_code)]
    pub seg_progress_mm: f64,
    pub traveled_length_mm: f64,
    pub path_index_1based: usize,
    pub mode: RunMode,
}

pub struct PanelState {
    pub jog_mode: JogMode,
    pub step_index: usize,
    pub target_ip: String,
    pub run_mode: RunMode,
    pub is_paused: bool,
    pub sim_speed_multiplier: f64,
    pub trajectory: Vec<MotionSegment>,
    pub current_seg_idx: usize,
    pub seg_progress_mm: f64,
    pub total_length_mm: f64,
    pub traveled_length_mm: f64,
    pub last_tick: Option<std::time::Instant>,
    pub last_hw_dispatched_seg: Option<usize>,
    pub paused_checkpoint: Option<PauseCheckpoint>,
    pub pause_buffer: Vec<PauseCheckpoint>,
}

impl Default for PanelState {
    fn default() -> Self {
        Self {
            jog_mode: JogMode::Step(10.0),
            step_index: 2, // 10mm
            target_ip: "10.1.1.168:502".to_string(),
            run_mode: RunMode::Idle,
            is_paused: false,
            sim_speed_multiplier: 1.0,
            trajectory: Vec::new(),
            current_seg_idx: 0,
            seg_progress_mm: 0.0,
            total_length_mm: 0.0,
            traveled_length_mm: 0.0,
            last_tick: None,
            last_hw_dispatched_seg: None,
            paused_checkpoint: None,
            pause_buffer: Vec::new(),
        }
    }
}

impl PanelState {
    pub fn start_run(
        &mut self,
        mode: RunMode,
        doc: &Document,
        current_head: kurbo::Point,
        controller: &Option<LaserController>,
    ) -> String {
        let traj = doc.build_motion_trajectory(current_head, 200.0);
        if traj.is_empty() {
            return "No active toolpaths to run on visible layers.".to_string();
        }
        self.total_length_mm = traj.iter().map(|s| s.length_mm).sum();
        self.trajectory = traj;
        self.current_seg_idx = 0;
        self.seg_progress_mm = 0.0;
        self.traveled_length_mm = 0.0;
        self.is_paused = false;
        self.run_mode = mode;
        self.last_tick = Some(std::time::Instant::now());
        self.last_hw_dispatched_seg = None;
        self.paused_checkpoint = None;
        self.pause_buffer.clear();

        if mode == RunMode::DryCut {
            if let Some(ref c) = controller {
                if c.is_connected() {
                    c.set_red_pointer(true);
                }
            }
            format!("Dry-Cut simulation started ({} segments, {:.1} mm).", self.trajectory.len(), self.total_length_mm)
        } else {
            format!("Cutting job started ({} segments, {:.1} mm).", self.trajectory.len(), self.total_length_mm)
        }
    }

    pub fn stop_run(&mut self, controller: &Option<LaserController>, status: &mut MachineStatus) {
        let was_dry = self.run_mode == RunMode::DryCut;
        self.run_mode = RunMode::Idle;
        self.is_paused = false;
        self.last_tick = None;
        self.last_hw_dispatched_seg = None;
        self.paused_checkpoint = None;
        self.pause_buffer.clear();
        status.laser_on = false;
        if was_dry {
            status.red_pointer_on = false;
        }
        if let Some(ref c) = controller {
            if c.is_connected() {
                if was_dry {
                    c.set_red_pointer(false);
                }
                c.stop();
            }
        }
    }

    /// Pause the active run/simulation, turn the laser off, stop controller motion,
    /// and record the exact head position and segment state in the pause buffer.
    pub fn pause_run(
        &mut self,
        controller: &Option<LaserController>,
        status: &mut MachineStatus,
        pulses_cfg: &PulseConfig,
    ) -> Option<String> {
        if self.run_mode == RunMode::Idle
            || self.is_paused
            || self.current_seg_idx >= self.trajectory.len()
        {
            return None;
        }

        let seg = self.trajectory[self.current_seg_idx];
        let frac = (self.seg_progress_mm / seg.length_mm.max(1e-9)).clamp(0.0, 1.0);
        let point_mm = kurbo::Point::new(
            seg.start.x + (seg.end.x - seg.start.x) * frac,
            seg.start.y + (seg.end.y - seg.start.y) * frac,
        );

        status.pos_x = pulses_cfg.mm_to_pulses_x(point_mm.x);
        status.pos_y = pulses_cfg.mm_to_pulses_y(point_mm.y);
        status.laser_on = false;

        if let Some(ref c) = controller {
            if c.is_connected() {
                c.stop();
            }
        }

        let cp = PauseCheckpoint {
            point_mm,
            seg_idx: self.current_seg_idx,
            seg_progress_mm: self.seg_progress_mm,
            traveled_length_mm: self.traveled_length_mm,
            path_index_1based: seg.path_index_1based,
            mode: self.run_mode,
        };
        self.is_paused = true;
        self.last_tick = None;
        self.last_hw_dispatched_seg = None;
        self.paused_checkpoint = Some(cp);
        self.pause_buffer.push(cp);

        Some(format!(
            "Paused at X: {:.2}, Y: {:.2} mm (Path #{}). Head unlocked for manual jogging.",
            point_mm.x, point_mm.y, cp.path_index_1based
        ))
    }

    /// Resume from a recorded pause checkpoint. If the head was jogged away while paused,
    /// prepends a rapid laser-OFF return segment back to the recorded pause position before
    /// continuing the interrupted segment and all remaining paths.
    pub fn continue_run(
        &mut self,
        current_head: kurbo::Point,
        controller: &Option<LaserController>,
    ) -> Option<String> {
        let cp = self.paused_checkpoint.take()?;

        let mut new_traj: Vec<MotionSegment> = Vec::new();
        let return_dist = current_head.distance(cp.point_mm);
        if return_dist > 1e-4 {
            new_traj.push(MotionSegment {
                start: current_head,
                end: cp.point_mm,
                length_mm: return_dist,
                speed_mm_s: 200.0,
                is_rapid: true,
                path_index_1based: cp.path_index_1based,
            });
        }

        if cp.seg_idx < self.trajectory.len() {
            let orig_seg = self.trajectory[cp.seg_idx];
            let rem_len = cp.point_mm.distance(orig_seg.end);
            if rem_len > 1e-6 {
                new_traj.push(MotionSegment {
                    start: cp.point_mm,
                    end: orig_seg.end,
                    length_mm: rem_len,
                    speed_mm_s: orig_seg.speed_mm_s,
                    is_rapid: orig_seg.is_rapid,
                    path_index_1based: orig_seg.path_index_1based,
                });
            }
            if cp.seg_idx + 1 < self.trajectory.len() {
                new_traj.extend_from_slice(&self.trajectory[(cp.seg_idx + 1)..]);
            }
        }

        if new_traj.is_empty() {
            self.run_mode = RunMode::Idle;
            self.is_paused = false;
            self.trajectory.clear();
            self.current_seg_idx = 0;
            self.seg_progress_mm = 0.0;
            self.last_tick = None;
            self.last_hw_dispatched_seg = None;
            if cp.mode == RunMode::DryCut {
                if let Some(ref c) = controller {
                    if c.is_connected() {
                        c.set_red_pointer(false);
                    }
                }
            }
            return Some("No remaining segments to continue; job complete.".to_string());
        }

        let rem_total: f64 = new_traj.iter().map(|s| s.length_mm).sum();
        self.trajectory = new_traj;
        self.current_seg_idx = 0;
        self.seg_progress_mm = 0.0;
        self.traveled_length_mm = cp.traveled_length_mm;
        self.total_length_mm = cp.traveled_length_mm + rem_total;
        self.run_mode = cp.mode;
        self.is_paused = false;
        self.last_tick = Some(std::time::Instant::now());
        self.last_hw_dispatched_seg = None;

        if cp.mode == RunMode::DryCut {
            if let Some(ref c) = controller {
                if c.is_connected() {
                    c.set_red_pointer(true);
                }
            }
        }

        if return_dist > 1e-4 {
            Some(format!(
                "Continuing from X: {:.2}, Y: {:.2} mm (returning {:.1} mm to pause point {:.2}, {:.2} mm).",
                current_head.x, current_head.y, return_dist, cp.point_mm.x, cp.point_mm.y
            ))
        } else {
            Some(format!(
                "Continuing from pause point X: {:.2}, Y: {:.2} mm (Path #{}).",
                cp.point_mm.x, cp.point_mm.y, cp.path_index_1based
            ))
        }
    }

    pub fn tick_simulation(
        &mut self,
        controller: &Option<LaserController>,
        status: &mut MachineStatus,
        pulses_cfg: &PulseConfig,
    ) -> Option<String> {
        if self.run_mode == RunMode::Idle || self.is_paused || self.trajectory.is_empty() {
            return None;
        }

        let now = std::time::Instant::now();
        let dt = self
            .last_tick
            .map(|t| (now - t).as_secs_f64().clamp(0.0, 0.1))
            .unwrap_or(0.016);
        self.last_tick = Some(now);

        let hw_connected = controller.as_ref().map_or(false, |c| c.is_connected());
        let speed_mult = if hw_connected { 1.0 } else { self.sim_speed_multiplier };

        let mut remaining_dt = dt;
        while remaining_dt > 1e-9 && self.current_seg_idx < self.trajectory.len() {
            if hw_connected && self.last_hw_dispatched_seg != Some(self.current_seg_idx) {
                if let Some(ref c) = controller {
                    let seg = self.trajectory[self.current_seg_idx];
                    let dx_p = pulses_cfg.mm_to_pulses_x(seg.end.x - seg.start.x);
                    let dy_p = pulses_cfg.mm_to_pulses_y(seg.end.y - seg.start.y);
                    if dx_p != 0 {
                        c.jog(Axis::X, dx_p);
                    }
                    if dy_p != 0 {
                        c.jog(Axis::Y, dy_p);
                    }
                }
                self.last_hw_dispatched_seg = Some(self.current_seg_idx);
            }

            let seg = self.trajectory[self.current_seg_idx];
            let eff_speed = (seg.speed_mm_s * speed_mult).max(1.0);
            let seg_remaining = (seg.length_mm - self.seg_progress_mm).max(0.0);
            let step_mm = eff_speed * remaining_dt;

            if step_mm < seg_remaining {
                self.seg_progress_mm += step_mm;
                self.traveled_length_mm += step_mm;
                remaining_dt = 0.0;
            } else {
                self.traveled_length_mm += seg_remaining;
                remaining_dt -= seg_remaining / eff_speed;
                self.seg_progress_mm = 0.0;
                self.current_seg_idx += 1;
            }
        }

        if self.current_seg_idx >= self.trajectory.len() {
            if let Some(last) = self.trajectory.last() {
                status.pos_x = pulses_cfg.mm_to_pulses_x(last.end.x);
                status.pos_y = pulses_cfg.mm_to_pulses_y(last.end.y);
            }
            let finished_label = if self.run_mode == RunMode::DryCut {
                "Dry-Cut simulation"
            } else {
                "Cutting job"
            };
            let total = self.total_length_mm;
            self.stop_run(controller, status);
            return Some(format!("{} completed ({:.1} mm total travel).", finished_label, total));
        }

        let seg = self.trajectory[self.current_seg_idx];
        let frac = (self.seg_progress_mm / seg.length_mm.max(1e-9)).clamp(0.0, 1.0);
        let cur_x = seg.start.x + (seg.end.x - seg.start.x) * frac;
        let cur_y = seg.start.y + (seg.end.y - seg.start.y) * frac;
        status.pos_x = pulses_cfg.mm_to_pulses_x(cur_x);
        status.pos_y = pulses_cfg.mm_to_pulses_y(cur_y);
        status.laser_on = (self.run_mode == RunMode::Cutting) && !seg.is_rapid;
        status.red_pointer_on = (self.run_mode == RunMode::DryCut) || status.red_pointer_on;

        None
    }
}

pub fn render_right_panels(
    ui: &mut Ui,
    panel_state: &mut PanelState,
    controller: &Option<LaserController>,
    status: &mut MachineStatus,
    pulses_cfg: &PulseConfig,
    doc: &mut Document,
    status_message: &mut String,
) {
    ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
        // ==========================================
        // 1. MONITOR PANEL (Collapsible with arrow icon)
        // ==========================================
        CollapsingHeader::new(RichText::new("📊 Monitor").strong().size(14.0))
            .default_open(true)
            .show(ui, |ui| {
                ui.add_space(2.0);

                // 1A. Head Position DRO (Moved here from Machine Controls)
                let pos_x_mm = pulses_cfg.pulses_to_mm_x(status.pos_x);
                let pos_y_mm = pulses_cfg.pulses_to_mm_y(status.pos_y);
                let pos_z_mm = pulses_cfg.pulses_to_mm_x(status.pos_z);

                Frame::group(ui.style()).fill(Color32::from_rgb(18, 20, 24)).show(ui, |ui| {
                    ui.label(RichText::new("Head Coordinates (WCS)").size(11.0).color(Color32::from_rgb(140, 150, 170)).strong());
                    ui.add_space(2.0);
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
                    ui.add_space(2.0);
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

                        ui.label("Red Pointer:");
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
                    ui.add_space(2.0);
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
                    ui.add_space(2.0);
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

                ui.add_space(6.0);
            });

        ui.add_space(4.0);
        ui.separator();

        // ==========================================
        // 2. MACHINE CONTROLS (Collapsible with arrow icon)
        // ==========================================
        CollapsingHeader::new(RichText::new("🎮 Machine Controls").strong().size(14.0))
            .default_open(true)
            .show(ui, |ui| {
                ui.add_space(2.0);

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

                ui.add_space(6.0);

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

                // Jog Control Pad (3x3 Grid: +/-Z to the right of +/-Y, with X+ centered between Z+ and Z-)
                let step_dist = match panel_state.jog_mode {
                    JogMode::Step(d) => d,
                    JogMode::Continuous => 10.0,
                };
                let step_pulses_x = pulses_cfg.mm_to_pulses_x(step_dist);
                let step_pulses_y = pulses_cfg.mm_to_pulses_y(step_dist);
                let step_pulses_z = (step_dist * pulses_cfg.pulses_per_mm_z).round() as i32;

                let btn_size = [72.0, 34.0];
                let pad_width = 72.0 * 3.0 + 12.0;
                let mut jog_cmd: Option<(Axis, i32)> = None;
                let mut stop_clicked = false;

                ui.horizontal(|ui| {
                    ui.add_space((ui.available_width() - pad_width).max(0.0) / 2.0);
                    Grid::new("jog_pad_3x3")
                        .num_columns(3)
                        .spacing([6.0, 6.0])
                        .show(ui, |ui| {
                            // Row 1: [empty] | Y+ | Z+
                            ui.allocate_exact_size(egui::vec2(btn_size[0], btn_size[1]), egui::Sense::hover());
                            if ui.add_sized(btn_size, Button::new(RichText::new("Y+ ▲").size(13.0))).clicked() {
                                jog_cmd = Some((Axis::Y, step_pulses_y));
                            }
                            let z_plus_btn = Button::new(RichText::new("Z+ ▲").size(13.0).color(Color32::from_rgb(130, 195, 255)))
                                .fill(Color32::from_rgb(28, 38, 56));
                            if ui.add_sized(btn_size, z_plus_btn).on_hover_text("Raise cutting head (Z+)").clicked() {
                                jog_cmd = Some((Axis::Z, step_pulses_z));
                            }
                            ui.end_row();

                            // Row 2: X- | STOP | X+ (middle between Z+ and Z-)
                            if ui.add_sized(btn_size, Button::new(RichText::new("◄ X-").size(13.0))).clicked() {
                                jog_cmd = Some((Axis::X, -step_pulses_x));
                            }
                            let stop_btn = Button::new(RichText::new("STOP").color(Color32::WHITE).strong())
                                .fill(Color32::from_rgb(180, 40, 40));
                            if ui.add_sized(btn_size, stop_btn).clicked() {
                                stop_clicked = true;
                            }
                            if ui.add_sized(btn_size, Button::new(RichText::new("X+ ►").size(13.0))).clicked() {
                                jog_cmd = Some((Axis::X, step_pulses_x));
                            }
                            ui.end_row();

                            // Row 3: [empty] | Y- | Z-
                            ui.allocate_exact_size(egui::vec2(btn_size[0], btn_size[1]), egui::Sense::hover());
                            if ui.add_sized(btn_size, Button::new(RichText::new("Y- ▼").size(13.0))).clicked() {
                                jog_cmd = Some((Axis::Y, -step_pulses_y));
                            }
                            let z_minus_btn = Button::new(RichText::new("Z- ▼").size(13.0).color(Color32::from_rgb(130, 195, 255)))
                                .fill(Color32::from_rgb(28, 38, 56));
                            if ui.add_sized(btn_size, z_minus_btn).on_hover_text("Lower cutting head (Z-)").clicked() {
                                jog_cmd = Some((Axis::Z, -step_pulses_z));
                            }
                            ui.end_row();
                        });
                });

                if stop_clicked {
                    panel_state.stop_run(controller, status);
                    *status_message = "Motion stopped.".to_string();
                } else if let Some((axis, pulses)) = jog_cmd {
                    if is_connected {
                        if let Some(c) = controller {
                            c.jog(axis, pulses);
                        }
                    } else {
                        match axis {
                            Axis::X => status.pos_x += pulses,
                            Axis::Y => status.pos_y += pulses,
                            Axis::Z => {
                                status.pos_z += pulses;
                                status.capacitive_height_mm =
                                    ((status.pos_z as f32) / (pulses_cfg.pulses_per_mm_z as f32)).max(0.0);
                            }
                            Axis::W => status.pos_w += pulses,
                        }
                    }
                }

                ui.add_space(8.0);
                ui.separator();
                ui.add_space(4.0);

                // Job Execution & Dry-Cut Simulation (2x2 Pendant Grid + Pause Buffer)
                ui.label(RichText::new("Job Execution & Dry-Cut:").strong().size(12.0));
                ui.add_space(3.0);

                let current_head = kurbo::Point::new(
                    pulses_cfg.pulses_to_mm_x(status.pos_x),
                    pulses_cfg.pulses_to_mm_y(status.pos_y),
                );
                let run_btn_w = ((ui.available_width() - 8.0) / 2.0).max(95.0);
                let has_pause = panel_state.paused_checkpoint.is_some();

                // Row 1: ▶ Start / 🔄 Restart  |  🧭 Dry-Cut
                ui.horizontal(|ui| {
                    let is_cutting = panel_state.run_mode == RunMode::Cutting && !panel_state.is_paused;
                    let start_fill = if is_cutting {
                        Color32::from_rgb(35, 160, 75)
                    } else {
                        Color32::from_rgb(26, 110, 52)
                    };
                    let start_label = if has_pause { "🔄 Restart" } else { "▶ Start" };
                    let start_btn = Button::new(RichText::new(start_label).size(13.0).color(Color32::WHITE).strong())
                        .fill(start_fill);
                    if ui
                        .add_sized([run_btn_w, 32.0], start_btn)
                        .on_hover_text(if has_pause {
                            "Restart cutting job from the beginning (discards pause checkpoint)"
                        } else {
                            "Start cutting job along sorted toolpath sequence"
                        })
                        .clicked()
                    {
                        *status_message = panel_state.start_run(RunMode::Cutting, doc, current_head, controller);
                    }

                    let is_dry = panel_state.run_mode == RunMode::DryCut && !panel_state.is_paused;
                    let dry_fill = if is_dry {
                        Color32::from_rgb(20, 130, 165)
                    } else {
                        Color32::from_rgb(24, 72, 95)
                    };
                    let dry_btn = Button::new(
                        RichText::new("🧭 Dry-Cut")
                            .size(13.0)
                            .color(Color32::from_rgb(130, 240, 255))
                            .strong(),
                    )
                    .fill(dry_fill);
                    if ui
                        .add_sized([run_btn_w, 32.0], dry_btn)
                        .on_hover_text(if is_connected {
                            "Move physical cutting head along toolpaths without firing the laser"
                        } else {
                            "Simulate pointer movement along toolpaths offline (with fast-forward)"
                        })
                        .clicked()
                    {
                        *status_message = panel_state.start_run(RunMode::DryCut, doc, current_head, controller);
                    }
                });

                ui.add_space(4.0);

                // Row 2: ⏸ Pause  |  ⏯ Continue
                ui.horizontal(|ui| {
                    let can_pause = panel_state.run_mode != RunMode::Idle && !panel_state.is_paused;
                    let can_continue = panel_state.is_paused && has_pause;

                    let pause_fill = if can_pause {
                        Color32::from_rgb(175, 115, 22)
                    } else {
                        Color32::from_rgb(48, 50, 62)
                    };
                    let pause_btn = Button::new(RichText::new("⏸ Pause").size(12.5).color(Color32::WHITE).strong())
                        .fill(pause_fill);
                    if ui
                        .add_enabled_ui(can_pause, |ui| ui.add_sized([run_btn_w, 30.0], pause_btn))
                        .inner
                        .on_hover_text("Pause job, turn laser off, record position in buffer, and allow manual jogging")
                        .clicked()
                    {
                        if let Some(msg) = panel_state.pause_run(controller, status, pulses_cfg) {
                            *status_message = msg;
                        }
                    }

                    let cont_fill = if can_continue {
                        Color32::from_rgb(28, 135, 108)
                    } else {
                        Color32::from_rgb(48, 50, 62)
                    };
                    let cont_btn = Button::new(RichText::new("⏯ Continue").size(12.5).color(Color32::WHITE).strong())
                        .fill(cont_fill);
                    if ui
                        .add_enabled_ui(can_continue, |ui| ui.add_sized([run_btn_w, 30.0], cont_btn))
                        .inner
                        .on_hover_text("Return from current position to recorded pause point and continue")
                        .clicked()
                    {
                        if let Some(msg) = panel_state.continue_run(current_head, controller) {
                            *status_message = msg;
                        }
                    }
                });

                // Pause Buffer Card (visible when a paused checkpoint is active)
                if let Some(cp) = panel_state.paused_checkpoint {
                    ui.add_space(4.0);
                    Frame::group(ui.style())
                        .fill(Color32::from_rgb(36, 30, 20))
                        .stroke(egui::Stroke::new(1.0, Color32::from_rgb(205, 145, 38)))
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new(format!(
                                    "📌 Paused @ X: {:.2}, Y: {:.2} mm (Path #{})",
                                    cp.point_mm.x, cp.point_mm.y, cp.path_index_1based
                                ))
                                .size(11.5)
                                .color(Color32::from_rgb(255, 205, 90))
                                .strong(),
                            );
                            ui.horizontal(|ui| {
                                let offset = current_head.distance(cp.point_mm);
                                let buf_info = if offset > 0.05 {
                                    format!(
                                        "Buffer: {} pt(s)  •  Jog offset: {:.1} mm",
                                        panel_state.pause_buffer.len(),
                                        offset
                                    )
                                } else {
                                    format!("Pause buffer: {} point(s)", panel_state.pause_buffer.len())
                                };
                                ui.label(
                                    RichText::new(buf_info)
                                        .size(11.0)
                                        .color(Color32::from_rgb(215, 205, 185)),
                                );
                                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                    let reset_btn = Button::new(
                                        RichText::new("⏹ Reset").size(11.0).color(Color32::WHITE).strong(),
                                    )
                                    .fill(Color32::from_rgb(155, 42, 42));
                                    if ui
                                        .add(reset_btn)
                                        .on_hover_text("Discard paused checkpoint and stop job")
                                        .clicked()
                                    {
                                        panel_state.stop_run(controller, status);
                                        *status_message = "Job reset; pause buffer cleared.".to_string();
                                    }
                                });
                            });
                        });
                }

                // Active Run Progress Bar
                if panel_state.run_mode != RunMode::Idle {

                    ui.add_space(4.0);
                    let progress = if panel_state.total_length_mm > 1e-6 {
                        (panel_state.traveled_length_mm / panel_state.total_length_mm).clamp(0.0, 1.0) as f32
                    } else {
                        0.0
                    };
                    let total_paths = doc.compute_cut_sequence().len();
                    let cur_path = panel_state
                        .trajectory
                        .get(panel_state.current_seg_idx)
                        .map(|s| s.path_index_1based)
                        .unwrap_or(total_paths);
                    let mode_tag = if panel_state.run_mode == RunMode::DryCut {
                        if is_connected { "Dry-Cut (HW)" } else { "Dry-Cut Sim" }
                    } else {
                        "Cutting"
                    };
                    let bar_text = if is_connected {
                        format!("{}: Path {}/{} ({:.0}%)", mode_tag, cur_path, total_paths, progress * 100.0)
                    } else {
                        format!(
                            "{}: Path {}/{} ({:.0}%) [{}x]",
                            mode_tag, cur_path, total_paths, progress * 100.0, panel_state.sim_speed_multiplier
                        )
                    };
                    ui.add(ProgressBar::new(progress).text(bar_text));
                }

                // Fast-forward speed multiplier when purely simulating (controller offline)
                if !is_connected {
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label(RichText::new("Fast-Forward:").size(11.5).color(Color32::from_rgb(170, 200, 220)).strong());
                        for &mult in &[1.0, 2.0, 4.0, 8.0, 16.0] {
                            let selected = (panel_state.sim_speed_multiplier - mult).abs() < 1e-6;
                            let label = format!("{}x", mult as u32);
                            if ui.selectable_label(selected, RichText::new(label).size(11.5).strong()).clicked() {
                                panel_state.sim_speed_multiplier = mult;
                            }
                        }
                    });
                }

                ui.add_space(6.0);
            });

        ui.add_space(4.0);
        ui.separator();

        // ==========================================
        // 3. CUTTING LAYERS (Collapsible with arrow icon)
        // ==========================================
        CollapsingHeader::new(RichText::new("🎨 Cutting Layers").strong().size(14.0))
            .default_open(true)
            .show(ui, |ui| {
                ui.add_space(2.0);

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
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pause_jog_and_continue_trajectory() {
        let pulses = PulseConfig::default();
        let mut status = MachineStatus::default();
        let mut state = PanelState::default();

        state.trajectory = vec![
            MotionSegment {
                start: kurbo::Point::new(0.0, 0.0),
                end: kurbo::Point::new(100.0, 0.0),
                length_mm: 100.0,
                speed_mm_s: 50.0,
                is_rapid: false,
                path_index_1based: 1,
            },
            MotionSegment {
                start: kurbo::Point::new(100.0, 0.0),
                end: kurbo::Point::new(100.0, 50.0),
                length_mm: 50.0,
                speed_mm_s: 50.0,
                is_rapid: false,
                path_index_1based: 1,
            },
        ];
        state.run_mode = RunMode::Cutting;
        state.current_seg_idx = 0;
        state.seg_progress_mm = 40.0;
        state.traveled_length_mm = 40.0;
        state.total_length_mm = 150.0;
        status.laser_on = true;

        // 1. Pause mid-cut at (40, 0)
        let msg = state.pause_run(&None, &mut status, &pulses);
        assert!(msg.is_some());
        assert!(state.is_paused);
        assert!(!status.laser_on);
        assert_eq!(state.pause_buffer.len(), 1);
        let cp = state.paused_checkpoint.expect("checkpoint should be recorded");
        assert!((cp.point_mm.x - 40.0).abs() < 1e-6);
        assert!(cp.point_mm.y.abs() < 1e-6);
        assert_eq!(cp.mode, RunMode::Cutting);

        // 2. Simulate user jogging away to (80, 60) while paused, then pressing Continue
        let jogged_pos = kurbo::Point::new(80.0, 60.0);
        let cont_msg = state.continue_run(jogged_pos, &None);
        assert!(cont_msg.is_some());
        assert!(!state.is_paused);
        assert!(state.paused_checkpoint.is_none());
        assert_eq!(state.run_mode, RunMode::Cutting);
        assert_eq!(state.pause_buffer.len(), 1);

        // Expect: [rapid return (80,60)->(40,0), remainder cut (40,0)->(100,0), seg2 (100,0)->(100,50)]
        assert_eq!(state.trajectory.len(), 3);
        let ret = state.trajectory[0];
        assert!(ret.is_rapid);
        assert!((ret.start.x - 80.0).abs() < 1e-6 && (ret.start.y - 60.0).abs() < 1e-6);
        assert!((ret.end.x - 40.0).abs() < 1e-6 && ret.end.y.abs() < 1e-6);
        assert!((ret.length_mm - 72.1110255).abs() < 1e-4);

        let rem = state.trajectory[1];
        assert!(!rem.is_rapid);
        assert!((rem.start.x - 40.0).abs() < 1e-6 && rem.start.y.abs() < 1e-6);
        assert!((rem.end.x - 100.0).abs() < 1e-6 && rem.end.y.abs() < 1e-6);
        assert!((rem.length_mm - 60.0).abs() < 1e-6);

        let seg2 = state.trajectory[2];
        assert!(!seg2.is_rapid);
        assert!((seg2.length_mm - 50.0).abs() < 1e-6);

        // 3. Pause a second time at midpoint of segment 1 ((70, 0)), do not jog, and continue
        state.current_seg_idx = 1;
        state.seg_progress_mm = 30.0;
        assert!(state.pause_run(&None, &mut status, &pulses).is_some());
        assert_eq!(state.pause_buffer.len(), 2);
        let cp2 = state.paused_checkpoint.unwrap();
        assert!((cp2.point_mm.x - 70.0).abs() < 1e-6);

        assert!(state.continue_run(cp2.point_mm, &None).is_some());
        // No rapid return needed since head wasn't jogged
        assert_eq!(state.trajectory.len(), 2);
        assert!(!state.trajectory[0].is_rapid);
        assert!((state.trajectory[0].start.x - 70.0).abs() < 1e-6);
        assert!((state.trajectory[0].length_mm - 30.0).abs() < 1e-6);

        // 4. Stop/reset clears checkpoint and pause buffer
        state.stop_run(&None, &mut status);
        assert_eq!(state.run_mode, RunMode::Idle);
        assert!(state.paused_checkpoint.is_none());
        assert!(state.pause_buffer.is_empty());
    }
}
