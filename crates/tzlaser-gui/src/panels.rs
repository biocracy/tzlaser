use egui::{Button, CollapsingHeader, Color32, Frame, Grid, ProgressBar, RichText, ScrollArea, Ui};
use tzlaser_core::document::closest_point_on_segment;
use tzlaser_core::{Document, MaterialLibrary, MotionSegment, PulseConfig};
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
    pub full_trajectory: Vec<MotionSegment>,
    pub current_seg_idx: usize,
    pub seg_progress_mm: f64,
    pub total_length_mm: f64,
    pub traveled_length_mm: f64,
    pub estimated_job_secs: f64,
    pub last_tick: Option<std::time::Instant>,
    pub last_hw_dispatched_seg: Option<usize>,
    pub hw_seg_dispatched_at: Option<std::time::Instant>,
    pub hw_seg_seen_moving: bool,
    pub hw_fifo_active: bool,
    pub hw_fifo_started_at: Option<std::time::Instant>,
    pub hw_fifo_total_secs: f64,
    pub paused_checkpoint: Option<PauseCheckpoint>,
    pub pause_buffer: Vec<PauseCheckpoint>,
    pub material_db: MaterialLibrary,
    pub active_layer_popup: Option<usize>,
    pub new_mat_name: String,
    pub new_mat_material: String,
    pub new_mat_thickness: f64,
    pub show_home_prompt: bool,
    pub prompted_home_this_connection: bool,
    pub oob_alert: Option<(String, bool)>,
    pub job_start_pos: kurbo::Point,
    pub sim_vx_pls: f32,
    pub sim_vy_pls: f32,
    pub sim_ax_pls: f32,
    pub sim_ay_pls: f32,
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
            full_trajectory: Vec::new(),
            current_seg_idx: 0,
            seg_progress_mm: 0.0,
            total_length_mm: 0.0,
            traveled_length_mm: 0.0,
            estimated_job_secs: 0.0,
            last_tick: None,
            last_hw_dispatched_seg: None,
            hw_seg_dispatched_at: None,
            hw_seg_seen_moving: false,
            hw_fifo_active: false,
            hw_fifo_started_at: None,
            hw_fifo_total_secs: 0.0,
            paused_checkpoint: None,
            pause_buffer: Vec::new(),
            material_db: MaterialLibrary::with_default_presets(),
            active_layer_popup: None,
            new_mat_name: String::new(),
            new_mat_material: "Mild Steel".to_string(),
            new_mat_thickness: 2.0,
            show_home_prompt: false,
            prompted_home_this_connection: false,
            oob_alert: None,
            job_start_pos: kurbo::Point::ZERO,
            sim_vx_pls: 0.0,
            sim_vy_pls: 0.0,
            sim_ax_pls: 0.0,
            sim_ay_pls: 0.0,
        }
    }
}

/// Return the selected bed origin corner in canvas world coordinates (mm),
/// matching `CanvasState::bed_width_mm` (`cut_width_mm`) and `bed_height_mm` (`cut_height_mm`).
pub fn origin_world_mm(pulse_config: &PulseConfig) -> kurbo::Point {
    let w = pulse_config.cut_width_mm();
    let h = pulse_config.cut_height_mm();
    if pulse_config.origin_corner.contains("Top-Left") {
        kurbo::Point::new(0.0, h)
    } else if pulse_config.origin_corner.contains("Bottom-Right") {
        kurbo::Point::new(w, 0.0)
    } else if pulse_config.origin_corner.contains("Top-Right") {
        kurbo::Point::new(w, h)
    } else {
        kurbo::Point::ZERO
    }
}

/// Return the current cutting head position in canvas world coordinates (mm).
pub fn head_world_mm(status: &MachineStatus, pulse_config: &PulseConfig) -> kurbo::Point {
    let origin = origin_world_mm(pulse_config);
    kurbo::Point::new(
        origin.x + pulse_config.pulses_to_mm_x(status.pos_x),
        origin.y + pulse_config.pulses_to_mm_y(status.pos_y),
    )
}

/// Convert a `MotionSegment` trajectory into a continuous 1-ms Motion FIFO tick stream `(dx, dy, flags)`.
/// Groups smooth consecutive segments (e.g. circle/arc chords) into continuous chains with
/// trapezoidal acceleration/deceleration ramps so curves flow without stopping and corners/starts
/// accelerate smoothly without shocking the machine.
pub fn build_trajectory_fifo_ticks(
    traj: &[MotionSegment],
    origin_world: kurbo::Point,
    pulses_cfg: &PulseConfig,
    start_px: i32,
    start_py: i32,
) -> Vec<(i8, i8, u8)> {
    let mut ticks = Vec::new();
    let mut cur_px = start_px;
    let mut cur_py = start_py;
    // Development safety: cap Dry-Cut to 5 mm/s and gentle 25 mm/s^2 acceleration
    let dev_speed_mm_s = 5.0_f64;
    let accel = pulses_cfg.jog_accel_mm_s2.clamp(10.0, 25.0);

    let segs: Vec<&MotionSegment> = traj.iter().filter(|s| s.length_mm > 1e-6).collect();
    if segs.is_empty() {
        return ticks;
    }

    // SC2000 Motion FIFO stream preamble: tick 0 = 0x30, followed by 58 zero-motion 0x02 warmup ticks
    ticks.push((0, 0, 0x30));
    for _ in 0..58 {
        ticks.push((0, 0, 0x02));
    }

    let seg_dir = |s: &MotionSegment| -> (f64, f64) {
        let dx = s.end.x - s.start.x;
        let dy = s.end.y - s.start.y;
        let len = (dx * dx + dy * dy).sqrt().max(1e-9);
        (dx / len, dy / len)
    };

    // Group into continuous chains (break on rapid transitions or sharp corners > ~30 deg)
    let mut idx = 0;
    while idx < segs.len() {
        let chain_start = idx;
        let is_rapid = segs[idx].is_rapid;
        let mut chain_end = idx + 1;
        while chain_end < segs.len() {
            if segs[chain_end].is_rapid != is_rapid {
                break;
            }
            let (ux, uy) = seg_dir(segs[chain_end - 1]);
            let (vx, vy) = seg_dir(segs[chain_end]);
            if ux * vx + uy * vy < 0.866 {
                break;
            }
            chain_end += 1;
        }

        let chain = &segs[chain_start..chain_end];
        let mut cum_len = Vec::with_capacity(chain.len() + 1);
        cum_len.push(0.0_f64);
        let mut total_len = 0.0_f64;
        for s in chain {
            total_len += s.length_mm;
            cum_len.push(total_len);
        }
        let v_target = dev_speed_mm_s;
        let half_pi = std::f64::consts::FRAC_PI_2;
        // Raised-cosine S-curve ramp: s(T_a) = 0.5 * v_peak * T_a, peak_accel = (PI/2) * v_peak / T_a
        // Setting T_a = (PI/2) * v_peak / accel ensures peak acceleration <= accel and jerk is C1-smooth.
        let d_ramp_full = half_pi * (v_target * v_target) / (2.0 * accel);
        let (v_peak, t_accel, d_accel, t_cruise, total_t) = if 2.0 * d_ramp_full >= total_len {
            let vp = ((total_len * accel) / half_pi).sqrt().max(0.5);
            let ta = half_pi * vp / accel;
            (vp, ta, 0.5 * total_len, 0.0, 2.0 * ta)
        } else {
            let ta = half_pi * v_target / accel;
            let dc = total_len - 2.0 * d_ramp_full;
            let tc = dc / v_target;
            (v_target, ta, d_ramp_full, tc, 2.0 * ta + tc)
        };

        let end_px = pulses_cfg.mm_to_pulses_x(chain.last().unwrap().end.x - origin_world.x);
        let end_py = pulses_cfg.mm_to_pulses_y(chain.last().unwrap().end.y - origin_world.y);
        let min_ticks_for_step =
            (((end_px - cur_px).abs().max((end_py - cur_py).abs()) as usize) + 126) / 127;
        let n_ticks = ((total_t * 1000.0).ceil() as usize)
            .max(min_ticks_for_step)
            .max(1);

        let mut seg_cursor = 0usize;
        for k in 1..=n_ticks {
            let s_dist = if k == n_ticks {
                total_len
            } else {
                let t = (k as f64 / n_ticks as f64) * total_t;
                if t < t_accel {
                    0.5 * v_peak * (t - (t_accel / std::f64::consts::PI) * (std::f64::consts::PI * t / t_accel).sin())
                } else if t < t_accel + t_cruise {
                    d_accel + v_peak * (t - t_accel)
                } else {
                    let rem = (total_t - t).max(0.0);
                    let s_rem = 0.5 * v_peak * (rem - (t_accel / std::f64::consts::PI) * (std::f64::consts::PI * rem / t_accel).sin());
                    (total_len - s_rem).min(total_len)
                }
            };

            while seg_cursor + 1 < chain.len() && s_dist > cum_len[seg_cursor + 1] {
                seg_cursor += 1;
            }
            let s0 = cum_len[seg_cursor];
            let s1 = cum_len[seg_cursor + 1];
            let frac = if s1 > s0 + 1e-9 {
                ((s_dist - s0) / (s1 - s0)).clamp(0.0, 1.0)
            } else {
                1.0
            };
            let seg = chain[seg_cursor];
            let wx = seg.start.x + (seg.end.x - seg.start.x) * frac;
            let wy = seg.start.y + (seg.end.y - seg.start.y) * frac;
            let next_px = pulses_cfg.mm_to_pulses_x(wx - origin_world.x);
            let next_py = pulses_cfg.mm_to_pulses_y(wy - origin_world.y);
            // SC2000 Motion FIFO ticks are subtracted from R1008/R1009 by the controller
            // (positive FIFO dx/dy moves R1008/R1009 in the negative direction).
            let tick_dx = (cur_px - next_px).clamp(-127, 127) as i8;
            let tick_dy = (cur_py - next_py).clamp(-127, 127) as i8;
            cur_px -= tick_dx as i32;
            cur_py -= tick_dy as i32;
            let flags = if is_rapid { 0x03 } else { 0x01 };
            ticks.push((tick_dx, tick_dy, flags));
        }

        // Flush any remaining quantized pulse difference at the chain end
        while cur_px != end_px || cur_py != end_py {
            let tick_dx = (cur_px - end_px).clamp(-127, 127) as i8;
            let tick_dy = (cur_py - end_py).clamp(-127, 127) as i8;
            cur_px -= tick_dx as i32;
            cur_py -= tick_dy as i32;
            let flags = if is_rapid { 0x03 } else { 0x01 };
            ticks.push((tick_dx, tick_dy, flags));
        }

        idx = chain_end;
    }

    // SC2000 Motion FIFO stream end-of-stream marker
    ticks.push((0, 0, 0xF0));
    ticks
}

/// Build `HomeConfig` matching the selected `origin_corner` and pulse scaling in `PulseConfig`.
pub fn home_config_from_pulse(pulse_config: &PulseConfig) -> tzlaser_protocol::HomeConfig {
    let x_dir = if pulse_config.origin_corner.contains("Right") {
        1
    } else {
        -1
    };
    let y_dir = if pulse_config.origin_corner.contains("Top") {
        1
    } else {
        -1
    };
    let ppm = pulse_config
        .pulses_per_mm_x
        .max(pulse_config.pulses_per_mm_y)
        .max(10.0);
    let max_dim = pulse_config
        .table_width_mm
        .max(pulse_config.table_height_mm)
        .max(500.0);
    tzlaser_protocol::HomeConfig {
        x_dir,
        y_dir,
        fast_speed_pls_s: ((20.0 * ppm).round() as u32).max(1_000),
        slow_speed_pls_s: ((2.0 * ppm).round() as u32).max(100),
        max_search_pulses: ((max_dim * 1.2 * ppm).round() as u32).max(50_000),
    }
}

/// Format a duration in seconds into a compact human-readable string (e.g. `23.0s`, `01:45`, `1:02:15`).
/// Verify all active cuttable toolpaths fit within the effective cutting area
/// (`table_width_mm - home_retract_x_mm` × `table_height_mm - home_retract_y_mm`) when soft limits are enabled.
/// Return true if a world-coordinate point (mm) lies within the effective cutting area `[0..cut_w] x [0..cut_h]`.
pub fn point_within_cut_area(pt: kurbo::Point, pulses_cfg: &PulseConfig) -> bool {
    if !pt.x.is_finite() || !pt.y.is_finite() {
        return false;
    }
    let cut_w = pulses_cfg.cut_width_mm().max(1.0);
    let cut_h = pulses_cfg.cut_height_mm().max(1.0);
    let tol = 0.05;
    pt.x >= -tol && pt.x <= cut_w + tol && pt.y >= -tol && pt.y <= cut_h + tol
}

pub fn check_cut_area_bounds(doc: &Document, pulses_cfg: &PulseConfig) -> Result<(), String> {
    let cut_w = pulses_cfg.cut_width_mm().max(1.0);
    let cut_h = pulses_cfg.cut_height_mm().max(1.0);
    let tol = 0.05;
    for layer in &doc.layers {
        if !layer.visible || layer.is_obstacle {
            continue;
        }
        for ent in &layer.entities {
            let bb = ent.bounding_box();
            if bb.x0 < -tol || bb.x1 > cut_w + tol || bb.y0 < -tol || bb.y1 > cut_h + tol {
                return Err(format!(
                    "Graphics extend outside the effective cutting area ({:.1} × {:.1} mm). Bounding box: X [{:.1}..{:.1}], Y [{:.1}..{:.1}] mm.",
                    cut_w, cut_h, bb.x0, bb.x1, bb.y0, bb.y1
                ));
            }
        }
    }
    Ok(())
}

pub fn format_duration_secs(secs: f64) -> String {
    let s = secs.max(0.0);
    if s < 60.0 {
        format!("{:.1}s", s)
    } else {
        let total = s.round() as u64;
        let hrs = total / 3600;
        let mins = (total % 3600) / 60;
        let rem_s = total % 60;
        if hrs > 0 {
            format!("{}:{:02}:{:02}", hrs, mins, rem_s)
        } else {
            format!("{:02}:{:02}", mins, rem_s)
        }
    }
}

impl PanelState {
    /// Return the 1-based cutting index of the active/paused path while a job is active or paused,
    /// so all paths with a smaller cutting index can be rendered at 30% alpha until Stop or completion.
    pub fn completed_before_path_1based(&self) -> Option<usize> {
        if self.run_mode == RunMode::Idle {
            return None;
        }
        let seg = self.trajectory.get(self.current_seg_idx)?;
        if self.current_seg_idx > 0
            && self.trajectory[self.current_seg_idx..]
                .iter()
                .all(|s| s.is_rapid)
        {
            Some(seg.path_index_1based + 1)
        } else {
            Some(seg.path_index_1based)
        }
    }

    /// Stop any active motion, reset head position to origin (0, 0), show an alert message,
    /// and prompt for re-homing if connected to physical hardware (offline requires no re-homing).
    pub fn trigger_out_of_bounds_recovery(
        &mut self,
        reason: &str,
        controller: &Option<LaserController>,
        status: &mut MachineStatus,
    ) -> String {
        self.stop_run(controller, status);
        status.pos_x = 0;
        status.pos_y = 0;
        let hw_connected = controller.as_ref().map_or(false, |c| c.is_connected());
        if hw_connected {
            status.xy_homed = false;
            self.show_home_prompt = true;
            self.oob_alert = Some((
                format!(
                    "{}\n\nThe head position has been reset to Origin (0, 0). Please re-home the machine before continuing.",
                    reason
                ),
                true,
            ));
            format!("⚠️ {} Head reset to Origin — re-homing required!", reason)
        } else {
            self.oob_alert = Some((
                format!(
                    "{}\n\nThe head position has been reset to Origin (0, 0). (Offline simulation: no re-homing necessary.)",
                    reason
                ),
                false,
            ));
            format!("⚠️ {} Head reset to Origin (0, 0).", reason)
        }
    }

    /// Validate that both the current head position and all visible graphics lie inside the cutting area.
    /// If not, triggers out-of-bounds recovery and returns Err(status_message).
    pub fn validate_motion_bounds(
        &mut self,
        doc: &Document,
        current_head: kurbo::Point,
        controller: &Option<LaserController>,
        status: &mut MachineStatus,
        pulses_cfg: &PulseConfig,
    ) -> Result<(), String> {
        if !point_within_cut_area(current_head, pulses_cfg) {
            let msg = format!(
                "Cutting head position ({:.1}, {:.1} mm) is outside the effective cutting area ({:.1} × {:.1} mm).",
                current_head.x,
                current_head.y,
                pulses_cfg.cut_width_mm(),
                pulses_cfg.cut_height_mm()
            );
            return Err(self.trigger_out_of_bounds_recovery(&msg, controller, status));
        }
        if let Err(err) = check_cut_area_bounds(doc, pulses_cfg) {
            return Err(self.trigger_out_of_bounds_recovery(&err, controller, status));
        }
        Ok(())
    }

    /// Apply an acceleration-filtered (jerk-smoothed & magnitude-clamped) motion filter
    /// so commanded velocity transitions smoothly without abrupt acceleration spikes.
    pub fn step_filtered_velocity(
        &mut self,
        target_vx: f32,
        target_vy: f32,
        max_accel_pls_s2: f32,
        dt_s: f32,
    ) -> (f32, f32) {
        let dt = dt_s.clamp(0.005, 0.08);
        let a_limit = max_accel_pls_s2.max(1_000.0);
        // Desired acceleration proportional to velocity error, clamped to ±a_limit
        let tau_v = 0.08_f32;
        let raw_ax = ((target_vx - self.sim_vx_pls) / tau_v).clamp(-a_limit, a_limit);
        let raw_ay = ((target_vy - self.sim_vy_pls) / tau_v).clamp(-a_limit, a_limit);
        // Low-pass filter on acceleration itself (tau_a = 45 ms) to eliminate jerk spikes
        let alpha = (dt / (0.045 + dt)).clamp(0.05, 0.75);
        self.sim_ax_pls += alpha * (raw_ax - self.sim_ax_pls);
        self.sim_ay_pls += alpha * (raw_ay - self.sim_ay_pls);
        self.sim_vx_pls += self.sim_ax_pls * dt;
        self.sim_vy_pls += self.sim_ay_pls * dt;
        (self.sim_vx_pls, self.sim_vy_pls)
    }

    pub fn reset_filtered_velocity(&mut self) {
        self.sim_vx_pls = 0.0;
        self.sim_vy_pls = 0.0;
        self.sim_ax_pls = 0.0;
        self.sim_ay_pls = 0.0;
    }

    pub fn start_run(
        &mut self,
        mode: RunMode,
        doc: &Document,
        current_head: kurbo::Point,
        controller: &Option<LaserController>,
    ) -> String {
        let traj = doc.build_motion_trajectory(current_head, 20.0);
        if traj.is_empty() {
            return "No active toolpaths to run on visible layers.".to_string();
        }
        self.total_length_mm = traj.iter().map(|s| s.length_mm).sum();
        self.job_start_pos = current_head;
        let est = doc.estimate_job_time(current_head);
        self.estimated_job_secs = est.total_s;
        self.full_trajectory = traj.clone();
        self.trajectory = traj;
        self.current_seg_idx = 0;
        self.seg_progress_mm = 0.0;
        self.traveled_length_mm = 0.0;
        self.is_paused = false;
        self.run_mode = mode;
        self.last_tick = Some(std::time::Instant::now());
        self.last_hw_dispatched_seg = None;
        self.hw_seg_dispatched_at = None;
        self.hw_seg_seen_moving = false;
        self.hw_fifo_active = false;
        self.hw_fifo_started_at = None;
        self.hw_fifo_total_secs = 0.0;
        self.paused_checkpoint = None;
        self.pause_buffer.clear();
        self.reset_filtered_velocity();

        let est_str = format_duration_secs(self.estimated_job_secs);
        if mode == RunMode::DryCut {
            if let Some(ref c) = controller {
                if c.is_connected() {
                    c.set_red_pointer(true);
                }
            }
            format!(
                "Dry-Cut simulation started ({} segments, {:.1} mm, Est: {}).",
                self.trajectory.len(),
                self.total_length_mm,
                est_str
            )
        } else {
            format!(
                "Cutting job started ({} segments, {:.1} mm, Est: {}).",
                self.trajectory.len(),
                self.total_length_mm,
                est_str
            )
        }
    }

    /// Convert the active trajectory into a 1-ms Motion FIFO tick stream and dispatch it
    /// to the connected controller for continuous, stutter-free Dry-Cut curve tracing.
    pub fn start_hw_dry_cut_fifo(
        &mut self,
        controller: &Option<LaserController>,
        status: &MachineStatus,
        pulses_cfg: &PulseConfig,
    ) {
        let Some(ref c) = controller else {
            return;
        };
        if !c.is_connected() || self.trajectory.is_empty() {
            return;
        }
        let ticks = build_trajectory_fifo_ticks(
            &self.trajectory,
            origin_world_mm(pulses_cfg),
            pulses_cfg,
            status.pos_x,
            status.pos_y,
        );
        if ticks.is_empty() {
            return;
        }
        self.hw_fifo_total_secs = ticks.len() as f64 / 1000.0;
        self.hw_fifo_started_at = Some(std::time::Instant::now());
        self.hw_fifo_active = true;
        self.hw_seg_seen_moving = false;
        c.start_fifo_stream(ticks);
    }

    pub fn stop_run(&mut self, controller: &Option<LaserController>, status: &mut MachineStatus) {
        let was_dry = self.run_mode == RunMode::DryCut;
        self.hw_fifo_active = false;
        self.hw_fifo_started_at = None;
        self.hw_seg_seen_moving = false;
        self.run_mode = RunMode::Idle;
        self.is_paused = false;
        self.last_tick = None;
        self.last_hw_dispatched_seg = None;
        self.full_trajectory.clear();
        self.estimated_job_secs = 0.0;
        self.paused_checkpoint = None;
        self.pause_buffer.clear();
        self.reset_filtered_velocity();
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

        if self.full_trajectory.is_empty() {
            self.full_trajectory = self.trajectory.clone();
        }

        let seg = self.trajectory[self.current_seg_idx];
        let frac = (self.seg_progress_mm / seg.length_mm.max(1e-9)).clamp(0.0, 1.0);
        let point_mm = kurbo::Point::new(
            seg.start.x + (seg.end.x - seg.start.x) * frac,
            seg.start.y + (seg.end.y - seg.start.y) * frac,
        );

        let origin_world = origin_world_mm(pulses_cfg);
        status.pos_x = pulses_cfg.mm_to_pulses_x(point_mm.x - origin_world.x);
        status.pos_y = pulses_cfg.mm_to_pulses_y(point_mm.y - origin_world.y);
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

    /// Step the head forward or backward along the cutting profile path while paused.
    /// Clamps to the current profile's endpoint/start-point when remaining distance < `step_mm`,
    /// and transitions to the next/previous profile when already at a start/endpoint.
    pub fn step_along_path(
        &mut self,
        forward: bool,
        step_mm: f64,
        controller: &Option<LaserController>,
        status: &mut MachineStatus,
        pulses_cfg: &PulseConfig,
    ) -> Option<String> {
        if !self.is_paused || step_mm <= 0.0 {
            return None;
        }
        let cp = self.paused_checkpoint?;

        if self.full_trajectory.is_empty() {
            self.full_trajectory = self.trajectory.clone();
        }
        if self.full_trajectory.is_empty() {
            return None;
        }

        struct ProfileSpan {
            path_index_1based: usize,
            first_seg_idx: usize,
            last_seg_idx: usize,
            total_len_mm: f64,
        }

        let mut spans: Vec<ProfileSpan> = Vec::new();
        for (idx, seg) in self.full_trajectory.iter().enumerate() {
            if seg.is_rapid {
                continue;
            }
            if let Some(last) = spans.last_mut() {
                if last.path_index_1based == seg.path_index_1based && last.last_seg_idx + 1 == idx {
                    last.last_seg_idx = idx;
                    last.total_len_mm += seg.length_mm;
                    continue;
                }
            }
            spans.push(ProfileSpan {
                path_index_1based: seg.path_index_1based,
                first_seg_idx: idx,
                last_seg_idx: idx,
                total_len_mm: seg.length_mm,
            });
        }
        if spans.is_empty() {
            for (idx, seg) in self.full_trajectory.iter().enumerate() {
                spans.push(ProfileSpan {
                    path_index_1based: seg.path_index_1based,
                    first_seg_idx: idx,
                    last_seg_idx: idx,
                    total_len_mm: seg.length_mm,
                });
            }
        }
        if spans.is_empty() {
            return None;
        }

        let tail = self.trajectory.len().saturating_sub(cp.seg_idx);
        let mapped_full_idx = self.full_trajectory.len().checked_sub(tail);

        let (cur_profile_idx, cur_s) = if let Some(full_idx) = mapped_full_idx {
            if let Some(p_idx) = spans
                .iter()
                .position(|sp| full_idx >= sp.first_seg_idx && full_idx <= sp.last_seg_idx)
            {
                let sp = &spans[p_idx];
                let mut s = 0.0;
                for i in sp.first_seg_idx..full_idx {
                    s += self.full_trajectory[i].length_mm;
                }
                let local_prog = if let Some(cur_seg) = self.trajectory.get(cp.seg_idx) {
                    let rem = (cur_seg.length_mm - cp.seg_progress_mm).max(0.0);
                    (self.full_trajectory[full_idx].length_mm - rem)
                        .clamp(0.0, self.full_trajectory[full_idx].length_mm)
                } else {
                    0.0
                };
                (p_idx, (s + local_prog).clamp(0.0, sp.total_len_mm))
            } else if let Some(p_idx) = spans
                .iter()
                .position(|sp| sp.path_index_1based == cp.path_index_1based)
            {
                let sp = &spans[p_idx];
                let s = if full_idx > sp.last_seg_idx {
                    sp.total_len_mm
                } else {
                    0.0
                };
                (p_idx, s)
            } else {
                (0, 0.0)
            }
        } else if let Some(p_idx) = spans
            .iter()
            .position(|sp| sp.path_index_1based == cp.path_index_1based)
        {
            (p_idx, 0.0)
        } else {
            (0, 0.0)
        };

        let sp = &spans[cur_profile_idx];
        let eps = 1e-4;
        let delta = step_mm;
        let (target_profile_idx, target_s, crossed) = if forward {
            if cur_s < sp.total_len_mm - eps {
                (cur_profile_idx, (cur_s + delta).min(sp.total_len_mm), false)
            } else if cur_profile_idx + 1 < spans.len() {
                let next_sp = &spans[cur_profile_idx + 1];
                (cur_profile_idx + 1, delta.min(next_sp.total_len_mm), true)
            } else {
                return Some(format!(
                    "Already at endpoint of final profile (Path #{}).",
                    sp.path_index_1based
                ));
            }
        } else if cur_s > eps {
            (cur_profile_idx, (cur_s - delta).max(0.0), false)
        } else if cur_profile_idx > 0 {
            let prev_sp = &spans[cur_profile_idx - 1];
            (cur_profile_idx - 1, (prev_sp.total_len_mm - delta).max(0.0), true)
        } else {
            return Some(format!(
                "Already at start-point of first profile (Path #{}).",
                sp.path_index_1based
            ));
        };

        let target_sp = &spans[target_profile_idx];
        let mut accum = 0.0;
        let mut target_seg_idx = target_sp.last_seg_idx;
        let mut local_prog = self.full_trajectory[target_seg_idx].length_mm;
        for i in target_sp.first_seg_idx..=target_sp.last_seg_idx {
            let seg = self.full_trajectory[i];
            if target_s <= accum + seg.length_mm + 1e-9 || i == target_sp.last_seg_idx {
                target_seg_idx = i;
                local_prog = (target_s - accum).clamp(0.0, seg.length_mm);
                break;
            }
            accum += seg.length_mm;
        }

        let t_seg = self.full_trajectory[target_seg_idx];
        let frac = (local_prog / t_seg.length_mm.max(1e-9)).clamp(0.0, 1.0);
        let point_mm = kurbo::Point::new(
            t_seg.start.x + (t_seg.end.x - t_seg.start.x) * frac,
            t_seg.start.y + (t_seg.end.y - t_seg.start.y) * frac,
        );

        let origin_world = origin_world_mm(pulses_cfg);
        let new_px = pulses_cfg.mm_to_pulses_x(point_mm.x - origin_world.x);
        let new_py = pulses_cfg.mm_to_pulses_y(point_mm.y - origin_world.y);
        if let Some(ref c) = controller {
            if c.is_connected() {
                let dx_p = new_px - status.pos_x;
                let dy_p = new_py - status.pos_y;
                if dx_p != 0 {
                    c.jog(Axis::X, dx_p);
                }
                if dy_p != 0 {
                    c.jog(Axis::Y, dy_p);
                }
            }
        }
        status.pos_x = new_px;
        status.pos_y = new_py;
        status.laser_on = false;

        let new_traveled: f64 = self.full_trajectory[..target_seg_idx]
            .iter()
            .map(|s| s.length_mm)
            .sum::<f64>()
            + local_prog;
        let full_total: f64 = self.full_trajectory.iter().map(|s| s.length_mm).sum();

        self.trajectory = self.full_trajectory.clone();
        self.current_seg_idx = target_seg_idx;
        self.seg_progress_mm = local_prog;
        self.traveled_length_mm = new_traveled;
        self.total_length_mm = full_total;
        self.last_hw_dispatched_seg = None;

        let new_cp = PauseCheckpoint {
            point_mm,
            seg_idx: target_seg_idx,
            seg_progress_mm: local_prog,
            traveled_length_mm: new_traveled,
            path_index_1based: target_sp.path_index_1based,
            mode: cp.mode,
        };
        self.paused_checkpoint = Some(new_cp);
        if let Some(last) = self.pause_buffer.last_mut() {
            *last = new_cp;
        } else {
            self.pause_buffer.push(new_cp);
        }

        let dir_label = if forward { "Forward" } else { "Backward" };
        if crossed {
            Some(format!(
                "Stepped {} to Path #{} ({:.1}/{:.1} mm) @ X: {:.2}, Y: {:.2} mm.",
                dir_label, target_sp.path_index_1based, target_s, target_sp.total_len_mm, point_mm.x, point_mm.y
            ))
        } else {
            Some(format!(
                "Stepped {} along Path #{} ({:.1}/{:.1} mm) @ X: {:.2}, Y: {:.2} mm.",
                dir_label, target_sp.path_index_1based, target_s, target_sp.total_len_mm, point_mm.x, point_mm.y
            ))
        }
    }

    /// Resume from a recorded pause checkpoint. If the head was jogged away while paused,
    /// prepends a rapid laser-OFF return segment back to the recorded pause position before
    /// continuing the interrupted segment and all remaining paths.
    pub fn continue_run(
        &mut self,
        doc: &Document,
        current_head: kurbo::Point,
        controller: &Option<LaserController>,
    ) -> Option<String> {
        let cp = self.paused_checkpoint.take()?;
        let exclusion_polys = doc.all_obstacle_exclusion_polygons();
        let rapid_speed = doc.travel_speed_mm_s(20.0);

        let mut new_traj: Vec<MotionSegment> = Vec::new();
        let return_dist = current_head.distance(cp.point_mm);
        if return_dist > 1e-4 {
            let waypoints = Document::route_rapid_between(current_head, cp.point_mm, &exclusion_polys);
            for pair in waypoints.windows(2) {
                let len = pair[0].distance(pair[1]);
                if len > 1e-4 {
                    new_traj.push(MotionSegment {
                        start: pair[0],
                        end: pair[1],
                        length_mm: len,
                        speed_mm_s: rapid_speed,
                        is_rapid: true,
                        path_index_1based: cp.path_index_1based,
                    });
                }
            }
        }

        if cp.seg_idx < self.trajectory.len() {
            let orig_seg = self.trajectory[cp.seg_idx];
            let rem_len = cp.point_mm.distance(orig_seg.end);
            if rem_len > 1e-6 {
                if orig_seg.is_rapid && !exclusion_polys.is_empty() {
                    let waypoints = Document::route_rapid_between(cp.point_mm, orig_seg.end, &exclusion_polys);
                    for pair in waypoints.windows(2) {
                        let len = pair[0].distance(pair[1]);
                        if len > 1e-4 {
                            new_traj.push(MotionSegment {
                                start: pair[0],
                                end: pair[1],
                                length_mm: len,
                                speed_mm_s: orig_seg.speed_mm_s,
                                is_rapid: true,
                                path_index_1based: orig_seg.path_index_1based,
                            });
                        }
                    }
                } else {
                    new_traj.push(MotionSegment {
                        start: cp.point_mm,
                        end: orig_seg.end,
                        length_mm: rem_len,
                        speed_mm_s: orig_seg.speed_mm_s,
                        is_rapid: orig_seg.is_rapid,
                        path_index_1based: orig_seg.path_index_1based,
                    });
                }
            }
            if cp.seg_idx + 1 < self.trajectory.len() {
                if exclusion_polys.is_empty() {
                    new_traj.extend_from_slice(&self.trajectory[(cp.seg_idx + 1)..]);
                } else {
                    // Re-route any remaining rapid segments in case an obstacle moved while paused
                    let rest = &self.trajectory[(cp.seg_idx + 1)..];
                    let mut i = 0;
                    while i < rest.len() {
                        if !rest[i].is_rapid {
                            new_traj.push(rest[i]);
                            i += 1;
                        } else {
                            let rapid_start = rest[i].start;
                            let path_idx = rest[i].path_index_1based;
                            let spd = rest[i].speed_mm_s;
                            let mut j = i;
                            while j + 1 < rest.len()
                                && rest[j + 1].is_rapid
                                && rest[j + 1].path_index_1based == path_idx
                            {
                                j += 1;
                            }
                            let rapid_end = rest[j].end;
                            let waypoints =
                                Document::route_rapid_between(rapid_start, rapid_end, &exclusion_polys);
                            for pair in waypoints.windows(2) {
                                let len = pair[0].distance(pair[1]);
                                if len > 1e-4 {
                                    new_traj.push(MotionSegment {
                                        start: pair[0],
                                        end: pair[1],
                                        length_mm: len,
                                        speed_mm_s: spd,
                                        is_rapid: true,
                                        path_index_1based: path_idx,
                                    });
                                }
                            }
                            i = j + 1;
                        }
                    }
                }
            }
        }

        if new_traj.is_empty() {
            self.run_mode = RunMode::Idle;
            self.is_paused = false;
            self.trajectory.clear();
            self.full_trajectory.clear();
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

    /// Move the head to the closest point on `doc`'s cutting path near `target_pt_mm`
    /// and enter Pause mode at that exact trajectory position (valid when Stopped or Paused).
    pub fn set_pause_at_path_point(
        &mut self,
        doc: &Document,
        target_pt_mm: kurbo::Point,
        controller: &Option<LaserController>,
        status: &mut MachineStatus,
        pulses_cfg: &PulseConfig,
    ) -> Option<String> {
        if self.run_mode != RunMode::Idle && !self.is_paused {
            return None;
        }

        let hw_connected = controller.as_ref().map_or(false, |c| c.is_connected());
        if self.run_mode == RunMode::Idle || self.full_trajectory.is_empty() {
            let origin_head = head_world_mm(status, pulses_cfg);
            let traj = doc.build_motion_trajectory(origin_head, 200.0);
            if traj.is_empty() {
                return None;
            }
            self.full_trajectory = traj;
        }

        let mut best: Option<(usize, kurbo::Point, f64, f64)> = None;
        for only_cutting in [true, false] {
            for (idx, seg) in self.full_trajectory.iter().enumerate() {
                if only_cutting && seg.is_rapid {
                    continue;
                }
                let (proj, dist, t) = closest_point_on_segment(target_pt_mm, seg.start, seg.end);
                if best.map_or(true, |(_, _, _, b_dist)| dist < b_dist) {
                    let prog = (t * seg.length_mm).clamp(0.0, seg.length_mm);
                    best = Some((idx, proj, prog, dist));
                }
            }
            if best.is_some() {
                break;
            }
        }

        let (best_seg_idx, best_pt, best_prog_mm, _) = best?;
        let best_seg = self.full_trajectory[best_seg_idx];
        let new_traveled: f64 = self.full_trajectory[..best_seg_idx]
            .iter()
            .map(|s| s.length_mm)
            .sum::<f64>()
            + best_prog_mm;
        let full_total: f64 = self.full_trajectory.iter().map(|s| s.length_mm).sum();

        if let Some(ref c) = controller {
            if c.is_connected() {
                c.stop();
            }
        }
        status.laser_on = false;

        if self.run_mode == RunMode::Idle {
            self.run_mode = if hw_connected {
                RunMode::Cutting
            } else {
                RunMode::DryCut
            };
        }
        if self.run_mode == RunMode::DryCut {
            status.red_pointer_on = true;
            if let Some(ref c) = controller {
                if c.is_connected() {
                    c.set_red_pointer(true);
                }
            }
        }

        self.trajectory = self.full_trajectory.clone();
        self.current_seg_idx = best_seg_idx;
        self.seg_progress_mm = best_prog_mm;
        self.traveled_length_mm = new_traveled;
        self.total_length_mm = full_total;
        self.is_paused = true;
        self.last_tick = None;
        self.last_hw_dispatched_seg = None;

        let cp = PauseCheckpoint {
            point_mm: best_pt,
            seg_idx: best_seg_idx,
            seg_progress_mm: best_prog_mm,
            traveled_length_mm: new_traveled,
            path_index_1based: best_seg.path_index_1based,
            mode: self.run_mode,
        };
        self.paused_checkpoint = Some(cp);
        self.pause_buffer.push(cp);

        Some(format!(
            "Resume point set on Path #{} @ X: {:.2}, Y: {:.2} mm. Press Continue to traverse and resume.",
            best_seg.path_index_1based, best_pt.x, best_pt.y
        ))
    }

    /// Synchronize updated layer cutting & travel speeds into active and buffered trajectories
    /// so speed edits in Cutting Layers take effect immediately (even while running or paused).
    pub fn sync_layer_speeds(&mut self, doc: &Document) {
        let seq = doc.compute_cut_sequence();
        let travel_speed = doc.travel_speed_mm_s(20.0);
        for traj in [&mut self.trajectory, &mut self.full_trajectory] {
            for seg in traj.iter_mut() {
                if seg.is_rapid {
                    seg.speed_mm_s = travel_speed;
                    continue;
                }
                if seg.path_index_1based == 0 {
                    continue;
                }
                if let Some(info) = seq.get(seg.path_index_1based - 1) {
                    if let Some(layer) = doc.layers.get(info.layer_idx) {
                        seg.speed_mm_s = layer.params.speed_mm_s.max(0.5);
                    }
                }
            }
        }
        if !self.full_trajectory.is_empty() {
            let motion_s: f64 = self
                .full_trajectory
                .iter()
                .map(|s| {
                    let denom = if s.is_rapid {
                        s.speed_mm_s.max(1.0)
                    } else {
                        s.speed_mm_s.max(0.5)
                    };
                    s.length_mm / denom
                })
                .sum();
            let pierce_s: f64 = seq
                .iter()
                .filter_map(|info| doc.layers.get(info.layer_idx))
                .map(|layer| (layer.params.pierce_time_ms as f64) / 1000.0)
                .sum();
            self.estimated_job_secs = motion_s + pierce_s;
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
        if hw_connected && self.hw_fifo_active {
            if status.is_moving {
                self.hw_seg_seen_moving = true;
            }
            let head_world = head_world_mm(status, pulses_cfg);
            while self.current_seg_idx < self.trajectory.len() {
                let seg = self.trajectory[self.current_seg_idx];
                if head_world.distance(seg.end) <= 0.8 {
                    self.traveled_length_mm += seg.length_mm;
                    self.seg_progress_mm = 0.0;
                    self.current_seg_idx += 1;
                } else {
                    self.seg_progress_mm = (seg.length_mm - head_world.distance(seg.end)).clamp(0.0, seg.length_mm);
                    break;
                }
            }
            let elapsed = self
                .hw_fifo_started_at
                .map(|t| (now - t).as_secs_f64())
                .unwrap_or(0.0);
            let fifo_done = (self.hw_seg_seen_moving && !status.is_moving)
                || (!status.is_moving && elapsed >= self.hw_fifo_total_secs + 0.5);
            if fifo_done {
                self.current_seg_idx = self.trajectory.len();
                self.traveled_length_mm = self.total_length_mm;
                self.hw_fifo_active = false;
            }
        } else if hw_connected {
            while self.current_seg_idx < self.trajectory.len() {
                let seg = self.trajectory[self.current_seg_idx];
                let head_world = head_world_mm(status, pulses_cfg);

                if self.last_hw_dispatched_seg != Some(self.current_seg_idx) {
                    let dx_p = pulses_cfg.mm_to_pulses_x(seg.end.x - head_world.x);
                    let dy_p = pulses_cfg.mm_to_pulses_y(seg.end.y - head_world.y);
                    if dx_p == 0 && dy_p == 0 {
                        self.traveled_length_mm += seg.length_mm;
                        self.seg_progress_mm = 0.0;
                        self.current_seg_idx += 1;
                        self.hw_seg_dispatched_at = None;
                        self.hw_seg_seen_moving = false;
                        continue;
                    }
                    if let Some(ref c) = controller {
                        let ppm = pulses_cfg
                            .pulses_per_mm_x
                            .max(pulses_cfg.pulses_per_mm_y)
                            .max(10.0);
                        let speed_pls = (seg.speed_mm_s.max(0.5) * ppm)
                            .round()
                            .clamp(200.0, 120_000.0) as u32;
                        let move_cfg = tzlaser_protocol::JogConfig {
                            min_speed: (speed_pls / 10).max(100),
                            jog_speed: speed_pls,
                            accel: (speed_pls * 4).clamp(2_000, 100_000),
                        };
                        c.move_xy(dx_p, dy_p, move_cfg);
                    }
                    self.last_hw_dispatched_seg = Some(self.current_seg_idx);
                    self.hw_seg_dispatched_at = Some(now);
                    self.hw_seg_seen_moving = false;
                    self.seg_progress_mm = 0.0;
                    break;
                }

                if status.is_moving {
                    self.hw_seg_seen_moving = true;
                }

                let elapsed = self
                    .hw_seg_dispatched_at
                    .map(|t| (now - t).as_secs_f64())
                    .unwrap_or(0.0);
                let dist_to_end = head_world.distance(seg.end);
                let expected_secs = seg.length_mm / seg.speed_mm_s.max(0.5);
                let finished = (self.hw_seg_seen_moving && !status.is_moving)
                    || (!status.is_moving && elapsed >= (expected_secs + 0.15).max(0.18));

                if finished {
                    self.traveled_length_mm += seg.length_mm;
                    self.seg_progress_mm = 0.0;
                    self.current_seg_idx += 1;
                    self.hw_seg_dispatched_at = None;
                    self.hw_seg_seen_moving = false;
                } else {
                    self.seg_progress_mm = (seg.length_mm - dist_to_end).clamp(0.0, seg.length_mm);
                    break;
                }
            }
        } else {
            let speed_mult = if self.run_mode != RunMode::DryCut {
                1.0
            } else {
                self.sim_speed_multiplier
            };

            let mut remaining_dt = dt;
            while remaining_dt > 1e-9 && self.current_seg_idx < self.trajectory.len() {
                let seg = self.trajectory[self.current_seg_idx];
                let eff_speed = (seg.speed_mm_s * speed_mult).max(0.5);
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
        }

        if self.current_seg_idx >= self.trajectory.len() {
            let end_pos = self
                .trajectory
                .last()
                .map(|last| last.end)
                .unwrap_or(self.job_start_pos);
            let origin_world = origin_world_mm(pulses_cfg);
            if !hw_connected {
                status.pos_x = pulses_cfg.mm_to_pulses_x(end_pos.x - origin_world.x);
                status.pos_y = pulses_cfg.mm_to_pulses_y(end_pos.y - origin_world.y);
            }

            let finished_label = if self.run_mode == RunMode::DryCut {
                "Dry-Cut simulation"
            } else {
                "Cutting job"
            };
            let total = self.total_length_mm;
            let start_pos = self.job_start_pos;
            self.stop_run(controller, status);

            // Execute configured After-Job Z-Home & XY return motion
            if pulses_cfg.post_job_z_home {
                if let Some(ref c) = controller {
                    if c.is_connected() {
                        c.home_z();
                    } else {
                        status.pos_z = 0;
                        status.capacitive_height_mm = 0.0;
                    }
                } else {
                    status.pos_z = 0;
                    status.capacitive_height_mm = 0.0;
                }
            }

            let target_pos = match pulses_cfg.post_job_action {
                tzlaser_core::PostJobAction::ToStartPosition => Some(start_pos),
                tzlaser_core::PostJobAction::ToEndPosition => None,
                tzlaser_core::PostJobAction::ToOrigin => Some(kurbo::Point::ZERO),
                tzlaser_core::PostJobAction::ToCoordinates => {
                    let tx = if pulses_cfg.soft_limits_enabled {
                        pulses_cfg.post_job_x_mm.clamp(0.0, pulses_cfg.cut_width_mm())
                    } else {
                        pulses_cfg.post_job_x_mm
                    };
                    let ty = if pulses_cfg.soft_limits_enabled {
                        pulses_cfg.post_job_y_mm.clamp(0.0, pulses_cfg.cut_height_mm())
                    } else {
                        pulses_cfg.post_job_y_mm
                    };
                    Some(kurbo::Point::new(tx, ty))
                }
            };

            if let Some(tp) = target_pos {
                let head_now = if hw_connected {
                    head_world_mm(status, pulses_cfg)
                } else {
                    end_pos
                };
                let dx_p = pulses_cfg.mm_to_pulses_x(tp.x - head_now.x);
                let dy_p = pulses_cfg.mm_to_pulses_y(tp.y - head_now.y);
                if let Some(ref c) = controller {
                    if c.is_connected() && (dx_p != 0 || dy_p != 0) {
                        let ppm = pulses_cfg
                            .pulses_per_mm_x
                            .max(pulses_cfg.pulses_per_mm_y)
                            .max(10.0);
                        let rapid_pls = (100.0 * ppm).round().clamp(500.0, 200_000.0) as u32;
                        c.move_xy(
                            dx_p,
                            dy_p,
                            tzlaser_protocol::JogConfig {
                                min_speed: (rapid_pls / 10).max(100),
                                jog_speed: rapid_pls,
                                accel: (rapid_pls * 10).max(2_000),
                            },
                        );
                    }
                }
                if !hw_connected {
                    status.pos_x = pulses_cfg.mm_to_pulses_x(tp.x - origin_world.x);
                    status.pos_y = pulses_cfg.mm_to_pulses_y(tp.y - origin_world.y);
                }
            }

            let z_note = if pulses_cfg.post_job_z_home {
                ", Z-Home"
            } else {
                ""
            };
            return Some(format!(
                "{} completed ({:.1} mm total travel; post-job: {}{}).",
                finished_label,
                total,
                pulses_cfg.post_job_action.label(),
                z_note
            ));
        }

        let seg = self.trajectory[self.current_seg_idx];
        let frac = (self.seg_progress_mm / seg.length_mm.max(1e-9)).clamp(0.0, 1.0);
        let cur_x = seg.start.x + (seg.end.x - seg.start.x) * frac;
        let cur_y = seg.start.y + (seg.end.y - seg.start.y) * frac;
        if !hw_connected {
            let origin_world = origin_world_mm(pulses_cfg);
            status.pos_x = pulses_cfg.mm_to_pulses_x(cur_x - origin_world.x);
            status.pos_y = pulses_cfg.mm_to_pulses_y(cur_y - origin_world.y);
        }
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
    signal_graph: &mut crate::signal_graph::SignalGraphState,
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

                if let JogMode::Step(mut step_val) = panel_state.jog_mode {
                    let step_options = [0.1, 1.0, 10.0, 50.0, 100.0];
                    ui.horizontal_wrapped(|ui| {
                        for (i, &step) in step_options.iter().enumerate() {
                            let selected = (step_val - step).abs() < 1e-6;
                            if ui.selectable_label(selected, format!("{}mm", step)).clicked() {
                                panel_state.step_index = i;
                                panel_state.jog_mode = JogMode::Step(step);
                                step_val = step;
                            }
                        }
                        if ui
                            .add(
                                egui::DragValue::new(&mut step_val)
                                    .range(0.01..=1000.0)
                                    .speed(0.5)
                                    .suffix(" mm"),
                            )
                            .changed()
                        {
                            panel_state.jog_mode = JogMode::Step(step_val);
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

                let jog_dot_btn = |ui: &mut Ui,
                                   label: &str,
                                   text_color: Option<Color32>,
                                   fill: Option<Color32>,
                                   limit_active: bool,
                                   hover: &str|
                 -> bool {
                    let mut rt = RichText::new(label).size(13.0);
                    if let Some(c) = text_color {
                        rt = rt.color(c);
                    }
                    let mut btn = Button::new(rt);
                    if let Some(bg) = fill {
                        btn = btn.fill(bg);
                    }
                    let resp = ui.add_sized(btn_size, btn).on_hover_text(hover);
                    let dot_center = egui::pos2(resp.rect.right() - 7.5, resp.rect.top() + 7.5);
                    if limit_active {
                        ui.painter().circle_filled(
                            dot_center,
                            5.0,
                            Color32::from_rgba_unmultiplied(255, 95, 70, 85),
                        );
                        ui.painter()
                            .circle_filled(dot_center, 3.0, Color32::from_rgb(255, 75, 65));
                    } else {
                        ui.painter().circle(
                            dot_center,
                            2.8,
                            Color32::from_rgb(52, 58, 70),
                            egui::Stroke::new(1.0, Color32::from_rgb(95, 102, 118)),
                        );
                    }
                    resp.clicked()
                };

                ui.horizontal(|ui| {
                    ui.add_space((ui.available_width() - pad_width).max(0.0) / 2.0);
                    Grid::new("jog_pad_3x3")
                        .num_columns(3)
                        .spacing([6.0, 6.0])
                        .show(ui, |ui| {
                            // Row 1: [empty] | Y+ | Z+
                            ui.allocate_exact_size(egui::vec2(btn_size[0], btn_size[1]), egui::Sense::hover());
                            if jog_dot_btn(
                                ui,
                                "Y+ ▲",
                                None,
                                None,
                                status.y_pos_limit(),
                                if status.y_pos_limit() {
                                    "Jog Y+ (Y+ limit switch TRIGGERED)"
                                } else {
                                    "Jog Y+"
                                },
                            ) {
                                jog_cmd = Some((Axis::Y, step_pulses_y));
                            }
                            if jog_dot_btn(
                                ui,
                                "Z+ ▲",
                                Some(Color32::from_rgb(130, 195, 255)),
                                Some(Color32::from_rgb(28, 38, 56)),
                                status.z_pos_limit(),
                                if status.z_pos_limit() {
                                    "Raise cutting head (Z+ upper limit switch TRIGGERED)"
                                } else {
                                    "Raise cutting head (Z+)"
                                },
                            ) {
                                jog_cmd = Some((Axis::Z, step_pulses_z));
                            }
                            ui.end_row();

                            // Row 2: X- | STOP | X+ (middle between Z+ and Z-)
                            if jog_dot_btn(
                                ui,
                                "◄ X-",
                                None,
                                None,
                                status.x_neg_limit(),
                                if status.x_neg_limit() {
                                    "Jog X- (X- limit switch TRIGGERED)"
                                } else {
                                    "Jog X-"
                                },
                            ) {
                                jog_cmd = Some((Axis::X, -step_pulses_x));
                            }
                            let stop_btn = Button::new(RichText::new("STOP").color(Color32::WHITE).strong())
                                .fill(Color32::from_rgb(180, 40, 40));
                            if ui.add_sized(btn_size, stop_btn).clicked() {
                                stop_clicked = true;
                            }
                            if jog_dot_btn(
                                ui,
                                "X+ ►",
                                None,
                                None,
                                status.x_pos_limit(),
                                if status.x_pos_limit() {
                                    "Jog X+ (X+ limit switch TRIGGERED)"
                                } else {
                                    "Jog X+"
                                },
                            ) {
                                jog_cmd = Some((Axis::X, step_pulses_x));
                            }
                            ui.end_row();

                            // Row 3: [empty] | Y- | Z-
                            ui.allocate_exact_size(egui::vec2(btn_size[0], btn_size[1]), egui::Sense::hover());
                            if jog_dot_btn(
                                ui,
                                "Y- ▼",
                                None,
                                None,
                                status.y_neg_limit(),
                                if status.y_neg_limit() {
                                    "Jog Y- (Y- limit switch TRIGGERED)"
                                } else {
                                    "Jog Y-"
                                },
                            ) {
                                jog_cmd = Some((Axis::Y, -step_pulses_y));
                            }
                            if jog_dot_btn(
                                ui,
                                "Z- ▼",
                                Some(Color32::from_rgb(130, 195, 255)),
                                Some(Color32::from_rgb(28, 38, 56)),
                                status.z_neg_cap_limit(),
                                if status.z_neg_cap_limit() {
                                    "Lower cutting head (Capacitive sensor limit TRIGGERED)"
                                } else {
                                    "Lower cutting head (Z-, capacitive sensor limit)"
                                },
                            ) {
                                jog_cmd = Some((Axis::Z, -step_pulses_z));
                            }
                            ui.end_row();
                        });
                });

                if stop_clicked {
                    panel_state.stop_run(controller, status);
                    *status_message = "Motion stopped.".to_string();
                } else if let Some((axis, pulses)) = jog_cmd {
                    let mut clamped_pulses = pulses;
                    let mut limit_msg: Option<String> = None;
                    if pulses_cfg.soft_limits_enabled && (axis == Axis::X || axis == Axis::Y) {
                        let (cur_mm, delta_mm, max_mm) = match axis {
                            Axis::X => (
                                pulses_cfg.pulses_to_mm_x(status.pos_x).abs(),
                                pulses_cfg.pulses_to_mm_x(pulses),
                                pulses_cfg.cut_width_mm(),
                            ),
                            Axis::Y => (
                                pulses_cfg.pulses_to_mm_y(status.pos_y).abs(),
                                pulses_cfg.pulses_to_mm_y(pulses),
                                pulses_cfg.cut_height_mm(),
                            ),
                            _ => (0.0, 0.0, 0.0),
                        };
                        let next_mm = cur_mm + delta_mm;
                        if next_mm < -0.01 || next_mm > max_mm + 0.01 {
                            clamped_pulses = 0;
                            limit_msg = Some(format!(
                                "Soft limit: {:?} target {:.1} mm outside effective cut area [0.0..{:.1} mm].",
                                axis, next_mm, max_mm
                            ));
                        }
                    }
                    if let Some(msg) = limit_msg {
                        *status_message = msg;
                    } else if is_connected {
                        if let Some(c) = controller {
                            c.jog(axis, clamped_pulses);
                        }
                    } else {
                        let est_dt = 0.05_f32;
                        let rate = (clamped_pulses as f32) / est_dt;
                        match axis {
                            Axis::X => {
                                status.pos_x += clamped_pulses;
                                signal_graph.record_offline(rate, 0.0, 0.0, 0.0, false, Some("Jog X"));
                            }
                            Axis::Y => {
                                status.pos_y += clamped_pulses;
                                signal_graph.record_offline(0.0, rate, 0.0, 0.0, false, Some("Jog Y"));
                            }
                            Axis::Z => {
                                status.pos_z += clamped_pulses;
                                status.capacitive_height_mm =
                                    ((status.pos_z as f32) / (pulses_cfg.pulses_per_mm_z as f32)).max(0.0);
                                signal_graph.record_offline(0.0, 0.0, rate, 0.0, false, Some("Jog Z"));
                            }
                            Axis::W => {
                                status.pos_w += clamped_pulses;
                                signal_graph.record_offline(0.0, 0.0, 0.0, rate, false, Some("Jog W"));
                            }
                        }
                    }
                }

                ui.add_space(8.0);
                ui.separator();
                ui.add_space(4.0);

                // Job Execution (2x2 Pendant Grid + Aux Toggles + Pause Buffer)
                ui.label(RichText::new("Job Execution:").strong().size(12.0));
                ui.add_space(3.0);

                let current_head = head_world_mm(status, pulses_cfg);
                let spacing_x = ui.spacing().item_spacing.x;
                let run_btn_w = ((ui.available_width() - spacing_x) / 2.0).floor().max(95.0);
                let has_pause = panel_state.paused_checkpoint.is_some();
                let can_pause = panel_state.run_mode != RunMode::Idle && !panel_state.is_paused;
                let can_continue = panel_state.is_paused && has_pause;
                let can_step = panel_state.is_paused && has_pause;

                // Row 1: (Start, Pause)
                ui.horizontal(|ui| {
                    let is_cutting = panel_state.run_mode == RunMode::Cutting && !panel_state.is_paused;
                    let start_fill = if !is_connected {
                        Color32::from_rgb(48, 50, 62)
                    } else if is_cutting {
                        Color32::from_rgb(35, 160, 75)
                    } else {
                        Color32::from_rgb(26, 110, 52)
                    };
                    let start_label = if has_pause { "🔄 Restart" } else { "▶ Start" };
                    let start_btn = Button::new(RichText::new(start_label).size(13.0).color(Color32::WHITE).strong())
                        .fill(start_fill);
                    if ui
                        .add_enabled_ui(is_connected, |ui| ui.add_sized([run_btn_w, 30.0], start_btn))
                        .inner
                        .on_hover_text(if has_pause {
                            "Restart cutting job from the beginning (discards pause checkpoint)"
                        } else {
                            "Start cutting job along sorted toolpath sequence"
                        })
                        .on_disabled_hover_text("Controller not connected (Start disabled offline)")
                        .clicked()
                    {
                        match panel_state.validate_motion_bounds(doc, current_head, controller, status, pulses_cfg) {
                            Ok(()) => {
                                *status_message = panel_state.start_run(RunMode::Cutting, doc, current_head, controller);
                            }
                            Err(msg) => {
                                *status_message = msg;
                            }
                        }
                    }

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
                });

                ui.add_space(4.0);

                // Row 2: (Dry-Cut, Continue)
                ui.horizontal(|ui| {
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
                        .add_sized([run_btn_w, 30.0], dry_btn)
                        .on_hover_text(if is_connected {
                            "Move physical cutting head along toolpaths without firing the laser"
                        } else {
                            "Simulate pointer movement along toolpaths offline (with fast-forward)"
                        })
                        .clicked()
                    {
                        match panel_state.validate_motion_bounds(doc, current_head, controller, status, pulses_cfg) {
                            Ok(()) => {
                                *status_message = panel_state.start_run(RunMode::DryCut, doc, current_head, controller);
                                panel_state.start_hw_dry_cut_fifo(controller, status, pulses_cfg);
                            }
                            Err(msg) => {
                                *status_message = msg;
                            }
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
                        match panel_state.validate_motion_bounds(doc, current_head, controller, status, pulses_cfg) {
                            Ok(()) => {
                                if let Some(msg) = panel_state.continue_run(doc, current_head, controller) {
                                    *status_message = msg;
                                    if panel_state.run_mode == RunMode::DryCut {
                                        panel_state.start_hw_dry_cut_fifo(controller, status, pulses_cfg);
                                    }
                                }
                            }
                            Err(msg) => {
                                *status_message = msg;
                            }
                        }
                    }
                });

                ui.add_space(4.0);

                // Row 3: (Frame, Follower)
                ui.horizontal(|ui| {
                    let frame_btn = Button::new(
                        RichText::new("🔲 Frame")
                            .size(12.5)
                            .color(Color32::from_rgb(255, 200, 50))
                            .strong(),
                    )
                    .fill(Color32::from_rgb(48, 42, 22));
                    if ui
                        .add_sized([run_btn_w, 28.0], frame_btn)
                        .on_hover_text("Trace the active job bounding box with the red guide pointer (laser off)")
                        .clicked()
                    {
                        if let Err(msg) = panel_state.validate_motion_bounds(doc, current_head, controller, status, pulses_cfg) {
                            *status_message = msg;
                        } else {
                        let mut bbox: Option<kurbo::Rect> = None;
                        for layer in &doc.layers {
                            if !layer.visible || layer.is_obstacle {
                                continue;
                            }
                            for ent in &layer.entities {
                                let b = ent.bounding_box();
                                bbox = Some(match bbox {
                                    Some(acc) => acc.union(b),
                                    None => b,
                                });
                            }
                        }
                        if let Some(b) = bbox {
                            if let Some(ref c) = controller {
                                if c.is_connected() {
                                    c.set_red_pointer(true);
                                    let corners = [
                                        kurbo::Point::new(b.x0, b.y0),
                                        kurbo::Point::new(b.x1, b.y0),
                                        kurbo::Point::new(b.x1, b.y1),
                                        kurbo::Point::new(b.x0, b.y1),
                                        kurbo::Point::new(b.x0, b.y0),
                                    ];
                                    let mut prev = current_head;
                                    for pt in corners {
                                        let dx = pulses_cfg.mm_to_pulses_x(pt.x - prev.x);
                                        let dy = pulses_cfg.mm_to_pulses_y(pt.y - prev.y);
                                        if dx != 0 {
                                            c.jog(Axis::X, dx);
                                        }
                                        if dy != 0 {
                                            c.jog(Axis::Y, dy);
                                        }
                                        prev = pt;
                                    }
                                }
                            }
                            *status_message = format!(
                                "Framing job bounds [{:.1}..{:.1}, {:.1}..{:.1}] mm with red guide pointer.",
                                b.x0, b.x1, b.y0, b.y1
                            );
                        } else {
                            *status_message = "No active cuttable geometry to frame.".to_string();
                        }
                        }
                    }

                    let fol_bg = if status.follower_active {
                        Color32::from_rgb(20, 75, 42)
                    } else {
                        Color32::from_rgb(38, 42, 50)
                    };
                    let fol_fg = if status.follower_active {
                        Color32::from_rgb(0, 255, 130)
                    } else {
                        Color32::WHITE
                    };
                    if ui
                        .add_sized(
                            [run_btn_w, 28.0],
                            Button::new(RichText::new("🛡 Follower").size(12.5).color(fol_fg).strong())
                                .fill(fol_bg),
                        )
                        .on_hover_text("Toggle capacitive height tracking follower servo")
                        .clicked()
                    {
                        status.follower_active = !status.follower_active;
                        if let Some(ref c) = controller {
                            if c.is_connected() {
                                c.set_follower_servo(status.follower_active);
                            }
                        }
                        *status_message = format!(
                            "Height follower servo: {}",
                            if status.follower_active { "ON" } else { "OFF" }
                        );
                    }
                });

                ui.add_space(4.0);

                // Row 4: (Red Dot, Gas)
                ui.horizontal(|ui| {
                    let red_bg = if status.red_pointer_on {
                        Color32::from_rgb(85, 28, 28)
                    } else {
                        Color32::from_rgb(38, 42, 50)
                    };
                    let red_fg = if status.red_pointer_on {
                        Color32::from_rgb(255, 115, 115)
                    } else {
                        Color32::WHITE
                    };
                    if ui
                        .add_sized(
                            [run_btn_w, 28.0],
                            Button::new(RichText::new("🔴 Red Dot").size(12.5).color(red_fg).strong())
                                .fill(red_bg),
                        )
                        .on_hover_text("Toggle 650nm visible red laser guide pointer")
                        .clicked()
                    {
                        status.red_pointer_on = !status.red_pointer_on;
                        if let Some(ref c) = controller {
                            if c.is_connected() {
                                c.set_red_pointer(status.red_pointer_on);
                            }
                        }
                        *status_message = format!(
                            "Red Dot guide: {}",
                            if status.red_pointer_on { "ON" } else { "OFF" }
                        );
                    }

                    let gas_bg = if status.gas_valve_open {
                        Color32::from_rgb(24, 68, 92)
                    } else {
                        Color32::from_rgb(38, 42, 50)
                    };
                    let gas_fg = if status.gas_valve_open {
                        Color32::from_rgb(90, 225, 255)
                    } else {
                        Color32::WHITE
                    };
                    if ui
                        .add_sized(
                            [run_btn_w, 28.0],
                            Button::new(RichText::new("💨 Gas").size(12.5).color(gas_fg).strong())
                                .fill(gas_bg),
                        )
                        .on_hover_text("Toggle assist gas valve")
                        .clicked()
                    {
                        status.gas_valve_open = !status.gas_valve_open;
                        if let Some(ref c) = controller {
                            if c.is_connected() {
                                c.set_gas(3, status.gas_valve_open);
                            }
                        }
                        *status_message = format!(
                            "Assist gas valve: {}",
                            if status.gas_valve_open { "OPEN" } else { "CLOSED" }
                        );
                    }
                });

                ui.add_space(4.0);

                // Row 5: (Backward, Forward)
                ui.horizontal(|ui| {
                    let step_fill = if can_step {
                        Color32::from_rgb(65, 95, 155)
                    } else {
                        Color32::from_rgb(48, 50, 62)
                    };

                    let back_btn = Button::new(RichText::new("⏮ Backward").size(12.5).color(Color32::WHITE).strong())
                        .fill(step_fill);
                    if ui
                        .add_enabled_ui(can_step, |ui| ui.add_sized([run_btn_w, 28.0], back_btn))
                        .inner
                        .on_hover_text(format!(
                            "Step backward {:.2} mm along active profile (or jump to previous profile endpoint)",
                            step_dist
                        ))
                        .on_disabled_hover_text("Pause job first to step backward along path")
                        .clicked()
                    {
                        if let Some(msg) =
                            panel_state.step_along_path(false, step_dist, controller, status, pulses_cfg)
                        {
                            *status_message = msg;
                        }
                    }

                    let fwd_btn = Button::new(RichText::new("⏭ Forward").size(12.5).color(Color32::WHITE).strong())
                        .fill(step_fill);
                    if ui
                        .add_enabled_ui(can_step, |ui| ui.add_sized([run_btn_w, 28.0], fwd_btn))
                        .inner
                        .on_hover_text(format!(
                            "Step forward {:.2} mm along active profile (or jump to next profile start)",
                            step_dist
                        ))
                        .on_disabled_hover_text("Pause job first to step forward along path")
                        .clicked()
                    {
                        if let Some(msg) =
                            panel_state.step_along_path(true, step_dist, controller, status, pulses_cfg)
                        {
                            *status_message = msg;
                        }
                    }
                });

                ui.add_space(4.0);

                // Row 6: (Home, Z-Home)
                ui.horizontal(|ui| {
                    let home_fill = if is_connected {
                        if status.is_homing_xy || status.is_homing_z {
                            Color32::from_rgb(165, 115, 25)
                        } else if status.xy_homed && status.z_homed {
                            Color32::from_rgb(24, 92, 62)
                        } else {
                            Color32::from_rgb(35, 78, 120)
                        }
                    } else {
                        Color32::from_rgb(48, 50, 62)
                    };
                    let home_btn = Button::new(RichText::new("🏠 Home").size(12.5).color(Color32::WHITE).strong())
                        .fill(home_fill);
                    if ui
                        .add_enabled_ui(is_connected, |ui| ui.add_sized([run_btn_w, 28.0], home_btn))
                        .inner
                        .on_hover_text(format!(
                            "Home all 3 axes (Z up to sensor, then X/Y toward {})",
                            pulses_cfg.origin_corner
                        ))
                        .on_disabled_hover_text("Connect to machine controller to home axes")
                        .clicked()
                    {
                        if let Some(ref c) = controller {
                            c.home_all(home_config_from_pulse(pulses_cfg));
                            *status_message = format!(
                                "Homing Z (up) and X/Y toward {}...",
                                pulses_cfg.origin_corner
                            );
                        }
                    }

                    let z_home_fill = if is_connected {
                        if status.is_homing_z {
                            Color32::from_rgb(165, 115, 25)
                        } else if status.z_homed {
                            Color32::from_rgb(24, 92, 62)
                        } else {
                            Color32::from_rgb(35, 78, 120)
                        }
                    } else {
                        Color32::from_rgb(48, 50, 62)
                    };
                    let z_home_btn = Button::new(RichText::new("⬆ Z-Home").size(12.5).color(Color32::WHITE).strong())
                        .fill(z_home_fill);
                    if ui
                        .add_enabled_ui(is_connected, |ui| ui.add_sized([run_btn_w, 28.0], z_home_btn))
                        .inner
                        .on_hover_text("Home Z-axis only (raise head Z-up until upper capacitive/limit sensor triggers)")
                        .on_disabled_hover_text("Connect to machine controller to home Z-axis")
                        .clicked()
                    {
                        if let Some(ref c) = controller {
                            c.home_z();
                            *status_message = "Homing Z-axis (moving Z-up to sensor)...".to_string();
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
                    let est_suffix = if panel_state.estimated_job_secs > 0.0 {
                        format!(" · Est {}", format_duration_secs(panel_state.estimated_job_secs))
                    } else {
                        String::new()
                    };
                    let bar_text = if !is_connected && panel_state.run_mode == RunMode::DryCut {
                        format!(
                            "{}: Path {}/{} ({:.0}%) [{}x]{}",
                            mode_tag, cur_path, total_paths, progress * 100.0, panel_state.sim_speed_multiplier, est_suffix
                        )
                    } else {
                        format!("{}: Path {}/{} ({:.0}%){}", mode_tag, cur_path, total_paths, progress * 100.0, est_suffix)
                    };
                    ui.add(ProgressBar::new(progress).text(bar_text));
                }

                // Simulation speed multiplier strictly for offline Dry-Cut simulation
                if !is_connected && panel_state.run_mode != RunMode::Cutting {
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new("Sim Speed:")
                                .size(11.5)
                                .color(Color32::from_rgb(170, 200, 220))
                                .strong(),
                        );
                        let fmt_speed = |m: f64| -> String {
                            let s = format!("{:.3}", m);
                            let s = s.trim_end_matches('0').trim_end_matches('.');
                            format!("{}x", s)
                        };
                        egui::ComboBox::from_id_salt("sim_speed_combo")
                            .selected_text(fmt_speed(panel_state.sim_speed_multiplier))
                            .width(72.0)
                            .show_ui(ui, |ui| {
                                for &mult in &[0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0] {
                                    let selected = (panel_state.sim_speed_multiplier - mult).abs() < 1e-6;
                                    if ui.selectable_label(selected, fmt_speed(mult)).clicked() {
                                        panel_state.sim_speed_multiplier = mult;
                                    }
                                }
                            });
                    });
                }

                ui.add_space(6.0);
            });

        ui.add_space(4.0);
        ui.separator();

        // ==========================================
        // 3. CUTTING LAYERS LEGEND (Click a layer for Properties & DB Pop-up)
        // ==========================================
        CollapsingHeader::new(RichText::new("🎨 Cutting Layers").strong().size(14.0))
            .default_open(true)
            .show(ui, |ui| {
                ui.add_space(2.0);
                ui.label(
                    RichText::new("Click a layer to edit parameters & DB profiles:")
                        .size(10.5)
                        .weak(),
                );
                ui.add_space(3.0);

                for (l_idx, layer) in doc.layers.iter_mut().enumerate() {
                    let is_open = panel_state.active_layer_popup == Some(l_idx);
                    let row_fill = if is_open {
                        Color32::from_rgb(35, 45, 58)
                    } else {
                        Color32::from_rgb(28, 30, 36)
                    };
                    let row_stroke = if is_open {
                        egui::Stroke::new(1.0, Color32::from_rgb(80, 175, 235))
                    } else {
                        egui::Stroke::new(1.0, Color32::from_rgb(44, 47, 56))
                    };

                    Frame::group(ui.style())
                        .fill(row_fill)
                        .stroke(row_stroke)
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                ui.checkbox(&mut layer.visible, "");
                                let swatch_color = Color32::from_rgb(layer.color.r, layer.color.g, layer.color.b);
                                let summary = if layer.is_travel {
                                    format!(
                                        "{:.0} mm/s · H:{:.1} mm",
                                        layer.params.speed_mm_s,
                                        layer.params.cut_height_mm
                                    )
                                } else if layer.is_obstacle {
                                    format!(
                                        "#{}, {:.2} m · Obstacle (No-Cut)",
                                        layer.total_path_count(),
                                        layer.total_path_length_m(),
                                    )
                                } else {
                                    format!(
                                        "#{}, {:.2} m · {:.0} mm/s · {:.0}% · {}",
                                        layer.total_path_count(),
                                        layer.total_path_length_m(),
                                        layer.params.speed_mm_s,
                                        layer.params.power_percent,
                                        layer.params.gas_type
                                    )
                                };
                                let btn = Button::new(
                                    RichText::new(format!("■  {}", layer.name))
                                        .color(swatch_color)
                                        .strong()
                                        .size(12.5),
                                )
                                .frame(false);
                                let clicked = ui
                                    .add(btn)
                                    .on_hover_text("Click to open Layer Properties & Material DB pop-up")
                                    .clicked();

                                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                    ui.label(
                                        RichText::new(summary)
                                            .size(10.5)
                                            .color(Color32::from_rgb(175, 185, 200)),
                                    );
                                });

                                if clicked {
                                    if is_open {
                                        panel_state.active_layer_popup = None;
                                    } else {
                                        panel_state.active_layer_popup = Some(l_idx);
                                    }
                                }
                            });
                        });
                }

                ui.add_space(4.0);
                let add_layer_btn = Button::new(
                    RichText::new("➕ Add Layer")
                        .size(12.0)
                        .color(Color32::from_rgb(120, 230, 170))
                        .strong(),
                )
                .fill(Color32::from_rgb(26, 46, 40));
                if ui
                    .add_sized([ui.available_width(), 26.0], add_layer_btn)
                    .on_hover_text("Create a new custom layer (preserved when running Auto-Classify)")
                    .clicked()
                {
                    let new_idx = doc.add_custom_layer();
                    panel_state.active_layer_popup = Some(new_idx);
                    *status_message = format!("Added custom layer '{}'.", doc.layers[new_idx].name);
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
        let empty_doc = Document::new();
        let cont_msg = state.continue_run(&empty_doc, jogged_pos, &None);
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

        assert!(state.continue_run(&empty_doc, cp2.point_mm, &None).is_some());
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

    #[test]
    fn test_step_backward_and_forward_along_path_and_across_profiles() {
        let pulses = PulseConfig::default();
        let mut status = MachineStatus::default();
        let mut state = PanelState::default();

        let rapid_1_to_2 = kurbo::Point::new(40.0, 20.0).distance(kurbo::Point::new(100.0, 10.0));
        let full = vec![
            MotionSegment {
                start: kurbo::Point::new(0.0, 0.0),
                end: kurbo::Point::new(10.0, 0.0),
                length_mm: 10.0,
                speed_mm_s: 200.0,
                is_rapid: true,
                path_index_1based: 1,
            },
            // Profile 1: 30mm horizontal + 20mm vertical = 50mm total
            MotionSegment {
                start: kurbo::Point::new(10.0, 0.0),
                end: kurbo::Point::new(40.0, 0.0),
                length_mm: 30.0,
                speed_mm_s: 50.0,
                is_rapid: false,
                path_index_1based: 1,
            },
            MotionSegment {
                start: kurbo::Point::new(40.0, 0.0),
                end: kurbo::Point::new(40.0, 20.0),
                length_mm: 20.0,
                speed_mm_s: 50.0,
                is_rapid: false,
                path_index_1based: 1,
            },
            MotionSegment {
                start: kurbo::Point::new(40.0, 20.0),
                end: kurbo::Point::new(100.0, 10.0),
                length_mm: rapid_1_to_2,
                speed_mm_s: 200.0,
                is_rapid: true,
                path_index_1based: 2,
            },
            // Profile 2: 40mm horizontal
            MotionSegment {
                start: kurbo::Point::new(100.0, 10.0),
                end: kurbo::Point::new(140.0, 10.0),
                length_mm: 40.0,
                speed_mm_s: 50.0,
                is_rapid: false,
                path_index_1based: 2,
            },
        ];

        state.trajectory = full.clone();
        state.full_trajectory = full;
        state.run_mode = RunMode::DryCut;
        state.current_seg_idx = 1;
        state.seg_progress_mm = 25.0; // At (35, 0), s = 25/50 along Profile 1
        state.traveled_length_mm = 35.0;
        state.total_length_mm = state.full_trajectory.iter().map(|s| s.length_mm).sum();

        // Stepping must be disabled when not paused
        assert!(state.step_along_path(true, 10.0, &None, &mut status, &pulses).is_none());

        // Pause at (35, 0)
        assert!(state.pause_run(&None, &mut status, &pulses).is_some());
        let cp0 = state.paused_checkpoint.unwrap();
        assert!((cp0.point_mm.x - 35.0).abs() < 1e-6 && cp0.point_mm.y.abs() < 1e-6);

        // 1. Forward 15mm: crosses corner from (35, 0) -> (40, 0) -> (40, 10), s = 40/50
        assert!(state.step_along_path(true, 15.0, &None, &mut status, &pulses).is_some());
        let cp1 = state.paused_checkpoint.unwrap();
        assert_eq!(cp1.path_index_1based, 1);
        assert!((cp1.point_mm.x - 40.0).abs() < 1e-6 && (cp1.point_mm.y - 10.0).abs() < 1e-6);

        // 2. Forward 25mm when only 10mm remain on Profile 1 -> clamps to endpoint (40, 20)
        assert!(state.step_along_path(true, 25.0, &None, &mut status, &pulses).is_some());
        let cp2 = state.paused_checkpoint.unwrap();
        assert_eq!(cp2.path_index_1based, 1);
        assert!((cp2.point_mm.x - 40.0).abs() < 1e-6 && (cp2.point_mm.y - 20.0).abs() < 1e-6);

        // 3. Forward 15mm from endpoint (40, 20) -> travels to Profile 2 start (100, 10) + 15mm = (115, 10)
        assert!(state.step_along_path(true, 15.0, &None, &mut status, &pulses).is_some());
        let cp3 = state.paused_checkpoint.unwrap();
        assert_eq!(cp3.path_index_1based, 2);
        assert!((cp3.point_mm.x - 115.0).abs() < 1e-6 && (cp3.point_mm.y - 10.0).abs() < 1e-6);
        assert!((pulses.pulses_to_mm_x(status.pos_x) - 115.0).abs() < 0.05);

        // 4. Backward 25mm when only 15mm from start of Profile 2 -> clamps to start (100, 10)
        assert!(state.step_along_path(false, 25.0, &None, &mut status, &pulses).is_some());
        let cp4 = state.paused_checkpoint.unwrap();
        assert_eq!(cp4.path_index_1based, 2);
        assert!((cp4.point_mm.x - 100.0).abs() < 1e-6 && (cp4.point_mm.y - 10.0).abs() < 1e-6);

        // 5. Backward 12mm from start (100, 10) -> travels to Profile 1 endpoint (40, 20) - 12mm = (40, 8)
        assert!(state.step_along_path(false, 12.0, &None, &mut status, &pulses).is_some());
        let cp5 = state.paused_checkpoint.unwrap();
        assert_eq!(cp5.path_index_1based, 1);
        assert!((cp5.point_mm.x - 40.0).abs() < 1e-6 && (cp5.point_mm.y - 8.0).abs() < 1e-6);

        // 6. Continue from stepped point cleanly picks up remaining Profile 1 (12mm) + rapid + Profile 2
        assert!(state.continue_run(&Document::new(), cp5.point_mm, &None).is_some());
        assert!(!state.is_paused);
        assert_eq!(state.trajectory.len(), 3);
        assert!((state.trajectory[0].length_mm - 12.0).abs() < 1e-6);
    }

    #[test]
    fn test_fast_forward_only_applies_to_dry_cut() {
        let pulses = PulseConfig::default();
        let mut status = MachineStatus::default();
        let mut state = PanelState::default();
        state.sim_speed_multiplier = 8.0;
        state.trajectory = vec![MotionSegment {
            start: kurbo::Point::new(0.0, 0.0),
            end: kurbo::Point::new(500.0, 0.0),
            length_mm: 500.0,
            speed_mm_s: 10.0,
            is_rapid: false,
            path_index_1based: 1,
        }];
        state.total_length_mm = 500.0;
        state.run_mode = RunMode::Cutting;
        state.last_tick = Some(std::time::Instant::now() - std::time::Duration::from_millis(100));

        // Cutting mode ignores sim_speed_multiplier (runs at 1x -> at most 0.1s * 10mm/s = 1.0mm)
        state.tick_simulation(&None, &mut status, &pulses);
        assert!(state.traveled_length_mm <= 1.05);

        // DryCut mode offline uses 8x multiplier (0.1s * 10mm/s * 8 = ~8.0mm)
        state.traveled_length_mm = 0.0;
        state.seg_progress_mm = 0.0;
        state.run_mode = RunMode::DryCut;
        state.last_tick = Some(std::time::Instant::now() - std::time::Duration::from_millis(100));
        state.tick_simulation(&None, &mut status, &pulses);
        assert!(state.traveled_length_mm >= 5.0);
    }

    #[test]
    fn test_set_pause_at_path_point_from_stopped_and_continue() {
        let pulses = PulseConfig::default();
        let mut status = MachineStatus::default();
        let mut state = PanelState::default();

        let mut doc = Document::new();
        doc.layers[0].entities.push(tzlaser_core::Entity::Polyline {
            points: vec![
                kurbo::Point::new(50.0, 50.0),
                kurbo::Point::new(150.0, 50.0),
                kurbo::Point::new(150.0, 150.0),
                kurbo::Point::new(50.0, 150.0),
            ],
            closed: true,
        });

        let (snapped_pt, path_idx, dist) = doc
            .closest_point_on_cut_paths(kurbo::Point::new(100.0, 52.0), 5.0)
            .expect("should snap near bottom edge");
        assert_eq!(path_idx, 1);
        assert!((snapped_pt.x - 100.0).abs() < 1e-6 && (snapped_pt.y - 50.0).abs() < 1e-6);
        assert!((dist - 2.0).abs() < 1e-6);

        // From stopped (Idle) state, set resume target on path WITHOUT teleporting the head
        assert_eq!(state.run_mode, RunMode::Idle);
        let msg = state.set_pause_at_path_point(&doc, snapped_pt, &None, &mut status, &pulses);
        assert!(msg.is_some());
        assert!(state.is_paused);
        assert_eq!(state.run_mode, RunMode::DryCut);
        assert!(!status.laser_on);
        assert_eq!(status.pos_x, 0);
        assert_eq!(status.pos_y, 0);
        assert_eq!(state.pause_buffer.len(), 1);
        let cp = state.paused_checkpoint.unwrap();
        assert!((cp.point_mm.x - 100.0).abs() < 1e-6 && (cp.point_mm.y - 50.0).abs() < 1e-6);

        // Continuing from current head (0, 0) must traverse the span to the resume point (100, 50)
        let cur_head = head_world_mm(&status, &pulses);
        assert!(state.continue_run(&doc, cur_head, &None).is_some());
        assert!(!state.is_paused);
        assert!(state.trajectory[0].is_rapid);
        assert!((state.trajectory[0].start.x - 0.0).abs() < 1e-6 && (state.trajectory[0].start.y - 0.0).abs() < 1e-6);
        assert!((state.trajectory[0].end.x - 100.0).abs() < 1e-6 && (state.trajectory[0].end.y - 50.0).abs() < 1e-6);

        // Active unpaused state must reject right-click jump-to-pause
        assert!(state.set_pause_at_path_point(&doc, snapped_pt, &None, &mut status, &pulses).is_none());
    }

    #[test]
    fn test_layer_speed_sync_updates_active_trajectory() {
        let mut status = MachineStatus::default();
        let mut state = PanelState::default();
        let mut doc = Document::new();
        doc.layers[0].params.speed_mm_s = 80.0;
        doc.layers[0].entities.push(tzlaser_core::Entity::Line {
            start: kurbo::Point::new(10.0, 0.0),
            end: kurbo::Point::new(110.0, 0.0),
        });

        state.start_run(RunMode::DryCut, &doc, kurbo::Point::new(0.0, 0.0), &None);
        // Seg 0 is rapid (20 mm/s), Seg 1 is cut (80 mm/s)
        assert!(state.trajectory[0].is_rapid);
        assert!(!state.trajectory[1].is_rapid);
        assert!((state.trajectory[1].speed_mm_s - 80.0).abs() < 1e-6);

        // User edits layer speed in Cutting Layers while paused or running -> sync updates trajectory
        doc.layers[0].params.speed_mm_s = 12.5;
        state.sync_layer_speeds(&doc);
        assert!((state.trajectory[0].speed_mm_s - 20.0).abs() < 1e-6);
        assert!((state.trajectory[1].speed_mm_s - 12.5).abs() < 1e-6);
        assert!((state.full_trajectory[1].speed_mm_s - 12.5).abs() < 1e-6);

        state.stop_run(&None, &mut status);
    }

    #[test]
    fn test_build_trajectory_fifo_ticks_ramps_and_reaches_target() {
        let cfg = PulseConfig::default();
        let origin = origin_world_mm(&cfg);
        let traj = vec![MotionSegment {
            start: origin,
            end: kurbo::Point::new(origin.x + 20.0, origin.y),
            is_rapid: false,
            speed_mm_s: 15.0,
            length_mm: 20.0,
            path_index_1based: 1,
        }];
        let ticks = build_trajectory_fifo_ticks(&traj, origin, &cfg, 0, 0);
        assert!(ticks.len() > 60);
        assert_eq!(ticks[0], (0, 0, 0x30));
        assert!(ticks[1..59].iter().all(|&t| t == (0, 0, 0x02)));
        assert_eq!(ticks.last(), Some(&(0, 0, 0xF0)));
        // Total FIFO pulses are negated relative to R1008 target position
        let sum_x: i32 = ticks.iter().map(|t| t.0 as i32).sum();
        assert_eq!(sum_x, -cfg.mm_to_pulses_x(20.0));
        // First motion tick velocity should be small (accelerating gently from rest at 5 mm/s)
        assert!(ticks[59].0.abs() <= 1);
    }

    #[test]
    fn test_pause_bottom_right_origin_no_jump_and_oob_recovery() {
        let mut cfg = PulseConfig::default();
        cfg.table_width_mm = 2000.0;
        cfg.table_height_mm = 1500.0;
        cfg.home_retract_x_mm = 10.0;
        cfg.home_retract_y_mm = 10.0;
        cfg.origin_corner = "Bottom-Right (Xmax, Y0)".to_string();

        let mut doc = Document::new();
        doc.layers[0].entities.push(tzlaser_core::Entity::Polyline {
            points: vec![kurbo::Point::new(1900.0, 50.0), kurbo::Point::new(1800.0, 50.0)],
            closed: false,
        });

        let mut state = PanelState::default();
        let mut status = MachineStatus::default();
        let origin = origin_world_mm(&cfg);
        assert!((origin.x - 1990.0).abs() < 1e-6);

        state.start_run(RunMode::DryCut, &doc, origin, &None);
        state.seg_progress_mm = 40.0;
        state.pause_run(&None, &mut status, &cfg);
        let head_after_pause = head_world_mm(&status, &cfg);
        let cp = state.paused_checkpoint.expect("checkpoint");
        // Head must remain at the exact pause checkpoint (no 1990 mm jump!)
        assert!(head_after_pause.distance(cp.point_mm) < 0.05);

        // Out-of-bounds graphics must fail validation and reset head to origin without requiring rehome offline
        doc.layers[0].entities.push(tzlaser_core::Entity::Polyline {
            points: vec![kurbo::Point::new(2500.0, 50.0), kurbo::Point::new(2600.0, 50.0)],
            closed: false,
        });
        let res = state.validate_motion_bounds(&doc, head_after_pause, &None, &mut status, &cfg);
        assert!(res.is_err());
        assert_eq!(status.pos_x, 0);
        assert_eq!(status.pos_y, 0);
        assert!(!state.show_home_prompt);
        let (_, needs_rehome) = state.oob_alert.as_ref().expect("oob_alert");
        assert!(!needs_rehome);
    }
}
