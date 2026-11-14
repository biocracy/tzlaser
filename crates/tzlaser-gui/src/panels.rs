use egui::{Button, CollapsingHeader, Color32, Frame, Grid, ProgressBar, RichText, ScrollArea, Ui};
use tzlaser_core::document::closest_point_on_segment;
use tzlaser_core::{Document, MaterialLibrary, MotionSegment, PulseConfig};
use tzlaser_protocol::{format_remote_button, Axis, LaserController, MachineStatus};

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
    pub active_continuous_jog: Option<(Axis, i32)>,
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
    pub paused_checkpoint: Option<PauseCheckpoint>,
    pub pause_buffer: Vec<PauseCheckpoint>,
    pub material_db: MaterialLibrary,
    pub active_layer_popup: Option<usize>,
    pub new_mat_name: String,
    pub new_mat_material: String,
    pub new_mat_thickness: f64,
}

impl Default for PanelState {
    fn default() -> Self {
        Self {
            jog_mode: JogMode::Step(1.0),
            step_index: 2, // 1.0mm
            active_continuous_jog: None,
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
            paused_checkpoint: None,
            pause_buffer: Vec::new(),
            material_db: MaterialLibrary::with_default_presets(),
            active_layer_popup: None,
            new_mat_name: String::new(),
            new_mat_material: "Mild Steel".to_string(),
            new_mat_thickness: 2.0,
        }
    }
}

/// Format a duration in seconds into a compact human-readable string (e.g. `23.0s`, `01:45`, `1:02:15`).
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
        self.paused_checkpoint = None;
        self.pause_buffer.clear();

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

    pub fn stop_run(&mut self, controller: &Option<LaserController>, status: &mut MachineStatus) {
        let was_dry = self.run_mode == RunMode::DryCut;
        self.run_mode = RunMode::Idle;
        self.is_paused = false;
        self.last_tick = None;
        self.last_hw_dispatched_seg = None;
        self.full_trajectory.clear();
        self.estimated_job_secs = 0.0;
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

        if self.full_trajectory.is_empty() {
            self.full_trajectory = self.trajectory.clone();
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

        let new_px = pulses_cfg.mm_to_pulses_x(point_mm.x);
        let new_py = pulses_cfg.mm_to_pulses_y(point_mm.y);
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
            let origin_head = kurbo::Point::new(
                pulses_cfg.pulses_to_mm_x(status.pos_x),
                pulses_cfg.pulses_to_mm_y(status.pos_y),
            );
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

        let new_px = pulses_cfg.mm_to_pulses_x(best_pt.x);
        let new_py = pulses_cfg.mm_to_pulses_y(best_pt.y);
        if let Some(ref c) = controller {
            if c.is_connected() {
                c.stop();
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
            "Head moved to Path #{} @ X: {:.2}, Y: {:.2} mm (Pause mode). Press Continue or step along path.",
            best_seg.path_index_1based, best_pt.x, best_pt.y
        ))
    }

    /// Synchronize updated layer cutting & travel speeds into active and buffered trajectories
    /// so speed edits in Cutting Layers take effect immediately (even while running or paused).
    pub fn sync_layer_speeds(&mut self, doc: &Document) {
        let seq = doc.compute_cut_sequence();
        let travel_speed = doc.travel_speed_mm_s(200.0);
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
        let speed_mult = if hw_connected || self.run_mode != RunMode::DryCut {
            1.0
        } else {
            self.sim_speed_multiplier
        };

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
                let pos_z_mm = pulses_cfg.pulses_to_mm_z(status.pos_z);

                let card_fill = if cfg!(target_os = "windows") {
                    Color32::from_rgb(22, 26, 34)
                } else {
                    Color32::from_rgb(18, 20, 24)
                };
                let card_header_color = if cfg!(target_os = "windows") {
                    Color32::from_rgb(185, 200, 225)
                } else {
                    Color32::from_rgb(140, 150, 170)
                };
                let standby_color = if cfg!(target_os = "windows") {
                    Color32::from_rgb(190, 200, 216)
                } else {
                    Color32::from_rgb(140, 145, 155)
                };

                Frame::group(ui.style()).fill(card_fill).show(ui, |ui| {
                    ui.label(RichText::new("Head Coordinates (WCS)").size(11.0).color(card_header_color).strong());
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
                Frame::group(ui.style()).fill(card_fill).show(ui, |ui| {
                    ui.label(RichText::new("Laser Source").size(11.0).color(card_header_color).strong());
                    ui.add_space(2.0);
                    Grid::new("laser_monitor_grid").num_columns(2).spacing([16.0, 3.0]).show(ui, |ui| {
                        ui.label("Emission:");
                        if status.laser_on {
                            ui.colored_label(Color32::from_rgb(255, 60, 60), "● ACTIVE / FIRING");
                        } else {
                            ui.colored_label(standby_color, "○ READY / STANDBY");
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
                Frame::group(ui.style()).fill(card_fill).show(ui, |ui| {
                    ui.label(RichText::new("Assist Gas Solenoid").size(11.0).color(card_header_color).strong());
                    ui.add_space(2.0);
                    Grid::new("gas_monitor_grid").num_columns(2).spacing([16.0, 3.0]).show(ui, |ui| {
                        ui.label("Valve Status:");
                        if status.gas_valve_open {
                            ui.colored_label(Color32::from_rgb(0, 220, 255), "● OPEN (Purging)");
                        } else {
                            ui.colored_label(standby_color, "○ CLOSED");
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
                Frame::group(ui.style()).fill(card_fill).show(ui, |ui| {
                    ui.label(RichText::new("Capacitive Height Sensor").size(11.0).color(card_header_color).strong());
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
                            ui.colored_label(standby_color, "○ STANDBY");
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

                // Connection Status Badge & Wireless Remote Control (`R1042`) Status
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

                if is_connected {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new("🎮 Remote:").size(11.5).strong());
                        let lcr_id = status.raw_registers.get(41).copied().unwrap_or(0);
                        if status.remote_button_mask != 0 {
                            ui.label(
                                RichText::new(format!("● {}", format_remote_button(status.remote_button_mask)))
                                    .size(11.5)
                                    .color(Color32::from_rgb(0, 235, 160))
                                    .strong(),
                            );
                        } else if lcr_id != 0 {
                            ui.label(
                                RichText::new(format!("Linked (ID {:08X}) · Ready", lcr_id))
                                    .size(11.0)
                                    .color(Color32::from_rgb(130, 225, 165)),
                            );
                        } else {
                            ui.label(
                                RichText::new("Ready (Listening R1042/R1043)")
                                    .size(11.0)
                                    .weak(),
                            );
                        }
                    });
                }

                ui.add_space(6.0);

                // Jog Mode & Step Selection
                let step_options = [0.1, 0.5, 1.0, 5.0, 10.0, 50.0];
                ui.horizontal(|ui| {
                    ui.label(RichText::new("Jog Mode:").strong());
                    let is_step = matches!(panel_state.jog_mode, JogMode::Step(_));
                    if ui.selectable_label(is_step, "Step").clicked() {
                        panel_state.jog_mode = JogMode::Step(step_options[panel_state.step_index.min(step_options.len() - 1)]);
                    }
                    let is_cont = matches!(panel_state.jog_mode, JogMode::Continuous);
                    if ui.selectable_label(is_cont, "Continuous").clicked() {
                        panel_state.jog_mode = JogMode::Continuous;
                    }
                });

                if let JogMode::Step(mut step_val) = panel_state.jog_mode {
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
                                    .speed(0.2)
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
                    JogMode::Continuous => 1.0,
                };
                let step_pulses_x = pulses_cfg.mm_to_pulses_x(step_dist);
                let step_pulses_y = pulses_cfg.mm_to_pulses_y(step_dist);
                let step_pulses_z = pulses_cfg.mm_to_pulses_z(step_dist);

                let btn_size = [72.0, 34.0];
                let pad_width = 72.0 * 3.0 + 12.0;
                let mut jog_cmd: Option<(Axis, i32)> = None;
                let mut held_jog: Option<(Axis, i32)> = None;
                let mut stop_clicked = false;

                ui.horizontal(|ui| {
                    ui.add_space((ui.available_width() - pad_width).max(0.0) / 2.0);
                    Grid::new("jog_pad_3x3")
                        .num_columns(3)
                        .spacing([6.0, 6.0])
                        .show(ui, |ui| {
                            // Row 1: [empty] | Y+ | Z+
                            ui.allocate_exact_size(egui::vec2(btn_size[0], btn_size[1]), egui::Sense::hover());
                            let y_plus_resp = ui.add_sized(btn_size, Button::new(RichText::new("Y+ ▲").size(13.0)));
                            if y_plus_resp.clicked() {
                                jog_cmd = Some((Axis::Y, step_pulses_y));
                            }
                            if y_plus_resp.is_pointer_button_down_on() {
                                held_jog = Some((Axis::Y, 1));
                            }

                            let z_plus_btn = Button::new(RichText::new("Z+ ▲").size(13.0).color(Color32::from_rgb(130, 195, 255)))
                                .fill(Color32::from_rgb(28, 38, 56));
                            let z_plus_resp = ui.add_sized(btn_size, z_plus_btn).on_hover_text("Raise cutting head (Z+)");
                            if z_plus_resp.clicked() {
                                jog_cmd = Some((Axis::Z, step_pulses_z));
                            }
                            if z_plus_resp.is_pointer_button_down_on() {
                                held_jog = Some((Axis::Z, 1));
                            }
                            ui.end_row();

                            // Row 2: X- | STOP | X+ (middle between Z+ and Z-)
                            let x_minus_resp = ui.add_sized(btn_size, Button::new(RichText::new("◄ X-").size(13.0)));
                            if x_minus_resp.clicked() {
                                jog_cmd = Some((Axis::X, -step_pulses_x));
                            }
                            if x_minus_resp.is_pointer_button_down_on() {
                                held_jog = Some((Axis::X, -1));
                            }

                            let stop_btn = Button::new(RichText::new("STOP").color(Color32::WHITE).strong())
                                .fill(Color32::from_rgb(180, 40, 40));
                            if ui.add_sized(btn_size, stop_btn).clicked() {
                                stop_clicked = true;
                            }

                            let x_plus_resp = ui.add_sized(btn_size, Button::new(RichText::new("X+ ►").size(13.0)));
                            if x_plus_resp.clicked() {
                                jog_cmd = Some((Axis::X, step_pulses_x));
                            }
                            if x_plus_resp.is_pointer_button_down_on() {
                                held_jog = Some((Axis::X, 1));
                            }
                            ui.end_row();

                            // Row 3: [empty] | Y- | Z-
                            ui.allocate_exact_size(egui::vec2(btn_size[0], btn_size[1]), egui::Sense::hover());
                            let y_minus_resp = ui.add_sized(btn_size, Button::new(RichText::new("Y- ▼").size(13.0)));
                            if y_minus_resp.clicked() {
                                jog_cmd = Some((Axis::Y, -step_pulses_y));
                            }
                            if y_minus_resp.is_pointer_button_down_on() {
                                held_jog = Some((Axis::Y, -1));
                            }

                            let z_minus_btn = Button::new(RichText::new("Z- ▼").size(13.0).color(Color32::from_rgb(130, 195, 255)))
                                .fill(Color32::from_rgb(28, 38, 56));
                            let z_minus_resp = ui.add_sized(btn_size, z_minus_btn).on_hover_text("Lower cutting head (Z-)");
                            if z_minus_resp.clicked() {
                                jog_cmd = Some((Axis::Z, -step_pulses_z));
                            }
                            if z_minus_resp.is_pointer_button_down_on() {
                                held_jog = Some((Axis::Z, -1));
                            }
                            ui.end_row();
                        });
                });

                if stop_clicked {
                    panel_state.active_continuous_jog = None;
                    panel_state.stop_run(controller, status);
                    *status_message = "Motion stopped.".to_string();
                } else if matches!(panel_state.jog_mode, JogMode::Continuous) {
                    if let Some((axis, raw_dir)) = held_jog {
                        let hw_dir = match axis {
                            Axis::X if pulses_cfg.invert_x => -raw_dir,
                            Axis::Y if pulses_cfg.invert_y => -raw_dir,
                            Axis::Z if pulses_cfg.invert_z => -raw_dir,
                            _ => raw_dir,
                        };
                        if panel_state.active_continuous_jog != Some((axis, hw_dir)) {
                            panel_state.active_continuous_jog = Some((axis, hw_dir));
                            if is_connected {
                                if let Some(c) = controller {
                                    c.start_continuous_jog(axis, hw_dir);
                                }
                            }
                            let sign_str = if raw_dir >= 0 { "+" } else { "-" };
                            *status_message = format!("Continuous jogging {:?}{}...", axis, sign_str);
                        }
                        if !is_connected {
                            let cont_delta = (0.5 * pulses_cfg.pulses_per_mm_x).round() as i32 * hw_dir;
                            match axis {
                                Axis::X => status.pos_x += cont_delta,
                                Axis::Y => status.pos_y += cont_delta,
                                Axis::Z => {
                                    status.pos_z += cont_delta;
                                    status.capacitive_height_mm =
                                        ((status.pos_z as f32).abs() / (pulses_cfg.pulses_per_mm_z as f32)).max(0.0);
                                }
                                Axis::W => status.pos_w += cont_delta,
                            }
                        }
                        ui.ctx().request_repaint();
                    } else if let Some((axis, _)) = panel_state.active_continuous_jog.take() {
                        if is_connected {
                            if let Some(c) = controller {
                                c.stop();
                            }
                        }
                        *status_message = format!("Stopped {:?} jog.", axis);
                    }
                } else {
                    if panel_state.active_continuous_jog.take().is_some() {
                        if is_connected {
                            if let Some(c) = controller {
                                c.stop();
                            }
                        }
                    }
                    if let Some((axis, pulses)) = jog_cmd {
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
                                        ((status.pos_z as f32).abs() / (pulses_cfg.pulses_per_mm_z as f32)).max(0.0);
                                }
                                Axis::W => status.pos_w += pulses,
                            }
                        }
                        let sign_mm = if pulses >= 0 { step_dist } else { -step_dist };
                        *status_message = format!("Step jog {:?} {:+.2} mm ({:+} pulses)", axis, sign_mm, pulses);
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
                        .add_enabled_ui(is_connected, |ui| ui.add_sized([run_btn_w, 32.0], start_btn))
                        .inner
                        .on_hover_text(if has_pause {
                            "Restart cutting job from the beginning (discards pause checkpoint)"
                        } else {
                            "Start cutting job along sorted toolpath sequence"
                        })
                        .on_disabled_hover_text("Controller not connected (Start disabled offline)")
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

                ui.add_space(4.0);

                // Row 3: ⏮ Backward  |  ⏭ Forward (active only while paused)
                ui.horizontal(|ui| {
                    let can_step = panel_state.is_paused && has_pause;
                    let step_fill = if can_step {
                        Color32::from_rgb(65, 95, 155)
                    } else {
                        Color32::from_rgb(48, 50, 62)
                    };

                    let back_btn = Button::new(RichText::new("⏮ Backward").size(12.5).color(Color32::WHITE).strong())
                        .fill(step_fill);
                    if ui
                        .add_enabled_ui(can_step, |ui| ui.add_sized([run_btn_w, 30.0], back_btn))
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
                        .add_enabled_ui(can_step, |ui| ui.add_sized([run_btn_w, 30.0], fwd_btn))
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

                // Pause Buffer Card (visible when a paused checkpoint is active)
                if let Some(cp) = panel_state.paused_checkpoint {
                    ui.add_space(4.0);
                    Frame::group(ui.style())
                        .fill(Color32::from_rgb(36, 30, 20))
                        .stroke(egui::Stroke::new(1.0_f32, Color32::from_rgb(205, 145, 38)))
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

                // Fast-forward speed multiplier strictly for offline Dry-Cut simulation
                if !is_connected && panel_state.run_mode != RunMode::Cutting {
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
                    } else if cfg!(target_os = "windows") {
                        Color32::from_rgb(34, 39, 49)
                    } else {
                        Color32::from_rgb(28, 30, 36)
                    };
                    let row_stroke = if is_open {
                        egui::Stroke::new(1.0_f32, Color32::from_rgb(80, 175, 235))
                    } else if cfg!(target_os = "windows") {
                        egui::Stroke::new(1.0_f32, Color32::from_rgb(64, 73, 92))
                    } else {
                        egui::Stroke::new(1.0_f32, Color32::from_rgb(44, 47, 56))
                    };
                    let summary_color = if cfg!(target_os = "windows") {
                        Color32::from_rgb(208, 218, 235)
                    } else {
                        Color32::from_rgb(175, 185, 200)
                    };

                    Frame::group(ui.style())
                        .fill(row_fill)
                        .stroke(row_stroke)
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                if !layer.is_travel {
                                    ui.checkbox(&mut layer.visible, "");
                                } else {
                                    ui.add_space(20.0);
                                }
                                let swatch_color = Color32::from_rgb(layer.color.r, layer.color.g, layer.color.b);
                                let summary = if layer.is_travel {
                                    format!(
                                        "{:.0} mm/s · H:{:.1} mm",
                                        layer.params.speed_mm_s,
                                        layer.params.cut_height_mm
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
                                            .color(summary_color),
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
        assert!(state.continue_run(cp5.point_mm, &None).is_some());
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

        // From stopped (Idle) state, move head to clicked path point and enter Pause mode
        assert_eq!(state.run_mode, RunMode::Idle);
        let msg = state.set_pause_at_path_point(&doc, snapped_pt, &None, &mut status, &pulses);
        assert!(msg.is_some());
        assert!(state.is_paused);
        assert_eq!(state.run_mode, RunMode::DryCut);
        assert!(!status.laser_on);
        assert!((pulses.pulses_to_mm_x(status.pos_x) - 100.0).abs() < 0.05);
        assert!((pulses.pulses_to_mm_y(status.pos_y) - 50.0).abs() < 0.05);
        assert_eq!(state.pause_buffer.len(), 1);
        let cp = state.paused_checkpoint.unwrap();
        assert!((cp.point_mm.x - 100.0).abs() < 1e-6 && (cp.point_mm.y - 50.0).abs() < 1e-6);

        // Stepping forward and continuing from this paused point work seamlessly
        assert!(state.step_along_path(true, 10.0, &None, &mut status, &pulses).is_some());
        let stepped = state.paused_checkpoint.unwrap().point_mm;
        assert!((stepped.x - 110.0).abs() < 1e-6 && (stepped.y - 50.0).abs() < 1e-6);

        assert!(state.continue_run(stepped, &None).is_some());
        assert!(!state.is_paused);
        assert!((state.trajectory[0].start.x - 110.0).abs() < 1e-6);

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
        // Seg 0 is rapid (200 mm/s), Seg 1 is cut (80 mm/s)
        assert!(state.trajectory[0].is_rapid);
        assert!(!state.trajectory[1].is_rapid);
        assert!((state.trajectory[1].speed_mm_s - 80.0).abs() < 1e-6);

        // User edits layer speed in Cutting Layers while paused or running -> sync updates trajectory
        doc.layers[0].params.speed_mm_s = 12.5;
        state.sync_layer_speeds(&doc);
        assert!((state.trajectory[0].speed_mm_s - 200.0).abs() < 1e-6);
        assert!((state.trajectory[1].speed_mm_s - 12.5).abs() < 1e-6);
        assert!((state.full_trajectory[1].speed_mm_s - 12.5).abs() < 1e-6);

        state.stop_run(&None, &mut status);
    }
}
