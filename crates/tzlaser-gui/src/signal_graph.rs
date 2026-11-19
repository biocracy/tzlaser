use eframe::egui::{
    self, pos2, vec2, Align2, Button, Color32, FontId, Pos2, Rect, RichText, Stroke, Ui,
};
use tzlaser_protocol::{AxisSignalSample, LaserController, SignalTelemetry};

pub struct SignalGraphState {
    pub expanded: bool,
    pub frozen: bool,
    pub window_secs: f64,
    pub show_x: bool,
    pub show_y: bool,
    pub show_z: bool,
    pub show_w: bool,
    pub show_ax: bool,
    pub show_ay: bool,
    pub show_az: bool,
    pub show_aw: bool,
    pub show_cap: bool,
    pub show_laser: bool,
    pub show_gas: bool,
    pub offline_telemetry: SignalTelemetry,
    pub frozen_snapshot: Option<(f64, Vec<AxisSignalSample>)>,
    pub last_active_snapshot: Option<(f64, Vec<AxisSignalSample>)>,
}

impl Default for SignalGraphState {
    fn default() -> Self {
        Self {
            expanded: true,
            frozen: false,
            window_secs: 15.0,
            show_x: true,
            show_y: true,
            show_z: true,
            show_w: true,
            show_ax: true,
            show_ay: true,
            show_az: false,
            show_aw: false,
            show_cap: false,
            show_laser: false,
            show_gas: false,
            offline_telemetry: SignalTelemetry::new(),
            frozen_snapshot: None,
            last_active_snapshot: None,
        }
    }
}

impl SignalGraphState {
    pub fn record_offline(
        &mut self,
        x_rate: f32,
        y_rate: f32,
        z_rate: f32,
        w_rate: f32,
        laser_on: bool,
        tag: Option<&str>,
    ) {
        self.offline_telemetry
            .push_command(x_rate, y_rate, z_rate, w_rate, laser_on, tag);
    }

    #[allow(clippy::too_many_arguments)]
    pub fn record_offline_full(
        &mut self,
        x_rate: f32,
        y_rate: f32,
        z_rate: f32,
        w_rate: f32,
        laser_on: bool,
        laser_power_pct: f32,
        laser_freq_hz: f32,
        capacitance_pf: f32,
        gas_valve_open: bool,
        tag: Option<&str>,
    ) {
        self.offline_telemetry.push_command_full(
            x_rate,
            y_rate,
            z_rate,
            w_rate,
            laser_on,
            laser_power_pct,
            laser_freq_hz,
            capacitance_pf,
            gas_valve_open,
            tag,
        );
    }

    pub fn tick_offline(&mut self) {
        self.offline_telemetry.tick_sample(50);
    }
}

/// Map laser pulse frequency (Hz) logarithmically onto a blue -> green -> yellow -> red/magenta ramp.
fn laser_freq_to_color(freq_hz: f32) -> Color32 {
    if freq_hz <= 0.0 {
        // CW / unspecified frequency
        return Color32::from_rgb(255, 130, 215);
    }
    let min_ln = 50.0_f32.ln();
    let max_ln = 20_000.0_f32.ln();
    let t = ((freq_hz.clamp(50.0, 20_000.0).ln() - min_ln) / (max_ln - min_ln)).clamp(0.0, 1.0);
    let lerp = |a: u8, b: u8, f: f32| -> u8 {
        (a as f32 + (b as f32 - a as f32) * f).round().clamp(0.0, 255.0) as u8
    };
    if t < 0.333 {
        let f = t / 0.333;
        Color32::from_rgb(lerp(70, 75, f), lerp(165, 235, f), lerp(255, 145, f))
    } else if t < 0.667 {
        let f = (t - 0.333) / 0.334;
        Color32::from_rgb(lerp(75, 255, f), lerp(235, 215, f), lerp(145, 65, f))
    } else {
        let f = (t - 0.667) / 0.333;
        Color32::from_rgb(lerp(255, 255, f), lerp(215, 70, f), lerp(65, 115, f))
    }
}

fn format_rate(rate: f32) -> String {
    let abs = rate.abs();
    if abs >= 1000.0 {
        format!("{:+.1}k", rate / 1000.0)
    } else {
        format!("{:+.0}", rate)
    }
}

pub fn render_signal_graph(
    ui: &mut Ui,
    state: &mut SignalGraphState,
    controller: &Option<LaserController>,
    is_active_motion: bool,
) {
    let (now_secs, samples) = if state.frozen {
        if let Some(ref snap) = state.frozen_snapshot {
            snap.clone()
        } else {
            let snap = fetch_snapshot(state, controller, is_active_motion);
            state.frozen_snapshot = Some(snap.clone());
            snap
        }
    } else {
        state.frozen_snapshot = None;
        fetch_snapshot(state, controller, is_active_motion)
    };

    let latest = samples.last().cloned().unwrap_or(AxisSignalSample {
        timestamp_secs: now_secs,
        x_rate: 0.0,
        y_rate: 0.0,
        z_rate: 0.0,
        w_rate: 0.0,
        laser_on: false,
        laser_power_pct: 0.0,
        laser_freq_hz: 0.0,
        capacitance_pf: 0.0,
        gas_valve_open: false,
        tag: None,
    });

    // Header strip
    ui.horizontal(|ui| {
        ui.label(
            RichText::new("📈 Axis Command Signals (pls/s)")
                .size(11.5)
                .strong()
                .color(Color32::from_rgb(180, 215, 255)),
        );
        ui.separator();

        let toggle_pill = |ui: &mut Ui, label: &str, active: &mut bool, col: Color32| {
            let fill = if *active {
                Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), 38)
            } else {
                Color32::from_rgb(30, 34, 42)
            };
            let fg = if *active { col } else { Color32::from_rgb(115, 120, 132) };
            if ui
                .add(
                    Button::new(RichText::new(label).size(10.5).color(fg).strong())
                        .fill(fill)
                        .min_size(vec2(26.0, 18.0)),
                )
                .clicked()
            {
                *active = !*active;
            }
        };

        toggle_pill(ui, "X", &mut state.show_x, Color32::from_rgb(255, 95, 95));
        toggle_pill(ui, "Y", &mut state.show_y, Color32::from_rgb(80, 230, 125));
        toggle_pill(ui, "Z", &mut state.show_z, Color32::from_rgb(95, 175, 255));
        toggle_pill(ui, "W", &mut state.show_w, Color32::from_rgb(255, 200, 80));

        ui.separator();
        ui.label(RichText::new("Acc:").size(10.0).weak());
        toggle_pill(ui, "aX", &mut state.show_ax, Color32::from_rgb(255, 145, 70));
        toggle_pill(ui, "aY", &mut state.show_ay, Color32::from_rgb(100, 235, 210));
        toggle_pill(ui, "aZ", &mut state.show_az, Color32::from_rgb(165, 145, 255));
        toggle_pill(ui, "aW", &mut state.show_aw, Color32::from_rgb(245, 225, 130));

        ui.separator();
        ui.label(RichText::new("Aux:").size(10.0).weak());
        toggle_pill(ui, "Cap", &mut state.show_cap, Color32::from_rgb(70, 220, 235));
        toggle_pill(ui, "Laser", &mut state.show_laser, Color32::from_rgb(245, 130, 200));
        toggle_pill(ui, "Gas", &mut state.show_gas, Color32::from_rgb(125, 240, 145));

        ui.separator();
        for w in [5.0_f64, 15.0, 30.0, 60.0] {
            let sel = (state.window_secs - w).abs() < 0.1;
            if ui
                .selectable_label(sel, RichText::new(format!("{:.0}s", w)).size(10.5))
                .clicked()
            {
                state.window_secs = w;
            }
        }

        ui.separator();
        let freeze_lbl = if state.frozen { "▶ Live" } else { "⏸ Freeze" };
        let freeze_fill = if state.frozen {
            Color32::from_rgb(125, 85, 24)
        } else {
            Color32::from_rgb(34, 38, 48)
        };
        if ui
            .add(Button::new(RichText::new(freeze_lbl).size(10.5)).fill(freeze_fill))
            .clicked()
        {
            state.frozen = !state.frozen;
            if !state.frozen {
                state.frozen_snapshot = None;
            }
        }

        if ui
            .add(Button::new(RichText::new("Clear").size(10.5)))
            .clicked()
        {
            if let Some(ref c) = controller {
                if c.is_connected() {
                    c.telemetry().lock().unwrap().clear();
                }
            }
            state.offline_telemetry.clear();
            state.frozen_snapshot = None;
            state.last_active_snapshot = None;
        }

        ui.separator();
        let cap_str = if state.show_cap {
            format!("  C:{:.1}pF", latest.capacitance_pf)
        } else {
            String::new()
        };
        let laser_str = if latest.laser_on {
            if latest.laser_freq_hz > 0.0 {
                format!("  [LASER {:.0}%@{:.0}Hz]", latest.laser_power_pct, latest.laser_freq_hz)
            } else {
                format!("  [LASER {:.0}%]", latest.laser_power_pct)
            }
        } else if state.show_laser {
            "  [LASER OFF]".to_string()
        } else {
            String::new()
        };
        let gas_str = if state.show_gas || latest.gas_valve_open {
            if latest.gas_valve_open { "  [GAS:OPEN]" } else { "  [GAS:SHUT]" }
        } else {
            ""
        };
        ui.label(
            RichText::new(format!(
                "X:{}  Y:{}  Z:{}  W:{}{}{}{}",
                format_rate(latest.x_rate),
                format_rate(latest.y_rate),
                format_rate(latest.z_rate),
                format_rate(latest.w_rate),
                cap_str,
                laser_str,
                gas_str,
            ))
            .size(10.5)
            .monospace()
            .color(Color32::from_rgb(195, 205, 220)),
        );

        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui
                .add(
                    Button::new(RichText::new("▼").size(11.0).strong())
                        .min_size(vec2(24.0, 18.0)),
                )
                .on_hover_text("Collapse Axis Command Signal Graph")
                .clicked()
            {
                state.expanded = false;
            }
        });
    });

    ui.add_space(2.0);

    // Plot canvas
    let avail = ui.available_size();
    let plot_h = avail.y.max(55.0);
    let (rect, _resp) = ui.allocate_exact_size(vec2(avail.x.max(100.0), plot_h), egui::Sense::hover());
    let painter = ui.painter_at(rect);

    painter.rect_filled(rect, 3.0, Color32::from_rgb(14, 17, 22));
    painter.rect_stroke(
        rect,
        3.0,
        Stroke::new(1.0_f32, Color32::from_rgb(42, 48, 60)),
        egui::StrokeKind::Inside,
    );

    let any_vel = state.show_x || state.show_y || state.show_z || state.show_w;
    let any_acc = state.show_ax || state.show_ay || state.show_az || state.show_aw;
    let any_bipolar = any_vel || any_acc;
    let any_unipolar = state.show_cap || state.show_laser || state.show_gas;

    let pad_left = if any_vel { 48.0 } else { 14.0 };
    let mut pad_right = 8.0_f32;
    if any_acc {
        pad_right += 50.0;
    }
    if state.show_cap {
        pad_right += 46.0;
    }
    if state.show_laser {
        pad_right += 40.0;
    }
    if state.show_gas {
        pad_right += 36.0;
    }
    let pad_top = 12.0;
    let pad_bottom = 18.0;
    let inner = Rect::from_min_max(
        pos2(rect.left() + pad_left, rect.top() + pad_top),
        pos2(
            (rect.right() - pad_right).max(rect.left() + pad_left + 10.0),
            (rect.bottom() - pad_bottom).max(rect.top() + pad_top + 10.0),
        ),
    );

    let t_max = now_secs;
    let t_min = t_max - state.window_secs;

    // Collect visible samples plus one sample just before t_min so the line enters cleanly from the left
    let mut visible: Vec<&AxisSignalSample> = Vec::new();
    let mut prev_before: Option<&AxisSignalSample> = None;
    for s in &samples {
        if s.timestamp_secs < t_min {
            prev_before = Some(s);
        } else if s.timestamp_secs <= t_max {
            if visible.is_empty() {
                if let Some(pb) = prev_before {
                    visible.push(pb);
                }
            }
            visible.push(s);
        }
    }
    if visible.is_empty() {
        if let Some(pb) = prev_before {
            visible.push(pb);
        }
    }

    // Compute symmetric Y scale (pls/s)
    let mut peak = 500.0_f32;
    for s in &visible {
        if state.show_x {
            peak = peak.max(s.x_rate.abs());
        }
        if state.show_y {
            peak = peak.max(s.y_rate.abs());
        }
        if state.show_z {
            peak = peak.max(s.z_rate.abs());
        }
        if state.show_w {
            peak = peak.max(s.w_rate.abs());
        }
    }
    let y_max = (peak * 1.15).max(500.0);

    // Compute per-sample acceleration (pls/s^2) aligned with `visible`
    let mut acc_samples: Vec<[f32; 4]> = Vec::with_capacity(visible.len());
    for i in 0..visible.len() {
        if i == 0 {
            acc_samples.push([0.0; 4]);
        } else {
            let dt = (visible[i].timestamp_secs - visible[i - 1].timestamp_secs).max(0.01) as f32;
            acc_samples.push([
                (visible[i].x_rate - visible[i - 1].x_rate) / dt,
                (visible[i].y_rate - visible[i - 1].y_rate) / dt,
                (visible[i].z_rate - visible[i - 1].z_rate) / dt,
                (visible[i].w_rate - visible[i - 1].w_rate) / dt,
            ]);
        }
    }

    let mut acc_peak = 1000.0_f32;
    for a in &acc_samples {
        if state.show_ax {
            acc_peak = acc_peak.max(a[0].abs());
        }
        if state.show_ay {
            acc_peak = acc_peak.max(a[1].abs());
        }
        if state.show_az {
            acc_peak = acc_peak.max(a[2].abs());
        }
        if state.show_aw {
            acc_peak = acc_peak.max(a[3].abs());
        }
    }
    let a_max = (acc_peak * 1.15).max(1000.0);

    // Compute unipolar Capacitance scale (pF, >= 0)
    let mut cap_peak = 15.0_f32;
    for s in &visible {
        if s.capacitance_pf.is_finite() && s.capacitance_pf > 0.0 {
            cap_peak = cap_peak.max(s.capacitance_pf);
        }
    }
    let cap_max = (cap_peak * 1.15).max(20.0);

    let to_x = |t: f64| -> f32 {
        let u = ((t - t_min) / state.window_secs).clamp(0.0, 1.0) as f32;
        inner.left() + u * inner.width()
    };
    let to_screen = |t: f64, val: f32| -> Pos2 {
        let v = (val / y_max).clamp(-1.0, 1.0);
        pos2(to_x(t), inner.center().y - v * (inner.height() * 0.5))
    };
    let to_screen_acc = |t: f64, val: f32| -> Pos2 {
        let v = (val / a_max).clamp(-1.0, 1.0);
        pos2(to_x(t), inner.center().y - v * (inner.height() * 0.5))
    };
    // Unipolar [0, max] mapping with dedicated X-axis at inner.bottom()
    let to_screen_unipolar = |t: f64, norm_0_to_1: f32| -> Pos2 {
        let v = norm_0_to_1.clamp(0.0, 1.0);
        pos2(to_x(t), inner.bottom() - v * inner.height())
    };

    // Grid lines & Y labels
    let zero_y = inner.center().y;
    if any_bipolar {
        painter.line_segment(
            [pos2(inner.left(), zero_y), pos2(inner.right(), zero_y)],
            Stroke::new(1.0_f32, Color32::from_rgb(55, 62, 75)),
        );
        for frac in [-1.0_f32, -0.5, 0.5, 1.0] {
            let y = zero_y - frac * (inner.height() * 0.5);
            painter.line_segment(
                [pos2(inner.left(), y), pos2(inner.right(), y)],
                Stroke::new(0.6_f32, Color32::from_rgb(28, 33, 42)),
            );
        }
    } else {
        for frac in [0.25_f32, 0.5, 0.75, 1.0] {
            let y = inner.bottom() - frac * inner.height();
            painter.line_segment(
                [pos2(inner.left(), y), pos2(inner.right(), y)],
                Stroke::new(0.6_f32, Color32::from_rgb(28, 33, 42)),
            );
        }
    }

    if any_vel {
        painter.text(
            pos2(inner.left() - 4.0, inner.top()),
            Align2::RIGHT_TOP,
            format!("+{}", format_rate(y_max).trim_start_matches('+')),
            FontId::monospace(9.5),
            Color32::from_rgb(130, 138, 152),
        );
        painter.text(
            pos2(inner.left() - 4.0, zero_y),
            Align2::RIGHT_CENTER,
            "0",
            FontId::monospace(9.5),
            Color32::from_rgb(130, 138, 152),
        );
        painter.text(
            pos2(inner.left() - 4.0, inner.bottom()),
            Align2::RIGHT_BOTTOM,
            format!("-{}", format_rate(y_max).trim_start_matches('+')),
            FontId::monospace(9.5),
            Color32::from_rgb(130, 138, 152),
        );
    }

    // Separate X-axis (0+ baseline at inner.bottom()) for unipolar (>0) overlay traces
    if any_unipolar {
        let uni_axis_col = Color32::from_rgb(80, 155, 195);
        painter.line_segment(
            [pos2(inner.left(), inner.bottom()), pos2(inner.right(), inner.bottom())],
            Stroke::new(1.3_f32, uni_axis_col),
        );
        if !any_vel {
            painter.text(
                pos2(inner.left() - 3.0, inner.bottom()),
                Align2::RIGHT_BOTTOM,
                "0+",
                FontId::monospace(8.5),
                uni_axis_col,
            );
        }
    }

    // Right-side Y-axes (each active channel gets its own non-overlapping column)
    let mut right_x = inner.right() + 4.0;
    if any_acc {
        let acc_lbl_col = Color32::from_rgb(175, 165, 210);
        painter.line_segment(
            [pos2(right_x - 2.0, inner.top()), pos2(right_x - 2.0, inner.bottom())],
            Stroke::new(0.8_f32, Color32::from_rgba_unmultiplied(175, 165, 210, 90)),
        );
        painter.text(
            pos2(right_x + 1.0, inner.top()),
            Align2::LEFT_TOP,
            format!("+{}", format_rate(a_max).trim_start_matches('+')),
            FontId::monospace(9.5),
            acc_lbl_col,
        );
        painter.text(
            pos2(right_x + 1.0, zero_y),
            Align2::LEFT_CENTER,
            "pls/s²",
            FontId::monospace(8.5),
            acc_lbl_col,
        );
        painter.text(
            pos2(right_x + 1.0, inner.bottom()),
            Align2::LEFT_BOTTOM,
            format!("-{}", format_rate(a_max).trim_start_matches('+')),
            FontId::monospace(9.5),
            acc_lbl_col,
        );
        right_x += 50.0;
    }

    if state.show_cap {
        let cap_col = Color32::from_rgb(70, 220, 235);
        painter.line_segment(
            [pos2(right_x - 2.0, inner.top()), pos2(right_x - 2.0, inner.bottom())],
            Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(70, 220, 235, 130)),
        );
        painter.text(
            pos2(right_x + 1.0, inner.top()),
            Align2::LEFT_TOP,
            format!("{:.1}pF", cap_max),
            FontId::monospace(9.0),
            cap_col,
        );
        painter.text(
            pos2(right_x + 1.0, inner.center().y),
            Align2::LEFT_CENTER,
            format!("{:.1}", cap_max * 0.5),
            FontId::monospace(8.5),
            cap_col,
        );
        painter.text(
            pos2(right_x + 1.0, inner.bottom()),
            Align2::LEFT_BOTTOM,
            "0pF",
            FontId::monospace(9.0),
            cap_col,
        );
        right_x += 46.0;
    }

    if state.show_laser {
        let laser_col = Color32::from_rgb(245, 130, 200);
        painter.line_segment(
            [pos2(right_x - 2.0, inner.top()), pos2(right_x - 2.0, inner.bottom())],
            Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(245, 130, 200, 130)),
        );
        painter.text(
            pos2(right_x + 1.0, inner.top()),
            Align2::LEFT_TOP,
            "100%",
            FontId::monospace(9.0),
            laser_col,
        );
        painter.text(
            pos2(right_x + 1.0, inner.center().y),
            Align2::LEFT_CENTER,
            "50%",
            FontId::monospace(8.5),
            laser_col,
        );
        painter.text(
            pos2(right_x + 1.0, inner.bottom()),
            Align2::LEFT_BOTTOM,
            "0%",
            FontId::monospace(9.0),
            laser_col,
        );
        right_x += 40.0;
    }

    if state.show_gas {
        let gas_col = Color32::from_rgb(125, 240, 145);
        let y_open = inner.bottom() - inner.height() * 0.88;
        painter.line_segment(
            [pos2(right_x - 2.0, inner.top()), pos2(right_x - 2.0, inner.bottom())],
            Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(125, 240, 145, 130)),
        );
        painter.text(
            pos2(right_x + 1.0, y_open),
            Align2::LEFT_CENTER,
            "OPEN",
            FontId::monospace(8.5),
            gas_col,
        );
        painter.text(
            pos2(right_x + 1.0, inner.bottom()),
            Align2::LEFT_BOTTOM,
            "SHUT",
            FontId::monospace(8.5),
            gas_col,
        );
    }

    // Time grid ticks
    let num_ticks = 5usize;
    for i in 0..=num_ticks {
        let u = (i as f32) / (num_ticks as f32);
        let x = inner.left() + u * inner.width();
        painter.line_segment(
            [pos2(x, inner.top()), pos2(x, inner.bottom())],
            Stroke::new(0.6_f32, Color32::from_rgb(28, 33, 42)),
        );
        let rel_t = -state.window_secs * (1.0 - u as f64);
        let lbl = if rel_t.abs() < 0.05 {
            "now".to_string()
        } else {
            format!("{:.0}s", rel_t)
        };
        let align = if i == 0 {
            Align2::LEFT_TOP
        } else if i == num_ticks {
            Align2::RIGHT_TOP
        } else {
            Align2::CENTER_TOP
        };
        painter.text(
            pos2(x, inner.bottom() + 2.0),
            align,
            lbl,
            FontId::monospace(9.0),
            Color32::from_rgb(120, 128, 142),
        );
    }

    if visible.is_empty() {
        return;
    }

    // Laser active shading
    for i in 0..visible.len() {
        if visible[i].laser_on {
            let t0 = visible[i].timestamp_secs.max(t_min);
            let t1 = if i + 1 < visible.len() {
                visible[i + 1].timestamp_secs.min(t_max)
            } else {
                t_max
            };
            if t1 > t0 {
                let x0 = to_screen(t0, 0.0).x;
                let x1 = to_screen(t1, 0.0).x;
                painter.rect_filled(
                    Rect::from_min_max(pos2(x0, inner.top()), pos2(x1, inner.bottom())),
                    0.0,
                    Color32::from_rgba_unmultiplied(235, 75, 165, 22),
                );
            }
        }
    }

    // Event markers (throttled so labels don't overlap)
    let mut last_tag_x = f32::NEG_INFINITY;
    for s in &visible {
        if s.timestamp_secs >= t_min {
            if let Some(ref tag) = s.tag {
                let x = to_screen(s.timestamp_secs, 0.0).x;
                if x - last_tag_x >= 44.0 {
                    painter.line_segment(
                        [pos2(x, inner.top()), pos2(x, inner.bottom())],
                        Stroke::new(0.8_f32, Color32::from_rgba_unmultiplied(150, 175, 220, 75)),
                    );
                    painter.text(
                        pos2(x + 2.0, inner.top() + 1.0),
                        Align2::LEFT_TOP,
                        tag,
                        FontId::proportional(8.5),
                        Color32::from_rgba_unmultiplied(190, 210, 245, 190),
                    );
                    last_tag_x = x;
                }
            }
        }
    }

    // Draw axis waveforms (step-hold so command changes are crisp)
    let draw_channel = |enabled: bool, color: Color32, extract: fn(&AxisSignalSample) -> f32| {
        if !enabled {
            return;
        }
        let mut pts: Vec<Pos2> = Vec::with_capacity(visible.len() * 2 + 2);
        for (i, s) in visible.iter().enumerate() {
            let val = extract(s);
            let t_clamped = s.timestamp_secs.max(t_min);
            let p = to_screen(t_clamped, val);
            if i > 0 {
                let prev_val = extract(visible[i - 1]);
                pts.push(to_screen(t_clamped, prev_val));
            }
            pts.push(p);
        }
        if let Some(last_s) = visible.last() {
            pts.push(to_screen(t_max, extract(last_s)));
        }
        if pts.len() >= 2 {
            painter.add(egui::Shape::line(pts, Stroke::new(1.5_f32, color)));
        }
    };

    draw_channel(state.show_w, Color32::from_rgb(255, 200, 80), |s| s.w_rate);
    draw_channel(state.show_z, Color32::from_rgb(95, 175, 255), |s| s.z_rate);
    draw_channel(state.show_y, Color32::from_rgb(80, 230, 125), |s| s.y_rate);
    draw_channel(state.show_x, Color32::from_rgb(255, 95, 95), |s| s.x_rate);

    // Draw acceleration curves on the secondary Y-axis
    let draw_acc_channel = |enabled: bool, color: Color32, axis_idx: usize| {
        if !enabled || visible.len() < 2 {
            return;
        }
        let mut pts: Vec<Pos2> = Vec::with_capacity(visible.len());
        for (i, s) in visible.iter().enumerate() {
            let t_clamped = s.timestamp_secs.max(t_min);
            pts.push(to_screen_acc(t_clamped, acc_samples[i][axis_idx]));
        }
        if pts.len() >= 2 {
            painter.add(egui::Shape::line(pts, Stroke::new(1.2_f32, color)));
        }
    };

    draw_acc_channel(state.show_aw, Color32::from_rgba_unmultiplied(245, 225, 130, 200), 3);
    draw_acc_channel(state.show_az, Color32::from_rgba_unmultiplied(165, 145, 255, 200), 2);
    draw_acc_channel(state.show_ay, Color32::from_rgba_unmultiplied(100, 235, 210, 210), 1);
    draw_acc_channel(state.show_ax, Color32::from_rgba_unmultiplied(255, 145, 70, 210), 0);

    // Draw Gas Valve State (unipolar 0 = SHUT, 0.88 = OPEN)
    if state.show_gas {
        let gas_col = Color32::from_rgb(125, 240, 145);
        let gas_fill = Color32::from_rgba_unmultiplied(125, 240, 145, 24);
        let mut pts: Vec<Pos2> = Vec::with_capacity(visible.len() * 2 + 2);
        for (i, s) in visible.iter().enumerate() {
            let t0 = s.timestamp_secs.max(t_min);
            let t1 = if i + 1 < visible.len() {
                visible[i + 1].timestamp_secs.min(t_max)
            } else {
                t_max
            };
            let v = if s.gas_valve_open { 0.88 } else { 0.0 };
            if s.gas_valve_open && t1 > t0 {
                let p_tl = to_screen_unipolar(t0, v);
                let p_br = to_screen_unipolar(t1, 0.0);
                painter.rect_filled(Rect::from_min_max(p_tl, p_br), 0.0, gas_fill);
            }
            if i > 0 {
                let prev_v = if visible[i - 1].gas_valve_open { 0.88 } else { 0.0 };
                pts.push(to_screen_unipolar(t0, prev_v));
            }
            pts.push(to_screen_unipolar(t0, v));
        }
        if let Some(last_s) = visible.last() {
            let v = if last_s.gas_valve_open { 0.88 } else { 0.0 };
            pts.push(to_screen_unipolar(t_max, v));
        }
        if pts.len() >= 2 {
            painter.add(egui::Shape::line(pts, Stroke::new(1.5_f32, gas_col)));
        }
    }

    // Draw Capacitive Height Sensor Capacitance (pF, unipolar 0..cap_max)
    if state.show_cap {
        let cap_col = Color32::from_rgb(70, 220, 235);
        let mut pts: Vec<Pos2> = Vec::with_capacity(visible.len() + 1);
        for s in &visible {
            let t_clamped = s.timestamp_secs.max(t_min);
            let norm = (s.capacitance_pf.max(0.0) / cap_max).clamp(0.0, 1.0);
            pts.push(to_screen_unipolar(t_clamped, norm));
        }
        if let Some(last_s) = visible.last() {
            let norm = (last_s.capacitance_pf.max(0.0) / cap_max).clamp(0.0, 1.0);
            pts.push(to_screen_unipolar(t_max, norm));
        }
        if pts.len() >= 2 {
            painter.add(egui::Shape::line(pts, Stroke::new(1.6_f32, cap_col)));
        }
    }

    // Draw Laser Power (%) as Y coordinate and Frequency (Hz) as Color
    if state.show_laser {
        for i in 0..visible.len() {
            let s = visible[i];
            let t0 = s.timestamp_secs.max(t_min);
            let t1 = if i + 1 < visible.len() {
                visible[i + 1].timestamp_secs.min(t_max)
            } else {
                t_max
            };
            let norm = if s.laser_on {
                (s.laser_power_pct / 100.0).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let col = if s.laser_on {
                laser_freq_to_color(s.laser_freq_hz)
            } else {
                Color32::from_rgba_unmultiplied(110, 115, 130, 140)
            };
            let width = if s.laser_on { 2.2_f32 } else { 1.0_f32 };

            if i > 0 {
                let prev = visible[i - 1];
                let prev_norm = if prev.laser_on {
                    (prev.laser_power_pct / 100.0).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                if (norm - prev_norm).abs() > 1e-4 {
                    let edge_col = if s.laser_on {
                        col
                    } else {
                        laser_freq_to_color(prev.laser_freq_hz)
                    };
                    painter.line_segment(
                        [to_screen_unipolar(t0, prev_norm), to_screen_unipolar(t0, norm)],
                        Stroke::new(1.6_f32, edge_col),
                    );
                }
            }
            if t1 > t0 {
                painter.line_segment(
                    [to_screen_unipolar(t0, norm), to_screen_unipolar(t1, norm)],
                    Stroke::new(width, col),
                );
            }
        }

        // Compact frequency-color legend in top-left of inner plot
        if inner.width() > 140.0 {
            let leg_x = inner.left() + 6.0;
            let leg_y = inner.top() + 4.0;
            let bar_w = 56.0_f32;
            let bar_h = 5.0_f32;
            painter.text(
                pos2(leg_x, leg_y + bar_h * 0.5),
                Align2::LEFT_CENTER,
                "50Hz",
                FontId::monospace(8.0),
                Color32::from_rgb(160, 175, 195),
            );
            let bar_start = leg_x + 26.0;
            let steps = 12usize;
            for k in 0..steps {
                let f0 = k as f32 / steps as f32;
                let f1 = (k + 1) as f32 / steps as f32;
                let freq = (50.0_f32.ln() + f0 * (20_000.0_f32.ln() - 50.0_f32.ln())).exp();
                painter.rect_filled(
                    Rect::from_min_max(
                        pos2(bar_start + f0 * bar_w, leg_y),
                        pos2(bar_start + f1 * bar_w, leg_y + bar_h),
                    ),
                    0.0,
                    laser_freq_to_color(freq),
                );
            }
            painter.text(
                pos2(bar_start + bar_w + 3.0, leg_y + bar_h * 0.5),
                Align2::LEFT_CENTER,
                "20kHz",
                FontId::monospace(8.0),
                Color32::from_rgb(160, 175, 195),
            );
        }
    }
}

fn fetch_snapshot(
    state: &mut SignalGraphState,
    controller: &Option<LaserController>,
    is_active_motion: bool,
) -> (f64, Vec<AxisSignalSample>) {
    let raw = if let Some(ref c) = controller {
        if c.is_connected() {
            let tel_arc = c.telemetry();
            let mut tel = tel_arc.lock().unwrap();
            if is_active_motion {
                tel.tick_sample(50);
            }
            tel.snapshot()
        } else {
            if is_active_motion {
                state.tick_offline();
            }
            state.offline_telemetry.snapshot()
        }
    } else {
        if is_active_motion {
            state.tick_offline();
        }
        state.offline_telemetry.snapshot()
    };

    if is_active_motion {
        state.last_active_snapshot = Some(raw.clone());
        return raw;
    }

    // Machine is paused or idle: freeze the graph unless a new discrete command (e.g. step jog) arrived
    let new_last_ts = raw.1.last().map(|s| s.timestamp_secs).unwrap_or(0.0);
    let prev_last_ts = state
        .last_active_snapshot
        .as_ref()
        .and_then(|(_, v)| v.last().map(|s| s.timestamp_secs))
        .unwrap_or(-1.0);
    if state.last_active_snapshot.is_none() || (new_last_ts - prev_last_ts).abs() > 1e-6 {
        let frozen_snap = (new_last_ts.max(0.001), raw.1);
        state.last_active_snapshot = Some(frozen_snap.clone());
        return frozen_snap;
    }

    state.last_active_snapshot.clone().unwrap_or(raw)
}
