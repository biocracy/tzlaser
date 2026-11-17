use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::motion::{
    build_fifo_finish_packets, build_fifo_packet, build_fifo_start_packets, build_init_packets,
    build_initial_enable_packets, build_jog_packet, build_stop_packets, build_xy_home_packets,
    build_xy_move_packet, build_z_calibrate_packet, build_z_handshake_packets,
    build_z_home_packet, build_z_jog_packet, build_z_poll_packet, build_z_stop_packet,
    decode_remote_jog, decode_z_poll_response, decode_z_poll_telemetry, fifo_flags, registers,
    remote_buttons, Axis, CalibrationMonitor, HomeConfig, JogConfig, MachineStatus,
    SignalTelemetry, CONTINUOUS_JOG_PULSES_XY, CONTINUOUS_JOG_PULSES_Z, DEFAULT_Z_JOG_SPEED,
};
use crate::packet::{Opcode, Packet};

pub enum Command {
    Jog { axis: Axis, pulses: i32 },
    JogWithConfig { axis: Axis, pulses: i32, config: JogConfig, auto_stop: bool },
    MoveXY { dx: i32, dy: i32, config: JogConfig },
    StartFifoStream { ticks: Vec<(i8, i8, u8)> },
    StartContinuousJog { axis: Axis, direction: i32 },
    Stop,
    HomeAll(HomeConfig),
    HomeZ,
    UpdateJogConfig { config: JogConfig, z_speed: u32 },
    WriteRegisters { addr: u32, values: Vec<u32> },
    CalibrateHeightSensor,
    SetZMaterialConfig { material_thickness_mm: f64, pulses_per_mm_z: f64 },
    ProbeSurfaceZ,
    SetFollowerServo(bool),
    SetRedPointer(bool),
    LaserPulse { duration_ms: u32, power_pct: f32 },
    SetGas { gas_type: u8, open: bool },
}

/// Thread-safe client managing UDP communications with the TZLASER hardware board
/// (Port 502 for XY motion controller, Port 999 for Z-axis & capacitive height controller).
pub struct LaserController {
    target_addr: SocketAddr,
    status: Arc<Mutex<MachineStatus>>,
    telemetry: Arc<Mutex<SignalTelemetry>>,
    running: Arc<AtomicBool>,
    connected: Arc<AtomicBool>,
    _seq: Arc<AtomicU16>,
    command_tx: std::sync::mpsc::Sender<Command>,
    _worker: Option<JoinHandle<()>>,
}

impl LaserController {
    pub fn new(target_addr: SocketAddr) -> Self {
        let (command_tx, command_rx) = std::sync::mpsc::channel::<Command>();
        let status = Arc::new(Mutex::new(MachineStatus::default()));
        let telemetry = Arc::new(Mutex::new(SignalTelemetry::new()));
        let running = Arc::new(AtomicBool::new(true));
        let connected = Arc::new(AtomicBool::new(false));
        let seq = Arc::new(AtomicU16::new(0));

        let status_clone = Arc::clone(&status);
        let telemetry_clone = Arc::clone(&telemetry);
        let running_clone = Arc::clone(&running);
        let connected_clone = Arc::clone(&connected);
        let seq_clone = Arc::clone(&seq);

        let worker = thread::spawn(move || {
            let socket = match UdpSocket::bind("0.0.0.0:0") {
                Ok(s) => s,
                Err(e) => {
                    log::error!("Failed to bind local UDP socket for port 502: {}", e);
                    return;
                }
            };
            socket
                .set_read_timeout(Some(Duration::from_millis(5)))
                .ok();

            let z_target_addr = SocketAddr::new(target_addr.ip(), 999);
            let z_socket = UdpSocket::bind("0.0.0.0:0").ok();
            if let Some(ref zs) = z_socket {
                zs.set_nonblocking(true).ok();
            }

            let mut buf = [0u8; 1500];
            let mut z_buf = [0u8; 512];
            let mut jog_cfg = JogConfig::default();
            let mut z_jog_speed = DEFAULT_Z_JOG_SPEED;

            let mut board_initialized = false;
            let mut z_status_seen = false;
            let mut latest_z: (i32, u32, u32) = (0, 0, 0);
            let mut pending_stop_p2_at: Option<Instant> = None;
            let mut auto_stop_at: Option<Instant> = None;
            let mut active_step_watch: Option<(Axis, i32, i32)> = None;
            let mut active_z_step_watch: Option<(i32, i32, Instant)> = None;
            let mut active_remote_jog: Option<(Axis, i32)> = None;
            let mut remote_jog_started_at: Option<Instant> = None;
            let mut pending_remote_stop_at: Option<Instant> = None;
            let mut prev_remote_mask: u32 = 0;
            let mut last_r1004: u32 = 0;
            let mut last_r1006: u32 = 8000;
            let mut last_r1022: u32 = 0;
            let mut pending_xy_home_after_z: Option<(HomeConfig, Instant, Instant)> = None;
            let mut pending_z_home_after_cal_stop_at: Option<Instant> = None;
            let mut cal_monitor: Option<CalibrationMonitor> = None;
            let mut active_remote_z_fast: bool = false;
            let mut ui_continuous_z_jog: bool = false;
            struct FifoStreamState {
                ticks: Vec<(i8, i8, u8)>,
                cursor: usize,
                fifo_seq: u32,
                estimated_free: u32,
                started_at: Instant,
                sent_run_flag: bool,
                seen_moving: bool,
                telemetry_cursor: usize,
                last_telemetry_at: Instant,
                last_log_at: Instant,
            }
            let mut active_fifo: Option<FifoStreamState> = None;

            // Helper to send a Port 502 packet and wait briefly for its ACK during initialization
            let send_and_wait_ack = |sock: &UdpSocket, pkt: &Packet, recv_buf: &mut [u8]| -> bool {
                if sock.send_to(&pkt.encode(), target_addr).is_err() {
                    return false;
                }
                let deadline = Instant::now() + Duration::from_millis(25);
                while Instant::now() < deadline && running_clone.load(Ordering::Relaxed) {
                    if let Ok((n, src)) = sock.recv_from(recv_buf) {
                        if src == target_addr {
                            if let Ok(resp) = Packet::decode(&recv_buf[..n]) {
                                if resp.seq == pkt.seq {
                                    return true;
                                }
                            }
                        }
                    }
                }
                false
            };

            // Helper to send a Port 999 Modbus RTU command (0x10) and wait briefly for its 8-byte ACK,
            // retrying once if an in-flight status poll response arrived first on the RS-485 bridge.
            let send_z_cmd_with_ack = |zs: &UdpSocket,
                                       cmd_bytes: &[u8],
                                       recv_buf: &mut [u8],
                                       z_seen: &mut bool,
                                       z_latest: &mut (i32, u32, u32)| {
                for _attempt in 0..2 {
                    let _ = zs.send_to(cmd_bytes, z_target_addr);
                    let deadline = Instant::now() + Duration::from_millis(28);
                    let mut retry_after_poll = false;
                    while Instant::now() < deadline && running_clone.load(Ordering::Relaxed) {
                        if let Ok((n, src)) = zs.recv_from(recv_buf) {
                            if src == z_target_addr {
                                if n >= 8 && recv_buf[0] == 0x01 && recv_buf[1] == 0x10 {
                                    return;
                                }
                                if let Some((pos_z, raw_cap, state_code)) =
                                    decode_z_poll_response(&recv_buf[..n])
                                {
                                    *z_seen = true;
                                    *z_latest = (pos_z, raw_cap, state_code);
                                    retry_after_poll = true;
                                    break;
                                }
                            }
                        } else {
                            thread::sleep(Duration::from_millis(1));
                        }
                    }
                    if !retry_after_poll {
                        break;
                    }
                }
            };

            // Send initial enable sequence on Port 502 and run full SC2000 board initialization
            // right away if Port 502 responds, so `ReadRegisters(1000, 144)` responses in the main loop
            // are never swallowed by `send_and_wait_ack`.
            let mut port502_ack = false;
            for pkt in build_initial_enable_packets(|| seq_clone.fetch_add(1, Ordering::Relaxed)) {
                if !running_clone.load(Ordering::Relaxed) {
                    return;
                }
                if send_and_wait_ack(&socket, &pkt, &mut buf) {
                    port502_ack = true;
                }
            }
            if port502_ack && running_clone.load(Ordering::Relaxed) {
                log::info!(
                    "Controller online at {}; running SC2000 board initialization sequence...",
                    target_addr
                );
                for init_pkt in build_init_packets(|| seq_clone.fetch_add(1, Ordering::Relaxed)) {
                    if !running_clone.load(Ordering::Relaxed) {
                        return;
                    }
                    let _ = send_and_wait_ack(&socket, &init_pkt, &mut buf);
                }
                board_initialized = true;
                connected_clone.store(true, Ordering::Relaxed);
                log::info!("Controller initialization sequence complete.");
            }
            if let Some(ref zs) = z_socket {
                for raw in build_z_handshake_packets() {
                    if !running_clone.load(Ordering::Relaxed) {
                        return;
                    }
                    let _ = zs.send_to(&raw, z_target_addr);
                    let deadline = Instant::now() + Duration::from_millis(22);
                    while Instant::now() < deadline && running_clone.load(Ordering::Relaxed) {
                        if let Ok((n, src)) = zs.recv_from(&mut z_buf) {
                            if src == z_target_addr {
                                if let Some((pos_z, raw_cap, state_code)) =
                                    decode_z_poll_response(&z_buf[..n])
                                {
                                    z_status_seen = true;
                                    latest_z = (pos_z, raw_cap, state_code);
                                }
                                break;
                            }
                        } else {
                            thread::sleep(Duration::from_millis(1));
                        }
                    }
                }
            }

            let mut last_poll = Instant::now() - Duration::from_secs(10);
            let mut last_z_poll = Instant::now();
            let mut last_enable_retry = Instant::now();

            while running_clone.load(Ordering::Relaxed) {
                // 0. Check scheduled stop-phase-2 (REG_COMMAND = 2 sent ~140ms after REG_JOG_RAMP)
                if let Some(t_p2) = pending_stop_p2_at {
                    if Instant::now() >= t_p2 {
                        pending_stop_p2_at = None;
                        let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                        let p2 = Packet::write_request(s, registers::REG_COMMAND, vec![2]);
                        let _ = socket.send_to(&p2.encode(), target_addr);
                    }
                }

                // Auto-retract Z to max height (Z-home) after stopping or aborting calibration
                if let Some(t_zhome) = pending_z_home_after_cal_stop_at {
                    if Instant::now() >= t_zhome {
                        pending_z_home_after_cal_stop_at = None;
                        if let Some(ref zs) = z_socket {
                            let raw_home = build_z_home_packet();
                            send_z_cmd_with_ack(
                                zs,
                                &raw_home,
                                &mut z_buf,
                                &mut z_status_seen,
                                &mut latest_z,
                            );
                            status_clone.lock().unwrap().is_homing_z = true;
                            last_z_poll = Instant::now() - Duration::from_secs(1);
                            log::info!("[CAL] Auto-homing Z to max height after calibration stop/abort");
                        }
                    }
                }

                // Check scheduled delayed remote stop (ensures brief remote button taps move for at least 150ms)
                if let Some(t_rem_stop) = pending_remote_stop_at {
                    if Instant::now() >= t_rem_stop {
                        pending_remote_stop_at = None;
                        let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                        let p1 = Packet::write_request(
                            s,
                            registers::REG_JOG_RAMP,
                            vec![118_000, 1_180_000],
                        );
                        let _ = socket.send_to(&p1.encode(), target_addr);
                        pending_stop_p2_at = Some(Instant::now() + Duration::from_millis(140));
                        telemetry_clone.lock().unwrap().push_command(0.0, 0.0, 0.0, 0.0, false, None);
                    }
                }

                // Check scheduled auto-stop after a finite XY Step jog completes
                if let Some(t_stop) = auto_stop_at {
                    if Instant::now() >= t_stop {
                        auto_stop_at = None;
                        active_step_watch = None;
                        let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                        let p1 = Packet::write_request(
                            s,
                            registers::REG_JOG_RAMP,
                            vec![118_000, 1_180_000],
                        );
                        let _ = socket.send_to(&p1.encode(), target_addr);
                        pending_stop_p2_at = Some(Instant::now() + Duration::from_millis(140));
                        telemetry_clone.lock().unwrap().push_command(0.0, 0.0, 0.0, 0.0, false, None);
                    }
                }

                // Clear expired Z step watch window
                if let Some((_, _, exp)) = active_z_step_watch {
                    if Instant::now() >= exp {
                        active_z_step_watch = None;
                    }
                }

                // 1. Process pending commands from UI
                while let Ok(cmd) = command_rx.try_recv() {
                    match cmd {
                        Command::Jog { axis, pulses } => {
                            if pulses == 0 {
                                continue;
                            }
                            if axis == Axis::Z {
                                if let Some(ref zs) = z_socket {
                                    // For a finite Z step jog, `cmd = 9` on Port 999 takes the exact
                                    // relative pulse target (`hw_pulses = -pulses`) and moves to target.
                                    let eff_z_speed = z_jog_speed.max(120);
                                    let raw = build_z_jog_packet(pulses, eff_z_speed);
                                    send_z_cmd_with_ack(
                                        zs,
                                        &raw,
                                        &mut z_buf,
                                        &mut z_status_seen,
                                        &mut latest_z,
                                    );
                                    active_z_step_watch = Some((
                                        latest_z.0,
                                        pulses.unsigned_abs() as i32,
                                        Instant::now() + Duration::from_millis(2500),
                                    ));
                                    last_z_poll = Instant::now() - Duration::from_secs(1);
                                    let z_rate = (pulses.signum() as f32) * ((z_jog_speed.max(10) as f32) * 100.0);
                                    telemetry_clone.lock().unwrap().push_command(0.0, 0.0, z_rate, 0.0, false, Some("Jog Z"));
                                }
                            } else {
                                if pending_stop_p2_at.take().is_some() {
                                    let s_rst = seq_clone.fetch_add(1, Ordering::Relaxed);
                                    let p2 = Packet::write_request(
                                        s_rst,
                                        registers::REG_COMMAND,
                                        vec![2],
                                    );
                                    let _ = send_and_wait_ack(&socket, &p2, &mut buf);
                                    thread::sleep(Duration::from_millis(25));
                                }
                                if last_r1004 != 0 {
                                    let s_en = seq_clone.fetch_add(1, Ordering::Relaxed);
                                    let en = Packet::write_request(
                                        s_en,
                                        registers::REG_ENABLE_FLAG,
                                        vec![1],
                                    );
                                    let _ = send_and_wait_ack(&socket, &en, &mut buf);
                                    last_r1004 = 0;
                                }
                                let cur_pos = {
                                    let st = status_clone.lock().unwrap();
                                    match axis {
                                        Axis::X => st.pos_x,
                                        Axis::Y => st.pos_y,
                                        Axis::Z => st.pos_z,
                                        Axis::W => st.pos_w,
                                    }
                                };
                                let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                                let pkt = build_jog_packet(s, axis, pulses, &jog_cfg);
                                let _ = socket.send_to(&pkt.encode(), target_addr);
                                let rate = (pulses.signum() as f32) * (jog_cfg.jog_speed as f32);
                                let (xr, yr, wr) = match axis {
                                    Axis::X => (rate, 0.0, 0.0),
                                    Axis::Y => (0.0, rate, 0.0),
                                    Axis::W => (0.0, 0.0, rate),
                                    Axis::Z => (0.0, 0.0, 0.0),
                                };
                                telemetry_clone.lock().unwrap().push_command(xr, yr, 0.0, wr, false, Some("Jog"));

                                active_step_watch =
                                    Some((axis, cur_pos, pulses.unsigned_abs() as i32));
                                let move_secs = (pulses.unsigned_abs() as f64
                                    / jog_cfg.jog_speed.max(1) as f64)
                                    * 2.5
                                    + 1.0;
                                auto_stop_at = Some(
                                    Instant::now()
                                        + Duration::from_secs_f64(move_secs.clamp(0.15, 45.0)),
                                );
                            }
                        }
                        Command::JogWithConfig {
                            axis,
                            pulses,
                            config,
                            auto_stop,
                        } => {
                            if pulses == 0 {
                                continue;
                            }
                            if axis == Axis::Z {
                                if let Some(ref zs) = z_socket {
                                    let custom_z_speed = ((config.jog_speed as f64) / 200.0)
                                        .round()
                                        .clamp(1.0, 500.0)
                                        as u32;
                                    let raw = build_z_jog_packet(pulses, custom_z_speed);
                                    send_z_cmd_with_ack(
                                        zs,
                                        &raw,
                                        &mut z_buf,
                                        &mut z_status_seen,
                                        &mut latest_z,
                                    );
                                    let z_timeout_secs = ((pulses.unsigned_abs() as f64)
                                        / config.jog_speed.max(1) as f64
                                        + 1.0)
                                        .clamp(2.5, 120.0);
                                    active_z_step_watch = Some((
                                        latest_z.0,
                                        pulses.unsigned_abs() as i32,
                                        Instant::now() + Duration::from_secs_f64(z_timeout_secs),
                                    ));
                                    last_z_poll = Instant::now() - Duration::from_secs(1);
                                    let z_rate = (pulses.signum() as f32) * ((z_jog_speed.max(10) as f32) * 100.0);
                                    telemetry_clone.lock().unwrap().push_command(0.0, 0.0, z_rate, 0.0, false, Some("Jog Z"));
                                }
                                continue;
                            }
                            if pending_stop_p2_at.take().is_some() {
                                let s_rst = seq_clone.fetch_add(1, Ordering::Relaxed);
                                let p2 = Packet::write_request(
                                    s_rst,
                                    registers::REG_COMMAND,
                                    vec![2],
                                );
                                let _ = send_and_wait_ack(&socket, &p2, &mut buf);
                                thread::sleep(Duration::from_millis(25));
                            }
                            if last_r1004 != 0 {
                                let s_en = seq_clone.fetch_add(1, Ordering::Relaxed);
                                let en = Packet::write_request(
                                    s_en,
                                    registers::REG_ENABLE_FLAG,
                                    vec![1],
                                );
                                let _ = send_and_wait_ack(&socket, &en, &mut buf);
                                last_r1004 = 0;
                            }
                            let cur_pos = {
                                let st = status_clone.lock().unwrap();
                                match axis {
                                    Axis::X => st.pos_x,
                                    Axis::Y => st.pos_y,
                                    Axis::Z => st.pos_z,
                                    Axis::W => st.pos_w,
                                }
                            };
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let pkt = build_jog_packet(s, axis, pulses, &config);
                            let _ = socket.send_to(&pkt.encode(), target_addr);
                            let rate = (pulses.signum() as f32) * (config.jog_speed as f32);
                            let (xr, yr, wr) = match axis {
                                Axis::X => (rate, 0.0, 0.0),
                                Axis::Y => (0.0, rate, 0.0),
                                Axis::W => (0.0, 0.0, rate),
                                Axis::Z => (0.0, 0.0, 0.0),
                            };
                            telemetry_clone.lock().unwrap().push_command(xr, yr, 0.0, wr, false, Some("JogCfg"));

                            active_step_watch =
                                Some((axis, cur_pos, pulses.unsigned_abs() as i32));
                            if auto_stop {
                                let move_secs = (pulses.unsigned_abs() as f64
                                    / config.jog_speed.max(1) as f64)
                                    * 2.5
                                    + 1.5;
                                auto_stop_at = Some(
                                    Instant::now()
                                        + Duration::from_secs_f64(move_secs.max(0.20)),
                                );
                            } else {
                                auto_stop_at = None;
                            }
                        }
                        Command::MoveXY { dx, dy, config } => {
                            if dx == 0 && dy == 0 {
                                continue;
                            }
                            pending_xy_home_after_z = None;
                            auto_stop_at = None;
                            active_step_watch = None;
                            if pending_stop_p2_at.take().is_some() {
                                let s_rst = seq_clone.fetch_add(1, Ordering::Relaxed);
                                let p2 = Packet::write_request(
                                    s_rst,
                                    registers::REG_COMMAND,
                                    vec![2],
                                );
                                let _ = send_and_wait_ack(&socket, &p2, &mut buf);
                                thread::sleep(Duration::from_millis(25));
                            }
                            if last_r1004 != 0 {
                                let s_en = seq_clone.fetch_add(1, Ordering::Relaxed);
                                let en = Packet::write_request(
                                    s_en,
                                    registers::REG_ENABLE_FLAG,
                                    vec![1],
                                );
                                let _ = send_and_wait_ack(&socket, &en, &mut buf);
                                last_r1004 = 0;
                            }
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let pkt = build_xy_move_packet(s, dx, dy, &config);
                            let _ = socket.send_to(&pkt.encode(), target_addr);
                            let mag = ((dx as f64).hypot(dy as f64)).max(1.0);
                            let vx = ((dx as f64 / mag) * (config.jog_speed as f64)) as f32;
                            let vy = ((dy as f64 / mag) * (config.jog_speed as f64)) as f32;
                            telemetry_clone.lock().unwrap().push_command(vx, vy, 0.0, 0.0, false, Some("MoveXY"));
                        }
                        Command::StartFifoStream { ticks } => {
                            if ticks.is_empty() {
                                continue;
                            }
                            pending_xy_home_after_z = None;
                            auto_stop_at = None;
                            active_step_watch = None;
                            active_remote_jog = None;
                            if pending_stop_p2_at.take().is_some() {
                                let s_rst = seq_clone.fetch_add(1, Ordering::Relaxed);
                                let p2 = Packet::write_request(
                                    s_rst,
                                    registers::REG_COMMAND,
                                    vec![2],
                                );
                                let _ = send_and_wait_ack(&socket, &p2, &mut buf);
                                thread::sleep(Duration::from_millis(25));
                            }
                            let (px, py) = {
                                let st = status_clone.lock().unwrap();
                                (st.pos_x, st.pos_y)
                            };
                            let mut start_acks = Vec::new();
                            for spkt in build_fifo_start_packets(
                                || seq_clone.fetch_add(1, Ordering::Relaxed),
                                px,
                                py,
                            ) {
                                start_acks.push(send_and_wait_ack(&socket, &spkt, &mut buf));
                            }
                            eprintln!(
                                "[FIFO START] px={} py={} r1022={} acks={:?}",
                                px, py, last_r1022, start_acks
                            );
                            let now = Instant::now();
                            active_fifo = Some(FifoStreamState {
                                ticks,
                                cursor: 0,
                                fifo_seq: last_r1022.wrapping_add(1),
                                estimated_free: 8000,
                                started_at: now,
                                sent_run_flag: false,
                                seen_moving: false,
                                telemetry_cursor: 0,
                                last_telemetry_at: now,
                                last_log_at: now,
                            });
                        }
                        Command::StartContinuousJog { axis, direction } => {
                            let sign = if direction >= 0 { 1 } else { -1 };
                            if axis == Axis::Z {
                                if let Some(ref zs) = z_socket {
                                    ui_continuous_z_jog = true;
                                    let eff_z_speed = z_jog_speed.max(120);
                                    let raw = build_z_jog_packet(
                                        sign * CONTINUOUS_JOG_PULSES_Z,
                                        eff_z_speed,
                                    );
                                    send_z_cmd_with_ack(
                                        zs,
                                        &raw,
                                        &mut z_buf,
                                        &mut z_status_seen,
                                        &mut latest_z,
                                    );
                                    last_z_poll = Instant::now() - Duration::from_secs(1);
                                    let z_rate = (sign.signum() as f32) * ((z_jog_speed.max(10) as f32) * 100.0);
                                    telemetry_clone.lock().unwrap().push_command(0.0, 0.0, z_rate, 0.0, false, Some("Jog Z"));
                                }
                            } else {
                                auto_stop_at = None;
                                active_step_watch = None;
                                if pending_stop_p2_at.take().is_some() {
                                    let s_rst = seq_clone.fetch_add(1, Ordering::Relaxed);
                                    let p2 = Packet::write_request(
                                        s_rst,
                                        registers::REG_COMMAND,
                                        vec![2],
                                    );
                                    let _ = send_and_wait_ack(&socket, &p2, &mut buf);
                                    thread::sleep(Duration::from_millis(25));
                                }
                                if last_r1004 != 0 {
                                    let s_en = seq_clone.fetch_add(1, Ordering::Relaxed);
                                    let en = Packet::write_request(
                                        s_en,
                                        registers::REG_ENABLE_FLAG,
                                        vec![1],
                                    );
                                    let _ = send_and_wait_ack(&socket, &en, &mut buf);
                                    last_r1004 = 0;
                                }
                                let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                                let pkt = build_jog_packet(
                                    s,
                                    axis,
                                    sign * CONTINUOUS_JOG_PULSES_XY,
                                    &jog_cfg,
                                );
                                let _ = socket.send_to(&pkt.encode(), target_addr);
                                let rate = (sign as f32) * (jog_cfg.jog_speed as f32);
                                let (xr, yr, wr) = match axis {
                                    Axis::X => (rate, 0.0, 0.0),
                                    Axis::Y => (0.0, rate, 0.0),
                                    Axis::W => (0.0, 0.0, rate),
                                    Axis::Z => (0.0, 0.0, 0.0),
                                };
                                telemetry_clone.lock().unwrap().push_command(xr, yr, 0.0, wr, false, Some("ContJog"));
                            }
                        }
                        Command::Stop => {
                            auto_stop_at = None;
                            active_step_watch = None;
                            active_z_step_watch = None;
                            active_remote_jog = None;
                            active_remote_z_fast = false;
                            ui_continuous_z_jog = false;
                            pending_xy_home_after_z = None;
                            telemetry_clone.lock().unwrap().push_command(0.0, 0.0, 0.0, 0.0, false, Some("Stop"));
                            if active_fifo.take().is_some() {
                                for fpkt in build_fifo_finish_packets(|| {
                                    seq_clone.fetch_add(1, Ordering::Relaxed)
                                }) {
                                    let _ = send_and_wait_ack(&socket, &fpkt, &mut buf);
                                }
                            }
                            let (p1, _p2) = build_stop_packets(|| {
                                seq_clone.fetch_add(1, Ordering::Relaxed)
                            });
                            let _ = socket.send_to(&p1.encode(), target_addr);
                            pending_stop_p2_at = Some(Instant::now() + Duration::from_millis(140));

                            if let Some(ref zs) = z_socket {
                                let was_calibrating = {
                                    let st = status_clone.lock().unwrap();
                                    st.is_calibrating_z || st.z_state_code == 7 || cal_monitor.is_some()
                                };
                                cal_monitor = None;
                                let raw_stop = build_z_stop_packet();
                                send_z_cmd_with_ack(
                                    zs,
                                    &raw_stop,
                                    &mut z_buf,
                                    &mut z_status_seen,
                                    &mut latest_z,
                                );
                                let at_upper = {
                                    let mut st = status_clone.lock().unwrap();
                                    st.is_calibrating_z = false;
                                    st.z_pos_sensor
                                };
                                if was_calibrating && !at_upper {
                                    pending_z_home_after_cal_stop_at =
                                        Some(Instant::now() + Duration::from_millis(80));
                                } else {
                                    pending_z_home_after_cal_stop_at = None;
                                }
                                last_z_poll = Instant::now() - Duration::from_secs(1);
                            }
                        }
                        Command::HomeAll(home_cfg) => {
                            auto_stop_at = None;
                            active_step_watch = None;
                            active_z_step_watch = None;
                            active_remote_jog = None;
                            if active_fifo.take().is_some() {
                                for fpkt in build_fifo_finish_packets(|| {
                                    seq_clone.fetch_add(1, Ordering::Relaxed)
                                }) {
                                    let _ = send_and_wait_ack(&socket, &fpkt, &mut buf);
                                }
                            }

                            // 1. Home Z up first so the nozzle clears the sheet before XY moves.
                            //    XY homing will be triggered once Z homing completes (matching SC2000).
                            if let Some(ref zs) = z_socket {
                                let raw_z_home = build_z_home_packet();
                                send_z_cmd_with_ack(
                                    zs,
                                    &raw_z_home,
                                    &mut z_buf,
                                    &mut z_status_seen,
                                    &mut latest_z,
                                );
                                {
                                    let mut st = status_clone.lock().unwrap();
                                    st.is_homing_z = true;
                                }
                                last_z_poll = Instant::now() - Duration::from_secs(1);
                                pending_xy_home_after_z = Some((
                                    home_cfg,
                                    Instant::now(),
                                    Instant::now() + Duration::from_secs(15),
                                ));
                            } else {
                                pending_stop_p2_at = None;
                                for (addr, vals) in [
                                    (registers::REG_COMMAND, vec![2]),
                                    (1344, vec![0]),
                                    (registers::REG_ENABLE_FLAG, vec![1]),
                                    (registers::REG_COMMAND, vec![3]),
                                ] {
                                    let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                                    let p = Packet::write_request(s, addr, vals);
                                    let _ = send_and_wait_ack(&socket, &p, &mut buf);
                                }
                                last_r1004 = 0;
                                for hpkt in build_xy_home_packets(
                                    || seq_clone.fetch_add(1, Ordering::Relaxed),
                                    home_cfg,
                                ) {
                                    let _ = send_and_wait_ack(&socket, &hpkt, &mut buf);
                                }
                            }
                        }
                        Command::HomeZ => {
                            active_z_step_watch = None;
                            pending_xy_home_after_z = None;
                            if let Some(ref zs) = z_socket {
                                let raw_z_home = build_z_home_packet();
                                send_z_cmd_with_ack(
                                    zs,
                                    &raw_z_home,
                                    &mut z_buf,
                                    &mut z_status_seen,
                                    &mut latest_z,
                                );
                                last_z_poll = Instant::now() - Duration::from_secs(1);
                            }
                        }
                        Command::UpdateJogConfig { config, z_speed } => {
                            jog_cfg = config;
                            z_jog_speed = z_speed.clamp(1, 500);
                        }
                        Command::WriteRegisters { addr, values } => {
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let pkt = Packet::write_request(s, addr, values);
                            let _ = socket.send_to(&pkt.encode(), target_addr);
                        }
                        Command::CalibrateHeightSensor => {
                            log::info!("Initiating capacitive height sensor calibration routine (Port 999 cmd=7)");
                            pending_z_home_after_cal_stop_at = None;
                            if let Some(ref zs) = z_socket {
                                let raw_cal = build_z_calibrate_packet();
                                send_z_cmd_with_ack(
                                    zs,
                                    &raw_cal,
                                    &mut z_buf,
                                    &mut z_status_seen,
                                    &mut latest_z,
                                );
                                log::info!("[CAL] Port 999 cmd=7 sent");
                                cal_monitor = Some(CalibrationMonitor::new(Instant::now()));
                                {
                                    let mut st = status_clone.lock().unwrap();
                                    st.is_calibrating_z = true;
                                    st.z_state_code = 7;
                                    st.z_cal_error = None;
                                    st.z_calibrated_thickness_mm = st.z_material_thickness_mm;
                                }
                                last_z_poll = Instant::now() - Duration::from_secs(1);
                            } else {
                                log::warn!("[CAL] Cannot calibrate height sensor: Port 999 Z socket is not bound");
                            }
                        }
                        Command::SetZMaterialConfig {
                            material_thickness_mm,
                            pulses_per_mm_z,
                        } => {
                            let mut st = status_clone.lock().unwrap();
                            st.z_material_thickness_mm = material_thickness_mm.max(0.0);
                            st.z_pulses_per_mm = pulses_per_mm_z.max(1.0);
                        }
                        Command::ProbeSurfaceZ => {
                            log::info!("Probing sheet surface Z=0");
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let pkt = Packet::write_request(s, 2102, vec![1]);
                            let _ = socket.send_to(&pkt.encode(), target_addr);
                        }
                        Command::SetFollowerServo(enable) => {
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let pkt = Packet::write_request(s, 2104, vec![if enable { 1 } else { 0 }]);
                            let _ = socket.send_to(&pkt.encode(), target_addr);
                        }
                        Command::SetRedPointer(enable) => {
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let pkt = Packet::write_request(s, 2050, vec![if enable { 1 } else { 0 }]);
                            let _ = socket.send_to(&pkt.encode(), target_addr);
                        }
                        Command::LaserPulse { duration_ms, power_pct } => {
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let power_val = ((power_pct.clamp(0.0, 100.0) / 100.0) * 1000.0) as u32;
                            let pkt = Packet::write_request(s, 2060, vec![duration_ms, power_val]);
                            let _ = socket.send_to(&pkt.encode(), target_addr);
                        }
                        Command::SetGas { gas_type, open } => {
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let val = if open { gas_type as u32 } else { 0 };
                            let pkt = Packet::write_request(s, 2070, vec![val]);
                            let _ = socket.send_to(&pkt.encode(), target_addr);
                        }
                    }
                }

                // 2. If not yet connected, periodically re-send initial enable packets
                if !connected_clone.load(Ordering::Relaxed)
                    && last_enable_retry.elapsed() >= Duration::from_millis(800)
                {
                    last_enable_retry = Instant::now();
                    for pkt in
                        build_initial_enable_packets(|| seq_clone.fetch_add(1, Ordering::Relaxed))
                    {
                        let _ = socket.send_to(&pkt.encode(), target_addr);
                    }
                    if let Some(ref zs) = z_socket {
                        for raw in build_z_handshake_packets() {
                            let _ = zs.send_to(&raw, z_target_addr);
                        }
                    }
                }

                // 3. Poll Port 502 machine status every 30ms (matching MCCore=30 in SC2000 ipAdd.ini)
                if last_poll.elapsed() >= Duration::from_millis(30) {
                    last_poll = Instant::now();
                    let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                    let poll_pkt = Packet::read_request(s, registers::REG_STATUS_BASE, 144);
                    let _ = socket.send_to(&poll_pkt.encode(), target_addr);
                }

                // 4. Poll Port 999 Z-axis & height controller status (50ms during calibration, 80ms during Z motion, 120ms homing, 300ms idle)
                let (cal_active, z_busy) = {
                    let st = status_clone.lock().unwrap();
                    (
                        st.is_calibrating_z || cal_monitor.is_some(),
                        st.is_homing_z || st.z_state_code != 0,
                    )
                };
                let z_poll_interval = if cal_active {
                    Duration::from_millis(50)
                } else if active_z_step_watch.is_some()
                    || pending_xy_home_after_z.is_some()
                    || matches!(active_remote_jog, Some((Axis::Z, _)))
                {
                    Duration::from_millis(80)
                } else if z_busy {
                    Duration::from_millis(120)
                } else {
                    Duration::from_millis(300)
                };
                if last_z_poll.elapsed() >= z_poll_interval {
                    last_z_poll = Instant::now();
                    if let Some(ref zs) = z_socket {
                        let _ = zs.send_to(&build_z_poll_packet(), z_target_addr);
                    }
                }

                // 5. Receive all queued incoming responses from Port 502
                while let Ok((n, src)) = socket.recv_from(&mut buf) {
                    if src != target_addr {
                        continue;
                    }
                    if let Ok(pkt) = Packet::decode(&buf[..n]) {
                        connected_clone.store(true, Ordering::Relaxed);

                        // Re-run board init if never initialized or if R1004 reports uninitialized state (0x00880000)
                        let r1004_uninit = pkt.opcode == Opcode::ReadRegisters
                            && pkt.addr == registers::REG_STATUS_BASE
                            && pkt.data.get(4).copied().unwrap_or(0) == 0x0088_0000;
                        if !board_initialized || r1004_uninit {
                            board_initialized = true;
                            log::info!(
                                "Controller online at {}; running SC2000 board initialization sequence...",
                                target_addr
                            );
                            for init_pkt in build_init_packets(|| {
                                seq_clone.fetch_add(1, Ordering::Relaxed)
                            }) {
                                if !running_clone.load(Ordering::Relaxed) {
                                    break;
                                }
                                let _ = send_and_wait_ack(&socket, &init_pkt, &mut buf);
                            }
                            // Request a fresh 144-register status poll immediately after initialization
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let poll_pkt =
                                Packet::read_request(s, registers::REG_STATUS_BASE, 144);
                            let _ = socket.send_to(&poll_pkt.encode(), target_addr);
                            last_poll = Instant::now();
                            log::info!("Controller initialization sequence complete.");
                        }

                        if pkt.opcode == Opcode::ReadRegisters
                            && pkt.addr == registers::REG_STATUS_BASE
                            && pkt.data.len() >= 24
                        {
                            last_r1004 = pkt.data.get(4).copied().unwrap_or(0);
                            last_r1006 = pkt.data.get(6).copied().unwrap_or(last_r1006);
                            last_r1022 = pkt.data.get(22).copied().unwrap_or(last_r1022);
                            if let Some(ref mut fifo) = active_fifo {
                                fifo.estimated_free = last_r1006;
                                if pkt.data.get(7).copied().unwrap_or(0) != 0 {
                                    fifo.seen_moving = true;
                                }
                                if fifo.last_log_at.elapsed() >= Duration::from_millis(250) {
                                    fifo.last_log_at = Instant::now();
                                    let r8 = pkt.data.get(8).copied().unwrap_or(0) as i32;
                                    let r9 = pkt.data.get(9).copied().unwrap_or(0) as i32;
                                    eprintln!(
                                        "[FIFO 250ms] t={}ms cur={} free={} r1008={} r1009={}",
                                        fifo.started_at.elapsed().as_millis(),
                                        fifo.cursor,
                                        last_r1006,
                                        r8,
                                        r9,
                                    );
                                }
                            }
                            // Ensure hardware soft limits R1063..R1070 (addr = 1252) remain unbounded
                            // ([0x7FFFFFFF, 0x80000000, ...]) so ±472,000 continuous & remote jogs are never blocked.
                            if pkt.data.get(5).copied().unwrap_or(0) != 1
                                && pkt.data.len() > 68
                                && (pkt.data[63] != 0x7FFF_FFFF
                                    || pkt.data[64] != 0x8000_0000
                                    || pkt.data[65] != 0x7FFF_FFFF
                                    || pkt.data[66] != 0x8000_0000)
                            {
                                let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                                let sl_pkt = Packet::write_request(
                                    s,
                                    1252,
                                    vec![
                                        0x7FFF_FFFF,
                                        0x8000_0000,
                                        0x7FFF_FFFF,
                                        0x8000_0000,
                                        0x7FFF_FFFF,
                                        0x8000_0000,
                                        0x7FFF_FFFF,
                                        0x8000_0000,
                                    ],
                                );
                                let _ = socket.send_to(&sl_pkt.encode(), target_addr);
                            }

                            let mut new_st = MachineStatus::from_status_registers(&pkt.data);
                            if z_status_seen {
                                new_st.apply_z_status(latest_z.0, latest_z.1, latest_z.2);
                            }

                            // 5A. Wireless Remote Control (`R1042` / `R1043`) Button & Continuous Jog Dispatch
                            // The wireless remote is paired to the controller board, which exposes the
                            // pressed button bitmask in `R1042` and `R1043`. The PC software watches this
                            // bitmask and sends the corresponding X/Y (Port 502) or Z (Port 999) jog & stop packets.
                            let rising_mask = new_st.remote_button_mask & !prev_remote_mask;
                            prev_remote_mask = new_st.remote_button_mask;

                            if (rising_mask & remote_buttons::ANY_STOP) != 0 {
                                auto_stop_at = None;
                                active_step_watch = None;
                                active_z_step_watch = None;
                                active_remote_jog = None;
                                pending_remote_stop_at = None;
                                if active_fifo.take().is_some() {
                                    for fpkt in build_fifo_finish_packets(|| {
                                        seq_clone.fetch_add(1, Ordering::Relaxed)
                                    }) {
                                        let _ = send_and_wait_ack(&socket, &fpkt, &mut buf);
                                    }
                                }
                                let (p1, _p2) = build_stop_packets(|| {
                                    seq_clone.fetch_add(1, Ordering::Relaxed)
                                });
                                let _ = socket.send_to(&p1.encode(), target_addr);
                                pending_stop_p2_at =
                                    Some(Instant::now() + Duration::from_millis(140));
                                if let Some(ref zs) = z_socket {
                                    let was_calibrating = {
                                        let st = status_clone.lock().unwrap();
                                        st.is_calibrating_z || st.z_state_code == 7 || cal_monitor.is_some()
                                    };
                                    cal_monitor = None;
                                    let raw_stop = build_z_stop_packet();
                                    send_z_cmd_with_ack(
                                        zs,
                                        &raw_stop,
                                        &mut z_buf,
                                        &mut z_status_seen,
                                        &mut latest_z,
                                    );
                                    let at_upper = {
                                        let mut st = status_clone.lock().unwrap();
                                        st.is_calibrating_z = false;
                                        st.z_pos_sensor
                                    };
                                    if was_calibrating && !at_upper {
                                        pending_z_home_after_cal_stop_at =
                                            Some(Instant::now() + Duration::from_millis(80));
                                    } else {
                                        pending_z_home_after_cal_stop_at = None;
                                    }
                                    last_z_poll = Instant::now() - Duration::from_secs(1);
                                    telemetry_clone.lock().unwrap().push_command(0.0, 0.0, 0.0, 0.0, false, Some("Stop Z"));
                                }
                            } else if (rising_mask & remote_buttons::GUIDE) != 0 {
                                let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                                let next_red = if new_st.red_pointer_on { 0 } else { 1 };
                                let pkt_red = Packet::write_request(s, 2050, vec![next_red]);
                                let _ = socket.send_to(&pkt_red.encode(), target_addr);
                            }

                            // If Stop button is currently held down, do not start or continue any remote jog.
                            let is_stop_active = (new_st.remote_button_mask & remote_buttons::ANY_STOP) != 0;
                            let remote_jog = if is_stop_active {
                                None
                            } else {
                                decode_remote_jog(new_st.remote_button_mask)
                            };
                            let is_fast = (new_st.remote_button_mask
                                & remote_buttons::FAST_FLAG)
                                != 0;
                            let z_fast_changed = matches!(remote_jog, Some((Axis::Z, _)))
                                && is_fast != active_remote_z_fast;
                            if remote_jog != active_remote_jog || z_fast_changed {
                                // Stop previous remote jog if one was active
                                if let Some((prev_axis, _)) = active_remote_jog.take() {
                                    if prev_axis == Axis::Z {
                                        active_remote_z_fast = false;
                                        if !z_fast_changed {
                                            if let Some(ref zs) = z_socket {
                                                let raw_stop = build_z_stop_packet();
                                                send_z_cmd_with_ack(
                                                    zs,
                                                    &raw_stop,
                                                    &mut z_buf,
                                                    &mut z_status_seen,
                                                    &mut latest_z,
                                                );
                                                last_z_poll = Instant::now() - Duration::from_secs(1);
                                            }
                                        }
                                    } else {
                                        // If the user tapped a remote XY button very briefly (< 150ms) and released to Idle,
                                        // schedule the stop ramp at started_at + 150ms so short taps still move a clean step.
                                        let min_tap_window = Duration::from_millis(150);
                                        if remote_jog.is_none()
                                            && remote_jog_started_at
                                                .map_or(false, |t0| t0.elapsed() < min_tap_window)
                                        {
                                            pending_remote_stop_at = Some(
                                                remote_jog_started_at.unwrap() + min_tap_window,
                                            );
                                        } else {
                                            pending_remote_stop_at = None;
                                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                                            let p1 = Packet::write_request(
                                                s,
                                                registers::REG_JOG_RAMP,
                                                vec![118_000, 1_180_000],
                                            );
                                            let _ = socket.send_to(&p1.encode(), target_addr);
                                            pending_stop_p2_at =
                                                Some(Instant::now() + Duration::from_millis(140));
                                        }
                                    }
                                }

                                // Start new remote jog if a directional button is now held
                                if let Some((axis, dir)) = remote_jog {
                                    active_remote_jog = Some((axis, dir));
                                    remote_jog_started_at = Some(Instant::now());
                                    pending_remote_stop_at = None;
                                    if axis == Axis::Z {
                                        active_remote_z_fast = is_fast;
                                        if let Some(ref zs) = z_socket {
                                            let normal_z = z_jog_speed.max(120);
                                            let eff_z_speed = if is_fast {
                                                normal_z.saturating_mul(3).clamp(280, 450)
                                            } else {
                                                normal_z
                                            };
                                            let raw = build_z_jog_packet(
                                                dir * CONTINUOUS_JOG_PULSES_Z,
                                                eff_z_speed,
                                            );
                                            send_z_cmd_with_ack(
                                                zs,
                                                &raw,
                                                &mut z_buf,
                                                &mut z_status_seen,
                                                &mut latest_z,
                                            );
                                            last_z_poll = Instant::now() - Duration::from_secs(1);
                                        }
                                    } else {
                                        auto_stop_at = None;
                                        active_step_watch = None;
                                        if pending_stop_p2_at.take().is_some() {
                                            let s_rst =
                                                seq_clone.fetch_add(1, Ordering::Relaxed);
                                            let p2 = Packet::write_request(
                                                s_rst,
                                                registers::REG_COMMAND,
                                                vec![2],
                                            );
                                            let _ = send_and_wait_ack(&socket, &p2, &mut buf);
                                            thread::sleep(Duration::from_millis(25));
                                        }
                                        if last_r1004 != 0 {
                                            let s_en = seq_clone.fetch_add(1, Ordering::Relaxed);
                                            let en = Packet::write_request(
                                                s_en,
                                                registers::REG_ENABLE_FLAG,
                                                vec![1],
                                            );
                                            let _ = send_and_wait_ack(&socket, &en, &mut buf);
                                            last_r1004 = 0;
                                        }
                                        // Matching SC2000 Frame 957 ([5900, 5900, 59000]), wireless remote XY jogs
                                        // start directly at jog_speed (min_speed == jog_speed) so there is no lag,
                                        // and scale up when the remote's Fast flag (0x0200_0000) is active.
                                        let is_fast = (new_st.remote_button_mask
                                            & remote_buttons::FAST_FLAG)
                                            != 0;
                                        let base_speed = jog_cfg.jog_speed.max(3000);
                                        let rem_speed = if is_fast {
                                            base_speed.saturating_mul(2)
                                        } else {
                                            base_speed
                                        };
                                        let rem_accel = if is_fast {
                                            jog_cfg.accel.max(30000).saturating_mul(2)
                                        } else {
                                            jog_cfg.accel.max(30000)
                                        };
                                        let rem_jog_cfg = JogConfig {
                                            min_speed: rem_speed,
                                            jog_speed: rem_speed,
                                            accel: rem_accel,
                                        };
                                        let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                                        let pkt = build_jog_packet(
                                            s,
                                            axis,
                                            dir * CONTINUOUS_JOG_PULSES_XY,
                                            &rem_jog_cfg,
                                        );
                                        let _ = socket.send_to(&pkt.encode(), target_addr);
                                    }
                                }
                            }

                            // 5B. Step-mode position delta watch
                            if let Some((watch_axis, start_pos, target_delta)) = active_step_watch
                            {
                                let cur_pos = match watch_axis {
                                    Axis::X => new_st.pos_x,
                                    Axis::Y => new_st.pos_y,
                                    Axis::Z => new_st.pos_z,
                                    Axis::W => new_st.pos_w,
                                };
                                if (cur_pos - start_pos).abs() >= target_delta {
                                    active_step_watch = None;
                                    auto_stop_at = None;
                                    let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                                    let p1 = Packet::write_request(
                                        s,
                                        registers::REG_JOG_RAMP,
                                        vec![118_000, 1_180_000],
                                    );
                                    let _ = socket.send_to(&p1.encode(), target_addr);
                                    pending_stop_p2_at =
                                        Some(Instant::now() + Duration::from_millis(140));
                                }
                            }
                            let mut st = status_clone.lock().unwrap();
                            let prev_pos_z = st.pos_z;
                            let prev_cap_height = st.capacitive_height_mm;
                            let prev_cap_pf = st.capacitance_pf;
                            let prev_raw_cap = st.raw_capacitance;
                            let prev_z_state = st.z_state_code;
                            let prev_z_vel = st.z_velocity;
                            let prev_cal_z = st.is_calibrating_z;
                            let prev_follower = st.follower_active;
                            let prev_z_homed = st.z_homed;
                            let prev_homing_z = st.is_homing_z;
                            let prev_z_pos_sensor = st.z_pos_sensor;
                            let prev_z_neg_sensor = st.z_neg_sensor;
                            let prev_z_cal_error = st.z_cal_error.take();
                            let prev_z_touchdown = st.z_touchdown_pos_z;
                            let prev_z_cal_thick = st.z_calibrated_thickness_mm;
                            let prev_z_mat_thick = st.z_material_thickness_mm;
                            let prev_z_ppm = st.z_pulses_per_mm;
                            *st = new_st;
                            st.pos_z = prev_pos_z;
                            st.capacitive_height_mm = prev_cap_height;
                            st.capacitance_pf = prev_cap_pf;
                            st.raw_capacitance = prev_raw_cap;
                            st.z_state_code = prev_z_state;
                            st.z_velocity = prev_z_vel;
                            st.is_calibrating_z = prev_cal_z;
                            st.follower_active = prev_follower;
                            st.z_homed = prev_z_homed;
                            st.is_homing_z = prev_homing_z;
                            st.z_pos_sensor = prev_z_pos_sensor;
                            st.z_neg_sensor = prev_z_neg_sensor;
                            st.z_cal_error = prev_z_cal_error;
                            st.z_touchdown_pos_z = prev_z_touchdown;
                            st.z_calibrated_thickness_mm = prev_z_cal_thick;
                            st.z_material_thickness_mm = prev_z_mat_thick;
                            st.z_pulses_per_mm = prev_z_ppm;
                        }
                    }
                }

                // 6. Receive incoming responses from Port 999 (Z-axis / Capacitive Follower)
                if let Some(ref zs) = z_socket {
                    let mut cal_failure: Option<String> = None;
                    let mut mat_line_stop: Option<String> = None;
                    let mut orphaned_z_jog = false;
                    while let Ok((n, src)) = zs.recv_from(&mut z_buf) {
                        if src == z_target_addr {
                            if let Some(zt) = decode_z_poll_telemetry(&z_buf[..n]) {
                                z_status_seen = true;
                                latest_z = (zt.pos_z, zt.raw_cap, zt.state_code);
                                let clearance_mm = {
                                    let mut st = status_clone.lock().unwrap();
                                    if board_initialized {
                                        connected_clone.store(true, Ordering::Relaxed);
                                        st.is_connected = true;
                                    }
                                    st.apply_z_telemetry(zt);
                                    st.z_clearance_above_material_mm()
                                };
                                if zt.state_code == 9
                                    && active_z_step_watch.is_none()
                                    && !ui_continuous_z_jog
                                    && !matches!(active_remote_jog, Some((Axis::Z, _)))
                                {
                                    orphaned_z_jog = true;
                                }
                                if let Some(ref mut mon) = cal_monitor {
                                    if let Err(reason) = mon.check(&zt, Instant::now()) {
                                        cal_failure = Some(reason);
                                        break;
                                    } else if zt.z_velocity > 0
                                        && clearance_mm.map_or(false, |c| c < -0.35)
                                    {
                                        cal_failure = Some(format!(
                                            "Head went below material line ({:.3} mm) during calibration — pushing against sheet",
                                            clearance_mm.unwrap_or(0.0)
                                        ));
                                        break;
                                    } else if mon.is_finished(&zt) {
                                        if let Some(td_z) = mon.touchdown_pos_z() {
                                            let mut st = status_clone.lock().unwrap();
                                            st.z_touchdown_pos_z = Some(td_z);
                                            log::info!(
                                                "[CAL] Calibration complete: touchdown_pos_z={} pls ({:.3} mm from home, sheet_thickness={:.2} mm)",
                                                td_z,
                                                st.z_sheet_surface_mm_from_home().unwrap_or(0.0),
                                                st.z_calibrated_thickness_mm
                                            );
                                        }
                                        cal_monitor = None;
                                    }
                                } else if zt.z_velocity > 0
                                    && zt.state_code != 7
                                    && clearance_mm.map_or(false, |c| c < -0.10)
                                {
                                    mat_line_stop = Some(format!(
                                        "Head went below material line ({:.3} mm) — stopped to prevent pushing against sheet",
                                        clearance_mm.unwrap_or(0.0)
                                    ));
                                    break;
                                }
                            }
                        }
                    }
                    if let Some(reason) = cal_failure {
                        log::error!("[CAL] Calibration failure detected: {} -> stopping & homing Z", reason);
                        cal_monitor = None;
                        let raw_stop = build_z_stop_packet();
                        send_z_cmd_with_ack(
                            zs,
                            &raw_stop,
                            &mut z_buf,
                            &mut z_status_seen,
                            &mut latest_z,
                        );
                        {
                            let mut st = status_clone.lock().unwrap();
                            st.is_calibrating_z = false;
                            st.z_cal_error = Some(reason);
                            if !st.z_pos_sensor {
                                pending_z_home_after_cal_stop_at =
                                    Some(Instant::now() + Duration::from_millis(80));
                            }
                        }
                        last_z_poll = Instant::now() - Duration::from_secs(1);
                    } else if let Some(reason) = mat_line_stop {
                        log::warn!("[Z-GUARD] {}", reason);
                        active_z_step_watch = None;
                        active_remote_jog = None;
                        let raw_stop = build_z_stop_packet();
                        send_z_cmd_with_ack(
                            zs,
                            &raw_stop,
                            &mut z_buf,
                            &mut z_status_seen,
                            &mut latest_z,
                        );
                        status_clone.lock().unwrap().z_cal_error = Some(reason);
                        last_z_poll = Instant::now() - Duration::from_secs(1);
                    } else if orphaned_z_jog {
                        log::warn!("[Z-WATCHDOG] Z-axis still jogging with no button held -> sending Stop Z");
                        let raw_stop = build_z_stop_packet();
                        send_z_cmd_with_ack(
                            zs,
                            &raw_stop,
                            &mut z_buf,
                            &mut z_status_seen,
                            &mut latest_z,
                        );
                        last_z_poll = Instant::now() - Duration::from_secs(1);
                    }
                }

                // 7. Trigger deferred XY homing once Z homing has completed
                if let Some((home_cfg, started_at, deadline)) = pending_xy_home_after_z {
                    let z_done = {
                        let st = status_clone.lock().unwrap();
                        started_at.elapsed() >= Duration::from_millis(500)
                            && !st.is_homing_z
                            && (st.z_homed || latest_z.2 == 0)
                    };
                    if z_done || Instant::now() >= deadline {
                        let (r2, r4) = {
                            let st = status_clone.lock().unwrap();
                            (st.limit_sensor_mask, last_r1004)
                        };
                        eprintln!(
                            "[HOME] Z done (z_done={}, z_homed={}, z_state={}); limit_mask=0x{:02X} r1004=0x{:08X}; sending XY home (x_dir={}, y_dir={})",
                            z_done,
                            status_clone.lock().unwrap().z_homed,
                            latest_z.2,
                            r2,
                            r4,
                            home_cfg.x_dir,
                            home_cfg.y_dir,
                        );
                        pending_xy_home_after_z = None;
                        pending_stop_p2_at = None;
                        for (addr, vals) in [
                            (registers::REG_COMMAND, vec![2]),
                            (1344, vec![0]),
                            (registers::REG_ENABLE_FLAG, vec![1]),
                            (registers::REG_COMMAND, vec![3]),
                        ] {
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let p = Packet::write_request(s, addr, vals);
                            let _ = send_and_wait_ack(&socket, &p, &mut buf);
                        }
                        last_r1004 = 0;
                        let mut acks = Vec::new();
                        for hpkt in build_xy_home_packets(
                            || seq_clone.fetch_add(1, Ordering::Relaxed),
                            home_cfg,
                        ) {
                            acks.push(send_and_wait_ack(&socket, &hpkt, &mut buf));
                        }
                        eprintln!("[HOME] XY home acks: pkts={:?}", acks);
                    }
                }

                // 8. Stream 1-ms Motion FIFO ticks (`REG_FIFO_DATA = 100`) when active
                if let Some(mut fifo) = active_fifo.take() {
                    let mut sent_this_iter = 0;
                    while fifo.cursor < fifo.ticks.len()
                        && fifo.estimated_free > 120
                        && sent_this_iter < 16
                    {
                        let avail = (fifo.estimated_free - 60) as usize;
                        let take = (fifo.ticks.len() - fifo.cursor).min(60).min(avail);
                        if take == 0 {
                            break;
                        }
                        let chunk = &fifo.ticks[fifo.cursor..fifo.cursor + take];
                        let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                        let pkt = build_fifo_packet(
                            s,
                            fifo.fifo_seq,
                            chunk,
                        );
                        let _ = send_and_wait_ack(&socket, &pkt, &mut buf);
                        fifo.cursor += take;
                        fifo.fifo_seq = fifo.fifo_seq.wrapping_add(1);
                        fifo.estimated_free = fifo.estimated_free.saturating_sub(take as u32);
                        sent_this_iter += 1;
                    }

                    // Record real-time FIFO telemetry paced by actual elapsed time (~4 kHz SC2000 FIFO tick rate)
                    let elapsed_ticks = (fifo.started_at.elapsed().as_secs_f64() * 4000.0) as usize;
                    let target_tel_cursor = elapsed_ticks.min(fifo.ticks.len());
                    if fifo.last_telemetry_at.elapsed() >= Duration::from_millis(25)
                        && target_tel_cursor > fifo.telemetry_cursor
                    {
                        let slice = &fifo.ticks[fifo.telemetry_cursor..target_tel_cursor];
                        let (sum_x, sum_y) = slice.iter().fold((0i32, 0i32), |acc, &(dx, dy, fl)| {
                            if fl <= 0x03 {
                                let sx = if (fl & fifo_flags::X_NEG) != 0 { -(dx as i32) } else { dx as i32 };
                                let sy = if (fl & fifo_flags::Y_NEG) != 0 { -(dy as i32) } else { dy as i32 };
                                (acc.0 + sx, acc.1 + sy)
                            } else {
                                acc
                            }
                        });
                        let dt_s = (slice.len() as f32) * 0.00025;
                        let tag = if fifo.telemetry_cursor == 0 { Some("FIFO") } else { None };
                        telemetry_clone.lock().unwrap().push_command(
                            (sum_x as f32) / dt_s,
                            (sum_y as f32) / dt_s,
                            0.0,
                            0.0,
                            false,
                            tag,
                        );
                        fifo.telemetry_cursor = target_tel_cursor;
                        fifo.last_telemetry_at = Instant::now();
                    }

                    if !fifo.sent_run_flag
                        && (fifo.cursor >= fifo.ticks.len() || fifo.estimated_free < 500)
                    {
                        let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                        let pkt = Packet::write_request(s, registers::REG_COMMAND, vec![4]);
                        let _ = send_and_wait_ack(&socket, &pkt, &mut buf);
                        fifo.sent_run_flag = true;
                    }

                    let is_moving_now = status_clone.lock().unwrap().is_moving;
                    if is_moving_now {
                        fifo.seen_moving = true;
                    }
                    let est_secs = (fifo.ticks.len() as f64 / 4000.0) + 0.5;
                    let fifo_done = fifo.cursor >= fifo.ticks.len()
                        && ((fifo.seen_moving && !is_moving_now)
                            || (!is_moving_now
                                && fifo.started_at.elapsed() >= Duration::from_secs_f64(est_secs)));

                    if fifo_done {
                        for fpkt in build_fifo_finish_packets(|| {
                            seq_clone.fetch_add(1, Ordering::Relaxed)
                        }) {
                            let _ = send_and_wait_ack(&socket, &fpkt, &mut buf);
                        }
                        telemetry_clone.lock().unwrap().push_command(0.0, 0.0, 0.0, 0.0, false, Some("Done"));
                    } else {
                        active_fifo = Some(fifo);
                    }
                } else if status_clone.lock().unwrap().is_moving || auto_stop_at.is_some() {
                    telemetry_clone.lock().unwrap().tick_sample(50);
                }
            }
        });

        Self {
            target_addr,
            status,
            telemetry,
            running,
            connected,
            _seq: seq,
            command_tx,
            _worker: Some(worker),
        }
    }

    pub fn jog(&self, axis: Axis, pulses: i32) {
        let _ = self.command_tx.send(Command::Jog { axis, pulses });
    }

    pub fn jog_with_config(&self, axis: Axis, pulses: i32, config: JogConfig, auto_stop: bool) {
        let _ = self.command_tx.send(Command::JogWithConfig {
            axis,
            pulses,
            config,
            auto_stop,
        });
    }

    pub fn move_xy(&self, dx: i32, dy: i32, config: JogConfig) {
        let _ = self.command_tx.send(Command::MoveXY { dx, dy, config });
    }

    pub fn start_fifo_stream(&self, ticks: Vec<(i8, i8, u8)>) {
        let _ = self.command_tx.send(Command::StartFifoStream { ticks });
    }

    pub fn start_continuous_jog(&self, axis: Axis, direction: i32) {
        let _ = self
            .command_tx
            .send(Command::StartContinuousJog { axis, direction });
    }

    pub fn stop(&self) {
        let _ = self.command_tx.send(Command::Stop);
    }

    pub fn home_all(&self, cfg: HomeConfig) {
        let _ = self.command_tx.send(Command::HomeAll(cfg));
    }

    pub fn home_z(&self) {
        let _ = self.command_tx.send(Command::HomeZ);
    }

    pub fn update_jog_config(&self, config: JogConfig, z_speed: u32) {
        let _ = self
            .command_tx
            .send(Command::UpdateJogConfig { config, z_speed });
    }

    pub fn calibrate_height_sensor(&self) {
        let _ = self.command_tx.send(Command::CalibrateHeightSensor);
    }

    pub fn probe_surface_z(&self) {
        let _ = self.command_tx.send(Command::ProbeSurfaceZ);
    }

    pub fn set_follower_servo(&self, enable: bool) {
        let _ = self.command_tx.send(Command::SetFollowerServo(enable));
    }

    pub fn set_red_pointer(&self, enable: bool) {
        let _ = self.command_tx.send(Command::SetRedPointer(enable));
    }

    pub fn set_z_material_config(&self, material_thickness_mm: f64, pulses_per_mm_z: f64) {
        let _ = self.command_tx.send(Command::SetZMaterialConfig {
            material_thickness_mm,
            pulses_per_mm_z,
        });
    }

    pub fn laser_pulse(&self, duration_ms: u32, power_pct: f32) {
        let _ = self.command_tx.send(Command::LaserPulse { duration_ms, power_pct });
    }

    pub fn set_gas(&self, gas_type: u8, open: bool) {
        let _ = self.command_tx.send(Command::SetGas { gas_type, open });
    }

    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }

    pub fn get_status(&self) -> MachineStatus {
        self.status.lock().unwrap().clone()
    }

    pub fn telemetry(&self) -> Arc<Mutex<SignalTelemetry>> {
        Arc::clone(&self.telemetry)
    }

    pub fn target_addr(&self) -> SocketAddr {
        self.target_addr
    }
}

impl Drop for LaserController {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(worker) = self._worker.take() {
            let _ = worker.join();
        }
    }
}

