use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::motion::{
    build_init_packets, build_initial_enable_packets, build_jog_packet, build_stop_packets,
    build_z_handshake_packets, build_z_jog_packet, build_z_poll_packet, build_z_stop_packet,
    decode_z_poll_response, registers, Axis, JogConfig, MachineStatus, CONTINUOUS_JOG_PULSES_XY,
    CONTINUOUS_JOG_PULSES_Z, DEFAULT_Z_JOG_SPEED,
};
use crate::packet::{Opcode, Packet};

pub enum Command {
    Jog { axis: Axis, pulses: i32 },
    StartContinuousJog { axis: Axis, direction: i32 },
    Stop,
    UpdateJogConfig { config: JogConfig, z_speed: u32 },
    WriteRegisters { addr: u32, values: Vec<u32> },
    CalibrateHeightSensor,
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
        let running = Arc::new(AtomicBool::new(true));
        let connected = Arc::new(AtomicBool::new(false));
        let seq = Arc::new(AtomicU16::new(0));

        let status_clone = Arc::clone(&status);
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
                .set_read_timeout(Some(Duration::from_millis(15)))
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
            let mut xy_needs_reset = false;
            let mut pending_stop_p2_at: Option<Instant> = None;
            let mut auto_stop_at: Option<Instant> = None;
            let mut z_auto_stop_at: Option<Instant> = None;
            let mut active_step_watch: Option<(Axis, i32, i32)> = None;

            // Helper to send a Port 502 packet and wait briefly for its ACK during initialization
            let send_and_wait_ack = |sock: &UdpSocket, pkt: &Packet, recv_buf: &mut [u8]| -> bool {
                if sock.send_to(&pkt.encode(), target_addr).is_err() {
                    return false;
                }
                let deadline = Instant::now() + Duration::from_millis(25);
                while Instant::now() < deadline {
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

            // Send initial enable sequence on Port 502 and handshake on Port 999
            for pkt in build_initial_enable_packets(|| seq_clone.fetch_add(1, Ordering::Relaxed)) {
                let _ = send_and_wait_ack(&socket, &pkt, &mut buf);
            }
            if let Some(ref zs) = z_socket {
                for raw in build_z_handshake_packets() {
                    let _ = zs.send_to(&raw, z_target_addr);
                }
            }

            let mut last_poll = Instant::now() - Duration::from_secs(10);
            let mut last_z_poll = Instant::now() - Duration::from_secs(10);
            let mut last_enable_retry = Instant::now();

            while running_clone.load(Ordering::Relaxed) {
                // 0. Check scheduled stop-phase-2 (REG_COMMAND = 2 sent ~140ms after REG_JOG_RAMP)
                if let Some(t_p2) = pending_stop_p2_at {
                    if Instant::now() >= t_p2 {
                        pending_stop_p2_at = None;
                        let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                        let p2 = Packet::write_request(s, registers::REG_COMMAND, vec![2]);
                        let _ = socket.send_to(&p2.encode(), target_addr);
                        xy_needs_reset = false;
                    }
                }

                // Check scheduled auto-stop after a finite Step jog completes
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
                    }
                }

                // Check scheduled Z-axis auto-stop after a finite Z step jog
                if let Some(tz_stop) = z_auto_stop_at {
                    if Instant::now() >= tz_stop {
                        z_auto_stop_at = None;
                        if let Some(ref zs) = z_socket {
                            let _ = zs.send_to(&build_z_stop_packet(), z_target_addr);
                        }
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
                                    let raw = build_z_jog_packet(pulses, z_jog_speed);
                                    let _ = zs.send_to(&raw, z_target_addr);
                                    let z_secs = (pulses.unsigned_abs() as f64
                                        / (z_jog_speed.max(1) as f64 * 80.0))
                                        + 0.12;
                                    z_auto_stop_at = Some(
                                        Instant::now()
                                            + Duration::from_secs_f64(z_secs.clamp(0.08, 10.0)),
                                    );
                                }
                            } else {
                                if pending_stop_p2_at.take().is_some() || xy_needs_reset {
                                    let s_rst = seq_clone.fetch_add(1, Ordering::Relaxed);
                                    let p2 = Packet::write_request(
                                        s_rst,
                                        registers::REG_COMMAND,
                                        vec![2],
                                    );
                                    let _ = send_and_wait_ack(&socket, &p2, &mut buf);
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
                                xy_needs_reset = true;

                                active_step_watch =
                                    Some((axis, cur_pos, pulses.unsigned_abs() as i32));
                                let move_secs = (pulses.unsigned_abs() as f64
                                    / jog_cfg.jog_speed.max(1) as f64)
                                    + 0.10;
                                auto_stop_at = Some(
                                    Instant::now()
                                        + Duration::from_secs_f64(move_secs.clamp(0.06, 15.0)),
                                );
                            }
                        }
                        Command::StartContinuousJog { axis, direction } => {
                            let sign = if direction >= 0 { 1 } else { -1 };
                            if axis == Axis::Z {
                                z_auto_stop_at = None;
                                if let Some(ref zs) = z_socket {
                                    let raw = build_z_jog_packet(
                                        sign * CONTINUOUS_JOG_PULSES_Z,
                                        z_jog_speed,
                                    );
                                    let _ = zs.send_to(&raw, z_target_addr);
                                }
                            } else {
                                auto_stop_at = None;
                                active_step_watch = None;
                                if pending_stop_p2_at.take().is_some() || xy_needs_reset {
                                    let s_rst = seq_clone.fetch_add(1, Ordering::Relaxed);
                                    let p2 = Packet::write_request(
                                        s_rst,
                                        registers::REG_COMMAND,
                                        vec![2],
                                    );
                                    let _ = send_and_wait_ack(&socket, &p2, &mut buf);
                                }
                                let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                                let pkt = build_jog_packet(
                                    s,
                                    axis,
                                    sign * CONTINUOUS_JOG_PULSES_XY,
                                    &jog_cfg,
                                );
                                let _ = socket.send_to(&pkt.encode(), target_addr);
                                xy_needs_reset = true;
                            }
                        }
                        Command::Stop => {
                            auto_stop_at = None;
                            z_auto_stop_at = None;
                            active_step_watch = None;
                            let (p1, _p2) = build_stop_packets(|| {
                                seq_clone.fetch_add(1, Ordering::Relaxed)
                            });
                            let _ = socket.send_to(&p1.encode(), target_addr);
                            pending_stop_p2_at = Some(Instant::now() + Duration::from_millis(140));

                            if let Some(ref zs) = z_socket {
                                let _ = zs.send_to(&build_z_stop_packet(), z_target_addr);
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
                            log::info!("Initiating capacitive height sensor calibration routine");
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let pkt = Packet::write_request(s, 2100, vec![1]);
                            let _ = socket.send_to(&pkt.encode(), target_addr);
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

                // 3. Poll Port 502 machine status every 50ms
                if last_poll.elapsed() >= Duration::from_millis(50) {
                    last_poll = Instant::now();
                    let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                    let poll_pkt = Packet::read_request(s, registers::REG_STATUS_BASE, 144);
                    let _ = socket.send_to(&poll_pkt.encode(), target_addr);
                }

                // 4. Poll Port 999 Z-axis & height controller status every 500ms
                if last_z_poll.elapsed() >= Duration::from_millis(500) {
                    last_z_poll = Instant::now();
                    if let Some(ref zs) = z_socket {
                        let _ = zs.send_to(&build_z_poll_packet(), z_target_addr);
                    }
                }

                // 5. Receive incoming responses from Port 502
                match socket.recv_from(&mut buf) {
                    Ok((n, src)) if src == target_addr => {
                        if let Ok(pkt) = Packet::decode(&buf[..n]) {
                            connected_clone.store(true, Ordering::Relaxed);

                            if !board_initialized {
                                board_initialized = true;
                                log::info!(
                                    "Controller online at {}; running SC2000 board initialization sequence...",
                                    target_addr
                                );
                                for init_pkt in build_init_packets(|| {
                                    seq_clone.fetch_add(1, Ordering::Relaxed)
                                }) {
                                    let _ = send_and_wait_ack(&socket, &init_pkt, &mut buf);
                                }
                                log::info!("Controller initialization sequence complete.");
                            }

                            if pkt.opcode == Opcode::ReadRegisters
                                && pkt.addr == registers::REG_STATUS_BASE
                                && pkt.data.len() >= 24
                            {
                                let mut new_st = MachineStatus::from_status_registers(&pkt.data);
                                if z_status_seen {
                                    new_st.apply_z_status(latest_z.0, latest_z.1, latest_z.2);
                                }
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
                                *st = new_st;
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(_) => {}
                }

                // 6. Receive incoming responses from Port 999 (Z-axis / Capacitive Follower)
                if let Some(ref zs) = z_socket {
                    while let Ok((n, src)) = zs.recv_from(&mut z_buf) {
                        if src == z_target_addr {
                            if let Some((pos_z, raw_cap, state_code)) =
                                decode_z_poll_response(&z_buf[..n])
                            {
                                z_status_seen = true;
                                latest_z = (pos_z, raw_cap, state_code);
                                connected_clone.store(true, Ordering::Relaxed);
                                let mut st = status_clone.lock().unwrap();
                                st.is_connected = true;
                                st.apply_z_status(pos_z, raw_cap, state_code);
                            }
                        }
                    }
                }
            }
        });

        Self {
            target_addr,
            status,
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

    pub fn start_continuous_jog(&self, axis: Axis, direction: i32) {
        let _ = self
            .command_tx
            .send(Command::StartContinuousJog { axis, direction });
    }

    pub fn stop(&self) {
        let _ = self.command_tx.send(Command::Stop);
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

    pub fn target_addr(&self) -> SocketAddr {
        self.target_addr
    }
}

impl Drop for LaserController {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
    }
}

