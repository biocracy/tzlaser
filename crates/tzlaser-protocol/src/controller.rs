use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::motion::{build_jog_packet, build_stop_packets, Axis, JogConfig, MachineStatus};
use crate::packet::{Opcode, Packet};

pub enum Command {
    Jog { axis: Axis, pulses: i32 },
    Stop,
    WriteRegisters { addr: u32, values: Vec<u32> },
    CalibrateHeightSensor,
    ProbeSurfaceZ,
    SetFollowerServo(bool),
    SetRedPointer(bool),
    LaserPulse { duration_ms: u32, power_pct: f32 },
    SetGas { gas_type: u8, open: bool },
}

/// Thread-safe client managing UDP communications with the TZLASER hardware board.
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
        let seq = Arc::new(AtomicU16::new(1));

        let status_clone = Arc::clone(&status);
        let running_clone = Arc::clone(&running);
        let connected_clone = Arc::clone(&connected);
        let seq_clone = Arc::clone(&seq);

        let worker = thread::spawn(move || {
            let socket = match UdpSocket::bind("0.0.0.0:0") {
                Ok(s) => s,
                Err(e) => {
                    log::error!("Failed to bind local UDP socket: {}", e);
                    return;
                }
            };
            socket
                .set_read_timeout(Some(Duration::from_millis(50)))
                .ok();

            let mut last_poll = Instant::now() - Duration::from_secs(10);
            let mut buf = [0u8; 1500];
            let jog_cfg = JogConfig::default();

            while running_clone.load(Ordering::Relaxed) {
                // 1. Process pending commands from UI
                while let Ok(cmd) = command_rx.try_recv() {
                    match cmd {
                        Command::Jog { axis, pulses } => {
                            let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                            let pkt = build_jog_packet(s, axis, pulses, &jog_cfg);
                            let _ = socket.send_to(&pkt.encode(), target_addr);
                        }
                        Command::Stop => {
                            let (p1, p2) = build_stop_packets(|| seq_clone.fetch_add(1, Ordering::Relaxed));
                            let _ = socket.send_to(&p1.encode(), target_addr);
                            let _ = socket.send_to(&p2.encode(), target_addr);
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

                // 2. Poll machine status every 100ms
                if last_poll.elapsed() >= Duration::from_millis(100) {
                    last_poll = Instant::now();
                    let s = seq_clone.fetch_add(1, Ordering::Relaxed);
                    let poll_pkt = Packet::read_request(s, 1000, 144);
                    let _ = socket.send_to(&poll_pkt.encode(), target_addr);
                }

                // 3. Receive incoming responses
                match socket.recv_from(&mut buf) {
                    Ok((n, src)) if src == target_addr => {
                        if let Ok(pkt) = Packet::decode(&buf[..n]) {
                            connected_clone.store(true, Ordering::Relaxed);
                            if pkt.opcode == Opcode::ReadRegisters && pkt.addr == 1000 {
                                let mut st = status_clone.lock().unwrap();
                                *st = MachineStatus::from_status_registers(&pkt.data);
                            }
                        }
                    }
                    Ok(_) => {} // ignore non-target traffic
                    Err(_) => {
                        // timeout is normal
                    }
                }

                thread::sleep(Duration::from_millis(5));
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

    pub fn stop(&self) {
        let _ = self.command_tx.send(Command::Stop);
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

