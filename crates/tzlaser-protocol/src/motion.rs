use crate::crc::modbus_crc16;
use crate::packet::Packet;

/// Registers discovered from Wireshark captures of TZLASER (Port 502)
pub mod registers {
    pub const REG_FIFO_DATA: u32 = 100;    // 0x0064: Motion FIFO stream [fifo_seq, (tick_word0, tick_word1)...]
    pub const REG_COMMAND: u32 = 104;      // 0x0068: 1 = Start FIFO, 2 = Stop/Cancel, 3 = Init, 4 = FIFO Run/End
    pub const REG_MODE_CTRL: u32 = 108;    // 0x006C: 2 DWORDs mode/reset flags
    pub const REG_HOME_CMD: u32 = 112;     // 0x0070: 9 DWORDs XY homing parameters & trigger
    pub const REG_JOG_MOTION: u32 = 116;   // 0x0074: 7 DWORDs [vmin, vjog, accel, dx, dy, dz, dw]
    pub const REG_JOG_RAMP: u32 = 120;     // 0x0078: 2 DWORDs [118000, 1180000]
    pub const REG_ENABLE_FLAG: u32 = 124;  // 0x007C: 1 DWORD [1]
    pub const REG_STATUS_BASE: u32 = 1000; // 0x03E8: 144 DWORDs polling block
}

/// Default continuous jog travel distance in pulses on Port 502 (matches SC2000 `±472,000`).
pub const CONTINUOUS_JOG_PULSES_XY: i32 = 472_000;

/// Default continuous jog travel distance in pulses on Port 999 (matches SC2000 `±1,000,000`).
pub const CONTINUOUS_JOG_PULSES_Z: i32 = 1_000_000;

/// Default Z-axis jog speed register value on Port 999 (SC2000 `ZFFastJogSpeed = 50`).
pub const DEFAULT_Z_JOG_SPEED: u32 = 50;

/// Wireless remote control button bitmasks reported by the controller board in `R1042` (`regs[42]`)
/// (`RegName43` = LCR real-time key value) and mirrored in `R1043` (`regs[43]`, `RegName44` = LCR matched code value).
/// Extracted from SC2000 `MainApp.exe` key table at VA `0x006C1300`.
pub mod remote_buttons {
    pub const W_MINUS: u32 = 0x0000_0001;   // Bit 0: Jog W-
    pub const X_MINUS: u32 = 0x0000_0002;   // Bit 1: Jog X-
    pub const W_PLUS: u32 = 0x0000_0004;    // Bit 2: Jog W+
    pub const START: u32 = 0x0000_0008;     // Bit 3: Start
    pub const SPEED: u32 = 0x0000_0010;     // Bit 4: Fast/Slow speed toggle
    pub const FRAME: u32 = 0x0000_0020;     // Bit 5: Frame / Border trace
    pub const GUIDE: u32 = 0x0000_0040;     // Bit 6: Red light / Guide laser
    pub const Y_MINUS: u32 = 0x0000_0100;   // Bit 8: Jog Y-
    pub const Y_PLUS: u32 = 0x0000_0400;    // Bit 10: Jog Y+
    pub const PAUSE: u32 = 0x0000_0800;     // Bit 11: Pause
    pub const STEP_MODE: u32 = 0x0000_1000; // Bit 12: Step / Continuous mode
    pub const GAS: u32 = 0x0000_4000;       // Bit 14: Assist gas blow
    pub const Z_MINUS: u32 = 0x0001_0000;   // Bit 16: Jog Z- (lower head)
    pub const X_PLUS: u32 = 0x0002_0000;    // Bit 17: Jog X+
    pub const Z_PLUS: u32 = 0x0004_0000;    // Bit 18: Jog Z+ (raise head toward upper limit)
    pub const STOP: u32 = 0x0008_0000;      // Bit 19: Stop
    pub const ZERO: u32 = 0x0010_0000;      // Bit 20: Return to Zero
    pub const FN_MOD: u32 = 0x0020_0000;    // Bit 21: Fn modifier flag
    pub const FOLLOW: u32 = 0x0040_0000;    // Bit 22: Capacitive follower toggle
    pub const CLEAR: u32 = 0x0080_0000;     // Bit 23: Clear
    pub const STEP_FLAG: u32 = 0x0100_0000; // Bit 24: Remote step mode state flag
    pub const FAST_FLAG: u32 = 0x0200_0000; // Bit 25: Remote fast speed state flag
    pub const ORIGIN: u32 = 0x1000_0000;    // Bit 28: Return to Origin
    pub const PULSE: u32 = 0x4000_0000;     // Bit 30: Laser Burst / Pulse
    /// Sticky modifier/mode state bits in the upper byte of R1042 that may remain set when no action key is held.
    pub const STICKY_FLAGS: u32 = STEP_FLAG | FAST_FLAG | FN_MOD;
}

/// Decode a wireless remote control button bitmask (`R1042`) into a continuous jog `(Axis, direction)`.
pub fn decode_remote_jog(mask: u32) -> Option<(Axis, i32)> {
    if (mask & remote_buttons::X_MINUS) != 0 {
        Some((Axis::X, -1))
    } else if (mask & remote_buttons::X_PLUS) != 0 {
        Some((Axis::X, 1))
    } else if (mask & remote_buttons::Y_MINUS) != 0 {
        Some((Axis::Y, -1))
    } else if (mask & remote_buttons::Y_PLUS) != 0 {
        Some((Axis::Y, 1))
    } else if (mask & remote_buttons::Z_MINUS) != 0 {
        Some((Axis::Z, -1))
    } else if (mask & remote_buttons::Z_PLUS) != 0 {
        Some((Axis::Z, 1))
    } else if (mask & remote_buttons::W_MINUS) != 0 {
        Some((Axis::W, -1))
    } else if (mask & remote_buttons::W_PLUS) != 0 {
        Some((Axis::W, 1))
    } else {
        None
    }
}

/// Format a human-readable label for the active wireless remote control button bitmask (`R1042`).
pub fn format_remote_button(mask: u32) -> String {
    let key = mask & !remote_buttons::STICKY_FLAGS;
    if key == 0 {
        return "Idle".to_string();
    }
    if let Some((axis, dir)) = decode_remote_jog(key) {
        let sign = if dir >= 0 { "+" } else { "-" };
        let speed_tag = if (mask & remote_buttons::FAST_FLAG) != 0 {
            " [Fast]"
        } else {
            ""
        };
        return format!("Jog {:?}{}{} (0x{:X})", axis, sign, speed_tag, key);
    }
    let name = if (key & remote_buttons::START) != 0 {
        "Start"
    } else if (key & remote_buttons::PAUSE) != 0 {
        "Pause"
    } else if (key & remote_buttons::STOP) != 0 {
        "Stop"
    } else if (key & remote_buttons::FRAME) != 0 {
        "Frame"
    } else if (key & remote_buttons::SPEED) != 0 {
        "Speed"
    } else if (key & remote_buttons::STEP_MODE) != 0 {
        "Step/Cont"
    } else if (key & remote_buttons::GUIDE) != 0 {
        "Guide Laser"
    } else if (key & remote_buttons::GAS) != 0 {
        "Gas Blow"
    } else if (key & remote_buttons::ZERO) != 0 {
        "Return Zero"
    } else if (key & remote_buttons::ORIGIN) != 0 {
        "Return Origin"
    } else if (key & remote_buttons::FOLLOW) != 0 {
        "Follower"
    } else if (key & remote_buttons::PULSE) != 0 {
        "Laser Pulse"
    } else if (key & remote_buttons::CLEAR) != 0 {
        "Clear"
    } else {
        "Key"
    };
    format!("{} (0x{:X})", name, key)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
    W,
}

#[derive(Debug, Clone)]
pub struct JogConfig {
    pub min_speed: u32,  // pulses/sec (conservative default 1,000; SC2000 capture used 5,900)
    pub jog_speed: u32,  // pulses/sec (conservative default 2,000; SC2000 capture used 5,900)
    pub accel: u32,      // pulses/sec^2 (conservative default 16,000; SC2000 capture used 59,000)
}

impl Default for JogConfig {
    fn default() -> Self {
        Self {
            min_speed: 1000,
            jog_speed: 2000,
            accel: 16000,
        }
    }
}

/// Machine status decoded from Register 1000..1143 polling response.
#[derive(Debug, Clone)]
pub struct MachineStatus {
    pub pos_x: i32,
    pub pos_y: i32,
    pub pos_z: i32,
    pub pos_w: i32,
    pub is_connected: bool,
    pub laser_on: bool,
    pub laser_power_pct: f32,
    pub laser_temp_c: f32,
    pub red_pointer_on: bool,
    pub gas_valve_open: bool,
    pub gas_type: String,
    pub gas_pressure_bar: f32,
    pub capacitive_height_mm: f32,
    pub capacitance_pf: f32,
    pub follower_active: bool,
    pub remote_button_mask: u32,
    pub xy_homed: bool,
    pub z_homed: bool,
    pub is_homing_xy: bool,
    pub is_homing_z: bool,
    pub is_moving: bool,
    pub z_pos_sensor: bool,
    pub z_neg_sensor: bool,
    /// Upper byte of `R1002` (`(regs[2] >> 16) & 0xFF`): hardware limit sensor bitmask
    /// (observed: `0x01` = X+, `0x02` = X-, `0x90` = Y1-/Y2- dual-drive).
    pub limit_sensor_mask: u32,
    pub raw_registers: Vec<u32>,
}

impl Default for MachineStatus {
    fn default() -> Self {
        Self {
            pos_x: 0,
            pos_y: 0,
            pos_z: 0,
            pos_w: 0,
            is_connected: false,
            laser_on: false,
            laser_power_pct: 0.0,
            laser_temp_c: 23.5,
            red_pointer_on: false,
            gas_valve_open: false,
            gas_type: "Air".to_string(),
            gas_pressure_bar: 0.0,
            capacitive_height_mm: 50.0,
            capacitance_pf: 12.4,
            follower_active: false,
            remote_button_mask: 0,
            xy_homed: false,
            z_homed: false,
            is_homing_xy: false,
            is_homing_z: false,
            is_moving: false,
            z_pos_sensor: false,
            z_neg_sensor: false,
            limit_sensor_mask: 0,
            raw_registers: Vec::new(),
        }
    }
}

impl MachineStatus {
    pub fn from_status_registers(regs: &[u32]) -> Self {
        if regs.len() < 24 {
            return Self::default();
        }

        // Verified from SC2000 X/Y/Z Wireshark capture:
        // R1008 tracks live X position; R1009 (and R1010 dual-drive gantry) tracks live Y position.
        // (R1012 and R1014 are X and Y encoder feedback at 1/1.5 ratio; Z comes from Port 999 D9.)
        let pos_x = regs.get(8).copied().unwrap_or(0) as i32;
        let pos_y = regs.get(9).copied().unwrap_or(0) as i32;
        let pos_z = 0;
        let pos_w = regs.get(23).copied().unwrap_or(0) as i32;

        // Wireless remote control button bitmask is reported in R1042 (regs[42], RegName43: LCR real time key value)
        // and mirrored in R1043 (regs[43], RegName44: LCR matched code value).
        // Read `r43 | r42` so both matched remote presses and real-time key presses are captured without dropping transition frames.
        let r42 = regs.get(42).copied().unwrap_or(0);
        let r43 = regs.get(43).copied().unwrap_or(0);
        let remote_button_mask = r43 | r42;

        let laser_on = regs.get(30).copied().unwrap_or(0) & 0x01 != 0;
        let laser_power_pct = (regs.get(31).copied().unwrap_or(0) as f32) / 10.0;
        let laser_temp_c = 22.0 + ((regs.get(32).copied().unwrap_or(25) % 15) as f32) * 0.5;
        let red_pointer_on = regs.get(33).copied().unwrap_or(0) != 0;
        let gas_val = regs.get(34).copied().unwrap_or(0);
        let gas_valve_open = gas_val != 0;
        let gas_type = match gas_val {
            1 => "Oxygen (O₂)".to_string(),
            2 => "Nitrogen (N₂)".to_string(),
            3 => "Air".to_string(),
            _ => "Air".to_string(),
        };
        let gas_pressure_bar = if gas_valve_open { 8.5 } else { 0.0 };
        let raw_cap = regs.get(35).copied().unwrap_or(1240) as f32;
        let capacitance_pf = raw_cap / 100.0;
        let capacitive_height_mm = 50.0;
        let follower_active = regs.get(36).copied().unwrap_or(0) != 0;
        let xy_homed = (regs.get(3).copied().unwrap_or(0) & 0x03) == 0x03;
        let is_homing_xy = regs.get(5).copied().unwrap_or(0) == 1;
        let is_moving = regs.get(7).copied().unwrap_or(0) != 0;
        let limit_sensor_mask = (regs.get(2).copied().unwrap_or(0) >> 16) & 0xFF;

        Self {
            pos_x,
            pos_y,
            pos_z,
            pos_w,
            is_connected: true,
            laser_on,
            laser_power_pct,
            laser_temp_c,
            red_pointer_on,
            gas_valve_open,
            gas_type,
            gas_pressure_bar,
            capacitive_height_mm,
            capacitance_pf,
            follower_active,
            remote_button_mask,
            xy_homed,
            z_homed: false,
            is_homing_xy,
            is_homing_z: false,
            is_moving,
            z_pos_sensor: false,
            z_neg_sensor: false,
            limit_sensor_mask,
            raw_registers: regs.to_vec(),
        }
    }

    /// Returns true if any limit sensor on `axis` is currently active in `R1002`.
    pub fn axis_limit_active(&self, axis: Axis) -> bool {
        match axis {
            Axis::X => (self.limit_sensor_mask & 0x03) != 0,
            Axis::Y => (self.limit_sensor_mask & 0xFC) != 0,
            Axis::Z | Axis::W => false,
        }
    }

    pub fn x_pos_limit(&self) -> bool {
        (self.limit_sensor_mask & 0x01) != 0
    }

    pub fn x_neg_limit(&self) -> bool {
        (self.limit_sensor_mask & 0x02) != 0
    }

    pub fn y_pos_limit(&self) -> bool {
        (self.limit_sensor_mask & 0x6C) != 0
    }

    pub fn y_neg_limit(&self) -> bool {
        (self.limit_sensor_mask & 0x90) != 0
    }

    pub fn z_pos_limit(&self) -> bool {
        self.is_connected && self.z_pos_sensor
    }

    pub fn z_neg_cap_limit(&self) -> bool {
        self.is_connected && self.z_neg_sensor
    }

    /// Merge status decoded from the Port 999 Z-axis / capacitive follower poll response.
    pub fn apply_z_status(&mut self, pos_z: i32, raw_cap: u32, follower_state: u32) {
        self.pos_z = pos_z;
        if raw_cap > 0 {
            self.capacitance_pf = (raw_cap as f32) / 10_000.0;
        }
        self.capacitive_height_mm = ((pos_z as f32).abs() / 1_000.0).clamp(0.0, 150.0);
        self.follower_active = follower_state == 2;
    }
}

/// Build the initial 3-packet enable sequence sent immediately on Port 502 connection.
pub fn build_initial_enable_packets(mut seq_gen: impl FnMut() -> u16) -> Vec<Packet> {
    vec![
        Packet::read_request(seq_gen(), registers::REG_STATUS_BASE, 2),
        Packet::write_request(seq_gen(), 1452, vec![0]),
        Packet::write_request(seq_gen(), registers::REG_ENABLE_FLAG, vec![1]),
    ]
}

/// Build the full SC2000 board parameter and motion initialization sequence for Port 502.
/// Sending this sequence clears `R1004` (`0x00880000 -> 0`) and enables XY motion.
pub fn build_init_packets(mut seq_gen: impl FnMut() -> u16) -> Vec<Packet> {
    let mut pkts = vec![
        // Handshake / board identification exchange
        Packet::write_request(
            seq_gen(),
            180,
            vec![0xC082_6745, 0xD362_9CE7, 0x693A_C5D2, 0xA792_BD68],
        ),
        Packet::read_request(seq_gen(), 180, 1),
        Packet::read_request(seq_gen(), 184, 8),
        Packet::write_request(
            seq_gen(),
            180,
            vec![0xC082_6745, 0xD362_9CE7, 0x693A_C5D2, 0xA792_BD68],
        ),
        Packet::read_request(seq_gen(), 180, 1),
        Packet::read_request(seq_gen(), 184, 8),
        Packet::read_request(seq_gen(), 196, 32),
        Packet::write_request(seq_gen(), 172, vec![0x0000_764D]),
        Packet::read_request(seq_gen(), 176, 200),
        Packet::read_request(seq_gen(), 192, 256),
        // Motion & soft-limit configuration registers
        Packet::write_request(seq_gen(), 1284, vec![15]),
        Packet::write_request(seq_gen(), 1288, vec![0]),
        Packet::write_request(seq_gen(), registers::REG_ENABLE_FLAG, vec![1]),
        Packet::write_request(seq_gen(), 1228, vec![32864, 354, 100]),
        Packet::write_request(seq_gen(), 1304, vec![471_568]),
        Packet::write_request(seq_gen(), 1244, vec![24823]),
        Packet::write_request(seq_gen(), 1248, vec![0]),
        // Initialize hardware soft limits R1063..R1070 (addr = 1252, 8 DWORDs: X+, X-, Y1+, Y1-, Y2+, Y2-, W+, W-)
        // to unbounded [+MAX, -MAX] (matching SC2000 Frame 953) so ±472,000 continuous & remote jogs are never blocked.
        Packet::write_request(
            seq_gen(),
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
        ),
        Packet::write_request(seq_gen(), registers::REG_ENABLE_FLAG, vec![1]),
        Packet::write_request(seq_gen(), 1308, vec![16680]),
        Packet::write_request(seq_gen(), 1332, vec![1]),
        Packet::write_request(seq_gen(), 1204, vec![0x8000_0000]),
        Packet::write_request(seq_gen(), 1408, vec![0xFFFF_FFFF]),
        Packet::write_request(seq_gen(), 1408, vec![0x7FFF_FFFF]),
        Packet::write_request(seq_gen(), 1208, vec![10_000]),
        Packet::write_request(
            seq_gen(),
            registers::REG_MODE_CTRL,
            vec![0x8000_0000, 0x8000_0000],
        ),
        Packet::write_request(seq_gen(), 1312, vec![5000]),
        Packet::write_request(seq_gen(), 1404, vec![30]),
        Packet::write_request(seq_gen(), 140, vec![3]),
        Packet::write_request(seq_gen(), 1440, vec![1]),
    ];

    for addr in (1456..=1520).step_by(4) {
        pkts.push(Packet::write_request(seq_gen(), addr, vec![0]));
    }

    // Issue Init command (3) to REG_COMMAND (104) followed by mode control flags
    pkts.push(Packet::write_request(seq_gen(), registers::REG_COMMAND, vec![3]));
    pkts.push(Packet::write_request(seq_gen(), registers::REG_MODE_CTRL, vec![256, 0]));
    pkts.push(Packet::write_request(seq_gen(), registers::REG_MODE_CTRL, vec![1024, 0]));

    pkts
}

/// Build packets for Jogging an axis by a given number of pulses on Port 502.
pub fn build_jog_packet(seq: u16, axis: Axis, pulses: i32, config: &JogConfig) -> Packet {
    let mut dx: i32 = 0;
    let mut dy: i32 = 0;
    let mut dz: i32 = 0;
    let mut dw: i32 = 0;

    match axis {
        Axis::X => dx = pulses,
        Axis::Y => dy = pulses,
        Axis::Z => dz = pulses,
        Axis::W => dw = pulses,
    }

    let payload = vec![
        config.min_speed,
        config.jog_speed,
        config.accel,
        dx as u32,
        dy as u32,
        dz as u32,
        dw as u32,
    ];

    Packet::write_request(seq, registers::REG_JOG_MOTION, payload)
}

/// Build a coordinated 2-axis XY move packet on Port 502 (`REG_JOG_MOTION`).
pub fn build_xy_move_packet(seq: u16, dx: i32, dy: i32, config: &JogConfig) -> Packet {
    let payload = vec![
        config.min_speed,
        config.jog_speed,
        config.accel,
        dx as u32,
        dy as u32,
        0,
        0,
    ];
    Packet::write_request(seq, registers::REG_JOG_MOTION, payload)
}

/// Pack a single 1 ms FIFO motion tick `[dx, dy, dz, flags]` into a little-endian `u32`.
pub fn encode_fifo_tick(dx: i8, dy: i8, dz: i8, flags: u8) -> u32 {
    u32::from_le_bytes([dx as u8, dy as u8, dz as u8, flags])
}

/// Build a Port 502 Motion FIFO packet (`REG_FIFO_DATA = 100`) containing up to 60 1-ms ticks.
pub fn build_fifo_packet(seq: u16, fifo_seq: u32, ticks: &[(i8, i8, u8)]) -> Packet {
    let mut payload = Vec::with_capacity(1 + ticks.len() * 2);
    payload.push(fifo_seq);
    for &(dx, dy, flags) in ticks {
        payload.push(encode_fifo_tick(dx, dy, 0, flags));
        payload.push(0);
    }
    Packet::write_request(seq, registers::REG_FIFO_DATA, payload)
}

/// Build the Port 502 setup sequence to enter continuous Motion FIFO mode (`REG_FIFO_DATA = 100`).
pub fn build_fifo_start_packets(
    mut seq_gen: impl FnMut() -> u16,
    pos_x: i32,
    pos_y: i32,
) -> Vec<Packet> {
    let px = pos_x as u32;
    let py = pos_y as u32;
    vec![
        Packet::write_request(seq_gen(), 1388, vec![px, py, px, py]),
        Packet::write_request(seq_gen(), 1376, vec![0xFFFF_FFFF]),
        Packet::write_request(seq_gen(), 1244, vec![24820]),
        Packet::write_request(seq_gen(), registers::REG_ENABLE_FLAG, vec![2]),
        Packet::write_request(seq_gen(), registers::REG_COMMAND, vec![3]),
        Packet::write_request(seq_gen(), registers::REG_COMMAND, vec![1]),
        Packet::write_request(seq_gen(), registers::REG_ENABLE_FLAG, vec![3]),
    ]
}

/// Build the Port 502 teardown sequence after all FIFO ticks have finished executing.
pub fn build_fifo_finish_packets(mut seq_gen: impl FnMut() -> u16) -> Vec<Packet> {
    vec![
        Packet::write_request(seq_gen(), registers::REG_COMMAND, vec![4]),
        Packet::write_request(seq_gen(), registers::REG_COMMAND, vec![2]),
        Packet::write_request(seq_gen(), 1344, vec![0]),
        Packet::write_request(seq_gen(), 1244, vec![24823]),
        Packet::write_request(seq_gen(), registers::REG_ENABLE_FLAG, vec![1]),
    ]
}

/// Build packets to stop motion on Port 502 (`p1` = deceleration ramp, `p2` = stop command 2).
/// Note: SC2000 sends `p1` first, then sends `p2` ~140ms later.
pub fn build_stop_packets(mut seq_gen: impl FnMut() -> u16) -> (Packet, Packet) {
    let p1 = Packet::write_request(
        seq_gen(),
        registers::REG_JOG_RAMP,
        vec![118_000, 1_180_000],
    );
    let p2 = Packet::write_request(
        seq_gen(),
        registers::REG_COMMAND,
        vec![2], // Stop code 2
    );
    (p1, p2)
}

// ============================================================================
// Port 999: Z-Axis & Capacitive Height Controller (Modbus RTU over UDP)
// ============================================================================

/// Append Little-Endian Modbus CRC-16 to a raw Modbus RTU frame.
pub fn encode_modbus_rtu(frame_without_crc: &[u8]) -> Vec<u8> {
    let crc = modbus_crc16(frame_without_crc);
    let mut out = Vec::with_capacity(frame_without_crc.len() + 2);
    out.extend_from_slice(frame_without_crc);
    out.extend_from_slice(&crc.to_le_bytes());
    out
}

/// Build a Modbus RTU Function 0x03 (Read Holding Registers) request for Port 999.
pub fn build_modbus_read_request(slave: u8, start_addr: u16, word_count: u16) -> Vec<u8> {
    let mut body = [0u8; 6];
    body[0] = slave;
    body[1] = 0x03;
    body[2..4].copy_from_slice(&start_addr.to_be_bytes());
    body[4..6].copy_from_slice(&word_count.to_be_bytes());
    encode_modbus_rtu(&body)
}

/// Build the initial Port 999 read sequence observed when SC2000 connects.
pub fn build_z_handshake_packets() -> Vec<Vec<u8>> {
    vec![
        build_modbus_read_request(0x01, 0x03E8, 0x0004), // 010303e80004c479
        build_modbus_read_request(0x01, 0x03E8, 0x0040), // 010303e80040c44a
        build_modbus_read_request(0x01, 0x07D0, 0x0050), // 010307d00050457b
        build_modbus_read_request(0x01, 0x0820, 0x005A), // 01030820005ac65b
    ]
}

/// Build the periodic Port 999 status poll packet (`01 03 03e8 0040 c44a`).
pub fn build_z_poll_packet() -> Vec<u8> {
    build_modbus_read_request(0x01, 0x03E8, 0x0040)
}

/// Build a Port 999 Z-axis jog packet (Modbus Function 0x10 at register `0x0065`, 6 words).
/// Note on FTC10 / MCC3721NC_ZF hardware polarity:
/// - Raising the head toward the upper limit (`Z+`, `pulses > 0`) requires a **negative** hardware pulse target (`-pulses`).
/// - Lowering the head toward the sheet (`Z-`, `pulses < 0`) requires a **positive** hardware pulse target (`-pulses`).
/// Matches SC2000 packets:
/// - Z+ (`pulses = +1_000_000` -> hw `-1_000_000`, `speed = 100`): `0110006500060c0009000000640000bdc0fff08f12`
/// - Z- (`pulses = -1_000_000` -> hw `+1_000_000`, `speed = 100`): `0110006500060c00090000006400004240000fbf5e`
pub fn build_z_jog_packet(pulses: i32, speed: u32) -> Vec<u8> {
    let cmd: u32 = 9;
    let hw_pulses = pulses.saturating_neg();
    let p = hw_pulses as u32;
    let words: [u16; 6] = [
        (cmd & 0xFFFF) as u16,
        ((cmd >> 16) & 0xFFFF) as u16,
        (speed & 0xFFFF) as u16,
        ((speed >> 16) & 0xFFFF) as u16,
        (p & 0xFFFF) as u16,
        ((p >> 16) & 0xFFFF) as u16,
    ];

    let mut body = Vec::with_capacity(7 + 12);
    body.extend_from_slice(&[0x01, 0x10, 0x00, 0x65, 0x00, 0x06, 0x0C]);
    for w in words {
        body.extend_from_slice(&w.to_be_bytes());
    }
    encode_modbus_rtu(&body)
}

/// Build a Port 999 Z-axis stop packet (Modbus Function 0x10 at register `0x0065`, 2 words).
/// Matches SC2000 packet: `01100065000204000100006478`.
pub fn build_z_stop_packet() -> Vec<u8> {
    let body: [u8; 11] = [
        0x01, 0x10, 0x00, 0x65, 0x00, 0x02, 0x04, 0x00, 0x01, 0x00, 0x00,
    ];
    encode_modbus_rtu(&body)
}

/// Build a Port 999 Z-axis home packet (Modbus Function 0x10 at register `0x0065`, 2 words, `cmd = 2`).
/// Raises the head (`Z+`) until the upper limit/capacitive sensor triggers, then zeroes Z.
/// Matches SC2000 packet: `01100065000204000200009478`.
pub fn build_z_home_packet() -> Vec<u8> {
    let body: [u8; 11] = [
        0x01, 0x10, 0x00, 0x65, 0x00, 0x02, 0x04, 0x00, 0x02, 0x00, 0x00,
    ];
    encode_modbus_rtu(&body)
}

#[derive(Debug, Clone, Copy)]
pub struct HomeConfig {
    /// +1 = home X toward +X (Right), -1 = home X toward -X (Left)
    pub x_dir: i32,
    /// +1 = home Y toward +Y (Top), -1 = home Y toward -Y (Bottom)
    pub y_dir: i32,
    /// Fast seek speed in pulses/s
    pub fast_speed_pls_s: u32,
    /// Slow/creep backoff speed in pulses/s
    pub slow_speed_pls_s: u32,
    /// Maximum search distance in pulses
    pub max_search_pulses: u32,
}

/// Build the Port 502 XY homing sequence (`REG_HOME_CMD = 112`) toward the configured origin corner.
/// Observed in SC2000 homing capture:
/// - `1244`: `24823` (`0x60F7` sensor/limit config)
/// - `1412`: `[0, 0, 0, 0]` (clear home/work offset)
/// - `1252`: unbounded soft limits `R1063..R1070`
/// - `112`: `[flags_mask, slow, accel_step, fast, max_dist, slow, slow, slow, 0]`
///   where `flags_mask = 0x0103 | (x_dir >= 0 ? 0x10 : 0) | (y_dir >= 0 ? 0x20 : 0)` (`275` for Bottom-Right).
pub fn build_xy_home_packets(mut seq_gen: impl FnMut() -> u16, cfg: HomeConfig) -> Vec<Packet> {
    let x_bit = if cfg.x_dir >= 0 { 0x10 } else { 0 };
    let y_bit = if cfg.y_dir >= 0 { 0x20 } else { 0 };
    let flags_mask = 0x0103_u32 | x_bit | y_bit;

    let slow = cfg.slow_speed_pls_s.max(100);
    let accel_step = (slow / 5).max(50);
    let fast = cfg.fast_speed_pls_s.max(slow);
    let max_dist = cfg.max_search_pulses.max(10_000);

    vec![
        Packet::write_request(seq_gen(), 1244, vec![24823]),
        Packet::write_request(seq_gen(), 1412, vec![0, 0, 0, 0]),
        Packet::write_request(
            seq_gen(),
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
        ),
        Packet::write_request(
            seq_gen(),
            registers::REG_HOME_CMD,
            vec![flags_mask, slow, accel_step, fast, max_dist, slow, slow, slow, 0],
        ),
    ]
}

/// Check if Port 999 64-word status response indicates Z-axis has been homed (`D5 == 0x8000_0001`).
pub fn decode_z_homed_from_poll(bytes: &[u8]) -> Option<bool> {
    if bytes.len() < 133 || bytes[0] != 0x01 || bytes[1] != 0x03 || bytes[2] != 0x80 {
        return None;
    }
    let received_crc = u16::from_le_bytes([bytes[131], bytes[132]]);
    if received_crc != modbus_crc16(&bytes[..131]) {
        return None;
    }
    let base = 3 + 5 * 4;
    let w0 = u16::from_be_bytes([bytes[base], bytes[base + 1]]) as u32;
    let w1 = u16::from_be_bytes([bytes[base + 2], bytes[base + 3]]) as u32;
    let d5 = (w1 << 16) | w0;
    Some(d5 == 0x8000_0001)
}

/// Decode `(z_pos_upper_limit_sensor, z_neg_collision_sensor)` from Port 999 64-word status response.
/// - `D2` bit 0 (`(d2 & 1) != 0`) is set while the Z+ upper limit sensor is triggered (and clears immediately on release).
/// - `D3` non-zero or capacitive spike (`D7 > 650_000`) indicates a Z- / touch-off alarm.
///   (`D4` latches after a Z+ limit hit and `D5 == 0x8000_0001` indicates Z-homed, so neither is treated as a sensor trigger.)
pub fn decode_z_sensors_from_poll(bytes: &[u8]) -> Option<(bool, bool)> {
    if bytes.len() < 133 || bytes[0] != 0x01 || bytes[1] != 0x03 || bytes[2] != 0x80 {
        return None;
    }
    let received_crc = u16::from_le_bytes([bytes[131], bytes[132]]);
    if received_crc != modbus_crc16(&bytes[..131]) {
        return None;
    }
    let read_dword_ws = |dword_idx: usize| -> u32 {
        let base = 3 + dword_idx * 4;
        let w0 = u16::from_be_bytes([bytes[base], bytes[base + 1]]) as u32;
        let w1 = u16::from_be_bytes([bytes[base + 2], bytes[base + 3]]) as u32;
        (w1 << 16) | w0
    };
    let d2 = read_dword_ws(2);
    let d3 = read_dword_ws(3);
    let d7 = read_dword_ws(7);
    let z_pos = (d2 & 0x0000_0001) != 0;
    let z_neg = d3 != 0 || d7 > 650_000;
    Some((z_pos, z_neg))
}

/// Decode a Port 999 64-word status polling response (`01 03 80 <128 bytes> <crc16_le>`).
/// Returns `Some((pos_z, raw_capacitance, state_code))` if valid.
/// Note: Hardware `D9` is 0 at the top limit and becomes negative as the head moves down
/// (or decreases when raw negative pulses raise the head), so we negate `D9` so `pos_z`
/// increases when `Z+` raises the head and decreases when `Z-` lowers the head.
pub fn decode_z_poll_response(bytes: &[u8]) -> Option<(i32, u32, u32)> {
    if bytes.len() < 133 || bytes[0] != 0x01 || bytes[1] != 0x03 || bytes[2] != 0x80 {
        return None;
    }
    let received_crc = u16::from_le_bytes([bytes[131], bytes[132]]);
    let computed_crc = modbus_crc16(&bytes[..131]);
    if received_crc != computed_crc {
        return None;
    }

    let read_dword_ws = |dword_idx: usize| -> u32 {
        let base = 3 + dword_idx * 4;
        let w0 = u16::from_be_bytes([bytes[base], bytes[base + 1]]) as u32;
        let w1 = u16::from_be_bytes([bytes[base + 2], bytes[base + 3]]) as u32;
        (w1 << 16) | w0
    };

    let state_code = read_dword_ws(6); // D6 (words 12..13): 9 when jogging, 0 when idle
    let raw_cap = read_dword_ws(7);    // D7 (words 14..15): capacitance sensor value
    let raw_d9 = read_dword_ws(9) as i32; // D9 (words 18..19): raw Z pulse coordinate
    let pos_z = raw_d9.saturating_neg();

    Some((pos_z, raw_cap, state_code))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_z_port999_packets_match_wireshark_capture() {
        assert_eq!(
            hex::encode(build_z_poll_packet()),
            "010303e80040c44a"
        );
        // Z+ (+1_000_000 logical pulses -> -1_000_000 hardware pulses to raise head)
        assert_eq!(
            hex::encode(build_z_jog_packet(1_000_000, 100)),
            "0110006500060c0009000000640000bdc0fff08f12"
        );
        // Z- (-1_000_000 logical pulses -> +1_000_000 hardware pulses to lower head)
        assert_eq!(
            hex::encode(build_z_jog_packet(-1_000_000, 100)),
            "0110006500060c00090000006400004240000fbf5e"
        );
        assert_eq!(
            hex::encode(build_z_stop_packet()),
            "01100065000204000100006478"
        );

        let hs = build_z_handshake_packets();
        assert_eq!(hex::encode(&hs[0]), "010303e80004c479");
        assert_eq!(hex::encode(&hs[1]), "010303e80040c44a");
        assert_eq!(hex::encode(&hs[2]), "010307d00050457b");
        assert_eq!(hex::encode(&hs[3]), "01030820005ac65b");
    }

    #[test]
    fn test_decode_z_poll_response_from_wireshark() {
        // Frame 3427 from sc2000-connection handshake and jog x,y and z.pcapng
        let raw = hex::decode(
            "0103800eb20000138200010000000000000000000000000000000000000000d862000561a800005170ffff5170ffff523fffff0000000000000000000000000000000000000000000000000001000000000000000000000000000000000000000000000000000000000000000000000000000049f00002d862000500480000000000001c0a"
        ).unwrap();
        let (pos_z, raw_cap, state_code) = decode_z_poll_response(&raw).expect("valid Z poll response");
        assert_eq!(pos_z, 44688);
        assert_eq!(raw_cap, 0x0005_D862);
        assert_eq!(state_code, 0);
    }

    #[test]
    fn test_decode_remote_control_buttons_from_r1042() {
        assert_eq!(decode_remote_jog(0), None);
        assert_eq!(decode_remote_jog(0x0000_0002), Some((Axis::X, -1)));
        assert_eq!(decode_remote_jog(0x0002_0000), Some((Axis::X, 1)));
        assert_eq!(decode_remote_jog(0x0000_0100), Some((Axis::Y, -1)));
        assert_eq!(decode_remote_jog(0x0000_0400), Some((Axis::Y, 1)));
        assert_eq!(decode_remote_jog(0x0001_0000), Some((Axis::Z, -1)));
        assert_eq!(decode_remote_jog(0x0004_0000), Some((Axis::Z, 1)));
        assert_eq!(format_remote_button(0x0000_0020), "Frame (0x20)");
        assert_eq!(format_remote_button(0x0100_0000), "Idle");
        assert_eq!(format_remote_button(0x0202_0000), "Jog X+ [Fast] (0x20000)");
    }

    #[test]
    fn test_home_packets_match_wireshark_capture() {
        assert_eq!(
            hex::encode(build_z_home_packet()),
            "01100065000204000200009478"
        );

        let mut seq = 0u16;
        let gen = || {
            let s = seq;
            seq += 1;
            s
        };

        // Bottom-Right (+X, -Y) => flags_mask = 275 (0x0113), matching SC2000 packet #1401
        let pkts = build_xy_home_packets(
            gen,
            HomeConfig {
                x_dir: 1,
                y_dir: -1,
                fast_speed_pls_s: 59_000,
                slow_speed_pls_s: 1_180,
                max_search_pulses: 590_000,
            },
        );
        assert_eq!(pkts.len(), 4);
        let home_pkt = &pkts[3];
        assert_eq!(home_pkt.addr, registers::REG_HOME_CMD);
        assert_eq!(
            home_pkt.data,
            vec![275, 1180, 236, 59_000, 590_000, 1180, 1180, 1180, 0]
        );

        // Other 3 corners
        let bl = build_xy_home_packets(|| 0, HomeConfig { x_dir: -1, y_dir: -1, fast_speed_pls_s: 1000, slow_speed_pls_s: 200, max_search_pulses: 10_000 });
        assert_eq!(bl[3].data[0], 259); // 0x0103
        let tl = build_xy_home_packets(|| 0, HomeConfig { x_dir: -1, y_dir: 1, fast_speed_pls_s: 1000, slow_speed_pls_s: 200, max_search_pulses: 10_000 });
        assert_eq!(tl[3].data[0], 291); // 0x0123
        let tr = build_xy_home_packets(|| 0, HomeConfig { x_dir: 1, y_dir: 1, fast_speed_pls_s: 1000, slow_speed_pls_s: 200, max_search_pulses: 10_000 });
        assert_eq!(tr[3].data[0], 307); // 0x0133
    }

    #[test]
    fn test_decode_z_sensors_from_poll() {
        let make_poll = |d2: u32, d3: u32, d4: u32, d5: u32, d7: u32| -> Vec<u8> {
            let mut buf = vec![0u8; 133];
            buf[0] = 0x01;
            buf[1] = 0x03;
            buf[2] = 0x80;
            let mut write_dw = |idx: usize, val: u32| {
                let base = 3 + idx * 4;
                let w0 = (val & 0xFFFF) as u16;
                let w1 = ((val >> 16) & 0xFFFF) as u16;
                buf[base..base + 2].copy_from_slice(&w0.to_be_bytes());
                buf[base + 2..base + 4].copy_from_slice(&w1.to_be_bytes());
            };
            write_dw(2, d2);
            write_dw(3, d3);
            write_dw(4, d4);
            write_dw(5, d5);
            write_dw(7, d7);
            let crc = modbus_crc16(&buf[..131]);
            buf[131..133].copy_from_slice(&crc.to_le_bytes());
            buf
        };

        // Open air, homed (D5=0x8000_0001, D7=383_162): neither Z+ nor Z- triggered
        assert_eq!(
            decode_z_sensors_from_poll(&make_poll(0, 0, 0, 0x8000_0001, 383_162)),
            Some((false, false))
        );
        // Z+ upper limit touched (D2=1, D4=1): Z+ is true, Z- stays false
        assert_eq!(
            decode_z_sensors_from_poll(&make_poll(1, 0, 1, 0x8000_0000, 383_359)),
            Some((true, false))
        );
        // Z+ released while D4 remains latched (D2=0, D4=1): both false
        assert_eq!(
            decode_z_sensors_from_poll(&make_poll(0, 0, 1, 0x8000_0000, 383_356)),
            Some((false, false))
        );
        // Z- capacitive spike / collision (D7 > 650_000 or D3 != 0): Z- is true
        assert_eq!(
            decode_z_sensors_from_poll(&make_poll(0, 0, 0, 0x8000_0001, 700_000)),
            Some((false, true))
        );
        assert_eq!(
            decode_z_sensors_from_poll(&make_poll(0, 1, 0, 0x8000_0001, 383_162)),
            Some((false, true))
        );
    }
}

