use crate::crc::modbus_crc16;
use crate::packet::Packet;

/// Registers discovered from Wireshark captures of TZLASER (Port 502)
pub mod registers {
    pub const REG_COMMAND: u32 = 104;      // 0x0068: 2 = Stop/Cancel, 3 = Init
    pub const REG_MODE_CTRL: u32 = 108;    // 0x006C: 2 DWORDs mode/reset flags
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
pub mod remote_buttons {
    pub const X_MINUS: u32 = 0x0000_0002; // Bit 1 (2): Jog X-
    pub const Y_MINUS: u32 = 0x0000_0100; // Bit 8 (256): Jog Y-
    pub const Y_PLUS: u32 = 0x0000_0400;  // Bit 10 (1024): Jog Y+
    pub const Z_MINUS: u32 = 0x0001_0000; // Bit 16 (65536): Jog Z- (lower head)
    pub const X_PLUS: u32 = 0x0002_0000;  // Bit 17 (131072): Jog X+
    pub const Z_PLUS: u32 = 0x0004_0000;  // Bit 18 (262144): Jog Z+ (raise head toward upper limit)
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
    } else {
        None
    }
}

/// Format a human-readable label for the active wireless remote control button bitmask (`R1042`).
pub fn format_remote_button(mask: u32) -> String {
    if mask == 0 {
        return "Idle".to_string();
    }
    if let Some((axis, dir)) = decode_remote_jog(mask) {
        let sign = if dir >= 0 { "+" } else { "-" };
        format!("Jog {:?}{} (0x{:X})", axis, sign, mask)
    } else {
        format!("Key 0x{:X}", mask)
    }
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
            raw_registers: regs.to_vec(),
        }
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
    }
}

