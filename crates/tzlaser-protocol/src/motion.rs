use crate::packet::Packet;

/// Registers discovered from Wireshark captures of TZLASER
pub mod registers {
    pub const REG_COMMAND: u32 = 104;      // 0x0068: 2 = Stop/Cancel, 3 = Init
    pub const REG_JOG_MOTION: u32 = 116;   // 0x0074: 7 DWORDs [vmin, vjog, accel, dx, dy, dz, dw]
    pub const REG_JOG_RAMP: u32 = 120;     // 0x0078: 2 DWORDs [118000, 1180000]
    pub const REG_ENABLE_FLAG: u32 = 124;  // 0x007C: 1 DWORD [1]
    pub const REG_STATUS_BASE: u32 = 1000; // 0x03E8: 144 DWORDs polling block
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
    pub min_speed: u32,  // pulses/sec (e.g. 5,900)
    pub jog_speed: u32,  // pulses/sec (e.g. 5,900)
    pub accel: u32,      // pulses/sec^2 (e.g. 59,000)
}

impl Default for JogConfig {
    fn default() -> Self {
        Self {
            min_speed: 5900,
            jog_speed: 5900,
            accel: 59000,
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
            raw_registers: Vec::new(),
        }
    }
}

impl MachineStatus {
    pub fn from_status_registers(regs: &[u32]) -> Self {
        if regs.len() < 24 {
            return Self::default();
        }

        // From Wireshark trace: R1008 and R1012 track live positions
        let pos_x = regs.get(8).copied().unwrap_or(0) as i32;
        let pos_y = regs.get(12).copied().unwrap_or(0) as i32;
        let pos_z = regs.get(16).copied().unwrap_or(0) as i32;
        let pos_w = regs.get(23).copied().unwrap_or(0) as i32;

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
        let capacitive_height_mm = ((pos_z.max(0) as f32) / 1000.0).clamp(0.2, 50.0);
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
            raw_registers: regs.to_vec(),
        }
    }
}

/// Build packets for Jogging an axis by a given number of pulses.
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

/// Build packets to stop motion.
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

