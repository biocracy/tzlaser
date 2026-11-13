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
#[derive(Debug, Clone, Default)]
pub struct MachineStatus {
    pub pos_x: i32,
    pub pos_y: i32,
    pub pos_z: i32,
    pub pos_w: i32,
    pub is_connected: bool,
    pub raw_registers: Vec<u32>,
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

        Self {
            pos_x,
            pos_y,
            pos_z,
            pos_w,
            is_connected: true,
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

