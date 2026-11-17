pub mod controller;
pub mod crc;
pub mod motion;
pub mod packet;

pub use controller::{Command, LaserController};
pub use crc::modbus_crc16;
pub use motion::{
    build_z_calibrate_packet, decode_z_poll_telemetry, fifo_flags, format_remote_button,
    machine_directions, remote_buttons, Axis, AxisSignalSample, CalibrationMonitor, HomeConfig,
    JogConfig, MachineStatus, SignalTelemetry, ZPollTelemetry,
};
pub use packet::{Opcode, Packet, PacketError};

