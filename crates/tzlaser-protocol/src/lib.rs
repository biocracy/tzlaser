pub mod controller;
pub mod crc;
pub mod motion;
pub mod packet;

pub use controller::{Command, LaserController};
pub use crc::modbus_crc16;
pub use motion::{
    format_remote_button, remote_buttons, Axis, AxisSignalSample, HomeConfig, JogConfig,
    MachineStatus, SignalTelemetry,
};
pub use packet::{Opcode, Packet, PacketError};

