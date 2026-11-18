pub mod controller;
pub mod crc;
pub mod motion;
pub mod packet;
pub mod sc2000_ref;

pub use controller::{Command, LaserController};
pub use crc::modbus_crc16;
pub use motion::{
    build_live_cut_setup_packets, build_live_cut_teardown_packets, build_z_calibrate_packet,
    build_z_follow_packet, build_z_retract_packet, decode_z_poll_telemetry, fifo_flags,
    format_remote_button, machine_directions, remote_buttons, Axis, AxisSignalSample,
    CalibrationMonitor, HomeConfig, JogConfig, LiveCutContour, MachineStatus, SignalTelemetry,
    ZPollTelemetry,
};
pub use packet::{Opcode, Packet, PacketError};

