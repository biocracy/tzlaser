//! Reference protocol definitions & encoders disassembled from SC2000 (`NCModule.dll` / `NCModule.pdb`).
//!
//! Kept separate from `motion.rs` / `controller.rs` so the working implementation is untouched
//! and can be compared side-by-side.

use crate::packet::Packet;

/// Read-only motion card registers (`CROReg` in `NCModule.pdb`, `regs[0..50]` of the 144-DWORD poll at `1000`).
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McRoReg {
    ProductMark = 0,
    SoftwareVersion = 1,
    DIState = 2,
    ZeroState = 3,
    WarningState = 4,
    RunState = 5,
    AxisFifoRestNum = 6,
    AxisFifoState = 7,
    XPulseSum = 8,
    YPulseSum = 9,
    ZPulseSum = 10,
    WPulseSum = 11,
    XEncoder = 12,
    YEncoder = 13,
    ZEncoder = 14,
    WEncoder = 15,
    AxisFpgaFifoNum = 16,
    AxisFifoCapacity = 17,
    ADVal = 18,
    Date = 19,
    Time = 20,
    JoystickState = 21,
    FifoFrameIndex = 22,
    XStopPulse = 23,
    YStopPulse = 24,
    ZStopPulse = 25,
    WStopPulse = 26,
    EncoderFifoNum = 27,
    DOState = 28,
    FifoCurCmd = 29,
    LaserWarningState = 30,
    StopFifoNum = 31,
    XAxisVel = 32,
    YAxisVel = 33,
    ZAxisVel = 34,
    WAxisVel = 35,
    FlycutFifoRestNum = 36,
    FlycutEnableState = 37,
    FlycutFifoCapacity = 38,
    JoystickAddHigh4 = 39,
    JoystickAddMid4 = 40,
    JoystickAddLow4 = 41,
    JoystickKeyVal = 42,
    JoystickMatchKeyVal = 43,
    JoystickUsbAddHigh4 = 44,
    JoystickUsbAddLow4 = 45,
    CurrentContourIndex = 46,
    CurrentItemIndex = 47,
    ZSignalOriginPos = 48,
    ZSignalGoOriginState = 49,
}

impl McRoReg {
    pub const COUNT: usize = 50;

    /// Wire address when addressed individually (`1000 + 4 * index`).
    pub const fn wire_addr(self) -> u32 {
        1000 + 4 * (self as u32)
    }
}

/// Read-write motion card registers (`CRWReg` in `NCModule.pdb`, `regs[50..144]` of the 144-DWORD poll at `1000`).
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McRwReg {
    FifoControl = 0,
    IOOutputControl = 1,
    DA1Output = 2,
    DA2Output = 3,
    PWMFrequence = 4,
    PWMDutyCycle = 5,
    ZZeroPara = 6,
    DoubleDriverPara = 7,
    DoubleDriverTolerance = 8,
    DoubleDriverToleranceTime = 9,
    EnableFlag1Para = 10,
    EnableFlag2Para = 11,
    LimitInpTypePara = 12,
    XPositiveSoftLimit = 13,
    XNegativeSoftLimit = 14,
    YPositiveSoftLimit = 15,
    YNegativeSoftLimit = 16,
    ZPositiveSoftLimit = 17,
    ZNegativeSoftLimit = 18,
    WPositiveSoftLimit = 19,
    WNegativeSoftLimit = 20,
    ServoEnable = 21,
    EmergencyStopPort = 22,
    SoftEmergencyStopFlag = 23,
    SafeStopAcc = 24,
    SafeStopJerk = 25,
    AxisPara = 26,
    SafeAccFactor = 27,
    Timeout = 28,
    IPAddress = 29,
    SubnetMask = 30,
    DefaultGateway = 31,
    BaudRate = 32,
    LaserType = 33,
    DA1Adjust = 34,
    DA2Adjust = 35,
    PWMEnable = 36,
    FlycutModeTolerance = 37,
    ADAdjust = 38,
    HardwareVer = 39,
    JoystickRecAddHigh4 = 40,
    JoystickRecAddMid4 = 41,
    JoystickRecAddLow4 = 42,
    FTCSerialBaudRate = 43,
    DOMask = 44,
    JoystickUsbRecAddHigh4 = 45,
    JoystickUsbRecAddLow4 = 46,
    XPCSendSum = 47,
    YPCSendSum = 48,
    ZPCSendSum = 49,
    WPCSendSum = 50,
    FifoAlarmNum = 51,
    DOLockBit = 52,
    XPulseOffset = 53,
    YPulseOffset = 54,
    ZPulseOffset = 55,
    WPulseOffset = 56,
    DAControlFlag = 57,
    DAControlRange = 58,
    Rs485BaudRate = 59,
    ConfigFlag = 60,
    HardwareVerFlag = 61,
    EnablePWMFifo = 62,
    FifoMode = 63,
    DI1SmoothTime = 64,
    DI16SmoothTime = 79,
    Axis4Freq = 80,
    AxisInOutControlFlag = 81,
    EdgeSeekFlag = 82,
    EdgeSeekXPos = 83,
    EdgeSeekYPos = 84,
    EdgeSeekZPos = 85,
    EdgeSeekWPos = 86,
    DA3Output = 87,
    CurveFlag = 88,
    DA3Adjust = 89,
    XAxisZSignalPos = 90,
    YAxisZSignalPos = 91,
    ZAxisZSignalPos = 92,
    WAxisZSignalPos = 93,
}

impl McRwReg {
    pub const COUNT: usize = 94;

    /// Index into the 144-DWORD polling block (`50 + index`).
    pub const fn poll_index(self) -> usize {
        McRoReg::COUNT + (self as usize)
    }

    /// Wire address for `syncWriteReg` (`1200 + 4 * index`, e.g. `DOMask(44)` -> `1376 = 0x560`).
    pub const fn wire_addr(self) -> u32 {
        1000 + 4 * (McRoReg::COUNT as u32 + self as u32)
    }
}

/// Write-only motion card command registers (`CWOReg` in `NCModule.pdb`).
/// Wire address is `100 + 4 * index`.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McWoReg {
    WriteFifo = 0,         // 100 (0x64)
    FifoControl = 1,       // 104 (0x68)
    IOAdvControl = 2,      // 108 (0x6C)
    Zero = 3,              // 112 (0x70)
    Jog = 4,               // 116 (0x74)
    Stop = 5,              // 120 (0x78)
    StdControl = 6,        // 124 (0x7C)
    AdvControl = 7,        // 128 (0x80)
    ReadEncoder = 8,       // 132 (0x84)
    WriteFlycutFifo = 9,   // 136 (0x88)
    FTCMode = 10,          // 140 (0x8C)
    CardRunInfo = 11,      // 144 (0x90)
    WriteEncryptVal = 18,  // 172 (0xAC)
    ReadEncryptReturn = 19,// 176 (0xB0)
    EncryptDataSwitch = 20,// 180 (0xB4)
    EncryptID = 21,        // 184 (0xB8)
    DateTime = 22,         // 188 (0xBC)
    UserEncryptData = 23,  // 192 (0xC0)
    AdminEncryptData = 24, // 196 (0xC4)
    ParamData = 25,        // 200 (0xC8)
}

impl McWoReg {
    pub const fn wire_addr(self) -> u32 {
        100 + 4 * (self as u32)
    }
}

/// Interpolation FIFO item command (`CInterpItemCmd` in `NCModule.pdb`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterpItemCmd {
    None = 0,
    DevCmd = 1,
    ContourStart = 3,
    OpenSectionDrill = 4,
    OpenGradualDrill = 5,
    AFPos2 = 6,
    ZFFollow = 7,
    CloseFollow = 8,
    ResetFollow = 9,
    FixHeight = 10,
    NoFollow = 11,
    OpenFlycut = 12,
    CloseFlycut = 13,
    MotionEnd = 14,
    StartFlycut = 15,
    StopFlycut = 16,
    AdvFixHeight = 17,
}

/// On-Board Z-Follower commands (`COnBZFCommand` in `NCModule.pdb`, Port 999).
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OnBzfCommand {
    Stop = 1,
    GoOrigin = 2,
    Pos2Abs = 3,
    Follow = 4,
    Drill = 5,
    FrogJump = 6,
    Calib = 7,
    Pos2FollowRel = 8,
    Pos2Rel = 9,
    AdvFrogJump = 10,
    ServoCalib = 11,
    DrillFrogJump = 13,
    AdvDrillFrogJump = 14,
    SoftLimitSelect = 15,
    EdgeSeek = 17,
    AdvCmd = 18,
}

/// Mirror of SC2000's 20-byte `CInterpItem` struct (`NCModule.pdb` type `0x1BE6`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterpItem {
    pub axis: [i16; 4],
    pub do_mask: u16,
    pub pwm: u8,
    pub freq: u16,
    pub cmd: InterpItemCmd,
    pub misc_data: i32,
}

impl Default for InterpItem {
    fn default() -> Self {
        Self {
            axis: [0; 4],
            do_mask: 0,
            pwm: 0,
            freq: 0,
            cmd: InterpItemCmd::None,
            misc_data: 0,
        }
    }
}

/// Encode a single `InterpItem` into `(word0, word1)` (plus optional frequency-change `(w0, w1)`)
/// matching `CVirtualMachine::fillFifo` (`NCModule.dll` @ `0x1004e480`).
pub fn encode_interp_item(item: &InterpItem, prev_freq: &mut u16) -> Vec<u32> {
    let mut out = Vec::with_capacity(4);
    match item.cmd {
        InterpItemCmd::None => {
            let mut dir_nibble: u32 = 0;
            for (i, &delta) in item.axis.iter().enumerate() {
                if delta < 0 {
                    dir_nibble |= 1 << i;
                }
            }
            let dx = item.axis[0].unsigned_abs() as u32 & 0xFF;
            let dy = item.axis[1].unsigned_abs() as u32 & 0xFF;
            let dz = item.axis[2].unsigned_abs() as u32 & 0xFF;
            let dw = item.axis[3].unsigned_abs() as u32 & 0xFF;

            let word0 = (dir_nibble << 24) | ((item.pwm as u32) << 16) | (dx << 8) | dy;
            let word1 = ((item.do_mask as u32) << 16) | (dz << 8) | dw;
            out.push(word0);
            out.push(word1);

            if item.freq != *prev_freq {
                let freq_word = if item.freq == 0 {
                    0xF424
                } else {
                    0x8000 | (item.freq as u32 & 0x7FFF)
                };
                out.push(0x1000_0000);
                out.push(freq_word);
                *prev_freq = item.freq;
            }
        }
        InterpItemCmd::DevCmd => {
            let hi = (0x1000 | (item.axis[0] as u32 & 0x0FFF)) << 16;
            let lo = item.axis[1] as u16 as u32;
            out.push(hi | lo);
            out.push(item.misc_data as u32);
        }
        InterpItemCmd::ContourStart => {
            out.push(0x3000_0000);
            out.push(item.misc_data as u32);
        }
        InterpItemCmd::OpenSectionDrill
        | InterpItemCmd::OpenGradualDrill
        | InterpItemCmd::AFPos2
        | InterpItemCmd::ZFFollow
        | InterpItemCmd::CloseFollow
        | InterpItemCmd::ResetFollow
        | InterpItemCmd::FixHeight
        | InterpItemCmd::NoFollow
        | InterpItemCmd::OpenFlycut
        | InterpItemCmd::CloseFlycut => {
            out.push((item.cmd as u32) << 28);
            out.push(item.misc_data as u32);
        }
        InterpItemCmd::MotionEnd => {
            out.push(0xF000_0000);
            out.push(item.misc_data as u32);
        }
        InterpItemCmd::StartFlycut => {
            out.push(0x2000_0000);
            out.push(item.pwm as u32);
        }
        InterpItemCmd::StopFlycut => {
            out.push(0x2000_0000);
            out.push(0);
        }
        InterpItemCmd::AdvFixHeight => {
            out.push(0xA000_0000);
            out.push(item.misc_data as u32);
        }
    }
    out
}

/// Build a `WriteRegisters(100, ...)` FIFO frame matching `CVirtualMachine::fillFifo`.
pub fn build_sc2000_fifo_frame(seq: u16, frame_seq: u32, items: &[InterpItem]) -> Packet {
    let mut prev_freq = 0u16;
    let mut payload = Vec::with_capacity(1 + items.len() * 2);
    payload.push(frame_seq);
    for item in items {
        payload.extend(encode_interp_item(item, &mut prev_freq));
    }
    Packet::write_request(seq, McWoReg::WriteFifo.wire_addr(), payload)
}

/// Build a `setDO(port_1_based, state)` packet matching `CVirtualMachine::setDO` (`0x1003e660`).
pub fn build_set_do_packet(seq: u16, port_1_based: u8, state: bool) -> Packet {
    let bit = 1u32 << (port_1_based.saturating_sub(1) as u32);
    let val = if state { bit } else { 0 };
    Packet::write_request(seq, McWoReg::IOAdvControl.wire_addr(), vec![bit, val])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_addresses_match_disassembly() {
        assert_eq!(McWoReg::WriteFifo.wire_addr(), 100);
        assert_eq!(McWoReg::FifoControl.wire_addr(), 104);
        assert_eq!(McWoReg::IOAdvControl.wire_addr(), 108);
        assert_eq!(McWoReg::Zero.wire_addr(), 112);
        assert_eq!(McWoReg::Jog.wire_addr(), 116);
        assert_eq!(McWoReg::Stop.wire_addr(), 120);
        assert_eq!(McWoReg::StdControl.wire_addr(), 124);
        assert_eq!(McRwReg::DOMask.wire_addr(), 0x560);
    }

    #[test]
    fn interp_item_none_encoding() {
        let mut prev_freq = 0;
        let item = InterpItem {
            axis: [-5, 10, 0, 0],
            do_mask: 0x0001,
            pwm: 80,
            freq: 5000,
            cmd: InterpItemCmd::None,
            misc_data: 0,
        };
        let words = encode_interp_item(&item, &mut prev_freq);
        assert_eq!(words.len(), 4);
        assert_eq!(words[0], (0x1 << 24) | (80 << 16) | (5 << 8) | 10);
        assert_eq!(words[1], 0x0001_0000);
        assert_eq!(words[2], 0x1000_0000);
        assert_eq!(words[3], 0x8000 | 5000);
    }
}
