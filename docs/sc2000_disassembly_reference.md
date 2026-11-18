# SC2000 (`NCModule.dll` / `NCModule.pdb`) Disassembly Protocol Reference

Extracted from `org/SC2000_v1.0.392.13/Module/NCModule.dll` and `NCModule.pdb` (v1.0.392.13, built 2022-07-30), plus `MotionCtrl.dll` and `File/ipAdd.ini`.

This document is a **read-only reference** for comparing against and extending `crates/tzlaser-protocol` without altering the working codebase.

---

## 1. Network Endpoints (`File/ipAdd.ini`)

| Subsystem | Default IP | Port | Notes |
| --- | --- | --- | --- |
| Motion Card (`CMCHalAPI`) | `10.1.1.168` | `502` | UDP, `CExtModbus` (`cmtExt = 0`) |
| External Z-Follower (`CZFNetHalAPI`) | `10.1.1.169` | `502` | UDP, `CExtModbus` (`cmtZF = 1`) / `ZFExtModbus` (`cmtZFExt = 4`) |
| On-Board Z-Follower (`OnBZF`) | `10.1.1.168` | `999` | UDP (`cmtOnBZF = 3`) |
| On-Board Laser (`OnBLaser`) | `10.1.1.168` | `888` | UDP |
| Laser Source (`CLaserNetHalAPI`) | `10.1.1.170` | `10001` | `CIPGProtocolAdapter` / `CRaycusProtocolAdapter` |
| AutoFocus (`CAFNetHalAPI`) | `10.1.1.168` | `888` (Adv `666`) | `cmtAutoFocus = 9`, `cmtAFUpdate = 10` |
| Extended Controller (`CECNetHalAPI`) | `10.1.1.168` | `888` (Adv `666`) | `cmtEC = 11`, `cmtECUpdate = 12` |
| EC3710 | `10.1.1.170` | `502` | |

Timing & FIFO limits from `ipAdd.ini`:
- `MaxItemPerFrame = 60`
- `MaxFillItem = 2000`
- `MCFifoTime = 1600`
- `MCTimeout = 500`, `MCMaxSendTime = 2`, `MCMaxRecvTime = 3`, `MCSendInterval = 1`
- `FifoTimeout = 600`

---

## 2. `CExtModbus::encode` Wire Framing (Port 502 UDP)

Disassembled from `NCModule.dll` at `0x1002b160` (`CExtModbus::encode`):

```text
Offset  Size  Endian  Field
0       2     BE      seq (incrementing u16)
2       2     BE      proto_id (0x0000)
4       2     BE      length (remaining bytes before CRC)
6       1     -       unit_id (0x00)
7       1     -       opcode (0x20, 0x21, 0x26, 0x30, 0x40)
8       4     LE      addr (u32)
12      2     LE      count (u16)
14..    4*N   LE      data[0..N] (u32, for write opcodes 0x40 / 0x20 / 0x21 / 0x26)
end     2     LE      Modbus RTU CRC-16 (Modbus_RtuCRC at 0x1002b000)
```

Supported opcodes in `CExtModbus::encode` jump table (`0x1002b67c`):
- `0x30`: Read Registers (`count` DWORDs starting at `addr`) — matches `tzlaser_protocol::Opcode::ReadRegisters`
- `0x40`: Write Registers (`count` DWORDs starting at `addr`) — matches `tzlaser_protocol::Opcode::WriteRegisters`
- `0x20`, `0x21`, `0x26`: Extended/firmware/block transfer opcodes (same header layout, different length calculation)

---

## 3. Motion Card Register Map (`CROReg`, `CRWReg`, `CWOReg`)

### 3.1 Status Polling Block (`addr = 1000`, `count = 144` DWORDs)

Disassembled from `CVirtualMachine::updateMCStatus` (`0x1004ab70`):
- Issues `0x30` (ReadRegisters) at `addr = 1000 (0x3E8)` with `count = 144 (0x90)`.
- Response contains 144 `u32` values (`0x240` bytes of payload):
  - `regs[0..50]` (`CROReg[0..49]`): Read-Only status registers.
  - `regs[50..144]` (`CRWReg[0..93]`): Read-Write configuration/state registers.
- When writing a single `CRWReg[i]` via `CVirtualMachine::syncWriteReg`, the wire address is **`1000 + 4 * (50 + i) = 1200 + 4 * i`** (verified in `CVirtualMachine::setDO` at `0x1003e6bb`, which writes `crwrDOMask = 44` at `0x560 = 1376 = 1200 + 4 * 44`).

#### `CROReg` (`regs[0..49]`, wire base `1000 + 4 * i` if addressed individually)

| Index | Enum Name | Current `tzlaser-protocol` |
| --- | --- | --- |
| 0 | `crorProductMark` | |
| 1 | `crorSoftwareVersion` | |
| 2 | `crorDIState` | DI sensors |
| 3 | `crorZeroState` | Homing state |
| 4 | `crorWarningState` | Alarm/warning bits |
| 5 | `crorRunState` | Run state |
| 6 | `crorAxisFifoRestNum` | FIFO free space |
| 7 | `crorAxisFifoState` | FIFO state |
| 8..11 | `crorXPusleSum` .. `crorWPusleSum` | `X/Y/Z/W` pulse counters |
| 12..15 | `crorXEncoder` .. `crorWEncoder` | `X/Y/Z/W` encoder positions |
| 16 | `crorAxisFPGAFifoNum` | FPGA FIFO count |
| 17 | `crorAxisFifoCapcity` | FIFO capacity |
| 18 | `crorADVal` | AD value |
| 19..20 | `crorDate`, `crorTime` | RTC |
| 21 | `crorJoystickState` | |
| 22 | `crorFifoFrameIndex` | FIFO frame index |
| 23..26 | `crorXStopPulse` .. `crorWStopPulse` | Stop pulses |
| 27 | `crorEncoderFifoNum` | |
| 28 | `crorDOState` | DO outputs |
| 29 | `crorFifoCurCmd` | Current FIFO command |
| 30 | `crorLaserWarningState` | Laser warning state |
| 31 | `crorStopFifoNum` | |
| 32..35 | `crorXAxisVel` .. `crorWAxisVel` | Axis velocities |
| 36..38 | `crorFlycutFifoRestNum`, `crorFlycutEnableState`, `crorFlycutFifoCapcity` | Flycut FIFO |
| 39..41 | `crorJoystickAddHigh4/Mid4/Low4` | Wireless remote ID |
| 42 | `crorJoystickKeyVal` | `R1042` remote button bitmask |
| 43 | `crorJoystickMatchKeyVal` | `R1043` matched remote key |
| 44..45 | `crorJoystickUsbAddHigh4/Low4` | USB remote ID |
| 46..47 | `crorCurrentContourIndex`, `crorCurrentItemIndex` | Active contour/item |
| 48..49 | `crorZSignalOriginPos`, `crorZSignalGoOriginState` | Z-signal origin |

### 3.2 Write-Only Command Registers (`CWOReg`, wire address = `100 + 4 * index`)

| Index | Wire Addr | Hex | Enum Name | Current `tzlaser-protocol` |
| --- | --- | --- | --- | --- |
| 0 | `100` | `0x64` | `cworWriteFifo` | `REG_FIFO_DATA` |
| 1 | `104` | `0x68` | `cworFifoControl` | `REG_COMMAND` |
| 2 | `108` | `0x6C` | `cworIOAdvControl` | `REG_MODE_CTRL` (also used by `setDO`) |
| 3 | `112` | `0x70` | `cworZero` | `REG_HOME_CMD` |
| 4 | `116` | `0x74` | `cworJog` | `REG_JOG_MOTION` |
| 5 | `120` | `0x78` | `cworStop` | `REG_JOG_RAMP` |
| 6 | `124` | `0x7C` | `cworStdControl` | `REG_ENABLE_FLAG` |
| 7 | `128` | `0x80` | `cworAdvControl` | — |
| 8 | `132` | `0x84` | `cworReadEncoder` | — |
| 9 | `136` | `0x88` | `cworWriteFlycutFifo` | — |
| 10 | `140` | `0x8C` | `cworFTCMode` | — |
| 11 | `144` | `0x90` | `cworCardRunInfo` | — |
| 18..25 | `172..200` | `0xAC..0xC8` | `cworWriteEncryptVal` .. `cworParamData` | — |

---

## 4. Motion FIFO Tick Encoding (`CVirtualMachine::fillFifo`, `0x1004e480`)

Each `CInterpItem` (20 bytes: `Axis[4]: [i16; 4]`, `DO: u16`, `PWM: u8`, `Freq: u16`, `Cmd: u8`, `MiscData: i32`) is packed into `u32` words and sent to `addr = 100` (`cworWriteFifo`) in frames of up to `MaxItemPerFrame = 60` items:

- Frame header (prepended before items): `[word_count, frame_seq]` (written as `WriteRegisters(100, [word_count, frame_seq, ...items])`, verified at `0x1004e562` where `addr = 0x64` and two `0` placeholders are pushed).
- **`Cmd == 0` (`iicNone`, normal motion tick, 2 `u32` words)**:
  - `word0 = (dir_nibble << 24) | ((PWM as u32) << 16) | ((|dx| & 0xFF) << 8) | (|dy| & 0xFF)`
    - `dir_nibble`: bit 0 = X negative, bit 1 = Y negative, bit 2 = Z negative, bit 3 = W negative.
  - `word1 = ((DO as u32) << 16) | ((|dz| & 0xFF) << 8) | (|dw| & 0xFF)`
  - If `Freq` changed from the previous tick, an extra 2-word item is emitted:
    - `word0 = 0x1000_0000`, `word1 = 0x8000 | (Freq & 0x7FFF)` (or `0xF424` = `62500` when `Freq == 0`).
- **Control / Laser / Follow Commands (`Cmd != 0`, 2 `u32` words)**:
  - `Cmd == 1` (`iicDevCmd`): `word0 = ((0x1000 | (Axis[0] & 0xFFF)) << 16) | (Axis[1] as u16)`, `word1 = MiscData`
  - `Cmd == 3` (`iicContourStart`): `word0 = 0x3000_0000`, `word1 = MiscData`
  - `Cmd == 4..=13` (`iicOpenSectionDrill..iicCloseFlycut`): `word0 = (Cmd as u32) << 28`, `word1 = MiscData`
  - `Cmd == 14` (`iicMotionEnd`): `word0 = 0xF000_0000`, `word1 = MiscData`
  - `Cmd == 15` (`iicStartFlycut`): `word0 = 0x2000_0000`, `word1 = PWM as u32`
  - `Cmd == 16` (`iicStopFlycut`): `word0 = 0x2000_0000`, `word1 = 0`
  - `Cmd == 17` (`iicAdvFixHeight`): `word0 = 0xA000_0000`, `word1 = MiscData`

---

## 5. Subsystems Summary (from `NCModule.pdb`)

- **Z-Follower (`CZFROReg` / `CZFRWReg` / `CZFWOReg` / `CZFCommand` & `COnBZFCommand`)**:
  - Port `502` (`CZFNetHalAPI` at `10.1.1.169`) or Port `999` (`OnBZF` at `10.1.1.168`).
  - `COnBZFCommand`: `Stop=1`, `GoOrigin=2`, `Pos2Abs=3`, `Follow=4`, `Drill=5`, `FrogJump=6`, `Calib=7`, `Pos2FollowRel=8`, `Pos2Rel=9`, `AdvFrogJump=10`, `ServoCalib=11`, `DrillFrogJump=13`, `AdvDrillFrogJump=14`, `SoftLimitSelect=15`, `EdgeSeek=17`, `AdvCmd=18`.
- **Laser Source (`CRaycusProtocolAdapter` / `CIPGProtocolAdapter`)**:
  - `SetPWMMode(freq, duty, ...)` validates `50 <= freq <= 50000` (`0x32..=0xC350`), `duty <= 100`, `power <= 100` (`0x1002a610`).
- **AutoFocus (`CAutoFocusROReg/RW/WO/Command`)** and **Extended Controller (`CECROReg/RW/WO/Command`)**.
- **Motion Planner (`MotionCtrl.dll`)**:
  - Exports: `arcInterp`, `segInterp`, `segInterp_time`, `newContourSmooth`, `newVelocityPlanning`.
