use byteorder::{BigEndian, LittleEndian, ReadBytesExt, WriteBytesExt};
use std::io::Cursor;
use thiserror::Error;

use crate::crc::modbus_crc16;

#[derive(Error, Debug)]
pub enum PacketError {
    #[error("Packet too short: {0} bytes (minimum is 14)")]
    TooShort(usize),
    #[error("Length mismatch: header reports {expected} payload bytes, but packet has {actual}")]
    LengthMismatch { expected: usize, actual: usize },
    #[error("CRC mismatch: received 0x{received:04X}, computed 0x{computed:04X}")]
    CrcMismatch { received: u16, computed: u16 },
    #[error("Unknown opcode: 0x{0:04X}")]
    UnknownOpcode(u16),
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Opcode {
    ReadRegisters = 0x0030,
    WriteRegisters = 0x0040,
}

impl TryFrom<u16> for Opcode {
    type Error = PacketError;

    fn try_from(val: u16) -> Result<Self, Self::Error> {
        match val {
            0x0030 => Ok(Opcode::ReadRegisters),
            0x0040 => Ok(Opcode::WriteRegisters),
            other => Err(PacketError::UnknownOpcode(other)),
        }
    }
}

/// A decoded TZLASER Port 502 command or response packet.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet {
    pub seq: u16,
    pub opcode: Opcode,
    pub addr: u32,
    pub count: u16,
    pub data: Vec<u32>,
}

impl Packet {
    /// Create a Read Registers request (e.g., polling status at address 1000).
    pub fn read_request(seq: u16, addr: u32, count: u16) -> Self {
        Self {
            seq,
            opcode: Opcode::ReadRegisters,
            addr,
            count,
            data: Vec::new(),
        }
    }

    /// Create a Write Registers request (e.g., jogging or setting parameters).
    pub fn write_request(seq: u16, addr: u32, data: Vec<u32>) -> Self {
        Self {
            seq,
            opcode: Opcode::WriteRegisters,
            addr,
            count: data.len() as u16,
            data,
        }
    }

    /// Encode packet to raw bytes ready to send over UDP.
    pub fn encode(&self) -> Vec<u8> {
        // Build payload starting from offset 6 (Opcode)
        let mut payload = Vec::new();
        payload.write_u16::<BigEndian>(self.opcode as u16).unwrap();
        payload.write_u32::<LittleEndian>(self.addr).unwrap();
        payload.write_u16::<LittleEndian>(self.count).unwrap();

        for &val in &self.data {
            payload.write_u32::<LittleEndian>(val).unwrap();
        }

        let payload_len = payload.len() as u16;

        // Calculate CRC over [Length (2 bytes) + Payload]
        let mut crc_data = Vec::with_capacity(2 + payload.len());
        crc_data.write_u16::<BigEndian>(payload_len).unwrap();
        crc_data.extend_from_slice(&payload);
        let crc = modbus_crc16(&crc_data);

        // Assemble final packet
        let mut packet = Vec::with_capacity(6 + payload.len());
        packet.write_u16::<BigEndian>(self.seq).unwrap();
        packet.write_u16::<BigEndian>(crc).unwrap();
        packet.write_u16::<BigEndian>(payload_len).unwrap();
        packet.extend_from_slice(&payload);

        packet
    }

    /// Decode raw UDP packet from the wire.
    pub fn decode(bytes: &[u8]) -> Result<Self, PacketError> {
        if bytes.len() < 14 {
            return Err(PacketError::TooShort(bytes.len()));
        }

        let mut r = Cursor::new(bytes);
        let seq = r.read_u16::<BigEndian>()?;
        let received_crc = r.read_u16::<BigEndian>()?;
        let payload_len = r.read_u16::<BigEndian>()? as usize;

        if bytes.len() - 6 != payload_len {
            return Err(PacketError::LengthMismatch {
                expected: payload_len,
                actual: bytes.len() - 6,
            });
        }

        // Verify CRC over bytes[4..end]
        // Note: PC -> Board requests encode CRC in Big-Endian, while Board -> PC responses
        // encode the same Modbus CRC-16 in Little-Endian (byte-swapped).
        let computed_crc = modbus_crc16(&bytes[4..]);
        if received_crc != computed_crc && received_crc != computed_crc.swap_bytes() {
            return Err(PacketError::CrcMismatch {
                received: received_crc,
                computed: computed_crc,
            });
        }

        let opcode_raw = r.read_u16::<BigEndian>()?;
        let opcode = Opcode::try_from(opcode_raw)?;
        let addr = r.read_u32::<LittleEndian>()?;
        let count = r.read_u16::<LittleEndian>()?;

        let mut data = Vec::with_capacity(count as usize);
        let remaining_bytes = bytes.len() - 14;
        let dwords_available = remaining_bytes / 4;
        let to_read = (count as usize).min(dwords_available);

        for _ in 0..to_read {
            data.push(r.read_u32::<LittleEndian>()?);
        }

        Ok(Self {
            seq,
            opcode,
            addr,
            count,
            data,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_packet_decode_and_encode_against_wireshark() {
        // Wireshark packet: Jog left (-472000 pulses) to reg 116 (0x0074), seq 1547 (0x060b)
        let raw = hex::decode("060b394c002400407400000007000c1700000c17000078e6000040ccf8ff000000000000000000000000").unwrap();
        let pkt = Packet::decode(&raw).expect("Failed to decode wireshark packet");

        assert_eq!(pkt.seq, 1547);
        assert_eq!(pkt.opcode, Opcode::WriteRegisters);
        assert_eq!(pkt.addr, 116);
        assert_eq!(pkt.count, 7);
        assert_eq!(pkt.data[0], 5900);
        assert_eq!(pkt.data[1], 5900);
        assert_eq!(pkt.data[2], 59000);
        assert_eq!(pkt.data[3] as i32, -472000);

        // Re-encoding must produce the exact identical bytes on the wire
        let re_encoded = pkt.encode();
        assert_eq!(raw, re_encoded);
    }

    #[test]
    fn test_decode_board_response_with_little_endian_crc() {
        // Live board response to ReadRegisters(1000, 2) with Little-Endian CRC (0x21CF on wire for 0xCF21)
        let raw = hex::decode("000121cf00100030e803000002003523e5ac1bdb0000").unwrap();
        let pkt = Packet::decode(&raw).expect("Failed to decode board response");
        assert_eq!(pkt.seq, 1);
        assert_eq!(pkt.opcode, Opcode::ReadRegisters);
        assert_eq!(pkt.addr, 1000);
        assert_eq!(pkt.count, 2);
        assert_eq!(pkt.data.len(), 2);
    }
}

