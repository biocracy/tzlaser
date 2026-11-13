//! Modbus CRC-16 implementation as used in the TZLASER UDP protocol.
//!
//! Verified against captures from tzlaser-firstWire.pcapng:
//! Polynomial: 0x8005 (reversed 0xA001), Initial: 0xFFFF, Reflected.

/// Calculate the 16-bit Modbus CRC for the given data slice.
pub fn modbus_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            if (crc & 0x0001) != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc_against_wireshark_packets() {
        // Sample 1: Polling packet read 144 registers at 1000
        // Expected big-endian bytes: [0xec, 0x18] -> 0xec18
        let s1 = hex::decode("00080030e80300009000").unwrap();
        assert_eq!(modbus_crc16(&s1), 0xec18);

        // Sample 2: Write jog stop command to reg 104
        // Expected big-endian bytes: [0x31, 0x6d] -> 0x316d
        let s2 = hex::decode("000c004068000000010002000000").unwrap();
        assert_eq!(modbus_crc16(&s2), 0x316d);

        // Sample 3: Write jog velocity config to reg 120
        // Expected big-endian bytes: [0x5a, 0xa4] -> 0x5aa4
        let s3 = hex::decode("00100040780000000200f0cc010060011200").unwrap();
        assert_eq!(modbus_crc16(&s3), 0x5aa4);

        // Sample 4: Jog left (-472000 pulses) to reg 116
        // Expected big-endian bytes: [0x39, 0x4c] -> 0x394c
        let s4 = hex::decode("002400407400000007000c1700000c17000078e6000040ccf8ff000000000000000000000000").unwrap();
        assert_eq!(modbus_crc16(&s4), 0x394c);

        // Sample 5: Jog right (+472000 pulses) to reg 116
        // Expected big-endian bytes: [0xd8, 0x39] -> 0xd839
        let s5 = hex::decode("002400407400000007000c1700000c17000078e60000c0330700000000000000000000000000").unwrap();
        assert_eq!(modbus_crc16(&s5), 0xd839);
    }
}

