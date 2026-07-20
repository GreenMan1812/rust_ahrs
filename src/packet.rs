use byteorder::{ByteOrder, LittleEndian};
use std::fmt;

pub const HEADER: [u8; 4] = *b"####";
pub const PACKET_SIZE: usize = 44;
#[derive(Debug, Clone)]
pub struct RawPacket {
    pub floats: [f32; 9],
    pub uint_val: u32,
}

impl RawPacket {
    pub fn parse(buf: &[u8; PACKET_SIZE]) -> Self {
        assert_eq!(&buf[0..4], &HEADER);

        let mut floats = [0.0f32; 9];
        for i in 0..9 {
            floats[i] = LittleEndian::read_f32(&buf[4 + i * 4..8 + i * 4]);
        }

        let uint_val = LittleEndian::read_u32(&buf[40..44]);

        RawPacket { floats, uint_val }
    }
}

impl fmt::Display for RawPacket {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Packet {{")?;
        for (i, v) in self.floats.iter().enumerate() {
            write!(f, " f{}={:.4}", i, v)?;
        }
        write!(f, " | uint={}", self.uint_val)?;
        write!(f, " }}")
    }
}
