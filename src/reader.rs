use crate::packet::{RawPacket, HEADER, PACKET_SIZE};
use anyhow::{Context, Result};
use crossbeam_channel::Sender;
use std::io::Read;

enum ParseState {
    SearchingHeader,
    ReadingPayload { header_pos: usize },
}

pub fn reader_thread(port_path: &str, baud: u32, tx: Sender<RawPacket>) -> Result<()> {
    let port = serialport::new(port_path, baud)
        .timeout(std::time::Duration::from_millis(100))
        .open()
        .context("Failed to open serial port")?;

    println!("[reader] Port {} opened at {} baud", port_path, baud);

    let mut reader = port;
    let mut buf = [0u8; 1024];
    let mut state = ParseState::SearchingHeader;
    let mut packet_buf = [0u8; PACKET_SIZE];
    let mut packet_len: usize = 0;

    loop {
        let n = match reader.read(&mut buf) {
            Ok(n) => n,
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => {
                eprintln!("[reader] Read error: {}", e);
                std::thread::sleep(std::time::Duration::from_secs(1));
                continue;
            }
        };

        if n == 0 {
            continue;
        }

        let mut i = 0;
        while i < n {
            match &mut state {
                ParseState::SearchingHeader => {
                    if buf[i] == HEADER[0] {
                        packet_buf[0] = buf[i];
                        packet_len = 1;
                        state = ParseState::ReadingPayload { header_pos: 0 };
                    }
                }
                ParseState::ReadingPayload { header_pos } => {
                    packet_buf[packet_len] = buf[i];
                    packet_len += 1;

                    // Check header bytes as they come in
                    if packet_len <= 4 {
                        if buf[i] == HEADER[*header_pos + 1] {
                            *header_pos += 1;
                        } else {
                            // Header mismatch — reset and re-check this byte
                            state = ParseState::SearchingHeader;
                            packet_len = 0;
                            i -= 1; // re-process this byte
                            continue;
                        }
                    }

                    if packet_len == PACKET_SIZE {
                        let packet = RawPacket::parse(&packet_buf);
                        let _ = tx.send(packet);
                        state = ParseState::SearchingHeader;
                        packet_len = 0;
                    }
                }
            }
            i += 1;
        }
    }
}
