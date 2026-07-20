use crate::packet::RawPacket;
use crossbeam_channel::Receiver;
use nalgebra::Vector3;
use std::sync::{Arc, Mutex};

pub fn processor_thread(rx: Receiver<RawPacket>, samples: Arc<Mutex<Vec<Vector3<f32>>>>) {
    println!("[processor] Waiting for packets...");

    while let Ok(packet) = rx.recv() {
        let mag = Vector3::new(packet.floats[6], packet.floats[7], packet.floats[8]);

        if mag.norm() > 100.0 {
            continue;
        }

        if let Ok(mut buf) = samples.lock() {
            buf.push(mag);
        }

        println!("[processor] {}", packet);
    }

    println!("[processor] Channel closed, exiting");
}
