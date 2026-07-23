mod calibrator;
mod packet;
mod processor;
mod reader;
mod visualizer;

use anyhow::Result;
use clap::Parser;
use nalgebra::Vector3;
use std::sync::{Arc, Mutex};

#[derive(Parser)]
#[command(name = "ahrs-reader")]
#[command(about = "AHRS serial port data reader")]
struct Args {
    /// Serial port path (e.g., /dev/ttyUSB0 or COM3)
    port: String,

    /// Baud rate
    #[arg(short, long, default_value_t = 921600)]
    baud: u32,

    /// Launch GUI visualizer
    #[arg(long)]
    gui: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let samples: Arc<Mutex<Vec<Vector3<f32>>>> = Arc::new(Mutex::new(Vec::new()));

    let (tx, rx) = crossbeam_channel::unbounded();

    let port = args.port.clone();
    let baud = args.baud;

    let reader_handle = std::thread::Builder::new()
        .name("reader".into())
        .spawn(move || {
            if let Err(e) = reader::reader_thread(&port, baud, tx) {
                eprintln!("[reader] Fatal error: {}", e);
            }
        })?;

    let samples_for_processor = samples.clone();
    let processor_handle = std::thread::Builder::new()
        .name("processor".into())
        .spawn(move || {
            processor::processor_thread(rx, samples_for_processor);
        })?;

    println!(
        "AHRS Reader running on port {} @ {} baud. Press Ctrl+C to stop.",
        args.port, args.baud
    );

    if args.gui {
        println!("Launching GUI visualizer...");
        // GUI must run on the main thread on Linux (X11/Wayland)
        let gui_samples = samples.clone();
        let eframe_result = visualizer::run_gui(gui_samples);
        // If GUI closes, continue running reader/processor in background
        if let Err(e) = eframe_result {
            eprintln!("[gui] Error: {}", e);
        }
    } else {
        reader_handle.join().ok();
        processor_handle.join().ok();
    }

    Ok(())
}
