use eframe::egui;
use egui_plot::{Points, Plot};
use nalgebra::Vector3;
use std::sync::{Arc, Mutex};

pub struct AppState {
    pub samples: Arc<Mutex<Vec<Vector3<f32>>>>,
}

impl eframe::App for AppState {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();

        // Copy data under lock, then release before rendering
        let snapshot: Vec<Vector3<f32>> = {
            if let Ok(samples) = self.samples.lock() {
                samples.clone()
            } else {
                Vec::new()
            }
        };

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Magnetometer Data");
            ui.label(format!("Points: {}", snapshot.len()));

            ui.columns(3, |cols| {
                // XY projection
                Plot::new("xy_plot")
                    .width(250.0)
                    .height(250.0)
                    .data_aspect(1.0)
                    .show(&mut cols[0], |ui| {
                        let points: Vec<[f64; 2]> = snapshot
                            .iter()
                            .map(|v| [v.x as f64, v.y as f64])
                            .collect();
                        ui.points(Points::new(points).radius(3.0).color(egui::Color32::RED));
                    });

                // XZ projection
                Plot::new("xz_plot")
                    .width(250.0)
                    .height(250.0)
                    .data_aspect(1.0)
                    .show(&mut cols[1], |ui| {
                        let points: Vec<[f64; 2]> = snapshot
                            .iter()
                            .map(|v| [v.x as f64, v.z as f64])
                            .collect();
                        ui.points(Points::new(points).radius(3.0).color(egui::Color32::GREEN));
                    });

                // YZ projection
                Plot::new("yz_plot")
                    .width(250.0)
                    .height(250.0)
                    .data_aspect(1.0)
                    .show(&mut cols[2], |ui| {
                        let points: Vec<[f64; 2]> = snapshot
                            .iter()
                            .map(|v| [v.y as f64, v.z as f64])
                            .collect();
                        ui.points(Points::new(points).radius(3.0).color(egui::Color32::BLUE));
                    });
            });
        });
    }
}

pub fn run_gui(samples: Arc<Mutex<Vec<Vector3<f32>>>>) -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([900.0, 300.0])
            .with_title("AHRS Magnetometer Visualizer"),
        ..Default::default()
    };

    eframe::run_native(
        "AHRS Magnetometer Visualizer",
        options,
        Box::new(move |_cc| Ok(Box::new(AppState { samples }))),
    )
}
