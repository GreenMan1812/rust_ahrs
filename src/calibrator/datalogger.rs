use nalgebra::Vector3;
// use crate::calibrator::calibrator_state;
use crate::calibrator::calibrator_state::CalibratorState;
use crate::calibrator::classifier::Classifier;

pub struct DataLogger {
    pub uncalibrated_data: Vec<Vector3<f32>>,
    size_collection: usize,
    // границы
    min_bound: Vector3<f32>,
    max_bound: Vector3<f32>,
    // текущие экстремумы (всегда внутри границ)
    current_min: Vector3<f32>,
    current_max: Vector3<f32>,
    calibrator_state: CalibratorState,
    classifier: Box<dyn Classifier>,
}
impl DataLogger {
    pub fn new(size_collection: usize, bound: f32, classifier: Box<dyn Classifier>) -> Self {

        Self {
            uncalibrated_data: Vec::with_capacity(size_collection),
            size_collection,
            min_bound: Vector3::new(-bound, -bound, -bound),
            max_bound: Vector3::new(bound, bound, bound),
            current_min: Vector3::new(bound, bound, bound),
            current_max: Vector3::new(-bound, -bound, -bound),
            calibrator_state: CalibratorState::DataCollecting,
            classifier,
        }
    }
    pub fn add_data(&mut self, data: &Vector3<f32>) -> CalibratorState {
        for i in 0..3 {
            if data[i] > self.max_bound[i] || data[i] < self.min_bound[i] {
                // self.calibrator_state = CalibratorState::DataCollecting;
                return self.calibrator_state;
            }
        }
        for i in 0..3 {
            if data[i] > self.current_max[i] {
                self.current_max[i] = data[i];
            }
            if data[i] < self.current_min[i] {
                self.current_min[i] = data[i];
            }
        }
        if self.classifier.accept(data, &self.uncalibrated_data) && self.uncalibrated_data.len() < self.size_collection {
            self.uncalibrated_data.push(*data);
            self.calibrator_state = CalibratorState::DataCollecting;
        }
        if self.uncalibrated_data.len() >= self.size_collection {
            self.calibrator_state = CalibratorState::DataIsReady;
            return self.calibrator_state;
        }
        return self.calibrator_state;
    }
    pub fn clear_data(&mut self) {
        self.uncalibrated_data.clear();
    }
}
