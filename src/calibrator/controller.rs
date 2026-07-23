use nalgebra::Vector3;
use super::calibrator::Callibrator;
use super::calibrator_state::CalibratorState;
use super::classifier::AdaptiveThreshold;
use super::datalogger::DataLogger;



pub struct CalibrationController {
    calibrator: Callibrator,
    state: CalibratorState,
    sphere_iterations: usize,
    ellipsoid_iterations: usize,
    max_iterations: usize,
    convergence_epsilon: f32,
    stable_count: usize,
    required_stable: usize,
    data_logger: DataLogger,
}


impl CalibrationController {
    pub fn new(
        sample_count: usize,
        bound: f32,
        lambda: f32,
        max_iterations: usize,
        convergence_epsilon: f32,
    ) -> Self {
        let classifier = Box::new(AdaptiveThreshold::new(bound * 0.05, sample_count));
        let data_logger = DataLogger::new(sample_count, bound, classifier);
        let calibrator = Callibrator::new(lambda);
        Self {
            calibrator,
            state: CalibratorState::NotStarted,
            sphere_iterations: 0,
            ellipsoid_iterations: 0,
            max_iterations,
            convergence_epsilon,
            stable_count: 0,
            required_stable: 5,
            data_logger,
        }
    }
    pub fn update(&mut self, data: &Vector3<f32>)->bool {
        // let state = self.calibrator(data);
        match self.state {
            CalibratorState::NotStarted => {
                self.state = CalibratorState::DataCollecting;
                self.data_logger.add_data(data);
            }
            CalibratorState::DataCollecting => {
                self.state = self.data_logger.add_data(data);
            }
            CalibratorState::DataIsReady => {
                self.state = CalibratorState::SphereFittingStep;
                self.sphere_iterations = 0;
                self.ellipsoid_iterations = 0;
                self.stable_count = 0;
                // self.calibrator.step_sphere_fitting();
            }
            CalibratorState::SphereFittingStep => {
                let prev = self.calibrator.fitness;
                self.calibrator.step_sphere_fit(&self.data_logger.uncalibrated_data);
                self.sphere_iterations += 1;
                if (self.calibrator.fitness - prev).abs() < self.convergence_epsilon {
                    self.stable_count += 1;
                } else {
                    self.stable_count = 0;
                }
                if self.sphere_iterations >= self.max_iterations || self.stable_count >= self.required_stable {
                    self.state = CalibratorState::EllipsoidFittingStep;
                    self.stable_count = 0;
                }
            }
            CalibratorState::EllipsoidFittingStep => {
                let prev = self.calibrator.fitness;
                self.calibrator.step_ellipse_fit(&self.data_logger.uncalibrated_data);
                self.ellipsoid_iterations += 1;
                if (self.calibrator.fitness - prev).abs() < self.convergence_epsilon {
                    self.stable_count += 1;
                } else {
                    self.stable_count = 0;
                }
                if self.ellipsoid_iterations >= self.max_iterations || self.stable_count >= self.required_stable {
                    self.state = CalibratorState::FittingComplete;
                    self.stable_count = 0;
                }
            }
            CalibratorState::CalibrationFailed => {
                self.state = CalibratorState::NotStarted;
                self.data_logger.clear_data();
                return false;
            }
            CalibratorState::FittingComplete => {
                self.state = CalibratorState::NotStarted;
                self.data_logger.clear_data();
                return true;
            }
            CalibratorState::BadOrientations => {
                self.state = CalibratorState::NotStarted;
                self.data_logger.clear_data();
                return false;
            }
        }
        return false;
    }
}
