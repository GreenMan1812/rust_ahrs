use nalgebra::{Vector3, SVector, SMatrix};
use super::params::Params;
use super::calibrator_state::CalibratorState;

pub struct Callibrator {
    pub params: Params,
    pub fitness: f32,
    calibration_state: CalibratorState,
    lambda: f32,
}

impl Callibrator {
    pub fn new(lambda: f32) -> Self {
        Self {
            params: Params::default(),
            fitness: 1e30,
            calibration_state: CalibratorState::NotStarted,
            lambda,
        }
    }

    fn calc_mse(&self, params: &Params, samples: &Vec<Vector3<f32>>) -> f32 {
        let mut sum = 0.0f32;
        for sample in samples {
            sum += self.calc_error(sample, params).powi(2);
        }
        sum / samples.len() as f32
    }
    fn calc_sphere_jacobian(&self, data: &Vector3<f32>, params: &Params) -> [f32; 4] {

        let si = params.to_soft_iron();
        let hi = params.to_hard_iron();
        let w = si * (data - hi);
        let r = w.norm();
        if r < f32::EPSILON {
            return [1.0, 0.0, 0.0, 0.0];
        }
        [
            1.0f32,
            si.column(0).dot(&w) / r,
            si.column(1).dot(&w) / r,
            si.column(2).dot(&w) / r,
        ]
    }
    pub fn calc_ellipsoid_jacobian(&self, data: &Vector3<f32>, params: &Params) -> [f32; 9] {
        let y = data - params.to_hard_iron();

        let m = params.to_soft_iron();

        // v = M * y
        let v = m * y;
        let d = v.norm();

        // Защита от деления на ноль
        if d < f32::EPSILON {
            return [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0];
        }

        let inv_d = 1.0 / d;
        let (A, B, C) = (v.x, v.y, v.z);

        let diag_x = params.diag_x;
        let diag_y = params.diag_y;
        let diag_z = params.diag_z;
        let off_xy = params.off_xy;
        let off_xz = params.off_xz;
        let off_yz = params.off_yz;

        // ---- Производные по центру (b) ----
        // Так как b = -offset, то df/db = - df/doffset = (M^T * v) / d
        let dbx = (diag_x * A + off_xy * B + off_xz * C) * inv_d;
        let dby = (off_xy * A + diag_y * B + off_yz * C) * inv_d;
        let dbz = (off_xz * A + off_yz * B + diag_z * C) * inv_d;

        // ---- Производные по диагональным элементам ----
        let ddiag_x = -(y.x * A) * inv_d;
        let ddiag_y = -(y.y * B) * inv_d;
        let ddiag_z = -(y.z * C) * inv_d;

        // ---- Производные по внедиагональным элементам ----
        let doff_xy = -(y.y * A + y.x * B) * inv_d;
        let doff_xz = -(y.z * A + y.x * C) * inv_d;
        let doff_yz = -(y.z * B + y.y * C) * inv_d;

        [
            dbx, dby, dbz,
            ddiag_x, ddiag_y, ddiag_z,
            doff_xy, doff_xz, doff_yz,
        ]
    }
    fn calc_error(&self, data: &Vector3<f32>, params: &Params) -> f32 {
        let d = (params.to_soft_iron() * (data - params.to_hard_iron())).norm();
        params.radius - d
    }
    pub fn step_sphere_fit(&mut self, samples: &Vec<Vector3<f32>>) -> CalibratorState
    {
        if samples.len() == 0{
            return CalibratorState::NotStarted;
        }
        let lma_damping = 10.0;
        let mut _params1_fit = self.params;
        let mut _params2_fit = self.params;
        let mut jtj1 = SMatrix::<f32, 4, 4>::zeros();
        let mut jtj2 = SMatrix::<f32, 4, 4>::zeros();
        let mut jfi =  SVector::<f32, 4>::zeros();

        for data in samples {
            let sphere_jacob = self.calc_sphere_jacobian(data, &_params1_fit);
            let error = self.calc_error(data, &_params1_fit);

            for i in 0..4 {
                for j in 0..4 {
                    jtj1[(i, j)] += sphere_jacob[i] * sphere_jacob[j];
                    jtj2[(i, j)] += sphere_jacob[i] * sphere_jacob[j];
                }
                jfi[i] += sphere_jacob[i] * error;
            }
        }

        for i in 0..4 {
            jtj1[(i, i)] += self.lambda;
            jtj2[(i, i)] += self.lambda / lma_damping;
        }
        let inv_jtj1 = match jtj1.try_inverse() {
            Some(m) => m,
            None => return CalibratorState::CalibrationFailed,
        };
        let inv_jtj2 = match jtj2.try_inverse() {
            Some(m) => m,
            None => return CalibratorState::CalibrationFailed,
        };

        let delta1 = -inv_jtj1 * jfi;
        let delta2 = -inv_jtj2 * jfi;
        _params1_fit.radius += delta1[0];
        _params1_fit.c_x   += delta1[1];
        _params1_fit.c_y   += delta1[2];
        _params1_fit.c_z   += delta1[3];
        _params2_fit.radius += delta2[0];
        _params2_fit.c_x   += delta2[1];
        _params2_fit.c_y   += delta2[2];
        _params2_fit.c_z   += delta2[3];

        let fit1 = self.calc_mse(&_params1_fit, samples);
        let fit2 = self.calc_mse(&_params2_fit, samples);

        let mut _fitness = self.fitness;
        if fit1 > _fitness && fit2 > _fitness {
            self.lambda *= lma_damping;
        } else if fit2 < _fitness && fit2 < fit1 {
            self.lambda /= lma_damping;
            _params1_fit = _params2_fit;
            _fitness = fit2;
        } else if fit1 < _fitness {
            _fitness = fit1;
        }
        if !_fitness.is_nan() && self.fitness > _fitness {
            self.fitness = _fitness;
            self.params = _params1_fit;
        }
        self.calibration_state = CalibratorState::SphereFittingStep;
        return self.calibration_state;
    }


    pub fn step_ellipse_fit(&mut self, samples: &Vec<Vector3<f32>>) -> CalibratorState {
        if samples.len() == 0{
            return CalibratorState::NotStarted;
        }
        let lma_damping = 10.0;
        let mut _params1_fit = self.params;
        let mut _params2_fit = self.params;
        let mut jtj1 = SMatrix::<f32, 9, 9>::zeros();
        let mut jtj2 = SMatrix::<f32, 9, 9>::zeros();
        let mut jfi =  SVector::<f32, 9>::zeros();

        for data in samples {
            let ellipse_jacob = self.calc_ellipsoid_jacobian(data, &_params1_fit);
            let error = self.calc_error(data, &_params1_fit);

            for i in 0..9 {
                for j in 0..9 {
                    jtj1[(i, j)] += ellipse_jacob[i] * ellipse_jacob[j];
                    jtj2[(i, j)] += ellipse_jacob[i] * ellipse_jacob[j];
                }
                jfi[i] += ellipse_jacob[i] * error;
            }
        }

        for i in 0..9 {
            jtj1[(i, i)] += self.lambda;
            jtj2[(i, i)] += self.lambda / lma_damping;
        }
        let inv_jtj1 = match jtj1.try_inverse() {
            Some(m) => m,
            None => return CalibratorState::CalibrationFailed,
        };
        let inv_jtj2 = match jtj2.try_inverse() {
            Some(m) => m,
            None => return CalibratorState::CalibrationFailed,
        };

        let delta1 = -inv_jtj1 * jfi;
        let delta2 = -inv_jtj2 * jfi;

        _params1_fit.c_x   += delta1[0];
        _params1_fit.c_y   += delta1[1];
        _params1_fit.c_z   += delta1[2];
        _params1_fit.diag_x += delta1[3];
        _params1_fit.diag_y += delta1[4];
        _params1_fit.diag_z += delta1[5];
        _params1_fit.off_xy += delta1[6];
        _params1_fit.off_xz += delta1[7];
        _params1_fit.off_yz += delta1[8];

        _params2_fit.c_x   += delta2[0];
        _params2_fit.c_y   += delta2[1];
        _params2_fit.c_z   += delta2[2];
        _params2_fit.diag_x += delta2[3];
        _params2_fit.diag_y += delta2[4];
        _params2_fit.diag_z += delta2[5];
        _params2_fit.off_xy += delta2[6];
        _params2_fit.off_xz += delta2[7];
        _params2_fit.off_yz += delta2[8];

        let fit1 = self.calc_mse(&_params1_fit, samples);
        let fit2 = self.calc_mse(&_params2_fit, samples);

        let mut _fitness = self.fitness;
        if fit1 > _fitness && fit2 > _fitness {
            self.lambda *= lma_damping;
        } else if fit2 < _fitness && fit2 < fit1 {
            self.lambda /= lma_damping;
            _params1_fit = _params2_fit;
            _fitness = fit2;
        } else if fit1 < _fitness {
            _fitness = fit1;
        }
        if !_fitness.is_nan() && self.fitness > _fitness {
            self.fitness = _fitness;
            self.params = _params1_fit;
        }
        self.calibration_state = CalibratorState::EllipsoidFittingStep;
        return self.calibration_state;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Генерирует точки на сфере с заданным центром и радиусом
    fn generate_sphere_samples(center: Vector3<f32>, radius: f32, n: usize) -> Vec<Vector3<f32>> {
        let mut samples = Vec::with_capacity(n);
        for i in 0..n {
            let theta = 2.0 * std::f32::consts::PI * (i as f32) / (n as f32);
            let phi = std::f32::consts::PI * (i as f32) / (n as f32);
            let x = radius * phi.sin() * theta.cos() + center.x;
            let y = radius * phi.sin() * theta.sin() + center.y;
            let z = radius * phi.cos() + center.z;
            samples.push(Vector3::new(x, y, z));
        }
        samples
    }

    /// Генерирует точки на эллипсоиде (растянутая сфера)
    fn generate_ellipsoid_samples(
        center: Vector3<f32>,
        radii: Vector3<f32>,
        n: usize,
    ) -> Vec<Vector3<f32>> {
        let mut samples = Vec::with_capacity(n);
        for i in 0..n {
            let theta = 2.0 * std::f32::consts::PI * (i as f32) / (n as f32);
            let phi = std::f32::consts::PI * (i as f32) / (n as f32);
            let x = radii.x * phi.sin() * theta.cos() + center.x;
            let y = radii.y * phi.sin() * theta.sin() + center.y;
            let z = radii.z * phi.cos() + center.z;
            samples.push(Vector3::new(x, y, z));
        }
        samples
    }

    // ── calibrator state ──

    #[test]
    fn sphere_fit_empty_samples_returns_not_started() {
        let mut cal = Callibrator::new(1e-3);
        let state = cal.step_sphere_fit(&vec![]);
        assert_eq!(state, CalibratorState::NotStarted);
    }

    #[test]
    fn ellipse_fit_empty_samples_returns_not_started() {
        let mut cal = Callibrator::new(1e-3);
        let state = cal.step_ellipse_fit(&vec![]);
        assert_eq!(state, CalibratorState::NotStarted);
    }

    #[test]
    fn new_calibrator_has_default_fitness() {
        let cal = Callibrator::new(1e-3);
        assert_eq!(cal.fitness, 1e30);
        assert_eq!(cal.calibration_state, CalibratorState::NotStarted);
    }

    // ── error function ──

    #[test]
    fn calc_error_is_zero_on_perfect_sphere() {
        let p = Params::new([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], [0.0; 3], 10.0);
        let cal = Callibrator::new(1e-3);
        let point = Vector3::new(10.0, 0.0, 0.0);
        let err = cal.calc_error(&point, &p);
        assert!(err.abs() < 1e-5, "error = {}", err);
    }

    #[test]
    fn calc_error_positive_inside_sphere() {
        let p = Params::new([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], [0.0; 3], 10.0);
        let cal = Callibrator::new(1e-3);
        let point = Vector3::new(5.0, 0.0, 0.0);
        let err = cal.calc_error(&point, &p);
        assert!(err > 0.0, "error should be positive inside sphere, got {}", err);
    }

    #[test]
    fn calc_error_negative_outside_sphere() {
        let p = Params::new([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], [0.0; 3], 10.0);
        let cal = Callibrator::new(1e-3);
        let point = Vector3::new(15.0, 0.0, 0.0);
        let err = cal.calc_error(&point, &p);
        assert!(err < 0.0, "error should be negative outside sphere, got {}", err);
    }

    #[test]
    fn calc_error_with_hard_iron_offset() {
        let p = Params::new([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], [5.0, 0.0, 0.0], 10.0);
        let cal = Callibrator::new(1e-3);
        // data=(15,0,0), hard_iron=(5,0,0) → data-hi=(10,0,0) → norm=10 → error=0
        let point = Vector3::new(15.0, 0.0, 0.0);
        let err = cal.calc_error(&point, &p);
        assert!(err.abs() < 1e-5, "error = {}", err);
    }

    // ── jacobians ──

    #[test]
    fn sphere_jacobian_has_correct_dim() {
        let cal = Callibrator::new(1e-3);
        let p = Params::default();
        let data = Vector3::new(1.0, 2.0, 3.0);
        let j = cal.calc_sphere_jacobian(&data, &p);
        assert_eq!(j.len(), 4);
    }

    #[test]
    fn sphere_jacobian_first_element_is_one() {
        let cal = Callibrator::new(1e-3);
        let p = Params::default();
        let data = Vector3::new(1.0, 2.0, 3.0);
        let j = cal.calc_sphere_jacobian(&data, &p);
        assert!((j[0] - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn sphere_jacobian_at_zero_returns_fallback() {
        let cal = Callibrator::new(1e-3);
        let p = Params::default();
        let data = Vector3::new(0.0, 0.0, 0.0);
        let j = cal.calc_sphere_jacobian(&data, &p);
        // data=0, center=0, r=0 → fallback
        assert_eq!(j, [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn ellipsoid_jacobian_has_correct_dim() {
        let cal = Callibrator::new(1e-3);
        let p = Params::default();
        let data = Vector3::new(1.0, 2.0, 3.0);
        let j = cal.calc_ellipsoid_jacobian(&data, &p);
        assert_eq!(j.len(), 9);
    }

    #[test]
    fn ellipsoid_jacobian_at_center_returns_fallback() {
        let cal = Callibrator::new(1e-3);
        let p = Params::new([1.0; 6], [5.0, 5.0, 5.0], 1.0);
        let data = Vector3::new(5.0, 5.0, 5.0); // data == center
        let j = cal.calc_ellipsoid_jacobian(&data, &p);
        // v = M * (data - center) = 0 → fallback
        assert_eq!(j, [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0]);
    }

    // ── sphere fitting convergence ──

    #[test]
    fn sphere_fit_converges_to_known_center_and_radius() {
        let center = Vector3::new(10.0, -20.0, 5.0);
        let radius = 50.0;
        let samples = generate_sphere_samples(center, radius, 200);

        let mut cal = Callibrator::new(1e-3);
        for _ in 0..500 {
            cal.step_sphere_fit(&samples);
        }

        let err_cx = (cal.params.c_x - center.x).abs();
        let err_cy = (cal.params.c_y - center.y).abs();
        let err_cz = (cal.params.c_z - center.z).abs();
        let err_r = (cal.params.radius - radius).abs();

        assert!(err_cx < 1.0, "c_x: got {}, want ~{}", cal.params.c_x, center.x);
        assert!(err_cy < 1.0, "c_y: got {}, want ~{}", cal.params.c_y, center.y);
        assert!(err_cz < 1.0, "c_z: got {}, want ~{}", cal.params.c_z, center.z);
        assert!(err_r < 2.0, "radius: got {}, want ~{}", cal.params.radius, radius);
    }

    #[test]
    fn sphere_fit_fitness_decreases() {
        let center = Vector3::new(5.0, -10.0, 3.0);
        let radius = 30.0;
        let samples = generate_sphere_samples(center, radius, 100);

        let mut cal = Callibrator::new(1e-3);
        let initial_fitness = cal.fitness;
        for _ in 0..100 {
            cal.step_sphere_fit(&samples);
        }
        assert!(cal.fitness < initial_fitness, "fitness should decrease: {} -> {}", initial_fitness, cal.fitness);
    }

    // ── ellipsoid fitting ──

    #[test]
    fn ellipse_fit_on_stretched_sphere() {
        let center = Vector3::new(10.0, 20.0, -5.0);
        let radii = Vector3::new(50.0, 30.0, 40.0);
        let samples = generate_ellipsoid_samples(center, radii, 300);

        // Сначала sphere fit для начального приближения
        let mut cal = Callibrator::new(1e-3);
        for _ in 0..200 {
            cal.step_sphere_fit(&samples);
        }
        // Затем ellipsoid fit
        for _ in 0..500 {
            cal.step_ellipse_fit(&samples);
        }

        // Центр должен быть найден точно
        let err_cx = (cal.params.c_x - center.x).abs();
        let err_cy = (cal.params.c_y - center.y).abs();
        let err_cz = (cal.params.c_z - center.z).abs();
        assert!(err_cx < 2.0, "c_x: got {}", cal.params.c_x);
        assert!(err_cy < 2.0, "c_y: got {}", cal.params.c_y);
        assert!(err_cz < 2.0, "c_z: got {}", cal.params.c_z);
    }

    #[test]
    fn ellipse_fit_preserves_sphere_convergence() {
        let center = Vector3::new(0.0, 0.0, 0.0);
        let radius = 100.0;
        let samples = generate_sphere_samples(center, radius, 200);

        let mut cal = Callibrator::new(1e-3);
        for _ in 0..300 {
            cal.step_sphere_fit(&samples);
        }
        let after_sphere = cal.fitness;

        // Ellipsoid fit на сферических данных не должен ухудшить результат
        for _ in 0..200 {
            cal.step_ellipse_fit(&samples);
        }
        assert!(cal.fitness <= after_sphere + 1e-6, "fitness worsened: {} -> {}", after_sphere, cal.fitness);
    }
}
