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
    fn calc_sphere_jacobian(&self, data: &Vector3<f32>, params: &Params) -> Vec<f32> {

        let si = params.to_soft_iron();
        let hi = params.to_hard_iron();
        let w = si * (data - hi);
        let r = w.norm();
        if r < f32::EPSILON {
            return vec![1.0, 0.0, 0.0, 0.0];
        }
        vec![
            1.0f32,
            si.column(0).dot(&w) / r,
            si.column(1).dot(&w) / r,
            si.column(2).dot(&w) / r,
        ]
    }
    pub fn calc_ellipsoid_jacobian(&self, data: &Vector3<f32>, params: &Params) -> Vec<f32> {
        let y = data - params.to_hard_iron();

        let m = params.to_soft_iron();

        // v = M * y
        let v = m * y;
        let d = v.norm();

        // Защита от деления на ноль
        if d < f32::EPSILON {
            return vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0];
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

        vec![
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
