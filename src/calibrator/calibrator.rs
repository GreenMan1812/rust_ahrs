use nalgebra::{Matrix3, Vector3};
use std::{f32::{NAN, consts::PI}, hash};
// mod params;
use super::params::Params;
// mod datalogger;
use super::datalogger::DataLogger;
use super::calibrator_state::CalibratorState;
use super::classifier::Classifier;
// use super::
//
//
pub struct Callibrator {
    pub params: Params,
    fitness: f32,
}

impl Callibrator {
    pub fn new() -> Self {
        Self {
            params: Params::default(),
            fitness: NAN,
        }
    }

    fn calc_residular(&self, data: &Vector3<f32>) -> f32 {
        let si = self.params.to_soft_iron();
        let hi = self.params.to_hard_iron();
        let w = si * (data - hi);
        let r = w.norm();
        return self.params.expected_radius - r;
    }
    fn calc_mse(&self) -> f32 {
        let mut sum = 0.0f32;
        for sample in &self.data_for_calibration {
            sum += self.calc_residular(sample).powi(2);
        }
        sum / self.data_for_calibration.len() as f32
    }
    fn calc_sphere_jacobian(&self, data: &Vector3<f32>) -> Vec<f32> {

        let si = self.params.to_soft_iron();
        let hi = self.params.to_hard_iron();
        let w = si * (data - hi);
        let r = w.norm();
        vec![
            1.0f32,
            si.column(0).dot(&w) / r,
            si.column(1).dot(&w) / r,
            si.column(2).dot(&w) / r,
        ]
    }
    pub fn calc_ellipsoid_jacobian(&self, data: &Vector3<f32>) -> Vec<f32> {
        let y = data - self.params.to_hard_iron();

        let m = self.params.to_soft_iron();

        // v = M * y
        let v = m * y;
        let d = v.norm();

        // Защита от деления на ноль
        if d < f32::EPSILON {
            return vec![1.0, 1.0, 1.0, 0.0, 0.0, 0.0];
        }

        let inv_d = 1.0 / d;
        let (A, B, C) = (v.x, v.y, v.z);

        // let [diag_x, diag_y, diag_z, off_xy, off_xz, off_yz] =[ self.params.diag_x, self.params.diag_y, self.params.diag_z, self.params.off_xy, self.params.off_xz, self.params.off_yz];
        let diag_x = self.params.diag_x;
        let diag_y = self.params.diag_y;
        let diag_z = self.params.diag_z;
        let off_xy = self.params.off_xy;
        let off_xz = self.params.off_xz;
        let off_yz = self.params.off_yz;
        let dr = 1.0;

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
    fn calc_error(&self, data: &Vector3<f32>) -> f32 {
        let m = self.soft_iron_params_to_matrix();
        let w = m*(data - self.hard_iron_correction);
        let d = w.norm();
        self.expected_radius - d
    }
    pub fn step_fit<F>(&mut self, jacobian: F) -> u32
    where
        F: Fn(&Vector3<f32>) -> Vec<f32>,
    {

        if self.data_for_calibration.len() == 0{
            return -1;
        }
        let lma_damping = 10.0;
        let mut last_fitness =  self.fitness;
        let _params1_fit = self.
        return -1;
    }
}
