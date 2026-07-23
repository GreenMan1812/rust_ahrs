use nalgebra::{Matrix3, Vector3};

#[derive(Clone, Copy)]
pub struct Params {
    pub diag_x: f32,
    pub diag_y: f32,
    pub diag_z: f32,
    pub off_xy: f32,
    pub off_xz: f32,
    pub off_yz: f32,

    pub c_x: f32,
    pub c_y: f32,
    pub c_z: f32,
    pub radius: f32,
}
impl Params {
    pub fn new(sim: [f32; 6], hard_iron: [f32; 3], radius: f32) -> Self {
        Self {
            diag_x: sim[0],
            diag_y: sim[1],
            diag_z: sim[2],
            off_xy: sim[3],
            off_xz: sim[4],
            off_yz: sim[5],
            c_x: hard_iron[0],
            c_y: hard_iron[1],
            c_z: hard_iron[2],
            radius: radius,
        }
    }
    pub fn default() -> Self {
        Self::new([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], [0.0; 3], 1.0)
    }
    pub fn to_soft_iron(&self) -> Matrix3<f32> {
        Matrix3::new(
            self.diag_x, self.off_xy, self.off_xz,
            self.off_xy, self.diag_y, self.off_yz,
            self.off_xz, self.off_yz, self.diag_z,
        )
    }
    pub fn to_hard_iron(&self) -> Vector3<f32> {
        Vector3::new(self.c_x, self.c_y, self.c_z)
    }
    pub fn get_sphere_params(&self) -> [f32; 4] {
        [self.c_x, self.c_y, self.c_z, self.radius]
    }
    pub fn get_ellipsoid_params(&self) -> [f32; 9] {
        [self.c_x, self.c_y, self.c_z, self.diag_x, self.diag_y, self.diag_z, self.off_xy, self.off_xz, self.off_yz]
    }
    pub fn is_delta_params_less_other_sphere(&self, other: &Self, threshold: f32) -> bool {
        if (self.c_x - other.c_x).abs() > threshold {
            return false;
        }
        if (self.c_y - other.c_y).abs() > threshold {
            return false;
        }
        if (self.c_z - other.c_z).abs() > threshold {
            return false;
        }
        if (self.radius - other.radius).abs() > threshold {
            return false;
        }
        true
    }
    pub fn is_delta_params_less_other_ellipsoid(&self, other: &Self, threshold: f32) -> bool {
        if (self.c_x - other.c_x).abs() > threshold {
            return false;
        }
        if (self.c_y - other.c_y).abs() > threshold {
            return false;
        }
        if (self.c_z - other.c_z).abs() > threshold {
            return false;
        }
        if (self.diag_x - other.diag_x).abs() > threshold {
            return false;
        }
        if (self.diag_y - other.diag_y).abs() > threshold {
            return false;
        }
        if (self.diag_z - other.diag_z).abs() > threshold {
            return false;
        }
        if (self.off_xy - other.off_xy).abs() > threshold {
            return false;
        }
        if (self.off_xz - other.off_xz).abs() > threshold {
            return false;
        }
        if (self.off_yz - other.off_yz).abs() > threshold {
            return false;
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_params_produce_identity_matrix() {
        let p = Params::default();
        let m = p.to_soft_iron();
        assert!((m[(0, 0)] - 1.0).abs() < f32::EPSILON);
        assert!((m[(1, 1)] - 1.0).abs() < f32::EPSILON);
        assert!((m[(2, 2)] - 1.0).abs() < f32::EPSILON);
        assert!(m[(0, 1)].abs() < f32::EPSILON);
        assert!(m[(0, 2)].abs() < f32::EPSILON);
        assert!(m[(1, 2)].abs() < f32::EPSILON);
    }

    #[test]
    fn to_soft_iron_is_symmetric() {
        let p = Params::new([1.0, 2.0, 3.0, 0.1, 0.2, 0.3], [0.0; 3], 1.0);
        let m = p.to_soft_iron();
        assert!((m[(0, 1)] - m[(1, 0)]).abs() < f32::EPSILON);
        assert!((m[(0, 2)] - m[(2, 0)]).abs() < f32::EPSILON);
        assert!((m[(1, 2)] - m[(2, 1)]).abs() < f32::EPSILON);
    }

    #[test]
    fn to_hard_iron_matches_constructor() {
        let p = Params::new([1.0; 6], [1.5, 2.5, 3.5], 1.0);
        let hi = p.to_hard_iron();
        assert!((hi.x - 1.5).abs() < f32::EPSILON);
        assert!((hi.y - 2.5).abs() < f32::EPSILON);
        assert!((hi.z - 3.5).abs() < f32::EPSILON);
    }

    #[test]
    fn sphere_param_comparison_within_threshold() {
        let a = Params::new([1.0; 6], [1.0, 2.0, 3.0], 5.0);
        let b = Params::new([1.0; 6], [1.001, 2.001, 3.001], 5.001);
        assert!(a.is_delta_params_less_other_sphere(&b, 0.01));
        assert!(!a.is_delta_params_less_other_sphere(&b, 0.0001));
    }

    #[test]
    fn ellipsoid_param_comparison_within_threshold() {
        let a = Params::new([1.0, 2.0, 3.0, 0.1, 0.2, 0.3], [1.0, 2.0, 3.0], 1.0);
        let b = Params::new([1.001, 2.001, 3.001, 0.101, 0.201, 0.301], [1.001, 2.001, 3.001], 1.0);
        assert!(a.is_delta_params_less_other_ellipsoid(&b, 0.01));
        assert!(!a.is_delta_params_less_other_ellipsoid(&b, 0.0001));
    }
}
