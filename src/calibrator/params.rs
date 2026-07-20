use nalgebra::{Matrix3, Vector3};

struct Params {
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
}
