use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
pub struct AHRS {
    pub rotation: UnitQuaternion<f32>,
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
    pub mag: Vector3<f32>,
    pub dt: f32,
    pub update_callback: Option<Box<dyn Fn()>>,
}

impl AHRS {
    pub fn new() -> Self {
        Self {
            rotation: UnitQuaternion::identity(),
            accel: Vector3::zeros(),
            gyro: Vector3::zeros(),
            mag: Vector3::zeros(),
            dt: 0.0,
            update_callback: None,
        }
    }

}
