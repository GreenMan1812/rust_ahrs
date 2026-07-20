pub enum CalibratorState {
    not_started,
    data_collecting,
    data_is_ready,
    sphere_fitting_step,
    ellipsoid_fitting_step,
    fitting_complete,
    bad_orientations,
}
