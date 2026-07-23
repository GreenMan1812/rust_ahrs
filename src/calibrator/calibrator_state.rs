#[derive(Clone, Copy, PartialEq)]
pub enum CalibratorState {
    NotStarted,
    DataCollecting,
    DataIsReady,
    SphereFittingStep,
    EllipsoidFittingStep,
    FittingComplete,
    BadOrientations,
    CalibrationFailed,
}
