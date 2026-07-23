#[derive(Clone, Copy, PartialEq, Debug)]
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
