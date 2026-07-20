use nalgebra::Vector3;

// ── Трейт (контракт) ──────────────────────────
pub trait Classifier {
    fn accept(&self, data: &Vector3<f32>, existing: &[Vector3<f32>]) -> bool;
}

// ── Реализация 1: фиксированный покоординатный порог ──
pub struct CoordinateThreshold {
    threshold: f32,
}

impl CoordinateThreshold {
    pub fn new(threshold: f32) -> Self {
        Self { threshold }
    }
}

impl Classifier for CoordinateThreshold {
    fn accept(&self, data: &Vector3<f32>, existing: &[Vector3<f32>]) -> bool {
        for old in existing {
            if (data.x - old.x).abs() < self.threshold
                && (data.y - old.y).abs() < self.threshold
                && (data.z - old.z).abs() < self.threshold
            {
                return false;
            }
        }
        true
    }
}

// ── Реализация 2: адаптивный порог ────────────
pub struct AdaptiveThreshold {
    base_threshold: f32,
    max_points: usize,
}

impl AdaptiveThreshold {
    pub fn new(base_threshold: f32, max_points: usize) -> Self {
        Self { base_threshold, max_points }
    }
}

impl Classifier for AdaptiveThreshold {
    fn accept(&self, data: &Vector3<f32>, existing: &[Vector3<f32>]) -> bool {
        let fill = existing.len() as f32 / self.max_points as f32;
        let threshold = self.base_threshold * (1.0 - fill * 0.8);
        for old in existing {
            if (data.x - old.x).abs() < threshold
                && (data.y - old.y).abs() < threshold
                && (data.z - old.z).abs() < threshold
            {
                return false;
            }
        }
        true
    }
}
