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

#[cfg(test)]
mod tests {
    use super::*;

    // ── CoordinateThreshold ──

    #[test]
    fn coord_threshold_rejects_close_point() {
        let c = CoordinateThreshold::new(1.0);
        let existing = vec![Vector3::new(0.0, 0.0, 0.0)];
        assert!(!c.accept(&Vector3::new(0.5, 0.5, 0.5), &existing));
    }

    #[test]
    fn coord_threshold_accepts_distant_point() {
        let c = CoordinateThreshold::new(1.0);
        let existing = vec![Vector3::new(0.0, 0.0, 0.0)];
        assert!(c.accept(&Vector3::new(5.0, 5.0, 5.0), &existing));
    }

    #[test]
    fn coord_threshold_accepts_empty_list() {
        let c = CoordinateThreshold::new(1.0);
        let existing: Vec<Vector3<f32>> = vec![];
        assert!(c.accept(&Vector3::new(0.0, 0.0, 0.0), &existing));
    }

    #[test]
    fn coord_threshold_only_one_axis_close_accepts() {
        let c = CoordinateThreshold::new(1.0);
        let existing = vec![Vector3::new(0.0, 0.0, 0.0)];
        assert!(c.accept(&Vector3::new(0.5, 10.0, 10.0), &existing));
    }

    #[test]
    fn coord_threshold_rejects_all_axes_close() {
        let c = CoordinateThreshold::new(2.0);
        let existing = vec![Vector3::new(1.0, 1.0, 1.0)];
        assert!(!c.accept(&Vector3::new(1.5, 1.5, 1.5), &existing));
    }

    #[test]
    fn coord_threshold_multiple_existing_points() {
        let c = CoordinateThreshold::new(1.0);
        let existing = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 10.0, 10.0),
        ];
        assert!(!c.accept(&Vector3::new(0.5, 0.5, 0.5), &existing));
        assert!(!c.accept(&Vector3::new(9.5, 9.5, 9.5), &existing));
        assert!(c.accept(&Vector3::new(5.0, 5.0, 5.0), &existing));
    }

    // ── AdaptiveThreshold ──

    #[test]
    fn adaptive_threshold_high_when_empty() {
        let c = AdaptiveThreshold::new(10.0, 100);
        let empty: Vec<Vector3<f32>> = vec![];
        assert!(c.accept(&Vector3::new(5.0, 5.0, 5.0), &empty));
    }

    #[test]
    fn adaptive_threshold_low_when_full() {
        let c = AdaptiveThreshold::new(10.0, 100);
        let full: Vec<Vector3<f32>> = (0..100)
            .map(|i| Vector3::new(i as f32, 0.0, 0.0))
            .collect();
        assert!(!c.accept(&Vector3::new(0.5, 0.0, 0.0), &full));
    }

    #[test]
    fn adaptive_threshold_accepts_empty_list() {
        let c = AdaptiveThreshold::new(1.0, 100);
        let existing: Vec<Vector3<f32>> = vec![];
        assert!(c.accept(&Vector3::new(0.0, 0.0, 0.0), &existing));
    }
}
