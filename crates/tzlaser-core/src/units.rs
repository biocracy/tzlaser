#[derive(Debug, Clone)]
pub struct PulseConfig {
    pub pulses_per_mm_x: f64,
    pub pulses_per_mm_y: f64,
    pub pulses_per_mm_z: f64,
}

impl Default for PulseConfig {
    fn default() -> Self {
        Self {
            pulses_per_mm_x: 10_000.0,
            pulses_per_mm_y: 10_000.0,
            pulses_per_mm_z: 10_000.0,
        }
    }
}

impl PulseConfig {
    pub fn mm_to_pulses_x(&self, mm: f64) -> i32 {
        (mm * self.pulses_per_mm_x).round() as i32
    }

    pub fn pulses_to_mm_x(&self, pulses: i32) -> f64 {
        pulses as f64 / self.pulses_per_mm_x
    }

    pub fn mm_to_pulses_y(&self, mm: f64) -> i32 {
        (mm * self.pulses_per_mm_y).round() as i32
    }

    pub fn pulses_to_mm_y(&self, pulses: i32) -> f64 {
        pulses as f64 / self.pulses_per_mm_y
    }
}

