use serde::{Deserialize, Serialize};
use std::path::PathBuf;

fn default_table_width_mm() -> f64 {
    1300.0
}
fn default_table_height_mm() -> f64 {
    900.0
}
fn default_z_max_travel_mm() -> f64 {
    100.0
}
fn default_origin_corner() -> String {
    "Bottom-Left (X0, Y0)".to_string()
}
fn default_true() -> bool {
    true
}
fn default_pulses_per_mm() -> f64 {
    200.0
}
fn default_pulses_per_deg_w() -> f64 {
    100.0
}
fn default_jog_start_speed_mm_s() -> f64 {
    5.0
}
fn default_jog_speed_mm_s() -> f64 {
    10.0
}
fn default_jog_accel_mm_s2() -> f64 {
    80.0
}
fn default_z_jog_speed_reg() -> u32 {
    25
}
fn default_step_mm() -> f64 {
    1.0
}
fn default_controller_ip() -> String {
    "10.1.1.168:502".to_string()
}
fn default_z_follower_port() -> u16 {
    999
}

/// Persistent Table & Machine Properties configuration (pulses/mm, table dimensions,
/// jog speeds/accelerations, axis polarity, and controller network endpoints).
/// Automatically saved to disk when edited and recalled on startup.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PulseConfig {
    // 1. Table / Working Area Geometry
    #[serde(default = "default_table_width_mm")]
    pub table_width_mm: f64,
    #[serde(default = "default_table_height_mm")]
    pub table_height_mm: f64,
    #[serde(default = "default_z_max_travel_mm")]
    pub z_max_travel_mm: f64,
    #[serde(default = "default_origin_corner")]
    pub origin_corner: String,
    #[serde(default = "default_true")]
    pub soft_limits_enabled: bool,

    // 2. Axis Calibration (Pulses per mm / deg & Direction Polarity)
    #[serde(default = "default_pulses_per_mm")]
    pub pulses_per_mm_x: f64,
    #[serde(default = "default_pulses_per_mm")]
    pub pulses_per_mm_y: f64,
    #[serde(default = "default_pulses_per_mm")]
    pub pulses_per_mm_z: f64,
    #[serde(default = "default_pulses_per_deg_w")]
    pub pulses_per_deg_w: f64,
    #[serde(default)]
    pub invert_x: bool,
    #[serde(default)]
    pub invert_y: bool,
    #[serde(default)]
    pub invert_z: bool,

    // 3. Jogging & Motion Dynamics
    #[serde(default = "default_jog_start_speed_mm_s")]
    pub jog_start_speed_mm_s: f64,
    #[serde(default = "default_jog_speed_mm_s")]
    pub jog_speed_mm_s: f64,
    #[serde(default = "default_jog_accel_mm_s2")]
    pub jog_accel_mm_s2: f64,
    #[serde(default = "default_z_jog_speed_reg")]
    pub z_jog_speed_reg: u32,
    #[serde(default = "default_step_mm")]
    pub default_step_mm: f64,

    // 4. Controller Network Endpoints
    #[serde(default = "default_controller_ip")]
    pub controller_ip: String,
    #[serde(default = "default_z_follower_port")]
    pub z_follower_port: u16,

    #[serde(skip)]
    pub config_path: Option<PathBuf>,
}

impl Default for PulseConfig {
    fn default() -> Self {
        Self {
            table_width_mm: default_table_width_mm(),
            table_height_mm: default_table_height_mm(),
            z_max_travel_mm: default_z_max_travel_mm(),
            origin_corner: default_origin_corner(),
            soft_limits_enabled: true,
            pulses_per_mm_x: default_pulses_per_mm(),
            pulses_per_mm_y: default_pulses_per_mm(),
            pulses_per_mm_z: default_pulses_per_mm(),
            pulses_per_deg_w: default_pulses_per_deg_w(),
            invert_x: false,
            invert_y: false,
            invert_z: false,
            jog_start_speed_mm_s: default_jog_start_speed_mm_s(),
            jog_speed_mm_s: default_jog_speed_mm_s(),
            jog_accel_mm_s2: default_jog_accel_mm_s2(),
            z_jog_speed_reg: default_z_jog_speed_reg(),
            default_step_mm: default_step_mm(),
            controller_ip: default_controller_ip(),
            z_follower_port: default_z_follower_port(),
            config_path: None,
        }
    }
}

impl PulseConfig {
    /// Standard OS-appropriate path for persisting Table & Machine Properties JSON.
    pub fn default_config_path() -> Option<PathBuf> {
        #[cfg(target_os = "macos")]
        let base = std::env::var_os("HOME")
            .map(|h| PathBuf::from(h).join("Library").join("Application Support"));
        #[cfg(target_os = "windows")]
        let base = std::env::var_os("APPDATA").map(PathBuf::from);
        #[cfg(all(not(target_os = "macos"), not(target_os = "windows")))]
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")));

        base.map(|dir| dir.join("tzlaser").join("machine_properties.json"))
    }

    /// Load Table & Machine Properties from `path`, or initialize and save conservative defaults.
    pub fn load_or_init(path: Option<PathBuf>) -> Self {
        if let Some(ref p) = path {
            if p.exists() {
                if let Ok(raw) = std::fs::read_to_string(p) {
                    if let Ok(mut parsed) = serde_json::from_str::<PulseConfig>(&raw) {
                        parsed.config_path = Some(p.clone());
                        return parsed;
                    }
                }
            }
        }
        let mut cfg = Self::default();
        cfg.config_path = path;
        let _ = cfg.save();
        cfg
    }

    /// Persist current Table & Machine Properties to disk.
    pub fn save(&self) -> Result<(), String> {
        let Some(ref path) = self.config_path else {
            return Ok(());
        };
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        let json = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(path, json).map_err(|e| e.to_string())
    }

    pub fn mm_to_pulses_x(&self, mm: f64) -> i32 {
        let sign = if self.invert_x { -1.0 } else { 1.0 };
        (mm * self.pulses_per_mm_x.max(0.001) * sign).round() as i32
    }

    pub fn pulses_to_mm_x(&self, pulses: i32) -> f64 {
        let sign = if self.invert_x { -1.0 } else { 1.0 };
        (pulses as f64 / self.pulses_per_mm_x.max(0.001)) * sign
    }

    pub fn mm_to_pulses_y(&self, mm: f64) -> i32 {
        let sign = if self.invert_y { -1.0 } else { 1.0 };
        (mm * self.pulses_per_mm_y.max(0.001) * sign).round() as i32
    }

    pub fn pulses_to_mm_y(&self, pulses: i32) -> f64 {
        let sign = if self.invert_y { -1.0 } else { 1.0 };
        (pulses as f64 / self.pulses_per_mm_y.max(0.001)) * sign
    }

    pub fn mm_to_pulses_z(&self, mm: f64) -> i32 {
        let sign = if self.invert_z { -1.0 } else { 1.0 };
        (mm * self.pulses_per_mm_z.max(0.001) * sign).round() as i32
    }

    pub fn pulses_to_mm_z(&self, pulses: i32) -> f64 {
        let sign = if self.invert_z { -1.0 } else { 1.0 };
        (pulses as f64 / self.pulses_per_mm_z.max(0.001)) * sign
    }

    /// Convert configured mm/s jog parameters into controller pulse/sec parameters.
    pub fn jog_pulses_params(&self) -> (u32, u32, u32) {
        let scale = self.pulses_per_mm_x.max(1.0);
        let min_speed = (self.jog_start_speed_mm_s.max(0.5) * scale).round() as u32;
        let jog_speed = (self.jog_speed_mm_s.max(0.5) * scale).round() as u32;
        let accel = (self.jog_accel_mm_s2.max(1.0) * scale).round() as u32;
        (min_speed.max(50), jog_speed.max(50), accel.max(200))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pulse_config_save_and_reload_roundtrip() {
        let tmp_dir = std::env::temp_dir().join("tzlaser_test_props");
        let _ = std::fs::create_dir_all(&tmp_dir);
        let path = tmp_dir.join("machine_properties_test.json");
        let _ = std::fs::remove_file(&path);

        let mut cfg = PulseConfig::load_or_init(Some(path.clone()));
        assert!((cfg.pulses_per_mm_x - 200.0).abs() < 1e-6);

        cfg.table_width_mm = 1500.0;
        cfg.table_height_mm = 3000.0;
        cfg.pulses_per_mm_x = 590.0;
        cfg.pulses_per_mm_y = 590.0;
        cfg.pulses_per_mm_z = 1000.0;
        cfg.jog_speed_mm_s = 12.0;
        cfg.save().expect("should save machine properties");

        let reloaded = PulseConfig::load_or_init(Some(path.clone()));
        assert!((reloaded.table_width_mm - 1500.0).abs() < 1e-6);
        assert!((reloaded.table_height_mm - 3000.0).abs() < 1e-6);
        assert!((reloaded.pulses_per_mm_x - 590.0).abs() < 1e-6);
        assert!((reloaded.pulses_per_mm_y - 590.0).abs() < 1e-6);
        assert!((reloaded.pulses_per_mm_z - 1000.0).abs() < 1e-6);
        assert!((reloaded.jog_speed_mm_s - 12.0).abs() < 1e-6);

        let _ = std::fs::remove_file(&path);
    }
}

