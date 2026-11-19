use crate::document::{Color, CuttingParams, Document, Layer};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct MaterialPreset {
    pub name: String,
    pub material: String,
    pub thickness_mm: f64,
    pub params: CuttingParams,
    #[serde(default)]
    pub notes: String,
    #[serde(default)]
    pub is_builtin: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SavedLayerSpec {
    pub name: String,
    pub color: Color,
    pub visible: bool,
    pub params: CuttingParams,
}

impl From<&Layer> for SavedLayerSpec {
    fn from(layer: &Layer) -> Self {
        Self {
            name: layer.name.clone(),
            color: layer.color,
            visible: layer.visible,
            params: layer.params.clone(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayerConfigEntry {
    pub name: String,
    pub layers: Vec<SavedLayerSpec>,
}

/// Embedded JSON-backed database holding laser cutting material presets and saved layer setups.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialLibrary {
    pub materials: Vec<MaterialPreset>,
    #[serde(default)]
    pub layer_configs: Vec<LayerConfigEntry>,
    #[serde(skip)]
    db_path: Option<PathBuf>,
}

impl Default for MaterialLibrary {
    fn default() -> Self {
        Self::with_default_presets()
    }
}

impl MaterialLibrary {
    /// Create an in-memory material library seeded with standard laser cutting presets.
    pub fn with_default_presets() -> Self {
        Self {
            materials: vec![
                MaterialPreset {
                    name: "Mild Steel 1mm (Air)".to_string(),
                    material: "Mild Steel".to_string(),
                    thickness_mm: 1.0,
                    params: CuttingParams {
                        speed_mm_s: 20.0,
                        power_percent: 85.0,
                        pierce_time_ms: 150,
                        gas_type: "Air".to_string(),
                        cut_height_mm: 1.0,
                        up_height_mm: 20.0,
                        gas_between_paths: false,
                        frequency_hz: 5000,
                    },
                    notes: "Fast thin-sheet air cut".to_string(),
                    is_builtin: true,
                },
                MaterialPreset {
                    name: "Mild Steel 2mm (O2)".to_string(),
                    material: "Mild Steel".to_string(),
                    thickness_mm: 2.0,
                    params: CuttingParams {
                        speed_mm_s: 16.0,
                        power_percent: 95.0,
                        pierce_time_ms: 250,
                        gas_type: "O2".to_string(),
                        cut_height_mm: 1.2,
                        up_height_mm: 20.0,
                        gas_between_paths: false,
                        frequency_hz: 4000,
                    },
                    notes: "Clean oxygen-assisted cut".to_string(),
                    is_builtin: true,
                },
                MaterialPreset {
                    name: "Mild Steel 3mm (O2)".to_string(),
                    material: "Mild Steel".to_string(),
                    thickness_mm: 3.0,
                    params: CuttingParams {
                        speed_mm_s: 12.0,
                        power_percent: 100.0,
                        pierce_time_ms: 350,
                        gas_type: "O2".to_string(),
                        cut_height_mm: 1.2,
                        up_height_mm: 20.0,
                        gas_between_paths: false,
                        frequency_hz: 3000,
                    },
                    notes: "Standard 3mm mild steel".to_string(),
                    is_builtin: true,
                },
                MaterialPreset {
                    name: "Mild Steel 5mm (O2)".to_string(),
                    material: "Mild Steel".to_string(),
                    thickness_mm: 5.0,
                    params: CuttingParams {
                        speed_mm_s: 8.0,
                        power_percent: 100.0,
                        pierce_time_ms: 500,
                        gas_type: "O2".to_string(),
                        cut_height_mm: 1.5,
                        up_height_mm: 20.0,
                        gas_between_paths: false,
                        frequency_hz: 2000,
                    },
                    notes: "Heavy plate pierce & slow feed".to_string(),
                    is_builtin: true,
                },
                MaterialPreset {
                    name: "Stainless 1mm (N2)".to_string(),
                    material: "Stainless Steel".to_string(),
                    thickness_mm: 1.0,
                    params: CuttingParams {
                        speed_mm_s: 18.0,
                        power_percent: 90.0,
                        pierce_time_ms: 150,
                        gas_type: "N2".to_string(),
                        cut_height_mm: 0.8,
                        up_height_mm: 20.0,
                        gas_between_paths: false,
                        frequency_hz: 6000,
                    },
                    notes: "Oxide-free nitrogen cut".to_string(),
                    is_builtin: true,
                },
                MaterialPreset {
                    name: "Stainless 2mm (N2)".to_string(),
                    material: "Stainless Steel".to_string(),
                    thickness_mm: 2.0,
                    params: CuttingParams {
                        speed_mm_s: 14.0,
                        power_percent: 100.0,
                        pierce_time_ms: 250,
                        gas_type: "N2".to_string(),
                        cut_height_mm: 0.8,
                        up_height_mm: 20.0,
                        gas_between_paths: false,
                        frequency_hz: 5000,
                    },
                    notes: "High-pressure nitrogen".to_string(),
                    is_builtin: true,
                },
                MaterialPreset {
                    name: "Aluminum 1.5mm (N2)".to_string(),
                    material: "Aluminum".to_string(),
                    thickness_mm: 1.5,
                    params: CuttingParams {
                        speed_mm_s: 16.0,
                        power_percent: 95.0,
                        pierce_time_ms: 200,
                        gas_type: "N2".to_string(),
                        cut_height_mm: 0.8,
                        up_height_mm: 20.0,
                        gas_between_paths: false,
                        frequency_hz: 6000,
                    },
                    notes: "Reflective alloy cut".to_string(),
                    is_builtin: true,
                },
                MaterialPreset {
                    name: "Aluminum 3mm (N2)".to_string(),
                    material: "Aluminum".to_string(),
                    thickness_mm: 3.0,
                    params: CuttingParams {
                        speed_mm_s: 10.0,
                        power_percent: 100.0,
                        pierce_time_ms: 350,
                        gas_type: "N2".to_string(),
                        cut_height_mm: 1.0,
                        up_height_mm: 20.0,
                        gas_between_paths: false,
                        frequency_hz: 5000,
                    },
                    notes: "3mm aluminum plate".to_string(),
                    is_builtin: true,
                },
                MaterialPreset {
                    name: "Engrave / Mark".to_string(),
                    material: "General".to_string(),
                    thickness_mm: 0.0,
                    params: CuttingParams {
                        speed_mm_s: 20.0,
                        power_percent: 25.0,
                        pierce_time_ms: 0,
                        gas_type: "Air".to_string(),
                        cut_height_mm: 2.0,
                        up_height_mm: 20.0,
                        gas_between_paths: false,
                        frequency_hz: 20000,
                    },
                    notes: "Surface marking / etch pass".to_string(),
                    is_builtin: true,
                },
            ],
            layer_configs: Vec::new(),
            db_path: None,
        }
    }

    /// Standard OS-appropriate path for the internal material/layer database JSON file.
    pub fn default_db_path() -> Option<PathBuf> {
        #[cfg(target_os = "macos")]
        let base = std::env::var_os("HOME")
            .map(|h| PathBuf::from(h).join("Library").join("Application Support"));
        #[cfg(target_os = "windows")]
        let base = std::env::var_os("APPDATA").map(PathBuf::from);
        #[cfg(all(not(target_os = "macos"), not(target_os = "windows")))]
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")));

        base.map(|dir| dir.join("tzlaser").join("materials_db.json"))
    }

    /// Load the database from `path`, or create and persist a seeded default database if absent.
    pub fn load_or_init(path: Option<PathBuf>) -> Self {
        if let Some(ref p) = path {
            if p.exists() {
                if let Ok(raw) = std::fs::read_to_string(p) {
                    if let Ok(mut parsed) = serde_json::from_str::<MaterialLibrary>(&raw) {
                        parsed.db_path = Some(p.clone());
                        // Migrate legacy DB entries that didn't store `is_builtin`
                        if parsed.materials.iter().all(|m| !m.is_builtin) {
                            let defaults = Self::with_default_presets();
                            for m in &mut parsed.materials {
                                if defaults.materials.iter().any(|d| d.name == m.name) {
                                    m.is_builtin = true;
                                }
                            }
                        }
                        parsed.sort_user_before_defaults();
                        return parsed;
                    }
                }
            }
        }
        let mut db = Self::with_default_presets();
        db.db_path = path;
        let _ = db.save_db();
        db
    }

    /// Keep all user-saved profiles (`!is_builtin`) before built-in default profiles (`is_builtin`).
    pub fn sort_user_before_defaults(&mut self) {
        let mut user = Vec::with_capacity(self.materials.len());
        let mut builtins = Vec::with_capacity(self.materials.len());
        for m in self.materials.drain(..) {
            if m.is_builtin {
                builtins.push(m);
            } else {
                user.push(m);
            }
        }
        self.materials.extend(user);
        self.materials.extend(builtins);
    }

    pub fn db_path(&self) -> Option<&PathBuf> {
        self.db_path.as_ref()
    }

    /// Persist the material library & saved layer configurations to the internal JSON DB.
    pub fn save_db(&self) -> Result<(), String> {
        let Some(ref path) = self.db_path else {
            return Ok(());
        };
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .map_err(|e| format!("Failed to create DB directory: {}", e))?;
        }
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| format!("Failed to serialize material DB: {}", e))?;
        std::fs::write(path, json).map_err(|e| format!("Failed to write material DB: {}", e))
    }

    /// Insert or update a user-saved material preset by name (case-insensitive) and persist.
    /// User-saved presets are always placed before built-in default presets.
    pub fn upsert_material(&mut self, mut preset: MaterialPreset) -> Result<bool, String> {
        preset.is_builtin = false;
        let updated = if let Some(existing) = self
            .materials
            .iter_mut()
            .find(|m| m.name.eq_ignore_ascii_case(&preset.name))
        {
            *existing = preset;
            true
        } else {
            let insert_pos = self
                .materials
                .iter()
                .position(|m| m.is_builtin)
                .unwrap_or(self.materials.len());
            self.materials.insert(insert_pos, preset);
            false
        };
        self.sort_user_before_defaults();
        self.save_db()?;
        Ok(updated)
    }

    pub fn remove_material(&mut self, idx: usize) -> Result<(), String> {
        if idx < self.materials.len() {
            self.materials.remove(idx);
            self.save_db()?;
        }
        Ok(())
    }

    /// Save the document's current layer parameters into the internal DB under `name`.
    pub fn save_layer_config(
        &mut self,
        name: impl Into<String>,
        doc: &Document,
    ) -> Result<bool, String> {
        let entry = LayerConfigEntry {
            name: name.into(),
            layers: doc.layers.iter().map(SavedLayerSpec::from).collect(),
        };
        let updated = if let Some(existing) = self
            .layer_configs
            .iter_mut()
            .find(|c| c.name.eq_ignore_ascii_case(&entry.name))
        {
            *existing = entry;
            true
        } else {
            self.layer_configs.push(entry);
            false
        };
        self.save_db()?;
        Ok(updated)
    }

    pub fn remove_layer_config(&mut self, idx: usize) -> Result<(), String> {
        if idx < self.layer_configs.len() {
            self.layer_configs.remove(idx);
            self.save_db()?;
        }
        Ok(())
    }

    /// Apply a saved layer configuration from the DB onto `doc.layers` while preserving entities.
    pub fn apply_layer_config(&self, idx: usize, doc: &mut Document) -> Option<String> {
        let entry = self.layer_configs.get(idx)?;
        apply_layer_specs(&entry.layers, doc);
        Some(entry.name.clone())
    }

    /// Export the document's current layer parameters to a standalone JSON file.
    pub fn export_layers_to_file(doc: &Document, path: &Path) -> Result<(), String> {
        let specs: Vec<SavedLayerSpec> = doc.layers.iter().map(SavedLayerSpec::from).collect();
        let json = serde_json::to_string_pretty(&specs)
            .map_err(|e| format!("Failed to serialize layer configuration: {}", e))?;
        std::fs::write(path, json)
            .map_err(|e| format!("Failed to save layer configuration: {}", e))
    }

    /// Load layer parameters from a standalone JSON file and apply them to `doc.layers`.
    pub fn import_layers_from_file(doc: &mut Document, path: &Path) -> Result<usize, String> {
        let raw = std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read layer configuration: {}", e))?;
        let specs: Vec<SavedLayerSpec> = serde_json::from_str(&raw)
            .map_err(|e| format!("Invalid layer configuration format: {}", e))?;
        let count = specs.len();
        apply_layer_specs(&specs, doc);
        Ok(count)
    }
}

fn apply_layer_specs(specs: &[SavedLayerSpec], doc: &mut Document) {
    for (idx, spec) in specs.iter().enumerate() {
        if let Some(layer) = doc
            .layers
            .iter_mut()
            .find(|l| l.name.eq_ignore_ascii_case(&spec.name))
        {
            layer.visible = spec.visible;
            layer.color = spec.color;
            layer.params = spec.params.clone();
        } else if let Some(layer) = doc.layers.get_mut(idx) {
            layer.visible = spec.visible;
            layer.params = spec.params.clone();
        } else {
            let mut layer = Layer::new(spec.name.clone(), spec.color);
            layer.visible = spec.visible;
            layer.params = spec.params.clone();
            doc.layers.push(layer);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Entity;
    use kurbo::Point;

    #[test]
    fn test_material_db_roundtrip_and_layer_config_apply() {
        let temp_dir = std::env::temp_dir().join(format!(
            "tzlaser_mat_test_{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let db_file = temp_dir.join("materials_db.json");

        let mut db = MaterialLibrary::load_or_init(Some(db_file.clone()));
        assert!(!db.materials.is_empty());
        assert!(db_file.exists());

        // Upsert a custom material preset (must appear before built-in defaults)
        let custom = MaterialPreset {
            name: "Brass 1mm (N2)".to_string(),
            material: "Brass".to_string(),
            thickness_mm: 1.0,
            params: CuttingParams {
                speed_mm_s: 42.5,
                power_percent: 95.0,
                pierce_time_ms: 280,
                gas_type: "N2".to_string(),
                cut_height_mm: 1.1,
                up_height_mm: 20.0,
                gas_between_paths: false,
                frequency_hz: 4500,
            },
            notes: "Test brass".to_string(),
            is_builtin: false,
        };
        assert!(!db.upsert_material(custom.clone()).unwrap());
        assert_eq!(db.materials[0].name, "Brass 1mm (N2)");
        assert!(!db.materials[0].is_builtin);
        assert!(db.materials[1].is_builtin);

        // Save current document layers into DB while preserving entities on apply
        let mut doc = Document::new();
        doc.layers[0].entities.push(Entity::Circle {
            center: Point::new(50.0, 50.0),
            radius: 15.0,
        });
        doc.layers[0].params = custom.params.clone();
        assert!(!db.save_layer_config("Brass Setup", &doc).unwrap());

        // Reload DB from disk and verify persistence
        let reloaded = MaterialLibrary::load_or_init(Some(db_file.clone()));
        assert!(reloaded
            .materials
            .iter()
            .any(|m| m.name == "Brass 1mm (N2)" && (m.params.speed_mm_s - 42.5).abs() < 1e-6));
        assert_eq!(reloaded.layer_configs.len(), 1);

        // Mutate doc speed, then apply saved layer config and check entities remain intact
        doc.layers[0].params.speed_mm_s = 199.0;
        let applied = reloaded.apply_layer_config(0, &mut doc);
        assert_eq!(applied.as_deref(), Some("Brass Setup"));
        assert!((doc.layers[0].params.speed_mm_s - 42.5).abs() < 1e-6);
        assert_eq!(doc.layers[0].entities.len(), 1);

        // Export and import standalone file
        let export_file = temp_dir.join("layers.json");
        MaterialLibrary::export_layers_to_file(&doc, &export_file).unwrap();
        doc.layers[0].params.speed_mm_s = 12.0;
        let count = MaterialLibrary::import_layers_from_file(&mut doc, &export_file).unwrap();
        assert!(count >= 1);
        assert!((doc.layers[0].params.speed_mm_s - 42.5).abs() < 1e-6);

        let _ = std::fs::remove_dir_all(&temp_dir);
    }
}

