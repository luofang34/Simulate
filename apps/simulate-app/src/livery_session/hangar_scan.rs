//! Desktop `$HOME/hangar` and `SIMULATE_HANGAR_DIR` external aircraft package discovery.

use std::path::{Path, PathBuf};

use tracing::{info, warn};

use super::{ENV_HANGAR_DIR, ENV_IMPORT_LIVERY, ENV_IMPORT_VARIANT, LiverySession};

const BUILTIN_IDS: [&str; 3] = ["c172s", "c172p", "p38g"];

impl LiverySession {
    /// Scans `dir` for immediate child directories containing `syspack.json`, importing each
    /// non-colliding valid aircraft package in sorted lexical order.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn import_hangar_dir(&mut self, dir: &Path) -> Vec<String> {
        let Ok(entries) = std::fs::read_dir(dir) else {
            return Vec::new();
        };
        let mut candidates: Vec<PathBuf> = entries
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|path| path.is_dir() && path.join("syspack.json").is_file())
            .collect();
        candidates.sort();
        let mut imported = Vec::new();
        for pkg_dir in candidates {
            let folder_name = pkg_dir
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or_default();
            if BUILTIN_IDS.contains(&folder_name) {
                warn!(path = %pkg_dir.display(), "skipping external package colliding with built-in id");
                continue;
            }
            match self.import_variant_dir(&pkg_dir) {
                Ok(id) => {
                    info!(aircraft_id = %id, path = %pkg_dir.display(), "loaded external hangar plugin");
                    imported.push(id);
                }
                Err(err) => {
                    warn!(error = %err, path = %pkg_dir.display(), "failed loading hangar plugin directory");
                }
            }
        }
        imported
    }

    pub(super) fn import_environment_paths(&mut self) {
        if let Some(raw) = std::env::var_os(ENV_HANGAR_DIR) {
            for dir in std::env::split_paths(&raw) {
                let _ = self.import_hangar_dir(&dir);
            }
        } else {
            #[cfg(not(test))]
            if let Some(home) = std::env::var_os("HOME") {
                let default_hangar = PathBuf::from(home).join("hangar");
                if default_hangar.is_dir() {
                    let _ = self.import_hangar_dir(&default_hangar);
                }
            }
        }
        if let Some(var_dirs) = std::env::var_os(ENV_IMPORT_VARIANT) {
            for path in std::env::split_paths(&var_dirs) {
                match self.import_variant_dir(&path) {
                    Ok(id) => {
                        info!(aircraft_id = %id, path = %path.display(), "imported external variant pack");
                    }
                    Err(err) => {
                        warn!(error = %err, path = %path.display(), "external variant import failed");
                    }
                }
            }
        }
        if let Some(liv_paths) = std::env::var_os(ENV_IMPORT_LIVERY) {
            for path in std::env::split_paths(&liv_paths) {
                match self.import_livery_path(&path, None) {
                    Ok((id, liv)) => info!(
                        aircraft_id = %id,
                        tail = %liv.tail_code,
                        path = %path.display(),
                        "imported external aircraft livery"
                    ),
                    Err(err) => {
                        warn!(error = %err, path = %path.display(), "external livery import failed");
                    }
                }
            }
        }
    }
}
