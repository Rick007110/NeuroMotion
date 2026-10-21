//! Reading and writing project files.
//!
//! A `.nmproj` file is pretty-printed JSON: a small envelope (format tag + version)
//! around the serialized [`Project`]. The version lets later builds migrate old files.
//! [`open`] also accepts endorphin files, see [`endorphin`]. Motion leaves NeuroMotion
//! through the exporters: [`bvh`] (and [`motion`], which they share).

pub mod bvh;
pub mod endorphin;
pub mod motion;

use std::{fmt, fs, path::Path};

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::document::Project;

pub const FILE_EXTENSION: &str = "nmproj";
pub const FORMAT_VERSION: u32 = 2;
const FORMAT_TAG: &str = "neuromotion.project";

/// Extensions [`open`] understands, for file dialogs: NeuroMotion first, then endorphin.
pub const OPEN_EXTENSIONS: [&str; 3] = [FILE_EXTENSION, "ens", "nmc"];

/// The result of opening any supported file.
pub struct Opened {
    pub project: Project,
    /// `false` for imported files: saving must go to a new `.nmproj`.
    pub native: bool,
    /// What an importer converted or left out, for the user.
    pub notes: Vec<String>,
}

/// Opens a NeuroMotion project or an endorphin scene/character, chosen by extension.
pub fn open(path: &Path) -> Result<Opened, IoError> {
    let ext = path.extension().map(|e| e.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
    let imported = |import: endorphin::Import| Opened { project: import.project, native: false, notes: import.notes };
    match ext.as_str() {
        "ens" => endorphin::import_scene(path).map(imported),
        "nmc" => endorphin::import_character(path).map(imported),
        "nma" => Err(IoError::Format(
            "a pose needs a character to go on: open a scene or character first, then use File → Import Pose".into(),
        )),
        "nmb" | "nmw" => Err(IoError::Format(
            "endorphin stores behaviour and world files in a protected format that NeuroMotion can't read".into(),
        )),
        _ => Ok(Opened { project: load(path)?, native: true, notes: Vec::new() }),
    }
}

#[derive(Serialize)]
struct FileOut<'a> {
    format: &'a str,
    version: u32,
    project: &'a Project,
}

#[derive(Deserialize)]
struct FileIn {
    format: String,
    version: u32,
    project: Value,
}

#[derive(Debug)]
pub enum IoError {
    Io(std::io::Error),
    Json(serde_json::Error),
    Format(String),
}

impl fmt::Display for IoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IoError::Io(e) => write!(f, "file error: {e}"),
            IoError::Json(e) => write!(f, "could not parse project: {e}"),
            IoError::Format(msg) => f.write_str(msg),
        }
    }
}

impl std::error::Error for IoError {}

impl From<std::io::Error> for IoError {
    fn from(e: std::io::Error) -> Self {
        IoError::Io(e)
    }
}

impl From<serde_json::Error> for IoError {
    fn from(e: serde_json::Error) -> Self {
        IoError::Json(e)
    }
}

pub fn to_string(project: &Project) -> Result<String, IoError> {
    let file = FileOut {
        format: FORMAT_TAG,
        version: FORMAT_VERSION,
        project,
    };
    Ok(serde_json::to_string_pretty(&file)?)
}

pub fn from_str(text: &str) -> Result<Project, IoError> {
    let file: FileIn = serde_json::from_str(text)?;
    if file.format != FORMAT_TAG {
        return Err(IoError::Format(format!(
            "not a NeuroMotion project (format \"{}\")",
            file.format
        )));
    }
    if file.version > FORMAT_VERSION {
        return Err(IoError::Format(format!(
            "project was saved by a newer NeuroMotion (format v{}, this build reads up to v{FORMAT_VERSION})",
            file.version
        )));
    }
    let mut value = file.project;
    if file.version < 2 {
        migrate_v1(&mut value);
    }
    let mut project: Project = serde_json::from_value(value)?;
    project.normalize_ids();
    Ok(project)
}

/// v1 projects had a single top-level `skeleton`; v2 has `characters` and every track
/// names its character.
fn migrate_v1(project: &mut Value) {
    let Some(map) = project.as_object_mut() else { return };
    let ids = ["tracks", "clips"].iter().flat_map(|k| map.get(*k).and_then(Value::as_array)).flatten();
    let highest = ids.filter_map(|item| item.get("id").and_then(Value::as_u64)).max().unwrap_or(0);
    let character = highest.max(map.get("next_id").and_then(Value::as_u64).unwrap_or(0)) + 1;
    if let Some(skeleton) = map.remove("skeleton") {
        map.insert(
            "characters".into(),
            json!([{ "id": character, "name": "character01", "kind": "character", "skeleton": skeleton }]),
        );
    }
    if let Some(tracks) = map.get_mut("tracks").and_then(Value::as_array_mut) {
        for track in tracks.iter_mut().filter_map(Value::as_object_mut) {
            track.insert("character".into(), json!(character));
        }
    }
}

pub fn save(path: &Path, project: &Project) -> Result<(), IoError> {
    fs::write(path, to_string(project)?)?;
    Ok(())
}

pub fn load(path: &Path) -> Result<Project, IoError> {
    from_str(&fs::read_to_string(path)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::document::sample::placeholder_project;

    #[test]
    fn project_round_trips() {
        let project = placeholder_project();
        let text = to_string(&project).unwrap();
        assert_eq!(from_str(&text).unwrap(), project);
    }

    #[test]
    fn migrates_v1_projects() {
        let project = placeholder_project();
        let mut value = serde_json::to_value(&project).unwrap();
        let map = value.as_object_mut().unwrap();
        let skeleton = map["characters"][0]["skeleton"].clone();
        map.remove("characters");
        map.insert("skeleton".into(), skeleton);
        for track in map["tracks"].as_array_mut().unwrap() {
            track.as_object_mut().unwrap().remove("character");
        }
        let v1 = json!({ "format": FORMAT_TAG, "version": 1, "project": value }).to_string();

        let migrated = from_str(&v1).unwrap();
        assert_eq!(migrated.characters.len(), 1);
        assert_eq!(migrated.characters[0].skeleton, project.characters[0].skeleton);
        let character = migrated.characters[0].id;
        assert!(migrated.tracks.iter().all(|t| t.character == character));
        assert_eq!(migrated.evaluate(120.0), project.evaluate(120.0));
    }

    /// A v2 file as older builds wrote it (no physics, no event settings) still opens.
    #[test]
    fn reads_v2_files_from_older_builds() {
        let text = r#"{
          "format": "neuromotion.project",
          "version": 2,
          "project": {
            "settings": { "name": "Old", "fps": 30.0, "range": { "start": 0, "end": 60 } },
            "characters": [{
              "id": 1, "name": "character01", "kind": "character",
              "skeleton": { "name": "Two bones", "source": null, "joints": [
                { "name": "hips", "parent": null, "offset": [0.0, 0.95, 0.0] },
                { "name": "spine", "parent": 0, "offset": [0.0, 0.12, 0.0], "rest_rotation": [0.0, 0.0, 0.0, 1.0] }
              ] }
            }],
            "tracks": [ { "id": 2, "name": "Base", "character": 1, "muted": false } ],
            "clips": [{
              "id": 3,
              "props": { "name": "Bend", "track": 2, "start": 0, "duration": 60, "blend_in": 0, "blend_out": 0, "weight": 1.0, "enabled": true },
              "kind": "motion",
              "channels": [ { "joint": "spine", "property": "rot_x", "keys": [ { "frame": 0, "value": 20.0, "interp": "smooth" } ] } ]
            }],
            "next_id": 4
          }
        }"#;
        let project = from_str(text).unwrap();
        assert_eq!(project.characters[0].default_level, crate::document::SimLevel::Animated);
        assert!(project.characters[0].physics.is_none());
        assert_eq!(project.evaluate(30.0)[0].joints[1][0], 20.0);
        // Edits made by this build allocate ids past the loaded ones.
        let mut doc = crate::document::Document::new(project);
        assert!(doc.add_track(1, "Layer") > 3);
    }

    #[test]
    fn rejects_foreign_and_future_files() {
        let text = to_string(&placeholder_project()).unwrap();
        let foreign = text.replace(FORMAT_TAG, "something.else");
        assert!(matches!(from_str(&foreign), Err(IoError::Format(_))));
        let future = text.replacen("\"version\": 2", "\"version\": 99", 1);
        assert!(matches!(from_str(&future), Err(IoError::Format(_))));
    }
}
