//! File → Export: a small window to pick the character and units, then the file.
//!
//! The motion comes from the simulation when the project uses physics (finishing it
//! first if it's still running), otherwise from the keys; see `io::motion`.

use eframe::egui::{self, Align2, RichText};

use super::app::NeuroMotionApp;
use super::theme::grid;
use crate::document::{CharacterId, CharacterKind};
use crate::io::bvh;
use crate::io::fbx::{self, Encoding};
use crate::io::motion::{self, Units};
use crate::physics::Simulation;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    Bvh,
    Fbx,
}

impl Format {
    fn label(self) -> &'static str {
        match self {
            Format::Bvh => "BVH",
            Format::Fbx => "FBX",
        }
    }

    fn extension(self) -> &'static str {
        match self {
            Format::Bvh => bvh::FILE_EXTENSION,
            Format::Fbx => fbx::FILE_EXTENSION,
        }
    }
}

pub struct ExportDialog {
    format: Format,
    character: CharacterId,
    units: Units,
    encoding: Encoding,
}

impl NeuroMotionApp {
    /// Opens the export window for the selected character (or the first one).
    pub(super) fn open_export(&mut self, format: Format) {
        let project = &self.doc.project;
        let fallback = project
            .characters
            .iter()
            .find(|c| c.kind != CharacterKind::Environment)
            .or(project.characters.first())
            .map(|c| c.id);
        let Some(character) = self.selected_character().or(fallback) else {
            self.status = "There is no character to export.".into();
            return;
        };
        // Remember the last choices while the app runs.
        let (units, encoding) = self.export.as_ref().map_or_else(Default::default, |d| (d.units, d.encoding));
        self.export = Some(ExportDialog { format, character, units, encoding });
    }

    pub(super) fn export_window(&mut self, ctx: &egui::Context) {
        let Some(dialog) = &mut self.export else { return };
        let project = &self.doc.project;
        let (mut export, mut cancel) = (false, false);
        let physics = Simulation::is_needed(project);
        egui::Window::new(format!("Export {}", dialog.format.label()))
            .collapsible(false)
            .resizable(false)
            .anchor(Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                grid(ui, "export_options", |ui| {
                    ui.label("Character");
                    let current = project.character(dialog.character).map_or("—", |c| c.name.as_str());
                    egui::ComboBox::from_id_salt("export_character").selected_text(current).show_ui(ui, |ui| {
                        for c in &project.characters {
                            ui.selectable_value(&mut dialog.character, c.id, format!("{}  ·  {}", c.name, c.kind.label().to_lowercase()));
                        }
                    });
                    ui.end_row();
                    let range = project.settings.range;
                    ui.label("Frames");
                    ui.label(format!("{} – {}  ({} frames at {:.0} fps)", range.start, range.end, range.len() + 1, project.settings.fps));
                    ui.end_row();
                    ui.label("Motion");
                    let rigged = project.character(dialog.character).is_some_and(|c| c.physics.is_some());
                    ui.label(match (physics, rigged) {
                        (true, true) => "Simulation",
                        (true, false) => "Keys (this character has no physics body)",
                        (false, _) => "Keys (the project isn't simulated)",
                    });
                    ui.end_row();
                    ui.label("Units");
                    egui::ComboBox::from_id_salt("export_units").selected_text(dialog.units.label()).show_ui(ui, |ui| {
                        for units in Units::ALL {
                            ui.selectable_value(&mut dialog.units, units, units.label());
                        }
                    });
                    ui.end_row();
                    if dialog.format == Format::Fbx {
                        ui.label("Encoding");
                        egui::ComboBox::from_id_salt("export_encoding").selected_text(dialog.encoding.label()).show_ui(ui, |ui| {
                            for encoding in Encoding::ALL {
                                ui.selectable_value(&mut dialog.encoding, encoding, encoding.label());
                            }
                        });
                        ui.end_row();
                    }
                });
                ui.add_space(6.0);
                ui.weak("The project's frame range and frame rate are used; change them under Project.");
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    export = ui.button(RichText::new("Export…").strong()).clicked();
                    cancel = ui.button("Cancel").clicked();
                });
            });
        if cancel {
            self.export = None;
        } else if export {
            self.export_file();
        }
    }

    /// Asks where to save, then writes the file.
    fn export_file(&mut self) {
        let Some(dialog) = &self.export else { return };
        let (format, units, encoding) = (dialog.format, dialog.units, dialog.encoding);
        let project = &self.doc.project;
        let Some(index) = project.character_index(dialog.character) else { return };
        let character = &project.characters[index];
        let suggested = format!("{} {}.{}", project.settings.name, character.name, format.extension());
        let Some(path) = rfd::FileDialog::new()
            .add_filter(format.label(), &[format.extension()])
            .set_file_name(suggested)
            .save_file()
        else {
            return;
        };
        self.export = None;

        // Exports always reflect the current document: finish the simulation first.
        let simulation = if Simulation::is_needed(project) {
            motion::finish(&mut self.simulation, project, self.doc.revision());
            Some(&self.simulation)
        } else {
            None
        };
        let motion = motion::sample(project, index, simulation);
        let result = match format {
            Format::Bvh => bvh::save(&path, &character.skeleton, &motion, units),
            Format::Fbx => fbx::save(&path, &character.skeleton, &motion, units, encoding),
        };
        self.status = match result {
            Ok(()) => format!(
                "Exported {} frames of {} ({}) to {}",
                motion.frames.len(),
                character.name,
                if motion.simulated { "simulated" } else { "keys" },
                path.display()
            ),
            Err(e) => format!("Couldn't export {}: {e}", path.display()),
        };
    }
}
