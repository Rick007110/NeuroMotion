//! Menu bar: File, Edit, View, Help.

use eframe::egui::{self, Button, Ui};

use super::actions::Action;
use super::app::NeuroMotionApp;
use super::selection::Selection;
use super::shortcuts;
use crate::document::events::NewClip;
use crate::document::{Character, TrackId};

impl NeuroMotionApp {
    pub(super) fn menu_bar(&mut self, ui: &mut Ui, actions: &mut Vec<Action>) {
        egui::menu::bar(ui, |ui| {
            ui.menu_button("File", |ui| {
                item(ui, actions, Action::NewProject, "New Project", true);
                item(ui, actions, Action::OpenProject, "Open…", true);
                item(ui, actions, Action::OpenDemo, "Open Placeholder Demo", true);
                item(ui, actions, Action::OpenPhysicsDemo, "Open Physics Demo", true);
                item(ui, actions, Action::ImportPose, "Import Pose (.nma)…", true);
                ui.menu_button("Export", |ui| {
                    item(ui, actions, Action::ExportBvh, "BVH…", true);
                    item(ui, actions, Action::ExportFbx, "FBX…", true);
                });
                ui.separator();
                item(ui, actions, Action::SaveProject, "Save", true);
                item(ui, actions, Action::SaveProjectAs, "Save As…", true);
                ui.separator();
                item(ui, actions, Action::Quit, "Exit", true);
            });

            ui.menu_button("Edit", |ui| {
                let undo = self.doc.undo_label().map_or("Undo".to_owned(), |l| format!("Undo {l}"));
                let redo = self.doc.redo_label().map_or("Redo".to_owned(), |l| format!("Redo {l}"));
                item(ui, actions, Action::Undo, &undo, self.doc.can_undo());
                item(ui, actions, Action::Redo, &redo, self.doc.can_redo());
                ui.separator();
                item(ui, actions, Action::AddKey, "Add Key at Playhead", self.selection.channel().is_some());
                ui.menu_button("Add", |ui| {
                    let character = self.target_character().and_then(|id| self.doc.project.character(id));
                    add_clip_items(ui, actions, character, None);
                });
                item(ui, actions, Action::DuplicateClip, "Duplicate Clip", self.selection.clip().is_some());
                item(ui, actions, Action::BakeSimulation, "Bake Simulation", self.can_bake());
                item(ui, actions, Action::Delete, "Delete", self.selection.is_deletable());
                item(ui, actions, Action::Deselect, "Select None", self.selection != Selection::None);
            });

            ui.menu_button("View", |ui| {
                ui.checkbox(&mut self.show_inspector, "Inspector");
                ui.checkbox(&mut self.show_timeline, "Timeline");
                ui.separator();
                ui.checkbox(&mut self.viewport.show_grid, "Ground Grid");
                ui.checkbox(&mut self.viewport.show_joint_names, "Joint Names");
                ui.checkbox(&mut self.viewport.show_bodies, "Physics Bodies");
                item(ui, actions, Action::ResetCamera, "Reset Camera", true);
                ui.separator();
                item(ui, actions, Action::FitTimeline, "Fit Timeline to Range", true);
            });

            ui.menu_button("Help", |ui| {
                item(ui, actions, Action::ShowShortcuts, "Keyboard Shortcuts", true);
                item(ui, actions, Action::ShowAbout, "About NeuroMotion", true);
            });
        });
    }
}

/// One entry per kind of clip, for `character` (none: all disabled). `at` places the
/// clip on a track at a frame; `None` uses the selection and the playhead.
pub(super) fn add_clip_items(ui: &mut Ui, actions: &mut Vec<Action>, character: Option<&Character>, at: Option<(TrackId, i32)>) {
    for kind in NewClip::ALL {
        if kind == NewClip::Simulation {
            ui.separator();
        }
        let rigged = character.is_some_and(|c| c.physics.is_some());
        let enabled = character.is_some() && (rigged || !kind.needs_physics());
        let label = match kind {
            NewClip::Motion => "Empty Motion Clip",
            NewClip::Simulation => "Simulation Event",
            NewClip::Force => "Force Event",
            NewClip::Transition => "Transition Event",
            NewClip::ActivePose => "Active Pose Event",
            NewClip::Constraint => "Constraint Event",
        };
        let response = ui.add_enabled(enabled, Button::new(label));
        let response = if enabled {
            response.on_hover_text(kind.description())
        } else {
            response.on_disabled_hover_text("This character has no physics body")
        };
        if response.clicked() {
            actions.push(Action::AddClip { kind, at });
            ui.close_menu();
        }
    }
}

/// A menu entry that queues `action`, showing its shortcut from the bindings table.
fn item(ui: &mut Ui, actions: &mut Vec<Action>, action: Action, label: &str, enabled: bool) {
    let button = Button::new(label).shortcut_text(shortcuts::text_for(ui.ctx(), action));
    if ui.add_enabled(enabled, button).clicked() {
        actions.push(action);
        ui.close_menu();
    }
}
