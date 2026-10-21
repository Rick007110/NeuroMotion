//! Menu bar: File, Edit, View, Help.

use eframe::egui::{self, Button, Ui};

use super::actions::Action;
use super::app::NeuroMotionApp;
use super::selection::Selection;
use super::shortcuts;

impl NeuroMotionApp {
    pub(super) fn menu_bar(&mut self, ui: &mut Ui, actions: &mut Vec<Action>) {
        egui::menu::bar(ui, |ui| {
            ui.menu_button("File", |ui| {
                item(ui, actions, Action::NewProject, "New Project", true);
                item(ui, actions, Action::OpenProject, "Open…", true);
                item(ui, actions, Action::OpenDemo, "Open Placeholder Demo", true);
                item(ui, actions, Action::OpenPhysicsDemo, "Open Physics Demo", true);
                item(ui, actions, Action::ImportPose, "Import Pose (.nma)…", true);
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
                item(ui, actions, Action::DuplicateClip, "Duplicate Clip", self.selection.clip().is_some());
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

/// A menu entry that queues `action`, showing its shortcut from the bindings table.
fn item(ui: &mut Ui, actions: &mut Vec<Action>, action: Action, label: &str, enabled: bool) {
    let button = Button::new(label).shortcut_text(shortcuts::text_for(ui.ctx(), action));
    if ui.add_enabled(enabled, button).clicked() {
        actions.push(action);
        ui.close_menu();
    }
}
