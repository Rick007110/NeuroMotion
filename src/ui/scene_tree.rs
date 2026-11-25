//! The scene tree: every character, prop and the environment, and under each its
//! physics bodies. Clicking a name selects it, clicking the arrow folds it. The tree
//! only reads the project; selecting is UI state and never creates an undo entry.

use eframe::egui::{self, RichText, Ui, collapsing_header::CollapsingState};

use super::selection::Selection;
use super::theme;
use crate::document::{Character, CharacterKind, Project};

/// Groups in the order they're listed, with the heading shown above each.
const GROUPS: [(CharacterKind, &str); 3] = [
    (CharacterKind::Character, "Characters"),
    (CharacterKind::Prop, "Props"),
    (CharacterKind::Environment, "Environment"),
];

pub fn show(ui: &mut Ui, project: &Project, selection: &mut Selection) {
    ui.add_space(6.0);
    ui.label(RichText::new("SCENE").small().strong().color(theme::ACCENT));
    ui.label(
        RichText::new(&project.settings.name)
            .size(17.0)
            .color(theme::TEXT),
    );
    ui.add_space(2.0);
    ui.separator();

    egui::ScrollArea::both()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            if project.characters.is_empty() {
                ui.weak("Nothing in this scene yet.");
            }
            for (kind, title) in GROUPS {
                let members: Vec<&Character> = project
                    .characters
                    .iter()
                    .filter(|c| c.kind == kind)
                    .collect();
                if members.is_empty() {
                    continue;
                }
                ui.add_space(4.0);
                ui.label(RichText::new(title).small().weak());
                for character in members {
                    character_node(ui, character, selection);
                }
            }
        });
}

fn character_node(ui: &mut Ui, character: &Character, selection: &mut Selection) {
    let selected = *selection == Selection::Character(character.id);
    let Some(rig) = &character.physics else {
        // Nothing to unfold: indent to line up with the labels that have an arrow.
        ui.horizontal(|ui| {
            ui.add_space(ui.spacing().indent);
            if ui.selectable_label(selected, &character.name).clicked() {
                *selection = Selection::Character(character.id);
            }
            ui.weak("no physics");
        });
        return;
    };

    let id = ui.make_persistent_id(("scene_tree", character.id));
    // Characters open on first sight; the environment's many pieces start folded.
    let open = character.kind != CharacterKind::Environment;
    CollapsingState::load_with_default_open(ui.ctx(), id, open)
        .show_header(ui, |ui| {
            if ui.selectable_label(selected, &character.name).clicked() {
                *selection = Selection::Character(character.id);
            }
            ui.weak(format!("{} bodies", rig.bodies.len()));
        })
        .body(|ui| {
            for (index, body) in rig.bodies.iter().enumerate() {
                let this = Selection::Body {
                    character: character.id,
                    body: index,
                };
                ui.horizontal(|ui| {
                    if ui
                        .selectable_label(*selection == this, &body.name)
                        .clicked()
                    {
                        *selection = this;
                    }
                    ui.weak(format!("{:.1} kg", body.mass));
                });
            }
        });
}
