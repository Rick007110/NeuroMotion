//! Inspector controls for physics event clips: simulation switches, forces,
//! transitions, active poses, constraints (and imported behaviours).
//!
//! Every control edits the clip's settings map under the keys the importer and the
//! simulation use (see `document::events`), as one undoable edit per gesture. Settings
//! the controls don't cover are listed read-only underneath.

use std::collections::BTreeMap;

use eframe::egui::{self, DragValue, RichText, Slider, Ui};
use glam::Vec3;

use super::actions::Action;
use super::inspector::Gesture;
use super::theme::{grid, section};
use crate::document::events::{self, BodyRef, LockType, keys};
use crate::document::{Character, Clip, ClipId, Document, Edit, Project, SimLevel};

type Settings = BTreeMap<String, String>;

pub(super) fn show(ui: &mut Ui, doc: &mut Document, id: ClipId, event: &str, actions: &mut Vec<Action>) {
    let project = &doc.project;
    let Some(clip) = project.clip(id) else { return };
    let before = clip.source.clone();
    let character = project.clip_character(clip);
    let mut s = before.clone();
    let mut g = Gesture::default();
    match event {
        events::SIMULATION => simulation(ui, &mut s, &mut g),
        events::FORCE => force(ui, &mut s, &mut g, character),
        events::TRANSITION => transition(ui, &mut s),
        events::ACTIVE_POSE => active_pose(ui, &mut s, &mut g, clip, actions),
        events::CONSTRAINT => constraint(ui, &mut s, &mut g, project),
        events::BEHAVIOUR => behaviour(ui, &mut s, &mut g),
        _ => {}
    }
    other_settings(ui, &before, events::edited_keys(event));
    let edit = (s != before).then(|| Edit::ClipSource { clip: id, before, after: s });
    g.commit(doc, "Edit event settings", edit);
}

fn simulation(ui: &mut Ui, s: &mut Settings, g: &mut Gesture) {
    section(ui, "Simulation", |ui| {
        grid(ui, "event_simulation", |ui| {
            ui.label("Switch to");
            level_combo(ui, "simulation_level", s, keys::PHYSICS_LEVEL, SimLevel::Simulated);
            ui.end_row();
            ui.label("Momentum kept");
            number(ui, g, s, keys::MOMENTUM_SCALE, 1.0, |ui, v| ui.add(Slider::new(v, 0.0..=1.0)))
                .on_hover_text("How much of the animation's velocity carries into physics");
            ui.end_row();
        });
        ui.weak("Switches at the clip's first frame. The character stays at this level until another simulation or transition event.");
    });
}

fn force(ui: &mut Ui, s: &mut Settings, g: &mut Gesture, character: Option<&Character>) {
    section(ui, "Force", |ui| {
        let velocity = events::vector(s, keys::VELOCITY).unwrap_or(Vec3::ZERO);
        let (mut speed, mut heading, mut elevation) = events::push_parts(velocity);
        let mut changed = false;
        grid(ui, "event_force", |ui| {
            ui.label("Strength");
            let r = ui.add(DragValue::new(&mut speed).range(0.0..=100.0).speed(0.05).max_decimals(2).suffix(" m/s"));
            changed |= r.changed();
            g.drag(&r.on_hover_text("The change of velocity the push gives the body"));
            ui.end_row();
            ui.label("Heading");
            let r = ui.add(DragValue::new(&mut heading).range(-180.0..=180.0).speed(1.0).max_decimals(1).suffix("°"));
            changed |= r.changed();
            g.drag(&r.on_hover_text("Direction around the up axis: 0° = +Z (the way NeuroMotion's own characters face), 90° = +X, 180° = −Z"));
            ui.end_row();
            ui.label("Elevation");
            let r = ui.add(DragValue::new(&mut elevation).range(-90.0..=90.0).speed(1.0).max_decimals(1).suffix("°"));
            changed |= r.changed();
            g.drag(&r.on_hover_text("Degrees above horizontal"));
            ui.end_row();
            ui.label("Body");
            body_combo(ui, s, character);
            ui.end_row();
            ui.label("Velocity change");
            let v = events::vector(s, keys::VELOCITY).unwrap_or(Vec3::ZERO);
            ui.weak(format!("({:.2}, {:.2}, {:.2}) m/s", v.x, v.y, v.z));
            ui.end_row();
        });
        if changed {
            events::set_vector(s, keys::VELOCITY, events::push(speed, heading, elevation));
        }
        ui.weak("A one-frame push at the clip's first frame. It only acts while the character is simulated.");
    });
}

/// Which of the character's bodies a force pushes ("All bodies" when none is named).
fn body_combo(ui: &mut Ui, s: &mut Settings, character: Option<&Character>) {
    let current = events::targets(s);
    let text = match current.as_slice() {
        [] => "All bodies".to_owned(),
        [one] => one.body.clone(),
        many => format!("{} bodies", many.len()),
    };
    let (Some(character), Some(rig)) = (character, character.and_then(|c| c.physics.as_ref())) else {
        ui.weak(text);
        return;
    };
    let mut chosen: Option<Vec<BodyRef>> = None;
    egui::ComboBox::from_id_salt("force_body").selected_text(text).height(320.0).show_ui(ui, |ui| {
        if ui.selectable_label(current.is_empty(), "All bodies").clicked() {
            chosen = Some(Vec::new());
        }
        for body in &rig.bodies {
            let selected = current.len() == 1 && current[0].body == body.name;
            if ui.selectable_label(selected, &body.name).clicked() {
                chosen = Some(vec![BodyRef { character: character.name.clone(), body: body.name.clone() }]);
            }
        }
    });
    if let Some(targets) = chosen.filter(|t| *t != current) {
        events::set_targets(s, &targets);
    }
}

fn transition(ui: &mut Ui, s: &mut Settings) {
    section(ui, "Transition", |ui| {
        grid(ui, "event_transition", |ui| {
            ui.label("Then");
            level_combo(ui, "transition_level", s, keys::REQUIRED_LEVEL, SimLevel::Animated);
            ui.end_row();
        });
        ui.weak("Blends from the simulation back to the animation over the length of the clip, then switches to this level.");
    });
}

fn active_pose(ui: &mut Ui, s: &mut Settings, g: &mut Gesture, clip: &Clip, actions: &mut Vec<Action>) {
    section(ui, "Active pose", |ui| {
        grid(ui, "event_pose", |ui| {
            ui.label("Strength");
            number(ui, g, s, keys::STRENGTH, 1.0, |ui, v| ui.add(DragValue::new(v).range(0.0..=20.0).speed(0.02).max_decimals(2)))
                .on_hover_text("Multiplies the muscles' strength while they hold the pose");
            ui.end_row();
            ui.label("Ease in");
            number(ui, g, s, keys::ENTRY_BLEND, 0.0, |ui, v| ui.add(DragValue::new(v).range(0.0..=10.0).speed(0.01).max_decimals(2).suffix(" s")))
                .on_hover_text("Seconds the muscles take to reach full strength");
            ui.end_row();
            ui.label("Joints held");
            let mut joints: Vec<&str> = clip.channels.iter().map(|c| c.joint.as_str()).collect();
            joints.dedup();
            ui.label(joints.len().to_string());
            ui.end_row();
        });
        if ui.button("Capture pose at playhead").on_hover_text("Hold the pose the viewport shows now").clicked() {
            actions.push(Action::CapturePose(clip.id));
        }
        ui.weak("While the character is simulated, its muscles pull towards this pose.");
    });
}

fn constraint(ui: &mut Ui, s: &mut Settings, g: &mut Gesture, project: &Project) {
    section(ui, "Constraint", |ui| {
        let current = s.get(keys::LOCK_TYPE).and_then(|v| LockType::from_endorphin(v));
        let mut lock = current.unwrap_or(LockType::Lock);
        let original = lock;
        grid(ui, "event_constraint", |ui| {
            ui.label("Type");
            egui::ComboBox::from_id_salt("lock_type").selected_text(lock.label()).show_ui(ui, |ui| {
                for option in LockType::ALL {
                    ui.selectable_value(&mut lock, option, option.label());
                }
            });
            ui.end_row();
            ui.label("Position strength");
            let uses_position = lock != LockType::LockOrientation;
            ui.add_enabled_ui(uses_position, |ui| vector(ui, g, s, keys::POSITION_STRENGTH));
            ui.end_row();
            ui.label("Rotation strength");
            let uses_rotation = lock != LockType::LockPosition;
            ui.add_enabled_ui(uses_rotation, |ui| vector(ui, g, s, keys::ANGULAR_STRENGTH));
            ui.end_row();
        });
        if lock != original {
            s.insert(keys::LOCK_TYPE.to_owned(), lock.endorphin_name().to_owned());
        }

        ui.add_space(4.0);
        ui.label(RichText::new("Bodies").strong());
        let mut targets = events::targets(s);
        let mut remove = None;
        for (i, target) in targets.iter().enumerate() {
            ui.horizontal(|ui| {
                if ui.small_button("−").on_hover_text("Remove from the constraint").clicked() {
                    remove = Some(i);
                }
                ui.label(target.label());
            });
        }
        let mut add = None;
        egui::ComboBox::from_id_salt("constraint_add_body").selected_text("Add body…").height(320.0).show_ui(ui, |ui| {
            for character in project.characters.iter().filter(|c| c.physics.is_some()) {
                for body in character.physics.iter().flat_map(|r| &r.bodies) {
                    let candidate = BodyRef { character: character.name.clone(), body: body.name.clone() };
                    let listed = targets.contains(&candidate);
                    if ui.add_enabled(!listed, egui::Button::new(candidate.label()).frame(false)).clicked() {
                        add = Some(candidate);
                    }
                }
            }
        });
        if let Some(i) = remove {
            targets.remove(i);
        }
        if remove.is_some() || add.is_some() {
            targets.extend(add);
            events::set_targets(s, &targets);
        }
        ui.weak(match lock {
            LockType::Join => "Each body is held to the last one in the list, as they are when the clip starts.",
            _ => "Each body is held where it is when the clip starts, until the clip ends.",
        });
        ui.weak("Strengths are springs along each world axis (N/m and N·m/rad); 0 leaves an axis free.");
    });
}

fn behaviour(ui: &mut Ui, s: &mut Settings, g: &mut Gesture) {
    section(ui, "Behaviour", |ui| {
        grid(ui, "event_behaviour", |ui| {
            ui.label("Behaviour");
            let mut name = s.get(keys::BEHAVIOUR_NAME).cloned().unwrap_or_default();
            let r = ui.text_edit_singleline(&mut name);
            g.text(&r);
            if r.changed() {
                s.insert(keys::BEHAVIOUR_NAME.to_owned(), name);
            }
            ui.end_row();
        });
        ui.weak("endorphin's behaviours are protected and aren't simulated yet; the event is kept for when NeuroMotion has its own.");
    });
}

/// Settings the controls above don't cover, read-only.
fn other_settings(ui: &mut Ui, source: &Settings, edited: &[&str]) {
    let rest: Vec<(&String, &String)> = source.iter().filter(|(k, _)| !edited.contains(&k.as_str())).collect();
    if rest.is_empty() {
        return;
    }
    egui::CollapsingHeader::new(RichText::new("Other imported settings").strong())
        .default_open(false)
        .show(ui, |ui| {
            egui::Grid::new("event_other_settings").striped(true).num_columns(2).show(ui, |ui| {
                for (key, value) in rest {
                    ui.label(key);
                    ui.label(RichText::new(value).monospace());
                    ui.end_row();
                }
            });
        });
}

/// A simulation-level picker for an endorphin level setting.
fn level_combo(ui: &mut Ui, id: &str, s: &mut Settings, key: &str, default: SimLevel) {
    let original = events::level(s, key).unwrap_or(default);
    let mut level = original;
    egui::ComboBox::from_id_salt(id).selected_text(level.label()).show_ui(ui, |ui| {
        for option in SimLevel::ALL {
            ui.selectable_value(&mut level, option, option.label());
        }
    });
    if level != original {
        events::set_level(s, key, level);
    }
}

/// A numeric setting edited with `widget`; only written back when the user changes it.
fn number(
    ui: &mut Ui,
    g: &mut Gesture,
    s: &mut Settings,
    key: &str,
    default: f32,
    widget: impl FnOnce(&mut Ui, &mut f32) -> egui::Response,
) -> egui::Response {
    let mut value = events::number(s, key).unwrap_or(default);
    let response = widget(ui, &mut value);
    g.drag(&response);
    if response.changed() {
        events::set_number(s, key, value);
    }
    response
}

/// An `"x, y, z"` setting as three fields.
fn vector(ui: &mut Ui, g: &mut Gesture, s: &mut Settings, key: &str) {
    let mut v = events::vector(s, key).unwrap_or(Vec3::ZERO);
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 4.0;
        for (axis, label) in ["X", "Y", "Z"].into_iter().enumerate() {
            let r = ui.add(DragValue::new(&mut v[axis]).range(0.0..=1e6).speed(50.0).max_decimals(0));
            changed |= r.changed();
            g.drag(&r.on_hover_text(format!("Along world {label}")));
        }
    });
    if changed {
        events::set_vector(s, key, v);
    }
}
