//! Physics in the inspector: the project's scene-wide settings (editable, on the project
//! page) and one physics body (read-only, from the scene tree).

use eframe::egui::{self, DragValue, RichText, Ui};

use super::inspector::{Gesture, heading};
use super::selection::Selection;
use super::theme::{grid, section};
use crate::document::physics::{
    BodyDef, ContactModel, Geometry, Interaction, JointDef, PhysicsSettings,
};
use crate::document::{CharacterId, Document, Edit, Project};

/// Surface materials a shape can have, as the pair table offers them.
const MATERIALS: [&str; 8] = [
    "default", "ice", "rubber", "skin", "sponge", "steel", "tarmac", "velcro",
];

/// Friction values are Coulomb coefficients times this (docs/physics-design.md, lesson 21).
const FRICTION_PER_MU: f32 = 100.0;

fn contact_label(model: ContactModel) -> &'static str {
    match model {
        ContactModel::ForceCapped => "Force cap",
        ContactModel::Coulomb => "Coulomb",
    }
}

fn contact_hint(model: ContactModel) -> &'static str {
    match model {
        ContactModel::ForceCapped => {
            "Friction is the most force, in newtons, one contact can apply, whatever the load."
        }
        ContactModel::Coulomb => {
            "Friction is a coefficient times 100: the grip scales with how hard surfaces press."
        }
    }
}

/// Physics ticks the simulation runs per animation frame (`sim.rs` rounds the same way).
fn ticks_per_frame(rate: f32, fps: f32) -> usize {
    (rate / fps.max(1.0)).round().max(1.0) as usize
}

fn capitalised(name: &str) -> String {
    let mut chars = name.chars();
    chars
        .next()
        .map(|c| c.to_uppercase().chain(chars).collect())
        .unwrap_or_default()
}

/// How the project spells a material (shapes and ground), so a pair picked here matches
/// the shapes it is meant for: lookups compare names exactly.
fn spelling(project: &Project, material: &str) -> String {
    let ground = project.physics.ground.iter().map(|g| g.material.as_str());
    let shapes = project
        .characters
        .iter()
        .filter_map(|c| c.physics.as_ref())
        .flat_map(|rig| rig.bodies.iter())
        .flat_map(|b| b.shapes.iter().map(|s| s.material.as_str()));
    ground
        .chain(shapes)
        .find(|m| m.eq_ignore_ascii_case(material))
        .unwrap_or(material)
        .to_owned()
}

/// A drop-down over the known materials (plus whatever the file uses). `spell` maps a
/// picked name to the project's spelling of it.
fn material_combo(
    ui: &mut Ui,
    id: impl std::hash::Hash,
    value: &mut String,
    spell: &dyn Fn(&str) -> String,
) {
    egui::ComboBox::from_id_salt(id)
        .width(78.0)
        .selected_text(capitalised(value))
        .show_ui(ui, |ui| {
            let known = MATERIALS
                .iter()
                .any(|m| m.eq_ignore_ascii_case(value.as_str()));
            if !known {
                ui.selectable_value(value, value.clone(), capitalised(value));
            }
            for name in MATERIALS {
                let picked = value.eq_ignore_ascii_case(name);
                if ui.selectable_label(picked, capitalised(name)).clicked() && !picked {
                    *value = spell(name);
                }
            }
        });
}

/// The editable "Physics" group of the project page.
pub(super) fn settings_group(ui: &mut Ui, doc: &mut Document) {
    let before = doc.project.physics.clone();
    let fps = doc.project.settings.fps;
    let project = &doc.project;
    let spell = |name: &str| spelling(project, name);
    let mut p = before.clone();
    let mut g = Gesture::default();

    section(ui, "Physics", |ui| {
        grid(ui, "physics_general", |ui| {
            ui.label("Simulation rate");
            g.drag(
                &ui.add(
                    DragValue::new(&mut p.rate)
                        .range(30.0..=960.0)
                        .speed(1.0)
                        .suffix(" Hz"),
                ),
            );
            ui.end_row();
            ui.label("Per frame");
            let ticks = ticks_per_frame(p.rate, fps);
            ui.weak(format!(
                "{ticks} steps · runs at {:.0} Hz",
                ticks as f32 * fps
            ))
            .on_hover_text(
                "Each animation frame is divided into a whole number of steps, so the \
                     rate you get is a multiple of the frame rate.",
            );
            ui.end_row();

            ui.label("Gravity");
            ui.horizontal(|ui| {
                for (axis, value) in ["x", "y", "z"].into_iter().zip(p.gravity.iter_mut()) {
                    g.drag(&ui.add(DragValue::new(value).speed(0.05).prefix(format!("{axis} "))));
                }
            });
            ui.end_row();
            ui.label("");
            ui.weak("metres per second squared");
            ui.end_row();

            ui.label("Contact model");
            egui::ComboBox::from_id_salt("contact_model")
                .selected_text(contact_label(p.contact))
                .show_ui(ui, |ui| {
                    for model in [ContactModel::ForceCapped, ContactModel::Coulomb] {
                        ui.selectable_value(&mut p.contact, model, contact_label(model));
                    }
                })
                .response
                .on_hover_text(contact_hint(p.contact));
            ui.end_row();

            ui.label("Ground");
            match &p.ground {
                Some(ground) => ui.weak(format!(
                    "{} at {:.2} m",
                    capitalised(&ground.material),
                    ground.height
                )),
                None => ui.weak("none"),
            };
            ui.end_row();
        });
        ui.weak("Simulation length is the project's frame range, above.");

        egui::CollapsingHeader::new("Material interactions")
            .default_open(false)
            .show(ui, |ui| interactions_table(ui, &mut p, &mut g));
        egui::CollapsingHeader::new("Material pairs")
            .default_open(false)
            .show(ui, |ui| pairs_table(ui, &mut p, &spell));
    });
    let edit = (p != before).then_some(Edit::Physics { before, after: p });
    g.commit(doc, "Edit physics settings", edit);
}

fn interactions_table(ui: &mut Ui, p: &mut PhysicsSettings, g: &mut Gesture) {
    ui.weak(contact_hint(p.contact));
    let contact = p.contact;
    let names: Vec<String> = p.interactions.keys().cloned().collect();
    let mut remove = None;
    for name in &names {
        let used = p.pairs.iter().any(|(_, _, i)| i == name);
        let Some(interaction) = p.interactions.get_mut(name) else {
            continue;
        };
        egui::CollapsingHeader::new(RichText::new(name).strong())
            .id_salt(("interaction", name))
            .default_open(false)
            .show(ui, |ui| {
                grid(ui, &format!("interaction_values_{name}"), |ui| {
                    interaction_fields(ui, g, interaction, contact);
                });
                let deletable = name != "Default" && !used;
                let button = ui.add_enabled(deletable, egui::Button::new("Remove"));
                let tip = if name == "Default" {
                    "Default is what unlisted pairs fall back to"
                } else if used {
                    "A material pair uses this interaction"
                } else {
                    "Delete this interaction"
                };
                if button
                    .on_hover_text(tip)
                    .on_disabled_hover_text(tip)
                    .clicked()
                {
                    remove = Some(name.clone());
                }
            });
    }
    if let Some(name) = remove {
        p.interactions.remove(&name);
    }
    if ui.button("Add interaction").clicked() {
        let mut n = p.interactions.len() + 1;
        let name = loop {
            let candidate = format!("Interaction {n}");
            if !p.interactions.contains_key(&candidate) {
                break candidate;
            }
            n += 1;
        };
        let template = p
            .interactions
            .get("Default")
            .copied()
            .unwrap_or(Interaction {
                restitution: 0.2,
                friction: FRICTION_PER_MU,
                adhesion: 0.0,
                softness: 0.0,
            });
        p.interactions.insert(name, template);
    }
}

fn interaction_fields(ui: &mut Ui, g: &mut Gesture, i: &mut Interaction, contact: ContactModel) {
    ui.label("Restitution");
    g.drag(
        &ui.add(
            DragValue::new(&mut i.restitution)
                .range(0.0..=1.5)
                .speed(0.005),
        ),
    );
    ui.end_row();
    ui.label("Friction");
    ui.horizontal(|ui| {
        match contact {
            ContactModel::ForceCapped => {
                g.drag(
                    &ui.add(
                        DragValue::new(&mut i.friction)
                            .range(0.0..=100_000.0)
                            .speed(1.0)
                            .suffix(" N"),
                    ),
                );
            }
            ContactModel::Coulomb => {
                g.drag(
                    &ui.add(
                        DragValue::new(&mut i.friction)
                            .range(0.0..=10_000.0)
                            .speed(1.0),
                    ),
                );
                ui.weak(format!("μ {:.2}", i.friction / FRICTION_PER_MU));
            }
        };
    });
    ui.end_row();
    ui.label("Max adhesion");
    g.drag(
        &ui.add(
            DragValue::new(&mut i.adhesion)
                .range(0.0..=100_000.0)
                .speed(1.0)
                .suffix(" N"),
        ),
    );
    ui.end_row();
    ui.label("Softness");
    g.drag(
        &ui.add(
            DragValue::new(&mut i.softness)
                .range(0.0..=100.0)
                .speed(0.001),
        ),
    );
    ui.end_row();
}

fn pairs_table(ui: &mut Ui, p: &mut PhysicsSettings, spell: &dyn Fn(&str) -> String) {
    ui.weak("Which interaction applies when two surfaces touch. Unlisted pairs use Default.");
    let names: Vec<String> = p.interactions.keys().cloned().collect();
    let mut remove = None;
    for (n, (a, b, interaction)) in p.pairs.iter_mut().enumerate() {
        ui.push_id(("pair", n), |ui| {
            ui.horizontal(|ui| {
                material_combo(ui, "a", a, spell);
                ui.weak("on");
                material_combo(ui, "b", b, spell);
                if ui
                    .small_button("✕")
                    .on_hover_text("Remove this pair")
                    .clicked()
                {
                    remove = Some(n);
                }
            });
            ui.horizontal(|ui| {
                ui.add_space(14.0);
                ui.weak("acts as");
                egui::ComboBox::from_id_salt("interaction")
                    .selected_text(interaction.as_str())
                    .show_ui(ui, |ui| {
                        if !names.contains(interaction) {
                            ui.selectable_value(
                                interaction,
                                interaction.clone(),
                                interaction.clone(),
                            );
                        }
                        for name in &names {
                            ui.selectable_value(interaction, name.clone(), name);
                        }
                    });
            });
        });
        ui.add_space(2.0);
    }
    if let Some(n) = remove {
        p.pairs.remove(n);
    }
    if ui.button("Add pair").clicked() {
        let default = if p.interactions.contains_key("Default") {
            "Default".to_owned()
        } else {
            names
                .first()
                .cloned()
                .unwrap_or_else(|| "Default".to_owned())
        };
        p.pairs.push(("default".into(), "default".into(), default));
    }
}

/// A physics body, read-only: what the scene tree selects.
pub(super) fn body_page(
    ui: &mut Ui,
    doc: &Document,
    selection: &mut Selection,
    character: CharacterId,
    index: usize,
) {
    let project = &doc.project;
    let Some(owner) = project.character(character) else {
        return;
    };
    let Some(rig) = &owner.physics else { return };
    let Some(body) = rig.bodies.get(index) else {
        return;
    };

    heading(ui, "BODY", &body.name);
    section(ui, "Body", |ui| {
        grid(ui, "body_props", |ui| {
            ui.label("Belongs to");
            if ui.link(&owner.name).clicked() {
                *selection = Selection::Character(character);
            }
            ui.end_row();
            ui.label("Follows joint");
            ui.label(
                owner
                    .skeleton
                    .joints
                    .get(body.joint)
                    .map_or("?", |j| j.name.as_str()),
            );
            ui.end_row();
            ui.label("Mass");
            ui.label(format!("{:.2} kg", body.mass));
            ui.end_row();
            ui.label("Inertia");
            ui.label(format!(
                "{:.4} · {:.4} · {:.4} kg·m²",
                body.inertia[0], body.inertia[1], body.inertia[2]
            ));
            ui.end_row();
            ui.label("Centre of mass");
            ui.label(format!(
                "{:.3} · {:.3} · {:.3} m",
                body.frame.position[0], body.frame.position[1], body.frame.position[2]
            ));
            ui.end_row();
        });
    });
    shapes_section(ui, body);
    for joint in rig.joints.iter().filter(|j| j.body == index) {
        joint_section(ui, rig.bodies.as_slice(), joint, character, selection);
    }
    ui.add_space(6.0);
    ui.weak("Read-only: bodies come from the character or scene file.");
}

fn shapes_section(ui: &mut Ui, body: &BodyDef) {
    section(ui, "Shapes", |ui| {
        if body.shapes.is_empty() {
            ui.weak("No collision shapes: this body doesn't touch anything.");
        }
        for (n, shape) in body.shapes.iter().enumerate() {
            ui.push_id(("shape", n), |ui| {
                let (kind, dims) = describe(&shape.geometry);
                ui.label(RichText::new(format!("{}  {kind}", n + 1)).strong());
                grid(ui, "shape_props", |ui| {
                    ui.label("Size");
                    ui.label(dims);
                    ui.end_row();
                    ui.label("Material");
                    ui.label(capitalised(&shape.material));
                    ui.end_row();
                    ui.label("Offset");
                    ui.label(format!(
                        "{:.3} · {:.3} · {:.3} m",
                        shape.frame.position[0], shape.frame.position[1], shape.frame.position[2]
                    ));
                    ui.end_row();
                    ui.label("Touches");
                    ui.label(touches(shape));
                    ui.end_row();
                });
            });
        }
    });
}

fn describe(geometry: &Geometry) -> (&'static str, String) {
    match *geometry {
        Geometry::Sphere { radius } => ("Sphere", format!("radius {radius:.3} m")),
        Geometry::Capsule { radius, length } => (
            "Capsule",
            format!("radius {radius:.3} m · length {length:.3} m"),
        ),
        Geometry::Cylinder { radius, length } => (
            "Cylinder",
            format!("radius {radius:.3} m · length {length:.3} m"),
        ),
        Geometry::Box { size } => (
            "Box",
            format!("{:.3} · {:.3} · {:.3} m", size[0], size[1], size[2]),
        ),
    }
}

/// Which of the character's own bodies a shape collides with, in words.
fn touches(shape: &crate::document::physics::ShapeDef) -> String {
    let mut own = Vec::new();
    if shape.collides_with_parent {
        own.push("parent");
    }
    if shape.collides_with_siblings {
        own.push("siblings");
    }
    if shape.collides_with_ancestors {
        own.push("ancestors");
    }
    if own.is_empty() {
        "other objects only".to_owned()
    } else {
        format!("other objects, own {}", own.join(", "))
    }
}

fn joint_section(
    ui: &mut Ui,
    bodies: &[BodyDef],
    joint: &JointDef,
    character: CharacterId,
    selection: &mut Selection,
) {
    section(ui, "Joint to parent", |ui| {
        grid(ui, "body_joint", |ui| {
            ui.label("Parent body");
            if let Some(parent) = bodies.get(joint.parent_body)
                && ui.link(&parent.name).clicked()
            {
                *selection = Selection::Body {
                    character,
                    body: joint.parent_body,
                };
            }
            ui.end_row();
            ui.label("Twist");
            ui.label(format!(
                "{:.0}° to {:.0}°",
                joint.hard.twist_min.to_degrees(),
                joint.hard.twist_max.to_degrees()
            ));
            ui.end_row();
            ui.label("Swing");
            ui.label(format!(
                "±{:.0}° · ±{:.0}°",
                joint.hard.swing1.to_degrees(),
                joint.hard.swing2.to_degrees()
            ));
            ui.end_row();
            ui.label("Muscle");
            ui.label(&joint.muscle);
            ui.end_row();
        });
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_per_frame_match_the_simulation() {
        assert_eq!(ticks_per_frame(120.0, 30.0), 4);
        assert_eq!(ticks_per_frame(120.0, 24.0), 5);
        assert_eq!(ticks_per_frame(30.0, 60.0), 1);
    }

    #[test]
    fn material_names_read_well() {
        assert_eq!(capitalised("tarmac"), "Tarmac");
        assert_eq!(capitalised(""), "");
    }
}
