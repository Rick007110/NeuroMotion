//! Right-hand inspector: shows and edits whatever is selected.
//!
//! Each selection kind gets its controls grouped by what they affect (identity, timing,
//! blending, curve). Widgets edit a copy; differences are committed to the document as
//! one [`Edit`], so every change here is undoable.

use eframe::egui::{self, Color32, DragValue, Response, RichText, Slider, Ui};

use super::selection::Selection;
use super::theme::{self, grid, section};
use crate::document::{CharacterId, ClipId, ClipKind, Document, Edit, Interp, TrackId};

/// Collects how this frame's widget interactions should be recorded in the history.
#[derive(Default)]
struct Gesture {
    continuous: bool,
    finished: bool,
}

impl Gesture {
    /// Drag widgets: every step of a drag folds into one undo entry.
    fn drag(&mut self, r: &Response) {
        self.continuous |= r.dragged();
        self.finished |= r.drag_stopped();
    }

    /// Text fields: typing folds into one entry until focus leaves the field.
    fn text(&mut self, r: &Response) {
        self.continuous |= r.has_focus();
        self.finished |= r.lost_focus();
    }

    fn commit(self, doc: &mut Document, label: &str, edit: Option<Edit>) {
        if let Some(edit) = edit {
            doc.apply(label, edit, self.continuous);
        }
        if self.finished {
            doc.end_gesture();
        }
    }
}

pub fn show(ui: &mut Ui, doc: &mut Document, selection: &mut Selection, playhead: f32) {
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| match *selection {
            Selection::None => project_page(ui, doc, selection),
            Selection::Character(id) => character_page(ui, doc, selection, id),
            Selection::Track(id) => track_page(ui, doc, id),
            Selection::Clip(id) => clip_page(ui, doc, selection, id, playhead),
            Selection::Channel { clip, channel } => {
                channel_page(ui, doc, selection, clip, channel, playhead)
            }
            Selection::Key { clip, channel, frame } => {
                key_page(ui, doc, selection, clip, channel, frame)
            }
        });
}

fn heading(ui: &mut Ui, kind: &str, title: &str) {
    ui.add_space(6.0);
    ui.label(RichText::new(kind).small().strong().color(theme::ACCENT));
    ui.label(RichText::new(title).size(17.0).color(theme::TEXT));
    ui.add_space(2.0);
    ui.separator();
}

fn project_page(ui: &mut Ui, doc: &mut Document, selection: &mut Selection) {
    let before = doc.project.settings.clone();
    let mut s = before.clone();
    let mut g = Gesture::default();

    heading(ui, "PROJECT", &before.name);
    section(ui, "General", |ui| {
        grid(ui, "project_general", |ui| {
            ui.label("Name");
            g.text(&ui.text_edit_singleline(&mut s.name));
            ui.end_row();
        });
    });
    section(ui, "Time", |ui| {
        grid(ui, "project_time", |ui| {
            ui.label("Frame rate");
            g.drag(&ui.add(DragValue::new(&mut s.fps).range(1.0..=240.0).speed(0.1).suffix(" fps")));
            ui.end_row();
            ui.label("Start frame");
            g.drag(&ui.add(DragValue::new(&mut s.range.start).range(-100_000..=s.range.end - 1)));
            ui.end_row();
            ui.label("End frame");
            g.drag(&ui.add(DragValue::new(&mut s.range.end).range(s.range.start + 1..=100_000)));
            ui.end_row();
            ui.label("Length");
            let len = s.range.len();
            ui.weak(format!("{len} frames · {:.2} s", len as f32 / s.fps));
            ui.end_row();
        });
    });
    let edit = (s != before).then(|| Edit::Settings { before, after: s });
    g.commit(doc, "Edit project settings", edit);

    let project = &doc.project;
    section(ui, "Characters", |ui| {
        for character in &project.characters {
            ui.horizontal(|ui| {
                if ui.selectable_label(false, &character.name).clicked() {
                    *selection = Selection::Character(character.id);
                }
                ui.weak(format!(
                    "{} · {} joints",
                    character.kind.label(),
                    character.skeleton.joints.len()
                ));
            });
        }
    });
    section(ui, "Contents", |ui| {
        grid(ui, "project_contents", |ui| {
            ui.label("Tracks");
            ui.label(project.tracks.len().to_string());
            ui.end_row();
            ui.label("Clips");
            ui.label(project.clips.len().to_string());
            ui.end_row();
        });
        ui.weak("Select a track, clip or key on the timeline to edit it here.");
    });
}

fn character_page(ui: &mut Ui, doc: &mut Document, selection: &mut Selection, id: CharacterId) {
    let project = &doc.project;
    let Some(character) = project.character(id) else { return };
    let skeleton = &character.skeleton;

    heading(ui, character.kind.label().to_uppercase().as_str(), &character.name);
    section(ui, "Rig", |ui| {
        grid(ui, "character_rig", |ui| {
            ui.label("Skeleton");
            ui.label(&skeleton.name);
            ui.end_row();
            ui.label("Source");
            ui.weak(skeleton.source.as_deref().unwrap_or("built-in"));
            ui.end_row();
            ui.label("Joints");
            ui.label(skeleton.joints.len().to_string());
            ui.end_row();
        });
        egui::CollapsingHeader::new("Hierarchy")
            .default_open(false)
            .show(ui, |ui| {
                for joint in &skeleton.joints {
                    let mut depth = 0;
                    let mut parent = joint.parent;
                    while let Some(p) = parent {
                        depth += 1;
                        parent = skeleton.joints[p].parent;
                    }
                    ui.label(RichText::new(format!("{}{}", "  ".repeat(depth), joint.name)).monospace());
                }
            });
    });
    if let Some(rig) = &character.physics {
        section(ui, "Physics", |ui| {
            grid(ui, "character_physics", |ui| {
                ui.label("Bodies");
                ui.label(rig.bodies.len().to_string());
                ui.end_row();
                ui.label("Mass");
                ui.label(format!("{:.1} kg", rig.total_mass()));
                ui.end_row();
                ui.label("Joints");
                ui.label(rig.joints.len().to_string());
                ui.end_row();
                ui.label("Starts as");
                ui.label(character.default_level.label());
                ui.end_row();
                ui.label("Muscle presets");
                ui.label(rig.muscle_sets.iter().map(|s| s.name.as_str()).collect::<Vec<_>>().join(", "));
                ui.end_row();
            });
        });
    }
    section(ui, "Tracks", |ui| {
        for track in project.tracks_of(id) {
            ui.horizontal(|ui| {
                if ui.selectable_label(false, &track.name).clicked() {
                    *selection = Selection::Track(track.id);
                }
                ui.weak(format!("{} clips", project.clips_on_track(track.id).count()));
            });
        }
    });
}

fn track_page(ui: &mut Ui, doc: &mut Document, id: TrackId) {
    let Some(before) = doc.project.track(id).cloned() else { return };
    let clip_count = doc.project.clips_on_track(id).count();
    let mut t = before.clone();
    let mut g = Gesture::default();

    heading(ui, "TRACK", &before.name);
    section(ui, "Track", |ui| {
        grid(ui, "track_props", |ui| {
            ui.label("Name");
            g.text(&ui.text_edit_singleline(&mut t.name));
            ui.end_row();
            ui.label("Muted");
            ui.checkbox(&mut t.muted, "");
            ui.end_row();
            ui.label("Clips");
            ui.label(clip_count.to_string());
            ui.end_row();
        });
    });
    let edit = (t != before).then(|| Edit::Track { id, before, after: t });
    g.commit(doc, "Edit track", edit);
}

fn clip_page(ui: &mut Ui, doc: &mut Document, selection: &mut Selection, id: ClipId, playhead: f32) {
    let Some(clip) = doc.project.clip(id) else { return };
    let before = clip.props.clone();
    let kind = clip.kind.clone();
    let source = clip.source.clone();
    let character = doc.project.clip_character(clip).map(|c| (c.id, c.name.clone()));
    let influence = clip.influence(playhead);
    let channels: Vec<(String, usize)> = clip
        .channels
        .iter()
        .map(|c| (format!("{} · {}", c.joint, c.property.label()), c.keys.len()))
        .collect();
    let tracks: Vec<(TrackId, String)> =
        doc.project
            .tracks
            .iter()
            .filter(|t| character.as_ref().is_none_or(|(cid, _)| t.character == *cid))
            .map(|t| (t.id, t.name.clone()))
            .collect();

    let mut p = before.clone();
    let mut g = Gesture::default();

    let kind_label = match &kind {
        ClipKind::Motion => "MOTION CLIP".to_owned(),
        ClipKind::Event { event } => format!("{} EVENT", event.to_uppercase()),
    };
    heading(ui, &kind_label, &before.name);
    section(ui, "Clip", |ui| {
        grid(ui, "clip_identity", |ui| {
            if let Some((cid, name)) = &character {
                ui.label("Character");
                if ui.link(name).clicked() {
                    *selection = Selection::Character(*cid);
                }
                ui.end_row();
            }
            ui.label("Name");
            g.text(&ui.text_edit_singleline(&mut p.name));
            ui.end_row();
            ui.label("Enabled");
            ui.checkbox(&mut p.enabled, "");
            ui.end_row();
            ui.label("Track");
            let current = tracks
                .iter()
                .find(|(tid, _)| *tid == p.track)
                .map(|(_, name)| name.clone())
                .unwrap_or_default();
            egui::ComboBox::from_id_salt("clip_track")
                .selected_text(current)
                .show_ui(ui, |ui| {
                    for (tid, name) in &tracks {
                        ui.selectable_value(&mut p.track, *tid, name);
                    }
                });
            ui.end_row();
        });
    });
    section(ui, "Timing", |ui| {
        grid(ui, "clip_timing", |ui| {
            ui.label("Start");
            g.drag(&ui.add(DragValue::new(&mut p.start).speed(0.5)));
            ui.end_row();
            ui.label("Duration");
            g.drag(&ui.add(DragValue::new(&mut p.duration).range(1..=100_000).speed(0.5)));
            ui.end_row();
            ui.label("End");
            ui.weak(p.end().to_string());
            ui.end_row();
            ui.label("Local frame");
            if p.contains(playhead) {
                ui.weak(format!("{:.0}", playhead - p.start as f32));
            } else {
                ui.weak("playhead outside clip");
            }
            ui.end_row();
        });
    });
    section(ui, "Blending", |ui| {
        grid(ui, "clip_blend", |ui| {
            ui.label("Weight");
            g.drag(&ui.add(Slider::new(&mut p.weight, 0.0..=1.0)));
            ui.end_row();
            ui.label("Blend in");
            g.drag(&ui.add(DragValue::new(&mut p.blend_in).range(0..=p.duration).suffix(" fr")));
            ui.end_row();
            ui.label("Blend out");
            g.drag(&ui.add(DragValue::new(&mut p.blend_out).range(0..=p.duration).suffix(" fr")));
            ui.end_row();
            ui.label("Influence now");
            ui.add(egui::ProgressBar::new(influence).text(format!("{:.0}%", influence * 100.0)));
            ui.end_row();
        });
    });
    p.blend_in = p.blend_in.min(p.duration);
    p.blend_out = p.blend_out.min(p.duration);
    let edit = (p != before).then(|| Edit::ClipProps { clip: id, before, after: p });
    g.commit(doc, "Edit clip", edit);

    if !source.is_empty() {
        section(ui, "Imported settings", |ui| {
            ui.weak("Kept from the source file for the physics system; not simulated yet.");
            egui::Grid::new("clip_source").striped(true).num_columns(2).show(ui, |ui| {
                for (key, value) in &source {
                    ui.label(key);
                    ui.label(RichText::new(value).monospace());
                    ui.end_row();
                }
            });
        });
    }
    if !kind.is_motion() {
        return;
    }
    section(ui, "Channels", |ui| {
        for (i, (label, keys)) in channels.iter().enumerate() {
            ui.horizontal(|ui| {
                if ui.selectable_label(false, label).clicked() {
                    *selection = Selection::Channel { clip: id, channel: i };
                }
                ui.weak(format!("{keys} keys"));
            });
        }
    });
}

fn channel_page(
    ui: &mut Ui,
    doc: &mut Document,
    selection: &mut Selection,
    clip_id: ClipId,
    channel: usize,
    playhead: f32,
) {
    let Some(clip) = doc.project.clip(clip_id) else { return };
    let Some(ch) = clip.channels.get(channel) else { return };
    let local = (playhead - clip.props.start as f32).round() as i32;
    let inside = clip.props.contains(playhead);
    let value_now = ch.sample(playhead - clip.props.start as f32);
    let unit = ch.property.unit();
    let clip_name = clip.props.name.clone();
    let keys = ch.keys.clone();
    let title = format!("{} · {}", ch.joint, ch.property.label());
    let joint = ch.joint.clone();
    let property = ch.property.label();

    heading(ui, "CHANNEL", &title);
    section(ui, "Channel", |ui| {
        grid(ui, "channel_info", |ui| {
            ui.label("Joint");
            ui.label(&joint);
            ui.end_row();
            ui.label("Property");
            ui.label(property);
            ui.end_row();
            ui.label("Clip");
            if ui.link(&clip_name).clicked() {
                *selection = Selection::Clip(clip_id);
            }
            ui.end_row();
            ui.label("Value now");
            ui.weak(value_now.map_or("—".to_owned(), |v| format!("{v:.3}{unit}")));
            ui.end_row();
        });
    });
    section(ui, "Keys", |ui| {
        let add = ui.add_enabled(inside, egui::Button::new("Add key at playhead"));
        let add = if inside { add } else { add.on_disabled_hover_text("Move the playhead inside the clip") };
        if add.clicked() && doc.insert_key(clip_id, channel, local) {
            *selection = Selection::Key { clip: clip_id, channel, frame: local };
        }
        ui.add_space(4.0);
        egui::Grid::new("channel_keys").striped(true).num_columns(3).show(ui, |ui| {
            ui.weak("frame");
            ui.weak("value");
            ui.weak("curve");
            ui.end_row();
            for key in &keys {
                if ui.selectable_label(false, key.frame.to_string()).clicked() {
                    *selection = Selection::Key { clip: clip_id, channel, frame: key.frame };
                }
                ui.label(format!("{:.3}{unit}", key.value));
                ui.weak(key.interp.label());
                ui.end_row();
            }
        });
    });
}

fn key_page(
    ui: &mut Ui,
    doc: &mut Document,
    selection: &mut Selection,
    clip_id: ClipId,
    channel: usize,
    frame: i32,
) {
    let Some(clip) = doc.project.clip(clip_id) else { return };
    let Some(ch) = clip.channels.get(channel) else { return };
    let Some(before) = ch.key(frame).cloned() else { return };
    let start = clip.props.start;
    let clip_name = clip.props.name.clone();
    let channel_name = format!("{} · {}", ch.joint, ch.property.label());
    let (unit, speed) = if ch.property.is_rotation() { ("°", 0.25) } else { (" m", 0.002) };

    let mut k = before.clone();
    let mut g = Gesture::default();

    heading(ui, "KEYFRAME", &format!("{channel_name} @ {frame}"));
    section(ui, "Key", |ui| {
        grid(ui, "key_props", |ui| {
            ui.label("Frame (clip)");
            g.drag(&ui.add(DragValue::new(&mut k.frame).speed(0.25)));
            ui.end_row();
            ui.label("Frame (timeline)");
            ui.weak((start + k.frame).to_string());
            ui.end_row();
            ui.label("Value");
            g.drag(&ui.add(DragValue::new(&mut k.value).speed(speed).max_decimals(4).suffix(unit)));
            ui.end_row();
            ui.label("Curve to next");
            egui::ComboBox::from_id_salt("key_interp")
                .selected_text(k.interp.label())
                .show_ui(ui, |ui| {
                    for interp in Interp::ALL {
                        ui.selectable_value(&mut k.interp, interp, interp.label());
                    }
                });
            ui.end_row();
        });
    });
    if k != before {
        if doc.replace_key(clip_id, channel, frame, k.clone(), "Edit key", g.continuous) {
            *selection = Selection::Key { clip: clip_id, channel, frame: k.frame };
        }
    }
    if g.finished {
        doc.end_gesture();
    }

    section(ui, "Belongs to", |ui| {
        grid(ui, "key_owner", |ui| {
            ui.label("Clip");
            if ui.link(&clip_name).clicked() {
                *selection = Selection::Clip(clip_id);
            }
            ui.end_row();
            ui.label("Channel");
            if ui.link(&channel_name).clicked() {
                *selection = Selection::Channel { clip: clip_id, channel };
            }
            ui.end_row();
        });
    });
    ui.add_space(8.0);
    let delete = egui::Button::new(RichText::new("Delete key").color(Color32::from_rgb(240, 130, 120)));
    if ui.add(delete).clicked() && doc.remove_key(clip_id, channel, frame) {
        *selection = Selection::Channel { clip: clip_id, channel };
    }
}
