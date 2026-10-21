//! Built-in content: a placeholder humanoid rig and a short two-layer demo animation.

use std::f32::consts::TAU;

use super::model::{
    CharacterKind, Channel, ClipKind, ClipProps, FrameRange, Interp, Joint, Keyframe, Project,
    ProjectSettings, Property, SkeletonRef,
};

/// A 19-joint biped, Y up, facing +Z, standing on the ground plane.
pub fn humanoid_skeleton() -> SkeletonRef {
    let mut joints = Vec::new();
    let mut add = |name: &str, parent: Option<usize>, offset: [f32; 3]| {
        joints.push(Joint::new(name, parent, offset));
        joints.len() - 1
    };
    let hips = add("hips", None, [0.0, 0.95, 0.0]);
    let spine = add("spine", Some(hips), [0.0, 0.12, 0.0]);
    let chest = add("chest", Some(spine), [0.0, 0.22, 0.0]);
    let neck = add("neck", Some(chest), [0.0, 0.2, 0.0]);
    add("head", Some(neck), [0.0, 0.13, 0.02]);
    for (side, x) in [("l", 1.0f32), ("r", -1.0)] {
        let shoulder = add(&format!("{side}_shoulder"), Some(chest), [0.19 * x, 0.16, 0.0]);
        let elbow = add(&format!("{side}_elbow"), Some(shoulder), [0.0, -0.28, 0.0]);
        add(&format!("{side}_wrist"), Some(elbow), [0.0, -0.25, 0.0]);
    }
    for (side, x) in [("l", 1.0f32), ("r", -1.0)] {
        let hip = add(&format!("{side}_hip"), Some(hips), [0.1 * x, -0.06, 0.0]);
        let knee = add(&format!("{side}_knee"), Some(hip), [0.0, -0.42, 0.0]);
        let ankle = add(&format!("{side}_ankle"), Some(knee), [0.0, -0.41, 0.0]);
        add(&format!("{side}_toe"), Some(ankle), [0.0, -0.05, 0.13]);
    }
    SkeletonRef {
        name: "Placeholder biped".to_owned(),
        source: None,
        joints,
    }
}

/// An empty project: the placeholder character and a single empty track.
pub fn blank_project() -> Project {
    let mut project = Project::new(ProjectSettings {
        name: "Untitled".to_owned(),
        fps: 30.0,
        range: FrameRange { start: 0, end: 150 },
    });
    let character = project.add_character("character01", CharacterKind::Character, humanoid_skeleton());
    project.add_track(character, "Base");
    project
}

/// The startup project: an idle loop on the base track with a waving gesture layered
/// on top, so playback and blending have something to show.
pub fn placeholder_project() -> Project {
    let mut project = Project::new(ProjectSettings {
        name: "Placeholder".to_owned(),
        fps: 30.0,
        range: FrameRange { start: 0, end: 240 },
    });
    let character = project.add_character("character01", CharacterKind::Character, humanoid_skeleton());
    let base = project.add_track(character, "Base");
    let layer = project.add_track(character, "Gesture layer");

    let idle_len = 240;
    let wave = |period: f32, amp: f32, phase: f32, bias: f32| {
        move |t: f32| bias + amp * (TAU * t / period + phase).sin()
    };
    project.add_clip(
        ClipProps {
            name: "Idle breathe".to_owned(),
            track: base,
            start: 0,
            duration: idle_len,
            blend_in: 0,
            blend_out: 0,
            weight: 1.0,
            enabled: true,
        },
        ClipKind::Motion,
        vec![
            sampled("hips", Property::PosY, idle_len, 10, wave(80.0, 0.012, 0.0, -0.012)),
            sampled("hips", Property::RotY, idle_len, 20, wave(240.0, 7.0, 0.0, 0.0)),
            sampled("spine", Property::RotX, idle_len, 10, wave(80.0, 2.5, 0.0, 2.0)),
            sampled("chest", Property::RotZ, idle_len, 15, wave(120.0, 3.0, 0.5, 0.0)),
            sampled("head", Property::RotY, idle_len, 20, wave(160.0, 14.0, 1.0, 0.0)),
            sampled("head", Property::RotX, idle_len, 20, wave(100.0, 3.0, 0.0, -2.0)),
            sampled("l_shoulder", Property::RotZ, idle_len, 10, wave(80.0, 2.0, 0.0, 9.0)),
            sampled("r_shoulder", Property::RotZ, idle_len, 10, wave(80.0, -2.0, 0.0, -9.0)),
            held("l_elbow", Property::RotX, idle_len, -14.0),
            held("r_elbow", Property::RotX, idle_len, -14.0),
            held("l_hip", Property::RotX, idle_len, -4.0),
            held("r_hip", Property::RotX, idle_len, -4.0),
            sampled("l_knee", Property::RotX, idle_len, 10, wave(80.0, 2.0, 0.0, 6.0)),
            sampled("r_knee", Property::RotX, idle_len, 10, wave(80.0, 2.0, 0.0, 6.0)),
        ],
    );

    let wave_len = 110;
    project.add_clip(
        ClipProps {
            name: "Wave".to_owned(),
            track: layer,
            start: 70,
            duration: wave_len,
            blend_in: 20,
            blend_out: 20,
            weight: 1.0,
            enabled: true,
        },
        ClipKind::Motion,
        vec![
            held("r_shoulder", Property::RotZ, wave_len, -155.0),
            held("r_shoulder", Property::RotX, wave_len, -12.0),
            sampled("r_elbow", Property::RotZ, wave_len, 5, wave(20.0, 28.0, 0.0, -18.0)),
            held("r_elbow", Property::RotX, wave_len, 0.0),
            held("head", Property::RotZ, wave_len, -8.0),
            held("chest", Property::RotZ, wave_len, -4.0),
        ],
    );
    project
}

/// A channel keyed every `step` frames from a function of the clip-local frame.
fn sampled(joint: &str, property: Property, len: i32, step: i32, f: impl Fn(f32) -> f32) -> Channel {
    let mut frames: Vec<i32> = (0..len).step_by(step as usize).collect();
    frames.push(len);
    Channel {
        joint: joint.to_owned(),
        property,
        keys: frames
            .into_iter()
            .map(|frame| Keyframe {
                frame,
                value: f(frame as f32),
                interp: Interp::Smooth,
            })
            .collect(),
    }
}

/// A channel holding one value for the whole clip.
fn held(joint: &str, property: Property, len: i32, value: f32) -> Channel {
    sampled(joint, property, len, len, move |_| value)
}
