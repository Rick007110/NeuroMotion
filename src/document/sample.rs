//! Built-in content: a placeholder humanoid (skeleton and physics rig), a short
//! two-layer demo animation, and a physics demo.

use std::collections::BTreeMap;
use std::f32::consts::TAU;

use glam::{Quat, Vec3};

use super::model::{
    CharacterKind, Channel, ClipKind, ClipProps, FrameRange, Interp, Joint, Keyframe, Project,
    ProjectSettings, Property, SkeletonRef,
};
use super::physics::{
    BodyDef, Frame, Gains, Geometry, JointDef, Limits, MuscleGains, MuscleSet, PhysicsRig, ShapeDef,
    SimLevel,
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
    let character = add_placeholder_character(&mut project);
    project.add_track(character, "Base");
    project
}

fn add_placeholder_character(project: &mut Project) -> super::model::CharacterId {
    let skeleton = humanoid_skeleton();
    let rig = humanoid_rig(&skeleton);
    let id = project.add_character("character01", CharacterKind::Character, skeleton);
    if let Some(c) = project.characters.iter_mut().find(|c| c.id == id) {
        c.physics = Some(rig);
    }
    id
}

/// The startup project: an idle loop on the base track with a waving gesture layered
/// on top, so playback and blending have something to show.
pub fn placeholder_project() -> Project {
    let mut project = Project::new(ProjectSettings {
        name: "Placeholder".to_owned(),
        fps: 30.0,
        range: FrameRange { start: 0, end: 240 },
    });
    let character = add_placeholder_character(&mut project);
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

/// The placeholder biped's physical body, modelled on endorphin's standard character:
/// one capsule per bone (spheres and boxes for head and feet), about 75 kg in total,
/// twist/swing limits per joint, and endorphin-style muscle presets.
pub fn humanoid_rig(skeleton: &SkeletonRef) -> PhysicsRig {
    // (joint, mass kg, radius m, shape) - the bone runs from the joint to its first child.
    enum Kind {
        Bone,
        Head,
        Foot,
        Hand,
    }
    let spec: [(&str, f32, f32, Kind); 15] = [
        ("hips", 11.0, 0.09, Kind::Bone),
        ("spine", 9.0, 0.1, Kind::Bone),
        ("chest", 14.0, 0.12, Kind::Bone),
        ("neck", 1.2, 0.05, Kind::Bone),
        ("head", 5.0, 0.1, Kind::Head),
        ("l_shoulder", 2.2, 0.045, Kind::Bone),
        ("l_elbow", 1.4, 0.038, Kind::Bone),
        ("l_wrist", 0.5, 0.035, Kind::Hand),
        ("l_hip", 8.5, 0.07, Kind::Bone),
        ("l_knee", 3.6, 0.055, Kind::Bone),
        ("l_ankle", 1.1, 0.0, Kind::Foot),
        ("r_shoulder", 2.2, 0.045, Kind::Bone),
        ("r_elbow", 1.4, 0.038, Kind::Bone),
        ("r_wrist", 0.5, 0.035, Kind::Hand),
        ("r_hip", 8.5, 0.07, Kind::Bone),
    ];
    let mut entries: Vec<(&str, f32, f32, Kind)> = spec.into_iter().collect();
    entries.push(("r_knee", 3.6, 0.055, Kind::Bone));
    entries.push(("r_ankle", 1.1, 0.0, Kind::Foot));

    let first_child = |j: usize| skeleton.joints.iter().position(|c| c.parent == Some(j));
    let limbs = ["shoulder", "elbow", "wrist", "hip", "knee", "ankle"];
    let mut bodies = Vec::new();
    for (name, mass, radius, kind) in entries {
        let Some(joint) = skeleton.joint_index(name) else { continue };
        let bone = first_child(joint).map_or(Vec3::new(0.0, 0.1, 0.0), |c| Vec3::from(skeleton.joints[c].offset));
        let along = |dir: Vec3| Quat::from_rotation_arc(Vec3::Z, dir.normalize());
        let (geometry, frame, material) = match kind {
            Kind::Bone => {
                let length = (bone.length() - 2.0 * radius).max(0.0);
                (Geometry::Capsule { radius, length }, (along(bone), bone * 0.5), "skin")
            }
            Kind::Head => (Geometry::Sphere { radius }, (Quat::IDENTITY, Vec3::new(0.0, 0.08, 0.02)), "skin"),
            Kind::Hand => {
                let dir = Vec3::new(0.0, -0.09, 0.0);
                (Geometry::Capsule { radius, length: 0.02 }, (along(dir), dir * 0.5), "skin")
            }
            Kind::Foot => (
                Geometry::Box { size: [0.09, 0.07, 0.24] },
                (Quat::IDENTITY, Vec3::new(0.0, -0.035, 0.06)),
                "rubber",
            ),
        };
        let limb = limbs.iter().any(|l| name.ends_with(l));
        bodies.push(BodyDef {
            name: name.to_owned(),
            joint,
            frame: Frame { rotation: frame.0.to_array(), position: frame.1.to_array() },
            mass,
            inertia: geometry.inertia(mass),
            shapes: vec![ShapeDef {
                geometry,
                frame: Frame::IDENTITY,
                material: material.to_owned(),
                collides_with_parent: false,
                collides_with_siblings: limb,
                collides_with_ancestors: limb,
            }],
        });
    }

    let body_of = |joint: usize| bodies.iter().position(|b| b.joint == joint);
    // Half-angle ranges (rad): twist, swing about the bone's first and second side axes.
    let ball = |twist: f32, s1: f32, s2: f32| Limits { twist_min: -twist, twist_max: twist, swing1: s1, swing2: s2 };
    let mut joints = Vec::new();
    for (bi, body) in bodies.iter().enumerate() {
        let Some(parent_joint) = skeleton.joints[body.joint].parent else { continue };
        let Some(parent_body) = body_of(parent_joint) else { continue };
        let name = skeleton.joints[body.joint].name.clone();
        let bone = first_child(body.joint).map_or(Vec3::NEG_Y, |c| Vec3::from(skeleton.joints[c].offset));
        let twist_frame = Quat::from_rotation_arc(Vec3::X, bone.normalize());
        // Knees and elbows are hinges about the side (X) axis; the rest pose sits
        // just inside the straight-limb stop so they only bend one way.
        let hinge = |rest_twist: f32| (Quat::from_rotation_x(-rest_twist), Quat::IDENTITY, Limits { twist_min: -1.3, twist_max: 1.3, swing1: 0.01, swing2: 0.01 });
        let (primary, secondary, hard) = match name.trim_start_matches(['l', 'r', '_']) {
            "knee" => hinge(-1.25),
            "elbow" => hinge(1.25),
            "spine" | "chest" => (twist_frame, twist_frame, ball(0.4, 0.5, 0.4)),
            "neck" => (twist_frame, twist_frame, ball(0.6, 0.6, 0.6)),
            "head" => (twist_frame, twist_frame, ball(0.7, 0.6, 0.5)),
            "shoulder" => (twist_frame, twist_frame, ball(1.4, 2.2, 2.0)),
            "wrist" => (twist_frame, twist_frame, ball(0.3, 1.0, 0.5)),
            "hip" => (twist_frame, twist_frame, ball(0.7, 1.4, 0.8)),
            "ankle" => (twist_frame, twist_frame, ball(0.3, 0.6, 0.4)),
            _ => (twist_frame, twist_frame, ball(0.5, 0.5, 0.5)),
        };
        let hinged = matches!(name.trim_start_matches(['l', 'r', '_']), "knee" | "elbow");
        let soft = Limits {
            twist_min: hard.twist_min + 0.05,
            twist_max: hard.twist_max - 0.05,
            swing1: (hard.swing1 - 0.05).max(0.005),
            swing2: (hard.swing2 - 0.05).max(0.005),
        };
        joints.push(JointDef {
            muscle: name.clone(),
            name,
            joint: body.joint,
            body: bi,
            parent_body,
            primary: primary.to_array(),
            secondary: secondary.to_array(),
            hard,
            soft,
            soft_stiffness: if hinged { 10_000.0 } else { 10.0 },
            soft_damping: if hinged { 1_000.0 } else { 10.0 },
        });
    }

    PhysicsRig { bodies, joints, muscle_sets: endorphin_style_presets(), isotropic_inertia: true }
}

/// Muscle presets with endorphin's names and typical values.
fn endorphin_style_presets() -> Vec<MuscleSet> {
    let gains = |stiffness, damping, force_limit| Gains { stiffness, damping, force_limit };
    let both = |g: Gains| MuscleGains { twist: g, swing: g };
    let set = |name: &str, fallback: MuscleGains, special: &[(&str, MuscleGains)]| {
        let mut muscles = BTreeMap::new();
        for side in ["l_", "r_", ""] {
            for (joint, g) in special {
                muscles.insert(format!("{side}{joint}"), *g);
            }
        }
        MuscleSet { name: name.to_owned(), muscles, fallback }
    };
    let strong_hinge = MuscleGains { twist: gains(5000.0, 1000.0, 2000.0), swing: gains(3000.0, 500.0, 1000.0) };
    vec![
        set("Default", both(gains(50.0, 100.0, 50.0)), &[]),
        set("Normal", both(gains(1000.0, 500.0, 1000.0)), &[("hip", both(gains(3000.0, 500.0, 1000.0))), ("knee", strong_hinge), ("elbow", strong_hinge)]),
        set("DesiredPoseCalibration", both(gains(1000.0, 100.0, 500.0)), &[("hip", both(gains(5000.0, 500.0, 5000.0))), ("spine", both(gains(5000.0, 2000.0, 10000.0)))]),
        set("Strong", MuscleGains { twist: gains(6000.0, 1000.0, 3000.0), swing: gains(6000.0, 1000.0, 5000.0) }, &[]),
        set("Weak", both(gains(50.0, 100.0, 50.0)), &[]),
    ]
}

/// A short physics demo: the character idles, goes into simulation at frame 45 and is
/// shoved in the chest, then falls under gravity with its default (soft) muscles.
pub fn physics_demo_project() -> Project {
    let mut project = placeholder_project();
    project.settings.name = "Physics demo".to_owned();
    project.settings.range = FrameRange { start: 0, end: 150 };
    project.clips.retain(|c| c.props.name != "Wave");
    let character = project.characters[0].id;
    let events = project.add_track(character, "Simulation");
    let forces = project.add_track(character, "Forces");
    if let Some(c) = project.characters.iter_mut().find(|c| c.id == character) {
        c.default_level = SimLevel::Animated;
    }
    let event = |name: &str, track, start, duration| ClipProps {
        name: name.to_owned(),
        track,
        start,
        duration,
        blend_in: 0,
        blend_out: 0,
        weight: 1.0,
        enabled: true,
    };
    let simulate = project.add_clip(event("Go limp", events, 45, 1), ClipKind::Event { event: "Simulation".into() }, vec![]);
    let push = project.add_clip(event("Shove", forces, 45, 1), ClipKind::Event { event: "Force".into() }, vec![]);
    for (id, settings) in [
        (simulate, vec![("physicsLevel", "FullPhysics"), ("momentumScale", "1")]),
        (push, vec![("matrix.Vi", "0.4, 0.6, -3.5"), ("targets", "character01.MetaBodies.chest")]),
    ] {
        if let Some(clip) = project.clip_mut(id) {
            clip.source = settings.into_iter().map(|(k, v)| (k.to_owned(), v.to_owned())).collect();
        }
    }
    project
}
