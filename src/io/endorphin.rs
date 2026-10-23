//! Importers for endorphin 2.x files, so existing work opens in NeuroMotion.
//!
//! endorphin documents are zlib-compressed XML: `.ens` scenes, `.nmc` characters and
//! props, `.nma` poses. Format details (see docs/endorphin-formats.md):
//!
//! * Attributes whose name starts with `b` and whose value is 8 hex digits hold a
//!   float32 bit pattern (`bstrength="3dcccccd"` is 0.1).
//! * A `TransformMatrix` is four vectors: the rotation basis `Vi`, `Vj`, `Vk` (matrix
//!   columns) and the position `Vp`, all relative to the parent joint.
//! * Animation keys are timed in seconds. Quaternion keys store w, x, y, z in the
//!   `bx`, `by`, `bz`, `bw` attributes; rotations are full local joint orientations.
//! * Scene frames run at the engine's `positionBufferFrameRate`.
//!
//! Animation becomes keyframed motion clips. Physics events (forces, behaviours,
//! simulation switches, transitions, ...) become event clips whose settings are kept
//! in [`Clip::source`](crate::document::Clip::source) for the physics system.

use std::{collections::BTreeMap, fs, io::Read, path::Path};

use flate2::read::ZlibDecoder;
use glam::{EulerRot, Mat3, Quat, Vec3};
use roxmltree::Node;

use super::IoError;
use crate::document::physics::{
    BodyDef, Frame, Gains, Geometry, Ground, Interaction, JointDef, Limits, MuscleGains, MuscleSet,
    PhysicsRig, ShapeDef,
};
use crate::document::{
    CharacterId, CharacterKind, Channel, ClipKind, ClipProps, FrameRange, Interp, Joint,
    Keyframe, Project, ProjectSettings, Property, SimLevel, SkeletonRef,
};

/// An imported project plus human-readable notes on what was converted or left out.
pub struct Import {
    pub project: Project,
    pub notes: Vec<String>,
}

/// A pose ready to be placed on the timeline as a held clip.
pub struct PoseImport {
    pub character: CharacterId,
    pub name: String,
    pub channels: Vec<Channel>,
}

pub fn import_scene(path: &Path) -> Result<Import, IoError> {
    let text = read_document(path)?;
    let xml = parse(&text)?;
    let root = xml.root_element();
    if !root.has_tag_name("Scene") {
        return Err(IoError::Format("not an endorphin scene (.ens)".into()));
    }

    let engine = child(root, "Engine");
    let fps = engine
        .and_then(|e| attr_f32(e, "bpositionBufferFrameRate"))
        .filter(|f| *f > 0.0)
        .unwrap_or(60.0);
    let mut project = Project::new(ProjectSettings {
        name: file_stem(path),
        fps,
        range: FrameRange { start: 0, end: 1 },
    });
    let mut notes = Vec::new();
    let mut skipped_joints = 0;
    if let Some(engine) = engine {
        read_physics_settings(&mut project, root, engine);
    }
    let isotropic = engine.and_then(|e| e.attribute("enableNonSphericalInertialBodies")) != Some("true");

    for node in elements(child(root, "Characters")) {
        let kind = if node.attribute("typeName") == Some("animatableProp") {
            CharacterKind::Prop
        } else {
            CharacterKind::Character
        };
        add_character(&mut project, node, kind, isotropic, &mut skipped_joints);
    }
    if let Some(env) = child(root, "Environment") {
        if child(env, "Skeleton").is_some() {
            add_character(&mut project, env, CharacterKind::Environment, isotropic, &mut skipped_joints);
        }
    }
    // endorphin simulates everything from the first frame unless an event says otherwise,
    // scenery included: its massless pieces stay fixed, while props with mass are real
    // objects (which is why scenes lock or join them to hold them in place).
    for character in project.characters.iter_mut().filter(|c| c.physics.is_some()) {
        character.default_level = SimLevel::Simulated;
    }
    if project.characters.is_empty() {
        return Err(IoError::Format("the scene contains no characters".into()));
    }

    // Every timed element in the event sections becomes a clip.
    let mut events: Vec<Node> = ["Events", "ForceEvents", "SecondaryBehaviours", "TertiaryBehaviours", "SeverBehaviourEvents"]
        .into_iter()
        .flat_map(|section| elements(child(root, section)))
        .filter(|e| e.attribute("startFrameNumber").is_some())
        .collect();
    // Animation first so its tracks sit at the top of each character, then by kind.
    events.sort_by_key(|e| {
        let label = event_label(e.tag_name().name());
        (label != "Animation", label, attr_i32(e, "priority").unwrap_or(0), attr_i32(e, "startFrameNumber").unwrap_or(0))
    });

    let mut lanes = Lanes::default();

    // Each character's starting placement and pose, held underneath everything else.
    // It's not a muscle target ("active" = false), just where things begin.
    let start_frame = engine
        .and_then(|e| child(e, "PlaybackRange"))
        .and_then(|r| attr_i32(&r, "start"))
        .unwrap_or(0);
    let mut start_poses = 0;
    // Where each character sits in the world: a start pose's root is relative to it (the
    // gymnast hangs 1.7 m up, Catch Rope's runner starts on a roof 15.8 m up). Animation
    // events carry their own placement in the world.
    let placement = |name: &str| {
        elements(child(root, "Characters"))
            .chain(child(root, "Environment"))
            .find(|n| n.attribute("name") == Some(name))
            .and_then(transform)
    };
    for pose in elements(child(root, "Poses")) {
        let Some(character) = pose.attribute("character").and_then(|n| project.character_by_name(n)).cloned() else { continue };
        let channels = pose_channels(pose, &character.skeleton, placement(&character.name));
        if channels.is_empty() {
            continue;
        }
        let track = lanes.track(&mut project, character.id, "Start pose", i32::MIN, i32::MIN);
        let props = ClipProps {
            name: "Start pose".to_owned(),
            track,
            start: start_frame,
            duration: 1_000_000,
            blend_in: 0,
            blend_out: 0,
            weight: 1.0,
            enabled: true,
        };
        let id = project.add_clip(props, ClipKind::Motion, channels);
        if let Some(clip) = project.clip_mut(id) {
            clip.source.insert("active".into(), "false".into());
            clip.source.insert("endorphin.type".into(), "Pose".into());
        }
        start_poses += 1;
    }
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    let mut missing_targets = 0;
    let mut skipped_channels = 0;

    // Keys the scene holds outside events: a prop driven along a path, a runner's run-up,
    // a rotating floor, poses set by hand. They're relative to each character's placement,
    // like its start pose. One clip per character for the whole scene, above the start
    // pose and below animation events; like the start pose, not a muscle target.
    let mut scene_animations = 0;
    if let Some(animation) = child(root, "Animation") {
        for character in project.characters.clone() {
            let channels = animation_channels(
                animation,
                Some(&character.name),
                0,
                placement(&character.name),
                &character.skeleton,
                fps,
                &mut skipped_channels,
            );
            if channels.is_empty() {
                continue;
            }
            let track = lanes.track(&mut project, character.id, "Scene animation", i32::MIN, i32::MIN);
            let props = ClipProps {
                name: "Scene animation".to_owned(),
                track,
                start: 0,
                duration: 1_000_000,
                blend_in: 0,
                blend_out: 0,
                weight: 1.0,
                enabled: true,
            };
            let id = project.add_clip(props, ClipKind::Motion, channels);
            if let Some(clip) = project.clip_mut(id) {
                clip.source.insert("active".into(), "false".into());
                clip.source.insert("endorphin.type".into(), "Animation".into());
            }
            scene_animations += 1;
        }
    }
    let (mut first, mut last) = (i32::MAX, i32::MIN);
    for event in events {
        let Some(character) = event
            .attribute("character")
            .and_then(|name| project.character_by_name(name))
            .cloned()
        else {
            missing_targets += 1;
            continue;
        };
        let tag = event.tag_name().name();
        let label = event_label(tag);
        let start = attr_i32(&event, "startFrameNumber").unwrap_or(0);
        let end = attr_i32(&event, "endFrameNumber").unwrap_or(start + 1).max(start + 1);
        first = first.min(start);
        last = last.max(end);

        let (kind, channels) = if tag == "AnimationEvent" {
            let channels = child(event, "Animation").map_or_else(Vec::new, |animation| {
                let offset = attr_i32(&event, "frameOffset").unwrap_or(0);
                animation_channels(animation, None, offset, transform(event), &character.skeleton, fps, &mut skipped_channels)
            });
            (ClipKind::Motion, channels)
        } else {
            // Active poses carry their target pose as held channels.
            let channels = child(event, "Pose")
                .map(|pose| pose_channels(pose, &character.skeleton, None))
                .unwrap_or_default();
            (ClipKind::Event { event: label.to_owned() }, channels)
        };
        let blend_in = attr_f32(event, "bentryBlendPeriod").map_or(0, |s| (s * fps).round() as i32);
        let props = ClipProps {
            name: event_name(event, label),
            track: lanes.track(&mut project, character.id, label, start, end),
            start,
            duration: end - start,
            blend_in: blend_in.clamp(0, end - start),
            blend_out: 0,
            weight: 1.0,
            enabled: event.attribute("enabled") != Some("false"),
        };
        let id = project.add_clip(props, kind, channels);
        if let Some(clip) = project.clip_mut(id) {
            clip.source = event_settings(event);
        }
        *counts.entry(label).or_default() += 1;
    }

    let playback = engine.and_then(|e| child(e, "PlaybackRange")).and_then(|r| {
        Some(FrameRange { start: attr_i32(&r, "start")?, end: attr_i32(&r, "end")? })
    });
    project.settings.range = match playback {
        Some(range) if range.end > range.start => range,
        _ if first < last => FrameRange { start: first.min(0), end: last },
        _ => FrameRange { start: 0, end: (fps * 5.0) as i32 },
    };
    let range = project.settings.range;
    for clip in project.clips.iter_mut().filter(|c| c.props.name == "Start pose" && c.props.duration == 1_000_000) {
        clip.props.start = range.start;
        clip.props.duration = (range.end - range.start).max(1);
    }

    let people = project.characters.iter().filter(|c| c.kind == CharacterKind::Character).count();
    notes.push(format!(
        "Imported {} character(s) and {} prop/environment object(s) at {fps:.0} fps.",
        people,
        project.characters.len() - people
    ));
    for (label, n) in &counts {
        notes.push(format!("{n} × {label}"));
    }
    if start_poses > 0 {
        notes.push(format!("{start_poses} starting pose(s) placed as held clips under each character's animation."));
    }
    if scene_animations > 0 {
        notes.push(format!("{scene_animations} character(s) keyed in the scene itself, outside events: a Scene animation clip each."));
    }
    let rigs = project.characters.iter().filter(|c| c.physics.is_some()).count();
    if rigs > 0 {
        notes.push(format!(
            "Physics: {rigs} rig(s) with bodies, joint limits and muscle presets; simulated at {:.0} Hz.",
            project.physics.rate
        ));
    }
    if counts.contains_key("Behaviour") {
        notes.push("Behaviours are kept; Balance and CatchFall run NeuroMotion's own controllers, the others (stagger, writhe, ...) aren't simulated yet.".into());
    }
    if skipped_joints > 0 {
        notes.push(format!("Skipped {skipped_joints} virtual-muscle helper joint(s)."));
    }
    if skipped_channels > 0 {
        notes.push(format!("Skipped {skipped_channels} animation channel(s) for joints the rig doesn't have."));
    }
    if missing_targets > 0 {
        notes.push(format!("Skipped {missing_targets} event(s) aimed at objects that aren't characters."));
    }
    project.normalize_ids();
    Ok(Import { project, notes })
}

/// Opens a character or prop file as a project with one empty track.
pub fn import_character(path: &Path) -> Result<Import, IoError> {
    let text = read_document(path)?;
    let xml = parse(&text)?;
    let root = xml.root_element();
    // Motion-transfer rig templates are a bare <Skeleton> of rig nodes.
    let is_rig = root.has_tag_name("Skeleton");
    if !root.has_tag_name("Character") && !is_rig {
        return Err(IoError::Format("not an endorphin character (.nmc)".into()));
    }
    let mut project = Project::new(ProjectSettings {
        name: file_stem(path),
        fps: 30.0,
        range: FrameRange { start: 0, end: 150 },
    });
    let mut skipped = 0;
    let kind = if root.attribute("typeName") == Some("animatableProp") || path_is_prop(path) {
        CharacterKind::Prop
    } else {
        CharacterKind::Character
    };
    let id = if is_rig {
        let mut joints = Vec::new();
        add_joint(root, None, &mut joints, &mut Vec::new(), &mut skipped);
        let skeleton = SkeletonRef { name: file_stem(path), source: None, joints };
        project.add_character("rig", kind, skeleton)
    } else {
        add_character(&mut project, root, kind, true, &mut skipped)
    };
    if let Some(c) = project.characters.iter_mut().find(|c| c.id == id) {
        c.skeleton.source = Some(path.display().to_string());
    }
    project.add_track(id, "Base");
    let joints = project.characters[0].skeleton.joints.len();
    let what = if is_rig { "motion-transfer rig".to_owned() } else { kind.label().to_lowercase() };
    let mut notes = vec![format!("Imported {what} with {joints} joints.")];
    if skipped > 0 {
        notes.push(format!("Skipped {skipped} virtual-muscle helper joint(s)."));
    }
    Ok(Import { project, notes })
}

/// Reads a pose and converts it to channels for the best-matching character.
pub fn import_pose(path: &Path, project: &Project) -> Result<PoseImport, IoError> {
    let text = read_document(path)?;
    let xml = parse(&text)?;
    let root = xml.root_element();
    if !root.has_tag_name("Pose") {
        return Err(IoError::Format("not an endorphin pose (.nma)".into()));
    }
    if child(root, "DesiredPose").is_some() {
        return Err(IoError::Format(
            "this pose stores muscle (effector) targets instead of joint transforms, which NeuroMotion can't import yet".into(),
        ));
    }
    if root.attribute("refFrame").is_some_and(|f| !f.eq_ignore_ascii_case("parent")) {
        return Err(IoError::Format("only poses stored relative to parent joints are supported".into()));
    }
    let entries: Vec<(&str, Quat, Vec3)> = elements(child(root, "Relationships"))
        .filter_map(|r| {
            let (rot, pos) = transform(r)?;
            Some((r.attribute("name")?, rot, pos))
        })
        .collect();

    // Prefer the character the pose names, otherwise whichever shares the most joints.
    let matches = |skeleton: &SkeletonRef| entries.iter().filter(|(n, ..)| skeleton.joint_index(n).is_some()).count();
    let named = root.attribute("character").and_then(|n| project.character_by_name(n));
    let character = named
        .filter(|c| matches(&c.skeleton) > 0)
        .or_else(|| project.characters.iter().max_by_key(|c| matches(&c.skeleton)))
        .filter(|c| matches(&c.skeleton) > 0)
        .ok_or_else(|| IoError::Format("none of the project's characters have this pose's joints".into()))?;

    let channels = pose_channels(root, &character.skeleton, None);
    Ok(PoseImport { character: character.id, name: file_stem(path), channels })
}

/// Converts a pose's per-joint transforms (`<Relationships>` under `pose`) into held
/// channels relative to the skeleton's rest pose. `placement`, where the character sits
/// in the world, goes under its roots.
fn pose_channels(pose: Node, skeleton: &SkeletonRef, placement: Option<(Quat, Vec3)>) -> Vec<Channel> {
    let mut transforms: Vec<(usize, Quat, Vec3)> = elements(child(pose, "Relationships"))
        .filter_map(|r| {
            let (rot, pos) = transform(r)?;
            Some((skeleton.joint_index(r.attribute("name")?)?, rot, pos))
        })
        .collect();
    // Roots sit at `placement`, at rest where the pose doesn't move them.
    if let Some((place_rot, place_pos)) = placement {
        for (index, joint) in skeleton.joints.iter().enumerate().filter(|(_, j)| j.parent.is_none()) {
            if !transforms.iter().any(|t| t.0 == index) {
                transforms.push((index, Quat::from_array(joint.rest_rotation).normalize(), Vec3::from(joint.offset)));
            }
        }
        for (_, rot, pos) in transforms.iter_mut().filter(|t| skeleton.joints[t.0].parent.is_none()) {
            (*rot, *pos) = (place_rot * *rot, place_pos + place_rot * *pos);
        }
    }
    let mut channels = Vec::new();
    for (index, rot, pos) in transforms {
        let joint = &skeleton.joints[index];
        let name = joint.name.as_str();
        let euler = relative_euler(joint, rot);
        let offset = pos - Vec3::from(joint.offset);
        for (property, value) in rotation_props().into_iter().zip(euler).chain(position_props().into_iter().zip(offset.to_array())) {
            let near_zero = if property.is_rotation() { value.abs() < 0.01 } else { value.abs() < 1e-4 };
            if !near_zero {
                channels.push(Channel {
                    joint: name.to_owned(),
                    property,
                    keys: vec![Keyframe { frame: 0, value, interp: Interp::Linear }],
                });
            }
        }
    }
    channels
}

// ---------------------------------------------------------------------------------
// Characters

fn add_character(project: &mut Project, node: Node, kind: CharacterKind, isotropic: bool, skipped: &mut usize) -> CharacterId {
    let mut joints = Vec::new();
    let mut nodes = Vec::new();
    if let Some(root) = child(node, "Skeleton") {
        add_joint(root, None, &mut joints, &mut nodes, skipped);
    }
    let source = node.attribute("pathName").filter(|p| !p.is_empty()).map(str::to_owned);
    let rig = source
        .as_deref()
        .map(|p| file_stem(Path::new(&p.replace('\\', "/"))))
        .unwrap_or_else(|| kind.label().to_owned());
    let name = node.attribute("name").unwrap_or("character");
    let skeleton = SkeletonRef { name: rig, source, joints };
    let physics = read_rig(node, &nodes, &skeleton, isotropic, kind == CharacterKind::Environment);
    let id = project.add_character(name, kind, skeleton);
    if let Some(c) = project.characters.iter_mut().find(|c| c.id == id) {
        c.physics = physics;
    }
    id
}

fn add_joint<'a, 'i>(
    node: Node<'a, 'i>,
    parent: Option<usize>,
    joints: &mut Vec<Joint>,
    nodes: &mut Vec<Node<'a, 'i>>,
    skipped: &mut usize,
) {
    let name = node.attribute("name").unwrap_or("joint");
    // Virtual muscles are solver helpers linking distant bodies, not bones.
    if node.attribute("isVirtualMuscle") == Some("true") || name.contains("VirtualMuscle") {
        *skipped += 1;
        return;
    }
    let (rotation, position) = transform(node).unwrap_or((Quat::IDENTITY, Vec3::ZERO));
    joints.push(Joint {
        name: name.to_owned(),
        parent,
        offset: position.to_array(),
        rest_rotation: rotation.to_array(),
    });
    nodes.push(node);
    let index = joints.len() - 1;
    // Skeletons nest joints in <ChildJoints>; rig templates nest rig nodes in <ChildNodes>.
    for c in elements(child(node, "ChildJoints")).chain(elements(child(node, "ChildNodes"))) {
        add_joint(c, Some(index), joints, nodes, skipped);
    }
}

// ---------------------------------------------------------------------------------
// Animation

#[derive(Default)]
struct JointTrack {
    rotation: Vec<(f32, Quat)>,
    position: [Vec<(f32, f32)>; 3],
}

/// Channels from an `<Animation>` element: an animation event's (`owner` `None`), or the
/// scene's own, where each channel name starts with the character it keys (`owner`).
/// `placement` places the root in the world: the event's matrix, or the character's.
fn animation_channels(
    animation: Node,
    owner: Option<&str>,
    frame_offset: i32,
    placement: Option<(Quat, Vec3)>,
    skeleton: &SkeletonRef,
    fps: f32,
    skipped: &mut usize,
) -> Vec<Channel> {
    let mut tracks: BTreeMap<usize, JointTrack> = BTreeMap::new();
    for channel in elements(Some(animation)) {
        let Some(full) = channel.attribute("name") else { continue };
        let Some(full) = owner.map_or(Some(full), |o| full.strip_prefix(o).and_then(|rest| rest.strip_prefix('.'))) else { continue };
        let mut parts = full.rsplit('.');
        let (Some(property), Some(joint)) = (parts.next(), parts.next()) else { continue };
        let Some(index) = skeleton.joint_index(joint) else {
            *skipped += 1;
            continue;
        };
        let track = tracks.entry(index).or_default();
        let keys = elements(Some(channel));
        match (channel.tag_name().name(), property) {
            ("QuatChannel", _) => track.rotation = keys.filter_map(|k| {
                let q = Quat::from_xyzw(attr_f32(k, "by")?, attr_f32(k, "bz")?, attr_f32(k, "bw")?, attr_f32(k, "bx")?);
                Some((attr_f32(k, "bt")?, q.normalize()))
            }).collect(),
            ("Channel", "tx" | "ty" | "tz") => {
                let axis = (property.as_bytes()[1] - b'x') as usize;
                track.position[axis] = keys.filter_map(|k| Some((attr_f32(k, "bx")?, attr_f32(k, "by")?))).collect();
            }
            _ => *skipped += 1,
        }
    }

    // The placement puts the whole animation in the scene: bake it into the root. The
    // scene's own keys leave a root they don't key to the start pose.
    if let Some((place_rot, place_pos)) = placement {
        let is_identity = place_rot.angle_between(Quat::IDENTITY) < 1e-4 && place_pos.length() < 1e-5;
        let root = skeleton.joints.iter().position(|j| j.parent.is_none());
        if let Some(root) = root.filter(|r| !is_identity && (owner.is_none() || tracks.contains_key(r))) {
            let joint = &skeleton.joints[root];
            let track = tracks.entry(root).or_default();
            if track.rotation.is_empty() {
                track.rotation.push((0.0, Quat::from_array(joint.rest_rotation)));
            }
            for (axis, keys) in track.position.iter_mut().enumerate() {
                if keys.is_empty() {
                    keys.push((0.0, joint.offset[axis]));
                }
            }
            for (_, q) in &mut track.rotation {
                *q = (place_rot * *q).normalize();
            }
            let times = merged_times(&track.position);
            let sampled: Vec<(f32, Vec3)> = times
                .into_iter()
                .map(|t| {
                    let p = Vec3::new(sample_linear(&track.position[0], t), sample_linear(&track.position[1], t), sample_linear(&track.position[2], t));
                    (t, place_rot * p + place_pos)
                })
                .collect();
            for axis in 0..3 {
                track.position[axis] = sampled.iter().map(|(t, p)| (*t, p[axis])).collect();
            }
        }
    }

    let to_frame = |t: f32| (t * fps).round() as i32 - frame_offset;
    let mut channels = Vec::new();
    for (index, track) in tracks {
        let joint = &skeleton.joints[index];
        if !track.rotation.is_empty() {
            let mut previous = [0.0f32; 3];
            let mut keys: [Vec<Keyframe>; 3] = Default::default();
            for (i, (t, q)) in track.rotation.iter().enumerate() {
                let mut euler = relative_euler(joint, *q);
                if i > 0 {
                    unwrap_degrees(&mut euler, previous);
                }
                previous = euler;
                for axis in 0..3 {
                    push_key(&mut keys[axis], to_frame(*t), euler[axis]);
                }
            }
            for (property, keys) in rotation_props().into_iter().zip(keys) {
                push_channel(&mut channels, &joint.name, property, keys, 0.01);
            }
        }
        for (axis, property) in position_props().into_iter().enumerate() {
            let mut keys = Vec::new();
            for (t, value) in &track.position[axis] {
                push_key(&mut keys, to_frame(*t), value - joint.offset[axis]);
            }
            push_channel(&mut channels, &joint.name, property, keys, 1e-4);
        }
    }
    channels
}

/// XYZ Euler angles (degrees) that take a joint from its rest orientation to `full`.
fn relative_euler(joint: &Joint, full: Quat) -> [f32; 3] {
    let rest = Quat::from_array(joint.rest_rotation).normalize();
    let (x, y, z) = (rest.inverse() * full).normalize().to_euler(EulerRot::XYZ);
    [x.to_degrees(), y.to_degrees(), z.to_degrees()]
}

/// Shifts each angle by whole turns so it lands within 180° of the previous key,
/// keeping curves continuous instead of jumping at ±180°.
fn unwrap_degrees(angles: &mut [f32; 3], previous: [f32; 3]) {
    for (a, p) in angles.iter_mut().zip(previous) {
        *a -= ((*a - p) / 360.0).round() * 360.0;
    }
}

fn push_key(keys: &mut Vec<Keyframe>, frame: i32, value: f32) {
    let key = Keyframe { frame, value, interp: Interp::Linear };
    match keys.last_mut() {
        Some(last) if last.frame == frame => *last = key,
        _ => keys.push(key),
    }
}

/// Adds the channel unless every key is (near) zero, i.e. it never leaves the rest pose.
fn push_channel(channels: &mut Vec<Channel>, joint: &str, property: Property, mut keys: Vec<Keyframe>, epsilon: f32) {
    keys.sort_by_key(|k| k.frame);
    keys.dedup_by_key(|k| k.frame);
    if keys.iter().all(|k| k.value.abs() < epsilon) {
        return;
    }
    channels.push(Channel { joint: joint.to_owned(), property, keys });
}

fn rotation_props() -> [Property; 3] {
    [Property::RotX, Property::RotY, Property::RotZ]
}

fn position_props() -> [Property; 3] {
    [Property::PosX, Property::PosY, Property::PosZ]
}

fn merged_times(channels: &[Vec<(f32, f32)>; 3]) -> Vec<f32> {
    let mut times: Vec<f32> = channels.iter().flatten().map(|(t, _)| *t).collect();
    times.sort_by(f32::total_cmp);
    times.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    times
}

fn sample_linear(keys: &[(f32, f32)], t: f32) -> f32 {
    match keys.iter().position(|(kt, _)| *kt >= t) {
        None => keys.last().map_or(0.0, |k| k.1),
        Some(0) => keys[0].1,
        Some(i) => {
            let ((t0, v0), (t1, v1)) = (keys[i - 1], keys[i]);
            v0 + (v1 - v0) * ((t - t0) / (t1 - t0).max(1e-9))
        }
    }
}

// ---------------------------------------------------------------------------------
// Events

/// Groups events of one kind onto as few tracks per character as possible.
#[derive(Default)]
struct Lanes {
    lanes: Vec<(CharacterId, &'static str, crate::document::TrackId, i32)>,
}

impl Lanes {
    fn track(&mut self, project: &mut Project, character: CharacterId, label: &'static str, start: i32, end: i32) -> crate::document::TrackId {
        if let Some(lane) = self.lanes.iter_mut().find(|l| l.0 == character && l.1 == label && l.3 <= start) {
            lane.3 = end;
            return lane.2;
        }
        let existing = self.lanes.iter().filter(|l| l.0 == character && l.1 == label).count();
        let plural = if matches!(label, "Animation" | "Start pose" | "Scene animation") { label.to_owned() } else { format!("{label}s") };
        let name = if existing == 0 { plural } else { format!("{plural} {}", existing + 1) };
        let id = project.add_track(character, &name);
        self.lanes.push((character, label, id, end));
        id
    }
}

/// Friendly name for an endorphin event element.
fn event_label(tag: &str) -> &'static str {
    match tag {
        "AnimationEvent" => "Animation",
        "ForceEvent" => "Force",
        "SecondaryBehaviour" => "Behaviour",
        "TertiaryBehaviour" => "Constraint",
        "IncarnationEvent" => "Simulation",
        "DesiredPoseEvent" => "Active pose",
        "TransitionEvent" => "Transition",
        "RigEvent" => "Motion transfer",
        "SeverEvent" => "Sever",
        _ => "Event",
    }
}

fn event_name(event: Node, label: &str) -> String {
    ["displayName", "behaviourName", "name"]
        .into_iter()
        .find_map(|a| event.attribute(a).filter(|v| !v.is_empty()))
        .unwrap_or(label)
        .to_owned()
}

/// The event's own settings, decoded, minus what NeuroMotion already models.
fn event_settings(event: Node) -> BTreeMap<String, String> {
    const MODELLED: [&str; 10] = [
        "name", "displayName", "startFrameNumber", "endFrameNumber", "character", "enabled",
        "visible", "retailVersionVisible", "addToFrameBuffer", "animationVisible",
    ];
    let mut out = BTreeMap::new();
    out.insert("endorphin.type".to_owned(), event.tag_name().name().to_owned());
    for attr in event.attributes() {
        if MODELLED.contains(&attr.name()) {
            continue;
        }
        match decode_float_attr(attr.name(), attr.value()) {
            Some((name, value)) => out.insert(name, format_float(value)),
            None => out.insert(attr.name().to_owned(), attr.value().to_owned()),
        };
    }
    if let Some(m) = child(event, "TransformMatrix") {
        for axis in ["Vi", "Vj", "Vk", "Vp"] {
            if let Some(v) = vec3(child(m, axis)) {
                out.insert(format!("matrix.{axis}"), format!("{}, {}, {}", format_float(v.x), format_float(v.y), format_float(v.z)));
            }
        }
    }
    // Vector settings stored as child elements (e.g. constraint strengths).
    for c in elements(Some(event)) {
        if let (Some(v), true) = (vec3(Some(c)), c.tag_name().name() != "TransformMatrix") {
            let tag = c.tag_name().name();
            let key = tag[..1].to_ascii_lowercase() + &tag[1..];
            out.insert(key, format!("{}, {}, {}", format_float(v.x), format_float(v.y), format_float(v.z)));
        }
    }
    let targets: Vec<&str> = elements(child(event, "Objects")).filter_map(|o| o.text()).collect();
    if !targets.is_empty() {
        out.insert("targets".to_owned(), targets.join(", "));
    }
    out
}

// ---------------------------------------------------------------------------------
// Physics

/// Scene-wide settings: simulation rate, gravity, ground and contact materials.
fn read_physics_settings(project: &mut Project, scene: Node, engine: Node) {
    let settings = &mut project.physics;
    if let Some(rate) = attr_f32(engine, "bsimulationFrameRate").filter(|r| *r > 0.0) {
        settings.rate = rate;
    }
    if let Some(g) = vec3(child(engine, "Gravity")) {
        settings.gravity = g.to_array();
    }
    settings.ground = (scene.attribute("useDefaultGround") != Some("false"))
        .then(|| Ground { height: 0.0, material: "tarmac".into() });
    let Some(options) = child(engine, "MaterialOptions") else { return };
    let interactions: BTreeMap<String, Interaction> = elements(child(options, "MaterialInteractions"))
        .filter_map(|i| {
            Some((
                i.attribute("name")?.to_owned(),
                Interaction {
                    restitution: number(i, "restitution").unwrap_or(0.2),
                    friction: number(i, "friction").unwrap_or(100.0),
                    adhesion: number(i, "maxAdhesiveForce").unwrap_or(0.0),
                    softness: number(i, "softness").unwrap_or(0.0),
                },
            ))
        })
        .collect();
    if !interactions.is_empty() {
        settings.interactions = interactions;
        settings.pairs = elements(child(options, "MaterialPairs"))
            .filter_map(|p| {
                Some((
                    p.attribute("materialA")?.to_owned(),
                    p.attribute("materialB")?.to_owned(),
                    p.attribute("interaction")?.to_owned(),
                ))
            })
            .collect();
    }
}

/// Bodies, joints and muscle presets of a character (or prop, or environment).
/// `joint_nodes[i]` is the XML element of skeleton joint `i`.
/// `scenery`: the environment, whose pieces without an inertial body stay fixed in place.
fn read_rig(node: Node, joint_nodes: &[Node], skeleton: &SkeletonRef, isotropic: bool, scenery: bool) -> Option<PhysicsRig> {
    let body_of: Vec<Option<&str>> = joint_nodes.iter().map(|n| n.attribute("metaBody")).collect();
    let mut bodies = Vec::new();
    for mb in elements(child(node, "MetaBodies")).filter(|n| n.has_tag_name("MetaBody")) {
        let Some(name) = mb.attribute("name") else { continue };
        // A body belongs to the first (outermost) joint that names it; scenery pieces that
        // no joint names hang from the root.
        let Some(joint) = body_of.iter().position(|b| *b == Some(name)).or((!joint_nodes.is_empty()).then_some(0)) else { continue };
        let (rotation, position) = transform(mb).unwrap_or((Quat::IDENTITY, Vec3::ZERO));
        let (mass, inertia) = child(mb, "InertialBody")
            .and_then(|ib| {
                let geometry = geometry(child(ib, "Geometry")?)?;
                let density = child(ib, "Inertia").and_then(|i| number(i, "density")).unwrap_or(1.0);
                // endorphin densities are in g/cm^3.
                let mass = density * 1000.0 * geometry.volume();
                Some((mass, geometry.inertia(mass)))
            })
            // No mass (no inertial body, or an empty one): scenery stays fixed in place, like
            // the ground. A character's helper body (Back-Flip-To-Combo's zero-sized
            // NormalMass) gets a token mass instead, or it would hold the character in the air.
            .filter(|&(mass, _)| scenery || mass > 0.0)
            .unwrap_or(if scenery { (0.0, [0.0; 3]) } else { (1.0, [0.004; 3]) });
        let shapes = elements(Some(mb))
            .filter(|n| n.has_tag_name("CollisionSurface"))
            .filter_map(|cs| {
                // A collision surface without geometry of its own collides with the body's
                // mass shape: how endorphin gives props ("environment mass objects have a
                // collision object by default"), ropes and other rigs made in it collision.
                let shape = child(cs, "Geometry").or_else(|| child(mb, "InertialBody").and_then(|ib| child(ib, "Geometry")));
                let placed = if child(cs, "Geometry").is_some() { cs } else { child(mb, "InertialBody")? };
                let (r, p) = transform(placed).unwrap_or((Quat::IDENTITY, Vec3::ZERO));
                let flag = |name: &str| cs.attribute(name) != Some("false");
                Some(ShapeDef {
                    geometry: geometry(shape?)?,
                    frame: Frame { rotation: r.to_array(), position: p.to_array() },
                    material: cs.attribute("material").unwrap_or("default").to_owned(),
                    collides_with_parent: flag("collidesWithParent"),
                    collides_with_siblings: flag("collidesWithSiblings"),
                    collides_with_ancestors: flag("collidesWithAncestors"),
                })
            })
            .collect();
        bodies.push(BodyDef {
            name: name.to_owned(),
            joint,
            frame: Frame { rotation: rotation.to_array(), position: position.to_array() },
            mass,
            inertia,
            shapes,
        });
    }
    if bodies.is_empty() {
        return None;
    }
    let body_index = |name: &str| bodies.iter().position(|b| b.name == name);

    let effector_of: BTreeMap<&str, &str> = node
        .descendants()
        .filter(|n| n.has_tag_name("Effector"))
        .filter_map(|e| Some((e.attribute("jointName")?, e.attribute("name")?)))
        .collect();

    let mut joints = Vec::new();
    for (j, jn) in joint_nodes.iter().enumerate() {
        let tag = jn.tag_name().name();
        if tag != "SkeletalJoint" && tag != "Hinge" {
            continue;
        }
        let Some(body) = body_of[j].and_then(body_index) else { continue };
        // The parent body is the nearest ancestor joint's body that isn't this one.
        let mut ancestor = skeleton.joints[j].parent;
        let mut parent_body = None;
        while let Some(a) = ancestor {
            if let Some(b) = body_of[a].and_then(body_index).filter(|b| *b != body) {
                parent_body = Some(b);
                break;
            }
            ancestor = skeleton.joints[a].parent;
        }
        let Some(parent_body) = parent_body else { continue };
        let limit_frame = |tag: &str| {
            child(*jn, tag)
                .and_then(|f| basis(child(f, "TransformMatrix")?))
                .unwrap_or(Quat::IDENTITY)
                .to_array()
        };
        let (hard, soft, stiffness, damping) = if tag == "Hinge" {
            hinge_limits(*jn)
        } else {
            let limits = |tag: &str| {
                child(*jn, tag).map(|l| {
                    Limits::from_sizes(
                        number(l, "twistSize").unwrap_or(0.0),
                        number(l, "swing1Size").unwrap_or(0.0),
                        number(l, "swing2Size").unwrap_or(0.0),
                    )
                })
            };
            let hard = limits("HardLimits").unwrap_or(Limits::from_sizes(6.28, 6.0, 6.0));
            let soft_node = child(*jn, "SoftLimits");
            let avg = |a: &str, b: &str| {
                soft_node.map_or(0.0, |s| (number(s, a).unwrap_or(0.0) + number(s, b).unwrap_or(0.0)) * 0.5)
            };
            (hard, limits("SoftLimits").unwrap_or(hard), avg("twistStiffness", "swingStiffness"), avg("twistDamping", "swingDamping"))
        };
        let name = skeleton.joints[j].name.clone();
        joints.push(JointDef {
            muscle: effector_of.get(name.as_str()).map_or_else(|| name.clone(), |e| (*e).to_owned()),
            name,
            joint: j,
            body,
            parent_body,
            primary: limit_frame("PrimaryFrameOffset"),
            secondary: limit_frame("SecondaryFrameOffset"),
            hard,
            soft,
            soft_stiffness: stiffness,
            soft_damping: damping,
        });
    }

    let muscle_sets = elements(child(node, "Calibrations"))
        .filter_map(|cal| {
            let skeletal = elements(Some(cal))
                .filter(|p| p.has_tag_name("SkeletalEffectorParameters"))
                .filter_map(|p| {
                    let gains = |axis: &str| Gains {
                        stiffness: number(p, &format!("{axis}Stiffness")).unwrap_or(0.0),
                        damping: number(p, &format!("{axis}Damping")).unwrap_or(0.0),
                        force_limit: number(p, &format!("{axis}ForceLimit")).unwrap_or(0.0),
                    };
                    Some((p.attribute("name")?.to_owned(), MuscleGains { twist: gains("twist"), swing: gains("swing") }))
                });
            // 1-DOF effectors (finger hinges): gradient acts as stiffness, capped by the
            // base force cap. Damping isn't stored; a tenth of the gradient keeps them calm.
            let single = elements(Some(cal)).filter(|p| p.has_tag_name("EffectorParameters")).filter_map(|p| {
                let value = |tag: &str| child(p, tag)?.text()?.trim().parse::<f32>().ok();
                let gradient = value("Gradient").unwrap_or(1.0);
                let g = Gains { stiffness: gradient, damping: gradient * 0.1, force_limit: value("BaseForceCap").unwrap_or(2.0) };
                Some((p.attribute("name")?.to_owned(), MuscleGains { twist: g, swing: g }))
            });
            let muscles: BTreeMap<String, MuscleGains> = skeletal.chain(single).collect();
            let fallback = *muscles.values().next()?;
            Some(MuscleSet { name: cal.attribute("name")?.to_owned(), muscles, fallback })
        })
        .collect();

    Some(PhysicsRig { bodies, joints, muscle_sets, isotropic_inertia: isotropic })
}

/// A hinge's (lower, upper) range, as twist limits with swing locked.
fn hinge_limits(node: Node) -> (Limits, Limits, f32, f32) {
    let bound = |tag: &str, key: &str| {
        child(node, "JointLimits")
            .and_then(|l| child(l, tag))
            .and_then(|b| child(b, key))
            .and_then(|v| v.text())
            .and_then(|t| t.trim().parse::<f32>().ok())
    };
    let (lower, upper) = (bound("Lower", "Value").unwrap_or(-3.1), bound("Upper", "Value").unwrap_or(3.1));
    let limits = Limits { twist_min: lower, twist_max: upper, swing1: 0.01, swing2: 0.01 };
    (limits, limits, bound("Lower", "Stiffness").unwrap_or(0.0), bound("Lower", "Damping").unwrap_or(0.0))
}

fn geometry(g: Node) -> Option<Geometry> {
    let p = |i: usize| number(g, &format!("param{i}"));
    Some(match g.attribute("type")? {
        "Sphere" => Geometry::Sphere { radius: p(0)? },
        "Sphyl" => Geometry::Capsule { radius: p(0)?, length: p(1)? },
        "Cylinder" => Geometry::Cylinder { radius: p(0)?, length: p(1)? },
        "Cuboid" => Geometry::Box { size: [p(0)?, p(1)?, p(2)?] },
        _ => return None,
    })
}

/// A number stored either as a `b`-prefixed float bit pattern or as plain text.
fn number(node: Node, name: &str) -> Option<f32> {
    attr_f32(node, &format!("b{name}")).or_else(|| node.attribute(name)?.trim().parse().ok())
}

/// A `TransformMatrix`'s rotation only.
fn basis(m: Node) -> Option<Quat> {
    let basis = Mat3::from_cols(vec3(child(m, "Vi"))?, vec3(child(m, "Vj"))?, vec3(child(m, "Vk"))?);
    Some(Quat::from_mat3(&basis).normalize())
}

// ---------------------------------------------------------------------------------
// Low-level helpers

/// Reads an endorphin document, inflating it if compressed.
pub fn read_document(path: &Path) -> Result<String, IoError> {
    decode_document(&fs::read(path)?)
}

fn decode_document(bytes: &[u8]) -> Result<String, IoError> {
    let mut text = String::new();
    if ZlibDecoder::new(bytes).read_to_string(&mut text).is_ok() && text.trim_start().starts_with('<') {
        return Ok(text);
    }
    match std::str::from_utf8(bytes) {
        Ok(text) if text.trim_start().starts_with('<') => Ok(text.to_owned()),
        _ => Err(IoError::Format(
            "this file isn't readable endorphin data. If it was exported with a password, \
             re-export it from endorphin without one."
                .into(),
        )),
    }
}

fn parse(text: &str) -> Result<roxmltree::Document<'_>, IoError> {
    roxmltree::Document::parse(text).map_err(|e| IoError::Format(format!("invalid endorphin XML: {e}")))
}

fn child<'a, 'i>(node: Node<'a, 'i>, tag: &str) -> Option<Node<'a, 'i>> {
    node.children().find(|c| c.has_tag_name(tag))
}

fn elements<'a, 'i>(node: Option<Node<'a, 'i>>) -> impl Iterator<Item = Node<'a, 'i>> {
    node.into_iter().flat_map(|n| n.children()).filter(Node::is_element)
}

fn hex_f32(value: &str) -> Option<f32> {
    (value.len() == 8).then(|| u32::from_str_radix(value, 16).ok().map(f32::from_bits)).flatten()
}

fn decode_float_attr(name: &str, value: &str) -> Option<(String, f32)> {
    let rest = name.strip_prefix('b')?;
    rest.chars().next().filter(char::is_ascii_lowercase)?;
    Some((rest.to_owned(), hex_f32(value)?))
}

fn attr_f32(node: Node, name: &str) -> Option<f32> {
    hex_f32(node.attribute(name)?)
}

fn attr_i32(node: &Node, name: &str) -> Option<i32> {
    node.attribute(name)?.trim().parse().ok()
}

fn vec3(node: Option<Node>) -> Option<Vec3> {
    let n = node?;
    Some(Vec3::new(attr_f32(n, "bx")?, attr_f32(n, "by")?, attr_f32(n, "bz")?))
}

/// A node's `TransformMatrix` as rotation + position (relative to its parent).
fn transform(node: Node) -> Option<(Quat, Vec3)> {
    let m = child(node, "TransformMatrix")?;
    let basis = Mat3::from_cols(vec3(child(m, "Vi"))?, vec3(child(m, "Vj"))?, vec3(child(m, "Vk"))?);
    Some((Quat::from_mat3(&basis).normalize(), vec3(child(m, "Vp"))?))
}

fn format_float(v: f32) -> String {
    if v.is_finite() { format!("{}", (v * 1e5).round() / 1e5) } else { v.to_string() }
}

fn file_stem(path: &Path) -> String {
    path.file_stem().map_or_else(|| "Imported".to_owned(), |s| s.to_string_lossy().into_owned())
}

fn path_is_prop(path: &Path) -> bool {
    path.components().any(|c| c.as_os_str().eq_ignore_ascii_case("Props"))
}

/// Joint-level checks against a hand-written scene (no endorphin install needed).
#[cfg(test)]
mod tests {
    use super::*;

    fn hex(v: f32) -> String {
        format!("{:08x}", v.to_bits())
    }

    fn matrix(cols: [[f32; 3]; 3], p: [f32; 3]) -> String {
        let v = |tag: &str, x: [f32; 3]| format!(r#"<{tag} bx="{}" by="{}" bz="{}"/>"#, hex(x[0]), hex(x[1]), hex(x[2]));
        format!("<TransformMatrix>{}{}{}{}</TransformMatrix>", v("Vi", cols[0]), v("Vj", cols[1]), v("Vk", cols[2]), v("Vp", p))
    }

    /// A two-joint character whose child bone points along the parent's local +X, with
    /// the root rotated so local +X is world up (like endorphin's spine).
    fn scene_xml() -> String {
        let up = matrix([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 1.0, 0.0]);
        let ident = matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.5, 0.0, 0.0]);
        let place = matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [2.0, 0.0, 0.0]);
        // A key that rotates the child 90° about Z: w, x, y, z in bx, by, bz, bw.
        let s = std::f32::consts::FRAC_1_SQRT_2;
        format!(
            r#"<?xml version="1.0"?>
<Scene>
  <Engine bpositionBufferFrameRate="{fps}"><PlaybackRange start="0" end="30"/></Engine>
  <Characters>
    <Character name="hero" typeName="skeletalCharacter2.5">
      <Skeleton name="root">{up}<ChildJoints>
        <SkeletalJoint name="Spine">{ident}</SkeletalJoint>
        <SkeletalJoint name="LeftArmVirtualMuscle" isVirtualMuscle="true">{ident}</SkeletalJoint>
      </ChildJoints></Skeleton>
    </Character>
  </Characters>
  <Events>
    <AnimationEvent name="1" displayName="Bend" startFrameNumber="10" endFrameNumber="40" character="hero" frameOffset="0" enabled="true" bstrength="{strength}">
      {place}
      <Animation refFrame="parent">
        <QuatChannel name="hero.Joints.Spine.quat" type="linear">
          <Key bt="{t0}" bx="{one}" by="{zero}" bz="{zero}" bw="{zero}"/>
          <Key bt="{t1}" bx="{s}" by="{zero}" bz="{zero}" bw="{s}"/>
        </QuatChannel>
        <QuatChannel name="hero.Joints.Missing.quat" type="linear"><Key bt="{t0}" bx="{one}" by="{zero}" bz="{zero}" bw="{zero}"/></QuatChannel>
      </Animation>
    </AnimationEvent>
  </Events>
  <ForceEvents>
    <ForceEvent name="Push" startFrameNumber="20" endFrameNumber="21" character="hero" brelativeAcceleration="{rel}">
      <Objects><Object>hero.MetaBodies.Chest</Object></Objects>
    </ForceEvent>
  </ForceEvents>
</Scene>"#,
            fps = hex(30.0),
            strength = hex(0.25),
            rel = hex(0.2),
            t0 = hex(0.0),
            t1 = hex(1.0),
            one = hex(1.0),
            zero = hex(0.0),
            s = hex(s),
        )
    }

    fn import(xml: &str) -> Import {
        // Tests run in parallel: each import gets its own file.
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("neuromotion-test-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join(format!("scene-{n}.ens"));
        let mut encoder = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
        std::io::Write::write_all(&mut encoder, xml.as_bytes()).unwrap();
        fs::write(&path, encoder.finish().unwrap()).unwrap();
        import_scene(&path).unwrap()
    }

    #[test]
    fn scene_skeleton_and_settings() {
        let Import { project, notes } = import(&scene_xml());
        assert_eq!(project.settings.fps, 30.0);
        assert_eq!(project.settings.range, FrameRange { start: 0, end: 30 });
        let hero = &project.characters[0];
        let names: Vec<_> = hero.skeleton.joints.iter().map(|j| j.name.as_str()).collect();
        assert_eq!(names, ["root", "Spine"], "virtual muscles are skipped");
        assert!(notes.iter().any(|n| n.contains("virtual-muscle")));

        // The root's rest rotation maps its local +X to world up.
        let rest = Quat::from_array(hero.skeleton.joints[0].rest_rotation);
        assert!((rest * Vec3::X - Vec3::Y).length() < 1e-5);
    }

    #[test]
    fn animation_becomes_relative_keys() {
        let Import { project, .. } = import(&scene_xml());
        let clip = project.clips.iter().find(|c| c.kind.is_motion()).unwrap();
        assert_eq!((clip.props.start, clip.props.duration), (10, 30));
        assert_eq!(clip.props.name, "Bend");
        assert_eq!(clip.source.get("strength").map(String::as_str), Some("0.25"));

        let rz = clip.channels.iter().find(|c| c.joint == "Spine" && c.property == Property::RotZ).unwrap();
        assert_eq!(rz.keys.len(), 2);
        assert_eq!(rz.keys[1].frame, 30, "1 s at 30 fps");
        assert!((rz.keys[1].value - 90.0).abs() < 1e-3);

        // The event's placement matrix lands on the root as a world offset.
        let px = clip.channels.iter().find(|c| c.joint == "root" && c.property == Property::PosX).unwrap();
        assert!((px.keys[0].value - 2.0).abs() < 1e-5);
    }

    #[test]
    fn physics_events_keep_their_settings() {
        let Import { project, .. } = import(&scene_xml());
        let force = project.clips.iter().find(|c| !c.kind.is_motion()).unwrap();
        assert_eq!(force.kind, ClipKind::Event { event: "Force".into() });
        assert_eq!(force.source.get("relativeAcceleration").map(String::as_str), Some("0.2"));
        assert_eq!(force.source.get("targets").map(String::as_str), Some("hero.MetaBodies.Chest"));
        let track = project.track(force.props.track).unwrap();
        assert_eq!(track.name, "Forces");
    }

    /// A crate placed at (3, 2, 0) and turned 90° about Y, with a start pose, a mass
    /// object whose collision surface has no geometry of its own, and keys in the scene's
    /// own animation (plus a camera's, which no character owns).
    fn crate_scene_xml() -> String {
        let turned = matrix([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]], [3.0, 2.0, 0.0]);
        let rest = matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0]);
        let posed = matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 1.0, 0.5]);
        let key = |t: f32, v: f32| format!(r#"<Key bx="{}" by="{}"/>"#, hex(t), hex(v));
        format!(
            r#"<?xml version="1.0"?>
<Scene>
  <Engine bpositionBufferFrameRate="{fps}"><PlaybackRange start="0" end="30"/></Engine>
  <Characters>
    <Character name="crate" typeName="animatableProp">
      {turned}
      <MetaBodies>
        <MetaBody name="rootMB">{rest}
          <CollisionSurface collidesWithParent="true" collidesWithSiblings="true" collidesWithAncestors="true"/>
          <InertialBody><Geometry type="Cuboid" bparam0="{x}" bparam1="{y}" bparam2="{z}"/><Inertia density="1.0"/></InertialBody>
        </MetaBody>
      </MetaBodies>
      <Skeleton name="root" metaBody="rootMB">{rest}</Skeleton>
    </Character>
  </Characters>
  <Poses>
    <Pose character="crate" refFrame="Parent"><Relationships><Relationship name="root">{posed}</Relationship></Relationships></Pose>
  </Poses>
  <Animation refFrame="parent">
    <Channel name="crate.root.tx" type="linear">{tx0}{tx1}</Channel>
    <Channel name="crate.root.ty" type="linear">{ty0}{ty1}</Channel>
    <Channel name="crate.root.tz" type="linear">{tz0}{tz1}</Channel>
    <Channel name="Camera01.tx" type="linear">{tx0}</Channel>
  </Animation>
</Scene>"#,
            fps = hex(30.0),
            x = hex(0.4),
            y = hex(0.2),
            z = hex(0.6),
            tx0 = key(0.0, 0.0),
            tx1 = key(1.0, 2.0),
            ty0 = key(0.0, 1.0),
            ty1 = key(1.0, 1.0),
            tz0 = key(0.0, 0.5),
            tz1 = key(1.0, 0.5),
        )
    }

    #[test]
    fn placement_mass_object_collision_and_scene_keys() {
        let Import { project, notes } = import(&crate_scene_xml());
        let shapes = &project.characters[0].physics.as_ref().unwrap().bodies[0].shapes;
        assert!(
            matches!(shapes.as_slice(), [ShapeDef { geometry: Geometry::Box { size: [0.4, 0.2, 0.6] }, .. }]),
            "a collision surface without geometry collides with the mass object: {shapes:?}"
        );

        // The start pose's root sits in the character's placement: (0, 1, 0.5) turned
        // 90° about Y is (0.5, 1, 0), plus (3, 2, 0).
        let root_at = |name: &str, key: usize| {
            let clip = project.clips.iter().find(|c| c.props.name == name).unwrap();
            let axis = |p: Property| clip.channels.iter().find(|c| c.joint == "root" && c.property == p).map_or(0.0, |c| c.keys[key].value);
            Vec3::new(axis(Property::PosX), axis(Property::PosY), axis(Property::PosZ))
        };
        assert!(root_at("Start pose", 0).distance(Vec3::new(3.5, 3.0, 0.0)) < 1e-4, "{}", root_at("Start pose", 0));

        // The scene's own keys become a held clip, placed the same way: x = 2 at 1 s is
        // 2 m along the turned crate's -Z.
        assert!(root_at("Scene animation", 0).distance(Vec3::new(3.5, 3.0, 0.0)) < 1e-4);
        assert!(root_at("Scene animation", 1).distance(Vec3::new(3.5, 3.0, -2.0)) < 1e-4, "{}", root_at("Scene animation", 1));
        assert_eq!(project.clips.iter().filter(|c| c.props.name == "Scene animation").count(), 1, "the camera's keys are left out");
        assert!(notes.iter().any(|n| n.contains("keyed in the scene")));
    }

    #[test]
    fn rejects_unreadable_bytes() {
        assert!(matches!(decode_document(&[0x1b, 0xd1, 0xd7, 0xee, 0x3e]), Err(IoError::Format(_))));
    }

    /// Imports every scene, character and pose shipped with a local endorphin install.
    /// Run with `cargo test -- --ignored`; set `ENDORPHIN_DIR` if it isn't installed in
    /// the default location.
    #[test]
    #[ignore = "needs a local endorphin 2.5 install"]
    fn imports_every_bundled_file() {
        let root = std::env::var("ENDORPHIN_DIR")
            .unwrap_or_else(|_| r"C:\Program Files (x86)\NaturalMotion\endorphin 2.5.2".to_owned());
        let mut files = Vec::new();
        let mut stack = vec![std::path::PathBuf::from(root)];
        while let Some(dir) = stack.pop() {
            for entry in fs::read_dir(&dir).into_iter().flatten().flatten() {
                let path = entry.path();
                if path.is_dir() {
                    stack.push(path);
                } else {
                    files.push(path);
                }
            }
        }
        let ext = |p: &Path, e: &str| p.extension().is_some_and(|x| x.eq_ignore_ascii_case(e));
        let (mut ok, mut failed) = (0, Vec::new());
        let standard = files
            .iter()
            .find(|p| p.ends_with("Standard Simulation Character.nmc"))
            .expect("standard character");
        let host = import_character(standard).unwrap().project;
        for path in &files {
            let result = if ext(path, "ens") {
                import_scene(path).map(|i| (i.project.characters.len(), i.project.clips.len()))
            } else if ext(path, "nmc") {
                import_character(path).map(|i| (1, i.project.characters[0].skeleton.joints.len()))
            } else if ext(path, "nma") {
                import_pose(path, &host).map(|p| (1, p.channels.len()))
            } else {
                continue;
            };
            match result {
                Ok(_) => ok += 1,
                Err(e) => failed.push(format!("{}: {e}", path.display())),
            }
        }
        println!("imported {ok} files; {} failed", failed.len());
        for f in &failed {
            println!("  {f}");
        }
        // Expected failures: password-protected (encrypted) tutorial scenes, and poses
        // stored as muscle targets, which arrive with physics.
        assert!(failed.iter().all(|f| f.contains("password") || f.contains("effector")), "{failed:#?}");
        assert!(ok > 150);
    }

    #[test]
    fn euler_unwrap_avoids_jumps() {
        let mut a = [179.0, -170.0, 10.0];
        unwrap_degrees(&mut a, [-179.0, 175.0, 0.0]);
        assert_eq!(a, [-181.0, 190.0, 10.0]);
    }
}
