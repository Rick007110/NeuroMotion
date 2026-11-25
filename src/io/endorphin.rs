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
    BodyDef, ContactModel, Frame, Gains, Geometry, Ground, Interaction, JointDef, Limits,
    MuscleGains, MuscleSet, PhysicsRig, ShapeDef, SpringCap,
};
use crate::document::{
    Channel, Character, CharacterId, CharacterKind, ClipKind, ClipProps, FrameRange, Interp, Joint,
    Keyframe, Pose, Project, ProjectSettings, Property, SimLevel, SkeletonRef, world_transforms,
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
    /// What was left out, in words for the user (empty when everything fitted).
    pub notes: Vec<String>,
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
    let isotropic =
        engine.and_then(|e| e.attribute("enableNonSphericalInertialBodies")) != Some("true");

    for node in elements(child(root, "Characters")) {
        let kind = if node.attribute("typeName") == Some("animatableProp") {
            CharacterKind::Prop
        } else {
            CharacterKind::Character
        };
        add_character(&mut project, node, kind, isotropic, &mut skipped_joints);
    }
    if let Some(env) = child(root, "Environment")
        && child(env, "Skeleton").is_some()
    {
        add_character(
            &mut project,
            env,
            CharacterKind::Environment,
            isotropic,
            &mut skipped_joints,
        );
    }
    // endorphin simulates everything from the first frame unless an event says otherwise,
    // scenery included: its massless pieces stay fixed, while props with mass are real
    // objects (which is why scenes lock or join them to hold them in place).
    for character in project
        .characters
        .iter_mut()
        .filter(|c| c.physics.is_some())
    {
        character.default_level = SimLevel::Simulated;
    }
    if project.characters.is_empty() {
        return Err(IoError::Format("the scene contains no characters".into()));
    }

    // Every timed element in the event sections becomes a clip.
    let mut events: Vec<Node> = [
        "Events",
        "ForceEvents",
        "SecondaryBehaviours",
        "TertiaryBehaviours",
        "SeverBehaviourEvents",
    ]
    .into_iter()
    .flat_map(|section| elements(child(root, section)))
    .filter(|e| e.attribute("startFrameNumber").is_some())
    .collect();
    // Animation first so its tracks sit at the top of each character, then by kind.
    events.sort_by_key(|e| {
        let label = event_label(e.tag_name().name());
        (
            label != "Animation",
            label,
            attr_i32(e, "priority").unwrap_or(0),
            attr_i32(e, "startFrameNumber").unwrap_or(0),
        )
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
        let Some(character) = pose
            .attribute("character")
            .and_then(|n| project.character_by_name(n))
            .cloned()
        else {
            continue;
        };
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
            let track = lanes.track(
                &mut project,
                character.id,
                "Scene animation",
                i32::MIN,
                i32::MIN,
            );
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
                clip.source
                    .insert("endorphin.type".into(), "Animation".into());
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
        let end = attr_i32(&event, "endFrameNumber")
            .unwrap_or(start + 1)
            .max(start + 1);
        first = first.min(start);
        last = last.max(end);

        let (kind, channels) = if tag == "AnimationEvent" {
            let channels = child(event, "Animation").map_or_else(Vec::new, |animation| {
                let offset = attr_i32(&event, "frameOffset").unwrap_or(0);
                animation_channels(
                    animation,
                    None,
                    offset,
                    transform(event),
                    &character.skeleton,
                    fps,
                    &mut skipped_channels,
                )
            });
            (ClipKind::Motion, channels)
        } else {
            // Active poses carry their target pose as held channels.
            let channels = child(event, "Pose")
                .map(|pose| pose_channels(pose, &character.skeleton, None))
                .unwrap_or_default();
            (
                ClipKind::Event {
                    event: label.to_owned(),
                },
                channels,
            )
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

    let playback = engine
        .and_then(|e| child(e, "PlaybackRange"))
        .and_then(|r| {
            Some(FrameRange {
                start: attr_i32(&r, "start")?,
                end: attr_i32(&r, "end")?,
            })
        });
    // endorphin simulates and plays `numTicksToRun` physics ticks, which can run well
    // past the playback range (Car Hit: 400 ticks = 200 frames, range 0-162). Scenes
    // that set no run length carry the default of 10000, which isn't a length.
    let run_end = engine.and_then(|e| {
        let ticks = attr_i32(&e, "numTicksToRun").filter(|t| (1..10_000).contains(t))?;
        let rate = attr_f32(e, "bsimulationFrameRate").filter(|r| *r > 0.0)?;
        Some((ticks as f32 / rate * fps).round() as i32)
    });
    project.settings.range = match playback {
        Some(range) if range.end > range.start => FrameRange {
            start: range.start,
            end: range.end.max(run_end.unwrap_or(0)),
        },
        _ if first < last => FrameRange {
            start: first.min(0),
            end: last,
        },
        _ => FrameRange {
            start: 0,
            end: (fps * 5.0) as i32,
        },
    };
    let range = project.settings.range;
    for clip in project
        .clips
        .iter_mut()
        .filter(|c| c.props.name == "Start pose" && c.props.duration == 1_000_000)
    {
        clip.props.start = range.start;
        clip.props.duration = (range.end - range.start).max(1);
    }

    let people = project
        .characters
        .iter()
        .filter(|c| c.kind == CharacterKind::Character)
        .count();
    notes.push(format!(
        "Imported {} character(s) and {} prop/environment object(s) at {fps:.0} fps.",
        people,
        project.characters.len() - people
    ));
    for (label, n) in &counts {
        notes.push(format!("{n} × {label}"));
    }
    if start_poses > 0 {
        notes.push(format!(
            "{start_poses} starting pose(s) placed as held clips under each character's animation."
        ));
    }
    if scene_animations > 0 {
        notes.push(format!("{scene_animations} character(s) keyed in the scene itself, outside events: a Scene animation clip each."));
    }
    let rigs = project
        .characters
        .iter()
        .filter(|c| c.physics.is_some())
        .count();
    if rigs > 0 {
        notes.push(format!(
            "Physics: {rigs} rig(s) with bodies, joint limits and muscle presets; simulated at {:.0} Hz.",
            project.physics.rate
        ));
    }
    if counts.contains_key("Behaviour") {
        notes.push("Behaviours are kept with their settings; Balance, Stagger, CatchFall, BodyFoetal, BodyWrithe, Look and Reach and Writhe in mid-air (Explosion) run NeuroMotion's own controllers, the others aren't simulated yet.".into());
    }
    if skipped_joints > 0 {
        notes.push(format!(
            "Skipped {skipped_joints} virtual-muscle helper joint(s)."
        ));
    }
    if skipped_channels > 0 {
        notes.push(format!(
            "Skipped {skipped_channels} animation channel(s) for joints the rig doesn't have."
        ));
    }
    if missing_targets > 0 {
        notes.push(format!(
            "Skipped {missing_targets} event(s) aimed at objects that aren't characters."
        ));
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
        let skeleton = SkeletonRef {
            name: file_stem(path),
            source: None,
            joints,
        };
        project.add_character("rig", kind, skeleton)
    } else {
        add_character(&mut project, root, kind, true, &mut skipped)
    };
    if let Some(c) = project.characters.iter_mut().find(|c| c.id == id) {
        c.skeleton.source = Some(path.display().to_string());
    }
    project.add_track(id, "Base");
    let joints = project.characters[0].skeleton.joints.len();
    let what = if is_rig {
        "motion-transfer rig".to_owned()
    } else {
        kind.label().to_lowercase()
    };
    let mut notes = vec![format!("Imported {what} with {joints} joints.")];
    if skipped > 0 {
        notes.push(format!("Skipped {skipped} virtual-muscle helper joint(s)."));
    }
    Ok(Import { project, notes })
}

/// Reads a pose and converts it to channels for the best-matching character.
pub fn import_pose(path: &Path, project: &Project) -> Result<PoseImport, IoError> {
    pose_from_text(&read_document(path)?, file_stem(path), project)
}

/// [`import_pose`] on a pose's XML, named `name`.
pub fn pose_from_text(text: &str, name: String, project: &Project) -> Result<PoseImport, IoError> {
    let xml = parse(text)?;
    let root = xml.root_element();
    if !root.has_tag_name("Pose") {
        return Err(IoError::Format("not an endorphin pose (.nma)".into()));
    }
    if let Some(targets) = child(root, "DesiredPose") {
        let (character, channels, notes) = muscle_target_channels(root, targets, project)?;
        return Ok(PoseImport {
            character,
            name,
            channels,
            notes,
        });
    }
    if root
        .attribute("refFrame")
        .is_some_and(|f| !f.eq_ignore_ascii_case("parent"))
    {
        return Err(IoError::Format(
            "only poses stored relative to parent joints are supported".into(),
        ));
    }
    let entries: Vec<(&str, Quat, Vec3)> = elements(child(root, "Relationships"))
        .filter_map(|r| {
            let (rot, pos) = transform(r)?;
            Some((r.attribute("name")?, rot, pos))
        })
        .collect();

    // Prefer the character the pose names, otherwise whichever shares the most joints.
    let matches = |skeleton: &SkeletonRef| {
        entries
            .iter()
            .filter(|(n, ..)| skeleton.joint_index(n).is_some())
            .count()
    };
    let named = root
        .attribute("character")
        .and_then(|n| project.character_by_name(n));
    let character = named
        .filter(|c| matches(&c.skeleton) > 0)
        .or_else(|| {
            project
                .characters
                .iter()
                .max_by_key(|c| matches(&c.skeleton))
        })
        .filter(|c| matches(&c.skeleton) > 0)
        .ok_or_else(|| {
            IoError::Format("none of the project's characters have this pose's joints".into())
        })?;

    let channels = pose_channels(root, &character.skeleton, None);
    Ok(PoseImport {
        character: character.id,
        name,
        channels,
        notes: Vec::new(),
    })
}

/// Converts a pose's per-joint transforms (`<Relationships>` under `pose`) into held
/// channels relative to the skeleton's rest pose. `placement`, where the character sits
/// in the world, goes under its roots.
fn pose_channels(
    pose: Node,
    skeleton: &SkeletonRef,
    placement: Option<(Quat, Vec3)>,
) -> Vec<Channel> {
    let mut transforms: Vec<(usize, Quat, Vec3)> = elements(child(pose, "Relationships"))
        .filter_map(|r| {
            let (rot, pos) = transform(r)?;
            Some((skeleton.joint_index(r.attribute("name")?)?, rot, pos))
        })
        .collect();
    // Roots sit at `placement`, at rest where the pose doesn't move them.
    if let Some((place_rot, place_pos)) = placement {
        for (index, joint) in skeleton
            .joints
            .iter()
            .enumerate()
            .filter(|(_, j)| j.parent.is_none())
        {
            if !transforms.iter().any(|t| t.0 == index) {
                transforms.push((
                    index,
                    Quat::from_array(joint.rest_rotation).normalize(),
                    Vec3::from(joint.offset),
                ));
            }
        }
        for (_, rot, pos) in transforms
            .iter_mut()
            .filter(|t| skeleton.joints[t.0].parent.is_none())
        {
            (*rot, *pos) = (place_rot * *rot, place_pos + place_rot * *pos);
        }
    }
    let mut channels = Vec::new();
    for (index, rot, pos) in transforms {
        let joint = &skeleton.joints[index];
        let name = joint.name.as_str();
        let euler = relative_euler(joint, rot);
        let offset = pos - Vec3::from(joint.offset);
        for (property, value) in rotation_props()
            .into_iter()
            .zip(euler)
            .chain(position_props().into_iter().zip(offset.to_array()))
        {
            let near_zero = if property.is_rotation() {
                value.abs() < 0.01
            } else {
                value.abs() < 1e-4
            };
            if !near_zero {
                channels.push(Channel {
                    joint: name.to_owned(),
                    property,
                    keys: vec![Keyframe {
                        frame: 0,
                        value,
                        interp: Interp::Linear,
                    }],
                });
            }
        }
    }
    channels
}

// ---------------------------------------------------------------------------------
// Legacy poses: muscle targets

/// One single-axis hinge of a muscle-target pose's own skeleton, at rest, in the
/// pose's world frame.
struct LegacyHinge<'a> {
    name: &'a str,
    position: Vec3,
    /// The hinge's axis: the first column (`Vi`) of its frame.
    axis: Vec3,
    /// The muscle target driving it: effector name and angle (radians).
    target: Option<(&'a str, f32)>,
}

/// Rest directions that differ by more than this (radians) mean the character rests
/// differently from the pose's skeleton there (arms in a T-pose against arms down).
/// Smaller differences are proportions, left alone.
const REST_MISMATCH: f32 = 0.5;

/// Joints of endorphin's legacy skeleton that later characters merged into others,
/// with their names there, best first: the outer shoulder and the upper arm's twist
/// are the shoulder's, the forearm's twist its own joint or else the elbow's, the
/// shin's twist the knee's and the thigh's the hip's.
const LEGACY_JOINTS: [(&str, &[&str]); 5] = [
    ("OutShoulder", &["Shoulder"]),
    ("UpArm", &["Shoulder"]),
    ("LoArm", &["LoArmTwist", "Elbow"]),
    ("LoLeg", &["Knee"]),
    ("UpLeg", &["Hip"]),
];

/// Converts a legacy pose that stores muscle targets (`<DesiredPose>`: one angle per
/// single-axis effector, like `LKneeZ`) into held rotation channels on the project's
/// character they fit best. Returns that character, the channels and notes on what
/// was left out. See docs/endorphin-formats.md, "Muscle-target poses".
///
/// The pose carries its own skeleton, a chain of single-axis hinges per joint (`LHipX`,
/// `LHipZ`, then the thigh's twist), each with its rest frame in the world. An effector
/// drives the hinge into the body named after it (`LKneeZ` → `…_LKneeZMB`), about the
/// hinge frame's first axis, right-handed, by its angle; zero is the rest frame.
/// A character takes a hinge's target on the joint of the same name, else on the joint
/// whose muscle is the effector (`LKneeZ`), else the one whose muscle is the effector's
/// joint (`LKnee`, or its later name from [`LEGACY_JOINTS`]), where one joint collects
/// the rotations of all its axes. Where the character rests differently from the
/// pose's skeleton (a T-pose's arms against the pose's hanging ones), the limb is
/// first turned to the pose's rest, so it ends up where the pose puts it.
fn muscle_target_channels(
    root: Node,
    targets: Node,
    project: &Project,
) -> Result<(CharacterId, Vec<Channel>, Vec<String>), IoError> {
    let mut hinges = Vec::new();
    read_legacy_hinges(child(root, "Skeleton"), None, Quat::IDENTITY, &mut hinges);
    if hinges.is_empty() {
        return Err(IoError::Format(
            "this pose stores muscle targets but not the skeleton they belong to".into(),
        ));
    }
    let effectors: Vec<(&str, f32)> = elements(Some(targets))
        .filter_map(|e| {
            let angle = child(e, "DesiredAngle")?.text()?.trim().parse().ok()?;
            Some((e.attribute("name")?, angle))
        })
        .collect();
    // Each effector drives the hinge into the body named after it. A few have no such
    // body: the hip's third axis turns the thigh (`LHipZMB_LUpLegYMB` for `LHipY`), so an
    // effector left over takes the unclaimed hinge that leaves its joint's bodies into one
    // named for the same axis.
    let mut unplaced = Vec::new();
    for &(name, angle) in &effectors {
        let body = format!("{name}MB");
        match hinges
            .iter()
            .position(|h| h.name.split_once('_').is_some_and(|(_, c)| c == body))
        {
            Some(h) => hinges[h].target = Some((name, angle)),
            None => unplaced.push((name, angle)),
        }
    }
    let mut missing = Vec::new();
    for (name, angle) in unplaced {
        let alias = effector_axis(name).and_then(|(joint, axis)| {
            hinges.iter().position(|h| {
                h.target.is_none()
                    && h.name.split_once('_').is_some_and(|(p, c)| {
                        p.starts_with(joint) && c.ends_with(&format!("{axis}MB"))
                    })
            })
        });
        match alias {
            Some(h) => hinges[h].target = Some((name, angle)),
            None => missing.push(name),
        }
    }

    // The character: the one the pose names if any of it fits, else the best fit.
    let fit = |c: &Character| -> Vec<Option<usize>> {
        hinges
            .iter()
            .map(|h| {
                let (effector, _) = h.target?;
                c.skeleton.joint_index(h.name).or_else(|| {
                    let rig = c.physics.as_ref()?;
                    let by = |muscle: &str| rig.joints.iter().find(|j| j.muscle == muscle);
                    let (joint, _) = effector_axis(effector)?;
                    by(effector)
                        .or_else(|| by(joint))
                        .or_else(|| {
                            let (old, new) =
                                LEGACY_JOINTS.iter().find(|(old, _)| joint.ends_with(old))?;
                            let side = &joint[..joint.len() - old.len()];
                            new.iter().find_map(|n| by(&format!("{side}{n}")))
                        })
                        .map(|j| j.joint)
                })
            })
            .collect()
    };
    let count = |c: &&Character| fit(c).iter().flatten().count();
    let named = root
        .attribute("character")
        .and_then(|n| project.character_by_name(n))
        .filter(|c| count(c) > 0);
    let character = named
        .or_else(|| {
            // The first best fit, so ties go to the earlier character.
            project.characters.iter().fold(None, |best, c| match best {
                Some(b) if count(&b) >= count(&c) => Some(b),
                _ => Some(c),
            })
        })
        .filter(|c| count(c) > 0)
        .ok_or_else(|| {
            IoError::Format(
                "none of the project's characters have this pose's joints or muscles".into(),
            )
        })?;
    let joint_of = fit(character);
    let skeleton = &character.skeleton;
    let rest = world_transforms(skeleton, &Pose::rest(skeleton.joints.len()));

    // Where each target joint is: in the pose, its first hinge; on the character, the joint.
    let mut first = vec![None; skeleton.joints.len()];
    for (h, j) in joint_of.iter().enumerate() {
        if let Some(j) = *j {
            first[j].get_or_insert(h);
        }
    }
    let pose_at = |j: usize| first[j].map(|h| hinges[h].position);

    // Both are upright (Y up), but may face different ways: turn the pose about Y so a
    // left-right pair of its joints lines up with the character's (endorphin's legacy
    // poses face +X with their right side towards +Z).
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let facing = hinges.iter().enumerate().find_map(|(h, hinge)| {
        let (effector, _) = hinge.target?;
        let right = format!("R{}", effector.strip_prefix('L')?);
        let other = hinges
            .iter()
            .position(|o| o.target.is_some_and(|(e, _)| e == right))?;
        let (l, r) = (joint_of[h]?, joint_of[other]?);
        let a = flat(pose_at(r)? - pose_at(l)?);
        let b = flat(rest[r].1 - rest[l].1);
        (a.length() > 1e-3 && b.length() > 1e-3)
            .then(|| Quat::from_rotation_y(libm::atan2f(a.cross(b).y, a.dot(b))))
    });
    let mut notes = Vec::new();
    let turn = facing.unwrap_or_else(|| {
        notes.push("No left and right joint pair to tell which way the character faces; the pose's own axes were used.".into());
        Quat::IDENTITY
    });

    // Each joint's rotation in the world at rest: its hinges' turns, parent first.
    let mut turns: Vec<Option<Quat>> = vec![None; skeleton.joints.len()];
    for (h, hinge) in hinges.iter().enumerate() {
        if let (Some(j), Some((_, angle))) = (joint_of[h], hinge.target) {
            let q = turns[j].get_or_insert(Quat::IDENTITY);
            *q *= Quat::from_axis_angle((turn * hinge.axis).normalize(), angle);
        }
    }

    // Every joint's turn in the world, accumulated down the skeleton (parents come
    // first): the pose's (`posed`), and the one that first brings the character's rest
    // to the pose's (`rest_fix`). A mapped joint is fixed when the bone to its mapped
    // children (towards their middle, for several) points elsewhere than the pose's,
    // by the shortest turn, which its children inherit.
    let n = skeleton.joints.len();
    let mut children: Vec<Vec<usize>> = vec![Vec::new(); n];
    for j in (0..n).filter(|&j| first[j].is_some()) {
        let mut up = skeleton.joints[j].parent;
        while let Some(a) = up {
            if first[a].is_some() {
                children[a].push(j);
                break;
            }
            up = skeleton.joints[a].parent;
        }
    }
    let mut posed = vec![Quat::IDENTITY; n];
    let mut rest_fix = vec![Quat::IDENTITY; n];
    let mut fixed = Vec::new();
    let mut channels = Vec::new();
    for j in 0..n {
        let (parent_posed, parent_fix) = skeleton.joints[j]
            .parent
            .map_or((Quat::IDENTITY, Quat::IDENTITY), |p| {
                (posed[p], rest_fix[p])
            });
        posed[j] = parent_posed * turns[j].unwrap_or(Quat::IDENTITY);
        let mut fix = parent_fix;
        if let (Some(here), false) = (pose_at(j), children[j].is_empty()) {
            let middle = |at: &dyn Fn(usize) -> Vec3| {
                children[j].iter().map(|&c| at(c)).sum::<Vec3>() / children[j].len() as f32
            };
            let ours = parent_fix * (middle(&|c| rest[c].1) - rest[j].1);
            let theirs = turn * (middle(&|c| pose_at(c).unwrap_or(here)) - here);
            if ours.length() > 0.02
                && theirs.length() > 0.02
                && ours.angle_between(theirs) > REST_MISMATCH
            {
                fix = Quat::from_rotation_arc(ours.normalize(), theirs.normalize()) * parent_fix;
                fixed.push(skeleton.joints[j].name.as_str());
            }
        }
        rest_fix[j] = fix;
        // The joint's own turn in the world at rest: from its parent's to its own.
        let q = (parent_posed * parent_fix).inverse() * posed[j] * fix;
        if q.angle_between(Quat::IDENTITY) < 1e-4 {
            continue;
        }
        let name = skeleton.joints[j].name.as_str();
        // From the world at rest into the joint's own frame, as channels are.
        let world = rest[j].0;
        let local = (world.inverse() * q * world).normalize();
        let (x, y, z) = local.to_euler(EulerRot::XYZ);
        for (property, value) in
            rotation_props()
                .into_iter()
                .zip([x.to_degrees(), y.to_degrees(), z.to_degrees()])
        {
            if value.abs() >= 0.01 {
                channels.push(Channel {
                    joint: name.to_owned(),
                    property,
                    keys: vec![Keyframe {
                        frame: 0,
                        value,
                        interp: Interp::Linear,
                    }],
                });
            }
        }
    }
    let unfitted: Vec<&str> = hinges
        .iter()
        .zip(&joint_of)
        .filter(|(_, j)| j.is_none())
        .filter_map(|(h, _)| Some(h.target?.0))
        .chain(missing)
        .collect();
    if !unfitted.is_empty() {
        notes.push(format!(
            "{} muscle target(s) have no joint on {}: {}.",
            unfitted.len(),
            character.name,
            unfitted.join(", ")
        ));
    }
    if !fixed.is_empty() {
        notes.push(format!(
            "{} rest differently on {} than in the pose (a T-pose's arms, say), so they were first turned to the pose's rest: {}.",
            if fixed.len() == 1 { "One joint" } else { "Some joints" },
            character.name,
            fixed.join(", ")
        ));
    }
    Ok((character.id, channels, notes))
}

/// Splits a single-axis effector name into its joint and axis: `LKneeZ` → (`LKnee`, 'Z').
fn effector_axis(name: &str) -> Option<(&str, char)> {
    let axis = name
        .chars()
        .last()
        .filter(|c| matches!(c, 'X' | 'Y' | 'Z'))?;
    Some((&name[..name.len() - 1], axis))
}

/// A muscle-target pose's skeleton: its hinges, parents first. Their frames are stored
/// in the world (`refFrame="World"`); a frame relative to its parent is placed under it.
fn read_legacy_hinges<'a>(
    node: Option<Node<'a, '_>>,
    parent: Option<usize>,
    parent_rotation: Quat,
    hinges: &mut Vec<LegacyHinge<'a>>,
) {
    for h in elements(node.and_then(|n| child(n, "ChildJoints"))) {
        let Some(m) = child(h, "TransformMatrix") else {
            continue;
        };
        let column = |tag: &str| {
            let v = child(m, tag)?;
            let c = |a: &str| {
                attr_f32(v, &format!("b{a}")).or_else(|| v.attribute(a)?.trim().parse().ok())
            };
            Some(Vec3::new(c("x")?, c("y")?, c("z")?))
        };
        let (Some(i), Some(j), Some(k), Some(p)) =
            (column("Vi"), column("Vj"), column("Vk"), column("Vp"))
        else {
            continue;
        };
        let world = m
            .attribute("refFrame")
            .is_some_and(|f| f.eq_ignore_ascii_case("world"));
        let rotation = Quat::from_mat3(&Mat3::from_cols(i, j, k)).normalize();
        let (rotation, position) = match parent {
            Some(p_index) if !world => (
                parent_rotation * rotation,
                hinges[p_index].position + parent_rotation * p,
            ),
            _ => (rotation, p),
        };
        hinges.push(LegacyHinge {
            name: h.attribute("name").unwrap_or(""),
            position,
            axis: rotation * Vec3::X,
            target: None,
        });
        let index = hinges.len() - 1;
        read_legacy_hinges(Some(h), Some(index), rotation, hinges);
    }
}

// ---------------------------------------------------------------------------------
// Characters

fn add_character(
    project: &mut Project,
    node: Node,
    kind: CharacterKind,
    isotropic: bool,
    skipped: &mut usize,
) -> CharacterId {
    let mut joints = Vec::new();
    let mut nodes = Vec::new();
    if let Some(root) = child(node, "Skeleton") {
        add_joint(root, None, &mut joints, &mut nodes, skipped);
    }
    let source = node
        .attribute("pathName")
        .filter(|p| !p.is_empty())
        .map(str::to_owned);
    let rig = source
        .as_deref()
        .map(|p| file_stem(Path::new(&p.replace('\\', "/"))))
        .unwrap_or_else(|| kind.label().to_owned());
    let name = node.attribute("name").unwrap_or("character");
    let skeleton = SkeletonRef {
        name: rig,
        source,
        joints,
    };
    let physics = read_rig(
        node,
        &nodes,
        &skeleton,
        isotropic,
        kind == CharacterKind::Environment,
    );
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
        let Some(full) = channel.attribute("name") else {
            continue;
        };
        let Some(full) = owner.map_or(Some(full), |o| {
            full.strip_prefix(o).and_then(|rest| rest.strip_prefix('.'))
        }) else {
            continue;
        };
        let mut parts = full.rsplit('.');
        let (Some(property), Some(joint)) = (parts.next(), parts.next()) else {
            continue;
        };
        let Some(index) = skeleton.joint_index(joint) else {
            *skipped += 1;
            continue;
        };
        let track = tracks.entry(index).or_default();
        let keys = elements(Some(channel));
        match (channel.tag_name().name(), property) {
            ("QuatChannel", _) => {
                track.rotation = keys
                    .filter_map(|k| {
                        let q = Quat::from_xyzw(
                            attr_f32(k, "by")?,
                            attr_f32(k, "bz")?,
                            attr_f32(k, "bw")?,
                            attr_f32(k, "bx")?,
                        );
                        Some((attr_f32(k, "bt")?, q.normalize()))
                    })
                    .collect()
            }
            ("Channel", "tx" | "ty" | "tz") => {
                let axis = (property.as_bytes()[1] - b'x') as usize;
                track.position[axis] = keys
                    .filter_map(|k| Some((attr_f32(k, "bx")?, attr_f32(k, "by")?)))
                    .collect();
            }
            _ => *skipped += 1,
        }
    }

    // The placement puts the whole animation in the scene: bake it into the root. The
    // scene's own keys leave a root they don't key to the start pose.
    if let Some((place_rot, place_pos)) = placement {
        let is_identity =
            place_rot.angle_between(Quat::IDENTITY) < 1e-4 && place_pos.length() < 1e-5;
        let root = skeleton.joints.iter().position(|j| j.parent.is_none());
        if let Some(root) =
            root.filter(|r| !is_identity && (owner.is_none() || tracks.contains_key(r)))
        {
            let joint = &skeleton.joints[root];
            let track = tracks.entry(root).or_default();
            if track.rotation.is_empty() {
                track
                    .rotation
                    .push((0.0, Quat::from_array(joint.rest_rotation)));
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
                    let p = Vec3::new(
                        sample_linear(&track.position[0], t),
                        sample_linear(&track.position[1], t),
                        sample_linear(&track.position[2], t),
                    );
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
    let key = Keyframe {
        frame,
        value,
        interp: Interp::Linear,
    };
    match keys.last_mut() {
        Some(last) if last.frame == frame => *last = key,
        _ => keys.push(key),
    }
}

/// Adds the channel unless every key is (near) zero, i.e. it never leaves the rest pose.
fn push_channel(
    channels: &mut Vec<Channel>,
    joint: &str,
    property: Property,
    mut keys: Vec<Keyframe>,
    epsilon: f32,
) {
    keys.sort_by_key(|k| k.frame);
    keys.dedup_by_key(|k| k.frame);
    if keys.iter().all(|k| k.value.abs() < epsilon) {
        return;
    }
    channels.push(Channel {
        joint: joint.to_owned(),
        property,
        keys,
    });
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
    fn track(
        &mut self,
        project: &mut Project,
        character: CharacterId,
        label: &'static str,
        start: i32,
        end: i32,
    ) -> crate::document::TrackId {
        if let Some(lane) = self
            .lanes
            .iter_mut()
            .find(|l| l.0 == character && l.1 == label && l.3 <= start)
        {
            lane.3 = end;
            return lane.2;
        }
        let existing = self
            .lanes
            .iter()
            .filter(|l| l.0 == character && l.1 == label)
            .count();
        let plural = if matches!(label, "Animation" | "Start pose" | "Scene animation") {
            label.to_owned()
        } else {
            format!("{label}s")
        };
        let name = if existing == 0 {
            plural
        } else {
            format!("{plural} {}", existing + 1)
        };
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
        "name",
        "displayName",
        "startFrameNumber",
        "endFrameNumber",
        "character",
        "enabled",
        "visible",
        "retailVersionVisible",
        "addToFrameBuffer",
        "animationVisible",
    ];
    let mut out = BTreeMap::new();
    out.insert(
        "endorphin.type".to_owned(),
        event.tag_name().name().to_owned(),
    );
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
                out.insert(
                    format!("matrix.{axis}"),
                    format!(
                        "{}, {}, {}",
                        format_float(v.x),
                        format_float(v.y),
                        format_float(v.z)
                    ),
                );
            }
        }
    }
    // Vector settings stored as child elements (e.g. constraint strengths).
    for c in elements(Some(event)) {
        if let (Some(v), true) = (vec3(Some(c)), c.tag_name().name() != "TransformMatrix") {
            let tag = c.tag_name().name();
            let key = tag[..1].to_ascii_lowercase() + &tag[1..];
            out.insert(
                key,
                format!(
                    "{}, {}, {}",
                    format_float(v.x),
                    format_float(v.y),
                    format_float(v.z)
                ),
            );
        }
    }
    let targets: Vec<&str> = elements(child(event, "Objects"))
        .filter_map(|o| o.text())
        .collect();
    if !targets.is_empty() {
        out.insert("targets".to_owned(), targets.join(", "));
    }
    behaviour_parameters(event, &mut out);
    out
}

/// A behaviour event's own settings (`Parameters`): one `Variable` per setting, named
/// by `fk`, its value in `ja`. A setting without `ja` takes the behaviour's default,
/// which the scene doesn't say, so it is left out. A selection of bodies or collision
/// surfaces (`ja="MetaBodyAndCollisionSelection"`, a reach or look-at target) lists them
/// as `ValuePtr`s, kept comma separated (and left out when empty). Helper graphics (the
/// locators endorphin draws) aren't settings. The event's own attributes win if a name
/// is taken.
fn behaviour_parameters(event: Node, out: &mut BTreeMap<String, String>) {
    for var in elements(child(event, "Parameters")).filter(|v| v.has_tag_name("Variable")) {
        let (Some(key), Some(value)) = (var.attribute("fk"), var.attribute("ja")) else {
            continue;
        };
        match value {
            "HelperGraphic" => {}
            "MetaBodyAndCollisionSelection" => {
                let picked: Vec<&str> = var
                    .children()
                    .filter(|c| c.has_tag_name("ValuePtr"))
                    .filter_map(|c| c.attribute("ja"))
                    .collect();
                if !picked.is_empty() {
                    out.entry(key.to_owned())
                        .or_insert_with(|| picked.join(", "));
                }
            }
            _ => {
                out.entry(key.to_owned())
                    .or_insert_with(|| value.to_owned());
            }
        }
    }
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
    // endorphin's friction values are Coulomb coefficients × 100 (lesson 21).
    settings.contact = ContactModel::Coulomb;
    // endorphin's muscles move whole limbs (lesson 27).
    settings.spring_cap = SpringCap::Limbs;
    settings.ground = (scene.attribute("useDefaultGround") != Some("false")).then(|| Ground {
        height: 0.0,
        material: "tarmac".into(),
    });
    let Some(options) = child(engine, "MaterialOptions") else {
        return;
    };
    let interactions: BTreeMap<String, Interaction> =
        elements(child(options, "MaterialInteractions"))
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
fn read_rig(
    node: Node,
    joint_nodes: &[Node],
    skeleton: &SkeletonRef,
    isotropic: bool,
    scenery: bool,
) -> Option<PhysicsRig> {
    let body_of: Vec<Option<&str>> = joint_nodes
        .iter()
        .map(|n| n.attribute("metaBody"))
        .collect();
    let mut bodies = Vec::new();
    for mb in elements(child(node, "MetaBodies")).filter(|n| n.has_tag_name("MetaBody")) {
        let Some(name) = mb.attribute("name") else {
            continue;
        };
        // A body belongs to the first (outermost) joint that names it; scenery pieces that
        // no joint names hang from the root.
        let Some(joint) = body_of
            .iter()
            .position(|b| *b == Some(name))
            .or((!joint_nodes.is_empty()).then_some(0))
        else {
            continue;
        };
        let (rotation, position) = transform(mb).unwrap_or((Quat::IDENTITY, Vec3::ZERO));
        let (mass, inertia) = child(mb, "InertialBody")
            .and_then(|ib| {
                let geometry = geometry(child(ib, "Geometry")?)?;
                let density = child(ib, "Inertia")
                    .and_then(|i| number(i, "density"))
                    .unwrap_or(1.0);
                // endorphin densities are in g/cm^3.
                let mass = density * 1000.0 * geometry.volume();
                Some((mass, geometry.inertia(mass)))
            })
            // No mass (no inertial body, or an empty one): scenery stays fixed in place, like
            // the ground. A character's helper body (Back-Flip-To-Combo's zero-sized
            // NormalMass) gets a token mass instead, or it would hold the character in the air.
            .filter(|&(mass, _)| scenery || mass > 0.0)
            .unwrap_or(if scenery {
                (0.0, [0.0; 3])
            } else {
                (1.0, [0.004; 3])
            });
        let shapes = elements(Some(mb))
            .filter(|n| n.has_tag_name("CollisionSurface"))
            .filter_map(|cs| {
                // A collision surface without geometry of its own collides with the body's
                // mass shape: how endorphin gives props ("environment mass objects have a
                // collision object by default"), ropes and other rigs made in it collision.
                let shape = child(cs, "Geometry")
                    .or_else(|| child(mb, "InertialBody").and_then(|ib| child(ib, "Geometry")));
                let placed = if child(cs, "Geometry").is_some() {
                    cs
                } else {
                    child(mb, "InertialBody")?
                };
                let (r, p) = transform(placed).unwrap_or((Quat::IDENTITY, Vec3::ZERO));
                let flag = |name: &str| cs.attribute(name) != Some("false");
                Some(ShapeDef {
                    geometry: geometry(shape?)?,
                    frame: Frame {
                        rotation: r.to_array(),
                        position: p.to_array(),
                    },
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
            frame: Frame {
                rotation: rotation.to_array(),
                position: position.to_array(),
            },
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
        let Some(body) = body_of[j].and_then(body_index) else {
            continue;
        };
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
        let Some(parent_body) = parent_body else {
            continue;
        };
        let limit_frame = |tag: &str| {
            child(*jn, tag)
                .and_then(|f| basis(child(f, "TransformMatrix")?))
                .unwrap_or(Quat::IDENTITY)
                .to_array()
        };
        let (hard, soft, stiffness, damping) = if tag == "Hinge" {
            hinge_limits(*jn)
        } else {
            // endorphin's swing1 bounds the turn about the limit frame's Z axis (the
            // child's axis swinging towards Y) and swing2 the turn about Y: read the other
            // way round, the active poses endorphin's own scenes store sit up to five
            // times outside their joints' cones (toes, hands, clavicles, feet, hips),
            // and this way within them (physics-design.md, lesson 27).
            let limits = |tag: &str| {
                child(*jn, tag).map(|l| {
                    Limits::from_sizes(
                        number(l, "twistSize").unwrap_or(0.0),
                        number(l, "swing2Size").unwrap_or(0.0),
                        number(l, "swing1Size").unwrap_or(0.0),
                    )
                })
            };
            // 6.28 is the fallback twist size as written, not meant to be exactly TAU.
            #[allow(clippy::approx_constant)]
            let hard = limits("HardLimits").unwrap_or(Limits::from_sizes(6.28, 6.0, 6.0));
            let soft_node = child(*jn, "SoftLimits");
            let avg = |a: &str, b: &str| {
                soft_node.map_or(0.0, |s| {
                    (number(s, a).unwrap_or(0.0) + number(s, b).unwrap_or(0.0)) * 0.5
                })
            };
            (
                hard,
                limits("SoftLimits").unwrap_or(hard),
                avg("twistStiffness", "swingStiffness"),
                avg("twistDamping", "swingDamping"),
            )
        };
        let name = skeleton.joints[j].name.clone();
        joints.push(JointDef {
            muscle: effector_of
                .get(name.as_str())
                .map_or_else(|| name.clone(), |e| (*e).to_owned()),
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
                    Some((
                        p.attribute("name")?.to_owned(),
                        MuscleGains {
                            twist: gains("twist"),
                            swing: gains("swing"),
                        },
                    ))
                });
            // 1-DOF effectors (finger hinges): gradient acts as stiffness, capped by the
            // base force cap. Damping isn't stored; a tenth of the gradient keeps them calm.
            let single = elements(Some(cal))
                .filter(|p| p.has_tag_name("EffectorParameters"))
                .filter_map(|p| {
                    let value = |tag: &str| child(p, tag)?.text()?.trim().parse::<f32>().ok();
                    let gradient = value("Gradient").unwrap_or(1.0);
                    let g = Gains {
                        stiffness: gradient,
                        damping: gradient * 0.1,
                        force_limit: value("BaseForceCap").unwrap_or(2.0),
                    };
                    Some((
                        p.attribute("name")?.to_owned(),
                        MuscleGains { twist: g, swing: g },
                    ))
                });
            let muscles: BTreeMap<String, MuscleGains> = skeletal.chain(single).collect();
            let fallback = *muscles.values().next()?;
            Some(MuscleSet {
                name: cal.attribute("name")?.to_owned(),
                muscles,
                fallback,
            })
        })
        .collect();

    Some(PhysicsRig {
        bodies,
        joints,
        muscle_sets,
        isotropic_inertia: isotropic,
    })
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
    let (lower, upper) = (
        bound("Lower", "Value").unwrap_or(-3.1),
        bound("Upper", "Value").unwrap_or(3.1),
    );
    let limits = Limits {
        twist_min: lower,
        twist_max: upper,
        swing1: 0.01,
        swing2: 0.01,
    };
    (
        limits,
        limits,
        bound("Lower", "Stiffness").unwrap_or(0.0),
        bound("Lower", "Damping").unwrap_or(0.0),
    )
}

fn geometry(g: Node) -> Option<Geometry> {
    let p = |i: usize| number(g, &format!("param{i}"));
    Some(match g.attribute("type")? {
        "Sphere" => Geometry::Sphere { radius: p(0)? },
        "Sphyl" => Geometry::Capsule {
            radius: p(0)?,
            length: p(1)?,
        },
        "Cylinder" => Geometry::Cylinder {
            radius: p(0)?,
            length: p(1)?,
        },
        "Cuboid" => Geometry::Box {
            size: [p(0)?, p(1)?, p(2)?],
        },
        _ => return None,
    })
}

/// A number stored either as a `b`-prefixed float bit pattern or as plain text.
fn number(node: Node, name: &str) -> Option<f32> {
    attr_f32(node, &format!("b{name}")).or_else(|| node.attribute(name)?.trim().parse().ok())
}

/// A `TransformMatrix`'s rotation only.
fn basis(m: Node) -> Option<Quat> {
    let basis = Mat3::from_cols(
        vec3(child(m, "Vi"))?,
        vec3(child(m, "Vj"))?,
        vec3(child(m, "Vk"))?,
    );
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
    if ZlibDecoder::new(bytes).read_to_string(&mut text).is_ok()
        && text.trim_start().starts_with('<')
    {
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
    roxmltree::Document::parse(text)
        .map_err(|e| IoError::Format(format!("invalid endorphin XML: {e}")))
}

fn child<'a, 'i>(node: Node<'a, 'i>, tag: &str) -> Option<Node<'a, 'i>> {
    node.children().find(|c| c.has_tag_name(tag))
}

fn elements<'a, 'i>(node: Option<Node<'a, 'i>>) -> impl Iterator<Item = Node<'a, 'i>> {
    node.into_iter()
        .flat_map(|n| n.children())
        .filter(Node::is_element)
}

fn hex_f32(value: &str) -> Option<f32> {
    (value.len() == 8)
        .then(|| u32::from_str_radix(value, 16).ok().map(f32::from_bits))
        .flatten()
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
    Some(Vec3::new(
        attr_f32(n, "bx")?,
        attr_f32(n, "by")?,
        attr_f32(n, "bz")?,
    ))
}

/// A node's `TransformMatrix` as rotation + position (relative to its parent).
fn transform(node: Node) -> Option<(Quat, Vec3)> {
    let m = child(node, "TransformMatrix")?;
    let basis = Mat3::from_cols(
        vec3(child(m, "Vi"))?,
        vec3(child(m, "Vj"))?,
        vec3(child(m, "Vk"))?,
    );
    Some((Quat::from_mat3(&basis).normalize(), vec3(child(m, "Vp"))?))
}

fn format_float(v: f32) -> String {
    if v.is_finite() {
        format!("{}", (v * 1e5).round() / 1e5)
    } else {
        v.to_string()
    }
}

fn file_stem(path: &Path) -> String {
    path.file_stem().map_or_else(
        || "Imported".to_owned(),
        |s| s.to_string_lossy().into_owned(),
    )
}

fn path_is_prop(path: &Path) -> bool {
    path.components()
        .any(|c| c.as_os_str().eq_ignore_ascii_case("Props"))
}

/// A hand-written muscle-target pose in the shape of endorphin's legacy ones (not one
/// of its files), for the importer's tests and the physics stand-in.
#[cfg(test)]
pub mod legacy_fixture {
    use glam::Vec3;

    use crate::document::physics::PhysicsRig;

    /// Hip flexion, knee bend, lower-spine bend, shoulder raise and elbow bend (radians).
    pub const HIP: f32 = 0.5;
    pub const KNEE: f32 = -1.0;
    pub const SPINE: f32 = 0.3;
    pub const SHOULDER: f32 = 0.6;
    pub const ELBOW: f32 = 0.8;

    /// The skeleton's hinges as the legacy poses store them: (name, axis, world
    /// position), nested by indent level. Like them it faces +X, Y up, its right side
    /// towards +Z, and each joint is a chain of single-axis hinges (the hip's third axis
    /// turns the thigh, `LHipZMB_LUpLegYMB`, which `LHipY` drives).
    const HINGES: [(usize, &str, [f32; 3], [f32; 3]); 13] = [
        (0, "PelvisMB_LoSpineZMB", [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]),
        (
            1,
            "LoSpineZMB_LShoulderZMB",
            [0.0, 0.0, 1.0],
            [0.0, 1.4, -0.19],
        ),
        (
            2,
            "LShoulderZMB_LElbowZMB",
            [0.0, 0.0, 1.0],
            [0.0, 1.12, -0.19],
        ),
        (
            1,
            "LoSpineZMB_RShoulderZMB",
            [0.0, 0.0, 1.0],
            [0.0, 1.4, 0.19],
        ),
        (
            2,
            "RShoulderZMB_RElbowZMB",
            [0.0, 0.0, 1.0],
            [0.0, 1.12, 0.19],
        ),
        (0, "PelvisMB_LHipZMB", [0.0, 0.0, 1.0], [0.0, 0.9, -0.1]),
        (1, "LHipZMB_LUpLegYMB", [0.0, 1.0, 0.0], [0.0, 0.9, -0.1]),
        (2, "LUpLegYMB_LKneeZMB", [0.0, 0.0, 1.0], [0.0, 0.48, -0.1]),
        (0, "PelvisMB_RHipZMB", [0.0, 0.0, 1.0], [0.0, 0.9, 0.1]),
        (1, "RHipZMB_RUpLegYMB", [0.0, 1.0, 0.0], [0.0, 0.9, 0.1]),
        (2, "RUpLegYMB_RKneeZMB", [0.0, 0.0, 1.0], [0.0, 0.48, 0.1]),
        // Not a joint the other characters have: a hinge with no target.
        (0, "PelvisMB_TailXMB", [1.0, 0.0, 0.0], [0.0, 0.95, 0.0]),
        (1, "TailXMB_TailTipXMB", [1.0, 0.0, 0.0], [-0.2, 0.95, 0.0]),
    ];

    /// The hinges with their world positions and parents, for building a character
    /// with the pose's own skeleton.
    pub fn hinges() -> Vec<(&'static str, Option<usize>, Vec3)> {
        let mut out = Vec::new();
        let mut stack: Vec<usize> = Vec::new();
        for (i, &(level, name, _, position)) in HINGES.iter().enumerate() {
            stack.truncate(level);
            out.push((name, stack.last().copied(), Vec3::from(position)));
            stack.push(i);
        }
        out
    }

    /// The pose, turning each thigh by `twist` (radians) about its own length.
    /// `BaseNeckZ` names a hinge the skeleton doesn't have, as endorphin's own does.
    pub fn xml(twist: f32) -> String {
        let targets = [
            ("LoSpineZ", SPINE),
            ("LShoulderZ", SHOULDER),
            ("LElbowZ", ELBOW),
            ("RShoulderZ", SHOULDER),
            ("RElbowZ", ELBOW),
            ("LHipZ", HIP),
            ("LHipY", twist),
            ("LKneeZ", KNEE),
            ("RHipZ", HIP),
            ("RHipY", -twist),
            ("RKneeZ", KNEE),
            ("BaseNeckZ", 0.0),
        ];
        let mut xml = String::from(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<Pose name=\"\" character=\"legacy\">\n  <DesiredPose name=\"DefaultPose\">\n",
        );
        for (name, angle) in targets {
            xml += &format!(
                "    <EffectorParameters name=\"{name}\">\n      <DesiredAngle>{angle}</DesiredAngle>\n    </EffectorParameters>\n"
            );
        }
        // Reach targets, which the importer doesn't use.
        xml += "  </DesiredPose>\n  <Relationships>\n    <RPROMagnet name=\"LHandRPRO\">\n      <TransformMatrix refFrame=\"World\">\n        <Vi x=\"1.0\" y=\"0.0\" z=\"0.0\"/>\n        <Vj x=\"0.0\" y=\"1.0\" z=\"0.0\"/>\n        <Vk x=\"0.0\" y=\"0.0\" z=\"1.0\"/>\n        <Vp x=\"0.3\" y=\"1.0\" z=\"-0.2\"/>\n      </TransformMatrix>\n    </RPROMagnet>\n  </Relationships>\n  <Skeleton name=\"\">\n    <TransformMatrix refFrame=\"World\">\n      <Vi x=\"1.0\" y=\"0.0\" z=\"0.0\"/>\n      <Vj x=\"0.0\" y=\"1.0\" z=\"0.0\"/>\n      <Vk x=\"0.0\" y=\"0.0\" z=\"1.0\"/>\n      <Vp x=\"0.0\" y=\"0.95\" z=\"0.0\"/>\n    </TransformMatrix>\n    <ChildJoints>\n";
        let vector =
            |tag: &str, v: Vec3| format!("<{tag} x=\"{}\" y=\"{}\" z=\"{}\"/>", v.x, v.y, v.z);
        let mut open = 0;
        for &(level, name, axis, position) in &HINGES {
            while open > level {
                open -= 1;
                xml += "</ChildJoints></Hinge>\n";
            }
            // The hinge's frame: its axis first, the other two any right-handed pair.
            let i = Vec3::from(axis);
            let helper = if i.y.abs() < 0.9 { Vec3::Y } else { Vec3::Z };
            let j = (helper - i * i.dot(helper)).normalize();
            let k = i.cross(j);
            xml += &format!(
                "<Hinge name=\"{name}\"><TransformMatrix refFrame=\"World\">{}{}{}{}</TransformMatrix><ChildJoints>\n",
                vector("Vi", i),
                vector("Vj", j),
                vector("Vk", k),
                vector("Vp", Vec3::from(position)),
            );
            open += 1;
        }
        while open > 0 {
            open -= 1;
            xml += "</ChildJoints></Hinge>\n";
        }
        xml + "    </ChildJoints>\n  </Skeleton>\n</Pose>\n"
    }

    /// Gives the placeholder rig's muscles endorphin's effector names (`l_knee` becomes
    /// `LKnee`), as an imported character has them, keeping their gains.
    pub fn effector_names(rig: &mut PhysicsRig) {
        let rename = |muscle: &str| -> Option<String> {
            let (side, joint) = match muscle.split_once('_') {
                Some(("l", j)) => ("L", j),
                Some(("r", j)) => ("R", j),
                _ => ("", muscle),
            };
            let joint = match joint {
                "spine" => "LoSpine",
                "chest" => "UpSpine",
                "neck" => "Neck",
                "shoulder" => "Shoulder",
                "elbow" => "Elbow",
                "wrist" => "Hand",
                "hip" => "Hip",
                "knee" => "Knee",
                "ankle" => "Ankle",
                _ => return None,
            };
            Some(format!("{side}{joint}"))
        };
        for joint in &mut rig.joints {
            if let Some(name) = rename(&joint.muscle) {
                joint.muscle = name;
            }
        }
        for set in &mut rig.muscle_sets {
            set.muscles = std::mem::take(&mut set.muscles)
                .into_iter()
                .map(|(m, g)| (rename(&m).unwrap_or(m), g))
                .collect();
        }
    }
}

/// Joint-level checks against a hand-written scene (no endorphin install needed).
#[cfg(test)]
mod tests {
    use super::*;

    fn hex(v: f32) -> String {
        format!("{:08x}", v.to_bits())
    }

    fn matrix(cols: [[f32; 3]; 3], p: [f32; 3]) -> String {
        let v = |tag: &str, x: [f32; 3]| {
            format!(
                r#"<{tag} bx="{}" by="{}" bz="{}"/>"#,
                hex(x[0]),
                hex(x[1]),
                hex(x[2])
            )
        };
        format!(
            "<TransformMatrix>{}{}{}{}</TransformMatrix>",
            v("Vi", cols[0]),
            v("Vj", cols[1]),
            v("Vk", cols[2]),
            v("Vp", p)
        )
    }

    /// A two-joint character whose child bone points along the parent's local +X, with
    /// the root rotated so local +X is world up (like endorphin's spine).
    fn scene_xml() -> String {
        let up = matrix(
            [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0, 1.0, 0.0],
        );
        let ident = matrix(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.5, 0.0, 0.0],
        );
        let place = matrix(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [2.0, 0.0, 0.0],
        );
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
        let mut encoder =
            flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
        std::io::Write::write_all(&mut encoder, xml.as_bytes()).unwrap();
        fs::write(&path, encoder.finish().unwrap()).unwrap();
        import_scene(&path).unwrap()
    }

    /// endorphin plays the whole run, not only the playback range (Car Hit: 400 ticks at
    /// 120 Hz and 60 fps is 200 frames, playback range 0-162); 10000 ticks is the
    /// default of a scene that sets no length.
    #[test]
    fn range_follows_the_run_length() {
        let with_ticks = |ticks: u32, fps: f32| {
            let engine = format!(
                r#"<Engine bpositionBufferFrameRate="{}" bsimulationFrameRate="{}" numTicksToRun="{ticks}">"#,
                hex(fps),
                hex(120.0)
            );
            let xml = scene_xml().replace(
                &format!(r#"<Engine bpositionBufferFrameRate="{}">"#, hex(30.0)),
                &engine,
            );
            assert_ne!(xml, scene_xml());
            import(&xml).project.settings.range
        };
        assert_eq!(with_ticks(400, 60.0), FrameRange { start: 0, end: 200 });
        // 120 ticks at 30 fps is 30 frames: the playback range already covers it.
        assert_eq!(with_ticks(60, 30.0), FrameRange { start: 0, end: 30 });
        assert_eq!(with_ticks(10_000, 60.0), FrameRange { start: 0, end: 30 });
    }

    #[test]
    fn scene_skeleton_and_settings() {
        let Import { project, notes } = import(&scene_xml());
        assert_eq!(project.settings.fps, 30.0);
        assert_eq!(project.settings.range, FrameRange { start: 0, end: 30 });
        let hero = &project.characters[0];
        let names: Vec<_> = hero
            .skeleton
            .joints
            .iter()
            .map(|j| j.name.as_str())
            .collect();
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
        assert_eq!(
            clip.source.get("strength").map(String::as_str),
            Some("0.25")
        );

        let rz = clip
            .channels
            .iter()
            .find(|c| c.joint == "Spine" && c.property == Property::RotZ)
            .unwrap();
        assert_eq!(rz.keys.len(), 2);
        assert_eq!(rz.keys[1].frame, 30, "1 s at 30 fps");
        assert!((rz.keys[1].value - 90.0).abs() < 1e-3);

        // The event's placement matrix lands on the root as a world offset.
        let px = clip
            .channels
            .iter()
            .find(|c| c.joint == "root" && c.property == Property::PosX)
            .unwrap();
        assert!((px.keys[0].value - 2.0).abs() < 1e-5);
    }

    #[test]
    fn physics_events_keep_their_settings() {
        let Import { project, .. } = import(&scene_xml());
        let force = project.clips.iter().find(|c| !c.kind.is_motion()).unwrap();
        assert_eq!(
            force.kind,
            ClipKind::Event {
                event: "Force".into()
            }
        );
        assert_eq!(
            force.source.get("relativeAcceleration").map(String::as_str),
            Some("0.2")
        );
        assert_eq!(
            force.source.get("targets").map(String::as_str),
            Some("hero.MetaBodies.Chest")
        );
        let track = project.track(force.props.track).unwrap();
        assert_eq!(track.name, "Forces");
    }

    /// A behaviour's parameters become its settings, by name; a target selection lists
    /// what it picked. Defaults (no `ja`), empty selections and helper graphics are left
    /// out.
    #[test]
    fn behaviour_parameters_become_settings() {
        let reach = r#"  <SecondaryBehaviours>
    <SecondaryBehaviour behaviourName="Bob2: Look and Reach 2.5" index="18" name="Behaviour01" startFrameNumber="0" endFrameNumber="30" priority="0" character="hero">
      <Parameters>
        <Variable fk="LeftHandTarget" kg="5" ja="MetaBodyAndCollisionSelection" numRequiredObjects="0">
          <ValuePtr kg="3" ja="background.MetaBodies.Prop0"/>
        </Variable>
        <Variable fk="RightHandTarget" kg="5" ja="MetaBodyAndCollisionSelection" numRequiredObjects="0"/>
        <Variable fk="LeftHand_Speed" kg="0" ja="0.700" de="HandReach_Speed"/>
        <Variable fk="LeftHand_Delay" kg="0" de="HandReach_Delay"/>
        <Variable fk="LeftHand_UsePrediction" kg="2" ja="false" de="HandReach_UsePrediction"/>
        <Variable fk="LookAtTargetLocator" kg="5" ja="HelperGraphic" de="UserHelperGraphic"/>
      </Parameters>
      <TargetSelection numRequiredObjects="0"/>
      <Objects/>
    </SecondaryBehaviour>
  </SecondaryBehaviours>
</Scene>"#;
        let Import { project, .. } = import(&scene_xml().replace("</Scene>", reach));
        let reach = project
            .clips
            .iter()
            .find(|c| matches!(&c.kind, ClipKind::Event { event } if event == "Behaviour"))
            .unwrap();
        let get = |key: &str| reach.source.get(key).map(String::as_str);
        assert_eq!(get("behaviourName"), Some("Bob2: Look and Reach 2.5"));
        assert_eq!(get("LeftHandTarget"), Some("background.MetaBodies.Prop0"));
        assert_eq!(get("LeftHand_Speed"), Some("0.700"));
        assert_eq!(get("LeftHand_UsePrediction"), Some("false"));
        assert_eq!(get("RightHandTarget"), None);
        assert_eq!(get("LeftHand_Delay"), None);
        assert_eq!(get("LookAtTargetLocator"), None);
    }

    #[test]
    fn behaviour_events_keep_their_parameters() {
        // As endorphin writes them: one Variable per parameter, its value in `ja`, none
        // when the behaviour's default is used.
        let xml = scene_xml().replace(
            "</ForceEvents>",
            r#"</ForceEvents>
  <SecondaryBehaviours>
    <SecondaryBehaviour behaviourName="Bob2: Explosion 2.0" index="27" name="5" displayName="Writhe in mid-air 2.0" startFrameNumber="48" endFrameNumber="60" priority="2" character="hero">
      <Parameters>
        <Variable fk="ShoulderAmplitude" kg="0" ja="0.7" de="ShoulderAmplitude"/>
        <Variable fk="Speed" kg="0" ja="0.6" de="Frequency"/>
        <Variable fk="NoiseLevel" kg="0"/>
        <Variable fk="priority" kg="0" ja="9"/>
      </Parameters>
      <TargetSelection numRequiredObjects="0"/>
      <Objects/>
    </SecondaryBehaviour>
  </SecondaryBehaviours>"#,
        );
        let Import { project, .. } = import(&xml);
        let behaviour = project
            .clips
            .iter()
            .find(|c| {
                c.kind
                    == ClipKind::Event {
                        event: "Behaviour".into(),
                    }
            })
            .unwrap();
        let get = |key: &str| behaviour.source.get(key).map(String::as_str);
        assert_eq!(get("behaviourName"), Some("Bob2: Explosion 2.0"));
        assert_eq!(get("ShoulderAmplitude"), Some("0.7"));
        assert_eq!(get("Speed"), Some("0.6"));
        assert_eq!(get("NoiseLevel"), None);
        // The event's own attributes win over a parameter of the same name.
        assert_eq!(get("priority"), Some("2"));
    }

    /// A crate placed at (3, 2, 0) and turned 90° about Y, with a start pose, a mass
    /// object whose collision surface has no geometry of its own, and keys in the scene's
    /// own animation (plus a camera's, which no character owns).
    fn crate_scene_xml() -> String {
        let turned = matrix(
            [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
            [3.0, 2.0, 0.0],
        );
        let rest = matrix(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0, 0.0, 0.0],
        );
        let posed = matrix(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0, 1.0, 0.5],
        );
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
            matches!(
                shapes.as_slice(),
                [ShapeDef {
                    geometry: Geometry::Box {
                        size: [0.4, 0.2, 0.6]
                    },
                    ..
                }]
            ),
            "a collision surface without geometry collides with the mass object: {shapes:?}"
        );

        // The start pose's root sits in the character's placement: (0, 1, 0.5) turned
        // 90° about Y is (0.5, 1, 0), plus (3, 2, 0).
        let root_at = |name: &str, key: usize| {
            let clip = project.clips.iter().find(|c| c.props.name == name).unwrap();
            let axis = |p: Property| {
                clip.channels
                    .iter()
                    .find(|c| c.joint == "root" && c.property == p)
                    .map_or(0.0, |c| c.keys[key].value)
            };
            Vec3::new(
                axis(Property::PosX),
                axis(Property::PosY),
                axis(Property::PosZ),
            )
        };
        assert!(
            root_at("Start pose", 0).distance(Vec3::new(3.5, 3.0, 0.0)) < 1e-4,
            "{}",
            root_at("Start pose", 0)
        );

        // The scene's own keys become a held clip, placed the same way: x = 2 at 1 s is
        // 2 m along the turned crate's -Z.
        assert!(root_at("Scene animation", 0).distance(Vec3::new(3.5, 3.0, 0.0)) < 1e-4);
        assert!(
            root_at("Scene animation", 1).distance(Vec3::new(3.5, 3.0, -2.0)) < 1e-4,
            "{}",
            root_at("Scene animation", 1)
        );
        assert_eq!(
            project
                .clips
                .iter()
                .filter(|c| c.props.name == "Scene animation")
                .count(),
            1,
            "the camera's keys are left out"
        );
        assert!(notes.iter().any(|n| n.contains("keyed in the scene")));
    }

    /// Back-Flip-To-Combo's helper body: a character MetaBody whose inertial body is a
    /// zero-sized Sphyl of density 0. Fixed in place it would hold the character up, so it
    /// gets a token mass; the environment's massless piece stays fixed scenery.
    /// endorphin's swing1 is the turn about the limit frame's Z axis and swing2 the turn
    /// about Y: read the other way, the poses its scenes store sit far outside their
    /// joints' cones. Imported scenes also count whole limbs for the muscle cap.
    #[test]
    fn swing_sizes_and_spring_cap() {
        let rest = matrix(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0, 0.0, 0.0],
        );
        let below = matrix(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0, -0.4, 0.0],
        );
        let limits = |tag: &str, swing1: f32, swing2: f32| {
            format!(
                r#"<{tag} btwistSize="{}" bswing1Size="{}" bswing2Size="{}"/>"#,
                hex(1.0),
                hex(swing1),
                hex(swing2)
            )
        };
        let xml = format!(
            r#"<?xml version="1.0"?>
<Scene>
  <Engine bpositionBufferFrameRate="{fps}"><PlaybackRange start="0" end="30"/></Engine>
  <Characters>
    <Character name="hero" typeName="skeletalCharacter2.5">
      <MetaBodies>
        <MetaBody name="PelvisMass">{rest}
          <InertialBody><Geometry type="Sphere" bparam0="{r}"/><Inertia density="{one}"/></InertialBody>
        </MetaBody>
        <MetaBody name="ThighMass">{below}
          <InertialBody><Geometry type="Sphere" bparam0="{r}"/><Inertia density="{one}"/></InertialBody>
        </MetaBody>
      </MetaBodies>
      <Skeleton name="Pelvis" metaBody="PelvisMass">{rest}<ChildJoints>
        <SkeletalJoint name="Thigh" metaBody="ThighMass">{below}{soft}{hard}</SkeletalJoint>
      </ChildJoints></Skeleton>
    </Character>
  </Characters>
</Scene>"#,
            fps = hex(30.0),
            r = hex(0.1),
            one = hex(1000.0),
            soft = limits("SoftLimits", 1.5, 0.9),
            hard = limits("HardLimits", 1.7, 1.1),
        );
        let Import { project, .. } = import(&xml);
        let rig = project.characters[0].physics.as_ref().unwrap();
        let joint = &rig.joints[0];
        // Half of each size; swing1 bounds the turn about Z.
        assert!((joint.soft.swing2 - 0.75).abs() < 1e-5, "{:?}", joint.soft);
        assert!((joint.soft.swing1 - 0.45).abs() < 1e-5, "{:?}", joint.soft);
        assert!((joint.hard.swing2 - 0.85).abs() < 1e-5, "{:?}", joint.hard);
        assert!((joint.hard.swing1 - 0.55).abs() < 1e-5, "{:?}", joint.hard);
        assert_eq!(project.physics.spring_cap, SpringCap::Limbs);
    }

    #[test]
    fn massless_character_body_gets_a_token_mass_but_scenery_stays_fixed() {
        let rest = matrix(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0, 0.0, 0.0],
        );
        let xml = format!(
            r#"<?xml version="1.0"?>
<Scene>
  <Engine bpositionBufferFrameRate="{fps}"><PlaybackRange start="0" end="30"/></Engine>
  <Characters>
    <Character name="hero" typeName="skeletalCharacter2.5">
      <MetaBodies>
        <MetaBody name="NormalMass">{rest}
          <InertialBody><Geometry type="Sphyl" bparam0="{zero}" bparam1="{zero}"/><Inertia density="{zero}"/></InertialBody>
        </MetaBody>
      </MetaBodies>
      <Skeleton name="root" metaBody="NormalMass">{rest}</Skeleton>
    </Character>
  </Characters>
  <Environment name="environment">
    <MetaBodies>
      <MetaBody name="Box">{rest}
        <CollisionSurface><Geometry type="Cuboid" bparam0="{one}" bparam1="{one}" bparam2="{one}"/></CollisionSurface>
      </MetaBody>
    </MetaBodies>
    <Skeleton name="root" metaBody="Box">{rest}</Skeleton>
  </Environment>
</Scene>"#,
            fps = hex(30.0),
            zero = hex(0.0),
            one = hex(1.0),
        );
        let Import { project, .. } = import(&xml);
        let body = |kind: CharacterKind| {
            let c = project
                .characters
                .iter()
                .find(|c| c.kind == kind)
                .unwrap_or_else(|| panic!("no {kind:?}"));
            c.physics.as_ref().unwrap().bodies[0].clone()
        };
        let helper = body(CharacterKind::Character);
        assert_eq!(
            (helper.mass, helper.inertia),
            (1.0, [0.004; 3]),
            "{helper:?}"
        );
        let scenery = body(CharacterKind::Environment);
        assert_eq!(
            (scenery.mass, scenery.inertia),
            (0.0, [0.0; 3]),
            "{scenery:?}"
        );
        assert_eq!(scenery.shapes.len(), 1);
    }

    /// A project with the placeholder biped (faces +Z, left side towards +X), its
    /// muscles named as endorphin's effectors, as `name`.
    fn placeholder_project(names: &[&str], t_pose_left_arm: bool) -> Project {
        let mut project = Project::new(ProjectSettings {
            name: "Poses".into(),
            fps: 60.0,
            range: FrameRange { start: 0, end: 60 },
        });
        for name in names {
            let mut skeleton = crate::document::sample::humanoid_skeleton();
            if t_pose_left_arm {
                for (joint, length) in [("l_elbow", 0.28), ("l_wrist", 0.25)] {
                    let j = skeleton.joint_index(joint).unwrap();
                    skeleton.joints[j].offset = [length, 0.0, 0.0];
                }
            }
            let mut rig = crate::document::sample::humanoid_rig(&skeleton);
            legacy_fixture::effector_names(&mut rig);
            let id = project.add_character(name, CharacterKind::Character, skeleton);
            project
                .characters
                .iter_mut()
                .find(|c| c.id == id)
                .unwrap()
                .physics = Some(rig);
        }
        project
    }

    /// The first character's joints in the world, held in the imported pose.
    fn posed_joints(project: &Project, pose: &PoseImport) -> Vec<(Quat, Vec3)> {
        let skeleton = &project.characters[0].skeleton;
        let mut held = Pose::rest(skeleton.joints.len());
        for c in &pose.channels {
            held.joints[skeleton.joint_index(&c.joint).unwrap()][c.property.index()] =
                c.keys[0].value;
        }
        world_transforms(skeleton, &held)
    }

    fn held_values(pose: &PoseImport) -> BTreeMap<(String, &'static str), f32> {
        pose.channels
            .iter()
            .map(|c| ((c.joint.clone(), c.property.label()), c.keys[0].value))
            .collect()
    }

    fn assert_near(
        values: &BTreeMap<(String, &'static str), f32>,
        joint: &str,
        axis: &str,
        degrees: f32,
    ) {
        let key = (joint.to_owned(), axis);
        let value = values.get(&key).copied().unwrap_or(0.0);
        assert!(
            (value - degrees).abs() < 0.05,
            "{joint} {axis}: {value}° instead of {degrees}° in {values:?}"
        );
    }

    /// On a character with the pose's own skeleton, each target is a turn about its
    /// hinge's axis: here the hinges rest unturned, so the axes are the world's.
    #[test]
    fn muscle_targets_turn_the_pose_s_own_hinges() {
        let mut project = placeholder_project(&[], false);
        let hinges = legacy_fixture::hinges();
        let pelvis = Vec3::new(0.0, 0.95, 0.0);
        let mut joints = vec![Joint::new("PelvisMB", None, pelvis.to_array())];
        for &(name, parent, at) in &hinges {
            let (p, from) = parent.map_or((0, pelvis), |p| (p + 1, hinges[p].2));
            joints.push(Joint::new(name, Some(p), (at - from).to_array()));
        }
        let skeleton = SkeletonRef {
            name: "Legacy".into(),
            source: None,
            joints,
        };
        project.add_character("legacy", CharacterKind::Character, skeleton);
        let pose = pose_from_text(&legacy_fixture::xml(0.2), "dp".into(), &project).unwrap();
        let values = held_values(&pose);
        let deg = f32::to_degrees;
        assert_near(
            &values,
            "PelvisMB_LoSpineZMB",
            "rotate Z",
            -deg(legacy_fixture::SPINE),
        );
        assert_near(
            &values,
            "PelvisMB_LHipZMB",
            "rotate Z",
            deg(legacy_fixture::HIP),
        );
        assert_near(&values, "LHipZMB_LUpLegYMB", "rotate Y", deg(0.2));
        assert_near(&values, "RHipZMB_RUpLegYMB", "rotate Y", -deg(0.2));
        assert_near(
            &values,
            "LUpLegYMB_LKneeZMB",
            "rotate Z",
            deg(legacy_fixture::KNEE),
        );
        assert_near(
            &values,
            "RShoulderZMB_RElbowZMB",
            "rotate Z",
            deg(legacy_fixture::ELBOW),
        );
        assert_eq!(values.len(), 11, "one turn per target: {values:?}");
        assert_eq!(
            pose.notes,
            ["1 muscle target(s) have no joint on legacy: BaseNeckZ."]
        );
    }

    /// On another character the targets go to the joints whose muscles carry the
    /// effectors' names, turned to the way it faces: the placeholder faces +Z, the pose
    /// +X. Hips and shoulders swing forwards, knees bend back, elbows forwards, the
    /// spine bends forwards; all about the placeholder's side axis (X).
    #[test]
    fn muscle_targets_drive_the_joints_named_by_their_muscles() {
        let project = placeholder_project(&["walker"], false);
        let pose = pose_from_text(&legacy_fixture::xml(0.0), "dp".into(), &project).unwrap();
        let values = held_values(&pose);
        let deg = f32::to_degrees;
        for side in ["l", "r"] {
            assert_near(
                &values,
                &format!("{side}_hip"),
                "rotate X",
                -deg(legacy_fixture::HIP),
            );
            assert_near(
                &values,
                &format!("{side}_knee"),
                "rotate X",
                -deg(legacy_fixture::KNEE),
            );
            let shoulder = -deg(legacy_fixture::SHOULDER);
            assert_near(&values, &format!("{side}_shoulder"), "rotate X", shoulder);
            assert_near(
                &values,
                &format!("{side}_elbow"),
                "rotate X",
                -deg(legacy_fixture::ELBOW),
            );
        }
        assert_near(&values, "spine", "rotate X", deg(legacy_fixture::SPINE));
        assert_eq!(values.len(), 9, "only side-axis turns: {values:?}");

        // Where that puts the legs: knees forwards of the hips, feet back under the knees.
        let skeleton = &project.characters[0].skeleton;
        let at = posed_joints(&project, &pose);
        let z = |joint: &str| at[skeleton.joint_index(joint).unwrap()].1.z;
        assert!(
            z("l_knee") > 0.15 && z("l_ankle") < z("l_knee") - 0.15,
            "{at:?}"
        );
        assert!(z("head") > 0.1, "the spine bends forwards: {at:?}");
    }

    /// The thighs' twist (`LHipY`, the hinge with no body of its own) reaches the hips;
    /// mirrored targets give mirrored legs.
    #[test]
    fn muscle_target_twist_reaches_the_hips() {
        let project = placeholder_project(&["walker"], false);
        let pose = pose_from_text(&legacy_fixture::xml(0.4), "dp".into(), &project).unwrap();
        let skeleton = &project.characters[0].skeleton;
        let at = posed_joints(&project, &pose);
        let p = |joint: &str| at[skeleton.joint_index(joint).unwrap()].1;
        let (left, right) = (p("l_ankle"), p("r_ankle"));
        assert!((left.x + right.x).abs() < 1e-3 && (left.z - right.z).abs() < 1e-3);
        // The twist turns the bent shin sideways (it was straight back at 0.1 m out).
        assert!((left.x - 0.1).abs() > 0.05, "{left}");
    }

    /// A character resting differently where the pose turns it (a T-pose arm against
    /// the pose's hanging one) is first turned to the pose's rest there, so the arm
    /// ends up where it does on a character that rests like the pose.
    #[test]
    fn muscle_targets_bring_a_t_pose_arm_where_the_pose_puts_it() {
        let place = |t_pose: bool| {
            let project = placeholder_project(&["walker"], t_pose);
            let pose = pose_from_text(&legacy_fixture::xml(0.0), "dp".into(), &project).unwrap();
            let at = posed_joints(&project, &pose);
            let skeleton = &project.characters[0].skeleton;
            let p = |joint: &str| at[skeleton.joint_index(joint).unwrap()].1;
            (p("l_elbow"), p("l_wrist"), p("r_wrist"), pose.notes)
        };
        let (elbow, wrist, right, notes) = place(false);
        let (t_elbow, t_wrist, t_right, t_notes) = place(true);
        assert!(
            !notes.iter().any(|n| n.contains("first turned")),
            "{notes:?}"
        );
        assert!(t_elbow.distance(elbow) < 0.01, "{t_elbow} against {elbow}");
        assert!(t_wrist.distance(wrist) < 0.01, "{t_wrist} against {wrist}");
        assert!(t_right.distance(right) < 1e-4, "the other arm is untouched");
        assert!(
            t_notes
                .iter()
                .any(|n| n.contains("first turned") && n.contains("l_shoulder")),
            "{t_notes:?}"
        );
    }

    /// The character the pose names wins if it fits; one with no matching joints or
    /// muscles doesn't take the pose at all.
    #[test]
    fn muscle_target_pose_picks_its_character() {
        let project = placeholder_project(&["first", "legacy"], false);
        let pose = pose_from_text(&legacy_fixture::xml(0.0), "dp".into(), &project).unwrap();
        assert_eq!(
            Some(pose.character),
            project.character_by_name("legacy").map(|c| c.id)
        );

        let mut bare = placeholder_project(&["bare"], false);
        bare.characters[0].physics = None;
        let error = pose_from_text(&legacy_fixture::xml(0.0), "dp".into(), &bare);
        assert!(matches!(error, Err(IoError::Format(e)) if e.contains("joints or muscles")));
    }

    #[test]
    fn rejects_unreadable_bytes() {
        assert!(matches!(
            decode_document(&[0x1b, 0xd1, 0xd7, 0xee, 0x3e]),
            Err(IoError::Format(_))
        ));
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
        // Expected failures: password-protected (encrypted) tutorial scenes.
        assert!(failed.iter().all(|f| f.contains("password")), "{failed:#?}");
        assert!(ok > 150);
    }

    /// Inventory of the behaviours (and event types) the scenes of a local endorphin
    /// install use, for planning which behaviours to design next. Reads only the scenes'
    /// own event settings, never the protected behaviour files.
    /// See docs/behaviour-inventory.md.
    /// Run with `ENDORPHIN_DIR=... cargo test --release behaviour_inventory -- --ignored --nocapture`.
    #[test]
    #[ignore = "needs a local endorphin 2.5 install"]
    fn behaviour_inventory() {
        use crate::physics::behaviour::Behaviour;
        use std::collections::BTreeSet;

        #[derive(Default)]
        struct Usage {
            events: usize,
            scenes: BTreeSet<String>,
            settings: BTreeMap<String, BTreeSet<String>>,
        }
        impl Usage {
            fn add(&mut self, other: &Usage) {
                self.events += other.events;
                self.scenes.extend(other.scenes.iter().cloned());
                for (key, values) in &other.settings {
                    self.settings
                        .entry(key.clone())
                        .or_default()
                        .extend(values.iter().cloned());
                }
            }
        }

        // Scenes name behaviours like "Bob2: Balance 2.5": a library prefix, the
        // behaviour, and a version. The family is the behaviour on its own.
        fn family(name: &str) -> String {
            let name = name.rsplit(':').next().unwrap_or(name).trim();
            let words: Vec<&str> = name.split(' ').collect();
            let versioned = words.len() > 1
                && words.last().is_some_and(|w| {
                    w.contains('.') && w.chars().all(|c| c.is_ascii_digit() || c == '.')
                });
            if versioned {
                words[..words.len() - 1].join(" ")
            } else {
                name.to_owned()
            }
        }

        let root = std::env::var("ENDORPHIN_DIR")
            .unwrap_or_else(|_| r"C:\Program Files (x86)\NaturalMotion\endorphin 2.5.2".to_owned());
        let mut scenes = Vec::new();
        let mut stack = vec![std::path::PathBuf::from(root)];
        while let Some(dir) = stack.pop() {
            for entry in fs::read_dir(&dir).into_iter().flatten().flatten() {
                let path = entry.path();
                if path.is_dir() {
                    stack.push(path);
                } else if path
                    .extension()
                    .is_some_and(|x| x.eq_ignore_ascii_case("ens"))
                {
                    scenes.push(path);
                }
            }
        }
        scenes.sort();

        let mut behaviours: BTreeMap<String, Usage> = BTreeMap::new();
        let mut types: BTreeMap<String, (usize, BTreeSet<String>)> = BTreeMap::new();
        let (mut imported, mut skipped) = (0, 0);
        for path in &scenes {
            // Password-protected scenes don't import; skip them like the sweep does.
            let Ok(scene) = import_scene(path) else {
                skipped += 1;
                continue;
            };
            imported += 1;
            let name = path.file_stem().unwrap().to_string_lossy().into_owned();
            for clip in &scene.project.clips {
                let ClipKind::Event { event } = &clip.kind else {
                    continue;
                };
                let kind = clip
                    .source
                    .get("endorphin.type")
                    .cloned()
                    .unwrap_or_default();
                let entry = types.entry(kind).or_default();
                entry.0 += 1;
                entry.1.insert(name.clone());
                // Secondary behaviour events (what sim.rs calls "Behaviour").
                if event != "Behaviour" {
                    continue;
                }
                let behaviour = clip
                    .source
                    .get("behaviourName")
                    .unwrap_or(&clip.props.name)
                    .clone();
                let usage = behaviours.entry(behaviour).or_default();
                usage.events += 1;
                usage.scenes.insert(name.clone());
                for (key, value) in &clip.source {
                    if key != "endorphin.type" && key != "behaviourName" {
                        usage
                            .settings
                            .entry(key.clone())
                            .or_default()
                            .insert(value.clone());
                    }
                }
            }
            // A behaviour's parameters arrive with the clip's settings (the importer's
            // `behaviour_parameters`); those a scene leaves at endorphin's default
            // aren't listed.
        }

        let have = |name: &str| {
            if Behaviour::from_name(name).is_some() {
                "yes"
            } else {
                "no"
            }
        };
        fn ranked(map: &BTreeMap<String, Usage>) -> Vec<(String, &Usage)> {
            let mut v: Vec<_> = map.iter().map(|(k, u)| (k.clone(), u)).collect();
            v.sort_by(|a, b| {
                (b.1.scenes.len(), b.1.events, &a.0).cmp(&(a.1.scenes.len(), a.1.events, &b.0))
            });
            v
        }

        println!(
            "scenes: {} found, {imported} imported, {skipped} skipped (protected)",
            scenes.len()
        );
        println!("\nEVENT TYPES (type | events | scenes)");
        let mut by_count: Vec<_> = types.iter().collect();
        by_count.sort_by(|a, b| b.1.0.cmp(&a.1.0).then(a.0.cmp(b.0)));
        for (kind, (events, in_scenes)) in by_count {
            println!("{kind} | {events} | {}", in_scenes.len());
        }

        println!("\nBEHAVIOUR NAMES AS IN THE SCENES (name | scenes | events | from_name matches)");
        for (name, usage) in ranked(&behaviours) {
            println!(
                "{name} | {} | {} | {}",
                usage.scenes.len(),
                usage.events,
                have(&name)
            );
        }

        let mut families: BTreeMap<String, Usage> = BTreeMap::new();
        for (name, usage) in &behaviours {
            families.entry(family(name)).or_default().add(usage);
        }
        println!(
            "\nFAMILIES, version and library prefix dropped (family | scenes | events | from_name matches)"
        );
        for (name, usage) in ranked(&families) {
            println!(
                "\n## {name} | {} scenes | {} events | {}",
                usage.scenes.len(),
                usage.events,
                have(&name)
            );
            println!(
                "scenes: {}",
                usage.scenes.iter().cloned().collect::<Vec<_>>().join(", ")
            );
            for (key, values) in &usage.settings {
                let numbers: Vec<f64> = values.iter().filter_map(|v| v.parse().ok()).collect();
                if numbers.len() == values.len() && values.len() > 3 {
                    let min = numbers.iter().cloned().fold(f64::INFINITY, f64::min);
                    let max = numbers.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                    println!("  {key}: {min} .. {max} ({} distinct)", values.len());
                } else {
                    let shown: Vec<_> = values.iter().take(6).cloned().collect();
                    let more = if values.len() > 6 { " | ..." } else { "" };
                    println!("  {key}: {}{more}", shown.join(" | "));
                }
            }
        }
        assert!(imported > 0, "no scenes found; set ENDORPHIN_DIR");
    }

    #[test]
    fn euler_unwrap_avoids_jumps() {
        let mut a = [179.0, -170.0, 10.0];
        unwrap_degrees(&mut a, [-179.0, 175.0, 0.0]);
        assert_eq!(a, [-181.0, 190.0, 10.0]);
    }
}
