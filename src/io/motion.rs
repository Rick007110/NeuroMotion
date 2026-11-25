//! What the exporters write: one character's joint transforms at every frame of the
//! project range, and those transforms turned into per-joint curves.
//!
//! The motion comes from the simulation when the project uses physics (characters
//! without a physics rig still follow their animation), otherwise from the animation.

use std::collections::HashSet;
use std::time::{Duration, Instant};

use glam::{EulerRot, Quat, Vec3};

use crate::document::{Project, SkeletonRef, euler_near, local_transforms, world_transforms};
use crate::physics::{Simulation, interp};

/// One character's motion over the project range.
pub struct Motion {
    pub fps: f32,
    /// Timeline frame of `frames[0]` (the project range start).
    pub start: i32,
    /// World rotation and position of every skeleton joint, one entry per frame.
    pub frames: Vec<Vec<(Quat, Vec3)>>,
    /// Whether the frames came from the simulation rather than the keys.
    pub simulated: bool,
}

/// Length unit written to export files. NeuroMotion works in metres.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Units {
    #[default]
    Metres,
    Centimetres,
}

impl Units {
    pub const ALL: [Units; 2] = [Units::Metres, Units::Centimetres];

    pub fn label(self) -> &'static str {
        match self {
            Units::Metres => "Metres (Blender)",
            Units::Centimetres => "Centimetres (Maya, MotionBuilder)",
        }
    }

    /// Multiplier from metres to this unit.
    pub fn scale(self) -> f32 {
        match self {
            Units::Metres => 1.0,
            Units::Centimetres => 100.0,
        }
    }
}

/// Runs the project's simulation over its whole range, or returns `None` when nothing
/// in the project is ever simulated.
#[cfg(test)]
pub fn simulate(project: &Project) -> Option<Simulation> {
    if !Simulation::is_needed(project) {
        return None;
    }
    let mut simulation = Simulation::default();
    finish(&mut simulation, project, 0);
    Some(simulation)
}

/// Brings a simulation up to date with `revision` and computes every remaining frame.
pub fn finish(simulation: &mut Simulation, project: &Project, revision: u64) {
    simulation.sync(revision);
    while !simulation.advance(project, Instant::now() + Duration::from_secs(1)) {}
}

/// The motion of `characters[character]` over the project range. `simulation` must be
/// complete for the project as it is now (see [`simulate`]); pass `None` to export the
/// animation.
pub fn sample(project: &Project, character: usize, simulation: Option<&Simulation>) -> Motion {
    sample_at_rate(project, character, simulation, 1)
}

/// Like [`sample`], but writing `rate` frames per project frame (2 turns a 30 fps scene
/// into 60 fps). The extra frames are interpolated between the cached simulation frames
/// (positions lerp, rotations slerp; see `physics::interp`) or, for keys, evaluated at
/// the fractional frame, so exactly the motion the viewport plays. Every `rate`-th
/// frame is the project's own frame, unchanged. `Motion::start` is in output frames.
pub fn sample_at_rate(
    project: &Project,
    character: usize,
    simulation: Option<&Simulation>,
    rate: u32,
) -> Motion {
    let rate = rate.max(1);
    let range = project.settings.range;
    let skeleton = &project.characters[character].skeleton;
    let mut simulated = false;
    let count = (range.end - range.start).max(0) * rate as i32 + 1;
    let frames = (0..count)
        .map(|i| {
            // Exact for whole frames, so the project's own frames aren't touched.
            let frame = range.start as f64 + i as f64 / rate as f64;
            let state = simulation
                .and_then(|s| interp::frame_at(s, frame))
                .and_then(|mut f| (character < f.len()).then(|| f.swap_remove(character)))
                .flatten();
            match state {
                Some(state) if state.joints.len() == skeleton.joints.len() => {
                    simulated = true;
                    state.joints
                }
                _ => {
                    let pose = project.evaluate(frame as f32).swap_remove(character);
                    world_transforms(skeleton, &pose)
                }
            }
        })
        .collect();
    Motion {
        fps: project.settings.fps * rate as f32,
        start: range.start * rate as i32,
        frames,
        simulated,
    }
}

/// A joint's motion as exporters write it: full local rotations (rest included) as
/// Euler angles, and local translations where the joint moves away from its offset.
pub struct JointCurves {
    /// Rotation order, chosen per joint to stay clear of gimbal lock.
    pub order: EulerRot,
    /// Degrees per frame, continuous from frame to frame.
    pub rotation: Vec<[f32; 3]>,
    /// Metres per frame, relative to the parent joint's frame. Always present on roots;
    /// on other joints only when their offset is animated.
    pub translation: Option<Vec<Vec3>>,
}

/// Offsets that move less than this are treated as fixed (simulated joints stretch by
/// fractions of a millimetre, which no rig wants as animation).
const TRANSLATION_TOLERANCE: f32 = 1e-3;

/// Euler orders an exporter may pick, as (glam order, axis letters in multiplication order).
pub const ORDERS: [(EulerRot, &str); 6] = [
    (EulerRot::XYZ, "XYZ"),
    (EulerRot::XZY, "XZY"),
    (EulerRot::YXZ, "YXZ"),
    (EulerRot::YZX, "YZX"),
    (EulerRot::ZXY, "ZXY"),
    (EulerRot::ZYX, "ZYX"),
];

pub fn joint_curves(skeleton: &SkeletonRef, motion: &Motion) -> Vec<JointCurves> {
    let locals: Vec<Vec<(Quat, Vec3)>> = motion
        .frames
        .iter()
        .map(|world| local_transforms(skeleton, world))
        .collect();
    // Where each joint lands per frame as exported. Leaving out one joint's stretch moves
    // every joint below it, so a joint keeps its translation when leaving it out would
    // put the joint itself more than the tolerance from where it is: a hip, knee and
    // ankle each stretched by under 1 mm put the ankle 1.1 mm out when only each
    // joint's own offset counted.
    let mut placed: Vec<Vec<Vec3>> = Vec::with_capacity(skeleton.joints.len());
    skeleton
        .joints
        .iter()
        .enumerate()
        .map(|(i, joint)| {
            let rotations: Vec<Quat> = locals.iter().map(|l| l[i].0).collect();
            let order = euler_order(&rotations);
            let mut previous = [0.0; 3];
            let rotation = rotations
                .iter()
                .map(|q| {
                    previous = euler_near(*q, order, previous);
                    previous
                })
                .collect();
            let positions: Vec<Vec3> = locals.iter().map(|l| l[i].1).collect();
            let offset = Vec3::from(joint.offset);
            let parent = joint.parent.filter(|&p| p < i);
            let moves = parent.is_none_or(|p| {
                motion.frames.iter().enumerate().any(|(f, world)| {
                    let at = placed[p][f] + world[p].0 * offset;
                    positions[f].distance(offset) > TRANSLATION_TOLERANCE
                        || at.distance(world[i].1) > TRANSLATION_TOLERANCE
                })
            });
            placed.push(
                motion
                    .frames
                    .iter()
                    .enumerate()
                    .map(|(f, world)| match parent {
                        Some(p) if !moves => placed[p][f] + world[p].0 * offset,
                        _ => world[i].1,
                    })
                    .collect(),
            );
            // A joint that keeps its translation is placed from where its parent is
            // exported, not from where it really is: a spine stretched 0.9 mm and left
            // out moved a head written relative to the real neck 1.6 mm off.
            let translation = moves.then(|| match parent {
                Some(p) => motion
                    .frames
                    .iter()
                    .enumerate()
                    .map(|(f, world)| world[p].0.inverse() * (world[i].1 - placed[p][f]))
                    .collect(),
                None => positions,
            });
            JointCurves {
                order,
                rotation,
                translation,
            }
        })
        .collect()
}

/// XYZ (NeuroMotion's own order) unless the motion passes near its gimbal lock, then
/// whichever order keeps the middle angle furthest from ±90°.
fn euler_order(rotations: &[Quat]) -> EulerRot {
    let worst = |order: EulerRot| {
        rotations
            .iter()
            .map(|q| q.to_euler(order).1.abs())
            .fold(0.0f32, f32::max)
    };
    if worst(EulerRot::XYZ) < 75f32.to_radians() {
        return EulerRot::XYZ;
    }
    ORDERS
        .iter()
        .map(|o| o.0)
        .min_by(|a, b| worst(*a).total_cmp(&worst(*b)))
        .unwrap_or(EulerRot::XYZ)
}

/// Joint names that are safe in export files: no whitespace or quotes, and unique.
pub fn export_names(skeleton: &SkeletonRef) -> Vec<String> {
    let mut used = HashSet::new();
    skeleton
        .joints
        .iter()
        .map(|joint| {
            let clean: String = joint
                .name
                .chars()
                .map(|c| {
                    if c.is_whitespace() || matches!(c, '"' | ':' | '{' | '}' | ',') {
                        '_'
                    } else {
                        c
                    }
                })
                .collect();
            let base = if clean.is_empty() {
                "joint".to_owned()
            } else {
                clean
            };
            let mut name = base.clone();
            let mut n = 2;
            while !used.insert(name.clone()) {
                name = format!("{base}_{n}");
                n += 1;
            }
            name
        })
        .collect()
}

/// Sample exports for checking in other tools. `cargo test -- --ignored export_samples`
/// writes BVH and FBX files of the built-in projects (and of an endorphin scene, if
/// `NEUROMOTION_SAMPLE_SCENE` names one) to `target/export-samples` (or
/// `NEUROMOTION_EXPORT_DIR`), each with a `.json` of the joint positions it should
/// produce. `tools/blender_check.py` imports them into Blender and compares.
#[cfg(test)]
mod samples {
    use std::path::PathBuf;

    use serde_json::json;

    use super::*;
    use crate::document::CharacterKind;
    use crate::document::sample::{physics_demo_project, placeholder_project};
    use crate::io::{bvh, fbx};

    #[test]
    #[ignore = "writes files for checking exports in Blender, see tools/blender_check.py"]
    fn export_samples() {
        let dir = std::env::var("NEUROMOTION_EXPORT_DIR")
            .map_or_else(|_| PathBuf::from("target/export-samples"), PathBuf::from);
        std::fs::create_dir_all(&dir).unwrap();
        let mut projects = vec![
            ("placeholder".to_owned(), placeholder_project()),
            ("physics-demo".to_owned(), physics_demo_project()),
            (
                "rotated-rest".to_owned(),
                crate::io::bvh::tests::rotated_rest_project(),
            ),
        ];
        if let Ok(scene) = std::env::var("NEUROMOTION_SAMPLE_SCENE") {
            let path = PathBuf::from(&scene);
            let name = path
                .file_stem()
                .unwrap()
                .to_string_lossy()
                .replace(' ', "-");
            projects.push((
                name,
                crate::io::endorphin::import_scene(&path).unwrap().project,
            ));
        }
        for (name, project) in projects {
            let simulation = simulate(&project);
            for (ci, character) in project.characters.iter().enumerate() {
                if character.kind == CharacterKind::Environment
                    || character.skeleton.joints.is_empty()
                {
                    continue;
                }
                let motion = sample(&project, ci, simulation.as_ref());
                let skeleton = &character.skeleton;
                let base = format!("{name}-{}", character.name.replace(' ', "-"));
                let file = |suffix: &str| dir.join(format!("{base}{suffix}"));
                bvh::save(&file(".bvh"), skeleton, &motion, Units::Metres).unwrap();
                bvh::save(&file("-cm.bvh"), skeleton, &motion, Units::Centimetres).unwrap();
                fbx::save(
                    &file(".fbx"),
                    skeleton,
                    &motion,
                    Units::Metres,
                    fbx::Encoding::Binary,
                )
                .unwrap();
                fbx::save(
                    &file("-cm.fbx"),
                    skeleton,
                    &motion,
                    Units::Centimetres,
                    fbx::Encoding::Binary,
                )
                .unwrap();
                fbx::save(
                    &file("-ascii.fbx"),
                    skeleton,
                    &motion,
                    Units::Metres,
                    fbx::Encoding::Ascii,
                )
                .unwrap();

                // Joint positions (metres, Y up) at a few frames, by exported joint name.
                let last = motion.frames.len() - 1;
                let checked: Vec<usize> = [0, last / 4, last / 2, 3 * last / 4, last]
                    .into_iter()
                    .collect();
                let names = export_names(skeleton);
                let joints: serde_json::Map<String, serde_json::Value> = names
                    .iter()
                    .enumerate()
                    .map(|(j, n)| {
                        (
                            n.clone(),
                            json!(
                                checked
                                    .iter()
                                    .map(|&f| motion.frames[f][j].1.to_array())
                                    .collect::<Vec<_>>()
                            ),
                        )
                    })
                    .collect();
                let reference = json!({
                    "fps": motion.fps,
                    "start": motion.start,
                    "frames": checked,
                    "simulated": motion.simulated,
                    "joints": joints,
                });
                std::fs::write(
                    file(".json"),
                    serde_json::to_string_pretty(&reference).unwrap(),
                )
                .unwrap();
                println!("wrote {}", file("").display());
            }
        }
    }
}

#[cfg(test)]
mod rate_tests {
    use super::*;
    use crate::document::sample::{physics_demo_project, placeholder_project};

    #[test]
    fn doubled_rate_keeps_the_project_frames_and_fills_between() {
        for project in [physics_demo_project(), placeholder_project()] {
            let sim = simulate(&project);
            let one = sample(&project, 0, sim.as_ref());
            let two = sample_at_rate(&project, 0, sim.as_ref(), 2);
            assert_eq!(two.fps, one.fps * 2.0);
            assert_eq!(two.start, one.start * 2);
            assert_eq!(two.frames.len(), (one.frames.len() - 1) * 2 + 1);
            assert_eq!(two.simulated, one.simulated);
            for (i, frame) in one.frames.iter().enumerate() {
                assert_eq!(
                    &two.frames[i * 2],
                    frame,
                    "project frame {i} must not change"
                );
            }
            // An in-between frame sits between its neighbours' positions.
            let (a, b, m) = (&one.frames[5], &one.frames[6], &two.frames[11]);
            for j in 0..m.len() {
                let (lo, hi) = (a[j].1.min(b[j].1), a[j].1.max(b[j].1));
                assert!(m[j].1.cmpge(lo - 1e-4).all() && m[j].1.cmple(hi + 1e-4).all());
            }
        }
    }
}
