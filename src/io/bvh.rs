//! Biovision Hierarchy (`.bvh`) export: one character's skeleton and motion.
//!
//! * Each joint's `OFFSET` is its rest offset in the parent joint's frame, so the
//!   hierarchy is the character's own skeleton. Roots are written at the origin.
//! * Rotation channels hold the joint's full local rotation (rest orientation times
//!   the animated rotation), so the motion plays back exactly without the importer
//!   knowing anything about rest orientations. The channel order is XYZ unless that
//!   would pass through gimbal lock for a joint, then whichever order is safest.
//! * Roots get position channels holding their full position (the convention Blender
//!   and MotionBuilder read; with a zero `OFFSET` it doesn't matter). Other joints only
//!   get them if their offset is animated.
//!
//! Details: docs/export.md.

use std::fmt::Write as _;
use std::path::Path;

use super::IoError;
use super::motion::{JointCurves, Motion, ORDERS, Units, export_names, joint_curves};
use crate::document::SkeletonRef;

pub const FILE_EXTENSION: &str = "bvh";

pub fn save(path: &Path, skeleton: &SkeletonRef, motion: &Motion, units: Units) -> Result<(), IoError> {
    std::fs::write(path, to_string(skeleton, motion, units))?;
    Ok(())
}

pub fn to_string(skeleton: &SkeletonRef, motion: &Motion, units: Units) -> String {
    let curves = joint_curves(skeleton, motion);
    let names = export_names(skeleton);
    let scale = units.scale();
    let children = |parent: usize| {
        skeleton
            .joints
            .iter()
            .enumerate()
            .filter(move |(i, j)| j.parent == Some(parent) && parent < *i)
            .map(|(i, _)| i)
    };

    // Hierarchy, depth first; the motion lines list channels in the same order.
    let mut out = String::from("HIERARCHY\n");
    let mut order = Vec::new();
    let mut stack: Vec<(usize, usize, bool)> = Vec::new();
    for root in (0..skeleton.joints.len()).filter(|&i| skeleton.joints[i].parent.is_none_or(|p| p >= i)).rev() {
        stack.push((root, 0, true));
    }
    // (joint, depth, opening): closing entries emit the end site (for leaves) and brace.
    while let Some((i, depth, opening)) = stack.pop() {
        let indent = "\t".repeat(depth);
        let joint = &skeleton.joints[i];
        if !opening {
            if children(i).next().is_none() {
                // Leaves end in a short tip along their own bone, for importers that draw bones.
                let tip = glam::Vec3::from(joint.offset) * 0.5;
                let tip = if tip.length() > 1e-4 { tip } else { glam::Vec3::new(0.0, 0.05, 0.0) };
                let _ = writeln!(out, "{indent}\tEnd Site\n{indent}\t{{\n{indent}\t\tOFFSET {}\n{indent}\t}}", triple(tip.to_array(), scale));
            }
            let _ = writeln!(out, "{indent}}}");
            continue;
        }
        let root = depth == 0;
        let offset = if root { [0.0; 3] } else { joint.offset };
        let _ = writeln!(out, "{indent}{} {}\n{indent}{{", if root { "ROOT" } else { "JOINT" }, names[i]);
        let _ = writeln!(out, "{indent}\tOFFSET {}", triple(offset, scale));
        let _ = writeln!(out, "{indent}\tCHANNELS {}", channel_list(&curves[i]));
        order.push(i);
        stack.push((i, depth, false));
        let kids: Vec<usize> = children(i).collect();
        for &c in kids.iter().rev() {
            stack.push((c, depth + 1, true));
        }
    }

    let _ = writeln!(out, "MOTION\nFrames: {}\nFrame Time: {:.8}", motion.frames.len(), 1.0 / motion.fps.max(1e-3));
    for frame in 0..motion.frames.len() {
        let mut values: Vec<String> = Vec::new();
        for &i in &order {
            let c = &curves[i];
            if let Some(t) = &c.translation {
                values.extend(t[frame].to_array().map(|v| format!("{:.6}", v * scale)));
            }
            values.extend(c.rotation[frame].map(|v| format!("{v:.6}")));
        }
        out.push_str(&values.join(" "));
        out.push('\n');
    }
    out
}

fn channel_list(curves: &JointCurves) -> String {
    let axes = ORDERS.iter().find(|o| o.0 == curves.order).map_or("XYZ", |o| o.1);
    let mut channels: Vec<String> = Vec::new();
    if curves.translation.is_some() {
        channels.extend(["Xposition", "Yposition", "Zposition"].map(String::from));
    }
    channels.extend(axes.chars().map(|a| format!("{a}rotation")));
    format!("{} {}", channels.len(), channels.join(" "))
}

fn triple(v: [f32; 3], scale: f32) -> String {
    format!("{:.6} {:.6} {:.6}", v[0] * scale, v[1] * scale, v[2] * scale)
}

#[cfg(test)]
pub(crate) mod tests {
    use glam::{Quat, Vec3};

    use super::*;
    use crate::document::sample::{physics_demo_project, placeholder_project};
    use crate::document::{
        CharacterKind, Channel, ClipKind, ClipProps, FrameRange, Interp, Joint, Keyframe, Project, ProjectSettings,
        Property,
    };
    use crate::io::motion::{sample, simulate};

    /// A joint read back from a BVH file.
    struct Node {
        name: String,
        parent: Option<usize>,
        offset: Vec3,
        channels: Vec<String>,
    }

    struct Parsed {
        nodes: Vec<Node>,
        frame_time: f32,
        frames: Vec<Vec<f32>>,
    }

    /// A minimal BVH reader, independent of the writer.
    fn parse(text: &str) -> Parsed {
        let mut tokens = text.split_whitespace().peekable();
        assert_eq!(tokens.next(), Some("HIERARCHY"));
        let mut nodes: Vec<Node> = Vec::new();
        // Open blocks: Some(node) for joints, None for end sites.
        let mut open: Vec<Option<usize>> = Vec::new();
        let mut pending: Option<Option<usize>> = None;
        let number = |t: Option<&str>| t.unwrap().parse::<f32>().unwrap();
        loop {
            match tokens.next().expect("unexpected end of hierarchy") {
                "ROOT" | "JOINT" => {
                    let parent = open.last().copied().flatten();
                    nodes.push(Node { name: tokens.next().unwrap().to_owned(), parent, offset: Vec3::ZERO, channels: vec![] });
                    pending = Some(Some(nodes.len() - 1));
                }
                "End" => {
                    assert_eq!(tokens.next(), Some("Site"));
                    pending = Some(None);
                }
                "{" => open.push(pending.take().expect("brace without a joint")),
                "}" => {
                    open.pop().expect("unbalanced braces");
                }
                "OFFSET" => {
                    let v = Vec3::new(number(tokens.next()), number(tokens.next()), number(tokens.next()));
                    if let Some(Some(i)) = open.last() {
                        nodes[*i].offset = v;
                    }
                }
                "CHANNELS" => {
                    let n = tokens.next().unwrap().parse::<usize>().unwrap();
                    let i = open.last().copied().flatten().expect("channels outside a joint");
                    nodes[i].channels = (0..n).map(|_| tokens.next().unwrap().to_owned()).collect();
                }
                "MOTION" => break,
                other => panic!("unexpected token {other}"),
            }
        }
        assert!(open.is_empty(), "unclosed hierarchy");
        assert_eq!(tokens.next(), Some("Frames:"));
        let count = tokens.next().unwrap().parse::<usize>().unwrap();
        assert_eq!((tokens.next(), tokens.next()), (Some("Frame"), Some("Time:")));
        let frame_time = number(tokens.next());
        let per_frame: usize = nodes.iter().map(|n| n.channels.len()).sum();
        let values: Vec<f32> = tokens.map(|t| t.parse().unwrap()).collect();
        assert_eq!(values.len(), count * per_frame, "motion data doesn't match the channels");
        Parsed { nodes, frame_time, frames: values.chunks(per_frame).map(<[f32]>::to_vec).collect() }
    }

    /// World positions of every node at a frame. Position channels replace the offset.
    fn forward_kinematics(parsed: &Parsed, frame: usize) -> Vec<Vec3> {
        let mut values = parsed.frames[frame].iter();
        let mut world: Vec<(Quat, Vec3)> = Vec::new();
        for node in &parsed.nodes {
            let (mut rot, mut pos) = (Quat::IDENTITY, node.offset);
            for channel in &node.channels {
                let v = *values.next().unwrap();
                let axis = match &channel[..1] {
                    "X" => 0,
                    "Y" => 1,
                    _ => 2,
                };
                if channel.ends_with("position") {
                    pos[axis] = v;
                } else {
                    rot *= Quat::from_axis_angle(Vec3::AXES[axis], v.to_radians());
                }
            }
            world.push(match node.parent {
                Some(p) => (world[p].0 * rot, world[p].1 + world[p].0 * pos),
                None => (rot, pos),
            });
        }
        world.into_iter().map(|w| w.1).collect()
    }

    /// Exports a character, reads the file back and checks every joint lands within
    /// `tolerance` metres of the source motion at the given frames.
    fn assert_round_trip(project: &Project, character: usize, frames: &[i32], tolerance: f32) -> Parsed {
        let simulation = simulate(project);
        let motion = sample(project, character, simulation.as_ref());
        let skeleton = &project.characters[character].skeleton;
        let parsed = parse(&to_string(skeleton, &motion, Units::Metres));
        assert_eq!(parsed.nodes.len(), skeleton.joints.len());
        assert_eq!(parsed.frames.len(), motion.frames.len());
        assert!((parsed.frame_time - 1.0 / project.settings.fps).abs() < 1e-6);
        let names = export_names(skeleton);
        let mut worst = 0.0f32;
        for &frame in frames {
            let index = (frame - project.settings.range.start) as usize;
            let positions = forward_kinematics(&parsed, index);
            for (node, position) in parsed.nodes.iter().zip(positions) {
                let joint = names.iter().position(|n| *n == node.name).unwrap();
                let error = position.distance(motion.frames[index][joint].1);
                worst = worst.max(error);
                assert!(error < tolerance, "{} is {error} m off at frame {frame}", node.name);
            }
        }
        eprintln!("worst joint error {worst} m");
        parsed
    }

    #[test]
    fn placeholder_animation_round_trips() {
        let parsed = assert_round_trip(&placeholder_project(), 0, &[0, 37, 90, 120, 240], 1e-4);
        assert_eq!(parsed.frames.len(), 241);
        assert_eq!(parsed.nodes[0].offset, Vec3::ZERO);
        assert_eq!(parsed.nodes[0].channels.len(), 6, "the root has position channels");
        assert!(parsed.nodes[1..].iter().all(|n| n.channels.len() == 3), "other joints only rotate");
    }

    #[test]
    fn physics_demo_exports_the_simulation() {
        let project = physics_demo_project();
        let motion = sample(&project, 0, simulate(&project).as_ref());
        assert!(motion.simulated);
        // After the shove the character is on the ground, which the keys alone never do.
        let chest = project.characters[0].skeleton.joint_index("chest").unwrap();
        assert!(motion.frames[150][chest].1.y < 0.6);
        assert_round_trip(&project, 0, &[0, 44, 46, 60, 100, 150], 1e-3);
    }

    /// A small chain whose joints have rotated rest frames, one of them sitting exactly
    /// at XYZ gimbal lock, animated with large rotations and an animated offset.
    pub(crate) fn rotated_rest_project() -> Project {
        let rest = |q: Quat| q.normalize().to_array();
        let joints = vec![
            Joint { name: "root".into(), parent: None, offset: [0.2, 1.0, -0.3], rest_rotation: rest(Quat::from_rotation_z(0.7)) },
            Joint { name: "upper arm".into(), parent: Some(0), offset: [0.0, 0.3, 0.1], rest_rotation: rest(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)) },
            Joint { name: "fore arm".into(), parent: Some(1), offset: [0.25, 0.0, 0.0], rest_rotation: rest(Quat::from_euler(glam::EulerRot::XYZ, 0.4, -1.1, 2.0)) },
            Joint { name: "hand".into(), parent: Some(2), offset: [0.2, 0.05, 0.0], rest_rotation: rest(Quat::from_rotation_x(-2.5)) },
            Joint { name: "thumb".into(), parent: Some(2), offset: [0.1, 0.0, 0.04], rest_rotation: [0.0, 0.0, 0.0, 1.0] },
        ];
        let mut project = Project::new(ProjectSettings { name: "Rotated rest".into(), fps: 24.0, range: FrameRange { start: -5, end: 40 } });
        let character = project.add_character("arm", CharacterKind::Character, crate::document::SkeletonRef { name: "arm".into(), source: None, joints });
        let track = project.add_track(character, "Base");
        let keys = |values: &[(i32, f32)]| values.iter().map(|&(frame, value)| Keyframe { frame, value, interp: Interp::Smooth }).collect();
        let channel = |joint: &str, property, values: &[(i32, f32)]| Channel { joint: joint.into(), property, keys: keys(values) };
        project.add_clip(
            ClipProps { name: "Swing".into(), track, start: 0, duration: 40, blend_in: 0, blend_out: 0, weight: 1.0, enabled: true },
            ClipKind::Motion,
            vec![
                channel("root", Property::PosX, &[(0, 0.0), (40, 1.5)]),
                channel("root", Property::RotY, &[(0, 0.0), (20, 170.0), (40, 400.0)]),
                channel("upper arm", Property::RotX, &[(0, 0.0), (10, 80.0), (30, -60.0)]),
                channel("upper arm", Property::RotZ, &[(0, 10.0), (40, -95.0)]),
                channel("fore arm", Property::RotY, &[(0, 0.0), (20, 120.0), (40, 0.0)]),
                channel("hand", Property::PosZ, &[(0, 0.0), (40, 0.1)]),
                channel("hand", Property::RotX, &[(0, -30.0), (40, 200.0)]),
            ],
        );
        project
    }

    #[test]
    fn rotated_rest_frames_round_trip() {
        let project = rotated_rest_project();
        let parsed = assert_round_trip(&project, 0, &[-5, 0, 7, 13, 20, 33, 40], 1e-4);
        let by_name = |name: &str| parsed.nodes.iter().find(|n| n.name == name).unwrap();
        assert_eq!(by_name("hand").channels.len(), 6, "an animated offset gets position channels");
        assert_eq!(by_name("thumb").channels.len(), 3);
        assert_eq!(by_name("fore_arm").offset, Vec3::new(0.25, 0.0, 0.0), "names lose their spaces, offsets stay");
        assert_ne!(by_name("upper_arm").channels[0], "Xrotation", "a joint at XYZ gimbal lock gets another order");
    }

    #[test]
    fn rotation_curves_are_continuous() {
        let project = rotated_rest_project();
        let motion = sample(&project, 0, None);
        let parsed = parse(&to_string(&project.characters[0].skeleton, &motion, Units::Metres));
        // The root turns through 400° without its curves jumping between frames.
        for pair in parsed.frames.windows(2) {
            for (node, (a, b)) in parsed.nodes.iter().flat_map(|n| n.channels.iter().map(move |c| (n, c))).zip(pair[0].iter().zip(&pair[1])) {
                let step = if node.1.ends_with("rotation") { 60.0 } else { 0.2 };
                assert!((a - b).abs() < step, "{} {} jumps from {a} to {b}", node.0.name, node.1);
            }
        }
    }

    #[test]
    fn centimetres_scale_lengths_only() {
        let project = placeholder_project();
        let motion = sample(&project, 0, None);
        let skeleton = &project.characters[0].skeleton;
        let (m, cm) = (parse(&to_string(skeleton, &motion, Units::Metres)), parse(&to_string(skeleton, &motion, Units::Centimetres)));
        assert!((cm.nodes[1].offset - m.nodes[1].offset * 100.0).length() < 1e-3);
        // Root Y position is scaled, root X rotation is not.
        assert!((cm.frames[10][1] - m.frames[10][1] * 100.0).abs() < 1e-2);
        assert_eq!(cm.frames[10][3], m.frames[10][3]);
    }
}
