//! Stand-ins for endorphin sample scenes that didn't simulate cleanly, built in code
//! (endorphin's files only exist on machines with endorphin installed), and the sweep
//! that checks them and, with `--ignored`, the real scenes.
//!
//! Each stand-in reproduces the pattern behind one failure with the placeholder
//! character and simple props, using the same event settings (`Clip::source` keys) the
//! endorphin importer produces. The sweep applies the same checks to both:
//!
//! - no non-finite values;
//! - no body more than 10 cm below the ground (fixed scenery, and bodies without
//!   collision shapes, aside: those fall through everything, in endorphin too);
//! - no joint stretched by 5 cm or more (free props have no joint to stretch);
//! - no simulated body faster than 40 m/s;
//! - (stand-ins) no locked body drifting 5 cm or more from where its lock holds it;
//! - (stand-ins) no body centre inside solid scenery.

use std::time::{Duration, Instant};

use glam::{Quat, Vec3};

use super::sim::{Simulation, lock_spans, rig_bodies};
use crate::document::physics::{BodyDef, Frame, Gains, Geometry, JointDef, Limits, MuscleGains, MuscleSet, PhysicsRig, ShapeDef};
use crate::document::sample::{humanoid_rig, humanoid_skeleton};
use crate::document::{
    CharacterId, CharacterKind, Channel, ClipKind, ClipProps, FrameRange, Interp, Joint, Keyframe, Pose, Project,
    ProjectSettings, Property, SimLevel, SkeletonRef, world_transforms,
};

/// What the sweep measured over a simulated range.
#[derive(Debug)]
pub struct Report {
    pub nan: bool,
    /// Lowest body centre, m.
    pub lowest: f32,
    /// Largest change in a skeleton bone's length, m.
    pub stretch: f32,
    /// Fastest body while simulated, m/s.
    pub fastest: f32,
    /// Largest distance a locked body moved away from where its lock holds it, m
    /// (along the axes the lock holds).
    pub drift: f32,
    /// Deepest a character's or prop's body centre got inside fixed scenery, m.
    pub buried: f32,
    /// Where each worst value happened ("character.body @ frame"), in the order above.
    pub places: [String; 5],
    pub elapsed: Duration,
}

impl Report {
    /// The endorphin sweep's checks.
    pub fn ok(&self) -> bool {
        !self.nan && self.lowest > -0.1 && self.stretch < 0.05 && self.fastest < 40.0
    }

    /// The sweep's checks, plus locks holding and nothing sinking into scenery.
    pub fn held(&self) -> bool {
        self.ok() && self.drift < 0.05 && self.buried <= 0.0
    }
}

impl Report {
    /// Where the worst values happened, one per line.
    pub fn places(&self) -> String {
        let names = ["lowest body", "max joint stretch", "fastest", "lock drift", "buried"];
        names.iter().zip(&self.places).filter(|(_, p)| !p.is_empty()).map(|(n, p)| format!("    {n:<18} {p}\n")).collect()
    }
}

impl std::fmt::Display for Report {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "lowest body {:>6.3} m  max joint stretch {:.4} m  fastest {:>5.1} m/s  lock drift {:.3} m  buried {:.3} m{}",
            self.lowest,
            self.stretch,
            self.fastest,
            self.drift,
            self.buried,
            if self.nan { "  NaN" } else { "" }
        )
    }
}

/// Simulates a project's whole range.
pub fn simulate(project: &Project) -> (Simulation, Duration) {
    let started = Instant::now();
    let mut sim = Simulation::default();
    sim.sync(1);
    while !sim.advance(project, Instant::now() + Duration::from_secs(120)) {}
    (sim, started.elapsed())
}

/// Simulates a project and measures the results.
pub fn sweep(project: &Project) -> Report {
    let (sim, elapsed) = simulate(project);
    measure(project, &sim, elapsed)
}

pub fn measure(project: &Project, sim: &Simulation, elapsed: Duration) -> Report {
    let fps = project.settings.fps;
    let range = project.settings.range;
    let mut report = Report { nan: false, lowest: f32::MAX, stretch: 0.0, fastest: 0.0, drift: 0.0, buried: 0.0, places: Default::default(), elapsed };
    let body_name = |ci: usize, bi: usize| {
        let c = &project.characters[ci];
        format!("{}.{}", c.name, c.physics.as_ref().map_or("?", |r| r.bodies[bi].name.as_str()))
    };
    for frame in range.start..=range.end {
        let frame_bodies = sim.frame(frame).unwrap();
        // Fixed scenery: the environment's massless pieces, or all of it while it isn't
        // simulated. Its props with mass are simulated like any character's bodies.
        let fixed = |ci: usize, body: &BodyDef| {
            let c = &project.characters[ci];
            c.kind == CharacterKind::Environment
                && (body.mass <= 0.0 || frame_bodies[ci].as_ref().is_some_and(|cf| cf.level != SimLevel::Simulated))
        };
        // Fixed scenery shapes where they are this frame.
        let mut scenery = Vec::new();
        for (ci, cf) in frame_bodies.iter().enumerate() {
            let (Some(cf), Some(rig)) = (cf, &project.characters[ci].physics) else { continue };
            for (body, &(r, p)) in rig.bodies.iter().zip(&cf.bodies).filter(|(b, _)| fixed(ci, b)) {
                for shape in &body.shapes {
                    let (sr, sp) = (Quat::from_array(shape.frame.rotation), Vec3::from(shape.frame.position));
                    scenery.push((shape.geometry, r * sr, p + r * sp));
                }
            }
        }
        for (ci, cf) in frame_bodies.iter().enumerate() {
            let Some(cf) = cf else { continue };
            let character = &project.characters[ci];
            let skeleton = &character.skeleton;
            let rig = character.physics.as_ref();
            for (bi, (_, p)) in cf.bodies.iter().enumerate() {
                report.nan |= !p.is_finite();
                // Fixed scenery may well sit below the ground, and a body without collision
                // shapes falls through it.
                let body = rig.map(|r| &r.bodies[bi]);
                if body.is_none_or(|b| !fixed(ci, b) && !b.shapes.is_empty()) {
                    if p.y < report.lowest {
                        report.lowest = p.y;
                        report.places[0] = format!("{} @ frame {frame}", body_name(ci, bi));
                    }
                    for &(geometry, r, at) in &scenery {
                        let depth = depth_inside(geometry, r, at, *p);
                        if depth > report.buried {
                            report.buried = depth;
                            report.places[4] = format!("{} @ frame {frame}", body_name(ci, bi));
                        }
                    }
                }
            }
            // A body that no physics joint holds (a prop) moves freely of its parent.
            let free = |joint: usize| {
                rig.is_some_and(|r| r.bodies.iter().enumerate().any(|(bi, b)| b.joint == joint && !r.joints.iter().any(|jd| jd.body == bi)))
            };
            for (i, j) in skeleton.joints.iter().enumerate() {
                if let Some(p) = j.parent.filter(|_| !free(i)) {
                    let gap = cf.joints[i].1.distance(cf.joints[p].1);
                    let stretch = (gap - Vec3::from(j.offset).length()).abs();
                    if stretch > report.stretch {
                        report.stretch = stretch;
                        report.places[1] = format!("{}.{} @ frame {frame} ({:?})", character.name, j.name, cf.level);
                    }
                }
            }
            // Speed is only meaningful while simulated: kinematic bodies jump back onto
            // the animation when a transition ends.
            let prev = (frame > range.start).then(|| sim.frame(frame - 1).unwrap()[ci].as_ref().unwrap());
            if let Some(prev) = prev.filter(|p| p.level == SimLevel::Simulated && cf.level == SimLevel::Simulated) {
                for (bi, (a, b)) in cf.bodies.iter().zip(&prev.bodies).enumerate() {
                    let speed = a.1.distance(b.1) * fps;
                    if speed > report.fastest {
                        report.fastest = speed;
                        report.places[2] = format!("{} @ frame {frame}", body_name(ci, bi));
                    }
                }
            }
        }
    }
    // Locks keep the offset their bodies had when the constraint switched on.
    let pose = |frame: i32, (ci, bi): (usize, usize)| sim.frame(frame).unwrap()[ci].as_ref().unwrap().bodies[bi];
    for lock in lock_spans(project) {
        if lock.linear == Vec3::ZERO || lock.start < range.start || lock.start >= range.end {
            continue;
        }
        // Held point in the anchor's frame (or the world), and the axes held.
        let offset = |frame: i32| {
            let (_, p) = pose(frame, lock.body);
            match lock.anchor {
                Some(a) => {
                    let (ar, ap) = pose(frame, a);
                    ar.inverse() * (p - ap)
                }
                None => p,
            }
        };
        let held = Vec3::select(lock.linear.cmpgt(Vec3::ZERO), Vec3::ONE, Vec3::ZERO);
        let start = offset(lock.start);
        for frame in lock.start + 1..=lock.end.min(range.end) {
            let drift = ((offset(frame) - start) * held).length();
            if drift > report.drift {
                report.drift = drift;
                let anchor = lock.anchor.map_or("the world".to_owned(), |(ci, bi)| body_name(ci, bi));
                report.places[3] = format!("{} to {anchor} @ frame {frame}", body_name(lock.body.0, lock.body.1));
            }
        }
    }
    report
}

/// The physics set-up of a project, for reading alongside a failing sweep: ground,
/// rigs, level switches and constraint events as imported.
pub fn describe(project: &Project) -> String {
    let mut out = String::new();
    let ground = project.physics.ground.as_ref().map_or("none".to_owned(), |g| format!("y = {} ({})", g.height, g.material));
    out += &format!("    ground {ground}, {} Hz, {} fps\n", project.physics.rate, project.settings.fps);
    for (ci, c) in project.characters.iter().enumerate() {
        let Some(rig) = &c.physics else { continue };
        let mut shapes: std::collections::BTreeMap<&str, usize> = Default::default();
        for shape in rig.bodies.iter().flat_map(|b| &b.shapes) {
            let kind = match shape.geometry {
                Geometry::Sphere { .. } => "sphere",
                Geometry::Capsule { .. } => "capsule",
                Geometry::Cylinder { radius, length } if length < 2.0 * radius => "disc",
                Geometry::Cylinder { .. } => "cylinder",
                Geometry::Box { .. } => "box",
            };
            *shapes.entry(kind).or_default() += 1;
        }
        let lightest = rig.bodies.iter().map(|b| (b.mass, b.name.as_str())).min_by(|a, b| a.0.total_cmp(&b.0));
        let switches: Vec<String> = project
            .clips
            .iter()
            .filter(|clip| project.clip_character(clip).map(|x| x.id) == Some(c.id))
            .filter(|clip| matches!(&clip.kind, ClipKind::Event { event } if event == "Simulation" || event == "Transition" || event == "Behaviour"))
            .map(|clip| {
                let what = ["physicsLevel", "requiredPhysicsLevel", "behaviourName"].iter().find_map(|k| clip.source.get(*k)).map_or("?", String::as_str);
                format!("{}-{} {what}", clip.props.start, clip.props.end())
            })
            .collect();
        out += &format!(
            "    [{ci}] {} ({:?}, starts {:?}): {} bodies, {:.1} kg, lightest {:?}, shapes {:?}, events [{}]\n",
            c.name,
            c.kind,
            c.default_level,
            rig.bodies.len(),
            rig.total_mass(),
            lightest,
            shapes,
            switches.join("; ")
        );
    }
    for clip in &project.clips {
        if !matches!(&clip.kind, ClipKind::Event { event } if event == "Constraint") {
            continue;
        }
        let settings: Vec<String> = clip.source.iter().filter(|(k, _)| k.as_str() != "targets").map(|(k, v)| format!("{k}={v}")).collect();
        let targets = clip.source.get("targets").map_or("", String::as_str);
        let masses: Vec<String> = targets
            .split(',')
            .map(|t| {
                let mut parts = t.trim().split(".MetaBodies.");
                let (c, b) = (parts.next().unwrap_or(""), parts.next().unwrap_or(""));
                let rig = project.character_by_name(c).and_then(|c| c.physics.as_ref());
                match rig.and_then(|r| Some((r.bodies.get(r.body_index(b)?)?.mass, r.total_mass()))) {
                    Some((m, total)) => format!("{m:.2} kg of {total:.0} kg"),
                    None => "UNRESOLVED".to_owned(),
                }
            })
            .collect();
        out += &format!(
            "    constraint {}-{}{}: targets [{targets}] ({}), {}\n",
            clip.props.start,
            clip.props.end(),
            if clip.props.enabled { "" } else { " (disabled)" },
            masses.join(", "),
            settings.join(" ")
        );
    }
    out
}

/// How far `point` is inside a placed shape (negative outside).
fn depth_inside(geometry: Geometry, rotation: Quat, position: Vec3, point: Vec3) -> f32 {
    let local = rotation.inverse() * (point - position);
    let along_z = |radius: f32, half: f32| radius - Vec3::new(local.x, local.y, local.z - local.z.clamp(-half, half)).length();
    match geometry {
        Geometry::Sphere { radius } => radius - local.length(),
        Geometry::Capsule { radius, length } => along_z(radius, length * 0.5),
        Geometry::Cylinder { radius, length } => (radius - local.truncate().length()).min(length * 0.5 - local.z.abs()),
        Geometry::Box { size } => (Vec3::from(size) * 0.5 - local.abs()).min_element(),
    }
}

// ---------------------------------------------------------------------------------
// Building scenes

fn new_scene(name: &str, seconds: f32) -> Project {
    let fps = 60.0;
    Project::new(ProjectSettings { name: name.to_owned(), fps, range: FrameRange { start: 0, end: (seconds * fps) as i32 } })
}

fn props(name: &str, track: crate::document::TrackId, start: i32, duration: i32) -> ClipProps {
    ClipProps { name: name.to_owned(), track, start, duration, blend_in: 0, blend_out: 0, weight: 1.0, enabled: true }
}

fn channel(joint: &str, property: Property, keys: &[(i32, f32)]) -> Channel {
    Channel {
        joint: joint.to_owned(),
        property,
        keys: keys.iter().map(|&(frame, value)| Keyframe { frame, value, interp: Interp::Linear }).collect(),
    }
}

/// A channel holding one value.
fn held(joint: &str, property: Property, value: f32) -> Channel {
    channel(joint, property, &[(0, value)])
}

/// Adds a motion clip on its own track. Inactive clips only place the character (like
/// an imported start pose); active ones are also muscle targets once it's simulated.
fn motion(project: &mut Project, character: CharacterId, name: &str, start: i32, duration: i32, active: bool, channels: Vec<Channel>) {
    let track = project.add_track(character, name);
    let id = project.add_clip(props(name, track, start, duration), ClipKind::Motion, channels);
    if !active {
        project.clip_mut(id).unwrap().source.insert("active".into(), "false".into());
    }
}

/// Adds an event clip with imported-style settings.
fn event(project: &mut Project, character: CharacterId, kind: &str, start: i32, duration: i32, settings: &[(&str, &str)]) {
    let track = project.add_track(character, kind);
    let id = project.add_clip(props(kind, track, start, duration), ClipKind::Event { event: kind.into() }, vec![]);
    project.clip_mut(id).unwrap().source = settings.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
}

fn level(project: &mut Project, character: CharacterId, frame: i32, level: &str) {
    event(project, character, "Simulation", frame, 1, &[("physicsLevel", level), ("momentumScale", "1")]);
}

fn push(project: &mut Project, character: CharacterId, frame: i32, target: &str, dv: Vec3) {
    let dv = format!("{}, {}, {}", dv.x, dv.y, dv.z);
    event(project, character, "Force", frame, 1, &[("matrix.Vi", &dv), ("targets", target)]);
}

/// A lock or join constraint event; strengths are (position, angular).
fn constrain(project: &mut Project, on: CharacterId, start: i32, end: i32, lock_type: &str, targets: &[&str], strength: (Vec3, Vec3)) {
    let v = |v: Vec3| format!("{}, {}, {}", v.x, v.y, v.z);
    let (linear, angular) = (v(strength.0), v(strength.1));
    let targets = targets.join(", ");
    let settings = [("lockType", lock_type), ("targets", targets.as_str()), ("positionStrength", &linear), ("angularStrength", &angular)];
    event(project, on, "Constraint", start, end - start, &settings);
}

const STRONG: Vec3 = Vec3::splat(1e5);

/// The placeholder character with endorphin-like light extremities: fingers on each
/// hand and toes on each foot, on tight hinges (endorphin's character has 28 bodies).
fn person_rig() -> (SkeletonRef, PhysicsRig) {
    let mut skeleton = humanoid_skeleton();
    for side in ["l", "r"] {
        let wrist = skeleton.joint_index(&format!("{side}_wrist")).unwrap();
        skeleton.joints.push(Joint::new(&format!("{side}_fingers"), Some(wrist), [0.0, -0.09, 0.0]));
    }
    let mut rig = humanoid_rig(&skeleton);
    for side in ["l", "r"] {
        let extras = [
            ("fingers", "wrist", Piece::along(Vec3::new(0.0, -0.07, 0.0), 0.012, 0.06), 0.8),
            ("toe", "ankle", Piece::at(Geometry::Box { size: [0.09, 0.03, 0.08] }, 0.12, Quat::IDENTITY, Vec3::new(0.0, -0.005, 0.04)), 0.5),
        ];
        for (joint, parent, piece, bend) in extras {
            let name = format!("{side}_{joint}");
            let j = skeleton.joint_index(&name).unwrap();
            let parent_body = rig.body_index(&format!("{side}_{parent}")).unwrap();
            rig.bodies.push(BodyDef {
                name: name.clone(),
                joint: j,
                frame: Frame { rotation: piece.frame.0.to_array(), position: piece.frame.1.to_array() },
                mass: piece.mass,
                inertia: piece.geometry.inertia(piece.mass),
                shapes: vec![ShapeDef {
                    geometry: piece.geometry,
                    frame: Frame::IDENTITY,
                    material: if joint == "toe" { "rubber" } else { "skin" }.into(),
                    collides_with_parent: false,
                    collides_with_siblings: true,
                    collides_with_ancestors: true,
                }],
            });
            // A hinge about the side axis, like endorphin's fingers and toes.
            let hard = Limits { twist_min: -bend, twist_max: bend, swing1: 0.02, swing2: 0.02 };
            let soft = Limits { twist_min: -bend + 0.05, twist_max: bend - 0.05, swing1: 0.01, swing2: 0.01 };
            let identity = Quat::IDENTITY.to_array();
            rig.joints.push(JointDef {
                name: name.clone(),
                joint: j,
                body: rig.bodies.len() - 1,
                parent_body,
                primary: identity,
                secondary: identity,
                hard,
                soft,
                soft_stiffness: 10.0,
                soft_damping: 1.0,
                muscle: name,
            });
        }
    }
    (skeleton, rig)
}

/// Adds the stand-in character with its feet at `at`, facing `yaw` degrees from +Z,
/// held in a start pose (joint, property, value) for the whole scene.
fn person(project: &mut Project, name: &str, at: Vec3, yaw: f32, pose: &[(&str, Property, f32)]) -> CharacterId {
    add_person(project, name, person_rig(), at, yaw, pose)
}

/// Adds the placeholder character itself, standing at `at`.
fn placeholder(project: &mut Project, name: &str, at: Vec3, yaw: f32) -> CharacterId {
    let skeleton = humanoid_skeleton();
    let rig = humanoid_rig(&skeleton);
    add_person(project, name, (skeleton, rig), at, yaw, &[])
}

fn add_person(project: &mut Project, name: &str, (skeleton, rig): (SkeletonRef, PhysicsRig), at: Vec3, yaw: f32, pose: &[(&str, Property, f32)]) -> CharacterId {
    let id = project.add_character(name, CharacterKind::Character, skeleton);
    let c = project.characters.iter_mut().find(|c| c.id == id).unwrap();
    c.physics = Some(rig);
    c.default_level = SimLevel::Simulated;
    let mut channels = vec![
        held("hips", Property::PosX, at.x),
        held("hips", Property::PosY, at.y),
        held("hips", Property::PosZ, at.z),
        held("hips", Property::RotY, yaw),
    ];
    channels.extend(pose.iter().map(|&(joint, property, value)| held(joint, property, value)));
    let duration = project.settings.range.len();
    motion(project, id, "Start pose", 0, duration, false, channels);
    id
}

/// One rigid piece of a prop: shape, mass and where it sits relative to its joint.
struct Piece {
    geometry: Geometry,
    mass: f32,
    frame: (Quat, Vec3),
}

impl Piece {
    /// A capsule (or other swept shape) spanning `bone` from its joint.
    fn along(bone: Vec3, radius: f32, mass: f32) -> Self {
        let length = (bone.length() - 2.0 * radius).max(0.0);
        let rotation = Quat::from_rotation_arc(Vec3::Z, bone.normalize());
        Piece { geometry: Geometry::Capsule { radius, length }, mass, frame: (rotation, bone * 0.5) }
    }

    fn at(geometry: Geometry, mass: f32, rotation: Quat, position: Vec3) -> Self {
        Piece { geometry, mass, frame: (rotation, position) }
    }
}

/// Builds a prop from joints (name, parent, offset, piece) with ball joints between
/// pieces. Every joint gets one body. Muscles hold the rest pose with `gains`.
fn prop_rig(joints: Vec<(&str, Option<usize>, Vec3, Piece)>, limits: Limits, gains: Gains) -> (SkeletonRef, PhysicsRig) {
    let skeleton = SkeletonRef {
        name: "Prop".into(),
        source: None,
        joints: joints.iter().map(|(name, parent, offset, _)| Joint::new(name, *parent, offset.to_array())).collect(),
    };
    let mut bodies = Vec::new();
    let mut rig_joints = Vec::new();
    for (i, (name, parent, _, piece)) in joints.into_iter().enumerate() {
        let frame = Frame { rotation: piece.frame.0.to_array(), position: piece.frame.1.to_array() };
        bodies.push(BodyDef {
            name: name.to_owned(),
            joint: i,
            frame,
            mass: piece.mass,
            inertia: piece.geometry.inertia(piece.mass),
            shapes: vec![ShapeDef {
                geometry: piece.geometry,
                frame: Frame::IDENTITY,
                material: "default".into(),
                collides_with_parent: false,
                collides_with_siblings: false,
                collides_with_ancestors: false,
            }],
        });
        if let Some(parent) = parent {
            // Twist about the piece's own axis.
            let twist = Quat::from_rotation_arc(Vec3::X, piece.frame.0 * Vec3::Z).to_array();
            let soft = Limits {
                twist_min: limits.twist_min + 0.05,
                twist_max: limits.twist_max - 0.05,
                swing1: limits.swing1 - 0.05,
                swing2: limits.swing2 - 0.05,
            };
            rig_joints.push(JointDef {
                name: name.to_owned(),
                joint: i,
                body: i,
                parent_body: parent,
                primary: twist,
                secondary: twist,
                hard: limits,
                soft,
                soft_stiffness: 10.0,
                soft_damping: 10.0,
                muscle: "prop".into(),
            });
        }
    }
    let both = MuscleGains { twist: gains, swing: gains };
    let set = MuscleSet { name: "Default".into(), muscles: Default::default(), fallback: both };
    (skeleton, PhysicsRig { bodies, joints: rig_joints, muscle_sets: vec![set], isotropic_inertia: true })
}

fn add_prop(project: &mut Project, name: &str, kind: CharacterKind, (skeleton, rig): (SkeletonRef, PhysicsRig), level: SimLevel) -> CharacterId {
    let id = project.add_character(name, kind, skeleton);
    let c = project.characters.iter_mut().find(|c| c.id == id).unwrap();
    c.physics = Some(rig);
    c.default_level = level;
    id
}

/// A hanging chain of `count` capsules from `top`, each `length` long, ending in an
/// optional heavier tip piece.
fn chain(top: Vec3, count: usize, length: f32, radius: f32, mass: f32, tip: Option<Piece>, limits: Limits, gains: Gains) -> (SkeletonRef, PhysicsRig) {
    let down = Vec3::new(0.0, -length, 0.0);
    let mut joints: Vec<(&str, Option<usize>, Vec3, Piece)> = Vec::new();
    const NAMES: [&str; 12] = ["seg0", "seg1", "seg2", "seg3", "seg4", "seg5", "seg6", "seg7", "seg8", "seg9", "seg10", "seg11"];
    for (i, name) in NAMES.iter().enumerate().take(count) {
        let (parent, offset) = if i == 0 { (None, top) } else { (Some(i - 1), down) };
        joints.push((name, parent, offset, Piece::along(down, radius, mass)));
    }
    if let Some(tip) = tip {
        joints.push(("tip", Some(count - 1), down, tip));
    }
    prop_rig(joints, limits, gains)
}

/// Static or animated scenery: pieces (geometry, centre) hanging from one root joint.
fn scenery(pieces: &[(Geometry, Quat, Vec3)]) -> (SkeletonRef, PhysicsRig) {
    const NAMES: [&str; 4] = ["piece0", "piece1", "piece2", "piece3"];
    let skeleton = SkeletonRef { name: "Scenery".into(), source: None, joints: vec![Joint::new("root", None, [0.0; 3])] };
    let bodies = pieces
        .iter()
        .zip(NAMES)
        .map(|(&(geometry, rotation, position), name)| BodyDef {
            name: name.into(),
            joint: 0,
            frame: Frame { rotation: rotation.to_array(), position: position.to_array() },
            mass: 1000.0,
            inertia: geometry.inertia(1000.0),
            shapes: vec![ShapeDef {
                geometry,
                frame: Frame::IDENTITY,
                material: "default".into(),
                collides_with_parent: false,
                collides_with_siblings: false,
                collides_with_ancestors: false,
            }],
        })
        .collect();
    (skeleton, PhysicsRig { bodies, joints: Vec::new(), muscle_sets: Vec::new(), isotropic_inertia: true })
}

/// World transform of one of a character's bodies at a frame of its animation.
fn body_pose(project: &Project, character: CharacterId, body: &str, frame: f32) -> (Quat, Vec3) {
    let ci = project.character_index(character).unwrap();
    let c = &project.characters[ci];
    let rig = c.physics.as_ref().unwrap();
    let pose: Pose = project.evaluate(frame).swap_remove(ci);
    rig_bodies(rig, &world_transforms(&c.skeleton, &pose))[rig.body_index(body).unwrap()]
}

// ---------------------------------------------------------------------------------
// Behaviour scenes

fn behave(project: &mut Project, character: CharacterId, start: i32, end: i32, name: &str) {
    event(project, character, "Behaviour", start, end - start, &[("behaviourName", name)]);
}

/// The placeholder stands; at 1 s its chest is pushed at `speed` m/s along `dir`.
/// With `behaviour`, Balance runs throughout.
pub fn balance_push(speed: f32, dir: Vec3, behaviour: bool) -> Project {
    let mut p = new_scene("Balance", 4.0);
    let id = placeholder(&mut p, "stander", Vec3::ZERO, 0.0);
    if behaviour {
        behave(&mut p, id, 0, 240, "Balance");
    }
    push(&mut p, id, 60, "stander.MetaBodies.chest", dir.normalize() * speed);
    p
}

/// The placeholder is shoved over forwards at 0.5 s; with `behaviour`, CatchFall runs
/// from the shove on.
pub fn catch_fall_shove(shove: Vec3, behaviour: bool) -> Project {
    let mut p = new_scene("Catch fall", 2.5);
    let id = placeholder(&mut p, "faller", Vec3::ZERO, 0.0);
    if behaviour {
        behave(&mut p, id, 30, 150, "CatchFall");
    }
    push(&mut p, id, 30, "faller.MetaBodies.chest", shove);
    p
}

// ---------------------------------------------------------------------------------
// The stand-ins

/// Tackle: a tackler runs in with momentum (collision with momentum), is joined to the
/// other character by a strong JoinBodies while their bodies collide, drives it into the
/// ground and goes fully simulated mid-scene, still pressing on it.
pub fn tackle() -> Project {
    let mut p = new_scene("Tackle", 2.0);
    let victim = person(&mut p, "victim", Vec3::ZERO, 180.0, &[]);
    let reach = [
        ("spine", Property::RotX, 20.0),
        ("l_shoulder", Property::RotX, -80.0),
        ("r_shoulder", Property::RotX, -80.0),
        ("l_shoulder", Property::RotZ, -15.0),
        ("r_shoulder", Property::RotZ, 15.0),
        ("l_knee", Property::RotX, 25.0),
        ("r_knee", Property::RotX, 25.0),
    ];
    let tackler = person(&mut p, "tackler", Vec3::new(0.0, 0.0, -1.6), 0.0, &reach);
    // 5 m/s run, diving onto the victim and driving it into the ground, kinematic until
    // frame 30.
    let dive = vec![
        channel("hips", Property::PosZ, &[(0, -1.6), (15, -0.35), (30, 0.4)]),
        channel("hips", Property::PosY, &[(0, 0.0), (15, 0.0), (30, -0.2)]),
        channel("hips", Property::RotX, &[(0, 0.0), (15, 0.0), (30, 30.0)]),
    ];
    motion(&mut p, tackler, "Run and dive", 0, 30, true, dive);
    event(&mut p, tackler, "Simulation", 0, 1, &[("physicsLevel", "CollisionOnlyWithInertia"), ("momentumScale", "0.3")]);
    level(&mut p, tackler, 30, "FullPhysics");
    let targets = ["tackler.MetaBodies.l_wrist", "tackler.MetaBodies.r_wrist", "tackler.MetaBodies.chest", "tackler.MetaBodies.head", "victim.MetaBodies.chest"];
    constrain(&mut p, victim, 20, 120, "JoinBodies", &targets, (STRONG, STRONG));
    p
}

/// Balcony: four characters crowded together on a balcony slab with a railing, two of
/// them shoved into the others.
pub fn balcony() -> Project {
    let mut p = new_scene("Balcony", 2.0);
    let slab = Geometry::Box { size: [4.0, 0.3, 2.0] };
    let rail = Geometry::Box { size: [4.0, 1.0, 0.08] };
    let rig = scenery(&[(slab, Quat::IDENTITY, Vec3::new(0.0, 2.85, 0.0)), (rail, Quat::IDENTITY, Vec3::new(0.0, 3.5, 0.96))]);
    add_prop(&mut p, "balcony", CharacterKind::Environment, rig, SimLevel::CollisionOnly);
    let mut people = Vec::new();
    // Shoulder to shoulder, arms overlapping their neighbours'.
    for (i, x) in [-0.54f32, -0.18, 0.18, 0.54].into_iter().enumerate() {
        let name = ["guest1", "guest2", "guest3", "guest4"][i];
        let arms = [("l_shoulder", Property::RotZ, 12.0), ("r_shoulder", Property::RotZ, -12.0)];
        people.push(person(&mut p, name, Vec3::new(x, 3.0, 0.45), 0.0, &arms));
    }
    // One walks into the others from behind, then goes limp; two are shoved.
    let late = person(&mut p, "latecomer", Vec3::new(0.0, 3.0, -0.6), 0.0, &[("l_shoulder", Property::RotX, -60.0), ("r_shoulder", Property::RotX, -60.0)]);
    motion(&mut p, late, "Barge in", 0, 20, true, vec![channel("hips", Property::PosZ, &[(0, -0.6), (20, 0.2)])]);
    level(&mut p, late, 0, "CollisionOnlyWithInertia");
    level(&mut p, late, 20, "FullPhysics");
    push(&mut p, people[1], 25, "guest2.MetaBodies.chest", Vec3::new(0.0, 1.0, 5.0));
    push(&mut p, people[3], 30, "guest4.MetaBodies.chest", Vec3::new(-4.0, 0.0, 0.5));
    p
}

/// Catch Rope: a character runs and jumps (collision with momentum), then goes fully
/// simulated and grabs a light rope that hangs from a Lock holding only X and Y.
pub fn catch_rope() -> Project {
    let mut p = new_scene("Catch Rope", 2.0);
    let rope_limits = Limits { twist_min: -0.5, twist_max: 0.5, swing1: 1.2, swing2: 1.2 };
    let rig = chain(Vec3::new(0.0, 4.6, 0.0), 10, 0.3, 0.02, 0.15, None, rope_limits, Gains { stiffness: 0.5, damping: 0.05, force_limit: 5.0 });
    let rope = add_prop(&mut p, "rope", CharacterKind::Prop, rig, SimLevel::Simulated);
    constrain(&mut p, rope, 0, 120, "Lock", &["rope.MetaBodies.seg0"], (Vec3::new(1e5, 1e5, 0.0), Vec3::ZERO));

    let arms_up = [("l_shoulder", Property::RotX, -170.0), ("r_shoulder", Property::RotX, -170.0)];
    let catcher = person(&mut p, "catcher", Vec3::new(0.0, 0.0, -2.0), 0.0, &arms_up);
    let jump = vec![
        channel("hips", Property::PosZ, &[(0, -2.0), (20, -0.2)]),
        channel("hips", Property::PosY, &[(0, 0.0), (20, 0.4)]),
    ];
    motion(&mut p, catcher, "Run and jump", 0, 20, true, jump);
    level(&mut p, catcher, 0, "CollisionOnlyWithInertia");
    level(&mut p, catcher, 20, "FullPhysics");
    for hand in ["catcher.MetaBodies.l_wrist", "catcher.MetaBodies.r_wrist"] {
        constrain(&mut p, catcher, 20, 120, "JoinBodies", &[hand, "rope.MetaBodies.seg7"], (STRONG, STRONG));
    }
    p
}

/// Gymnast Rings: two light straps with rings hang from Locks; a gymnast hangs from the
/// rings by LockWithParent constraints, hands overlapping the rings, and is swung.
pub fn gymnast_rings() -> Project {
    let mut p = new_scene("Gymnast Rings", 2.0);
    let strap = Limits { twist_min: -0.5, twist_max: 0.5, swing1: 1.0, swing2: 1.0 };
    let gains = Gains { stiffness: 1.0, damping: 0.1, force_limit: 5.0 };
    for (name, x) in [("ring_l", 0.19f32), ("ring_r", -0.19)] {
        let ring = Piece::at(Geometry::Sphere { radius: 0.09 }, 0.3, Quat::IDENTITY, Vec3::new(0.0, -0.09, 0.0));
        let rig = chain(Vec3::new(x, 5.0, 0.0), 4, 0.5, 0.015, 0.2, Some(ring), strap, gains);
        let prop = add_prop(&mut p, name, CharacterKind::Prop, rig, SimLevel::Simulated);
        constrain(&mut p, prop, 0, 120, "Lock", &[&format!("{name}.MetaBodies.seg0")], (STRONG, Vec3::ZERO));
    }
    let arms_up = [("l_shoulder", Property::RotX, -180.0), ("r_shoulder", Property::RotX, -180.0)];
    let gymnast = person(&mut p, "gymnast", Vec3::new(0.0, 0.84, 0.0), 0.0, &arms_up);
    constrain(&mut p, gymnast, 0, 120, "LockWithParent", &["gymnast.MetaBodies.l_wrist", "ring_l.MetaBodies.tip"], (STRONG, STRONG));
    constrain(&mut p, gymnast, 0, 120, "LockWithParent", &["gymnast.MetaBodies.r_wrist", "ring_r.MetaBodies.tip"], (STRONG, STRONG));
    push(&mut p, gymnast, 30, "gymnast.MetaBodies.hips", Vec3::new(0.0, 0.0, 2.0));
    p
}

/// Bendy Stick: an animated (collision-only) character swings a springy stick that is
/// joined to its hand.
pub fn bendy_stick() -> Project {
    let mut p = new_scene("Bendy Stick", 2.0);
    let pose = [("r_shoulder", Property::RotX, -70.0), ("r_elbow", Property::RotX, -20.0)];
    let holder = person(&mut p, "holder", Vec3::ZERO, 0.0, &pose);
    let swing: Vec<(i32, f32)> = (0..=120).step_by(5).map(|f| (f, 40.0 * (f as f32 / 60.0 * std::f32::consts::TAU).sin())).collect();
    motion(&mut p, holder, "Swing", 0, 120, true, vec![channel("r_shoulder", Property::RotZ, &swing)]);
    level(&mut p, holder, 0, "CollisionOnly");

    // The stick starts in the hand, pointing along the forearm.
    let (hand_rot, hand_pos) = body_pose(&p, holder, "r_wrist", 0.0);
    let along = hand_rot * Vec3::Z;
    let along = if along.dot(Vec3::Z) < 0.0 { -along } else { along };
    let start = hand_pos + along * 0.06;
    let segment = along * 0.25;
    let limits = Limits { twist_min: -0.3, twist_max: 0.3, swing1: 0.6, swing2: 0.6 };
    let mut joints: Vec<(&str, Option<usize>, Vec3, Piece)> = Vec::new();
    for (i, name) in ["seg0", "seg1", "seg2", "seg3", "seg4", "seg5"].into_iter().enumerate() {
        let (parent, offset) = if i == 0 { (None, start) } else { (Some(i - 1), segment) };
        joints.push((name, parent, offset, Piece::along(segment, 0.02, 0.3)));
    }
    let rig = prop_rig(joints, limits, Gains { stiffness: 30.0, damping: 1.0, force_limit: 100.0 });
    add_prop(&mut p, "stick", CharacterKind::Prop, rig, SimLevel::Simulated);
    constrain(&mut p, holder, 0, 120, "JoinBodies", &["stick.MetaBodies.seg0", "holder.MetaBodies.r_wrist"], (STRONG, STRONG));
    p
}

/// Heli Leap: a heavy helicopter (fuselage, tail boom, skids) held in the air by a
/// Lock; a character jumps off its roof and falls to the ground.
pub fn heli_leap() -> Project {
    let mut p = new_scene("Heli Leap", 2.0);
    let fuselage = Piece::at(Geometry::Box { size: [1.6, 1.4, 3.0] }, 1500.0, Quat::IDENTITY, Vec3::ZERO);
    let tail = Piece::along(Vec3::new(0.0, 0.0, -4.0), 0.15, 150.0);
    let skid = |x: f32| Piece::at(Geometry::Capsule { radius: 0.05, length: 2.6 }, 40.0, Quat::IDENTITY, Vec3::new(x, 0.0, 0.0));
    let joints = vec![
        ("fuselage", None, Vec3::new(0.0, 6.0, 0.0), fuselage),
        ("tail", Some(0), Vec3::new(0.0, 0.3, -1.5), tail),
        ("skid_l", Some(0), Vec3::new(0.0, -1.0, 0.0), skid(0.7)),
        ("skid_r", Some(0), Vec3::new(0.0, -1.0, 0.0), skid(-0.7)),
    ];
    let limits = Limits { twist_min: -0.2, twist_max: 0.2, swing1: 0.2, swing2: 0.2 };
    let rig = prop_rig(joints, limits, Gains { stiffness: 2e5, damping: 2e4, force_limit: 1e5 });
    let heli = add_prop(&mut p, "helicopter", CharacterKind::Prop, rig, SimLevel::Simulated);
    constrain(&mut p, heli, 0, 120, "Lock", &["helicopter.MetaBodies.fuselage"], (STRONG, STRONG));

    let jumper = person(&mut p, "jumper", Vec3::new(0.0, 6.7, 0.5), 0.0, &[]);
    let leap = vec![
        channel("hips", Property::PosZ, &[(0, 0.5), (20, 2.0)]),
        channel("hips", Property::PosY, &[(0, 6.7), (20, 7.1)]),
    ];
    motion(&mut p, jumper, "Leap", 0, 20, true, leap);
    level(&mut p, jumper, 0, "CollisionOnlyWithInertia");
    level(&mut p, jumper, 20, "FullPhysics");
    p
}

/// Rotating Floor: no ground plane; the floor is a thin collision-only slab that spins
/// and rocks under a simulated character.
pub fn rotating_floor() -> Project {
    let mut p = new_scene("Rotating Floor", 2.0);
    p.physics.ground = None;
    let slab = Geometry::Box { size: [5.0, 0.1, 5.0] };
    let floor = add_prop(&mut p, "floor", CharacterKind::Environment, scenery(&[(slab, Quat::IDENTITY, Vec3::new(0.0, -0.05, 0.0))]), SimLevel::CollisionOnly);
    let rock: Vec<(i32, f32)> = (0..=120).step_by(3).map(|f| (f, 10.0 * (f as f32 / 60.0 * std::f32::consts::TAU).sin())).collect();
    motion(&mut p, floor, "Spin", 0, 120, false, vec![channel("root", Property::RotY, &[(0, 0.0), (120, 180.0)]), channel("root", Property::RotX, &rock)]);
    person(&mut p, "dancer", Vec3::new(0.6, 0.0, 0.0), 90.0, &[]);
    p
}

/// A variant of Rotating Floor with a turntable: a flat cylinder spinning about its axis.
pub fn turntable() -> Project {
    let mut p = new_scene("Turntable", 2.0);
    p.physics.ground = None;
    let disc = Geometry::Cylinder { radius: 2.5, length: 0.2 };
    let flat = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
    let floor = add_prop(&mut p, "turntable", CharacterKind::Environment, scenery(&[(disc, flat, Vec3::new(0.0, -0.1, 0.0))]), SimLevel::CollisionOnly);
    motion(&mut p, floor, "Spin", 0, 120, false, vec![channel("root", Property::RotY, &[(0, 0.0), (120, 180.0)])]);
    person(&mut p, "dancer", Vec3::new(0.6, 0.0, 0.0), 90.0, &[]);
    p
}

/// Car Hit: a car (collision with momentum) drives through a standing character at
/// 15 m/s, bumper first.
pub fn car_hit() -> Project {
    let mut p = new_scene("Car Hit", 2.0);
    let body = Piece::at(Geometry::Box { size: [1.8, 0.8, 4.2] }, 1200.0, Quat::IDENTITY, Vec3::ZERO);
    let bumper = Piece::at(Geometry::Box { size: [1.8, 0.25, 0.2] }, 50.0, Quat::IDENTITY, Vec3::ZERO);
    let joints = vec![("body", None, Vec3::new(0.0, 0.9, 0.0), body), ("bumper", Some(0), Vec3::new(0.0, -0.4, 2.15), bumper)];
    let rig = prop_rig(joints, Limits { twist_min: -0.1, twist_max: 0.1, swing1: 0.1, swing2: 0.1 }, Gains { stiffness: 1e4, damping: 1e3, force_limit: 1e4 });
    let car = add_prop(&mut p, "car", CharacterKind::Prop, rig, SimLevel::CollisionWithMomentum);
    motion(&mut p, car, "Drive", 0, 120, true, vec![channel("body", Property::PosZ, &[(0, -6.0), (120, 24.0)])]);
    level(&mut p, car, 0, "CollisionOnlyWithInertia");
    person(&mut p, "pedestrian", Vec3::ZERO, 90.0, &[]);
    p
}

/// Prints, per frame and character, the lowest body, the most stretched joint and the
/// fastest body, for finding where a scene goes wrong.
pub fn trace(project: &Project) {
    let (sim, _) = simulate(project);
    let range = project.settings.range;
    for frame in range.start + 1..=range.end {
        let mut line = format!("frame {frame:>4}");
        for (ci, cf) in sim.frame(frame).unwrap().iter().enumerate() {
            let (Some(cf), Some(prev)) = (cf, &sim.frame(frame - 1).unwrap()[ci]) else { continue };
            let c = &project.characters[ci];
            let rig = c.physics.as_ref().unwrap();
            let low = cf.bodies.iter().enumerate().min_by(|a, b| a.1.1.y.total_cmp(&b.1.1.y)).unwrap();
            let fast = cf.bodies.iter().zip(&prev.bodies).enumerate().map(|(i, (a, b))| (i, a.1.distance(b.1) * project.settings.fps)).max_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
            let stretch = c
                .skeleton
                .joints
                .iter()
                .enumerate()
                .filter_map(|(i, j)| Some((i, (cf.joints[i].1.distance(cf.joints[j.parent?].1) - Vec3::from(j.offset).length()).abs())))
                .max_by(|a, b| a.1.total_cmp(&b.1));
            line += &format!(
                " | {} {:?} low {} {:.2} fast {} {:.1}",
                c.name,
                cf.level,
                rig.bodies[low.0].name,
                low.1.1.y,
                rig.bodies[fast.0].name,
                fast.1
            );
            if let Some((j, s)) = stretch.filter(|s| s.1 > 0.01) {
                line += &format!(" stretch {} {:.3}", c.skeleton.joints[j].name, s);
            }
        }
        println!("{line}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `NM_TRACE=<stand-in> cargo test --release trace_stand_in -- --ignored --nocapture`
    /// (or `NM_TRACE=<path to an .ens scene>`: its first two seconds)
    #[test]
    #[ignore = "debugging aid"]
    fn trace_stand_in() {
        // Runs with every `--ignored` sweep; only does something when asked to.
        let Ok(name) = std::env::var("NM_TRACE") else { return };
        let describe_only = name.starts_with("describe:");
        let name = name.trim_start_matches("describe:");
        let project = match name {
            "tackle" => tackle(),
            "balcony" => balcony(),
            "catch_rope" => catch_rope(),
            "gymnast_rings" => gymnast_rings(),
            "bendy_stick" => bendy_stick(),
            "heli_leap" => heli_leap(),
            "rotating_floor" => rotating_floor(),
            "turntable" => turntable(),
            "car_hit" => car_hit(),
            "balance" => balance_push(1.5, Vec3::Z, true),
            "catch_fall" => catch_fall_shove(Vec3::new(0.0, 0.0, 3.5), true),
            path if path.ends_with(".ens") => {
                let mut project = crate::io::endorphin::import_scene(std::path::Path::new(path)).unwrap().project;
                project.settings.range.end = project.settings.range.start + 2 * project.settings.fps as i32;
                project
            }
            _ => panic!("set NM_TRACE to a stand-in's name or an .ens scene"),
        };
        if describe_only {
            let report = sweep(&project);
            print!("{report}\n{}{}", report.places(), describe(&project));
            return;
        }
        trace(&project);
    }

    fn check(project: Project) -> Simulation {
        let (sim, elapsed) = simulate(&project);
        let report = measure(&project, &sim, elapsed);
        println!("{:<16} {:>4} frames in {:>6.0?}  {report}", project.settings.name, project.settings.range.len() + 1, report.elapsed);
        assert!(report.held(), "{}: {report}\n{}{}", project.settings.name, report.places(), describe(&project));
        sim
    }

    /// Lowest body of a character at the last frame.
    fn resting_height(project: &Project, sim: &Simulation, name: &str) -> f32 {
        let ci = project.characters.iter().position(|c| c.name == name).unwrap();
        let frame = sim.frame(project.settings.range.end).unwrap()[ci].as_ref().unwrap();
        frame.bodies.iter().map(|b| b.1.y).fold(f32::MAX, f32::min)
    }

    #[test]
    fn tackle_stays_together() {
        check(tackle());
    }

    #[test]
    fn balcony_crowd_is_stable() {
        check(balcony());
    }

    #[test]
    fn caught_rope_holds() {
        check(catch_rope());
    }

    #[test]
    fn gymnast_hangs_from_rings() {
        check(gymnast_rings());
    }

    #[test]
    fn stick_stays_in_hand() {
        check(bendy_stick());
    }

    #[test]
    fn helicopter_stays_up() {
        check(heli_leap());
    }

    #[test]
    fn rotating_floor_carries_character() {
        check(rotating_floor());
    }

    #[test]
    fn turntable_carries_character() {
        let project = turntable();
        let sim = check(project.clone());
        // Lying or standing on the disc, not floating above it.
        let height = resting_height(&project, &sim, "dancer");
        assert!(height < 0.1, "dancer rests {height} m above the turntable");
    }

    /// A rope hanging straight down from a Lock with a heavy weight on the end stays
    /// still (with full joint warm starts it breathed until it exploded).
    #[test]
    fn hanging_weight_stays_still() {
        for (tip_mass, seg_mass) in [(20.0, 0.15), (80.0, 0.15), (20.0, 0.05)] {
            let mut p = new_scene("Hanging weight", 2.0);
            let limits = Limits { twist_min: -0.5, twist_max: 0.5, swing1: 1.2, swing2: 1.2 };
            let tip = Piece::at(Geometry::Sphere { radius: 0.15 }, tip_mass, Quat::IDENTITY, Vec3::new(0.0, -0.15, 0.0));
            let rig = chain(Vec3::new(0.0, 4.0, 0.0), 8, 0.3, 0.02, seg_mass, Some(tip), limits, Gains { stiffness: 0.5, damping: 0.05, force_limit: 5.0 });
            let rope = add_prop(&mut p, "rope", CharacterKind::Prop, rig, SimLevel::Simulated);
            constrain(&mut p, rope, 0, 120, "Lock", &["rope.MetaBodies.seg0"], (STRONG, Vec3::ZERO));
            let report = sweep(&p);
            println!("weight {tip_mass} kg on {seg_mass} kg links: {report}");
            assert!(report.held() && report.fastest < 2.0, "{report}");
        }
    }

    /// LockWithParent naming one body holds it to its parent body: a rope locked at the
    /// top, with its second segment locked to the first, keeps that bend.
    #[test]
    fn lock_with_parent_alone_holds_to_the_parent_body() {
        let mut p = new_scene("Lock with parent", 1.0);
        let limits = Limits { twist_min: -0.5, twist_max: 0.5, swing1: 1.2, swing2: 1.2 };
        let rig = chain(Vec3::new(0.0, 3.0, 0.0), 4, 0.3, 0.02, 0.5, None, limits, Gains { stiffness: 0.5, damping: 0.05, force_limit: 5.0 });
        let rope = add_prop(&mut p, "rope", CharacterKind::Prop, rig, SimLevel::Simulated);
        // Bent sideways, so without the lock the second segment would swing down.
        motion(&mut p, rope, "Start pose", 0, 60, false, vec![held("seg1", Property::RotZ, 60.0)]);
        constrain(&mut p, rope, 0, 60, "Lock", &["rope.MetaBodies.seg0"], (STRONG, STRONG));
        constrain(&mut p, rope, 0, 60, "LockWithParent", &["rope.MetaBodies.seg1"], (STRONG, STRONG));
        assert_eq!(lock_spans(&p).len(), 2);
        check(p);
    }

    /// An animation cut moves a keyed character 1 m in one frame. The stick joined to its
    /// hand catches up (lock corrections are capped at 3 m/s) instead of being flung.
    #[test]
    fn animation_cut_does_not_fling_what_it_holds() {
        let mut p = bendy_stick();
        let holder = p.characters.iter().find(|c| c.name == "holder").unwrap().id;
        let mut cut = channel("hips", Property::PosX, &[(0, 0.0), (60, 1.0)]);
        cut.keys[0].interp = Interp::Step;
        motion(&mut p, holder, "Cut", 0, 120, true, vec![cut]);
        let report = sweep(&p);
        println!("Cut              {report}");
        assert!(!report.nan && report.fastest < 40.0 && report.stretch < 0.1, "{report}");
    }

    /// endorphin's Car Hit stretched a joint by 6 cm. The stand-in's character takes the
    /// hit within limits, so this guards against regressions rather than reproducing it.
    #[test]
    fn car_hit_keeps_joints_together() {
        check(car_hit());
    }
}

#[cfg(test)]
mod behaviour_tests {
    use super::*;

    /// Lowest height of the named bodies' centres over a frame range.
    fn lowest(project: &Project, sim: &Simulation, bodies: &[&str], frames: std::ops::RangeInclusive<i32>) -> f32 {
        let rig = project.characters[0].physics.as_ref().unwrap();
        let mut low = f32::MAX;
        for frame in frames {
            let cf = sim.frame(frame).unwrap()[0].as_ref().unwrap();
            for name in bodies {
                low = low.min(cf.bodies[rig.body_index(name).unwrap()].1.y);
            }
        }
        low
    }

    #[test]
    fn balance_keeps_the_placeholder_upright_after_a_push() {
        // Backwards, the capture point leaves the placeholder's short heel (6 cm behind
        // the ankle): that takes a step, which Balance doesn't do yet.
        for (behaviour, dir) in [(true, Vec3::Z), (true, Vec3::X), (true, -Vec3::X), (false, Vec3::Z)] {
            let project = balance_push(1.5, dir, behaviour);
            let (sim, _) = simulate(&project);
            // Standing, the head's centre is at 1.7 m and the hips' at 1.0 m; check the
            // 3 s after the push.
            let head = lowest(&project, &sim, &["head"], 60..=240);
            let hips = lowest(&project, &sim, &["hips"], 60..=240);
            println!("balance {behaviour}, push 1.5 m/s along {dir}: head down to {head:.2} m, hips {hips:.2} m");
            if behaviour {
                assert!(head > 1.45 && hips > 0.8, "fell pushed along {dir}: head {head}, hips {hips}");
            } else {
                assert!(head < 1.0, "limp character should fall: head {head}");
            }
        }
    }

    /// The first frame each part comes within 4 cm of the ground (`None`: never).
    fn first_touch(project: &Project, sim: &Simulation, parts: &[(&str, f32)]) -> Option<i32> {
        let rig = project.characters[0].physics.as_ref().unwrap();
        let range = project.settings.range;
        (range.start..=range.end).find(|&frame| {
            let cf = sim.frame(frame).unwrap()[0].as_ref().unwrap();
            parts.iter().any(|&(name, radius)| cf.bodies[rig.body_index(name).unwrap()].1.y < radius + 0.04)
        })
    }

    #[test]
    fn catch_fall_lands_on_the_hands_first() {
        let shoves = [Vec3::new(0.0, 0.0, 3.5), Vec3::new(0.0, 0.0, 5.0), Vec3::new(2.5, 0.0, 2.5), Vec3::new(3.5, 0.0, 0.0), Vec3::new(0.0, 0.0, -3.5)];
        let cases = shoves.iter().map(|&s| (s, true)).chain([(shoves[0], false)]);
        for (shove, behaviour) in cases {
            let project = catch_fall_shove(shove, behaviour);
            let (sim, _) = simulate(&project);
            let hands = first_touch(&project, &sim, &[("l_wrist", 0.035), ("r_wrist", 0.035)]);
            let head = first_touch(&project, &sim, &[("head", 0.1)]);
            let chest = first_touch(&project, &sim, &[("chest", 0.12)]);
            println!("catch fall {behaviour}, shoved {shove}: hands {hands:?}, head {head:?}, chest {chest:?}");
            let first = |a: Option<i32>, b: Option<i32>| a.is_some_and(|a| b.is_none_or(|b| a < b));
            let hands_first = first(hands, head) && first(hands, chest);
            if behaviour {
                assert!(hands_first, "shoved {shove}: hands {hands:?}, head {head:?}, chest {chest:?}");
            } else {
                // Shoved forwards, a limp character lands on its face.
                assert!(!hands_first, "a limp fall shouldn't land on the hands first");
            }
        }
    }
}

/// Simulates every sample scene shipped with a local endorphin install and checks the
/// results stay physical. Run with `cargo test --release -- --ignored --nocapture`; set
/// `ENDORPHIN_DIR` if endorphin isn't installed in the default location.
#[cfg(test)]
mod endorphin_scenes {
    use super::*;

    #[test]
    #[ignore = "needs a local endorphin 2.5 install"]
    fn sample_scenes_simulate_stably() {
        let root = std::env::var("ENDORPHIN_DIR")
            .unwrap_or_else(|_| r"C:\Program Files (x86)\NaturalMotion\endorphin 2.5.2".to_owned());
        let dir = std::path::Path::new(&root).join("Resources/Scenes/Sample Scenes");
        let mut failures = Vec::new();
        let mut paths: Vec<_> = std::fs::read_dir(&dir).unwrap().flatten().map(|e| e.path()).collect();
        paths.sort();
        for path in paths {
            let name = path.file_stem().unwrap().to_string_lossy().into_owned();
            if name.contains("Large File") {
                continue;
            }
            let mut project = crate::io::endorphin::import_scene(&path).unwrap().project;
            // Keep the check quick: the first 2 seconds.
            let fps = project.settings.fps as i32;
            project.settings.range.end = project.settings.range.start + 2 * fps;
            let report = sweep(&project);
            let ok = report.ok();
            println!(
                "{} {name:<16} {:>4} frames in {:>6.0?}  {report}",
                if ok { "ok  " } else { "FAIL" },
                project.settings.range.len() + 1,
                report.elapsed
            );
            // What to look at: where the worst values happened and the set-up behind them.
            let constrained = project.clips.iter().any(|c| matches!(&c.kind, ClipKind::Event { event } if event == "Constraint"));
            if !ok || constrained || report.drift >= 0.05 || report.buried > 0.0 {
                print!("{}{}", report.places(), describe(&project));
            }
            if !ok {
                failures.push(name);
            }
        }
        assert!(failures.is_empty(), "{failures:?}");
    }
}
