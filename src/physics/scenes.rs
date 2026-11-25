//! Stand-ins for endorphin sample scenes that didn't simulate cleanly, built in code
//! (endorphin's files only exist on machines with endorphin installed), and the sweep
//! that checks them and, with `--ignored`, the real scenes.
//!
//! Each stand-in reproduces the pattern behind one failure with the placeholder
//! character and simple props, using the same event settings (`Clip::source` keys) the
//! endorphin importer produces. The sweep applies the same checks to both:
//!
//! - no non-finite values;
//! - no simulated body more than 10 cm below the ground (fixed scenery, and bodies
//!   without collision shapes, aside: those fall through everything, in endorphin too);
//!   in a scene without a ground plane, where things may fall as far as they like, no
//!   body more than 10 cm under the underside of fixed scenery it is beneath (fallen
//!   through the floor);
//! - no joint stretched by 5 cm or more (free props have no joint to stretch, and
//!   joints a sever event cuts none from the frame they are cut);
//! - no simulated body faster than 40 m/s;
//! - (stand-ins) no locked body drifting 5 cm or more from where its lock holds it;
//! - (stand-ins) no body centre inside solid scenery.

use std::time::{Duration, Instant};

use glam::{Quat, Vec3};

use super::sim::{Simulation, cut_joints, lock_spans, rig_bodies};
use crate::document::physics::{
    BodyDef, ContactModel, Frame, Gains, Geometry, JointDef, Limits, MuscleGains, MuscleSet,
    PhysicsRig, ShapeDef, SpringCap, VirtualMuscleDef,
};
use crate::document::sample::{humanoid_rig, humanoid_skeleton};
use crate::document::{
    Channel, CharacterId, CharacterKind, ClipKind, ClipProps, FrameRange, Interp, Joint, Keyframe,
    Pose, Project, ProjectSettings, Property, SimLevel, SkeletonRef, world_transforms,
};

/// What the sweep measured over a simulated range.
#[derive(Debug)]
pub struct Report {
    pub nan: bool,
    /// Lowest body centre above the ground plane, m (its height, when the scene has no
    /// ground plane).
    pub lowest: f32,
    /// Whether the scene has a ground plane to measure `lowest` against.
    pub ground: bool,
    /// Deepest a body centre got under the underside of fixed scenery it is beneath, m,
    /// in a scene without a ground plane (0 with one).
    pub under: f32,
    /// Largest change in a skeleton bone's length, m.
    pub stretch: f32,
    /// Fastest body while simulated, m/s.
    pub fastest: f32,
    /// Largest distance a locked body moved away from where its lock holds it, m
    /// (along the axes the lock holds).
    pub drift: f32,
    /// Deepest a character's or prop's body centre got inside fixed scenery, m.
    pub buried: f32,
    /// Where each worst value happened ("character.body @ frame"): lowest, stretch,
    /// fastest, drift, buried, under.
    pub places: [String; 6],
    pub elapsed: Duration,
}

impl Report {
    /// The endorphin sweep's checks.
    pub fn ok(&self) -> bool {
        !self.nan
            && (!self.ground || self.lowest > -0.1)
            && self.under < 0.1
            && self.stretch < 0.05
            && self.fastest < 40.0
    }

    /// The sweep's checks, plus locks holding and nothing sinking into scenery.
    pub fn held(&self) -> bool {
        self.ok() && self.drift < 0.05 && self.buried <= 0.0
    }
}

impl Report {
    /// Where the worst values happened, one per line.
    pub fn places(&self) -> String {
        let names = [
            "lowest body",
            "max joint stretch",
            "fastest",
            "lock drift",
            "buried",
            "under scenery",
        ];
        names
            .iter()
            .zip(&self.places)
            .filter(|(_, p)| !p.is_empty())
            .map(|(n, p)| format!("    {n:<18} {p}\n"))
            .collect()
    }
}

impl std::fmt::Display for Report {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "lowest body {:>6.3} m{}  under scenery {:.3} m  max joint stretch {:.4} m  fastest {:>5.1} m/s  lock drift {:.3} m  buried {:.3} m{}",
            self.lowest,
            if self.ground { "" } else { " (no ground)" },
            self.under,
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
    measure_frames(project, sim, elapsed, project.settings.range)
}

/// [`measure`] over part of the simulated range only.
pub fn measure_frames(
    project: &Project,
    sim: &Simulation,
    elapsed: Duration,
    range: FrameRange,
) -> Report {
    let fps = project.settings.fps;
    let ground = project.physics.ground.as_ref().map(|g| g.height);
    let cut = cut_joints(project);
    let mut report = Report {
        nan: false,
        lowest: f32::MAX,
        ground: ground.is_some(),
        under: 0.0,
        stretch: 0.0,
        fastest: 0.0,
        drift: 0.0,
        buried: 0.0,
        places: Default::default(),
        elapsed,
    };
    let body_name = |ci: usize, bi: usize| {
        let c = &project.characters[ci];
        format!(
            "{}.{}",
            c.name,
            c.physics
                .as_ref()
                .map_or("?", |r| r.bodies[bi].name.as_str())
        )
    };
    for frame in range.start..=range.end {
        let frame_bodies = sim.frame(frame).unwrap();
        // Fixed scenery: the environment's massless pieces, or all of it while it isn't
        // simulated. Its props with mass are simulated like any character's bodies.
        let fixed = |ci: usize, body: &BodyDef| {
            let c = &project.characters[ci];
            c.kind == CharacterKind::Environment
                && (body.mass <= 0.0
                    || frame_bodies[ci]
                        .as_ref()
                        .is_some_and(|cf| cf.level != SimLevel::Simulated))
        };
        // Fixed scenery shapes where they are this frame.
        let mut scenery = Vec::new();
        for (ci, cf) in frame_bodies.iter().enumerate() {
            let (Some(cf), Some(rig)) = (cf, &project.characters[ci].physics) else {
                continue;
            };
            for (body, &(r, p)) in rig
                .bodies
                .iter()
                .zip(&cf.bodies)
                .filter(|(b, _)| fixed(ci, b))
            {
                for shape in &body.shapes {
                    let (sr, sp) = (
                        Quat::from_array(shape.frame.rotation),
                        Vec3::from(shape.frame.position),
                    );
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
                // Only the solver's doing counts: keyed bodies sit where their animation puts
                // them (a hidden carrier underground, say), fixed scenery may well sit below
                // the ground, and a body without collision shapes falls through it.
                let body = rig.map(|r| &r.bodies[bi]);
                if cf.level == SimLevel::Simulated
                    && body.is_none_or(|b| !fixed(ci, b) && !b.shapes.is_empty())
                {
                    let height = p.y - ground.unwrap_or(0.0);
                    if height < report.lowest {
                        report.lowest = height;
                        report.places[0] = format!("{} @ frame {frame}", body_name(ci, bi));
                    }
                    for &(geometry, r, at) in &scenery {
                        let depth = depth_inside(geometry, r, at, *p);
                        if depth > report.buried {
                            report.buried = depth;
                            report.places[4] = format!("{} @ frame {frame}", body_name(ci, bi));
                        }
                        // With a ground, things beneath scenery (a balcony, a beam) are
                        // usually meant to be there; without one, they fell through it.
                        let under = match ground {
                            Some(_) => 0.0,
                            None => depth_under(geometry, r, at, *p),
                        };
                        if under > report.under {
                            report.under = under;
                            report.places[5] = format!("{} @ frame {frame}", body_name(ci, bi));
                        }
                    }
                }
            }
            // A body that no physics joint holds (a prop) moves freely of its parent.
            let free = |joint: usize| {
                rig.is_some_and(|r| {
                    r.bodies
                        .iter()
                        .enumerate()
                        .any(|(bi, b)| b.joint == joint && !r.joints.iter().any(|jd| jd.body == bi))
                })
            };
            let severed = |joint: usize| {
                cut.iter()
                    .any(|&(c, j, start)| (c, j) == (ci, joint) && start <= frame)
            };
            for (i, j) in skeleton.joints.iter().enumerate() {
                if let Some(p) = j.parent.filter(|_| !free(i) && !severed(i)) {
                    let stretch = bone_stretch(j, &cf.joints, i, p);
                    if stretch > report.stretch {
                        report.stretch = stretch;
                        report.places[1] = format!(
                            "{}.{} @ frame {frame} ({:?})",
                            character.name, j.name, cf.level
                        );
                    }
                }
            }
            // Speed is only meaningful while simulated: kinematic bodies jump back onto
            // the animation when a transition ends.
            let prev =
                (frame > range.start).then(|| sim.frame(frame - 1).unwrap()[ci].as_ref().unwrap());
            if let Some(prev) =
                prev.filter(|p| p.level == SimLevel::Simulated && cf.level == SimLevel::Simulated)
            {
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
    let pose = |frame: i32, (ci, bi): (usize, usize)| {
        sim.frame(frame).unwrap()[ci].as_ref().unwrap().bodies[bi]
    };
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
                let anchor = lock
                    .anchor
                    .map_or("the world".to_owned(), |(ci, bi)| body_name(ci, bi));
                report.places[3] = format!(
                    "{} to {anchor} @ frame {frame}",
                    body_name(lock.body.0, lock.body.1)
                );
            }
        }
    }
    report
}

/// How far joint `i` has come away from its parent `parent`: the change in the length
/// of its bone, m. `at` is every joint's world transform.
fn bone_stretch(joint: &Joint, at: &[(Quat, Vec3)], i: usize, parent: usize) -> f32 {
    (at[i].1.distance(at[parent].1) - Vec3::from(joint.offset).length()).abs()
}

/// The largest stretch over `frames` of the named character's joints that `counts`
/// picks (by joint name), m, and where it happened.
pub fn joint_stretch(
    project: &Project,
    sim: &Simulation,
    character: &str,
    frames: std::ops::RangeInclusive<i32>,
    counts: impl Fn(&str) -> bool,
) -> (f32, String) {
    let ci = project
        .characters
        .iter()
        .position(|c| c.name == character)
        .unwrap();
    let joints = &project.characters[ci].skeleton.joints;
    let mut worst = (0.0, String::new());
    for frame in frames {
        let cf = sim.frame(frame).unwrap()[ci].as_ref().unwrap();
        for (i, j) in joints.iter().enumerate() {
            if let Some(p) = j.parent.filter(|_| counts(&j.name)) {
                let stretch = bone_stretch(j, &cf.joints, i, p);
                if stretch > worst.0 {
                    worst = (stretch, format!("{character}.{} @ frame {frame}", j.name));
                }
            }
        }
    }
    worst
}

/// The physics set-up of a project, for reading alongside a failing sweep: ground,
/// rigs, level switches and constraint events as imported.
pub fn describe(project: &Project) -> String {
    let mut out = String::new();
    let ground = project
        .physics
        .ground
        .as_ref()
        .map_or("none".to_owned(), |g| {
            format!("y = {} ({})", g.height, g.material)
        });
    out += &format!(
        "    ground {ground}, {} Hz, {} fps\n",
        project.physics.rate, project.settings.fps
    );
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
        let lightest = rig
            .bodies
            .iter()
            .map(|b| (b.mass, b.name.as_str()))
            .min_by(|a, b| a.0.total_cmp(&b.0));
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
        let settings: Vec<String> = clip
            .source
            .iter()
            .filter(|(k, _)| k.as_str() != "targets")
            .map(|(k, v)| format!("{k}={v}"))
            .collect();
        let targets = clip.source.get("targets").map_or("", String::as_str);
        let masses: Vec<String> = targets
            .split(',')
            .map(|t| {
                let mut parts = t.trim().split(".MetaBodies.");
                let (c, b) = (parts.next().unwrap_or(""), parts.next().unwrap_or(""));
                let rig = project
                    .character_by_name(c)
                    .and_then(|c| c.physics.as_ref());
                match rig.and_then(|r| Some((r.bodies.get(r.body_index(b)?)?.mass, r.total_mass())))
                {
                    Some((m, total)) => format!("{m:.2} kg of {total:.0} kg"),
                    None => "UNRESOLVED".to_owned(),
                }
            })
            .collect();
        out += &format!(
            "    constraint {}-{}{}: targets [{targets}] ({}), {}\n",
            clip.props.start,
            clip.props.end(),
            if clip.props.enabled {
                ""
            } else {
                " (disabled)"
            },
            masses.join(", "),
            settings.join(" ")
        );
    }
    out
}

/// How far `point` is inside a placed shape (negative outside).
fn depth_inside(geometry: Geometry, rotation: Quat, position: Vec3, point: Vec3) -> f32 {
    let local = rotation.inverse() * (point - position);
    let along_z = |radius: f32, half: f32| {
        radius - Vec3::new(local.x, local.y, local.z - local.z.clamp(-half, half)).length()
    };
    match geometry {
        Geometry::Sphere { radius } => radius - local.length(),
        Geometry::Capsule { radius, length } => along_z(radius, length * 0.5),
        Geometry::Cylinder { radius, length } => {
            (radius - local.truncate().length()).min(length * 0.5 - local.z.abs())
        }
        Geometry::Box { size } => (Vec3::from(size) * 0.5 - local.abs()).min_element(),
    }
}

/// How far `point` is under the underside of a placed shape, within its outline seen
/// from above (0 when it isn't beneath it). "Up" is whichever of the shape's own axes is
/// closest to the world's, so a tilting floor keeps its top and underside.
fn depth_under(geometry: Geometry, rotation: Quat, position: Vec3, point: Vec3) -> f32 {
    let half = match geometry {
        Geometry::Sphere { radius } => Vec3::splat(radius),
        Geometry::Capsule { radius, length } => Vec3::new(radius, radius, radius + length * 0.5),
        Geometry::Cylinder { radius, length } => Vec3::new(radius, radius, length * 0.5),
        Geometry::Box { size } => Vec3::from(size) * 0.5,
    };
    let local = rotation.inverse() * (point - position);
    let up = rotation.inverse() * Vec3::Y;
    let axis = (0..3)
        .max_by(|&a, &b| up[a].abs().total_cmp(&up[b].abs()))
        .unwrap();
    let beside = (0..3).any(|i| i != axis && local[i].abs() > half[i]);
    if beside {
        return 0.0;
    }
    (-half[axis] - local[axis] * up[axis].signum()).max(0.0)
}

// ---------------------------------------------------------------------------------
// Building scenes

fn new_scene(name: &str, seconds: f32) -> Project {
    let fps = 60.0;
    Project::new(ProjectSettings {
        name: name.to_owned(),
        fps,
        range: FrameRange {
            start: 0,
            end: (seconds * fps) as i32,
        },
    })
}

fn props(name: &str, track: crate::document::TrackId, start: i32, duration: i32) -> ClipProps {
    ClipProps {
        name: name.to_owned(),
        track,
        start,
        duration,
        blend_in: 0,
        blend_out: 0,
        weight: 1.0,
        enabled: true,
    }
}

fn channel(joint: &str, property: Property, keys: &[(i32, f32)]) -> Channel {
    Channel {
        joint: joint.to_owned(),
        property,
        keys: keys
            .iter()
            .map(|&(frame, value)| Keyframe {
                frame,
                value,
                interp: Interp::Linear,
            })
            .collect(),
    }
}

/// A channel holding one value.
fn held(joint: &str, property: Property, value: f32) -> Channel {
    channel(joint, property, &[(0, value)])
}

/// Adds a motion clip on its own track. Inactive clips only place the character (like
/// an imported start pose); active ones are also muscle targets once it's simulated.
fn motion(
    project: &mut Project,
    character: CharacterId,
    name: &str,
    start: i32,
    duration: i32,
    active: bool,
    channels: Vec<Channel>,
) {
    let track = project.add_track(character, name);
    let id = project.add_clip(
        props(name, track, start, duration),
        ClipKind::Motion,
        channels,
    );
    if !active {
        project
            .clip_mut(id)
            .unwrap()
            .source
            .insert("active".into(), "false".into());
    }
}

/// Adds an event clip with imported-style settings.
fn event(
    project: &mut Project,
    character: CharacterId,
    kind: &str,
    start: i32,
    duration: i32,
    settings: &[(&str, &str)],
) {
    let track = project.add_track(character, kind);
    let id = project.add_clip(
        props(kind, track, start, duration),
        ClipKind::Event { event: kind.into() },
        vec![],
    );
    project.clip_mut(id).unwrap().source = settings
        .iter()
        .map(|(k, v)| ((*k).to_owned(), (*v).to_owned()))
        .collect();
}

fn level(project: &mut Project, character: CharacterId, frame: i32, level: &str) {
    event(
        project,
        character,
        "Simulation",
        frame,
        1,
        &[("physicsLevel", level), ("momentumScale", "1")],
    );
}

fn push(project: &mut Project, character: CharacterId, frame: i32, target: &str, dv: Vec3) {
    let dv = format!("{}, {}, {}", dv.x, dv.y, dv.z);
    event(
        project,
        character,
        "Force",
        frame,
        1,
        &[("matrix.Vi", &dv), ("targets", target)],
    );
}

/// A lock or join constraint event; strengths are (position, angular).
fn constrain(
    project: &mut Project,
    on: CharacterId,
    start: i32,
    end: i32,
    lock_type: &str,
    targets: &[&str],
    strength: (Vec3, Vec3),
) {
    let v = |v: Vec3| format!("{}, {}, {}", v.x, v.y, v.z);
    let (linear, angular) = (v(strength.0), v(strength.1));
    let targets = targets.join(", ");
    let settings = [
        ("lockType", lock_type),
        ("targets", targets.as_str()),
        ("positionStrength", &linear),
        ("angularStrength", &angular),
    ];
    event(project, on, "Constraint", start, end - start, &settings);
}

const STRONG: Vec3 = Vec3::splat(1e5);

/// The placeholder character with endorphin-like light extremities: fingers on each
/// hand and toes on each foot, on tight hinges (endorphin's character has 28 bodies).
fn person_rig() -> (SkeletonRef, PhysicsRig) {
    let mut skeleton = humanoid_skeleton();
    for side in ["l", "r"] {
        let wrist = skeleton.joint_index(&format!("{side}_wrist")).unwrap();
        skeleton.joints.push(Joint::new(
            &format!("{side}_fingers"),
            Some(wrist),
            [0.0, -0.09, 0.0],
        ));
    }
    let mut rig = humanoid_rig(&skeleton);
    for side in ["l", "r"] {
        let extras = [
            (
                "fingers",
                "wrist",
                Piece::along(Vec3::new(0.0, -0.07, 0.0), 0.012, 0.06),
                0.8,
            ),
            (
                "toe",
                "ankle",
                Piece::at(
                    Geometry::Box {
                        size: [0.09, 0.03, 0.08],
                    },
                    0.12,
                    Quat::IDENTITY,
                    Vec3::new(0.0, -0.005, 0.04),
                ),
                0.5,
            ),
        ];
        for (joint, parent, piece, bend) in extras {
            let name = format!("{side}_{joint}");
            let j = skeleton.joint_index(&name).unwrap();
            let parent_body = rig.body_index(&format!("{side}_{parent}")).unwrap();
            rig.bodies.push(BodyDef {
                name: name.clone(),
                joint: j,
                frame: Frame {
                    rotation: piece.frame.0.to_array(),
                    position: piece.frame.1.to_array(),
                },
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
            let hard = Limits {
                twist_min: -bend,
                twist_max: bend,
                swing1: 0.02,
                swing2: 0.02,
            };
            let soft = Limits {
                twist_min: -bend + 0.05,
                twist_max: bend - 0.05,
                swing1: 0.01,
                swing2: 0.01,
            };
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

/// `person_rig` with a clavicle between the chest and each upper arm, like endorphin's
/// characters: a light body (0.4 kg) on a stiff ball joint at the sternum.
fn person_rig_with_clavicles() -> (SkeletonRef, PhysicsRig) {
    let (mut skeleton, mut rig) = person_rig();
    for (side, x) in [("l", 1.0f32), ("r", -1.0)] {
        let chest = skeleton.joint_index("chest").unwrap();
        let shoulder = skeleton.joint_index(&format!("{side}_shoulder")).unwrap();
        // Insert the clavicle joint just before the shoulder, so parents still come first.
        let at = shoulder;
        let bump = |i: usize| if i >= at { i + 1 } else { i };
        for j in &mut skeleton.joints {
            j.parent = j.parent.map(bump);
        }
        for b in &mut rig.bodies {
            b.joint = bump(b.joint);
        }
        for j in &mut rig.joints {
            j.joint = bump(j.joint);
        }
        let shoulder = shoulder + 1;
        let to_shoulder = Vec3::from(skeleton.joints[shoulder - 1].offset);
        let root = Vec3::new(0.04 * x, to_shoulder.y - 0.02, 0.0);
        skeleton.joints.insert(
            at,
            Joint::new(&format!("{side}_clavicle"), Some(chest), root.to_array()),
        );
        skeleton.joints[shoulder].parent = Some(at);
        skeleton.joints[shoulder].offset = (to_shoulder - root).to_array();

        let piece = Piece::along(to_shoulder - root, 0.03, 0.4);
        let chest_body = rig.body_index("chest").unwrap();
        rig.bodies.push(BodyDef {
            name: format!("{side}_clavicle"),
            joint: at,
            frame: Frame {
                rotation: piece.frame.0.to_array(),
                position: piece.frame.1.to_array(),
            },
            mass: piece.mass,
            inertia: piece.geometry.inertia(piece.mass),
            shapes: vec![ShapeDef {
                geometry: piece.geometry,
                frame: Frame::IDENTITY,
                material: "skin".into(),
                collides_with_parent: false,
                collides_with_siblings: false,
                collides_with_ancestors: false,
            }],
        });
        let clavicle_body = rig.bodies.len() - 1;
        let shoulder_body = rig.body_index(&format!("{side}_shoulder")).unwrap();
        let twist = Quat::from_rotation_arc(Vec3::X, (to_shoulder - root).normalize()).to_array();
        let hard = Limits {
            twist_min: -0.2,
            twist_max: 0.2,
            swing1: 0.3,
            swing2: 0.3,
        };
        let soft = Limits {
            twist_min: -0.15,
            twist_max: 0.15,
            swing1: 0.25,
            swing2: 0.25,
        };
        let shoulder_joint = rig
            .joints
            .iter_mut()
            .find(|j| j.body == shoulder_body)
            .unwrap();
        shoulder_joint.parent_body = clavicle_body;
        rig.joints.push(JointDef {
            name: format!("{side}_clavicle"),
            joint: at,
            body: clavicle_body,
            parent_body: chest_body,
            primary: twist,
            secondary: twist,
            hard,
            soft,
            soft_stiffness: 10.0,
            soft_damping: 10.0,
            muscle: format!("{side}_clavicle"),
        });
    }
    (skeleton, rig)
}

/// `person_rig_with_clavicles` with each forearm split in two like endorphin's
/// characters: the forearm to half way, then a forearm-twist body that only turns
/// about the bone, then the hand. With `t_pose`, it rests with its arms out to the
/// sides, as endorphin's characters do.
pub(crate) fn person_rig_like_endorphins(t_pose: bool) -> (SkeletonRef, PhysicsRig) {
    let (mut skeleton, mut rig) = person_rig_with_clavicles();
    for (side, x) in [("l", 1.0f32), ("r", -1.0)] {
        let wrist = skeleton.joint_index(&format!("{side}_wrist")).unwrap();
        let elbow = skeleton.joints[wrist].parent.unwrap();
        // Insert the twist joint just before the wrist, so parents still come first.
        let at = wrist;
        let bump = |i: usize| if i >= at { i + 1 } else { i };
        for j in &mut skeleton.joints {
            j.parent = j.parent.map(bump);
        }
        for b in &mut rig.bodies {
            b.joint = bump(b.joint);
        }
        for j in &mut rig.joints {
            j.joint = bump(j.joint);
        }
        let wrist = wrist + 1;
        let half = Vec3::from(skeleton.joints[wrist - 1].offset) * 0.5;
        skeleton.joints.insert(
            at,
            Joint::new(
                &format!("{side}_forearm_twist"),
                Some(elbow),
                half.to_array(),
            ),
        );
        skeleton.joints[wrist].parent = Some(at);
        skeleton.joints[wrist].offset = half.to_array();

        // The forearm keeps its first half; the twist body takes the rest of its mass.
        let forearm = rig.body_index(&format!("{side}_elbow")).unwrap();
        let first = Piece::along(half, 0.038, 0.8);
        let body = &mut rig.bodies[forearm];
        body.frame = Frame {
            rotation: first.frame.0.to_array(),
            position: first.frame.1.to_array(),
        };
        body.mass = first.mass;
        body.inertia = first.geometry.inertia(first.mass);
        body.shapes[0].geometry = first.geometry;
        let piece = Piece::along(half, 0.035, 0.6);
        rig.bodies.push(BodyDef {
            name: format!("{side}_forearm_twist"),
            joint: at,
            frame: Frame {
                rotation: piece.frame.0.to_array(),
                position: piece.frame.1.to_array(),
            },
            mass: piece.mass,
            inertia: piece.geometry.inertia(piece.mass),
            shapes: vec![ShapeDef {
                geometry: piece.geometry,
                frame: Frame::IDENTITY,
                material: "skin".into(),
                collides_with_parent: false,
                collides_with_siblings: true,
                collides_with_ancestors: false,
            }],
        });
        let twist_body = rig.bodies.len() - 1;
        let hand = rig.body_index(&format!("{side}_wrist")).unwrap();
        rig.joints
            .iter_mut()
            .find(|j| j.body == hand)
            .unwrap()
            .parent_body = twist_body;
        // Twist about the bone only, like endorphin's forearm twist joints.
        let along = Quat::from_rotation_arc(Vec3::X, half.normalize()).to_array();
        let limits = |twist: f32, swing: f32| Limits {
            twist_min: -twist,
            twist_max: twist,
            swing1: swing,
            swing2: swing,
        };
        rig.joints.push(JointDef {
            name: format!("{side}_forearm_twist"),
            joint: at,
            body: twist_body,
            parent_body: forearm,
            primary: along,
            secondary: along,
            hard: limits(1.6, 0.01),
            soft: limits(1.55, 0.005),
            soft_stiffness: 10.0,
            soft_damping: 1.0,
            muscle: format!("{side}_elbow"),
        });
        if t_pose {
            let shoulder = skeleton.joint_index(&format!("{side}_shoulder")).unwrap();
            skeleton.joints[shoulder].rest_rotation =
                Quat::from_rotation_z(x * std::f32::consts::FRAC_PI_2).to_array();
        }
    }
    (skeleton, rig)
}

/// Adds the stand-in character with its feet at `at`, facing `yaw` degrees from +Z,
/// held in a start pose (joint, property, value) for the whole scene.
fn person(
    project: &mut Project,
    name: &str,
    at: Vec3,
    yaw: f32,
    pose: &[(&str, Property, f32)],
) -> CharacterId {
    add_person(project, name, person_rig(), at, yaw, pose)
}

/// Adds the placeholder character itself, standing at `at`.
fn placeholder(project: &mut Project, name: &str, at: Vec3, yaw: f32) -> CharacterId {
    let skeleton = humanoid_skeleton();
    let rig = humanoid_rig(&skeleton);
    add_person(project, name, (skeleton, rig), at, yaw, &[])
}

fn add_person(
    project: &mut Project,
    name: &str,
    (skeleton, rig): (SkeletonRef, PhysicsRig),
    at: Vec3,
    yaw: f32,
    pose: &[(&str, Property, f32)],
) -> CharacterId {
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
    channels.extend(
        pose.iter()
            .map(|&(joint, property, value)| held(joint, property, value)),
    );
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
        Piece {
            geometry: Geometry::Capsule { radius, length },
            mass,
            frame: (rotation, bone * 0.5),
        }
    }

    fn at(geometry: Geometry, mass: f32, rotation: Quat, position: Vec3) -> Self {
        Piece {
            geometry,
            mass,
            frame: (rotation, position),
        }
    }
}

/// Builds a prop from joints (name, parent, offset, piece) with ball joints between
/// pieces. Every joint gets one body. Muscles hold the rest pose with `gains`.
fn prop_rig(
    joints: Vec<(&str, Option<usize>, Vec3, Piece)>,
    limits: Limits,
    gains: Gains,
) -> (SkeletonRef, PhysicsRig) {
    let skeleton = SkeletonRef {
        name: "Prop".into(),
        source: None,
        joints: joints
            .iter()
            .map(|(name, parent, offset, _)| Joint::new(name, *parent, offset.to_array()))
            .collect(),
    };
    let mut bodies = Vec::new();
    let mut rig_joints = Vec::new();
    for (i, (name, parent, _, piece)) in joints.into_iter().enumerate() {
        let frame = Frame {
            rotation: piece.frame.0.to_array(),
            position: piece.frame.1.to_array(),
        };
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
    let both = MuscleGains {
        twist: gains,
        swing: gains,
    };
    let set = MuscleSet {
        name: "Default".into(),
        muscles: Default::default(),
        fallback: both,
        linear: Default::default(),
    };
    (
        skeleton,
        PhysicsRig {
            bodies,
            joints: rig_joints,
            muscle_sets: vec![set],
            isotropic_inertia: true,
            virtual_muscles: Vec::new(),
        },
    )
}

fn add_prop(
    project: &mut Project,
    name: &str,
    kind: CharacterKind,
    (skeleton, rig): (SkeletonRef, PhysicsRig),
    level: SimLevel,
) -> CharacterId {
    let id = project.add_character(name, kind, skeleton);
    let c = project.characters.iter_mut().find(|c| c.id == id).unwrap();
    c.physics = Some(rig);
    c.default_level = level;
    id
}

/// A hanging chain of `count` capsules from `top`, each `length` long, ending in an
/// optional heavier tip piece.
fn chain(
    top: Vec3,
    count: usize,
    length: f32,
    radius: f32,
    mass: f32,
    tip: Option<Piece>,
    limits: Limits,
    gains: Gains,
) -> (SkeletonRef, PhysicsRig) {
    let down = Vec3::new(0.0, -length, 0.0);
    let mut joints: Vec<(&str, Option<usize>, Vec3, Piece)> = Vec::new();
    const NAMES: [&str; 12] = [
        "seg0", "seg1", "seg2", "seg3", "seg4", "seg5", "seg6", "seg7", "seg8", "seg9", "seg10",
        "seg11",
    ];
    for (i, name) in NAMES.iter().enumerate().take(count) {
        let (parent, offset) = if i == 0 {
            (None, top)
        } else {
            (Some(i - 1), down)
        };
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
    let skeleton = SkeletonRef {
        name: "Scenery".into(),
        source: None,
        joints: vec![Joint::new("root", None, [0.0; 3])],
    };
    let bodies = pieces
        .iter()
        .zip(NAMES)
        .map(|(&(geometry, rotation, position), name)| BodyDef {
            name: name.into(),
            joint: 0,
            frame: Frame {
                rotation: rotation.to_array(),
                position: position.to_array(),
            },
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
    (
        skeleton,
        PhysicsRig {
            bodies,
            joints: Vec::new(),
            muscle_sets: Vec::new(),
            isotropic_inertia: true,
            virtual_muscles: Vec::new(),
        },
    )
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
// Cylinder scenes (docs/physics-design.md, lesson 8)

/// Adds a free prop of one body and one shape, its centre at `at`, turned by
/// `rotation` (a cylinder's axis is its local Z).
fn solid(
    project: &mut Project,
    name: &str,
    geometry: Geometry,
    mass: f32,
    rotation: Quat,
    at: Vec3,
) -> CharacterId {
    let free = Limits {
        twist_min: 0.0,
        twist_max: 0.0,
        swing1: 0.0,
        swing2: 0.0,
    };
    let no_muscle = Gains {
        stiffness: 0.0,
        damping: 0.0,
        force_limit: 0.0,
    };
    let piece = Piece::at(geometry, mass, rotation, Vec3::ZERO);
    let (skeleton, mut rig) = prop_rig(vec![("body", None, at, piece)], free, no_muscle);
    // Its own inertia, not a sphere's, so a log rolls like one.
    rig.isotropic_inertia = false;
    add_prop(
        project,
        name,
        CharacterKind::Prop,
        (skeleton, rig),
        SimLevel::Simulated,
    )
}

/// Turns a cylinder's axis (local Z) upright.
fn upright() -> Quat {
    Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)
}

/// Turns a cylinder's axis (local Z) along X, lying on its side.
fn along_x() -> Quat {
    Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)
}

/// Single cylinders on the ground, on boxes and under a falling ball, 2 m apart along
/// X: one dropped 3 cm onto its cap leaning 3°, a log on its side rolled along +Z, a
/// disc dropped from 0.3 m tilted 10°, one dropped onto a box leaning 3°, one lying on
/// a box and one on the ground, and two balls dropped on a pillar's cap, one just
/// inside its rim and one just outside.
pub fn cylinders() -> Project {
    let mut p = new_scene("Cylinders", 3.0);
    p.physics.contact = ContactModel::Coulomb;
    let cylinder = |radius, length| Geometry::Cylinder { radius, length };
    let block = Geometry::Box {
        size: [1.0, 0.5, 1.0],
    };
    let pillar = cylinder(0.2, 1.0);
    add_prop(
        &mut p,
        "scenery",
        CharacterKind::Environment,
        scenery(&[
            (block, Quat::IDENTITY, Vec3::new(6.0, 0.25, 0.0)),
            (block, Quat::IDENTITY, Vec3::new(8.0, 0.25, 0.0)),
            (pillar, upright(), Vec3::new(10.0, 0.5, 0.0)),
        ]),
        SimLevel::CollisionOnly,
    );
    // Short enough that the old capsule stand-in for it (round ends) would not stand.
    solid(
        &mut p,
        "standing",
        cylinder(0.15, 0.5),
        20.0,
        Quat::from_rotation_z(0.05) * upright(),
        Vec3::new(0.0, 0.28, 0.0),
    );
    let log = solid(
        &mut p,
        "log",
        cylinder(0.1, 0.8),
        15.0,
        along_x(),
        Vec3::new(2.0, 0.1, 0.0),
    );
    push(&mut p, log, 0, "log.MetaBodies.body", Vec3::Z);
    solid(
        &mut p,
        "disc",
        cylinder(0.3, 0.04),
        5.0,
        Quat::from_rotation_z(0.17) * upright(),
        Vec3::new(4.0, 0.3, 0.0),
    );
    solid(
        &mut p,
        "on box",
        cylinder(0.15, 0.4),
        15.0,
        Quat::from_rotation_x(0.05) * upright(),
        Vec3::new(6.0, 0.72, 0.0),
    );
    solid(
        &mut p,
        "across box",
        cylinder(0.1, 0.6),
        10.0,
        along_x(),
        Vec3::new(8.0, 0.62, 0.0),
    );
    solid(
        &mut p,
        "lying",
        cylinder(0.1, 0.8),
        15.0,
        along_x(),
        Vec3::new(12.0, 0.11, 0.0),
    );
    let ball = Geometry::Sphere { radius: 0.05 };
    solid(
        &mut p,
        "ball inside",
        ball,
        1.0,
        Quat::IDENTITY,
        Vec3::new(10.17, 1.2, 0.0),
    );
    solid(
        &mut p,
        "ball outside",
        ball,
        1.0,
        Quat::IDENTITY,
        Vec3::new(10.0, 1.2, 0.22),
    );
    p
}

/// Cylinders resting on cylinders: three discs stacked, a post standing on a disc, a
/// log lying on a plate, and a pyramid of three logs. All start 5 mm to 1 cm
/// apart and fall into place. Also the cylinder-heavy scene timed per step.
pub fn cylinder_pile() -> Project {
    let mut p = new_scene("Cylinder pile", 3.0);
    p.physics.contact = ContactModel::Coulomb;
    let cylinder = |radius, length| Geometry::Cylinder { radius, length };
    let disc = cylinder(0.3, 0.06);
    for (name, y) in [("disc0", 0.035), ("disc1", 0.105), ("disc2", 0.175)] {
        solid(&mut p, name, disc, 4.0, upright(), Vec3::new(0.0, y, 0.0));
    }
    solid(
        &mut p,
        "base",
        cylinder(0.4, 0.06),
        6.0,
        upright(),
        Vec3::new(2.0, 0.035, 0.0),
    );
    solid(
        &mut p,
        "post",
        cylinder(0.12, 0.6),
        10.0,
        upright(),
        Vec3::new(2.0, 0.37, 0.0),
    );
    // A log lying on a plate, and a pyramid of three logs, the top one wedged in the
    // groove between the two under it.
    solid(
        &mut p,
        "plate",
        cylinder(0.5, 0.06),
        8.0,
        upright(),
        Vec3::new(4.0, 0.035, 0.0),
    );
    let log = cylinder(0.1, 0.8);
    solid(
        &mut p,
        "pin",
        log,
        12.0,
        along_x(),
        Vec3::new(4.0, 0.165, 0.0),
    );
    for (name, z) in [("under0", -0.1), ("under1", 0.1)] {
        solid(&mut p, name, log, 12.0, along_x(), Vec3::new(6.0, 0.105, z));
    }
    let groove = 0.1 + 0.03f32.sqrt();
    solid(
        &mut p,
        "top",
        log,
        12.0,
        along_x(),
        Vec3::new(6.0, groove + 0.005, 0.0),
    );
    p
}

// ---------------------------------------------------------------------------------
// Behaviour scenes

fn behave(project: &mut Project, character: CharacterId, start: i32, end: i32, name: &str) {
    event(
        project,
        character,
        "Behaviour",
        start,
        end - start,
        &[("behaviourName", name)],
    );
}

/// The placeholder stands; at 1 s its chest is pushed at `speed` m/s along `dir`.
/// With `behaviour`, Balance runs throughout.
pub fn balance_push(speed: f32, dir: Vec3, behaviour: bool) -> Project {
    pushed(behaviour.then_some("Balance"), speed, dir)
}

/// Hard forward pushes (m/s): Stagger keeps stepping after them and goes down, where
/// Balance stops in a few steps and nearly always stands.
pub const HARD_PUSHES: [f32; 4] = [2.7, 2.8, 2.9, 3.0];

/// The placeholder stands, running `behaviour` throughout (if any); at 1 s its chest
/// is pushed at `speed` m/s along `dir`.
pub fn pushed(behaviour: Option<&str>, speed: f32, dir: Vec3) -> Project {
    let mut p = new_scene(behaviour.unwrap_or("Push"), 4.0);
    let id = placeholder(&mut p, "stander", Vec3::ZERO, 0.0);
    if let Some(name) = behaviour {
        behave(&mut p, id, 0, 240, name);
    }
    push(
        &mut p,
        id,
        60,
        "stander.MetaBodies.chest",
        dir.normalize() * speed,
    );
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

/// Look and Reach, as endorphin's "Hands reach and look at" sets it up: the stand-in
/// (with fingers) stands with its chest locked in place, both hands reaching for a
/// small animated ball and the head looking at it, from the start, at the 2.5 scene's
/// settings (speed 0.7, strength 0.7, at most 0.9 of the arm's length, look-at speed
/// 1). The ball starts at `ball` and moves by `moves` over the 3 s; with `t_pose`, the
/// arms start straight out to the sides, as endorphin's characters rest.
pub fn reach_for(ball: Vec3, moves: Vec3, t_pose: bool) -> Project {
    let mut p = new_scene("Look and Reach", 3.0);
    let arms = [
        ("l_shoulder", Property::RotZ, 90.0),
        ("r_shoulder", Property::RotZ, -90.0),
    ];
    let pose: &[(&str, Property, f32)] = if t_pose { &arms } else { &[] };
    let id = person(&mut p, "reacher", Vec3::ZERO, 0.0, pose);
    let sphere = Geometry::Sphere { radius: 0.05 };
    let target = add_prop(
        &mut p,
        "ball",
        CharacterKind::Prop,
        scenery(&[(sphere, Quat::IDENTITY, ball)]),
        SimLevel::Animated,
    );
    let end = p.settings.range.end;
    let path = vec![
        channel("root", Property::PosX, &[(0, 0.0), (end, moves.x)]),
        channel("root", Property::PosY, &[(0, 0.0), (end, moves.y)]),
        channel("root", Property::PosZ, &[(0, 0.0), (end, moves.z)]),
    ];
    motion(&mut p, target, "Path", 0, end, false, path);
    constrain(
        &mut p,
        id,
        0,
        end,
        "Lock",
        &["reacher.MetaBodies.chest"],
        (STRONG, STRONG),
    );
    let ball = "ball.MetaBodies.piece0";
    let settings = [
        ("behaviourName", "Bob2: Look and Reach 2.5"),
        ("LeftHandTarget", ball),
        ("RightHandTarget", ball),
        ("LookAtTarget", ball),
        ("LeftHand_Speed", "0.700"),
        ("RightHand_Speed", "0.700"),
        ("LeftHand_Strength", "0.700"),
        ("RightHand_Strength", "0.700"),
        ("LeftHand_MaxExtension", "0.900"),
        ("RightHand_MaxExtension", "0.900"),
        ("LookAtSpeed", "1.000"),
        ("LegTorsoStiffness", "0.8"),
    ];
    event(&mut p, id, "Behaviour", 0, end, &settings);
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
    event(
        &mut p,
        tackler,
        "Simulation",
        0,
        1,
        &[
            ("physicsLevel", "CollisionOnlyWithInertia"),
            ("momentumScale", "0.3"),
        ],
    );
    level(&mut p, tackler, 30, "FullPhysics");
    let targets = [
        "tackler.MetaBodies.l_wrist",
        "tackler.MetaBodies.r_wrist",
        "tackler.MetaBodies.chest",
        "tackler.MetaBodies.head",
        "victim.MetaBodies.chest",
    ];
    constrain(
        &mut p,
        victim,
        20,
        120,
        "JoinBodies",
        &targets,
        (STRONG, STRONG),
    );
    p
}

/// Balcony: four characters crowded together on a balcony slab with a railing, two of
/// them shoved into the others.
pub fn balcony() -> Project {
    let mut p = new_scene("Balcony", 2.0);
    let slab = Geometry::Box {
        size: [4.0, 0.3, 2.0],
    };
    let rail = Geometry::Box {
        size: [4.0, 1.0, 0.08],
    };
    let rig = scenery(&[
        (slab, Quat::IDENTITY, Vec3::new(0.0, 2.85, 0.0)),
        (rail, Quat::IDENTITY, Vec3::new(0.0, 3.5, 0.96)),
    ]);
    add_prop(
        &mut p,
        "balcony",
        CharacterKind::Environment,
        rig,
        SimLevel::CollisionOnly,
    );
    let mut people = Vec::new();
    // Shoulder to shoulder, arms overlapping their neighbours'.
    for (i, x) in [-0.54f32, -0.18, 0.18, 0.54].into_iter().enumerate() {
        let name = ["guest1", "guest2", "guest3", "guest4"][i];
        let arms = [
            ("l_shoulder", Property::RotZ, 12.0),
            ("r_shoulder", Property::RotZ, -12.0),
        ];
        people.push(person(&mut p, name, Vec3::new(x, 3.0, 0.45), 0.0, &arms));
    }
    // One walks into the others from behind, then goes limp; two are shoved.
    let late = person(
        &mut p,
        "latecomer",
        Vec3::new(0.0, 3.0, -0.6),
        0.0,
        &[
            ("l_shoulder", Property::RotX, -60.0),
            ("r_shoulder", Property::RotX, -60.0),
        ],
    );
    motion(
        &mut p,
        late,
        "Barge in",
        0,
        20,
        true,
        vec![channel("hips", Property::PosZ, &[(0, -0.6), (20, 0.2)])],
    );
    level(&mut p, late, 0, "CollisionOnlyWithInertia");
    level(&mut p, late, 20, "FullPhysics");
    push(
        &mut p,
        people[1],
        25,
        "guest2.MetaBodies.chest",
        Vec3::new(0.0, 1.0, 5.0),
    );
    push(
        &mut p,
        people[3],
        30,
        "guest4.MetaBodies.chest",
        Vec3::new(-4.0, 0.0, 0.5),
    );
    p
}

/// Catch Rope: a character runs and jumps (collision with momentum), then goes fully
/// simulated and grabs a light rope that hangs from a Lock holding only X and Y.
pub fn catch_rope() -> Project {
    let mut p = new_scene("Catch Rope", 2.0);
    let rope_limits = Limits {
        twist_min: -0.5,
        twist_max: 0.5,
        swing1: 1.2,
        swing2: 1.2,
    };
    let rig = chain(
        Vec3::new(0.0, 4.6, 0.0),
        10,
        0.3,
        0.02,
        0.15,
        None,
        rope_limits,
        Gains {
            stiffness: 0.5,
            damping: 0.05,
            force_limit: 5.0,
        },
    );
    let rope = add_prop(
        &mut p,
        "rope",
        CharacterKind::Prop,
        rig,
        SimLevel::Simulated,
    );
    constrain(
        &mut p,
        rope,
        0,
        120,
        "Lock",
        &["rope.MetaBodies.seg0"],
        (Vec3::new(1e5, 1e5, 0.0), Vec3::ZERO),
    );

    let arms_up = [
        ("l_shoulder", Property::RotX, -170.0),
        ("r_shoulder", Property::RotX, -170.0),
    ];
    let catcher = person(&mut p, "catcher", Vec3::new(0.0, 0.0, -2.0), 0.0, &arms_up);
    let jump = vec![
        channel("hips", Property::PosZ, &[(0, -2.0), (20, -0.2)]),
        channel("hips", Property::PosY, &[(0, 0.0), (20, 0.4)]),
    ];
    motion(&mut p, catcher, "Run and jump", 0, 20, true, jump);
    level(&mut p, catcher, 0, "CollisionOnlyWithInertia");
    level(&mut p, catcher, 20, "FullPhysics");
    for hand in ["catcher.MetaBodies.l_wrist", "catcher.MetaBodies.r_wrist"] {
        constrain(
            &mut p,
            catcher,
            20,
            120,
            "JoinBodies",
            &[hand, "rope.MetaBodies.seg7"],
            (STRONG, STRONG),
        );
    }
    p
}

/// Gymnast Rings: two light straps with rings hang from Locks; a gymnast hangs from the
/// rings by LockWithParent constraints, hands overlapping the rings, and is swung.
pub fn gymnast_rings() -> Project {
    let mut p = new_scene("Gymnast Rings", 2.0);
    let strap = Limits {
        twist_min: -0.5,
        twist_max: 0.5,
        swing1: 1.0,
        swing2: 1.0,
    };
    let gains = Gains {
        stiffness: 1.0,
        damping: 0.1,
        force_limit: 5.0,
    };
    for (name, x) in [("ring_l", 0.19f32), ("ring_r", -0.19)] {
        let ring = Piece::at(
            Geometry::Sphere { radius: 0.09 },
            0.3,
            Quat::IDENTITY,
            Vec3::new(0.0, -0.09, 0.0),
        );
        let rig = chain(
            Vec3::new(x, 5.0, 0.0),
            4,
            0.5,
            0.015,
            0.2,
            Some(ring),
            strap,
            gains,
        );
        let prop = add_prop(&mut p, name, CharacterKind::Prop, rig, SimLevel::Simulated);
        constrain(
            &mut p,
            prop,
            0,
            120,
            "Lock",
            &[&format!("{name}.MetaBodies.seg0")],
            (STRONG, Vec3::ZERO),
        );
    }
    let arms_up = [
        ("l_shoulder", Property::RotX, -180.0),
        ("r_shoulder", Property::RotX, -180.0),
    ];
    let gymnast = person(&mut p, "gymnast", Vec3::new(0.0, 0.84, 0.0), 0.0, &arms_up);
    constrain(
        &mut p,
        gymnast,
        0,
        120,
        "LockWithParent",
        &["gymnast.MetaBodies.l_wrist", "ring_l.MetaBodies.tip"],
        (STRONG, STRONG),
    );
    constrain(
        &mut p,
        gymnast,
        0,
        120,
        "LockWithParent",
        &["gymnast.MetaBodies.r_wrist", "ring_r.MetaBodies.tip"],
        (STRONG, STRONG),
    );
    push(
        &mut p,
        gymnast,
        30,
        "gymnast.MetaBodies.hips",
        Vec3::new(0.0, 0.0, 2.0),
    );
    p
}

/// Bendy Stick: an animated (collision-only) character swings a springy stick that is
/// joined to its hand.
pub fn bendy_stick() -> Project {
    let mut p = new_scene("Bendy Stick", 2.0);
    let pose = [
        ("r_shoulder", Property::RotX, -70.0),
        ("r_elbow", Property::RotX, -20.0),
    ];
    let holder = person(&mut p, "holder", Vec3::ZERO, 0.0, &pose);
    let swing: Vec<(i32, f32)> = (0..=120)
        .step_by(5)
        .map(|f| {
            (
                f,
                40.0 * libm::sinf(f as f32 / 60.0 * std::f32::consts::TAU),
            )
        })
        .collect();
    motion(
        &mut p,
        holder,
        "Swing",
        0,
        120,
        true,
        vec![channel("r_shoulder", Property::RotZ, &swing)],
    );
    level(&mut p, holder, 0, "CollisionOnly");

    // The stick starts in the hand, pointing along the forearm.
    let (hand_rot, hand_pos) = body_pose(&p, holder, "r_wrist", 0.0);
    let along = hand_rot * Vec3::Z;
    let along = if along.dot(Vec3::Z) < 0.0 {
        -along
    } else {
        along
    };
    let start = hand_pos + along * 0.06;
    let segment = along * 0.25;
    let limits = Limits {
        twist_min: -0.3,
        twist_max: 0.3,
        swing1: 0.6,
        swing2: 0.6,
    };
    let mut joints: Vec<(&str, Option<usize>, Vec3, Piece)> = Vec::new();
    for (i, name) in ["seg0", "seg1", "seg2", "seg3", "seg4", "seg5"]
        .into_iter()
        .enumerate()
    {
        let (parent, offset) = if i == 0 {
            (None, start)
        } else {
            (Some(i - 1), segment)
        };
        joints.push((name, parent, offset, Piece::along(segment, 0.02, 0.3)));
    }
    let rig = prop_rig(
        joints,
        limits,
        Gains {
            stiffness: 30.0,
            damping: 1.0,
            force_limit: 100.0,
        },
    );
    add_prop(
        &mut p,
        "stick",
        CharacterKind::Prop,
        rig,
        SimLevel::Simulated,
    );
    constrain(
        &mut p,
        holder,
        0,
        120,
        "JoinBodies",
        &["stick.MetaBodies.seg0", "holder.MetaBodies.r_wrist"],
        (STRONG, STRONG),
    );
    p
}

/// Gun drop: Walk With Gun's pattern. A character holds a 550 kg prop in both hands,
/// joined to them (1e5 N, 1e4 N·m) the way the scene's `JoinBodies` does, and drops
/// with it onto the ground. The fingers hang a few centimetres below the prop's
/// underside, so once it lands they are between the hand it holds and the ground:
/// either the hand turns out of the way or the prop rests on them. Coulomb friction,
/// as an imported scene has it (lesson 21).
pub fn gun_drop() -> Project {
    gun_drop_nudged(0.0)
}

/// Gun drop with the prop's mass changed by the relative amount `nudge`, to see how
/// much a test's numbers move when the scene does (`nudges`).
pub fn gun_drop_nudged(nudge: f32) -> Project {
    let mut p = new_scene("Gun drop", 3.0);
    p.physics.contact = ContactModel::Coulomb;
    let pose = [
        ("l_shoulder", Property::RotX, -75.0),
        ("r_shoulder", Property::RotX, -75.0),
        ("l_elbow", Property::RotX, -30.0),
        ("r_elbow", Property::RotX, -30.0),
    ];
    // endorphin's fingers weigh 0.39 kg each; the stand-in's default is lighter.
    let (skeleton, mut rig) = person_rig();
    for side in ["l", "r"] {
        let i = rig.body_index(&format!("{side}_fingers")).unwrap();
        let body = &mut rig.bodies[i];
        body.mass = 0.39;
        body.inertia = body.shapes[0].geometry.inertia(body.mass);
    }
    let holder = add_person(&mut p, "holder", (skeleton, rig), Vec3::ZERO, 0.0, &pose);
    // The prop sits between the hands, its underside 3 cm above the fingers' centres:
    // the fingers reach below it, as in the scene.
    let (_, left) = body_pose(&p, holder, "l_wrist", 0.0);
    let (_, right) = body_pose(&p, holder, "r_wrist", 0.0);
    let fingers = [
        body_pose(&p, holder, "l_fingers", 0.0).1.y,
        body_pose(&p, holder, "r_fingers", 0.0).1.y,
    ];
    let size = Vec3::new((left - right).length() + 0.1, 0.12, 0.12);
    let mut centre = (left + right) / 2.0;
    centre.y = fingers[0].max(fingers[1]) + 0.03 + size.y / 2.0;
    let rigid = Limits {
        twist_min: 0.0,
        twist_max: 0.0,
        swing1: 0.0,
        swing2: 0.0,
    };
    let none = Gains {
        stiffness: 0.0,
        damping: 0.0,
        force_limit: 0.0,
    };
    let gun = Piece::at(
        Geometry::Box {
            size: size.to_array(),
        },
        550.0 * (1.0 + nudge),
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    add_prop(
        &mut p,
        "gun",
        CharacterKind::Prop,
        prop_rig(vec![("body", None, centre, gun)], rigid, none),
        SimLevel::Simulated,
    );
    constrain(
        &mut p,
        holder,
        0,
        180,
        "JoinBodies",
        &[
            "gun.MetaBodies.body",
            "holder.MetaBodies.r_wrist",
            "holder.MetaBodies.l_wrist",
        ],
        (STRONG, Vec3::splat(1e4)),
    );
    p
}

/// Heli Leap: a heavy helicopter (fuselage, tail boom, skids) held in the air by a
/// Lock; a character jumps off its roof and falls to the ground.
pub fn heli_leap() -> Project {
    let mut p = new_scene("Heli Leap", 2.0);
    let fuselage = Piece::at(
        Geometry::Box {
            size: [1.6, 1.4, 3.0],
        },
        1500.0,
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    let tail = Piece::along(Vec3::new(0.0, 0.0, -4.0), 0.15, 150.0);
    let skid = |x: f32| {
        Piece::at(
            Geometry::Capsule {
                radius: 0.05,
                length: 2.6,
            },
            40.0,
            Quat::IDENTITY,
            Vec3::new(x, 0.0, 0.0),
        )
    };
    let joints = vec![
        ("fuselage", None, Vec3::new(0.0, 6.0, 0.0), fuselage),
        ("tail", Some(0), Vec3::new(0.0, 0.3, -1.5), tail),
        ("skid_l", Some(0), Vec3::new(0.0, -1.0, 0.0), skid(0.7)),
        ("skid_r", Some(0), Vec3::new(0.0, -1.0, 0.0), skid(-0.7)),
    ];
    let limits = Limits {
        twist_min: -0.2,
        twist_max: 0.2,
        swing1: 0.2,
        swing2: 0.2,
    };
    let rig = prop_rig(
        joints,
        limits,
        Gains {
            stiffness: 2e5,
            damping: 2e4,
            force_limit: 1e5,
        },
    );
    let heli = add_prop(
        &mut p,
        "helicopter",
        CharacterKind::Prop,
        rig,
        SimLevel::Simulated,
    );
    constrain(
        &mut p,
        heli,
        0,
        120,
        "Lock",
        &["helicopter.MetaBodies.fuselage"],
        (STRONG, STRONG),
    );

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
    let slab = Geometry::Box {
        size: [5.0, 0.1, 5.0],
    };
    let floor = add_prop(
        &mut p,
        "floor",
        CharacterKind::Environment,
        scenery(&[(slab, Quat::IDENTITY, Vec3::new(0.0, -0.05, 0.0))]),
        SimLevel::CollisionOnly,
    );
    let rock: Vec<(i32, f32)> = (0..=120)
        .step_by(3)
        .map(|f| {
            (
                f,
                10.0 * libm::sinf(f as f32 / 60.0 * std::f32::consts::TAU),
            )
        })
        .collect();
    motion(
        &mut p,
        floor,
        "Spin",
        0,
        120,
        false,
        vec![
            channel("root", Property::RotY, &[(0, 0.0), (120, 180.0)]),
            channel("root", Property::RotX, &rock),
        ],
    );
    person(&mut p, "dancer", Vec3::new(0.6, 0.0, 0.0), 90.0, &[]);
    p
}

/// A variant of Rotating Floor with a turntable: a flat cylinder spinning about its axis.
pub fn turntable() -> Project {
    let mut p = new_scene("Turntable", 2.0);
    p.physics.ground = None;
    let disc = Geometry::Cylinder {
        radius: 2.5,
        length: 0.2,
    };
    let flat = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
    let floor = add_prop(
        &mut p,
        "turntable",
        CharacterKind::Environment,
        scenery(&[(disc, flat, Vec3::new(0.0, -0.1, 0.0))]),
        SimLevel::CollisionOnly,
    );
    motion(
        &mut p,
        floor,
        "Spin",
        0,
        120,
        false,
        vec![channel("root", Property::RotY, &[(0, 0.0), (120, 180.0)])],
    );
    person(&mut p, "dancer", Vec3::new(0.6, 0.0, 0.0), 90.0, &[]);
    p
}

/// Rotating Floor's last seconds: no ground plane; a long collision-only slab with a
/// wall at one end tilts 20° about its middle, carrying a character standing near the
/// low end about 1.7 m below where the ground would be, on the floor all the while.
pub fn tilting_floor() -> Project {
    let mut p = new_scene("Tilting floor", 2.0);
    p.physics.ground = None;
    let slab = Geometry::Box {
        size: [4.0, 0.3, 12.0],
    };
    let wall = Geometry::Box {
        size: [4.0, 2.0, 0.2],
    };
    let floor = add_prop(
        &mut p,
        "floor",
        CharacterKind::Environment,
        scenery(&[
            (slab, Quat::IDENTITY, Vec3::new(0.0, -0.15, 0.0)),
            (wall, Quat::IDENTITY, Vec3::new(0.0, 1.0, 5.9)),
        ]),
        SimLevel::CollisionOnly,
    );
    // Positive turns about X lower +Z: the walled end, where the character stands.
    motion(
        &mut p,
        floor,
        "Tilt",
        0,
        120,
        false,
        vec![channel("root", Property::RotX, &[(0, 0.0), (60, 20.0)])],
    );
    person(&mut p, "dancer", Vec3::new(0.0, 0.0, 5.0), 180.0, &[]);
    p
}

/// Heli Leap's jumper: no ground plane; a character runs off the edge of a roof (its
/// top at y = 0, the building under it) and falls past the building's side.
pub fn roof_edge() -> Project {
    let mut p = new_scene("Roof edge", 2.0);
    p.physics.ground = None;
    let roof = Geometry::Box {
        size: [6.0, 1.0, 6.0],
    };
    let building = Geometry::Box {
        size: [5.0, 8.0, 5.0],
    };
    add_prop(
        &mut p,
        "building",
        CharacterKind::Environment,
        scenery(&[
            (roof, Quat::IDENTITY, Vec3::new(0.0, -0.5, -3.0)),
            (building, Quat::IDENTITY, Vec3::new(0.0, -5.0, -3.0)),
        ]),
        SimLevel::CollisionOnly,
    );
    let jumper = person(&mut p, "jumper", Vec3::new(0.0, 0.0, -1.5), 0.0, &[]);
    let run = vec![
        channel("hips", Property::PosZ, &[(0, -1.5), (30, 0.5)]),
        channel("hips", Property::PosY, &[(0, 0.0), (30, 0.1)]),
    ];
    motion(&mut p, jumper, "Run", 0, 30, true, run);
    level(&mut p, jumper, 0, "CollisionOnlyWithInertia");
    level(&mut p, jumper, 30, "FullPhysics");
    p
}

/// Catch Rope after the catch: a rope locked at the top beside a roof is set swinging by
/// a force event that sits on the environment's track but names the rope's end (as the
/// real scene's does), and its last link is cut at 1 s. The roof is the environment's
/// massless piece, with a character standing on it.
pub fn cut_rope() -> Project {
    let mut p = new_scene("Cut rope", 2.0);
    let roof = Geometry::Box {
        size: [4.0, 0.4, 4.0],
    };
    let (skeleton, mut rig) = scenery(&[(roof, Quat::IDENTITY, Vec3::new(0.0, 2.8, 0.0))]);
    rig.bodies[0].mass = 0.0;
    rig.bodies[0].inertia = [0.0; 3];
    let building = add_prop(
        &mut p,
        "building",
        CharacterKind::Environment,
        (skeleton, rig),
        SimLevel::Simulated,
    );
    let rig = chain(
        Vec3::new(0.0, 6.5, 2.6),
        10,
        0.3,
        0.02,
        0.15,
        None,
        Limits {
            twist_min: -0.5,
            twist_max: 0.5,
            swing1: 1.2,
            swing2: 1.2,
        },
        Gains {
            stiffness: 0.5,
            damping: 0.05,
            force_limit: 5.0,
        },
    );
    let rope = add_prop(
        &mut p,
        "rope",
        CharacterKind::Prop,
        rig,
        SimLevel::Simulated,
    );
    constrain(
        &mut p,
        rope,
        0,
        120,
        "Lock",
        &["rope.MetaBodies.seg0"],
        (Vec3::new(1e5, 1e5, 0.0), Vec3::ZERO),
    );
    push(
        &mut p,
        building,
        1,
        "rope.MetaBodies.seg9",
        Vec3::new(0.31, 0.0, 0.95),
    );
    event(
        &mut p,
        rope,
        "Sever",
        60,
        1,
        &[("targets", "rope.MetaBodies.seg9")],
    );
    person(&mut p, "climber", Vec3::new(0.0, 3.0, -1.0), 0.0, &[]);
    p
}

/// Car Hit: a car (collision with momentum) drives through a standing character at
/// 15 m/s, bumper first.
pub fn car_hit() -> Project {
    car_hit_nudged(0.0)
}

/// Car Hit with the car's speed changed by the relative amount `nudge` (`nudges`).
pub fn car_hit_nudged(nudge: f32) -> Project {
    let mut p = new_scene("Car Hit", 2.0);
    let body = Piece::at(
        Geometry::Box {
            size: [1.8, 0.8, 4.2],
        },
        1200.0,
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    let bumper = Piece::at(
        Geometry::Box {
            size: [1.8, 0.25, 0.2],
        },
        50.0,
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    let joints = vec![
        ("body", None, Vec3::new(0.0, 0.9, 0.0), body),
        ("bumper", Some(0), Vec3::new(0.0, -0.4, 2.15), bumper),
    ];
    let rig = prop_rig(
        joints,
        Limits {
            twist_min: -0.1,
            twist_max: 0.1,
            swing1: 0.1,
            swing2: 0.1,
        },
        Gains {
            stiffness: 1e4,
            damping: 1e3,
            force_limit: 1e4,
        },
    );
    let car = add_prop(
        &mut p,
        "car",
        CharacterKind::Prop,
        rig,
        SimLevel::CollisionWithMomentum,
    );
    motion(
        &mut p,
        car,
        "Drive",
        0,
        120,
        true,
        vec![channel(
            "body",
            Property::PosZ,
            &[(0, -6.0), (120, -6.0 + 30.0 * (1.0 + nudge))],
        )],
    );
    level(&mut p, car, 0, "CollisionOnlyWithInertia");
    person(&mut p, "pedestrian", Vec3::ZERO, 90.0, &[]);
    p
}

/// Car Hit as endorphin's scene sets it up: a 747 kg car, simulated, joined by a strong
/// JoinBodies to a keyed carrier hidden underground, which drives it into a standing
/// character at 8 m/s. The bonnet's edge, 0.77 m up, catches the arm hanging at the
/// character's side and drives it into their ribs; the car carries them on the bonnet
/// and stops dead when the carrier's keys end at 1.65 s. The character has clavicles,
/// as endorphin's do (the scene's failing joint was a clavicle).
///
/// The join gives way when the carrier stops: stopping 747 kg from 8 m/s at 1e5 N takes
/// 60 ms and 24 cm, so the lock drifts by design.
pub fn car_carry() -> Project {
    car_carry_at(8.0)
}

/// Car carry with the carrier driving at `speed` m/s instead of 8 (to see how much a
/// slightly different hit changes).
pub fn car_carry_at(speed: f32) -> Project {
    let mut p = new_scene("Car carry", 2.0);
    // Friction as endorphin's scenes have it once imported (lesson 21).
    p.physics.contact = ContactModel::Coulomb;
    let mass = 747.0;
    // Chassis 0.15-0.77 m above the ground, a cabin behind the bonnet. Car Hit's car
    // clears the ground by about 15 cm too. With 7 cm, which this stand-in had until
    // friction gripped in proportion to the load (lesson 21), a foot caught under the
    // chassis is dragged along the road at 8 m/s under the car's weight.
    let chassis = Geometry::Box {
        size: [1.7, 0.62, 4.2],
    };
    let cabin = Geometry::Box {
        size: [1.5, 0.55, 2.0],
    };
    let skeleton = SkeletonRef {
        name: "Car".into(),
        source: None,
        joints: vec![Joint::new("body", None, [0.0, 0.42, 0.0])],
    };
    let shape = |geometry, position: Vec3| ShapeDef {
        geometry,
        frame: Frame {
            rotation: Quat::IDENTITY.to_array(),
            position: position.to_array(),
        },
        material: "default".into(),
        collides_with_parent: false,
        collides_with_siblings: false,
        collides_with_ancestors: false,
    };
    let rig = PhysicsRig {
        bodies: vec![BodyDef {
            name: "body".into(),
            joint: 0,
            frame: Frame::IDENTITY,
            mass,
            inertia: chassis.inertia(mass),
            shapes: vec![
                shape(chassis, Vec3::new(0.0, 0.04, 0.0)),
                shape(cabin, Vec3::new(0.0, 0.6, -0.7)),
            ],
        }],
        joints: Vec::new(),
        muscle_sets: Vec::new(),
        isotropic_inertia: true,
        virtual_muscles: Vec::new(),
    };
    let car = add_prop(
        &mut p,
        "car",
        CharacterKind::Prop,
        (skeleton, rig),
        SimLevel::Simulated,
    );
    let block = Piece::at(
        Geometry::Box {
            size: [0.5, 0.5, 0.5],
        },
        100.0,
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    let still = Limits {
        twist_min: 0.0,
        twist_max: 0.0,
        swing1: 0.0,
        swing2: 0.0,
    };
    let no_muscle = Gains {
        stiffness: 0.0,
        damping: 0.0,
        force_limit: 0.0,
    };
    let carrier = add_prop(
        &mut p,
        "carrier",
        CharacterKind::Prop,
        prop_rig(
            vec![("box", None, Vec3::new(0.0, -1.0, 0.0), block)],
            still,
            no_muscle,
        ),
        SimLevel::CollisionOnly,
    );
    // Both start 4 m behind the character; the carrier's keys end at 1.65 s.
    motion(
        &mut p,
        car,
        "Place",
        0,
        120,
        false,
        vec![held("body", Property::PosZ, -4.0)],
    );
    motion(
        &mut p,
        carrier,
        "Drive",
        0,
        120,
        false,
        vec![channel(
            "box",
            Property::PosZ,
            &[(0, -4.0), (99, -4.0 + speed * 1.65)],
        )],
    );
    constrain(
        &mut p,
        car,
        0,
        120,
        "JoinBodies",
        &["car.MetaBodies.body", "carrier.MetaBodies.box"],
        (STRONG, STRONG),
    );
    // Side on to the car, its left arm towards it.
    add_person(
        &mut p,
        "pedestrian",
        person_rig_with_clavicles(),
        Vec3::ZERO,
        90.0,
        &[],
    );
    p
}

/// Car Hit's join on its own: a 747 kg car on the ground, joined (1e5 N and N·m) to a
/// light carrier hidden underground, which is keyed to drive off at 8 m/s at once, stop
/// dead at 1 s and stay. A force of 1e5 N moves 747 kg at 134 m/s², so the car falls
/// behind for the first few frames (8 m/s takes 60 ms) and runs on past the stop, by
/// design; what matters is that it then catches up (lesson 20).
pub fn heavy_join() -> Project {
    let mut p = new_scene("Heavy join", 2.0);
    let car = Piece::at(
        Geometry::Box {
            size: [1.7, 0.62, 4.2],
        },
        747.0,
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    let carrier = Piece::at(
        Geometry::Box {
            size: [0.1, 0.1, 0.1],
        },
        0.004,
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    let still = Limits {
        twist_min: 0.0,
        twist_max: 0.0,
        swing1: 0.0,
        swing2: 0.0,
    };
    let no_muscle = Gains {
        stiffness: 0.0,
        damping: 0.0,
        force_limit: 0.0,
    };
    let car = add_prop(
        &mut p,
        "car",
        CharacterKind::Prop,
        prop_rig(
            vec![("body", None, Vec3::new(0.0, 0.31, 0.0), car)],
            still,
            no_muscle,
        ),
        SimLevel::Simulated,
    );
    let carrier = add_prop(
        &mut p,
        "carrier",
        CharacterKind::Prop,
        prop_rig(
            vec![("box", None, Vec3::new(0.0, -1.0, 0.0), carrier)],
            still,
            no_muscle,
        ),
        SimLevel::CollisionOnly,
    );
    motion(
        &mut p,
        carrier,
        "Drive",
        0,
        120,
        false,
        vec![channel(
            "box",
            Property::PosZ,
            &[(0, 0.0), (60, 8.0), (120, 8.0)],
        )],
    );
    constrain(
        &mut p,
        car,
        0,
        120,
        "JoinBodies",
        &["car.MetaBodies.body", "carrier.MetaBodies.box"],
        (STRONG, STRONG),
    );
    p
}

/// How far Heavy join's car is from where its join holds it, m, at each frame.
pub fn heavy_join_gaps(project: &Project, sim: &Simulation) -> Vec<f32> {
    let index = |name: &str| {
        project
            .characters
            .iter()
            .position(|c| c.name == name)
            .unwrap()
    };
    let (car, carrier) = (index("car"), index("carrier"));
    let offset = |f: i32| {
        let frame = sim.frame(f).unwrap();
        let at = |ci: usize| frame[ci].as_ref().unwrap().bodies[0].1;
        at(car) - at(carrier)
    };
    let start = offset(0);
    (0..=project.settings.range.end)
        .map(|f| offset(f).distance(start))
        .collect()
}

/// Car Hit's pedestrian landing on the bonnet, as a box: Heavy join's car drives at
/// 8 m/s, and a 75 kg box (0.5 m), thrown at 4 m/s the other way, drops 1.4 m onto
/// its bonnet half a second in, sliding back over it at 12 m/s. How much of the
/// car's speed the box picks up is friction's doing, under the load of its landing
/// and its weight (lesson 21).
pub fn bonnet_slide() -> Project {
    let mut p = heavy_join();
    p.settings.name = "Bonnet slide".into();
    // Friction as endorphin's scenes have it once imported (lesson 21).
    p.physics.contact = ContactModel::Coulomb;
    let piece = Piece::at(
        Geometry::Box {
            size: [0.5, 0.5, 0.5],
        },
        75.0,
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    let free = Limits {
        twist_min: 0.0,
        twist_max: 0.0,
        swing1: 0.0,
        swing2: 0.0,
    };
    let no_muscle = Gains {
        stiffness: 0.0,
        damping: 0.0,
        force_limit: 0.0,
    };
    let id = add_prop(
        &mut p,
        "box",
        CharacterKind::Prop,
        prop_rig(
            vec![("body", None, Vec3::new(0.0, 2.0, 7.0), piece)],
            free,
            no_muscle,
        ),
        SimLevel::Simulated,
    );
    push(
        &mut p,
        id,
        0,
        "box.MetaBodies.body",
        Vec3::new(0.0, 0.0, -4.0),
    );
    p
}

/// Bonnet slide measured: the box's velocity along the road (m/s) at each frame, and
/// the first frame it touches the car.
pub fn bonnet_slide_track(project: &Project, sim: &Simulation) -> (Vec<f32>, i32) {
    let ci = project
        .characters
        .iter()
        .position(|c| c.name == "box")
        .unwrap();
    let z = |f: i32| sim.frame(f).unwrap()[ci].as_ref().unwrap().bodies[0].1.z;
    let fps = project.settings.fps;
    let end = project.settings.range.end;
    let speeds = (0..=end)
        .map(|f| (z((f + 1).min(end)) - z((f - 1).max(0))) * fps / 2.0)
        .collect();
    // Landed: its centre within 0.3 m of the bonnet (0.62 m up).
    let landed = (0..=end)
        .find(|&f| sim.frame(f).unwrap()[ci].as_ref().unwrap().bodies[0].1.y < 0.92)
        .unwrap_or(end);
    (speeds, landed)
}

/// Car carry's hit and carry: from the start until the carrier's keys end and the car
/// stops, frame 99 (the hit is around frame 15). Lesson 15 is about this part.
pub const CAR_CARRY_HIT: std::ops::RangeInclusive<i32> = 0..=99;

/// What `car_carry_keeps_joints_together` measures.
pub struct CarCarry {
    /// The sweep over the whole scene.
    pub whole: Report,
    /// The sweep over the hit and carry (`CAR_CARRY_HIT`).
    pub hit: Report,
    /// The largest stretch in the hit and carry of a joint other than the toes' and
    /// fingers' hinges (light bodies that the bonnet strikes directly), m, and where.
    pub body: (f32, String),
    /// The sweep over the rest of the scene, after the stop.
    pub after: Report,
    /// `body` for the rest of the scene, after the stop, m, and where.
    pub after_body: (f32, String),
}

/// Car carry measured, with the carrier driving at `speed` m/s.
pub fn car_carry_numbers(speed: f32) -> CarCarry {
    let project = car_carry_at(speed);
    let (sim, elapsed) = simulate(&project);
    let (start, stop, end) = (
        *CAR_CARRY_HIT.start(),
        *CAR_CARRY_HIT.end(),
        project.settings.range.end,
    );
    let frames = |start, end| measure_frames(&project, &sim, elapsed, FrameRange { start, end });
    let hinge = |name: &str| name.ends_with("_toe") || name.ends_with("_fingers");
    CarCarry {
        whole: measure(&project, &sim, elapsed),
        hit: frames(start, stop),
        body: joint_stretch(&project, &sim, "pedestrian", CAR_CARRY_HIT, |n| !hinge(n)),
        after: frames(stop + 1, end),
        after_body: joint_stretch(&project, &sim, "pedestrian", stop + 1..=end, |n| !hinge(n)),
    }
}

/// Back-Flip-To-Combo (a user's scene, not one of endorphin's samples): a character
/// stands on a 2.5 m box, weakly locked at the toes until frame 206 and held at the
/// pelvis against sideways drift and against turning about Y and Z until 305. From frame
/// 246 to 297 two locks with a negative angular strength (-100 N·m about X: drives, not
/// holds) spin pelvis and upper spine backwards, and it flips off the back of the box. At
/// 120 fps, like the scene.
///
/// The scene's character stands until the flip; the placeholder, without the scene's
/// animation, doesn't: left alone it tips forwards from about frame 70 and is leaning
/// about 25° when the drive starts, too far for 200 N·m to bring back. With `balance`,
/// Balance keeps it standing until the drive starts. 5 s long: with Coulomb friction
/// (lesson 21) its feet grip the box for longer, and left alone it lands at frame 412
/// and only settles by 520.
pub fn backflip(balance: bool) -> Project {
    let mut p = new_scene("Back flip", 5.0);
    // Friction as endorphin's scenes have it once imported (lesson 21).
    p.physics.contact = ContactModel::Coulomb;
    p.physics.spring_cap = SpringCap::Limbs;
    p.settings.fps = 120.0;
    p.settings.range.end = 600;
    let block = Geometry::Box {
        size: [1.2, 2.5, 1.2],
    };
    add_prop(
        &mut p,
        "box",
        CharacterKind::Environment,
        scenery(&[(block, Quat::IDENTITY, Vec3::new(0.0, 1.25, 0.0))]),
        SimLevel::CollisionOnly,
    );
    // Facing +Z with the heels 25 cm from the box's back edge.
    let flipper = person(&mut p, "flipper", Vec3::new(0.0, 2.5, -0.3), 0.0, &[]);
    let toes = ["flipper.MetaBodies.l_toe", "flipper.MetaBodies.r_toe"];
    constrain(
        &mut p,
        flipper,
        0,
        206,
        "Lock",
        &toes,
        (Vec3::splat(25.0), Vec3::splat(25.0)),
    );
    constrain(
        &mut p,
        flipper,
        0,
        305,
        "Lock",
        &["flipper.MetaBodies.hips"],
        (Vec3::new(1e5, 0.0, 0.0), Vec3::new(0.0, 1e5, 1e5)),
    );
    let spin = (Vec3::ZERO, Vec3::new(-100.0, 0.0, 0.0));
    constrain(
        &mut p,
        flipper,
        246,
        297,
        "Lock",
        &["flipper.MetaBodies.hips", "flipper.MetaBodies.chest"],
        spin,
    );
    if balance {
        behave(&mut p, flipper, 0, 246, "Balance");
    }
    p
}

/// Car Hit's hand-back: a character with clavicles drops limp, curls up on the ground
/// (BodyFoetal from 1 s), and from 2.25 s a 0.75 s transition blends it back onto its
/// standing animation, overlapping the end of the curl at 2.75 s as the scene's
/// transition overlaps BodyFoetal's end. Curled on the ground and standing are far
/// apart, so every joint's blended rotation is far from both poses' (lesson 18).
pub fn curl_hand_back() -> Project {
    let mut p = new_scene("Curl hand-back", 3.5);
    let id = add_person(
        &mut p,
        "curler",
        person_rig_with_clavicles(),
        Vec3::ZERO,
        0.0,
        &[],
    );
    behave(&mut p, id, 60, 165, "BodyFoetal");
    event(
        &mut p,
        id,
        "Transition",
        135,
        45,
        &[("requiredPhysicsLevel", "No Simulation")],
    );
    p
}

/// Writhe in mid-air, like endorphin's behaviour scene of that name: a character
/// (the placeholder, or with `like_endorphins` a rig resting in a T-pose with
/// clavicles and forearm twists) drops from 4 m. With `behaviour`, Explosion runs
/// throughout with the scene's settings (WritheInMidAir 2.5's).
pub fn mid_air(like_endorphins: bool, behaviour: bool) -> Project {
    mid_air_nudged(like_endorphins, behaviour, 0.0)
}

/// [`mid_air`] with the starting height changed by the relative amount `nudge`
/// (`nudges`).
pub fn mid_air_nudged(like_endorphins: bool, behaviour: bool, nudge: f32) -> Project {
    let mut p = new_scene("Writhe in mid-air", 3.5);
    let at = Vec3::new(0.0, 4.0 * (1.0 + nudge), 0.0);
    let id = if like_endorphins {
        add_person(
            &mut p,
            "writher",
            person_rig_like_endorphins(true),
            at,
            0.0,
            &[],
        )
    } else {
        placeholder(&mut p, "writher", at, 0.0)
    };
    if behaviour {
        event(
            &mut p,
            id,
            "Behaviour",
            0,
            210,
            &[
                ("behaviourName", "Bob2: WritheInMidAir 2.5"),
                ("Amplitude", "0.6"),
                ("Speed", "0.6"),
                ("PhaseOffset", "0.000"),
                ("Strength", "0.5"),
                ("HeadTurn", "0.6"),
                ("BackArch", "1"),
            ],
        );
    }
    p
}

/// An active pose event on a character, driving `targets` (`character.Joints.name`,
/// none = every joint) towards its held channels.
fn active_pose(
    project: &mut Project,
    on: CharacterId,
    start: i32,
    end: i32,
    targets: &[&str],
    channels: Vec<Channel>,
) {
    let track = project.add_track(on, "Active pose");
    let id = project.add_clip(
        props("Active pose", track, start, end - start),
        ClipKind::Event {
            event: "Active pose".into(),
        },
        channels,
    );
    let source = &mut project.clip_mut(id).unwrap().source;
    source.insert("strength".into(), "1".into());
    if !targets.is_empty() {
        source.insert("targets".into(), targets.join(", "));
    }
}

/// Back-Flip-To-Combo splits its character between overlapping active poses: one
/// event drives the legs, another, overlapping it, the spine and arms, and so on. Like
/// endorphin's, each pose here stores the whole body (the joints it doesn't drive at
/// rest) and names the joints it drives. Hung by the hips so it doesn't fall, the
/// placeholder bends its knees by 60° in one event from frame 10, then its elbows by
/// 60° in another from frame 20 that leaves the knees at rest.
pub fn split_poses() -> Project {
    let mut p = new_scene("Split poses", 3.0);
    let id = person(&mut p, "poser", Vec3::new(0.0, 1.5, 0.0), 0.0, &[]);
    constrain(
        &mut p,
        id,
        0,
        180,
        "Lock",
        &["poser.MetaBodies.hips"],
        (STRONG, STRONG),
    );
    let pose = |knee: f32, elbow: f32| {
        ["l", "r"]
            .into_iter()
            .flat_map(|side| {
                [
                    held(&format!("{side}_knee"), Property::RotX, knee),
                    held(&format!("{side}_elbow"), Property::RotX, elbow),
                ]
            })
            .collect::<Vec<_>>()
    };
    let knees = ["poser.Joints.l_knee", "poser.Joints.r_knee"];
    let elbows = ["poser.Joints.l_elbow", "poser.Joints.r_elbow"];
    active_pose(&mut p, id, 10, 180, &knees, pose(60.0, 0.0));
    active_pose(&mut p, id, 20, 180, &elbows, pose(0.0, -60.0));
    p
}

/// Back-Flip-To-Combo's first pose: a character with clavicles, hung by the hips, holds
/// its arms straight out in front from frame 0, eased in over 0.5 s, with the muscle cap
/// `cap`. A 0.4 kg clavicle holds the whole arm up.
pub fn arms_out(cap: SpringCap) -> Project {
    let mut p = new_scene("Arms out", 2.0);
    p.physics.spring_cap = cap;
    // Tuned on force-capped friction, the default when it was written. With Coulomb
    // friction (now the default, lesson 26) the limb cap gives the same -32° / -9°, but
    // the two-body cap's weak arms no longer hang still: rubbing against the body they
    // creep up to -43° at frame 50 and settle near -53°, and the comparison this scene
    // makes is between the caps, not the friction models.
    p.physics.contact = ContactModel::ForceCapped;
    let id = add_person(
        &mut p,
        "poser",
        person_rig_with_clavicles(),
        Vec3::new(0.0, 1.5, 0.0),
        0.0,
        &[],
    );
    constrain(
        &mut p,
        id,
        0,
        120,
        "Lock",
        &["poser.MetaBodies.hips"],
        (STRONG, STRONG),
    );
    // The whole body is driven, as the scene's first pose drives every joint: the
    // spine at rest, the arms forward.
    let arms = ["l", "r"].map(|side| held(&format!("{side}_shoulder"), Property::RotX, -90.0));
    active_pose(&mut p, id, 0, 120, &[], arms.to_vec());
    let clip = p.clips.last_mut().unwrap();
    clip.source.insert("entryBlendPeriod".into(), "0.5".into());
    p
}

/// Virtual muscles: hung by the hips, the placeholder (with clavicles) reaches both arms
/// straight out in front with an active pose at strength 10 (Bendy Stick's "Grab the
/// stick"). With the two-body muscle cap its shoulders can't hold the arms up (lesson
/// 27, `arms_out`). `vm` gives it chest-to-hand virtual muscles pulling the centre of
/// each hand, with these (position, orientation) gains per unit of strength in the
/// pose preset; endorphin's characters list theirs at 1 in every field. `named`: the
/// pose names them (as endorphin's whole-body poses do) or only the arm joints, which
/// leaves them off.
pub fn virtual_reach(vm: Option<(Gains, Gains)>, named: bool) -> Project {
    let mut p = new_scene("Virtual reach", 2.0);
    // As in `arms_out`, whose arms this cap leaves hanging.
    p.physics.spring_cap = SpringCap::Bodies;
    p.physics.contact = ContactModel::ForceCapped;
    let (skeleton, mut rig) = person_rig_with_clavicles();
    if let Some((linear, angular)) = vm {
        let chest = rig.body_index("chest").unwrap();
        for side in ["l", "r"] {
            let name = format!("{side}_ArmVirtualMuscle");
            rig.virtual_muscles.push(VirtualMuscleDef {
                name: name.clone(),
                muscle: name.clone(),
                parent_body: chest,
                body: rig.body_index(&format!("{side}_wrist")).unwrap(),
                point: [0.0; 3],
                position: true,
                orientation: true,
            });
            for set in &mut rig.muscle_sets {
                set.linear.insert(name.clone(), linear);
                set.muscles.insert(
                    name.clone(),
                    MuscleGains {
                        twist: angular,
                        swing: angular,
                    },
                );
            }
        }
    }
    let id = add_person(
        &mut p,
        "reacher",
        (skeleton, rig),
        Vec3::new(0.0, 1.5, 0.0),
        0.0,
        &[],
    );
    constrain(
        &mut p,
        id,
        0,
        120,
        "Lock",
        &["reacher.MetaBodies.hips"],
        (STRONG, STRONG),
    );
    let arms = ["l", "r"].map(|side| held(&format!("{side}_shoulder"), Property::RotX, -90.0));
    let mut targets: Vec<String> = ["l", "r"]
        .iter()
        .flat_map(|s| ["shoulder", "elbow", "wrist"].map(|j| format!("reacher.Joints.{s}_{j}")))
        .collect();
    if named {
        targets.extend(["l", "r"].map(|s| format!("reacher.Joints.{s}_ArmVirtualMuscle")));
    }
    let targets: Vec<&str> = targets.iter().map(String::as_str).collect();
    active_pose(&mut p, id, 0, 120, &targets, arms.to_vec());
    p.clips
        .last_mut()
        .unwrap()
        .source
        .insert("strength".into(), "10".into());
    p
}

/// Back-Flip-To-Combo's crouch: one event names the knees and stores the ankles
/// bent too; nothing names the ankles. Hung by the hips, the placeholder bends its knees
/// by 60° and (stored, unnamed) its ankles by 30° from frame 10.
pub fn unnamed_ankles() -> Project {
    let mut p = new_scene("Unnamed ankles", 2.0);
    let id = person(&mut p, "poser", Vec3::new(0.0, 1.5, 0.0), 0.0, &[]);
    constrain(
        &mut p,
        id,
        0,
        120,
        "Lock",
        &["poser.MetaBodies.hips"],
        (STRONG, STRONG),
    );
    let pose = ["l", "r"]
        .into_iter()
        .flat_map(|side| {
            [
                held(&format!("{side}_knee"), Property::RotX, 60.0),
                held(&format!("{side}_ankle"), Property::RotX, -30.0),
            ]
        })
        .collect();
    active_pose(
        &mut p,
        id,
        10,
        120,
        &["poser.Joints.l_knee", "poser.Joints.r_knee"],
        pose,
    );
    p
}

/// A legacy pose that stores muscle targets instead of joint transforms (endorphin's
/// `dp.nma` format), imported and used as an active pose. The pose is the importer's
/// hand-written fixture (hips flexed 0.5 rad, knees -1.0, elbows 0.8, on the pose's own
/// hinges), the character the placeholder with its muscles named as endorphin's
/// effectors, hung by the hips so it doesn't fall; the pose drives it from frame 10.
pub fn legacy_pose() -> Project {
    use crate::io::endorphin::{legacy_fixture, pose_from_text};
    let mut p = new_scene("Legacy pose", 3.0);
    let (skeleton, mut rig) = person_rig();
    legacy_fixture::effector_names(&mut rig);
    let at = Vec3::new(0.0, 1.5, 0.0);
    let id = add_person(&mut p, "poser", (skeleton, rig), at, 0.0, &[]);
    constrain(
        &mut p,
        id,
        0,
        180,
        "Lock",
        &["poser.MetaBodies.hips"],
        (STRONG, STRONG),
    );
    let pose = pose_from_text(&legacy_fixture::xml(0.0), "dp".into(), &p).unwrap();
    active_pose(&mut p, id, 10, 180, &[], pose.channels);
    p
}

/// A character whose head may collide with its parent (endorphin's `collidesWithParent`)
/// and already overlaps the neck at rest, like Back-Flip-To-Combo's: the placeholder's
/// head sphere grown and moved down onto the neck. Hung by the hips, with the soft
/// muscles a character starts with.
pub fn head_on_neck() -> Project {
    let mut p = new_scene("Head on neck", 1.0);
    let (skeleton, mut rig) = person_rig();
    let head = rig.body_index("head").unwrap();
    let shape = &mut rig.bodies[head].shapes[0];
    shape.geometry = Geometry::Sphere { radius: 0.12 };
    shape.collides_with_parent = true;
    rig.bodies[head].frame.position = [0.0, -0.02, 0.08];
    let id = add_person(
        &mut p,
        "nodder",
        (skeleton, rig),
        Vec3::new(0.0, 1.5, 0.0),
        0.0,
        &[],
    );
    constrain(
        &mut p,
        id,
        0,
        60,
        "Lock",
        &["nodder.MetaBodies.hips"],
        (STRONG, STRONG),
    );
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
            let (Some(cf), Some(prev)) = (cf, &sim.frame(frame - 1).unwrap()[ci]) else {
                continue;
            };
            let c = &project.characters[ci];
            let rig = c.physics.as_ref().unwrap();
            let low = cf
                .bodies
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.1.y.total_cmp(&b.1.1.y))
                .unwrap();
            let fast = cf
                .bodies
                .iter()
                .zip(&prev.bodies)
                .enumerate()
                .map(|(i, (a, b))| (i, a.1.distance(b.1) * project.settings.fps))
                .max_by(|a, b| a.1.total_cmp(&b.1))
                .unwrap();
            let stretch = c
                .skeleton
                .joints
                .iter()
                .enumerate()
                .filter_map(|(i, j)| {
                    Some((
                        i,
                        (cf.joints[i].1.distance(cf.joints[j.parent?].1)
                            - Vec3::from(j.offset).length())
                        .abs(),
                    ))
                })
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

/// Relative nudges to a scene's inputs (a push's speed, a car's), for surveying how
/// much a test's numbers move when the scene changes very slightly. Other platforms'
/// maths libraries can round `sin`, `atan2` and the like differently in the last bit,
/// and a chaotic moment (a foot landing, an arm whipping round) turns that into a
/// different outcome; an assertion that holds across these nudges, with margin, doesn't
/// hang on those bits. 0 first, then ±1, ±2, … × `NM_NUDGE_STEP` (1e-4) up to
/// `NM_NUDGES` (4) steps each way: 9 runs by default.
pub fn nudges() -> Vec<f32> {
    let var = |name: &str| std::env::var(name).ok().and_then(|s| s.parse().ok());
    let (n, step): (i32, f32) = (
        var("NM_NUDGES").unwrap_or(4.0) as i32,
        var("NM_NUDGE_STEP").unwrap_or(1e-4),
    );
    std::iter::once(0.0)
        .chain((1..=n).flat_map(|k| [k as f32 * step, -(k as f32) * step]))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A stand-in by name, or an endorphin scene by path (`.ens`) cut to its first
    /// `NM_SECONDS` (`seconds` if unset), for the debugging aids below.
    fn named_project(name: &str, seconds: f32) -> Project {
        match name {
            "tackle" => tackle(),
            "balcony" => balcony(),
            "catch_rope" => catch_rope(),
            "gymnast_rings" => gymnast_rings(),
            "bendy_stick" => bendy_stick(),
            "gun_drop" => gun_drop(),
            "heli_leap" => heli_leap(),
            "rotating_floor" => rotating_floor(),
            "turntable" => turntable(),
            "cylinders" => cylinders(),
            "cylinder_pile" => cylinder_pile(),
            "tilting_floor" => tilting_floor(),
            "roof_edge" => roof_edge(),
            "cut_rope" => cut_rope(),
            "car_hit" => car_hit(),
            "heavy_join" => heavy_join(),
            "bonnet_slide" => bonnet_slide(),
            "car_carry" => car_carry(),
            "backflip" => backflip(true),
            "backflip_passive" => backflip(false),
            "curl_hand_back" => curl_hand_back(),
            "arms_out" => arms_out(SpringCap::Limbs),
            "arms_out_bodies" => arms_out(SpringCap::Bodies),
            "unnamed_ankles" => unnamed_ankles(),
            "mid_air" => mid_air(false, true),
            "mid_air_limp" => mid_air(false, false),
            "mid_air_t_pose" => mid_air(true, true),
            "mid_air_t_pose_limp" => mid_air(true, false),
            "legacy_pose" => legacy_pose(),
            "balance" => balance_push(1.5, Vec3::Z, true),
            "balance_step" => balance_push(1.5, -Vec3::Z, true),
            "balance_steps" => balance_push(HARD_PUSHES[1], Vec3::Z, true),
            "stagger" => pushed(Some("Stagger"), HARD_PUSHES[1], Vec3::Z),
            "catch_fall" => catch_fall_shove(Vec3::new(0.0, 0.0, 3.5), true),
            "look_and_reach" => reach_for(Vec3::new(0.4, 1.8, 1.2), Vec3::ZERO, true),
            "look_and_reach_moving" => {
                reach_for(Vec3::new(-0.6, 1.5, 1.0), Vec3::new(1.5, 0.0, 0.0), true)
            }
            path if path.ends_with(".ens") => {
                let mut project = crate::io::endorphin::import_scene(std::path::Path::new(path))
                    .unwrap()
                    .project;
                let seconds = std::env::var("NM_SECONDS")
                    .ok()
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(seconds);
                let frames = (seconds * project.settings.fps) as i32;
                project.settings.range.end = project
                    .settings
                    .range
                    .end
                    .min(project.settings.range.start.saturating_add(frames));
                project
            }
            _ => panic!("expected a stand-in's name or an .ens scene, got {name:?}"),
        }
    }

    /// `NM_TRACE=<stand-in> cargo test --release trace_stand_in -- --ignored --nocapture`
    /// (or `NM_TRACE=<path to an .ens scene>`: its first two seconds, or `NM_SECONDS`)
    #[test]
    #[ignore = "debugging aid"]
    fn trace_stand_in() {
        // Runs with every `--ignored` sweep; only does something when asked to.
        let Ok(name) = std::env::var("NM_TRACE") else {
            return;
        };
        let describe_only = name.starts_with("describe:");
        let project = named_project(name.trim_start_matches("describe:"), 2.0);
        if describe_only {
            let report = sweep(&project);
            print!("{report}\n{}{}", report.places(), describe(&project));
            return;
        }
        trace(&project);
    }

    /// `NM_DUMP=<stand-in or .ens path> cargo test --release dump_scene_motion -- --ignored --nocapture`
    ///
    /// Simulates a scene and writes its motion as CSV, for comparing it with a
    /// reference (endorphin's own run, exported or measured) outside the tests. Into
    /// `NM_DUMP_OUT` (default `target/dump/<scene name>`):
    ///
    /// - `bodies.csv`: per frame, character and rig body, the body's world position
    ///   (m) and rotation (quaternion x, y, z, w) and the character's simulation level;
    /// - `joints.csv`: the same for every skeleton joint, characters without a rig
    ///   included (they follow their animation). This is the pose the viewport draws
    ///   and exports write: while a transition blends back to animation the joints
    ///   are blended and the bodies aren't;
    /// - `rig.csv`: every body's mass and its frame relative to the joint it hangs
    ///   from, to place bodies on the joints;
    /// - `shapes.csv`: every collision shape, relative to its body, for drawing.
    ///
    /// `NM_SECONDS` sets how long to simulate from the start of the range, past its
    /// end if need be (endorphin runs a scene for its own number of ticks, which can be
    /// longer than the playback range); the whole range by default.
    #[test]
    #[ignore = "debugging aid"]
    fn dump_scene_motion() {
        use std::fmt::Write as _;
        let Ok(name) = std::env::var("NM_DUMP") else {
            return;
        };
        let mut project = named_project(&name, f32::MAX);
        if let Some(seconds) = std::env::var("NM_SECONDS")
            .ok()
            .and_then(|s| s.parse::<f32>().ok())
        {
            let range = &mut project.settings.range;
            range.end = range.start + (seconds * project.settings.fps).round() as i32;
        }
        let out = std::path::PathBuf::from(std::env::var("NM_DUMP_OUT").unwrap_or_else(|_| {
            format!(
                "{}/target/dump/{}",
                env!("CARGO_MANIFEST_DIR"),
                project.settings.name
            )
        }));
        std::fs::create_dir_all(&out).unwrap();
        let (sim, elapsed) = simulate(&project);
        let range = project.settings.range;
        let fps = project.settings.fps;
        let level_name = |level: Option<SimLevel>| match level {
            Some(SimLevel::Animated) => "animated",
            Some(SimLevel::CollisionOnly) => "collision_only",
            Some(SimLevel::CollisionWithMomentum) => "collision_with_momentum",
            Some(SimLevel::Simulated) => "simulated",
            None => "no_rig",
        };
        let row = |csv: &mut String, head: &str, (q, p): (Quat, Vec3)| {
            writeln!(
                csv,
                "{head},{:.5},{:.5},{:.5},{:.6},{:.6},{:.6},{:.6}",
                p.x, p.y, p.z, q.x, q.y, q.z, q.w
            )
            .unwrap();
        };
        const POSE: &str = "x,y,z,qx,qy,qz,qw";
        let mut bodies = format!("frame,seconds,character,level,body,{POSE}\n");
        let mut joints = format!("frame,seconds,character,level,joint,parent,{POSE}\n");
        for frame in range.start..=range.end {
            let state = sim.frame(frame).unwrap();
            let poses = project.evaluate(frame as f32);
            let t = (frame - range.start) as f32 / fps;
            for (ci, c) in project.characters.iter().enumerate() {
                let cf = state[ci].as_ref();
                let level = level_name(cf.map(|cf| cf.level));
                let at = match cf {
                    Some(cf) => cf.joints.clone(),
                    None => world_transforms(&c.skeleton, &poses[ci]),
                };
                for (j, joint) in c.skeleton.joints.iter().enumerate() {
                    let parent = joint
                        .parent
                        .map_or("", |p| c.skeleton.joints[p].name.as_str());
                    let head = format!("{frame},{t:.4},{},{level},{},{parent}", c.name, joint.name);
                    row(&mut joints, &head, at[j]);
                }
                let (Some(cf), Some(rig)) = (cf, &c.physics) else {
                    continue;
                };
                for (body, at) in rig.bodies.iter().zip(&cf.bodies) {
                    let head = format!("{frame},{t:.4},{},{level},{}", c.name, body.name);
                    row(&mut bodies, &head, *at);
                }
            }
        }
        let mut bodies_on_joints = format!("character,body,joint,mass,{POSE}\n");
        let mut shapes =
            format!("character,body,mass,shape,geometry,radius,length,sx,sy,sz,{POSE}\n");
        let frame_of = |f: &crate::document::physics::Frame| {
            (Quat::from_array(f.rotation), Vec3::from(f.position))
        };
        for c in &project.characters {
            let Some(rig) = &c.physics else { continue };
            for body in &rig.bodies {
                let joint = &c.skeleton.joints[body.joint].name;
                let head = format!("{},{},{joint},{}", c.name, body.name, body.mass);
                row(&mut bodies_on_joints, &head, frame_of(&body.frame));
                for (si, shape) in body.shapes.iter().enumerate() {
                    let (kind, r, l, size) = match shape.geometry {
                        Geometry::Sphere { radius } => ("sphere", radius, 0.0, [0.0; 3]),
                        Geometry::Capsule { radius, length } => {
                            ("capsule", radius, length, [0.0; 3])
                        }
                        Geometry::Cylinder { radius, length } => {
                            ("cylinder", radius, length, [0.0; 3])
                        }
                        Geometry::Box { size } => ("box", 0.0, 0.0, size),
                    };
                    let head = format!(
                        "{},{},{},{si},{kind},{r},{l},{},{},{}",
                        c.name, body.name, body.mass, size[0], size[1], size[2]
                    );
                    row(&mut shapes, &head, frame_of(&shape.frame));
                }
            }
        }
        for (file, text) in [
            ("bodies.csv", bodies),
            ("joints.csv", joints),
            ("rig.csv", bodies_on_joints),
            ("shapes.csv", shapes),
        ] {
            std::fs::write(out.join(file), text).unwrap();
        }
        println!(
            "{}: frames {}-{} at {fps} fps simulated in {elapsed:.0?}, written to {}",
            project.settings.name,
            range.start,
            range.end,
            out.display()
        );
    }

    fn check(project: Project) -> Simulation {
        let (sim, elapsed) = simulate(&project);
        let report = measure(&project, &sim, elapsed);
        println!(
            "{:<16} {:>4} frames in {:>6.0?}  {report}",
            project.settings.name,
            project.settings.range.len() + 1,
            report.elapsed
        );
        assert!(
            report.held(),
            "{}: {report}\n{}{}",
            project.settings.name,
            report.places(),
            describe(&project)
        );
        sim
    }

    /// Lowest body of a character at the last frame.
    fn resting_height(project: &Project, sim: &Simulation, name: &str) -> f32 {
        let ci = project
            .characters
            .iter()
            .position(|c| c.name == name)
            .unwrap();
        let frame = sim.frame(project.settings.range.end).unwrap()[ci]
            .as_ref()
            .unwrap();
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

    /// A one-body prop's world transform at every frame.
    fn prop_track(project: &Project, sim: &Simulation, name: &str) -> Vec<(Quat, Vec3)> {
        let ci = project
            .characters
            .iter()
            .position(|c| c.name == name)
            .unwrap();
        (0..=project.settings.range.end)
            .map(|f| sim.frame(f).unwrap()[ci].as_ref().unwrap().bodies[0])
            .collect()
    }

    /// How a prop ends: its centre, the angle of its axis (local Z) from upright,
    /// radians, and its fastest speed over the last half second, m/s (jitter at rest).
    fn settled(project: &Project, sim: &Simulation, name: &str) -> (Vec3, f32, f32) {
        let track = prop_track(project, sim, name);
        let fps = project.settings.fps;
        let (rotation, center) = *track.last().unwrap();
        let tilt = libm::acosf((rotation * Vec3::Z).dot(Vec3::Y).abs().min(1.0));
        let last = track.len() - (fps * 0.5) as usize;
        let jitter = track[last..]
            .windows(2)
            .map(|w| w[0].1.distance(w[1].1) * fps)
            .fold(0.0, f32::max);
        (center, tilt, jitter)
    }

    /// The lowest a prop's centre goes once it has first come within 1 mm of `rest`
    /// (sinking into what it lands on), m.
    fn lowest_after_landing(project: &Project, sim: &Simulation, name: &str, rest: f32) -> f32 {
        let track = prop_track(project, sim, name);
        let landed = track.iter().position(|t| t.1.y < rest + 0.001).unwrap_or(0);
        track[landed..]
            .iter()
            .map(|t| t.1.y)
            .fold(f32::MAX, f32::min)
    }

    /// Checks a prop that comes to rest: its centre's height (m) to a millimetre,
    /// whether it stands (axis upright) or lies (axis level) within half a degree,
    /// that it never sinks a millimetre below its rest, and that it is still.
    fn rests(project: &Project, sim: &Simulation, name: &str, height: f32, standing: bool) {
        let (centre, tilt, jitter) = settled(project, sim, name);
        let lowest = lowest_after_landing(project, sim, name, height);
        let lean = if standing {
            tilt
        } else {
            std::f32::consts::FRAC_PI_2 - tilt
        };
        println!(
            "{name:<12} centre y {:.4} m  tilt {:.2}°  lowest {lowest:.4} m  jitter {jitter:.4} m/s",
            centre.y,
            tilt.to_degrees()
        );
        assert!(
            (centre.y - height).abs() < 0.001,
            "{name} rests at {} m, not {height} m",
            centre.y
        );
        assert!(
            lean.to_degrees() < 0.5,
            "{name} leans {}°",
            lean.to_degrees()
        );
        assert!(lowest > height - 0.001, "{name} sinks to {lowest} m");
        assert!(jitter < 0.002, "{name} still moves at {jitter} m/s");
    }

    /// Lesson 8: cylinders collide as cylinders, with flat caps and a sharp rim.
    #[test]
    fn cylinders_rest_on_caps_and_sides() {
        let project = cylinders();
        let sim = check(project.clone());
        // Stood on its cap; as the capsule it used to be, its round end tipped it over.
        rests(&project, &sim, "standing", 0.25, true);
        rests(&project, &sim, "disc", 0.02, true);
        rests(&project, &sim, "on box", 0.7, true);
        rests(&project, &sim, "across box", 0.6, false);
        rests(&project, &sim, "lying", 0.1, false);
        // On the cap 3 cm in from the rim, where a round end would roll it off (a
        // ball's local Z starts level and it does not roll).
        rests(&project, &sim, "ball inside", 1.05, false);
        // Its centre 2 cm past the rim: the sharp edge throws it outwards to the ground.
        let (ball, _, _) = settled(&project, &sim, "ball outside");
        println!("ball outside ends at {ball}");
        assert!((ball.y - 0.05).abs() < 0.001 && ball.z > 0.3, "{ball}");

        // The log rolls straight along +Z on its side, at 2/3 of the 1 m/s it was given
        // (a solid cylinder rolling), without slowing, drifting or sinking.
        let track = prop_track(&project, &sim, "log");
        let fps = project.settings.fps;
        let (rotation, end) = *track.last().unwrap();
        let speed = (end.z - track[track.len() - 31].1.z) * fps / 30.0;
        let lowest = track.iter().map(|t| t.1.y).fold(f32::MAX, f32::min);
        let axis = rotation * Vec3::Z;
        println!("log ends at {end}, rolling at {speed:.4} m/s, axis {axis}, lowest {lowest:.4} m");
        assert!(end.z > 1.8, "log rolled only {} m", end.z);
        assert!((speed - 2.0 / 3.0).abs() < 0.02, "log rolls at {speed} m/s");
        assert!((end.x - 2.0).abs() < 0.005, "log drifted to x = {}", end.x);
        assert!(axis.x.abs() > 0.9999, "log turned: axis {axis}");
        assert!(
            lowest > 0.099 && (end.y - 0.1).abs() < 0.001,
            "log at {lowest} m"
        );
    }

    /// Cylinders on cylinders: caps stacked, a cap on a side, sides side by side.
    #[test]
    fn cylinder_pile_settles() {
        let project = cylinder_pile();
        let (sim, elapsed) = simulate(&project);
        let report = measure(&project, &sim, elapsed);
        let steps = project.settings.range.end as f32 / project.settings.fps * project.physics.rate;
        println!(
            "{report}  {:.0} µs per step",
            elapsed.as_secs_f32() * 1e6 / steps
        );
        assert!(report.held(), "{report}\n{}", report.places());
        for (name, height) in [
            ("disc0", 0.03),
            ("disc1", 0.09),
            ("disc2", 0.15),
            ("base", 0.03),
            ("post", 0.36),
            ("plate", 0.03),
        ] {
            rests(&project, &sim, name, height, true);
        }
        let groove = 0.1 + 0.03f32.sqrt();
        for (name, height) in [
            ("pin", 0.16),
            ("under0", 0.1),
            ("under1", 0.1),
            ("top", groove),
        ] {
            rests(&project, &sim, name, height, false);
        }
    }

    /// Without a ground plane, things may go as low as the scenery takes them: a floor
    /// tilting below where the ground would be, or the void beside a roof.
    #[test]
    fn tilting_floor_carries_character_below_zero() {
        let project = tilting_floor();
        let sim = check(project.clone());
        let height = resting_height(&project, &sim, "dancer");
        assert!(height < -1.0, "the dancer only went down to {height} m");
    }

    #[test]
    fn jumper_falls_past_the_roof_edge() {
        let project = roof_edge();
        let sim = check(project.clone());
        let height = resting_height(&project, &sim, "jumper");
        assert!(height < -5.0, "the jumper only fell to {height} m");
    }

    /// The check that replaces "below the ground" in scenes without one still sees a
    /// body fall through the floor: Rotating Floor's dancer, started under the slab.
    #[test]
    fn sweep_sees_a_body_under_the_floor_without_ground() {
        let mut project = rotating_floor();
        let dancer = project.characters.last().unwrap().id;
        let track = project.tracks_of(dancer).next().unwrap().id;
        let start = project
            .clips
            .iter_mut()
            .find(|c| c.props.track == track)
            .unwrap();
        for channel in &mut start.channels {
            if channel.joint == "hips" && channel.property == Property::PosY {
                channel.keys[0].value = -1.0;
            }
        }
        let report = sweep(&project);
        assert!(
            !report.ok() && report.under > 0.5,
            "a dancer under the floor passed: {report}"
        );
    }

    /// Catch Rope: a force naming the rope's end, on the environment's track, pushes the
    /// rope and not the environment, whose massless roof stays put; the link a sever
    /// event cuts falls away without counting as a joint coming apart.
    #[test]
    fn force_on_another_track_pushes_what_it_names() {
        let project = cut_rope();
        let sim = check(project.clone());
        let roof = |frame: i32| sim.frame(frame).unwrap()[0].as_ref().unwrap().bodies[0].1;
        let moved = roof(120).distance(roof(0));
        assert!(moved < 1e-4, "the massless roof moved {moved} m");
        let end = |frame: i32| sim.frame(frame).unwrap()[1].as_ref().unwrap().bodies[9].1;
        let swing = end(30).z - end(0).z;
        assert!(swing > 0.05, "the rope's end only swung {swing} m");
        let fell = end(60).y - end(120).y;
        assert!(fell > 2.0, "the cut end only fell {fell} m");
    }

    /// A rope hanging straight down from a Lock with a heavy weight on the end stays
    /// still (with full joint warm starts it breathed until it exploded).
    #[test]
    fn hanging_weight_stays_still() {
        for (tip_mass, seg_mass) in [(20.0, 0.15), (80.0, 0.15), (20.0, 0.05)] {
            let mut p = new_scene("Hanging weight", 2.0);
            let limits = Limits {
                twist_min: -0.5,
                twist_max: 0.5,
                swing1: 1.2,
                swing2: 1.2,
            };
            let tip = Piece::at(
                Geometry::Sphere { radius: 0.15 },
                tip_mass,
                Quat::IDENTITY,
                Vec3::new(0.0, -0.15, 0.0),
            );
            let rig = chain(
                Vec3::new(0.0, 4.0, 0.0),
                8,
                0.3,
                0.02,
                seg_mass,
                Some(tip),
                limits,
                Gains {
                    stiffness: 0.5,
                    damping: 0.05,
                    force_limit: 5.0,
                },
            );
            let rope = add_prop(
                &mut p,
                "rope",
                CharacterKind::Prop,
                rig,
                SimLevel::Simulated,
            );
            constrain(
                &mut p,
                rope,
                0,
                120,
                "Lock",
                &["rope.MetaBodies.seg0"],
                (STRONG, Vec3::ZERO),
            );
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
        let limits = Limits {
            twist_min: -0.5,
            twist_max: 0.5,
            swing1: 1.2,
            swing2: 1.2,
        };
        let rig = chain(
            Vec3::new(0.0, 3.0, 0.0),
            4,
            0.3,
            0.02,
            0.5,
            None,
            limits,
            Gains {
                stiffness: 0.5,
                damping: 0.05,
                force_limit: 5.0,
            },
        );
        let rope = add_prop(
            &mut p,
            "rope",
            CharacterKind::Prop,
            rig,
            SimLevel::Simulated,
        );
        // Bent sideways, so without the lock the second segment would swing down.
        motion(
            &mut p,
            rope,
            "Start pose",
            0,
            60,
            false,
            vec![held("seg1", Property::RotZ, 60.0)],
        );
        constrain(
            &mut p,
            rope,
            0,
            60,
            "Lock",
            &["rope.MetaBodies.seg0"],
            (STRONG, STRONG),
        );
        constrain(
            &mut p,
            rope,
            0,
            60,
            "LockWithParent",
            &["rope.MetaBodies.seg1"],
            (STRONG, STRONG),
        );
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
        assert!(
            !report.nan && report.fastest < 40.0 && report.stretch < 0.1,
            "{report}"
        );
    }

    /// Back flip: per frame, the hips' turn about world X so far (degrees, unwrapped;
    /// negative is backwards for a character facing +Z), and the lowest body centre.
    fn flip_track(project: &Project, sim: &Simulation) -> Vec<(f32, f32)> {
        let ci = project
            .characters
            .iter()
            .position(|c| c.name == "flipper")
            .unwrap();
        let hips = project.characters[ci]
            .physics
            .as_ref()
            .unwrap()
            .body_index("hips")
            .unwrap();
        let frame = |f: i32| sim.frame(f).unwrap()[ci].as_ref().unwrap();
        let start = frame(0).bodies[hips].0;
        let (mut turned, mut last) = (0.0f32, 0.0f32);
        (0..=project.settings.range.end)
            .map(|f| {
                let cf = frame(f);
                // The hips' starting up direction, carried along.
                let up = cf.bodies[hips].0 * start.inverse() * Vec3::Y;
                let angle = libm::atan2f(up.z, up.y);
                let step = (angle - last + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU)
                    - std::f32::consts::PI;
                turned += step;
                last = angle;
                (
                    turned.to_degrees(),
                    cf.bodies.iter().map(|b| b.1.y).fold(f32::MAX, f32::min),
                )
            })
            .collect()
    }

    #[test]
    fn backflip_drive_flips_the_character_off_the_box() {
        for balance in [true, false] {
            let project = backflip(balance);
            let sim = check(project.clone());
            let track = flip_track(&project, &sim);
            // Off the box: the lowest body centre 10 cm below its top.
            let leaves = track.iter().position(|&(_, low)| low < 2.4).unwrap() as i32;
            let lands = track.iter().position(|&(_, low)| low < 0.15).unwrap() as i32;
            let ci = project
                .characters
                .iter()
                .position(|c| c.name == "flipper")
                .unwrap();
            let rig = project.characters[ci].physics.as_ref().unwrap();
            let end = sim.frame(project.settings.range.end).unwrap()[ci]
                .as_ref()
                .unwrap();
            let hips = end.bodies[rig.body_index("hips").unwrap()].1;
            println!(
                "back flip (balance {balance}): {:.0}° at frame 246, {:.0}° at 297, leaves the box at frame {leaves}, lands at {lands}, \
                 {:.0}° in all, hips end at {hips:.2}",
                track[246].0,
                track[297].0,
                track.last().unwrap().0
            );
            assert!(
                track.last().unwrap().1 < 0.15 && hips.y < 0.5,
                "not on the ground: hips at {hips}"
            );
            assert!(hips.z.abs() > 0.6, "still on the box: hips at {hips}");
            if balance {
                assert!(
                    track[246].0.abs() < 5.0,
                    "leaning {}° before the drive",
                    track[246].0
                );
                assert!(
                    leaves > 246,
                    "left the box at frame {leaves}, before the drive"
                );
                assert!(
                    track.last().unwrap().0 < -180.0,
                    "turned {}°",
                    track.last().unwrap().0
                );
                assert!(hips.z < -0.6, "landed in front of the box: {hips}");
            }
        }
    }

    /// Angle (degrees) between the bones either side of `joint`, from its parent to it
    /// and from it to `child`, at a frame.
    fn bend(project: &Project, sim: &Simulation, frame: i32, joint: &str, child: &str) -> f32 {
        let skeleton = &project.characters[0].skeleton;
        let at = |name: &str| {
            sim.frame(frame).unwrap()[0].as_ref().unwrap().joints
                [skeleton.joint_index(name).unwrap()]
            .1
        };
        let j = skeleton.joint_index(joint).unwrap();
        let parent = &skeleton.joints[skeleton.joints[j].parent.unwrap()].name;
        let (a, b, c) = (at(parent), at(joint), at(child));
        (b - a).angle_between(c - b).to_degrees()
    }

    /// Overlapping active poses each drive the joints they name (lesson 19). The
    /// elbows' event starts last; it used to drive the whole body, so the knees went
    /// back to rest when it started (0° at frame 150, with the elbows at 59°). Now
    /// 59° and 60°.
    #[test]
    fn overlapping_active_poses_each_drive_their_own_joints() {
        let project = split_poses();
        let sim = check(project.clone());
        let knees = bend(&project, &sim, 150, "l_knee", "l_ankle");
        let elbows = bend(&project, &sim, 150, "l_elbow", "l_wrist");
        let early = bend(&project, &sim, 18, "l_elbow", "l_wrist");
        println!(
            "split poses: knees {knees:.0}°, elbows {elbows:.0}° at frame 150; elbows {early:.0}° at 18"
        );
        assert!((45.0..75.0).contains(&knees), "knees bent {knees}°");
        assert!((45.0..75.0).contains(&elbows), "elbows bent {elbows}°");
        assert!(early < 15.0, "elbows bent {early}° before their event");
    }

    /// The left arm's elevation at a frame: degrees above horizontal, shoulder to wrist.
    fn arm_elevation(project: &Project, sim: &Simulation, frame: i32) -> f32 {
        let skeleton = &project.characters[0].skeleton;
        let at = |name: &str| {
            sim.frame(frame).unwrap()[0].as_ref().unwrap().joints
                [skeleton.joint_index(name).unwrap()]
            .1
        };
        let arm = at("l_wrist") - at("l_shoulder");
        libm::atan2f(arm.y, libm::hypotf(arm.x, arm.z)).to_degrees()
    }

    /// A clavicle holds the whole arm up (lesson 27). With the muscle cap
    /// counting only the joint's two bodies, the 0.4 kg clavicle's and the shoulder's
    /// muscles are capped far below what the arm needs: posed straight out, the arms
    /// stay hanging (-89° at frame 120). Counting the arm they move, the arms come up
    /// over the 0.5 s ease-in (-32° at frame 30) and hold 9° below horizontal.
    #[test]
    fn a_clavicle_holds_the_arm_out() {
        let elevation = |cap| {
            let project = arms_out(cap);
            let sim = check(project.clone());
            (
                arm_elevation(&project, &sim, 30),
                arm_elevation(&project, &sim, 120),
            )
        };
        let (bodies, limbs) = (elevation(SpringCap::Bodies), elevation(SpringCap::Limbs));
        println!(
            "arms out: elevation at frames 30 / 120 with the two-body cap {:.0}° / {:.0}°, \
             with the limb cap {:.0}° / {:.0}°",
            bodies.0, bodies.1, limbs.0, limbs.1
        );
        assert!(limbs.1 > -20.0, "arms at {}° with the limb cap", limbs.1);
        assert!(
            bodies.1 < limbs.1 - 45.0,
            "two-body cap {}°, limb cap {}°",
            bodies.1,
            limbs.1
        );
        // Eased in over 0.5 s: on their way up at a quarter second.
        assert!(
            (-75.0..-10.0).contains(&limbs.0),
            "arms at {}° at frame 30",
            limbs.0
        );
    }

    /// How far the hands' centres are from where the pose puts them relative to the
    /// chest (m, the worse hand), averaged over frames 60-120, after the reach.
    fn reach_error(project: &Project, sim: &Simulation) -> f32 {
        let c = &project.characters[0];
        let rig = c.physics.as_ref().unwrap();
        let mut pose = Pose::rest(c.skeleton.joints.len());
        for side in ["l", "r"] {
            let j = c.skeleton.joint_index(&format!("{side}_shoulder")).unwrap();
            pose.joints[j][0] = -90.0;
        }
        let wanted = rig_bodies(rig, &world_transforms(&c.skeleton, &pose));
        let chest = rig.body_index("chest").unwrap();
        let relative = |bodies: &[(Quat, Vec3)], hand: usize| {
            bodies[chest].0.inverse() * (bodies[hand].1 - bodies[chest].1)
        };
        let frames = 60..=120;
        let total: f32 = frames
            .clone()
            .map(|frame| {
                let now = &sim.frame(frame).unwrap()[0].as_ref().unwrap().bodies;
                ["l", "r"]
                    .iter()
                    .map(|side| {
                        let hand = rig.body_index(&format!("{side}_wrist")).unwrap();
                        (relative(now, hand) - relative(&wanted, hand)).length()
                    })
                    .fold(0.0, f32::max)
            })
            .sum();
        total / frames.count() as f32
    }

    /// Virtual muscles (`virtual_reach`): with its shoulders too weak to hold the arms
    /// up under the two-body cap, the reacher's hands swing about 0.3 m from where the
    /// pose puts them (0.12-0.48 m over frames 60-120). Chest-to-hand virtual muscles
    /// at 20 N/m, 2 N·s/m and 20 N per unit of strength (strength 10) bring them within
    /// 5 cm and hold them there, with no joint pulled apart. endorphin's own gains, 1 in
    /// every field, are too weak to lift the arms and brake them instead (0.6 m: the
    /// orientation drive's damping resists the arm turning). A pose that doesn't name
    /// the virtual muscles leaves them off: the same motion, exactly, as without them.
    #[test]
    fn virtual_muscles_pull_the_hands_to_the_pose() {
        let gains = |stiffness, damping, force_limit| Gains {
            stiffness,
            damping,
            force_limit,
        };
        let stand_in = (gains(20.0, 2.0, 20.0), gains(1.0, 1.0, 1.0));
        let endorphins = (gains(1.0, 1.0, 1.0), gains(1.0, 1.0, 1.0));
        let run = |vm: Option<(Gains, Gains)>, named: bool| {
            let project = virtual_reach(vm, named);
            let sim = check(project.clone());
            let (stretch, at) = joint_stretch(&project, &sim, "reacher", 0..=120, |_| true);
            (reach_error(&project, &sim), stretch, at, sim)
        };
        let (without, without_stretch, _, plain) = run(None, true);
        let (with, stretch, at, _) = run(Some(stand_in), true);
        let (stock, stock_stretch, _, _) = run(Some(endorphins), true);
        let (unnamed, _, _, off) = run(Some(stand_in), false);
        println!(
            "virtual reach, hand error over frames 60-120: {without:.3} m without virtual \
             muscles, {with:.3} m with them, {stock:.3} m with endorphin's gains; max joint \
             stretch {stretch:.4} m at {at} ({without_stretch:.4} m without, \
             {stock_stretch:.4} m with endorphin's gains)"
        );
        assert!(
            without > 0.15,
            "hands {without} m off without virtual muscles"
        );
        assert!(with < 0.1, "hands {with} m off with them");
        assert!(
            with < 0.5 * without,
            "hands {with} m off with them, {without} without"
        );
        assert!(stretch < 0.01, "a joint stretched {stretch} m at {at}");
        assert!(stock_stretch < 0.01, "a joint stretched {stock_stretch} m");
        assert_eq!(
            unnamed, without,
            "a pose that doesn't name them leaves them off"
        );
        for frame in [30, 120] {
            assert_eq!(
                off.frame(frame).unwrap()[0].as_ref().unwrap().bodies,
                plain.frame(frame).unwrap()[0].as_ref().unwrap().bodies
            );
        }
    }

    /// A joint no running pose names follows the pose that drives its limb (the lesson
    /// on active poses): the ankles bend with the knees' event that stores them.
    #[test]
    fn an_unnamed_joint_follows_its_limbs_pose() {
        let project = unnamed_ankles();
        let sim = check(project.clone());
        let knee = bend(&project, &sim, 100, "l_knee", "l_ankle");
        let rest = bend(&project, &sim, 5, "l_ankle", "l_toe");
        let ankle = bend(&project, &sim, 100, "l_ankle", "l_toe");
        println!(
            "unnamed ankles: knees {knee:.0}°, ankles {:.0}° from rest",
            ankle - rest
        );
        assert!((45.0..75.0).contains(&knee), "knees bent {knee}°");
        assert!(
            (ankle - rest).abs() > 20.0,
            "ankles turned {}°",
            ankle - rest
        );
    }

    /// A legacy pose's muscle targets drive the joints to their angles as an active
    /// pose. Before, the pose didn't import at all ("stores muscle (effector) targets
    /// instead of joint transforms"). Now, held for two seconds: knees 57° (the target
    /// is 1.0 rad, 57°), elbows 45° (0.8 rad, 46°), hips flexed 28° (0.5 rad, 29°).
    #[test]
    fn legacy_pose_drives_joints_to_its_muscle_targets() {
        let project = legacy_pose();
        let sim = check(project.clone());
        let skeleton = &project.characters[0].skeleton;
        let joints = |f: i32| &sim.frame(f).unwrap()[0].as_ref().unwrap().joints;
        let at = |f: i32, name: &str| joints(f)[skeleton.joint_index(name).unwrap()];
        // The thigh's angle from straight down in the hips' frame, degrees.
        let flexion = |f: i32| {
            let (hips, _) = at(f, "hips");
            let thigh = hips.inverse() * (at(f, "l_knee").1 - at(f, "l_hip").1);
            thigh.angle_between(-Vec3::Y).to_degrees()
        };
        let knees = bend(&project, &sim, 150, "l_knee", "l_ankle");
        let right = bend(&project, &sim, 150, "r_knee", "r_ankle");
        let elbows = bend(&project, &sim, 150, "l_elbow", "l_wrist");
        let hips = flexion(150);
        let early = bend(&project, &sim, 8, "l_knee", "l_ankle");
        println!(
            "legacy pose: knees {knees:.0}° (right {right:.0}°), elbows {elbows:.0}°, hips {hips:.0}° \
             at frame 150; knees {early:.0}° at 8"
        );
        assert!((47.0..67.0).contains(&knees), "knees bent {knees}°");
        assert!((knees - right).abs() < 2.0, "knees {knees}° and {right}°");
        assert!((36.0..56.0).contains(&elbows), "elbows bent {elbows}°");
        assert!((19.0..39.0).contains(&hips), "hips flexed {hips}°");
        // Forwards: the knee ends up in front of the hip (the character faces +Z).
        assert!(at(150, "l_knee").1.z > at(150, "l_hip").1.z + 0.1);
        assert!(early < 10.0, "knees bent {early}° before the event");
    }

    /// A head that may collide with its parent but overlaps the neck at rest isn't
    /// thrown off it when the simulation starts (lesson 19). It turned 44° against the
    /// neck within the first second; now 2.6°.
    #[test]
    fn a_shape_overlapping_its_parent_at_rest_does_not_push_it() {
        let project = head_on_neck();
        let sim = check(project.clone());
        let rig = project.characters[0].physics.as_ref().unwrap();
        let (head, neck) = (
            rig.body_index("head").unwrap(),
            rig.body_index("neck").unwrap(),
        );
        // The head's turn relative to the neck.
        let rot = |f: i32| {
            let bodies = &sim.frame(f).unwrap()[0].as_ref().unwrap().bodies;
            bodies[neck].0.inverse() * bodies[head].0
        };
        let turned = (0..=60)
            .map(|f| rot(0).angle_between(rot(f)).to_degrees())
            .fold(0.0f32, f32::max);
        println!("head on neck: turned {turned:.1}° at most");
        assert!(turned < 5.0, "head turned {turned}°");
    }

    /// endorphin's Car Hit stretched a joint by 6 cm. The stand-in's character takes the
    /// hit within limits, so this guards against regressions rather than reproducing it.
    /// The keyed car drives on and runs the pedestrian over: over 41 runs with its speed
    /// nudged by up to ±2e-3 (`NM_SURVEY=car_hit nudge_survey`) the worst joint opens
    /// 0.56 to 2.4 cm (1.95 to 12.7 cm, 5 runs over 5 cm, before lesson 29).
    #[test]
    fn car_hit_keeps_joints_together() {
        check(car_hit());
    }

    /// A fingerprint of everything a simulation produced: every frame's joints, bodies
    /// and level for every character, bit for bit (64-bit FNV-1a).
    fn fingerprint(sim: &Simulation, frames: std::ops::RangeInclusive<i32>) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325u64;
        let mut eat = |bits: u32| {
            for byte in bits.to_le_bytes() {
                hash = (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3);
            }
        };
        for frame in frames {
            for c in sim.frame(frame).unwrap().iter().flatten() {
                eat(c.level as u32);
                for (q, v) in c.joints.iter().chain(&c.bodies) {
                    q.to_array()
                        .into_iter()
                        .chain(v.to_array())
                        .for_each(|x| eat(x.to_bits()));
                }
            }
        }
        hash
    }

    /// The simulation is bit-identical on every platform (lesson 30): CI runs this on
    /// Linux, Windows and macOS, in debug, and it runs here in release too. Three
    /// stand-ins that turn a last-bit difference into a different outcome within a
    /// second or two (a car running a character over, Explosion's landing, Stagger
    /// stepping after a hard push), each hashed whole. A change to the physics changes
    /// these on purpose: run the test, check the other tests still pass, and copy the
    /// new values it prints. If one platform alone disagrees, something in the physics
    /// is rounding the platform's way again (a maths function not from `libm`, say).
    #[test]
    fn simulations_are_bit_identical_everywhere() {
        let scenes = [
            ("car hit", car_hit(), 0x13fe_6b43_2574_a7a3u64),
            ("explosion", mid_air(true, true), 0x946d_7389_3aab_f2fd),
            (
                "stagger",
                pushed(Some("Stagger"), 2.8, Vec3::Z),
                0xe2ab_f4f1_7094_e2cc,
            ),
        ];
        let mut wrong = Vec::new();
        for (name, project, expected) in scenes {
            let (sim, _) = simulate(&project);
            let range = project.settings.range;
            let hash = fingerprint(&sim, range.start..=range.end);
            println!("{name}: {hash:#018x}");
            if hash != expected {
                wrong.push(format!("{name}: {hash:#018x}, expected {expected:#018x}"));
                // Frame by frame, to compare with a platform that agrees and find
                // where they part.
                let frames: Vec<String> = (range.start..=range.end)
                    .map(|f| format!("{:04x}", fingerprint(&sim, f..=f) & 0xffff))
                    .collect();
                println!("{name} by frame: {}", frames.join(" "));
            }
        }
        assert!(wrong.is_empty(), "{}", wrong.join("; "));
    }

    /// Heavy join (lesson 20): the car falls behind its carrier as it drives off (23 cm
    /// at frame 4) and runs on past the stop (23 cm), as 1e5 N allows, but catches up
    /// within 7 frames each time. It used to stay 15-18 cm behind for the whole drive
    /// and the second after the stop: a lock at its strength only ever closed the gap
    /// at strength × substep / mass, 0.3 m/s here.
    #[test]
    fn heavy_join_catches_up() {
        let project = heavy_join();
        let (sim, _) = simulate(&project);
        let gaps = heavy_join_gaps(&project, &sim);
        let worst = |frames: std::ops::RangeInclusive<usize>| {
            gaps[frames].iter().copied().fold(0.0f32, f32::max)
        };
        println!(
            "heavy join: gap {:.3} m at frame 5, {:.3} at 15, {:.3} worst over 20-60, {:.3} worst after the stop, {:.3} over 80-120",
            gaps[5],
            gaps[15],
            worst(20..=60),
            worst(61..=120),
            worst(80..=120)
        );
        assert!(worst(20..=60) < 0.01, "behind while driving: {gaps:?}");
        assert!(worst(80..=120) < 0.01, "not back after the stop: {gaps:?}");
    }

    /// Bonnet slide (lesson 21): the box, moving at 4 m/s against the car, is stopped
    /// dead as it lands, as endorphin's Car Hit pedestrian is (-0.3 m/s 3 frames after
    /// landing, -0.05 after 6), and the car then drags it along. With friction capped
    /// at 100 N per contact, as before, it slid on at 3.7 m/s and only slowly lost
    /// that (2.7 m/s 24 frames after landing).
    #[test]
    fn bonnet_slide_stops_the_box() {
        let mut after = Vec::new();
        for contact in [ContactModel::Coulomb, ContactModel::ForceCapped] {
            let mut project = bonnet_slide();
            project.physics.contact = contact;
            let (sim, _) = simulate(&project);
            let (speeds, landed) = bonnet_slide_track(&project, &sim);
            let at = |f: i32| speeds[f as usize];
            println!(
                "bonnet slide ({contact:?}): lands at frame {landed}; along the road {:.2} m/s before, \
                 {:.2} 3 frames after, {:.2} 6 after, {:.2} 12 after, {:.2} 24 after",
                at(landed - 2),
                at(landed + 3),
                at(landed + 6),
                at(landed + 12),
                at(landed + 24)
            );
            after.push(at(landed + 6));
        }
        assert_eq!(bonnet_slide().physics.contact, ContactModel::Coulomb);
        assert!(
            after[0].abs() < 1.0,
            "still {} m/s 6 frames after landing",
            after[0]
        );
    }

    /// What the gun drop measures: the lowest finger centre above the ground, m, how far
    /// either hand got from where its join holds it on the prop, m, and how far it turned
    /// away from the prop, degrees.
    fn gun_drop_numbers(project: &Project, sim: &Simulation) -> (f32, f32, f32) {
        let holder = project
            .characters
            .iter()
            .position(|c| c.name == "holder")
            .unwrap();
        let gun = project
            .characters
            .iter()
            .position(|c| c.name == "gun")
            .unwrap();
        let rig = project.characters[holder].physics.as_ref().unwrap();
        let body = |name: &str| rig.body_index(name).unwrap();
        let at = |frame: i32, ci: usize, bi: usize| {
            sim.frame(frame).unwrap()[ci].as_ref().unwrap().bodies[bi]
        };
        let (mut low, mut gap, mut turn) = (f32::MAX, 0.0f32, 0.0f32);
        for side in ["l", "r"] {
            let hand = body(&format!("{side}_wrist"));
            let finger = body(&format!("{side}_fingers"));
            let held = |frame: i32| {
                let (gr, gp) = at(frame, gun, 0);
                let (hr, hp) = at(frame, holder, hand);
                (gr.inverse() * (hp - gp), gr.inverse() * hr)
            };
            let (start, start_turn) = held(0);
            for frame in 1..=project.settings.range.end {
                low = low.min(at(frame, holder, finger).1.y);
                let (offset, rotation) = held(frame);
                gap = gap.max(offset.distance(start));
                turn = turn.max((start_turn.inverse() * rotation).to_scaled_axis().length());
            }
        }
        (low, gap, turn.to_degrees())
    }

    /// Walk With Gun's pattern: a character holding a 550 kg prop drops with it. The
    /// fingers hang below the prop's underside, so on landing they are between the
    /// hand that holds the prop and the ground. Contacts are soft at a fixed frequency,
    /// which makes their stiffness go with the finger's own 0.39 kg rather than with
    /// what presses on it: the finger sank 1.4 cm into the ground (2.5 cm in the scene
    /// with Coulomb friction). Contacts against the ground now join the direct solve
    /// of a group with a lock in it (lesson 25).
    /// Over the nudged runs (the prop's mass ±4e-4) the finger stays above the ground.
    #[test]
    fn gun_drop_keeps_the_fingers_out_of_the_ground() {
        for nudge in nudges() {
            let project = gun_drop_nudged(nudge);
            let (sim, elapsed) = simulate(&project);
            let report = measure(&project, &sim, elapsed);
            let (low, gap, turn) = gun_drop_numbers(&project, &sim);
            println!(
                "Gun drop ({nudge:+.0e}): {report}\n{}  lowest finger {low:.4} m, hand off the prop by {gap:.4} m and turned {turn:.1}°",
                report.places()
            );
            assert!(report.ok(), "{report}\n{}", report.places());
            assert!(low > -0.005, "a finger sank to {low} m (nudge {nudge})");
            assert!(
                gap < 0.02,
                "a hand came {gap} m off the prop (nudge {nudge})"
            );
        }
    }

    /// endorphin's Car Hit: a heavy car joined to a keyed carrier catches the
    /// character's arm against their ribs, carries them on the bonnet and stops dead.
    /// The shoulder came apart by 5.8 cm in the hit (lesson 15). The join is meant to
    /// give way when the carrier stops dead (1e5 N can't stop 747 kg at once), so its
    /// drift isn't checked.
    ///
    /// Lesson 15's bound covers the hit and carry, up to the stop (`CAR_CARRY_HIT`): no
    /// joint of the body opens 2 cm. Over 201 runs with the car's speed nudged by up to
    /// ±1e-3 (`nudge_survey`), the worst was 0.5 cm, an ankle. The toes' and fingers'
    /// hinges, light bodies the bonnet strikes directly at 8 m/s, open 0.5 to 2.7 cm
    /// at the hit (frames 15-21) over those runs, so they are held to 3.5 cm.
    ///
    /// After the stop the pedestrian slides off the bonnet and the left arm whips
    /// round against their own chest and neck. That opened the shoulder by up to 7 cm
    /// (#23) until lesson 17; over the same runs no joint of the body now opens more
    /// than 0.7 cm after the stop (2.1 cm here before), and none at all more than
    /// 2.7 cm in the whole scene (the toes at the hit), so the whole scene is held to
    /// 3.5 cm and the body after the stop to 1.5 cm.
    ///
    /// Since lesson 21 (Coulomb friction, and the car 15 cm off the ground) the same
    /// 201 runs give, up to the stop, 0.3 to 0.7 cm for the body and 0.5 to 2.2 cm in
    /// all; after the stop, 0.1 to 1.1 cm.
    #[test]
    fn car_carry_keeps_joints_together() {
        let CarCarry {
            whole,
            hit,
            body: (body, place),
            after_body: (after_body, after_place),
            ..
        } = car_carry_numbers(8.0);
        println!(
            "Car carry: {whole}\n{}  hit and carry: max joint stretch {:.4} m at {}; {body:.4} m without the toes and fingers, at {place}\n  after the stop, without the toes and fingers: {after_body:.4} m at {after_place}",
            whole.places(),
            hit.stretch,
            hit.places[1]
        );
        assert!(
            whole.ok() && whole.buried <= 0.0,
            "{whole}\n{}{}",
            whole.places(),
            describe(&car_carry())
        );
        assert!(
            body < 0.02,
            "a joint opened {body} m in the hit and carry, at {place}"
        );
        assert!(
            hit.stretch < 0.035,
            "{} m in the hit and carry, at {}",
            hit.stretch,
            hit.places[1]
        );
        assert!(
            whole.stretch < 0.035,
            "{} m in the whole scene, at {}",
            whole.stretch,
            whole.places[1]
        );
        assert!(
            after_body < 0.015,
            "a joint opened {after_body} m after the stop, at {after_place}"
        );
    }
}

#[cfg(test)]
mod behaviour_tests {
    use super::*;

    /// Lowest height of the named bodies' centres over a frame range.
    fn lowest(
        project: &Project,
        sim: &Simulation,
        bodies: &[&str],
        frames: std::ops::RangeInclusive<i32>,
    ) -> f32 {
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
        // The ankle strategy alone holds these. Backwards, the capture point leaves the
        // placeholder's short heel (6 cm behind the ankle): that takes a step
        // (`balance_steps_when_the_ankles_cannot_hold`).
        for (behaviour, dir) in [
            (true, Vec3::Z),
            (true, Vec3::X),
            (true, -Vec3::X),
            (false, Vec3::Z),
        ] {
            let project = balance_push(1.5, dir, behaviour);
            let (sim, _) = simulate(&project);
            // Standing, the head's centre is at 1.7 m and the hips' at 1.0 m; check the
            // 3 s after the push.
            let head = lowest(&project, &sim, &["head"], 60..=240);
            let hips = lowest(&project, &sim, &["hips"], 60..=240);
            println!(
                "balance {behaviour}, push 1.5 m/s along {dir}: head down to {head:.2} m, hips {hips:.2} m"
            );
            if behaviour {
                assert!(
                    head > 1.45 && hips > 0.8,
                    "fell pushed along {dir}: head {head}, hips {hips}"
                );
            } else {
                assert!(head < 1.0, "limp character should fall: head {head}");
            }
        }
    }

    /// How far the feet moved over the ground (the furthest either went) and how many
    /// steps they took (a foot lifted 3 cm and put back down), from frame `from` on.
    fn footwork(project: &Project, sim: &Simulation, from: i32) -> (f32, usize) {
        let rig = project.characters[0].physics.as_ref().unwrap();
        let feet = ["l_ankle", "r_ankle"].map(|name| rig.body_index(name).unwrap());
        let at =
            |frame: i32, foot: usize| sim.frame(frame).unwrap()[0].as_ref().unwrap().bodies[foot].1;
        let (mut moved, mut steps) = (0.0f32, 0);
        for foot in feet {
            let start = at(from, foot);
            let mut up = false;
            for frame in from..=project.settings.range.end {
                let p = at(frame, foot);
                moved = moved.max((p - start).with_y(0.0).length());
                if p.y > start.y + 0.03 {
                    up = true;
                } else if up && p.y < start.y + 0.01 {
                    up = false;
                    steps += 1;
                }
            }
        }
        (moved, steps)
    }

    /// How one push went (`pushed`).
    struct Outcome {
        /// The head stayed above 1.45 m and the hips above 0.8 m for the 3 s after it.
        up: bool,
        /// Steps taken (`footwork`).
        steps: usize,
        /// How far the feet went over the ground, m.
        moved: f32,
        /// The motion stayed physical: the sweep's checks.
        physical: bool,
        line: String,
    }

    impl Outcome {
        /// Stayed up and clearly stepped to do it (feet moved 10 cm, at least one step).
        fn saved(&self) -> bool {
            self.up && self.moved > 0.1 && self.steps >= 1
        }
    }

    fn outcome(behaviour: &str, speed: f32, dir: Vec3) -> Outcome {
        let project = pushed(Some(behaviour), speed, dir);
        let (sim, elapsed) = simulate(&project);
        let head = lowest(&project, &sim, &["head"], 60..=240);
        let hips = lowest(&project, &sim, &["hips"], 60..=240);
        let (moved, steps) = footwork(&project, &sim, 59);
        let report = measure(&project, &sim, elapsed);
        Outcome {
            up: head > 1.45 && hips > 0.8,
            steps,
            moved,
            physical: report.ok(),
            line: format!(
                "{behaviour}, push {speed} m/s along {dir}: head down to {head:.2} m, hips {hips:.2} m, feet moved {moved:.2} m in {steps} steps; {report}"
            ),
        }
    }

    /// Several pushes (direction, m/s), their speeds scaled by 1 + `nudge`, each
    /// simulated on a thread of its own (each run is deterministic on its own).
    fn outcomes(behaviour: &str, pushes: &[(Vec3, f32)], nudge: f32) -> Vec<Outcome> {
        std::thread::scope(|s| {
            let runs: Vec<_> = pushes
                .iter()
                .map(|&(dir, speed)| {
                    s.spawn(move || outcome(behaviour, speed * (1.0 + nudge), dir))
                })
                .collect();
            runs.into_iter().map(|run| run.join().unwrap()).collect()
        })
    }

    /// Chest pushes: direction and speed, m/s.
    type Pushes = Vec<(Vec3, f32)>;

    /// Pushes every 0.1 m/s from `from` to `to` along `dir`.
    fn speeds(dir: Vec3, from: f32, to: f32) -> Pushes {
        let tenths = |v: f32| (v * 10.0).round() as i32;
        (tenths(from)..=tenths(to))
            .map(|t| (dir, t as f32 / 10.0))
            .collect()
    }

    /// The pushes `balance_steps_when_the_ankles_cannot_hold` checks, every 0.1 m/s
    /// from just past what the ankle strategy holds on its own (it falls from 1.4 m/s
    /// backwards, 1.9 to the left, 1.7 to the right and 1.7 forwards;
    /// docs/behaviours.md), and how many of each set Balance must step and stay up
    /// after. Near its limit, which pushes it survives turns on small differences in
    /// where a foot lands, so each set is checked as a count. Over 101 runs with the
    /// speeds nudged by up to ±1e-3 (`nudge_survey`) the counts were 5-8 of 8
    /// backwards, 3-7 of 22 sideways and 6-11 of 12 forwards when these minimums were
    /// set, one below the worst; with Coulomb friction and the higher swing they are
    /// 6-8, 8-12 and 9-12.
    fn stepping_sets() -> [(&'static str, Pushes, usize); 3] {
        let sideways = [speeds(Vec3::X, 1.9, 2.8), speeds(-Vec3::X, 1.7, 2.8)].concat();
        [
            ("backwards", speeds(-Vec3::Z, 1.4, 2.1), 4),
            ("sideways", sideways, 2),
            ("forwards", speeds(Vec3::Z, 1.7, 2.8), 5),
        ]
    }

    #[test]
    fn balance_steps_when_the_ankles_cannot_hold() {
        for (label, pushes, at_least) in stepping_sets() {
            let runs = outcomes("Balance", &pushes, 0.0);
            for run in &runs {
                println!("{}", run.line);
                assert!(run.physical, "{}", run.line);
            }
            let saved = runs.iter().filter(|r| r.saved()).count();
            println!(
                "{label}: stepped and stayed up after {saved} of {}",
                runs.len()
            );
            assert!(
                saved >= at_least,
                "{label}: stepped and stayed up after only {saved} of {} pushes",
                runs.len()
            );
        }
    }

    /// Stepping on the stand-in character with toes (its soles run onto them), turned
    /// to face other ways than the rig's rest pose: pushed backwards at 1.5 m/s, it
    /// steps back and stays up.
    #[test]
    fn balance_steps_with_toes_facing_any_way() {
        for yaw in [0.0f32, 90.0] {
            let mut p = new_scene("Balance", 4.0);
            let id = person(&mut p, "stander", Vec3::ZERO, yaw, &[]);
            behave(&mut p, id, 0, 240, "Balance");
            let back = Quat::from_rotation_y(yaw.to_radians()) * -Vec3::Z;
            push(&mut p, id, 60, "stander.MetaBodies.chest", back * 1.5);
            let (sim, elapsed) = simulate(&p);
            let head = lowest(&p, &sim, &["head"], 60..=240);
            let (moved, steps) = footwork(&p, &sim, 59);
            let report = measure(&p, &sim, elapsed);
            println!(
                "balance with toes, facing {yaw}°, pushed back 1.5 m/s: head down to {head:.2} m, feet moved {moved:.2} m in {steps} steps; {report}"
            );
            assert!(head > 1.45, "fell facing {yaw}°: head {head}");
            assert!(moved > 0.1 && steps >= 1, "didn't step: {moved} m, {steps}");
            assert!(report.ok(), "{report}");
        }
    }

    /// What `stagger_keeps_stepping_after_a_hard_push` measures for the hard pushes,
    /// their speeds scaled by 1 + `nudge`: Stagger's runs, and how far Balance's feet
    /// went in all (m).
    fn stagger_and_balance(nudge: f32) -> (Vec<Outcome>, f32) {
        let pushes = HARD_PUSHES.map(|speed| (Vec3::Z, speed));
        let balance = outcomes("Balance", &pushes, nudge);
        (
            outcomes("Stagger", &pushes, nudge),
            balance.iter().map(|r| r.moved).sum(),
        )
    }

    /// Stagger keeps stepping after a hard push where Balance stops in a few steps and
    /// stands: at least 2 steps after each of the hard pushes and 3 a push on average,
    /// and its feet go more than 1.5 times as far as Balance's. How many steps one
    /// push takes turns on small differences in where a foot lands (pushed at 2.5 m/s,
    /// 2 to 6 over 9 nudged runs). Over 101 runs with the speeds nudged by up to ±1e-3
    /// (`nudge_survey`), the fewest after one hard push was 3, the total 25 to 41, and
    /// the feet went 1.9 to 4.5 times as far as Balance's (3, 17 to 31 and 2.3 to 4.2
    /// on force-capped friction, before Stagger shuffled and stepped short).
    #[test]
    fn stagger_keeps_stepping_after_a_hard_push() {
        let (stagger, balance_moved) = stagger_and_balance(0.0);
        for run in &stagger {
            println!("{}", run.line);
            assert!(run.physical, "{}", run.line);
            assert!(run.steps >= 2, "only {} steps: {}", run.steps, run.line);
        }
        let steps: usize = stagger.iter().map(|r| r.steps).sum();
        let moved: f32 = stagger.iter().map(|r| r.moved).sum();
        println!(
            "Stagger: {steps} steps in all, feet moved {moved:.2} m; Balance's feet moved {balance_moved:.2} m"
        );
        assert!(
            steps >= 3 * HARD_PUSHES.len(),
            "only {steps} steps after {} pushes",
            HARD_PUSHES.len()
        );
        assert!(
            moved > 1.5 * balance_moved,
            "feet moved {moved} m, Balance's {balance_moved} m"
        );
    }

    /// How far the numbers the stepping tests and `car_carry_keeps_joints_together`
    /// assert on move when the scenes are nudged (`nudges`): run it after changing
    /// the physics or a behaviour, and keep each test's bound clear of the worst.
    /// `NM_SURVEY=car|car_hit|explosion|balance|stagger` runs one part (all when unset); `NM_NUDGES` and
    /// `NM_NUDGE_STEP` set the nudges. `cargo test --release nudge_survey -- --ignored
    /// --nocapture`
    #[test]
    #[ignore = "survey"]
    fn nudge_survey() {
        let part = std::env::var("NM_SURVEY").unwrap_or_default();
        let wanted = |name: &str| part.is_empty() || part == name;
        let nudges = nudges();
        let range = |v: &mut Vec<f32>| {
            v.sort_by(f32::total_cmp);
            format!("{:.2} to {:.2}", v[0], v[v.len() - 1])
        };
        println!(
            "{} runs, nudged by up to {:e}",
            nudges.len(),
            nudges.iter().fold(0.0f32, |a, n| a.max(n.abs()))
        );
        if wanted("car") {
            let [mut whole, mut hit, mut body, mut after, mut after_body] =
                [(); 5].map(|_| Vec::new());
            let mut over = 0;
            for &nudge in &nudges {
                let c = car_carry_numbers(8.0 * (1.0 + nudge));
                println!(
                    "car carry {nudge:+.1e}: hit and carry {:.2} cm at {}, without toes and fingers {:.2} cm at {}; after the stop {:.2} cm at {}, without toes and fingers {:.2} cm at {}",
                    c.hit.stretch * 100.0,
                    c.hit.places[1],
                    c.body.0 * 100.0,
                    c.body.1,
                    c.after.stretch * 100.0,
                    c.after.places[1],
                    c.after_body.0 * 100.0,
                    c.after_body.1,
                );
                over += usize::from(!(c.whole.ok() && c.whole.buried <= 0.0));
                whole.push(c.whole.stretch * 100.0);
                hit.push(c.hit.stretch * 100.0);
                body.push(c.body.0 * 100.0);
                after.push(c.after.stretch * 100.0);
                after_body.push(c.after_body.0 * 100.0);
            }
            println!(
                "car carry, max joint stretch (cm): hit and carry {}, without toes and fingers {}; after the stop {}, without toes and fingers {}; whole scene {}, failing the sweep in {over} runs",
                range(&mut hit),
                range(&mut body),
                range(&mut after),
                range(&mut after_body),
                range(&mut whole),
            );
        }
        if wanted("car_hit") {
            let mut stretch = Vec::new();
            for &nudge in &nudges {
                let project = car_hit_nudged(nudge);
                let (sim, elapsed) = simulate(&project);
                let r = measure(&project, &sim, elapsed);
                println!(
                    "car hit {nudge:+.1e}: stretch {:.2} cm at {}, fastest {:.1} m/s at {}, lowest {:.3} m at {}",
                    r.stretch * 100.0,
                    r.places[1],
                    r.fastest,
                    r.places[2],
                    r.lowest,
                    r.places[0]
                );
                stretch.push(r.stretch * 100.0);
            }
            println!(
                "car hit, max joint stretch (cm): {} (test wants < 5)",
                range(&mut stretch)
            );
        }
        if wanted("explosion") {
            let [
                mut high,
                mut low,
                mut turned,
                mut landed,
                mut tilt,
                mut settled,
                mut stretch,
            ] = [(); 7].map(|_| Vec::new());
            for &nudge in &nudges {
                let project = mid_air_nudged(true, true, nudge);
                let (sim, elapsed) = simulate(&project);
                let r = measure(&project, &sim, elapsed);
                let f = flight(&project, &sim);
                let e = explosion_numbers(&f);
                println!(
                    "explosion {nudge:+.1e}: arms up to {:.0}° at {}, down to {:.0}° at {}; turned {:.0}°; lands at {} on {}; tilt {:.0}° at 200; settled {:.3} m/s; stretch {:.2} cm at {}",
                    e.high,
                    e.at_high,
                    e.low,
                    e.at_low,
                    e.turned,
                    e.landed,
                    f.lowest[e.landed].1,
                    f.tilt[200],
                    e.settled,
                    r.stretch * 100.0,
                    r.places[1]
                );
                high.push(e.high);
                low.push(e.low);
                turned.push(e.turned);
                landed.push(e.landed as f32);
                tilt.push(f.tilt[200]);
                settled.push(e.settled);
                stretch.push(r.stretch * 100.0);
            }
            // Settling is asserted on in hundredths.
            settled.iter_mut().for_each(|s| *s *= 100.0);
            println!(
                "explosion: arms up {} °, down {} °, turned {} °, lands at frame {}, trunk tilted {} ° at 200, bodies moving {} cm/s at the end (test wants < 10), max joint stretch {} cm (sweep wants < 5)",
                range(&mut high),
                range(&mut low),
                range(&mut turned),
                range(&mut landed),
                range(&mut tilt),
                range(&mut settled),
                range(&mut stretch),
            );
        }
        if wanted("balance") {
            for (label, pushes, at_least) in stepping_sets() {
                let mut counts = Vec::new();
                let mut unphysical = 0;
                for &nudge in &nudges {
                    let runs = outcomes("Balance", &pushes, nudge);
                    let row: String = runs
                        .iter()
                        .map(|r| if r.saved() { '#' } else { '.' })
                        .collect();
                    let saved = runs.iter().filter(|r| r.saved()).count();
                    unphysical += runs.iter().filter(|r| !r.physical).count();
                    println!("balance {label} {nudge:+.1e}: {row} {saved}");
                    counts.push(saved);
                }
                counts.sort();
                println!(
                    "balance {label}: stepped and stayed up after {} to {} of {} (test wants {at_least}); {unphysical} unphysical runs",
                    counts[0],
                    counts[counts.len() - 1],
                    pushes.len()
                );
            }
        }
        if wanted("stagger") {
            let (mut fewest, mut total, mut ratio) = (Vec::new(), Vec::new(), Vec::new());
            for &nudge in &nudges {
                let (stagger, balance_moved) = stagger_and_balance(nudge);
                let steps: Vec<_> = stagger.iter().map(|r| r.steps).collect();
                let moved: f32 = stagger.iter().map(|r| r.moved).sum();
                println!(
                    "stagger {nudge:+.1e}: steps {steps:?}, feet moved {moved:.2} m; Balance's {balance_moved:.2} m; unphysical {}",
                    stagger.iter().filter(|r| !r.physical).count()
                );
                fewest.push(*steps.iter().min().unwrap() as f32);
                total.push(steps.iter().sum::<usize>() as f32);
                ratio.push(moved / balance_moved);
            }
            println!(
                "stagger: fewest steps after one push {}; steps in all {}; feet moved {} times Balance's",
                range(&mut fewest),
                range(&mut total),
                range(&mut ratio)
            );
        }
    }

    /// Which chest pushes a behaviour survives (head above 1.45 m for 3 s): one row
    /// per direction, one column per speed from 1.0 to 3.0 m/s in 0.1 steps, `#` for
    /// stayed up. `NM_SWEEP=<behaviour> cargo test --release push_sweep -- --ignored
    /// --nocapture` (Balance when unset). docs/behaviours.md has the results.
    #[test]
    #[ignore = "survey"]
    fn push_sweep() {
        let name = std::env::var("NM_SWEEP").unwrap_or_else(|_| "Balance".to_owned());
        let dirs = [
            ("forwards ", Vec3::Z),
            ("backwards", -Vec3::Z),
            ("left     ", Vec3::X),
            ("right    ", -Vec3::X),
        ];
        let mut total = 0;
        for (label, dir) in dirs {
            let row: String = (10..=30)
                .map(|tenths| {
                    let project = pushed(Some(&name), tenths as f32 / 10.0, dir);
                    let (sim, _) = simulate(&project);
                    let up = lowest(&project, &sim, &["head"], 60..=240) > 1.45;
                    total += usize::from(up);
                    if up { '#' } else { '.' }
                })
                .collect();
            println!("{name} {label} {row}");
        }
        println!("{name}: {total} of 84 pushes survived");
    }

    /// Stepping keeps no state: the same push gives the same motion, to the bit.
    #[test]
    fn stepping_is_deterministic() {
        for behaviour in ["Balance", "Stagger"] {
            let project = pushed(Some(behaviour), 2.0, Vec3::new(1.0, 0.0, -1.0));
            let (a, _) = simulate(&project);
            let (b, _) = simulate(&project);
            for frame in 0..=project.settings.range.end {
                let (fa, fb) = (a.frame(frame).unwrap(), b.frame(frame).unwrap());
                let (fa, fb) = (fa[0].as_ref().unwrap(), fb[0].as_ref().unwrap());
                assert!(
                    fa.bodies.iter().zip(&fb.bodies).all(|(x, y)| x == y),
                    "{behaviour}: runs differ at frame {frame}"
                );
            }
        }
    }

    /// The first frame each part comes within 4 cm of the ground (`None`: never).
    fn first_touch(project: &Project, sim: &Simulation, parts: &[(&str, f32)]) -> Option<i32> {
        let rig = project.characters[0].physics.as_ref().unwrap();
        let range = project.settings.range;
        (range.start..=range.end).find(|&frame| {
            let cf = sim.frame(frame).unwrap()[0].as_ref().unwrap();
            parts
                .iter()
                .any(|&(name, radius)| cf.bodies[rig.body_index(name).unwrap()].1.y < radius + 0.04)
        })
    }

    #[test]
    fn catch_fall_lands_on_the_hands_first() {
        let shoves = [
            Vec3::new(0.0, 0.0, 3.5),
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(2.5, 0.0, 2.5),
            Vec3::new(3.5, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -3.5),
        ];
        let cases = shoves
            .iter()
            .map(|&s| (s, true))
            .chain([(shoves[0], false)]);
        for (shove, behaviour) in cases {
            let project = catch_fall_shove(shove, behaviour);
            let (sim, _) = simulate(&project);
            let hands = first_touch(&project, &sim, &[("l_wrist", 0.035), ("r_wrist", 0.035)]);
            let head = first_touch(&project, &sim, &[("head", 0.1)]);
            let chest = first_touch(&project, &sim, &[("chest", 0.12)]);
            println!(
                "catch fall {behaviour}, shoved {shove}: hands {hands:?}, head {head:?}, chest {chest:?}"
            );
            let first = |a: Option<i32>, b: Option<i32>| a.is_some_and(|a| b.is_none_or(|b| a < b));
            let hands_first = first(hands, head) && first(hands, chest);
            if behaviour {
                assert!(
                    hands_first,
                    "shoved {shove}: hands {hands:?}, head {head:?}, chest {chest:?}"
                );
            } else {
                // Shoved forwards, a limp character lands on its face.
                assert!(
                    !hands_first,
                    "a limp fall shouldn't land on the hands first"
                );
            }
        }
    }

    /// The placeholder drops limp at the start, then `behaviour` (if any) runs from 1 s to
    /// the end of a 3.5 s scene.
    fn limp_then(behaviour: Option<&str>) -> Project {
        let mut p = new_scene("Curl", 3.5);
        let id = placeholder(&mut p, "victim", Vec3::ZERO, 0.0);
        if let Some(name) = behaviour {
            behave(&mut p, id, 60, 210, name);
        }
        p
    }

    /// Position of a rig body at a frame.
    fn at(project: &Project, sim: &Simulation, body: &str, frame: i32) -> Vec3 {
        let rig = project.characters[0].physics.as_ref().unwrap();
        let cf = sim.frame(frame).unwrap()[0].as_ref().unwrap();
        cf.bodies[rig.body_index(body).unwrap()].1
    }

    /// (knees to chest, head to knees, hands to head): average distances at a frame.
    fn curl_distances(project: &Project, sim: &Simulation, frame: i32) -> (f32, f32, f32) {
        let pos = |name: &str| at(project, sim, name, frame);
        let knees = (pos("l_knee") + pos("r_knee")) / 2.0;
        let hands =
            (pos("l_wrist").distance(pos("head")) + pos("r_wrist").distance(pos("head"))) / 2.0;
        (
            knees.distance(pos("chest")),
            pos("head").distance(knees),
            hands,
        )
    }

    #[test]
    fn body_foetal_curls_up() {
        let (limp, curled) = (limp_then(None), limp_then(Some("BodyFoetal")));
        let ((sim_limp, _), (sim_curled, time)) = (simulate(&limp), simulate(&curled));
        let (a, b) = (
            curl_distances(&limp, &sim_limp, 180),
            curl_distances(&curled, &sim_curled, 180),
        );
        println!(
            "limp: knees-chest {:.2} m, head-knees {:.2} m, hands-head {:.2} m",
            a.0, a.1, a.2
        );
        println!(
            "foetal: knees-chest {:.2} m, head-knees {:.2} m, hands-head {:.2} m",
            b.0, b.1, b.2
        );
        let report = measure(&curled, &sim_curled, time);
        println!("foetal run: {report}");
        assert!(report.ok(), "{report}");
        assert!(
            b.0 < a.0 * 0.8,
            "knees not closer to the chest: {a:?} {b:?}"
        );
        assert!(b.1 < a.1 * 0.8, "head not closer to the knees: {a:?} {b:?}");
        assert!(b.2 < a.2 * 0.5, "hands not over the head: {a:?} {b:?}");
    }

    /// endorphin's Car Hit came apart by 7.9 cm at a clavicle mid-way through its
    /// transition back to animation, while the simulated bodies themselves stayed
    /// within 1 cm: the blend shortened the bones it drew (lesson 18).
    #[test]
    fn transition_keeps_bones_whole() {
        let project = curl_hand_back();
        let (sim, time) = simulate(&project);
        let report = measure(&project, &sim, time);
        let (blend, place) = joint_stretch(&project, &sim, "curler", 135..=180, |_| true);
        println!("curl hand-back: {report}\n  in the transition: {blend:.4} m at {place}");
        assert!(report.ok(), "{report}");
        assert!(blend < 0.02, "{blend:.4} m at {place}");
    }

    /// Car Hit's drawn pose jumped for exactly one frame at its transition's last
    /// frame: 111 blended (-0.97 m), 112 the raw simulated pose (-1.85 m), 113 the
    /// animation (-0.93 m). The blend ran over `start <= frame < end` while the level
    /// only changes to animated after frame `end` is recorded (lesson "transition end").
    /// The curled character lies far from its standing animation, so a pop shows.
    #[test]
    fn transition_ends_on_the_animation_without_a_pop() {
        let project = curl_hand_back();
        let (sim, _) = simulate(&project);
        let (start, end) = (135, 180);
        let joints = |f: i32| &sim.frame(f).unwrap()[0].as_ref().unwrap().joints;
        // Largest move of any joint between two frames, in metres.
        let step = |a: i32, b: i32| {
            joints(a)
                .iter()
                .zip(joints(b))
                .map(|(x, y)| x.1.distance(y.1))
                .fold(0.0_f32, f32::max)
        };
        // How fast the blend moves the pose before the end, the pace to compare with.
        let pace = (end - 6..end - 1)
            .map(|f| step(f, f + 1))
            .fold(0.0, f32::max);
        let (into, out) = (step(end - 1, end), step(end, end + 1));
        println!(
            "transition {start}-{end}: pace {pace:.3} m/frame, into end {into:.3} m, out of end {out:.3} m"
        );
        assert!(
            into < pace * 2.0,
            "pops into the last frame: {into:.3} m vs pace {pace:.3} m"
        );
        assert!(
            out < pace * 2.0,
            "pops out of the last frame: {out:.3} m vs pace {pace:.3} m"
        );
        // The last frame already is the animation.
        let anim = world_transforms(
            &project.characters[0].skeleton,
            &project.evaluate(end as f32)[0],
        );
        let off = joints(end)
            .iter()
            .zip(&anim)
            .map(|(j, a)| j.1.distance(a.1))
            .fold(0.0_f32, f32::max);
        assert!(off < 1e-3, "frame {end} is {off:.3} m from the animation");
    }

    /// Look and Reach at a frame: how far each arm (shoulder joint to finger tips)
    /// points off the ball and how far its fingers are from it, and how far the head's
    /// forward points off it, in degrees and metres.
    struct Reaching {
        arm_off: [f32; 2],
        hand_gap: [f32; 2],
        head_off: f32,
    }

    fn reaching(project: &Project, sim: &Simulation, frame: i32) -> Reaching {
        let c = &project.characters[0];
        let rig = c.physics.as_ref().unwrap();
        let cf = sim.frame(frame).unwrap()[0].as_ref().unwrap();
        let ball = sim.frame(frame).unwrap()[1].as_ref().unwrap().bodies[0].1;
        let deg = |a: Vec3, b: Vec3| a.angle_between(b).to_degrees();
        let body = |name: &str| cf.bodies[rig.body_index(name).unwrap()];
        let arm = |side: &str| {
            let shoulder =
                cf.joints[c.skeleton.joint_index(&format!("{side}_shoulder")).unwrap()].1;
            let tip = body(&format!("{side}_fingers")).1;
            (deg(tip - shoulder, ball - shoulder), tip.distance(ball))
        };
        let (l, r) = (arm("l"), arm("r"));
        // The head faces +Z at rest.
        let rest = rig_bodies(
            rig,
            &world_transforms(&c.skeleton, &Pose::rest(c.skeleton.joints.len())),
        );
        let head = rig.body_index("head").unwrap();
        let (rotation, at) = cf.bodies[head];
        let forward = rotation * rest[head].0.inverse() * Vec3::Z;
        Reaching {
            arm_off: [l.0, r.0],
            hand_gap: [l.1, r.1],
            head_off: deg(forward, ball - at),
        }
    }

    /// The first frame both arms point within `degrees` of the ball.
    fn reached(project: &Project, sim: &Simulation, degrees: f32) -> Option<i32> {
        (0..=project.settings.range.end).find(|&f| {
            let r = reaching(project, sim, f);
            r.arm_off[0] < degrees && r.arm_off[1] < degrees
        })
    }

    /// Look and Reach with the ball in front of the chest, up to the left (out of
    /// reach, as in endorphin's scene): measured against endorphin's run of "Hands
    /// reach and look at" (docs/behaviours.md). There both arms swing from a T-pose to
    /// point at the box by frame 20-24 and the head has turned to it by frame 15.
    #[test]
    fn look_and_reach_points_the_arms_and_head_at_the_target() {
        let ball = Vec3::new(0.4, 1.8, 1.2);
        for t_pose in [false, true] {
            let project = reach_for(ball, Vec3::ZERO, t_pose);
            let (sim, time) = simulate(&project);
            let report = measure(&project, &sim, time);
            let first = reached(&project, &sim, 20.0);
            let head = (0..=180).find(|&f| reaching(&project, &sim, f).head_off < 15.0);
            let end = reaching(&project, &sim, 120);
            println!(
                "look and reach (T-pose {t_pose}): arms within 20° at frame {first:?}, head \
                 within 15° at {head:?}; at 2 s arms {:.1}° {:.1}° off, fingers {:.2} {:.2} m \
                 from the ball, head {:.1}° off. {report}",
                end.arm_off[0], end.arm_off[1], end.hand_gap[0], end.hand_gap[1], end.head_off
            );
            assert!(report.ok(), "{report}");
            // endorphin: frame 20-24. Here 22 (hanging arms) and 23 (T-pose).
            assert!(
                first.is_some_and(|f| (16..=30).contains(&f)),
                "arms not there at endorphin's pace: {first:?}"
            );
            assert!(head.is_some_and(|f| f <= 20), "head too slow: {head:?}");
            assert!(end.arm_off.iter().all(|&a| a < 8.0), "arms off");
            assert!(end.head_off < 8.0, "head off {}", end.head_off);
        }
    }

    /// A ball within reach: both hands get to it (fingers' centres within 8 cm of its
    /// centre, the two hands kept 8 cm apart).
    #[test]
    fn look_and_reach_gets_the_hands_to_a_target_in_reach() {
        let project = reach_for(Vec3::new(0.1, 1.35, 0.45), Vec3::ZERO, true);
        let (sim, time) = simulate(&project);
        let report = measure(&project, &sim, time);
        let end = reaching(&project, &sim, 120);
        println!(
            "in reach: fingers {:.3} {:.3} m from the ball at 2 s. {report}",
            end.hand_gap[0], end.hand_gap[1]
        );
        assert!(report.ok(), "{report}");
        assert!(end.hand_gap.iter().all(|&g| g < 0.08), "{:?}", end.hand_gap);
    }

    /// The hands follow a ball moving 1.5 m sideways over 3 s, and the head with them.
    #[test]
    fn look_and_reach_follows_a_moving_target() {
        let project = reach_for(Vec3::new(-0.6, 1.5, 1.0), Vec3::new(1.5, 0.0, 0.0), true);
        let (sim, time) = simulate(&project);
        let report = measure(&project, &sim, time);
        assert!(report.ok(), "{report}");
        let worst = (60..=180).map(|f| reaching(&project, &sim, f)).fold(
            (0.0f32, 0.0f32),
            |(arm, head), r| {
                (
                    arm.max(r.arm_off[0]).max(r.arm_off[1]),
                    head.max(r.head_off),
                )
            },
        );
        println!(
            "following: arms at most {:.1}° off, head {:.1}° off from 1 s to 3 s. {report}",
            worst.0, worst.1
        );
        assert!(worst.0 < 12.0 && worst.1 < 10.0, "{worst:?}");
    }

    /// Two runs give the same bodies, bit for bit, and moving the ball by up to 4e-4 m
    /// (as another platform's rounding might move a body) moves the numbers above only
    /// a little.
    #[test]
    fn look_and_reach_is_deterministic_and_robust() {
        let ball = Vec3::new(0.4, 1.8, 1.2);
        let project = reach_for(ball, Vec3::ZERO, true);
        let (sim, _) = simulate(&project);
        let (again, _) = simulate(&project);
        for frame in [30, 90, 180] {
            assert_eq!(
                sim.frame(frame).unwrap()[0].as_ref().unwrap().bodies,
                again.frame(frame).unwrap()[0].as_ref().unwrap().bodies,
                "frame {frame} differs between runs"
            );
        }
        let base = (
            reached(&project, &sim, 20.0).unwrap(),
            reaching(&project, &sim, 120),
        );
        for nudge in nudges() {
            let project = reach_for(ball + Vec3::splat(nudge), Vec3::ZERO, true);
            let (sim, _) = simulate(&project);
            let first = reached(&project, &sim, 20.0).unwrap();
            let end = reaching(&project, &sim, 120);
            let moved = (0..2)
                .map(|i| (end.arm_off[i] - base.1.arm_off[i]).abs())
                .fold(0.0f32, f32::max);
            println!(
                "nudge {nudge:+.0e}: within 20° at frame {first} (unnudged {}), arms moved by \
                 at most {moved:.3}°",
                base.0
            );
            assert!((first - base.0).abs() <= 2, "{first} vs {}", base.0);
            assert!(moved < 0.5, "arms moved {moved}°");
        }
    }

    #[test]
    fn writhe_keeps_moving_and_is_deterministic() {
        let project = limp_then(Some("Writhe"));
        let (sim, time) = simulate(&project);
        let report = measure(&project, &sim, time);
        println!("writhe run: {report}");
        assert!(report.ok(), "{report}");
        let rig = project.characters[0].physics.as_ref().unwrap();
        // Speed of each body over the last second, from its movement between frames.
        let mut moving = 0;
        let mut total = 0.0;
        let fps = project.settings.fps;
        for name in rig.bodies.iter().map(|b| b.name.as_str()) {
            let travelled: f32 = (180..210)
                .map(|f| at(&project, &sim, name, f).distance(at(&project, &sim, name, f + 1)))
                .sum();
            let speed = travelled * fps / 30.0;
            total += speed;
            moving += usize::from(speed > 0.1);
        }
        println!(
            "writhe: {moving} of {} bodies moving, mean {:.2} m/s",
            rig.bodies.len(),
            total / rig.bodies.len() as f32
        );
        assert!(moving >= 4, "only {moving} bodies move");
        // The same scene again gives the same bodies, bit for bit.
        let (again, _) = simulate(&project);
        for frame in [90, 150, 210] {
            let (a, b) = (
                sim.frame(frame).unwrap()[0].as_ref().unwrap(),
                again.frame(frame).unwrap()[0].as_ref().unwrap(),
            );
            assert_eq!(a.bodies, b.bodies, "frame {frame} differs between runs");
        }
    }

    /// What `mid_air` shows, per frame: (left and right upper arms' elevation above
    /// the horizontal, deg; how far the pelvis has turned since frame 0, deg; the
    /// trunk's tilt from upright, deg; lowest body centre and its name).
    struct Flight {
        arms: Vec<(f32, f32)>,
        turned: Vec<f32>,
        tilt: Vec<f32>,
        lowest: Vec<(f32, String)>,
        speed: Vec<f32>,
    }

    fn flight(project: &Project, sim: &Simulation) -> Flight {
        let c = &project.characters[0];
        let rig = c.physics.as_ref().unwrap();
        let joint = |n: &str| c.skeleton.joint_index(n).unwrap();
        let hips = rig.body_index("hips").unwrap();
        let start = sim.frame(0).unwrap()[0].as_ref().unwrap().bodies[hips].0;
        let mut f = Flight {
            arms: vec![],
            turned: vec![],
            tilt: vec![],
            lowest: vec![],
            speed: vec![],
        };
        let mut last: Option<Vec<Vec3>> = None;
        for frame in 0..=project.settings.range.end {
            let cf = sim.frame(frame).unwrap()[0].as_ref().unwrap();
            let at = |n: &str| cf.joints[joint(n)].1;
            let elevation = |from: &str, to: &str| {
                let v = at(to) - at(from);
                libm::atan2f(v.y, v.with_y(0.0).length()).to_degrees()
            };
            f.arms.push((
                elevation("l_shoulder", "l_elbow"),
                elevation("r_shoulder", "r_elbow"),
            ));
            f.turned.push(
                (cf.bodies[hips].0 * start.inverse())
                    .to_axis_angle()
                    .1
                    .to_degrees(),
            );
            let trunk = (at("neck") - at("hips")).normalize();
            f.tilt
                .push(libm::acosf(trunk.y.clamp(-1.0, 1.0)).to_degrees());
            let (b, low) = cf
                .bodies
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.1.y.total_cmp(&b.1.1.y))
                .unwrap();
            f.lowest.push((low.1.y, rig.bodies[b].name.clone()));
            let now: Vec<Vec3> = cf.bodies.iter().map(|b| b.1).collect();
            f.speed.push(last.map_or(0.0, |last| {
                let fps = project.settings.fps;
                now.iter()
                    .zip(&last)
                    .map(|(a, b)| a.distance(*b))
                    .sum::<f32>()
                    * fps
                    / now.len() as f32
            }));
            last = Some(now);
        }
        f
    }

    /// Highest (or lowest, `-1`) mean elevation of the two upper arms over `frames`,
    /// with its frame.
    fn arm_extreme(f: &Flight, frames: std::ops::RangeInclusive<usize>, sign: f32) -> (f32, usize) {
        frames
            .map(|i| ((f.arms[i].0 + f.arms[i].1) / 2.0, i))
            .max_by(|a, b| (sign * a.0).total_cmp(&(sign * b.0)))
            .unwrap()
    }

    /// What `explosion_flails_in_mid_air_like_endorphins` asserts on: the arms'
    /// highest mean elevation before frame 30 and its frame, the lowest after it until
    /// landing and its frame, deg; the pelvis' largest turn in the air, deg; the
    /// landing frame; the bodies' mean speed over the last 20 frames, m/s.
    struct ExplosionNumbers {
        high: f32,
        at_high: usize,
        low: f32,
        at_low: usize,
        turned: f32,
        landed: usize,
        settled: f32,
    }

    fn explosion_numbers(f: &Flight) -> ExplosionNumbers {
        let landed = f.lowest.iter().position(|l| l.0 < 0.1).unwrap();
        let (high, at_high) = arm_extreme(f, 0..=30, 1.0);
        let (low, at_low) = arm_extreme(f, 30..=landed, -1.0);
        let turned = f.turned[..landed].iter().fold(0.0f32, |a, &b| a.max(b));
        let settled = f.speed[190..].iter().sum::<f32>() / f.speed[190..].len() as f32;
        ExplosionNumbers {
            high,
            at_high,
            low,
            at_low,
            turned,
            landed,
            settled,
        }
    }

    /// Explosion, measured against endorphin's "Writhe in mid-air 2.5" (both
    /// characters start in a T-pose 4 m up, Amplitude and Speed 0.6, Strength 0.5):
    /// its arms go from out to the sides up to +35-38° between frames 20 and 30, down
    /// to -40 to -58° at 42-46 and are coming back up at 52; the body falls upright,
    /// lands feet first at about frame 56, goes down on its back and lies still.
    /// Over 41 runs with the drop's height nudged by up to ±2e-3 (`NM_SURVEY=explosion
    /// nudge_survey`) no joint opens more than 1.4 cm (15 cm before lesson 29), and the
    /// bodies move 0.23 to 2.9 cm/s over the last 20 frames, this run 1.1 (up to 20
    /// before lesson 31 damped a limp body's joints). Results are bit-identical
    /// everywhere, so this run is the same on CI.
    #[test]
    fn explosion_flails_in_mid_air_like_endorphins() {
        let project = mid_air(true, true);
        let (sim, time) = simulate(&project);
        let report = measure(&project, &sim, time);
        println!("explosion run: {report}");
        assert!(report.ok(), "{report}");
        let f = flight(&project, &sim);
        let ExplosionNumbers {
            high,
            at_high,
            low,
            at_low,
            turned,
            landed,
            settled,
        } = explosion_numbers(&f);
        println!(
            "explosion: arms up to {high:.0}° at frame {at_high}, down to {low:.0}° at \
             {at_low}; pelvis turned {turned:.0}° in the air; lands at frame {landed} on \
             {}; trunk tilted {:.0}° at frame 200; bodies moving {settled:.2} m/s over the \
             last 20 frames",
            f.lowest[landed].1, f.tilt[200]
        );
        assert!((30.0..60.0).contains(&high), "arms up to {high}°");
        assert!((14..=28).contains(&at_high), "highest at {at_high}");
        assert!((-70.0..-30.0).contains(&low), "arms down to {low}°");
        assert!((36..=54).contains(&at_low), "lowest at {at_low}");
        assert!(turned < 20.0, "turned {turned}° in the air");
        assert!((50..=60).contains(&landed), "landed at {landed}");
        assert!(
            ["ankle", "toe", "knee"]
                .iter()
                .any(|n| f.lowest[landed].1.ends_with(n)),
            "landed on {}",
            f.lowest[landed].1
        );
        assert!(f.tilt[200] > 60.0, "trunk {}° from upright", f.tilt[200]);
        assert!(settled < 0.1, "still moving at {settled} m/s");

        // Limp, the same character drops with its arms out and lands sprawled.
        let limp = mid_air(true, false);
        let (limp_sim, _) = simulate(&limp);
        let lf = flight(&limp, &limp_sim);
        let still = lf.arms[..landed]
            .iter()
            .fold(0.0f32, |a, b| a.max(b.0.abs()).max(b.1.abs()));
        println!("limp: arms within {still:.1}° of out to the sides until it lands");
        assert!(still < 5.0, "limp arms moved {still}°");
    }

    /// The same on the placeholder, which rests with its arms hanging: the arms come up
    /// and flap, it stays upright in the air, and two runs agree bit for bit.
    #[test]
    fn explosion_works_on_the_placeholder_and_is_deterministic() {
        let project = mid_air(false, true);
        let (sim, time) = simulate(&project);
        let report = measure(&project, &sim, time);
        println!("explosion on the placeholder: {report}");
        assert!(report.ok(), "{report}");
        let f = flight(&project, &sim);
        let landed = f.lowest.iter().position(|l| l.0 < 0.1).unwrap();
        let (high, at_high) = arm_extreme(&f, 5..=30, 1.0);
        let (low, at_low) = arm_extreme(&f, 30..=landed, -1.0);
        let turned = f.turned[..landed].iter().fold(0.0f32, |a, &b| a.max(b));
        println!(
            "placeholder: arms up to {high:.0}° at frame {at_high}, down to {low:.0}° at \
             {at_low}; turned {turned:.0}° in the air; lands at {landed} on {}",
            f.lowest[landed].1
        );
        assert!(high > 10.0 && low < -20.0, "arms {low}° to {high}°");
        assert!(turned < 20.0, "turned {turned}° in the air");
        let (again, _) = simulate(&project);
        for frame in [30, 60, 120] {
            let (a, b) = (
                sim.frame(frame).unwrap()[0].as_ref().unwrap(),
                again.frame(frame).unwrap()[0].as_ref().unwrap(),
            );
            assert_eq!(a.bodies, b.bodies, "frame {frame} differs between runs");
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
        let mut paths: Vec<_> = std::fs::read_dir(&dir)
            .unwrap()
            .flatten()
            .map(|e| e.path())
            .collect();
        paths.sort();
        for path in paths {
            let name = path.file_stem().unwrap().to_string_lossy().into_owned();
            // `NM_SCENES=Car,Rope` sweeps only scenes whose names contain one of these.
            if let Ok(only) = std::env::var("NM_SCENES")
                && !only.split(',').any(|part| name.contains(part.trim()))
            {
                continue;
            }
            // The whole range: a scene can go wrong long after its first seconds.
            let project = crate::io::endorphin::import_scene(&path).unwrap().project;
            let report = sweep(&project);
            let ok = report.ok();
            println!(
                "{} {name:<16} {:>4} frames in {:>6.0?}  {report}",
                if ok { "ok  " } else { "FAIL" },
                project.settings.range.len() + 1,
                report.elapsed
            );
            // What to look at: where the worst values happened and the set-up behind them.
            let constrained = project
                .clips
                .iter()
                .any(|c| matches!(&c.kind, ClipKind::Event { event } if event == "Constraint"));
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

/// The physics settings the editor changes (gravity, rate, contact model, material
/// interactions and pairs) must reach the simulation, and leave results alone when
/// they aren't touched.
#[cfg(test)]
mod settings_tests {
    use super::*;
    use crate::document::physics::{Interaction, PhysicsSettings};
    use crate::document::{Document, Edit};

    /// A 0.5 m box released at rest `height` m up, over the default ground, for two
    /// seconds at 60 frames per second.
    fn dropped_box(height: f32) -> Project {
        let mut p = new_scene("Dropped box", 2.0);
        let free = Limits {
            twist_min: 0.0,
            twist_max: 0.0,
            swing1: 0.0,
            swing2: 0.0,
        };
        let no_muscle = Gains {
            stiffness: 0.0,
            damping: 0.0,
            force_limit: 0.0,
        };
        let piece = Piece::at(
            Geometry::Box {
                size: [0.5, 0.5, 0.5],
            },
            10.0,
            Quat::IDENTITY,
            Vec3::ZERO,
        );
        add_prop(
            &mut p,
            "box",
            CharacterKind::Prop,
            prop_rig(
                vec![("body", None, Vec3::new(0.0, height, 0.0), piece)],
                free,
                no_muscle,
            ),
            SimLevel::Simulated,
        );
        p
    }

    fn box_at(sim: &Simulation, frame: i32) -> Vec3 {
        sim.frame(frame).unwrap()[0].as_ref().unwrap().bodies[0].1
    }

    /// Applies `change` to the project's physics settings the way the inspector does.
    fn edited(project: &Project, change: impl FnOnce(&mut PhysicsSettings)) -> Project {
        let mut doc = Document::new(project.clone());
        let before = doc.project.physics.clone();
        let mut after = before.clone();
        change(&mut after);
        doc.apply(
            "Edit physics settings",
            Edit::Physics { before, after },
            false,
        );
        doc.project
    }

    /// How high the box bounces after its first landing, m.
    fn rebound(project: &Project) -> f32 {
        let (sim, _) = simulate(project);
        let end = project.settings.range.end;
        let ys: Vec<f32> = (0..=end).map(|f| box_at(&sim, f).y).collect();
        let landing = (0..ys.len())
            .min_by(|&a, &b| ys[a].total_cmp(&ys[b]))
            .unwrap();
        // The lowest point before the box rises again is the first landing.
        let first_landing = (1..ys.len())
            .find(|&i| ys[i] > ys[i - 1] && ys[i - 1] < 0.6)
            .map_or(landing, |i| i - 1);
        let peak = ys[first_landing..].iter().copied().fold(f32::MIN, f32::max);
        peak - ys[first_landing]
    }

    /// Nudge-proof bounds: how far the numbers moved is in the assertion messages.
    #[test]
    fn zero_gravity_keeps_a_body_still() {
        let project = dropped_box(2.0);
        let (falling, _) = simulate(&project);
        let (floating, _) = simulate(&edited(&project, |p| p.gravity = [0.0; 3]));
        let end = project.settings.range.end;
        eprintln!(
            "default: y {:.3} -> {:.3}; zero gravity: y {:.3} -> {:.3}",
            box_at(&falling, 0).y,
            box_at(&falling, end).y,
            box_at(&floating, 0).y,
            box_at(&floating, end).y
        );
        assert!(box_at(&falling, end).y < 0.3, "it lands under gravity");
        for f in [0, end / 2, end] {
            let p = box_at(&floating, f);
            assert!(
                (p - Vec3::new(0.0, 2.0, 0.0)).length() < 1e-3,
                "frame {f}: {p}"
            );
        }
    }

    #[test]
    fn edited_gravity_sets_how_fast_it_falls() {
        // Released 10 m up: at 9.81 m/s² a box released 5 m up was already on the
        // ground (centre 0.25 m up) by frame 59, so it seemed to fall 4.72 m (lesson 32).
        let project = dropped_box(10.0);
        // After one second of free fall from rest: g / 2 metres.
        let drop_after_one_second = |g: f32| {
            let moon = edited(&project, |p| p.gravity = [0.0, -g, 0.0]);
            let (sim, _) = simulate(&moon);
            10.0 - box_at(&sim, 60).y
        };
        let earth = drop_after_one_second(9.81);
        let moon = drop_after_one_second(1.62);
        eprintln!("one second of fall: {earth:.4} m at 9.81, {moon:.4} m at 1.62");
        // Free fall is g / 2: 4.905 and 0.81 m. Semi-implicit Euler at 480 substeps a
        // second adds g·h·t / 2: 4.9152 and 0.8117 m.
        assert!((earth - 4.905).abs() < 0.015, "{earth}");
        assert!((moon - 0.81).abs() < 0.005, "{moon}");
        assert!(earth > 5.0 * moon, "{earth} vs {moon}");
        // Sideways gravity too.
        let sideways = edited(&project, |p| p.gravity = [3.0, 0.0, 0.0]);
        let (sim, _) = simulate(&sideways);
        let p = box_at(&sim, 60);
        assert!((p.x - 1.5).abs() < 0.01 && (p.y - 10.0).abs() < 1e-3, "{p}");
    }

    /// Stand-in for lesson 32: a free body falls g·t²/2 and a thrown one follows the
    /// parabola, with nothing taking energy from it at speed. Free fall has no knife
    /// edge (nothing in it is chaotic), so the bounds are the integrator's own error.
    #[test]
    fn a_free_body_follows_the_parabola() {
        let g = 9.81;
        // Semi-implicit Euler: n substeps of h after a start at rest fall g·h²·n(n+1)/2,
        // which is g·t²/2 plus g·h·t/2 (1.0 cm after a second at 480 substeps a second).
        let h = 1.0 / 480.0;
        let euler = |t: f32| {
            let n = (t / h).round();
            g * h * h * n * (n + 1.0) / 2.0
        };
        let project = dropped_box(10.0);
        let (sim, _) = simulate(&project);
        let fallen = 10.0 - box_at(&sim, 60).y;
        let start = Vec3::new(0.0, 10.0, 0.0);
        let thrown = {
            let mut p = project.clone();
            let id = p.characters[0].id;
            push(
                &mut p,
                id,
                0,
                "box.MetaBodies.body",
                Vec3::new(3.0, 4.0, -2.0),
            );
            p
        };
        let (flight, _) = simulate(&thrown);
        let mut worst = 0.0f32;
        for frame in [10, 30, 60, 90] {
            let t = frame as f32 / 60.0;
            let p = box_at(&flight, frame);
            let parabola = start + Vec3::new(3.0, 4.0, -2.0) * t - Vec3::Y * (g * t * t / 2.0);
            let off = (p - parabola).length();
            eprintln!(
                "thrown, frame {frame}: {p} against {parabola}, {off:.4} m off ({:.4} m \
                 for the integrator)",
                (p.y - (start.y + 4.0 * t - euler(t))).abs()
            );
            assert!(off < 0.02, "frame {frame}: {p} against {parabola}");
            // Nothing velocity-dependent: the integrator's own error, to 1 mm.
            assert!(
                (p.y - (start.y + 4.0 * t - euler(t))).abs() < 1e-3,
                "frame {frame}: {p}"
            );
            assert!((p.x - 3.0 * t).abs() < 1e-3 && (p.z + 2.0 * t).abs() < 1e-3);
            worst = worst.max(off);
        }
        eprintln!(
            "free fall in one second: {fallen:.4} m (g/2 4.905, the integrator {:.4}); \
             thrown, at most {worst:.4} m off the parabola",
            euler(1.0)
        );
        assert!((fallen - euler(1.0)).abs() < 1e-3, "fell {fallen} m");
        assert!((fallen - 4.905).abs() < 0.015, "fell {fallen} m");
    }

    #[test]
    fn edited_restitution_changes_the_bounce() {
        let project = dropped_box(1.5);
        let dead = rebound(&edited(&project, |p| {
            p.interactions.get_mut("Default").unwrap().restitution = 0.0;
        }));
        let lively = rebound(&edited(&project, |p| {
            p.interactions.get_mut("Default").unwrap().restitution = 0.9;
        }));
        eprintln!("rebound: {dead:.3} m at restitution 0, {lively:.3} m at 0.9");
        assert!(dead < 0.1, "{dead}");
        assert!(lively > 0.5, "{lively}");
    }

    #[test]
    fn a_material_pair_picks_the_interaction() {
        let project = dropped_box(1.5);
        let plain = rebound(&edited(&project, |p| {
            p.interactions.get_mut("Default").unwrap().restitution = 0.0;
        }));
        // The box's shapes are "default" and the ground "tarmac": pair them with a
        // bouncy interaction added in the same edit.
        let bouncy = rebound(&edited(&project, |p| {
            p.interactions.get_mut("Default").unwrap().restitution = 0.0;
            p.interactions.insert(
                "Bouncy".into(),
                Interaction {
                    restitution: 0.9,
                    friction: 100.0,
                    adhesion: 0.0,
                    softness: 0.0,
                },
            );
            p.pairs
                .push(("tarmac".into(), "default".into(), "Bouncy".into()));
        }));
        eprintln!("rebound: {plain:.3} m unpaired, {bouncy:.3} m paired with Bouncy");
        assert!(plain < 0.1, "{plain}");
        assert!(bouncy > 0.5, "{bouncy}");
    }

    #[test]
    fn edited_friction_changes_how_far_a_box_slides() {
        let mut project = dropped_box(0.26);
        let id = project.characters[0].id;
        push(
            &mut project,
            id,
            30,
            "box.MetaBodies.body",
            Vec3::new(3.0, 0.0, 0.0),
        );
        let slid = |friction: f32, contact: ContactModel| {
            let edit = edited(&project, |p| {
                p.contact = contact;
                p.interactions.get_mut("Default").unwrap().friction = friction;
            });
            let (sim, _) = simulate(&edit);
            box_at(&sim, edit.settings.range.end).x
        };
        let grippy = slid(100.0, ContactModel::Coulomb);
        let slippery = slid(5.0, ContactModel::Coulomb);
        eprintln!("slid {grippy:.2} m at mu 1, {slippery:.2} m at mu 0.05");
        // v² / 2μg: about 0.46 m at μ 1; at μ 0.05 it is still moving at the end.
        assert!((0.3..0.7).contains(&grippy), "{grippy}");
        assert!(slippery > 2.0 * grippy, "{slippery}");
    }

    #[test]
    fn a_different_simulation_rate_still_agrees() {
        let project = dropped_box(5.0);
        let at = |rate: f32| {
            let (sim, _) = simulate(&edited(&project, |p| p.rate = rate));
            box_at(&sim, 60).y
        };
        let (slow, normal, fast) = (at(60.0), at(120.0), at(240.0));
        eprintln!(
            "height after one second: {slow:.3} at 60 Hz, {normal:.3} at 120, {fast:.3} at 240"
        );
        assert!((normal - fast).abs() < 0.05, "{normal} vs {fast}");
        assert!((normal - slow).abs() < 0.1, "{normal} vs {slow}");
    }

    #[test]
    fn untouched_and_undone_settings_leave_the_motion_identical() {
        let project = dropped_box(1.5);
        let (reference, _) = simulate(&project);

        // Edit, then undo: same project, same motion, bit for bit.
        let mut doc = Document::new(project.clone());
        let before = doc.project.physics.clone();
        let mut after = before.clone();
        after.gravity = [0.0; 3];
        after.contact = ContactModel::Coulomb;
        doc.apply(
            "Edit physics settings",
            Edit::Physics { before, after },
            false,
        );
        doc.undo();
        assert_eq!(doc.project, project);
        let (again, _) = simulate(&doc.project);
        for f in 0..=project.settings.range.end {
            assert_eq!(box_at(&reference, f), box_at(&again, f), "frame {f}");
        }
    }

    #[test]
    fn the_default_settings_are_what_the_tests_were_tuned_on() {
        let p = PhysicsSettings::default();
        assert_eq!(p.rate, 120.0);
        assert_eq!(p.gravity, [0.0, -9.81, 0.0]);
        // Coulomb since lesson 26; the stand-ins that need force caps say so.
        assert_eq!(p.contact, ContactModel::Coulomb);
        let default = p.interactions["Default"];
        assert_eq!(
            (
                default.restitution,
                default.friction,
                default.adhesion,
                default.softness
            ),
            (0.2, 100.0, 0.0, 0.0)
        );
        assert_eq!(p.interactions["Grippy"].friction, 300.0);
        assert_eq!(p.interactions.len(), 2);
        assert_eq!(p.pairs.len(), 2);
    }

    #[test]
    fn saving_and_reopening_keeps_the_edited_settings_and_the_motion() {
        let project = edited(&dropped_box(1.5), |p| {
            p.gravity = [0.5, -4.0, 0.0];
            p.rate = 240.0;
            p.contact = ContactModel::Coulomb;
            p.pairs
                .push(("steel".into(), "ice".into(), "Grippy".into()));
        });
        let text = crate::io::to_string(&project).unwrap();
        let reopened = crate::io::from_str(&text).unwrap();
        assert_eq!(reopened.physics, project.physics);
        let (a, _) = simulate(&project);
        let (b, _) = simulate(&reopened);
        let end = project.settings.range.end;
        assert_eq!(box_at(&a, end), box_at(&b, end));
    }
}
