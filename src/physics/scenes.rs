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
    PhysicsRig, ShapeDef,
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
    };
    (
        skeleton,
        PhysicsRig {
            bodies,
            joints: rig_joints,
            muscle_sets: vec![set],
            isotropic_inertia: true,
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
        .map(|f| (f, 40.0 * (f as f32 / 60.0 * std::f32::consts::TAU).sin()))
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
        .map(|f| (f, 10.0 * (f as f32 / 60.0 * std::f32::consts::TAU).sin()))
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
        vec![channel("body", Property::PosZ, &[(0, -6.0), (120, 24.0)])],
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
            "balance" => balance_push(1.5, Vec3::Z, true),
            "balance_step" => balance_push(1.5, -Vec3::Z, true),
            "balance_steps" => balance_push(HARD_PUSHES[1], Vec3::Z, true),
            "stagger" => pushed(Some("Stagger"), HARD_PUSHES[1], Vec3::Z),
            "catch_fall" => catch_fall_shove(Vec3::new(0.0, 0.0, 3.5), true),
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
        let tilt = (rotation * Vec3::Z).dot(Vec3::Y).abs().min(1.0).acos();
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
                let angle = up.z.atan2(up.y);
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
    #[test]
    fn car_hit_keeps_joints_together() {
        check(car_hit());
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
    /// backwards, 3-7 of 22 sideways and 6-11 of 12 forwards; each minimum here is one
    /// below the worst.
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
    /// (`nudge_survey`), the fewest after one hard push was 3, the total 17 to 31, and
    /// the feet went 2.3 to 4.2 times as far as Balance's.
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
    /// `NM_SURVEY=car|balance|stagger` runs one part (all when unset); `NM_NUDGES` and
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
            if name.contains("Large File") {
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
