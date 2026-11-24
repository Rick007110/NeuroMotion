//! Runs a project through the physics world and caches every frame.
//!
//! Characters with a physics rig become groups of rigid bodies. Each frame, every
//! character is at a simulation level taken from its simulation and transition events:
//! animated (bodies follow the keys and nothing touches them), collision-only (follow
//! the keys, push others away), collision-with-momentum (also hand over their
//! velocity), or simulated (fully dynamic). Kinematic bodies always move with the
//! animation's velocity, so switching to simulated carries the motion's momentum into
//! physics. Simulated characters' muscles pull towards an active pose, an active
//! animation, or the pose they had when simulation began (see docs/physics-design.md),
//! and behaviours (balance, catch fall) adjust them as they go (docs/behaviours.md).
//!
//! Results are computed incrementally under a time budget so editing never blocks
//! the UI, and recomputed from the start whenever the document changes.

use std::time::Instant;

use glam::{Quat, Vec3};

use super::behaviour::{Aim, Behaviour, Drive, Layout, LookAndReach};
use super::world::{Body, CollisionFlags, Joint, Lock, Motion, Muscle, Shape, World};
use crate::document::physics::{Gains, MuscleGains, MuscleSet, PhysicsRig, SimLevel};
use crate::document::{CharacterKind, Clip, ClipKind, Pose, Project, world_transforms};

/// One character's simulated state at a frame.
#[derive(Clone, Debug)]
pub struct CharacterFrame {
    /// World rotation and position of every skeleton joint.
    pub joints: Vec<(Quat, Vec3)>,
    /// World transform of every rig body.
    pub bodies: Vec<(Quat, Vec3)>,
    pub level: SimLevel,
}

/// All characters at one frame (`None` for characters without physics).
pub type Frame = Vec<Option<CharacterFrame>>;

#[derive(Default)]
pub struct Simulation {
    revision: Option<u64>,
    start: i32,
    end: i32,
    frames: Vec<Frame>,
    running: Option<Running>,
}

struct Running {
    world: World,
    bindings: Vec<Option<Binding>>,
    plans: Vec<Plan>,
    constraints: Vec<Constraint>,
    /// Last frame computed.
    frame: i32,
}

/// Where a character's bodies and joints live in the world.
struct Binding {
    first_body: usize,
    first_joint: usize,
    rig: PhysicsRig,
    /// Skeleton joint → rig body hanging from it.
    body_of_joint: Vec<Option<usize>>,
    level: SimLevel,
    /// Muscle targets (joint local rotations) captured when simulation began.
    frozen: Vec<Quat>,
    /// Where its limbs are, for behaviours (`None` if it isn't a biped).
    layout: Option<Layout>,
}

/// A character's physics events, read from its clips.
#[derive(Default)]
struct Plan {
    /// (frame, level) switches in frame order.
    switches: Vec<(i32, SimLevel)>,
    momentum: Vec<(i32, f32)>,
    /// (frame, Δv, rig body indices).
    forces: Vec<(i32, Vec3, Vec<usize>)>,
    /// (start, end) windows blending simulation back to animation.
    transitions: Vec<(i32, i32)>,
    poses: Vec<PoseTarget>,
    /// (start, end, strength) of clips whose animation the muscles follow.
    active: Vec<(i32, i32, f32)>,
    /// Behaviour events NeuroMotion has a controller for.
    behaviours: Vec<BehaviourSpan>,
    /// Joints cut or switched off, from the frame each event starts.
    severs: Vec<Sever>,
}

/// A behaviour event: when it runs, which controller, and the settings it reads.
struct BehaviourSpan {
    start: i32,
    end: i32,
    behaviour: Behaviour,
    /// The event's settings, as imported.
    source: std::collections::BTreeMap<String, String>,
    /// Look and Reach's settings, with its targets found in the world (`build`).
    reach: Option<LookAndReach>,
}

/// A sever event aimed at one joint of the rig.
#[derive(Clone, Copy)]
struct Sever {
    /// First frame the joint is severed; it stays so for the rest of the scene.
    start: i32,
    /// Index into the rig's joints.
    joint: usize,
    /// Cut the joint, or only switch its muscle off.
    cut: bool,
}

/// A lock/join constraint event, across characters.
struct Constraint {
    start: i32,
    end: i32,
    kind: LockKind,
    /// World body indices, in event order.
    bodies: Vec<usize>,
    linear: Vec3,
    angular: Vec3,
    /// Captured when the constraint switches on.
    locks: Option<Vec<Lock>>,
}

#[derive(Clone, Copy, PartialEq)]
enum LockKind {
    /// Each body to where it is in the world.
    World { position: bool, orientation: bool },
    /// Every body to the last one listed.
    Join,
    /// Like a join; a lone body is held to its parent body in its own rig.
    Parent,
}

struct PoseTarget {
    start: i32,
    end: i32,
    blend: i32,
    strength: f32,
    /// Local rotation target per skeleton joint (`None` = not part of the pose).
    targets: Vec<Option<Quat>>,
}

impl Simulation {
    /// Throws away results computed for an older document revision.
    pub fn sync(&mut self, revision: u64) {
        if self.revision != Some(revision) {
            *self = Self {
                revision: Some(revision),
                ..Self::default()
            };
        }
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Whether any character in the project is ever simulated.
    pub fn is_needed(project: &Project) -> bool {
        project.characters.iter().enumerate().any(|(i, c)| {
            c.physics.is_some()
                && c.kind != CharacterKind::Environment
                && plan(project, i)
                    .switches
                    .iter()
                    .map(|s| s.1)
                    .chain([c.default_level])
                    .any(|l| l != SimLevel::Animated)
        })
    }

    /// Computes more frames until `deadline`. Returns true once the whole range is done.
    pub fn advance(&mut self, project: &Project, deadline: Instant) -> bool {
        if self.running.is_none() && self.frames.is_empty() {
            self.begin(project);
        }
        let Some(running) = self.running.as_mut() else {
            return true;
        };
        while running.frame < self.end {
            let next = step_frame(project, running);
            self.frames.push(next);
            if Instant::now() >= deadline {
                break;
            }
        }
        let done = running.frame >= self.end;
        if done {
            self.running = None;
        }
        done
    }

    /// The simulated state at `frame`, if computed.
    pub fn frame(&self, frame: i32) -> Option<&Frame> {
        let index = usize::try_from(frame - self.start).ok()?;
        self.frames.get(index)
    }

    /// (frames computed, frames in range).
    pub fn progress(&self) -> (usize, usize) {
        (
            self.frames.len(),
            (self.end - self.start + 1).max(0) as usize,
        )
    }

    fn begin(&mut self, project: &Project) {
        let range = project.settings.range;
        self.start = range.start;
        self.end = range.end;
        let mut running = build(project);
        running.frame = range.start;
        self.frames
            .push(record(project, &running, range.start as f32));
        self.running = Some(running);
    }
}

/// World transforms of a rig's bodies for given skeleton joint transforms.
pub fn rig_bodies(rig: &PhysicsRig, joints: &[(Quat, Vec3)]) -> Vec<(Quat, Vec3)> {
    rig.bodies
        .iter()
        .map(|body| {
            let (jr, jp) = joints
                .get(body.joint)
                .copied()
                .unwrap_or((Quat::IDENTITY, Vec3::ZERO));
            let fr = Quat::from_array(body.frame.rotation).normalize();
            (
                (jr * fr).normalize(),
                jp + jr * Vec3::from(body.frame.position),
            )
        })
        .collect()
}

// ---------------------------------------------------------------------------------
// Building the world

fn build(project: &Project) -> Running {
    let settings = &project.physics;
    let mut world = World::new(Vec3::from(settings.gravity), settings.contact);

    // Materials: every name used by the ground or any shape.
    let mut materials: Vec<String> = Vec::new();
    let mut material = |name: &str| -> usize {
        if let Some(i) = materials.iter().position(|m| m == name) {
            return i;
        }
        materials.push(name.to_owned());
        materials.len() - 1
    };
    if let Some(ground) = &settings.ground {
        world.ground = Some((ground.height, material(&ground.material)));
    }

    let start = project.settings.range.start as f32;
    let poses = project.evaluate(start);
    let mut bindings = Vec::new();
    let mut plans = Vec::new();
    for (ci, character) in project.characters.iter().enumerate() {
        plans.push(plan(project, ci));
        let Some(rig) = &character.physics else {
            bindings.push(None);
            continue;
        };
        let skeleton = &character.skeleton;
        let rest = world_transforms(skeleton, &Pose::rest(skeleton.joints.len()));
        let now = world_transforms(skeleton, &poses[ci]);
        let mut body_of_joint = vec![None; skeleton.joints.len()];
        for (bi, body) in rig.bodies.iter().enumerate() {
            if body.joint < body_of_joint.len() && body_of_joint[body.joint].is_none() {
                body_of_joint[body.joint] = Some(bi);
            }
        }
        let place = |transforms: &[(Quat, Vec3)], bi: usize| {
            let body = &rig.bodies[bi];
            let (jr, jp) = transforms[body.joint];
            let frame_rot = Quat::from_array(body.frame.rotation).normalize();
            (
                (jr * frame_rot).normalize(),
                jp + jr * Vec3::from(body.frame.position),
            )
        };
        let first_body = world.bodies.len();
        let first_shape = world.shapes.len();
        let level = initial_level(&plans[ci], character.default_level, start as i32);

        for (bi, body) in rig.bodies.iter().enumerate() {
            let (rotation, position) = place(&now, bi);
            let inertia = if rig.isotropic_inertia {
                Vec3::splat(body.inertia.iter().sum::<f32>() / 3.0)
            } else {
                Vec3::from(body.inertia)
            };
            let parent = parent_body(skeleton, &body_of_joint, body.joint).map(|p| first_body + p);
            world.bodies.push(Body {
                position,
                rotation,
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
                inv_mass: if body.mass > 0.0 {
                    1.0 / body.mass
                } else {
                    0.0
                },
                inv_inertia: Vec3::new(recip(inertia.x), recip(inertia.y), recip(inertia.z)),
                motion: motion_for(level, 1.0),
                group: ci,
                parent,
            });
            for shape in &body.shapes {
                let (r, p) = shape_frame(shape);
                world.shapes.push(Shape {
                    body: Some(first_body + bi),
                    geometry: shape.geometry,
                    rotation: r,
                    position: p,
                    material: material(&shape.material),
                    flags: CollisionFlags {
                        parent: shape.collides_with_parent,
                        siblings: shape.collides_with_siblings,
                        ancestors: shape.collides_with_ancestors,
                    },
                });
            }
        }
        unflag_overlapping_parents(&mut world, first_shape, first_body, |bi| place(&rest, bi));
        let first_joint = world.joints.len();
        for jd in &rig.joints {
            // Keep world joints aligned with rig joints: invalid ones become disabled placeholders.
            let valid = jd.body < rig.bodies.len()
                && jd.parent_body < rig.bodies.len()
                && jd.joint < rest.len();
            let Some(pj) = skeleton
                .joints
                .get(jd.joint)
                .and_then(|j| j.parent)
                .filter(|_| valid)
            else {
                let mut placeholder = Joint::new(
                    first_body,
                    first_body,
                    Vec3::ZERO,
                    Vec3::ZERO,
                    Quat::IDENTITY,
                    Quat::IDENTITY,
                    Quat::IDENTITY,
                    Quat::IDENTITY,
                    jd.hard,
                    jd.soft,
                    0.0,
                    0.0,
                );
                placeholder.enabled = false;
                world.joints.push(placeholder);
                continue;
            };
            let (qa, xa) = place(&rest, jd.parent_body);
            let (qb, xb) = place(&rest, jd.body);
            let (jr, jp) = rest[jd.joint];
            let (ia, ib) = (qa.inverse(), qb.inverse());
            let joint = Joint::new(
                first_body + jd.parent_body,
                first_body + jd.body,
                ia * (jp - xa),
                ib * (jp - xb),
                (ia * jr * Quat::from_array(jd.primary)).normalize(),
                (ib * jr * Quat::from_array(jd.secondary)).normalize(),
                (ia * rest[pj].0).normalize(),
                (ib * jr).normalize(),
                jd.hard,
                jd.soft,
                jd.soft_stiffness,
                jd.soft_damping,
            );
            world.joints.push(joint);
        }

        bindings.push(Some(Binding {
            first_body,
            first_joint,
            rig: rig.clone(),
            body_of_joint,
            level,
            frozen: local_rotations(skeleton, &poses[ci]),
            layout: Layout::new(rig, skeleton),
        }));
    }

    let settings = &project.physics;
    world.set_materials(materials.len(), |a, b| {
        settings.interaction(&materials[a], &materials[b])
    });
    // Targets name bodies of any character, so they are found once all are in the world.
    for plan in &mut plans {
        for span in &mut plan.behaviours {
            if span.behaviour == Behaviour::LookAndReach {
                let aim = |name: &str| aim_at(project, &bindings, name);
                span.reach = Some(LookAndReach::read(&span.source, &aim));
            }
        }
    }
    let constraints = constraints(project, &bindings);
    Running {
        world,
        bindings,
        plans,
        constraints,
        frame: 0,
    }
}

/// Reads lock/join constraint events into world-body terms.
fn constraints(project: &Project, bindings: &[Option<Binding>]) -> Vec<Constraint> {
    let world_body =
        |(ci, bi): (usize, usize)| bindings.get(ci)?.as_ref().map(|b| b.first_body + bi);
    constraint_clips(project)
        .filter_map(|clip| {
            let kind = lock_kind(clip)?;
            let bodies: Vec<usize> = constraint_targets(project, clip)
                .into_iter()
                .filter_map(world_body)
                .collect();
            Some(Constraint {
                start: clip.props.start,
                end: clip.props.end(),
                kind,
                bodies,
                linear: clip_vec3(clip, "positionStrength"),
                angular: clip_vec3(clip, "angularStrength"),
                locks: None,
            })
        })
        .collect()
}

/// A behaviour's target as a point on a body in the world. Scenes name a body
/// (`character.MetaBodies.Body`: its centre) or a collision surface by its path
/// (`character.root.<joints>.Body.Surface`: the body it is on; the shape's centre if
/// the body has only one, as shapes aren't named, else the body's).
fn aim_at(project: &Project, bindings: &[Option<Binding>], name: &str) -> Option<Aim> {
    let (character, path) = name.trim().split_once('.')?;
    let ci = project
        .characters
        .iter()
        .position(|c| c.name == character)?;
    let binding = bindings.get(ci)?.as_ref()?;
    let rig = &binding.rig;
    let mut parts = path.rsplit('.');
    let (last, before) = (parts.next()?, parts.next());
    let (body, surface) = match rig.body_index(last) {
        Some(b) => (b, false),
        None => (rig.body_index(before?)?, true),
    };
    let offset = match rig.bodies[body].shapes.as_slice() {
        [one] if surface => Vec3::from(one.frame.position),
        _ => Vec3::ZERO,
    };
    Some(Aim {
        body: binding.first_body + body,
        offset,
    })
}

fn constraint_clips(project: &Project) -> impl Iterator<Item = &Clip> {
    project.clips.iter().filter(|c| {
        c.props.enabled && matches!(&c.kind, ClipKind::Event { event } if event == "Constraint")
    })
}

fn lock_kind(clip: &Clip) -> Option<LockKind> {
    Some(match clip.source.get("lockType").map(String::as_str)? {
        "Lock" => LockKind::World {
            position: true,
            orientation: true,
        },
        "LockPosition" => LockKind::World {
            position: true,
            orientation: false,
        },
        "LockOrientation" => LockKind::World {
            position: false,
            orientation: true,
        },
        "JoinBodies" => LockKind::Join,
        "LockWithParent" => LockKind::Parent,
        _ => return None,
    })
}

/// A constraint event's target bodies as (character index, rig body index), in event
/// order. Targets are named `<character>.MetaBodies.<body>`.
fn constraint_targets(project: &Project, clip: &Clip) -> Vec<(usize, usize)> {
    let resolve = |name: &str| -> Option<(usize, usize)> {
        let mut parts = name.trim().split(".MetaBodies.");
        let (character, body) = (parts.next()?, parts.next()?);
        let ci = project
            .characters
            .iter()
            .position(|c| c.name == character)?;
        Some((
            ci,
            project.characters[ci].physics.as_ref()?.body_index(body)?,
        ))
    };
    clip.source
        .get("targets")
        .map_or_else(Vec::new, |t| t.split(',').filter_map(resolve).collect())
}

/// Which body each lock of a constraint holds, and to what (`None`: the world).
fn lock_pairs<T: Copy + PartialEq>(
    kind: LockKind,
    bodies: &[T],
    parent: impl Fn(T) -> Option<T>,
) -> Vec<(Option<T>, T)> {
    match kind {
        LockKind::World { .. } => bodies.iter().map(|&b| (None, b)).collect(),
        LockKind::Parent if bodies.len() == 1 => parent(bodies[0])
            .map(|p| (Some(p), bodies[0]))
            .into_iter()
            .collect(),
        LockKind::Join | LockKind::Parent => {
            let Some((&anchor, rest)) = bodies.split_last() else {
                return Vec::new();
            };
            rest.iter()
                .filter(|&&b| b != anchor)
                .map(|&b| (Some(anchor), b))
                .collect()
        }
    }
}

fn clip_vec3(clip: &Clip, key: &str) -> Vec3 {
    clip.source
        .get(key)
        .and_then(|v| parse_vec3(v))
        .unwrap_or(Vec3::ZERO)
}

/// Skeleton joints that sever events cut, as (character index, skeleton joint index,
/// first frame cut), for the tests: a cut joint's bone comes apart on purpose.
#[cfg(test)]
pub(crate) fn cut_joints(project: &Project) -> Vec<(usize, usize, i32)> {
    let mut out = Vec::new();
    for (ci, character) in project.characters.iter().enumerate() {
        let Some(rig) = &character.physics else {
            continue;
        };
        for sever in plan(project, ci).severs.iter().filter(|s| s.cut) {
            out.push((ci, rig.joints[sever.joint].joint, sever.start));
        }
    }
    out
}

/// A lock as the tests see it: bodies as (character index, rig body index).
#[cfg(test)]
pub(crate) struct LockSpan {
    pub start: i32,
    pub end: i32,
    pub anchor: Option<(usize, usize)>,
    pub body: (usize, usize),
    /// Position strength per axis (0 = free).
    pub linear: Vec3,
}

/// Every lock the project's constraint events create.
#[cfg(test)]
pub(crate) fn lock_spans(project: &Project) -> Vec<LockSpan> {
    let mut out = Vec::new();
    for clip in constraint_clips(project) {
        let Some(kind) = lock_kind(clip) else {
            continue;
        };
        let linear = match kind {
            LockKind::World {
                position: false, ..
            } => Vec3::ZERO,
            _ => clip_vec3(clip, "positionStrength"),
        };
        let parent = |(ci, bi): (usize, usize)| {
            let c = &project.characters[ci];
            let rig = c.physics.as_ref()?;
            let mut joint = c.skeleton.joints[rig.bodies[bi].joint].parent;
            while let Some(j) = joint {
                if let Some(p) = rig.bodies.iter().position(|b| b.joint == j) {
                    return Some((ci, p));
                }
                joint = c.skeleton.joints[j].parent;
            }
            None
        };
        for (anchor, body) in lock_pairs(kind, &constraint_targets(project, clip), parent) {
            out.push(LockSpan {
                start: clip.props.start,
                end: clip.props.end(),
                anchor,
                body,
                linear,
            });
        }
    }
    out
}

/// Switches constraints on and off for this frame, capturing offsets as they start.
fn update_constraints(running: &mut Running, frame: i32) {
    let world = &mut running.world;
    world.locks.clear();
    for c in running.constraints.iter_mut() {
        if frame < c.start || frame >= c.end {
            c.locks = None;
            continue;
        }
        let bodies = &world.bodies;
        let locks = c.locks.get_or_insert_with(|| {
            let (position, orientation) = match c.kind {
                LockKind::World {
                    position,
                    orientation,
                } => (position, orientation),
                LockKind::Join | LockKind::Parent => (true, true),
            };
            lock_pairs(c.kind, &c.bodies, |b| bodies[b].parent)
                .into_iter()
                .map(|(anchor, b)| {
                    let (ar, ap) = anchor.map_or((Quat::IDENTITY, Vec3::ZERO), |a| {
                        (bodies[a].rotation, bodies[a].position)
                    });
                    Lock {
                        anchor,
                        body: b,
                        target: (
                            ar.inverse() * bodies[b].rotation,
                            ar.inverse() * (bodies[b].position - ap),
                        ),
                        linear: if position { c.linear } else { Vec3::ZERO },
                        angular: if orientation { c.angular } else { Vec3::ZERO },
                    }
                })
                .collect()
        });
        world.locks.extend(locks.iter().copied());
    }
}

/// A shape that may collide with its parent body (endorphin's `collidesWithParent`) but
/// already overlaps one of the parent's shapes in the rig's rest pose would be pushed
/// off it at once, up to 3 m/s, every time the simulation starts: Back-Flip-To-Combo's
/// head, whose shapes sit inside the neck, was thrown 50° forward in its first
/// ten frames (lesson 19). Such a shape doesn't collide with its parent,
/// unless the parent is fixed: a prop sitting in the scenery it belongs to (Heli
/// Leap's and Bendy Stick's background) stands on it.
fn unflag_overlapping_parents(
    world: &mut World,
    first_shape: usize,
    first_body: usize,
    rest: impl Fn(usize) -> (Quat, Vec3),
) {
    let placed = |world: &World, s: usize| {
        let shape = &world.shapes[s];
        let (r, p) = rest(shape.body? - first_body);
        Some(super::collide::Placed {
            geometry: shape.geometry,
            rotation: r * shape.rotation,
            position: p + r * shape.position,
        })
    };
    let shapes = first_shape..world.shapes.len();
    for s in shapes.clone() {
        let Some(body) = world.shapes[s].body else {
            continue;
        };
        let Some(parent) = world.bodies[body].parent else {
            continue;
        };
        // Resting on a fixed parent (a prop on the scenery it hangs from) is support.
        if !world.shapes[s].flags.parent || world.bodies[parent].inv_mass == 0.0 {
            continue;
        }
        let overlaps = shapes.clone().any(|o| {
            let (Some(a), Some(b)) = (placed(world, s), placed(world, o)) else {
                return false;
            };
            let mut contacts = Vec::new();
            if world.shapes[o].body == Some(parent) {
                super::collide::between(&a, &b, 0.0, &mut contacts);
            }
            contacts.iter().any(|c| c.separation < 0.0)
        });
        if overlaps {
            world.shapes[s].flags.parent = false;
        }
    }
}

fn shape_frame(shape: &crate::document::physics::ShapeDef) -> (Quat, Vec3) {
    (
        Quat::from_array(shape.frame.rotation).normalize(),
        Vec3::from(shape.frame.position),
    )
}

fn parent_body(
    skeleton: &crate::document::SkeletonRef,
    body_of_joint: &[Option<usize>],
    joint: usize,
) -> Option<usize> {
    let mut j = skeleton.joints[joint].parent;
    while let Some(i) = j {
        if let Some(b) = body_of_joint[i] {
            return Some(b);
        }
        j = skeleton.joints[i].parent;
    }
    None
}

fn recip(v: f32) -> f32 {
    if v > 1e-12 { 1.0 / v } else { 0.0 }
}

fn motion_for(level: SimLevel, momentum: f32) -> Motion {
    match level {
        SimLevel::Animated => Motion::Ghost,
        SimLevel::CollisionOnly => Motion::Kinematic { momentum: 0.0 },
        SimLevel::CollisionWithMomentum => Motion::Kinematic { momentum },
        SimLevel::Simulated => Motion::Dynamic,
    }
}

/// Every joint's local rotation (relative to its parent joint) for a pose.
fn local_rotations(skeleton: &crate::document::SkeletonRef, pose: &Pose) -> Vec<Quat> {
    skeleton
        .joints
        .iter()
        .enumerate()
        .map(|(i, j)| {
            crate::document::pose_local_rotation(
                j.rest_rotation,
                pose.joints.get(i).copied().unwrap_or([0.0; 6]),
            )
        })
        .collect()
}

// ---------------------------------------------------------------------------------
// Reading events

fn plan(project: &Project, character_index: usize) -> Plan {
    let character = &project.characters[character_index];
    let fps = project.settings.fps.max(1.0);
    let mut plan = Plan::default();
    let number = |clip: &crate::document::Clip, key: &str| {
        clip.source
            .get(key)
            .and_then(|v| v.trim().parse::<f32>().ok())
    };
    for clip in &project.clips {
        if !clip.props.enabled {
            continue;
        }
        // A force pushes the bodies it names, whichever character's track holds it.
        if matches!(&clip.kind, ClipKind::Event { event } if event == "Force") {
            let own = project.clip_character(clip).map(|c| c.id) == Some(character.id);
            if let (Some(rig), Some(dv)) = (
                &character.physics,
                clip.source.get("matrix.Vi").and_then(|v| parse_vec3(v)),
            ) && let Some(bodies) = force_bodies(clip, &character.name, rig, own)
            {
                plan.forces.push((clip.props.start, dv, bodies));
            }
            continue;
        }
        if project.clip_character(clip).map(|c| c.id) != Some(character.id) {
            continue;
        }
        let (start, end) = (clip.props.start, clip.props.end());
        match &clip.kind {
            ClipKind::Motion => {
                // Imported animations say whether they're active; our own clips always are.
                if clip.source.get("active").is_none_or(|v| v == "true") {
                    plan.active
                        .push((start, end, number(clip, "strength").unwrap_or(1.0)));
                }
            }
            ClipKind::Event { event } => match event.as_str() {
                "Simulation" => {
                    if let Some(level) = clip
                        .source
                        .get("physicsLevel")
                        .and_then(|v| SimLevel::from_endorphin(v))
                    {
                        plan.switches.push((start, level));
                        plan.momentum
                            .push((start, number(clip, "momentumScale").unwrap_or(1.0)));
                    }
                }
                "Transition" => {
                    plan.transitions.push((start, end));
                    let level = clip
                        .source
                        .get("requiredPhysicsLevel")
                        .and_then(|v| SimLevel::from_endorphin(v))
                        .unwrap_or(SimLevel::Animated);
                    plan.switches.push((end, level));
                }
                "Behaviour" => {
                    let name = clip.source.get("behaviourName").unwrap_or(&clip.props.name);
                    if let Some(behaviour) = Behaviour::from_name(name) {
                        plan.behaviours.push(BehaviourSpan {
                            start,
                            end,
                            behaviour,
                            source: clip.source.clone(),
                            reach: None,
                        });
                    }
                }
                "Sever" => {
                    let Some(rig) = &character.physics else {
                        continue;
                    };
                    let cut = clip
                        .source
                        .get("severType")
                        .is_none_or(|v| !v.eq_ignore_ascii_case("MuscleOff"));
                    let named = clip.source.get("targets").into_iter().flat_map(|t| {
                        t.split(',')
                            .filter_map(|n| rig.body_index(n.trim().rsplit('.').next()?))
                    });
                    for body in named {
                        // A body with no joint (the root) has nothing to cut.
                        if let Some(joint) = rig.joints.iter().position(|j| j.body == body) {
                            plan.severs.push(Sever { start, joint, cut });
                        }
                    }
                }
                "Active pose" => {
                    let skeleton = &character.skeleton;
                    let mut pose = Pose::rest(skeleton.joints.len());
                    for channel in &clip.channels {
                        if let (Some(j), Some(v)) =
                            (skeleton.joint_index(&channel.joint), channel.sample(0.0))
                        {
                            pose.joints[j][channel.property.index()] = v;
                        }
                    }
                    // The joints it drives: those its `targets` name
                    // (`character.Joints.Name`, as endorphin lists them), or all of them.
                    // Scenes split the body between overlapping events, legs in one and
                    // arms in another. A joint the pose leaves at rest has no channel
                    // (the importer drops those), but is still held at rest.
                    let named: Vec<usize> = clip
                        .source
                        .get("targets")
                        .into_iter()
                        .flat_map(|t| t.split(','))
                        .filter_map(|n| skeleton.joint_index(n.trim().rsplit('.').next()?))
                        .collect();
                    let present: Vec<bool> = (0..skeleton.joints.len())
                        .map(|j| named.is_empty() || named.contains(&j))
                        .collect();
                    let locals = local_rotations(skeleton, &pose);
                    plan.poses.push(PoseTarget {
                        start,
                        end,
                        blend: number(clip, "entryBlendPeriod")
                            .map_or(0, |s| (s * fps).round() as i32),
                        strength: number(clip, "strength").unwrap_or(1.0),
                        targets: locals
                            .into_iter()
                            .zip(present)
                            .map(|(q, p)| p.then_some(q))
                            .collect(),
                    });
                }
                _ => {}
            },
        }
    }
    plan.switches.sort_by_key(|s| s.0);
    plan.momentum.sort_by_key(|s| s.0);
    plan
}

/// The bodies of a character's rig (`rig`, named `character`) a force event pushes:
/// every body it names as `character.MetaBodies.<body>`, or a bare body name when the
/// clip is on the character's own track (`own`). A force naming no body pushes every
/// body of its own character (an empty list). `None` when it pushes none of them: a
/// force aimed at another character's body (Catch Rope's push on its rope sits on the
/// environment's track) must not fall back to pushing everything here.
fn force_bodies(clip: &Clip, character: &str, rig: &PhysicsRig, own: bool) -> Option<Vec<usize>> {
    let targets = crate::document::events::targets(&clip.source);
    if targets.is_empty() {
        return own.then(Vec::new);
    }
    let bodies: Vec<usize> = targets
        .iter()
        .filter(|t| t.character == character || (t.character.is_empty() && own))
        .filter_map(|t| rig.body_index(&t.body))
        .collect();
    (!bodies.is_empty()).then_some(bodies)
}

fn parse_vec3(text: &str) -> Option<Vec3> {
    let v: Vec<f32> = text
        .split(',')
        .filter_map(|p| p.trim().parse().ok())
        .collect();
    (v.len() == 3).then(|| Vec3::new(v[0], v[1], v[2]))
}

fn level_at(plan: &Plan, default: SimLevel, frame: i32) -> SimLevel {
    plan.switches
        .iter()
        .rev()
        .find(|(f, _)| *f <= frame)
        .map_or(default, |s| s.1)
}

fn initial_level(plan: &Plan, default: SimLevel, frame: i32) -> SimLevel {
    level_at(plan, default, frame)
}

fn momentum_at(plan: &Plan, frame: i32) -> f32 {
    plan.momentum
        .iter()
        .rev()
        .find(|(f, _)| *f <= frame)
        .map_or(1.0, |m| m.1)
}

// ---------------------------------------------------------------------------------
// Stepping

fn step_frame(project: &Project, running: &mut Running) -> Frame {
    let fps = project.settings.fps.max(1.0);
    let frame = running.frame;
    let ticks = (project.physics.rate / fps).round().max(1.0) as usize;
    let dt = 1.0 / fps / ticks as f32;

    // Level changes and impulses happen at frame boundaries.
    for (ci, binding) in running.bindings.iter_mut().enumerate() {
        let Some(binding) = binding else { continue };
        let character = &project.characters[ci];
        let plan = &running.plans[ci];
        let level = level_at(plan, character.default_level, frame);
        if level != binding.level {
            if level == SimLevel::Simulated {
                // Hold the pose the character has as it goes limp-ish into physics.
                binding.frozen = current_locals(&running.world, binding, project, ci, frame as f32);
            }
            binding.level = level;
        }
        let motion = motion_for(level, momentum_at(plan, frame));
        for b in 0..binding.rig.bodies.len() {
            running.world.bodies[binding.first_body + b].motion = motion;
        }
        // Severed from the event's first frame on. This is read from the plan every
        // frame, so a restarted simulation needs no remembered state. A cut joint stops
        // acting entirely, and its child body no longer counts the old parent as its
        // parent, so the two collide like any unjointed bodies (rather than being left
        // to pass through each other as neighbours in a skeleton do).
        for sever in plan.severs.iter().filter(|s| s.cut && s.start <= frame) {
            let child = binding.first_body + binding.rig.joints[sever.joint].body;
            let joint = &mut running.world.joints[binding.first_joint + sever.joint];
            joint.enabled = false;
            joint.muscle = None;
            running.world.bodies[child].parent = None;
        }
        for (start, dv, bodies) in &plan.forces {
            if *start == frame && level == SimLevel::Simulated {
                let targets: Vec<usize> = if bodies.is_empty() {
                    (0..binding.rig.bodies.len()).collect()
                } else {
                    bodies.clone()
                };
                for b in targets {
                    running.world.bodies[binding.first_body + b].linear_velocity += *dv;
                }
            }
        }
    }

    update_constraints(running, frame);

    for tick in 0..ticks {
        let t = frame as f32 + (tick + 1) as f32 / ticks as f32;
        let poses = project.evaluate(t);
        for (ci, binding) in running.bindings.iter().enumerate() {
            let Some(binding) = binding else { continue };
            let character = &project.characters[ci];
            let targets = world_transforms(&character.skeleton, &poses[ci]);
            if binding.level == SimLevel::Simulated {
                set_muscles(
                    &mut running.world,
                    binding,
                    &running.plans[ci],
                    project,
                    ci,
                    &poses[ci],
                    frame,
                    t,
                );
            } else {
                // Move kinematic bodies onto the animation, with matching velocity.
                let bodies = binding.first_body..binding.first_body + binding.rig.bodies.len();
                let moves: Vec<(Quat, Vec3, Vec3, Vec3)> = rig_bodies(&binding.rig, &targets)
                    .into_iter()
                    .zip(&running.world.bodies[bodies.clone()])
                    .map(|((rot, pos), b)| {
                        let mut dq = rot * b.rotation.inverse();
                        if dq.w < 0.0 {
                            dq = -dq;
                        }
                        (rot, pos, (pos - b.position) / dt, dq.to_scaled_axis() / dt)
                    })
                    .collect();
                // A cut, or snapping back from simulation: place the whole character, don't
                // fling it (or what it holds or touches).
                let jump = moves
                    .iter()
                    .any(|&(.., v, w)| v.length() * dt > JUMP || w.length() * dt > JUMP_TURN);
                for (b, (rot, pos, v, w)) in running.world.bodies[bodies].iter_mut().zip(moves) {
                    if jump {
                        (
                            b.position,
                            b.rotation,
                            b.linear_velocity,
                            b.angular_velocity,
                        ) = (pos, rot, Vec3::ZERO, Vec3::ZERO);
                    } else {
                        (b.linear_velocity, b.angular_velocity) = (v, w);
                    }
                }
                let joints = binding.first_joint..binding.first_joint + binding.rig.joints.len();
                for joint in &mut running.world.joints[joints] {
                    joint.muscle = None;
                }
            }
        }
        running.world.step(dt);
        // Snap kinematic bodies exactly onto the animation (integration drifts slightly).
        for (ci, binding) in running.bindings.iter().enumerate() {
            let Some(binding) = binding else { continue };
            if binding.level == SimLevel::Simulated {
                continue;
            }
            let targets = world_transforms(&project.characters[ci].skeleton, &poses[ci]);
            for (bi, body) in binding.rig.bodies.iter().enumerate() {
                let (jr, jp) = targets[body.joint];
                let fr = Quat::from_array(body.frame.rotation).normalize();
                let b = &mut running.world.bodies[binding.first_body + bi];
                b.rotation = (jr * fr).normalize();
                b.position = jp + jr * Vec3::from(body.frame.position);
            }
        }
    }
    running.frame += 1;
    record(project, running, running.frame as f32)
}

/// A keyed body moving further (m) or turning further (rad) than this in one physics
/// tick (60 m/s or 120 rad/s at 120 Hz) is jumping, not moving, and its character is
/// placed without velocity.
const JUMP: f32 = 0.5;
const JUMP_TURN: f32 = 1.0;

/// Points each simulated joint's muscle at this frame's target pose.
fn set_muscles(
    world: &mut World,
    binding: &Binding,
    plan: &Plan,
    project: &Project,
    ci: usize,
    anim: &Pose,
    frame: i32,
    t: f32,
) {
    let skeleton = &project.characters[ci].skeleton;
    let active = plan
        .active
        .iter()
        .filter(|(s, e, _)| *s <= frame && frame < *e)
        .map(|(.., strength)| *strength)
        .fold(None, |m: Option<f32>, s| Some(m.map_or(s, |m| m.max(s))));
    let anim_locals = active.map(|_| local_rotations(skeleton, anim));

    let set = |name: &str| {
        binding
            .rig
            .muscle_set(name)
            .or_else(|| binding.rig.muscle_sets.first())
    };
    let gains_of = |set: Option<&MuscleSet>, muscle: &str, strength: f32| {
        let g = set.map_or(DEFAULT_GAINS, |s| s.gains(muscle));
        MuscleGains {
            twist: scale(g.twist, strength),
            swing: scale(g.swing, strength),
        }
    };
    let (base_set, base_strength) = match active {
        Some(s) => (set("Normal"), s),
        None => (set("Default"), 1.0),
    };
    let pose_set = set("DesiredPoseCalibration");
    // Active poses running now, oldest first. Each holds only the joints it drives, so
    // events covering different parts of the body (legs in one, arms in another) act
    // together. Where several hold the same joint, each moves the target and gains over
    // from what was there before it during its ease-in, so the latest to start wins.
    let poses: Vec<(&PoseTarget, f32)> = {
        let mut running: Vec<&PoseTarget> = plan
            .poses
            .iter()
            .filter(|p| p.start <= frame && frame < p.end)
            .collect();
        running.sort_by_key(|p| p.start);
        running
            .into_iter()
            .map(|p| {
                let ramp = if p.blend > 0 {
                    ((frame - p.start) as f32 / p.blend as f32).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                (p, ramp)
            })
            .collect()
    };
    let joints = &binding.rig.joints;
    let (mut targets, mut gains): (Vec<Quat>, Vec<MuscleGains>) = joints
        .iter()
        .map(|jd| {
            let mut target = anim_locals
                .as_ref()
                .map_or(binding.frozen[jd.joint], |l| l[jd.joint]);
            let mut gains = gains_of(base_set, &jd.muscle, base_strength);
            for &(p, ramp) in &poses {
                let Some(pose) = p.targets.get(jd.joint).copied().flatten() else {
                    continue;
                };
                let held = gains_of(pose_set, &jd.muscle, p.strength);
                target = target.slerp(pose, ramp);
                gains = MuscleGains {
                    twist: blend(gains.twist, held.twist, ramp),
                    swing: blend(gains.swing, held.swing, ramp),
                };
            }
            (target, gains)
        })
        .unzip();
    // The latest behaviour running now takes over from there.
    let span = plan
        .behaviours
        .iter()
        .rev()
        .find(|b| b.start <= frame && frame < b.end);
    if let (Some(span), Some(layout)) = (span, &binding.layout) {
        let mut drive = Drive {
            world,
            first_body: binding.first_body,
            first_joint: binding.first_joint,
            rig: &binding.rig,
            layout,
            targets: &mut targets,
            gains: &mut gains,
            elapsed: ((t - span.start as f32) / project.settings.fps.max(1.0)).max(0.0),
            settings: span.reach.as_ref(),
        };
        span.behaviour.drive(&mut drive);
    }
    for (j, (target, gains)) in targets.into_iter().zip(gains).enumerate() {
        let Some(joint) = world.joints.get_mut(binding.first_joint + j) else {
            break;
        };
        let off = plan.severs.iter().any(|s| s.joint == j && s.start <= frame);
        joint.muscle = (!off).then_some(Muscle {
            target,
            twist: gains.twist,
            swing: gains.swing,
        });
    }
}

const DEFAULT_GAINS: MuscleGains = MuscleGains {
    twist: Gains {
        stiffness: 50.0,
        damping: 100.0,
        force_limit: 50.0,
    },
    swing: Gains {
        stiffness: 50.0,
        damping: 100.0,
        force_limit: 50.0,
    },
};

/// `a` moved fraction `t` of the way to `b`.
fn blend(a: Gains, b: Gains, t: f32) -> Gains {
    let mix = |a: f32, b: f32| a + (b - a) * t;
    Gains {
        stiffness: mix(a.stiffness, b.stiffness),
        damping: mix(a.damping, b.damping),
        force_limit: mix(a.force_limit, b.force_limit),
    }
}

fn scale(g: Gains, s: f32) -> Gains {
    Gains {
        stiffness: g.stiffness * s,
        damping: g.damping * s,
        force_limit: g.force_limit * s,
    }
}

/// Joint local rotations of a character, taken from its bodies where it has them.
fn current_locals(
    world: &World,
    binding: &Binding,
    project: &Project,
    ci: usize,
    t: f32,
) -> Vec<Quat> {
    let skeleton = &project.characters[ci].skeleton;
    let joints = joints_from_bodies(world, binding, skeleton, &project.evaluate(t)[ci]);
    skeleton
        .joints
        .iter()
        .enumerate()
        .map(|(i, j)| match j.parent {
            Some(p) => (joints[p].0.inverse() * joints[i].0).normalize(),
            None => joints[i].0,
        })
        .collect()
}

/// Skeleton joint transforms from body poses; joints without a body follow the
/// animation relative to their parent.
fn joints_from_bodies(
    world: &World,
    binding: &Binding,
    skeleton: &crate::document::SkeletonRef,
    anim: &Pose,
) -> Vec<(Quat, Vec3)> {
    let anim_world = world_transforms(skeleton, anim);
    let mut out: Vec<(Quat, Vec3)> = Vec::with_capacity(skeleton.joints.len());
    for (i, joint) in skeleton.joints.iter().enumerate() {
        let transform = if let Some(bi) = binding.body_of_joint[i] {
            let body = &world.bodies[binding.first_body + bi];
            let def = &binding.rig.bodies[bi];
            let rot = (body.rotation * Quat::from_array(def.frame.rotation).normalize().inverse())
                .normalize();
            (rot, body.position - rot * Vec3::from(def.frame.position))
        } else if let Some(p) = joint.parent.filter(|&p| p < i) {
            // Keep the animated local transform under the simulated parent.
            let (pr, pp) = anim_world[p];
            let (r, q) = anim_world[i];
            let local_rot = pr.inverse() * r;
            let local_pos = pr.inverse() * (q - pp);
            let (sr, sp) = out[p];
            ((sr * local_rot).normalize(), sp + sr * local_pos)
        } else {
            anim_world[i]
        };
        out.push(transform);
    }
    out
}

fn record(project: &Project, running: &Running, t: f32) -> Frame {
    let frame = t.round() as i32;
    let poses = project.evaluate(t);
    running
        .bindings
        .iter()
        .enumerate()
        .map(|(ci, binding)| {
            let binding = binding.as_ref()?;
            let skeleton = &project.characters[ci].skeleton;
            let mut joints = joints_from_bodies(&running.world, binding, skeleton, &poses[ci]);
            // Transitions blend the simulated pose back onto the animation. The blend
            // includes the last frame (weight 1): the level only changes to animated
            // after that frame is recorded, so without it the last frame would show the
            // raw simulated pose between a blended and an animated one.
            let plan = &running.plans[ci];
            if let Some(&(start, end)) = plan
                .transitions
                .iter()
                .find(|(s, e)| *s <= frame && frame <= *e)
            {
                let w = ((frame - start) as f32 / (end - start).max(1) as f32).clamp(0.0, 1.0);
                // Blend orientations, then rebuild positions down the hierarchy so bones
                // keep their length mid-blend. Each bone's offset is read in its own
                // pose's parent frame (simulated in simulated, animated in animated):
                // read in the blended parent's frame, the simulated offset turns away
                // from the animated one and the lerp between them shortens the bone
                // (lesson 18).
                let anim = world_transforms(skeleton, &poses[ci]);
                let mut blended: Vec<(Quat, Vec3)> = Vec::with_capacity(joints.len());
                for (i, joint) in skeleton.joints.iter().enumerate() {
                    let rot = joints[i].0.slerp(anim[i].0, w).normalize();
                    let pos = match joint.parent.filter(|&p| p < i) {
                        Some(p) => {
                            let offset = joints[p].0.inverse() * (joints[i].1 - joints[p].1);
                            let anim_offset = anim[p].0.inverse() * (anim[i].1 - anim[p].1);
                            blended[p].1 + blended[p].0 * offset.lerp(anim_offset, w)
                        }
                        None => joints[i].1.lerp(anim[i].1, w),
                    };
                    blended.push((rot, pos));
                }
                joints = blended;
            }
            let bodies = (0..binding.rig.bodies.len())
                .map(|b| {
                    let body = &running.world.bodies[binding.first_body + b];
                    (body.rotation, body.position)
                })
                .collect();
            Some(CharacterFrame {
                joints,
                bodies,
                level: binding.level,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::document::sample::physics_demo_project;
    use std::time::Duration;

    fn simulate(project: &Project) -> Simulation {
        let mut sim = Simulation::default();
        sim.sync(1);
        while !sim.advance(project, Instant::now() + Duration::from_secs(60)) {}
        sim
    }

    /// World position of a body by name, at a frame.
    fn body_at(project: &Project, sim: &Simulation, frame: i32, name: &str) -> Vec3 {
        let rig = project.characters[0].physics.as_ref().unwrap();
        let b = rig.body_index(name).unwrap();
        sim.frame(frame).unwrap()[0].as_ref().unwrap().bodies[b].1
    }

    #[test]
    fn demo_character_is_shoved_over_and_stays_in_one_piece() {
        let project = physics_demo_project();
        assert!(Simulation::is_needed(&project));
        let started = Instant::now();
        let sim = simulate(&project);
        eprintln!(
            "simulated {} frames in {:?}",
            sim.progress().0,
            started.elapsed()
        );
        assert_eq!(sim.progress().0, sim.progress().1);

        // Before the switch the bodies follow the animation exactly.
        let skeleton = &project.characters[0].skeleton;
        let anim = world_transforms(skeleton, &project.evaluate(30.0)[0]);
        let frame30 = sim.frame(30).unwrap()[0].as_ref().unwrap();
        for (a, s) in anim.iter().zip(&frame30.joints) {
            assert!(
                a.1.distance(s.1) < 1e-3,
                "animated pose drifted before simulation"
            );
        }
        assert_eq!(frame30.level, SimLevel::Animated);

        let chest_before = body_at(&project, &sim, 45, "chest");
        let chest_after = body_at(&project, &sim, 150, "chest");
        let hips_after = body_at(&project, &sim, 150, "hips");
        eprintln!("chest {chest_before} -> {chest_after}, hips {hips_after}");
        assert!(
            chest_after.y < 0.6,
            "should have fallen, chest at {chest_after}"
        );
        assert!(
            chest_after.z < chest_before.z - 0.2,
            "should fall backwards"
        );

        for frame in 0..=150 {
            let f = sim.frame(frame).unwrap()[0].as_ref().unwrap();
            for (_, p) in &f.bodies {
                assert!(p.is_finite());
                assert!(
                    p.y > -0.05,
                    "body sank through the ground at frame {frame}: {p}"
                );
            }
            // Joints stay connected: each skeleton joint's position from its own body
            // stays close to where its parent body's frame puts it.
            for (i, j) in skeleton.joints.iter().enumerate() {
                if let Some(p) = j.parent {
                    let bone = Vec3::from(j.offset).length();
                    let gap = f.joints[i].1.distance(f.joints[p].1);
                    assert!(
                        (gap - bone).abs() < 0.03,
                        "joint {} stretched by {} at frame {frame}",
                        j.name,
                        gap - bone
                    );
                }
            }
        }
    }

    /// The demo character with a sever event on `body`'s joint at `frame`.
    fn severed(frame: i32, body: &str, kind: &str) -> Project {
        let mut project = physics_demo_project();
        let character = project.characters[0].id;
        let track = project.add_track(character, "Severs");
        let id = project.add_clip(
            crate::document::ClipProps {
                name: "Sever".into(),
                track,
                start: frame,
                duration: 1,
                blend_in: 0,
                blend_out: 0,
                weight: 1.0,
                enabled: true,
            },
            ClipKind::Event {
                event: "Sever".into(),
            },
            vec![],
        );
        let clip = project.clip_mut(id).unwrap();
        clip.source
            .insert("targets".into(), format!("character01.MetaBodies.{body}"));
        clip.source.insert("severType".into(), kind.into());
        project
    }

    /// How far the forearm's elbow is from the end of the upper arm's bone.
    fn elbow_gap(project: &Project, sim: &Simulation, frame: i32) -> f32 {
        let c = &project.characters[0];
        let rig = c.physics.as_ref().unwrap();
        let state = sim.frame(frame).unwrap()[0].as_ref().unwrap();
        let upper = rig.body_index("l_shoulder").unwrap();
        let half = Vec3::from(rig.bodies[upper].frame.position).length();
        let (rot, pos) = state.bodies[upper];
        let end = pos + rot * Vec3::Z * half;
        state.joints[c.skeleton.joint_index("l_elbow").unwrap()]
            .1
            .distance(end)
    }

    #[test]
    fn severed_forearm_comes_away_and_follows_its_own_body() {
        let project = severed(60, "l_elbow", "Cut");
        let sim = simulate(&project);
        let intact_project = physics_demo_project();
        let intact = simulate(&intact_project);
        for f in [0, 59] {
            assert!(elbow_gap(&project, &sim, f) < 0.03, "joined at frame {f}");
        }
        // After the cut the elbow joint follows the forearm, not the upper arm.
        let apart = (60..=150)
            .map(|f| elbow_gap(&project, &sim, f))
            .fold(0.0f32, f32::max);
        assert!(apart > 0.15, "forearm only came {apart} m away");
        let together = (0..=150)
            .map(|f| elbow_gap(&intact_project, &intact, f))
            .fold(0.0f32, f32::max);
        assert!(together < 0.03, "intact arm stretched {together} m");

        // Exports see it too: the severed elbow gets a translation curve (it moves
        // away from its offset), the intact one stays rotation only.
        use crate::io::motion::{joint_curves, sample};
        let skeleton = &project.characters[0].skeleton;
        let elbow = skeleton.joint_index("l_elbow").unwrap();
        let curves = |p: &Project, s: &Simulation| joint_curves(skeleton, &sample(p, 0, Some(s)));
        assert!(curves(&project, &sim)[elbow].translation.is_some());
        assert!(
            curves(&intact_project, &intact)[elbow]
                .translation
                .is_none()
        );
        for frame in 0..=150 {
            let f = sim.frame(frame).unwrap()[0].as_ref().unwrap();
            for (_, p) in &f.bodies {
                assert!(
                    p.is_finite() && p.length() < 10.0,
                    "exploded at {frame}: {p}"
                );
                assert!(p.y > -0.05, "sank through the ground at {frame}: {p}");
            }
        }
    }

    #[test]
    fn severed_simulation_is_deterministic() {
        let project = severed(60, "l_elbow", "Cut");
        let (a, b) = (simulate(&project), simulate(&project));
        let pos = |s: &Simulation, f| s.frame(f).unwrap()[0].as_ref().unwrap().bodies.clone();
        for f in [61, 100, 150] {
            assert_eq!(pos(&a, f), pos(&b, f));
        }
        // Frames before the event are the intact simulation's.
        let intact = simulate(&physics_demo_project());
        assert_eq!(pos(&a, 59), pos(&intact, 59));
    }

    #[test]
    fn muscle_off_goes_limp_but_stays_attached() {
        let off = severed(60, "l_elbow", "MuscleOff");
        let sim = simulate(&off);
        let intact_project = physics_demo_project();
        let intact = simulate(&intact_project);
        for f in 0..=150 {
            assert!(elbow_gap(&off, &sim, f) < 0.03, "came apart at frame {f}");
        }
        let hand = |p: &Project, s: &Simulation| body_at(p, s, 150, "l_wrist");
        let moved = hand(&off, &sim).distance(hand(&intact_project, &intact));
        assert!(moved > 0.02, "switching the muscle off changed nothing");
    }

    #[test]
    fn severing_a_body_with_no_joint_does_nothing() {
        let project = severed(60, "hips", "Cut");
        let (a, b) = (simulate(&project), simulate(&physics_demo_project()));
        let pos = |s: &Simulation| s.frame(150).unwrap()[0].as_ref().unwrap().bodies.clone();
        assert_eq!(pos(&a), pos(&b));
    }

    #[test]
    fn demo_simulation_is_deterministic() {
        let project = physics_demo_project();
        let (a, b) = (simulate(&project), simulate(&project));
        let pos = |s: &Simulation| s.frame(150).unwrap()[0].as_ref().unwrap().bodies.clone();
        assert_eq!(pos(&a), pos(&b));
    }
}
