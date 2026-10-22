//! Physics events the editor creates and edits: simulation switches, pushes,
//! transitions back to animation, active poses and constraints.
//!
//! Their settings live in [`Clip::source`](super::Clip::source) under the keys endorphin
//! scenes are imported with, which are also what the simulation (`crate::physics::sim`)
//! reads, so events made here and imported ones behave the same. The helpers below
//! give those strings types.

use std::collections::BTreeMap;

use glam::{Quat, Vec3};

use super::bake::bake_channels;
use super::model::{Channel, Character, ClipKind, Project};
use super::physics::SimLevel;

/// Event names, as `ClipKind::Event { event }` spells them.
pub const SIMULATION: &str = "Simulation";
pub const FORCE: &str = "Force";
pub const TRANSITION: &str = "Transition";
pub const ACTIVE_POSE: &str = "Active pose";
pub const CONSTRAINT: &str = "Constraint";
pub const BEHAVIOUR: &str = "Behaviour";

/// Setting keys, as imported from endorphin and read by the simulation.
pub mod keys {
    /// Simulation: the level switched to (endorphin's names, e.g. `FullPhysics`).
    pub const PHYSICS_LEVEL: &str = "physicsLevel";
    /// Simulation: how much of the animation's momentum carries into physics (0..1).
    pub const MOMENTUM_SCALE: &str = "momentumScale";
    /// Force: the velocity change in m/s, `"x, y, z"`.
    pub const VELOCITY: &str = "matrix.Vi";
    /// Force, constraint: bodies as `"character.MetaBodies.Body"`, comma separated.
    pub const TARGETS: &str = "targets";
    /// Transition: the level the character ends at.
    pub const REQUIRED_LEVEL: &str = "requiredPhysicsLevel";
    /// Active pose (and active animations): muscle strength multiplier.
    pub const STRENGTH: &str = "strength";
    /// Active pose: seconds to ease the pose in.
    pub const ENTRY_BLEND: &str = "entryBlendPeriod";
    /// Constraint: `Lock`, `LockPosition`, `LockOrientation` or `JoinBodies`.
    pub const LOCK_TYPE: &str = "lockType";
    /// Constraint: spring stiffness per axis, `"x, y, z"` (0 leaves an axis free).
    pub const POSITION_STRENGTH: &str = "positionStrength";
    pub const ANGULAR_STRENGTH: &str = "angularStrength";
    /// Behaviour: endorphin's behaviour name.
    pub const BEHAVIOUR_NAME: &str = "behaviourName";
}

/// The keys each event's typed controls cover; anything else is shown as-is.
pub fn edited_keys(event: &str) -> &'static [&'static str] {
    match event {
        SIMULATION => &[keys::PHYSICS_LEVEL, keys::MOMENTUM_SCALE],
        FORCE => &[keys::VELOCITY, keys::TARGETS],
        TRANSITION => &[keys::REQUIRED_LEVEL],
        ACTIVE_POSE => &[keys::STRENGTH, keys::ENTRY_BLEND],
        CONSTRAINT => &[keys::LOCK_TYPE, keys::TARGETS, keys::POSITION_STRENGTH, keys::ANGULAR_STRENGTH],
        BEHAVIOUR => &[keys::BEHAVIOUR_NAME],
        _ => &[],
    }
}

/// The kinds of clip the editor can add.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NewClip {
    Motion,
    Simulation,
    Force,
    Transition,
    ActivePose,
    Constraint,
}

impl NewClip {
    pub const ALL: [NewClip; 6] =
        [NewClip::Motion, NewClip::Simulation, NewClip::Force, NewClip::Transition, NewClip::ActivePose, NewClip::Constraint];

    pub fn label(self) -> &'static str {
        match self {
            NewClip::Motion => "Motion clip",
            NewClip::Simulation => "Simulation",
            NewClip::Force => "Force",
            NewClip::Transition => "Transition",
            NewClip::ActivePose => "Active pose",
            NewClip::Constraint => "Constraint",
        }
    }

    pub fn description(self) -> &'static str {
        match self {
            NewClip::Motion => "An empty clip to key joints in",
            NewClip::Simulation => "Switches the character's simulation level (e.g. goes limp into physics)",
            NewClip::Force => "A push: a sudden change of velocity on one body",
            NewClip::Transition => "Blends the simulation back to the animation",
            NewClip::ActivePose => "Muscles pull towards the pose at the playhead",
            NewClip::Constraint => "Locks bodies in place or joins them together",
        }
    }

    /// Whether the character needs a physics body for this to do anything.
    pub fn needs_physics(self) -> bool {
        self != NewClip::Motion
    }

    fn event(self) -> Option<&'static str> {
        Some(match self {
            NewClip::Motion => return None,
            NewClip::Simulation => SIMULATION,
            NewClip::Force => FORCE,
            NewClip::Transition => TRANSITION,
            NewClip::ActivePose => ACTIVE_POSE,
            NewClip::Constraint => CONSTRAINT,
        })
    }

    /// Name of the track the importer would put this kind on.
    pub fn track_name(self) -> &'static str {
        match self {
            NewClip::Motion => "Animation",
            NewClip::Simulation => "Simulations",
            NewClip::Force => "Forces",
            NewClip::Transition => "Transitions",
            NewClip::ActivePose => "Active poses",
            NewClip::Constraint => "Constraints",
        }
    }
}

/// Everything about a new clip except where it goes.
pub struct ClipTemplate {
    pub name: String,
    pub kind: ClipKind,
    pub duration: i32,
    pub channels: Vec<Channel>,
    pub source: BTreeMap<String, String>,
}

/// A new clip of `kind` for `character`, with settings that do something sensible right
/// away. `pose` (world joint transforms) is what an active pose holds.
pub fn template(kind: NewClip, project: &Project, character: &Character, pose: Option<&[(Quat, Vec3)]>) -> ClipTemplate {
    let fps = project.settings.fps.max(1.0);
    let seconds = |s: f32| ((s * fps).round() as i32).max(1);
    let mut source = BTreeMap::new();
    let (name, duration, channels) = match kind {
        NewClip::Motion => ("Motion", seconds(2.0), Vec::new()),
        NewClip::Simulation => {
            set_level(&mut source, keys::PHYSICS_LEVEL, SimLevel::Simulated);
            set_number(&mut source, keys::MOMENTUM_SCALE, 1.0);
            ("Simulate", 1, Vec::new())
        }
        NewClip::Force => {
            // Backwards and a little up, into the heaviest body (usually the torso).
            set_vector(&mut source, keys::VELOCITY, push(3.0, 180.0, 10.0));
            let heaviest = character.physics.as_ref().and_then(|rig| {
                rig.bodies.iter().max_by(|a, b| a.mass.total_cmp(&b.mass)).map(|b| b.name.clone())
            });
            let targets: Vec<BodyRef> = heaviest.into_iter().map(|body| BodyRef { character: character.name.clone(), body }).collect();
            set_targets(&mut source, &targets);
            ("Push", 1, Vec::new())
        }
        NewClip::Transition => {
            set_level(&mut source, keys::REQUIRED_LEVEL, SimLevel::Animated);
            ("Back to animation", seconds(0.5), Vec::new())
        }
        NewClip::ActivePose => {
            set_number(&mut source, keys::STRENGTH, 1.0);
            set_number(&mut source, keys::ENTRY_BLEND, 0.2);
            ("Hold pose", seconds(1.0), pose.map(|p| pose_channels(character, p)).unwrap_or_default())
        }
        NewClip::Constraint => {
            source.insert(keys::LOCK_TYPE.to_owned(), LockType::Lock.endorphin_name().to_owned());
            let root = character.physics.as_ref().and_then(|rig| rig.bodies.first()).map(|b| b.name.clone());
            let targets: Vec<BodyRef> = root.into_iter().map(|body| BodyRef { character: character.name.clone(), body }).collect();
            set_targets(&mut source, &targets);
            set_vector(&mut source, keys::POSITION_STRENGTH, Vec3::splat(DEFAULT_LOCK_STRENGTH));
            set_vector(&mut source, keys::ANGULAR_STRENGTH, Vec3::splat(DEFAULT_LOCK_STRENGTH));
            ("Lock", seconds(1.0), Vec::new())
        }
    };
    ClipTemplate {
        name: name.to_owned(),
        kind: kind.event().map_or(ClipKind::Motion, |e| ClipKind::Event { event: e.to_owned() }),
        duration,
        channels,
        source,
    }
}

/// Spring stiffness new constraints start with: stiff enough to hold a body firmly.
pub const DEFAULT_LOCK_STRENGTH: f32 = 10_000.0;

/// An active pose's held channels: every joint's rotation in `pose` (world transforms),
/// keyed once.
pub fn pose_channels(character: &Character, pose: &[(Quat, Vec3)]) -> Vec<Channel> {
    if pose.len() != character.skeleton.joints.len() {
        return Vec::new();
    }
    bake_channels(&character.skeleton, &[pose.to_vec()]).into_iter().filter(|c| c.property.is_rotation()).collect()
}

/// A velocity change from a speed (m/s), a heading (degrees about the up axis, 0 = +Z,
/// 90 = +X) and an elevation (degrees above horizontal).
pub fn push(speed: f32, heading: f32, elevation: f32) -> Vec3 {
    let (h, e) = (heading.to_radians(), elevation.to_radians());
    Vec3::new(h.sin() * e.cos(), e.sin(), h.cos() * e.cos()) * speed
}

/// The inverse of [`push`]: (speed, heading, elevation).
pub fn push_parts(velocity: Vec3) -> (f32, f32, f32) {
    let speed = velocity.length();
    if speed < 1e-6 {
        return (0.0, 0.0, 0.0);
    }
    let d = velocity / speed;
    let heading = match d.x.atan2(d.z).to_degrees() {
        // Straight back reads as 180°, not −180°.
        h if h < -179.999 => 180.0,
        h if h.abs() < 1e-4 => 0.0,
        h => h,
    };
    let elevation = d.y.clamp(-1.0, 1.0).asin().to_degrees();
    (speed, heading, elevation)
}

impl SimLevel {
    /// The name endorphin (and [`SimLevel::from_endorphin`]) uses.
    pub fn endorphin_name(self) -> &'static str {
        match self {
            SimLevel::Animated => "NoPhysics",
            SimLevel::CollisionOnly => "CollisionOnly",
            SimLevel::CollisionWithMomentum => "CollisionOnlyWithInertia",
            SimLevel::Simulated => "FullPhysics",
        }
    }
}

pub fn level(source: &BTreeMap<String, String>, key: &str) -> Option<SimLevel> {
    source.get(key).and_then(|v| SimLevel::from_endorphin(v))
}

pub fn set_level(source: &mut BTreeMap<String, String>, key: &str, level: SimLevel) {
    source.insert(key.to_owned(), level.endorphin_name().to_owned());
}

pub fn number(source: &BTreeMap<String, String>, key: &str) -> Option<f32> {
    source.get(key).and_then(|v| v.trim().parse().ok())
}

pub fn set_number(source: &mut BTreeMap<String, String>, key: &str, value: f32) {
    source.insert(key.to_owned(), format_number(value));
}

pub fn vector(source: &BTreeMap<String, String>, key: &str) -> Option<Vec3> {
    let v: Vec<f32> = source.get(key)?.split(',').filter_map(|p| p.trim().parse().ok()).collect();
    (v.len() == 3).then(|| Vec3::new(v[0], v[1], v[2]))
}

pub fn set_vector(source: &mut BTreeMap<String, String>, key: &str, v: Vec3) {
    source.insert(key.to_owned(), v.to_array().map(format_number).join(", "));
}

/// Like the importer: up to five decimals, no trailing zeros.
fn format_number(v: f32) -> String {
    let rounded = (v * 1e5).round() / 1e5;
    format!("{}", if rounded == 0.0 { 0.0 } else { rounded })
}

/// A rig body named the way events target it: `character.MetaBodies.body`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BodyRef {
    pub character: String,
    pub body: String,
}

impl BodyRef {
    pub fn label(&self) -> String {
        format!("{} · {}", self.character, self.body)
    }
}

pub fn targets(source: &BTreeMap<String, String>) -> Vec<BodyRef> {
    let Some(list) = source.get(keys::TARGETS) else { return Vec::new() };
    list.split(',')
        .map(str::trim)
        .filter(|t| !t.is_empty())
        .map(|t| match t.split_once(".MetaBodies.") {
            Some((character, body)) => BodyRef { character: character.to_owned(), body: body.to_owned() },
            None => BodyRef { character: String::new(), body: t.rsplit('.').next().unwrap_or(t).to_owned() },
        })
        .collect()
}

/// Writes the target list; an empty list removes the key (a force then pushes every body).
pub fn set_targets(source: &mut BTreeMap<String, String>, targets: &[BodyRef]) {
    if targets.is_empty() {
        source.remove(keys::TARGETS);
        return;
    }
    let list: Vec<String> = targets.iter().map(|t| format!("{}.MetaBodies.{}", t.character, t.body)).collect();
    source.insert(keys::TARGETS.to_owned(), list.join(", "));
}

/// How a constraint holds its bodies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LockType {
    /// Each body stays where it was, position and orientation.
    Lock,
    LockPosition,
    LockOrientation,
    /// Every body is held to the last one in the list.
    Join,
}

impl LockType {
    pub const ALL: [LockType; 4] = [LockType::Lock, LockType::LockPosition, LockType::LockOrientation, LockType::Join];

    pub fn label(self) -> &'static str {
        match self {
            LockType::Lock => "Lock in place",
            LockType::LockPosition => "Lock position only",
            LockType::LockOrientation => "Lock orientation only",
            LockType::Join => "Join bodies together",
        }
    }

    pub fn endorphin_name(self) -> &'static str {
        match self {
            LockType::Lock => "Lock",
            LockType::LockPosition => "LockPosition",
            LockType::LockOrientation => "LockOrientation",
            LockType::Join => "JoinBodies",
        }
    }

    pub fn from_endorphin(value: &str) -> Option<Self> {
        Some(match value {
            "Lock" => LockType::Lock,
            "LockPosition" => LockType::LockPosition,
            "LockOrientation" => LockType::LockOrientation,
            "JoinBodies" | "LockWithParent" => LockType::Join,
            _ => return None,
        })
    }
}
