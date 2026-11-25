//! Physical description of characters and the scene: bodies, joints, limits, muscles
//! and contact materials. Plain serializable data; the simulation lives in
//! `crate::physics`. Units are SI (metres, kilograms, seconds, radians, newtons).
//!
//! The model follows endorphin's so imported characters behave the same way:
//! one rigid body per bone segment, ball joints with a twist range and an elliptical
//! swing cone (each with a soft zone inside a hard stop), and per-joint muscles that
//! drive twist and swing separately with stiffness, damping and a force limit.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// A rotation `[x, y, z, w]` plus a position, relative to some parent frame.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub rotation: [f32; 4],
    pub position: [f32; 3],
}

impl Frame {
    pub const IDENTITY: Frame = Frame {
        rotation: [0.0, 0.0, 0.0, 1.0],
        position: [0.0, 0.0, 0.0],
    };
}

/// How much of the physics a character takes part in at a given moment.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimLevel {
    /// Keyframes drive it and nothing else interacts with it.
    #[default]
    Animated,
    /// Keyframes drive it; it pushes other objects out of the way.
    CollisionOnly,
    /// Keyframes drive it; it also hands its momentum to what it hits.
    CollisionWithMomentum,
    /// Fully simulated.
    Simulated,
}

impl SimLevel {
    pub const ALL: [SimLevel; 4] = [
        SimLevel::Animated,
        SimLevel::CollisionOnly,
        SimLevel::CollisionWithMomentum,
        SimLevel::Simulated,
    ];

    pub fn label(self) -> &'static str {
        match self {
            SimLevel::Animated => "Animated",
            SimLevel::CollisionOnly => "Collision only",
            SimLevel::CollisionWithMomentum => "Collision with momentum",
            SimLevel::Simulated => "Simulated",
        }
    }

    /// Parses endorphin's `physicsLevel` / `requiredPhysicsLevel` values.
    pub fn from_endorphin(value: &str) -> Option<Self> {
        Some(match value {
            "NoPhysics" | "No Simulation" => SimLevel::Animated,
            "CollisionOnly" | "Collision Only" => SimLevel::CollisionOnly,
            "CollisionOnlyWithInertia" | "Collision With Momentum" => {
                SimLevel::CollisionWithMomentum
            }
            "FullPhysics" | "Full Simulation" => SimLevel::Simulated,
            _ => return None,
        })
    }
}

/// The physical body of one character or prop.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhysicsRig {
    pub bodies: Vec<BodyDef>,
    pub joints: Vec<JointDef>,
    /// Named muscle presets ("Default", "Normal", ...). The first is used when nothing
    /// else asks for one.
    pub muscle_sets: Vec<MuscleSet>,
    /// Treat every body's inertia as a sphere's (endorphin's default).
    pub isotropic_inertia: bool,
    /// Cartesian springs between distant bodies (chest to hand, pelvis to heel) that
    /// active poses and animations pull with. They aren't bones: nothing in the
    /// skeleton or exported motion shows them. Projects saved before them have none.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub virtual_muscles: Vec<VirtualMuscleDef>,
}

impl PhysicsRig {
    pub fn body_index(&self, name: &str) -> Option<usize> {
        self.bodies.iter().position(|b| b.name == name)
    }

    pub fn muscle_set(&self, name: &str) -> Option<&MuscleSet> {
        self.muscle_sets.iter().find(|s| s.name == name)
    }

    pub fn total_mass(&self) -> f32 {
        self.bodies.iter().map(|b| b.mass).sum()
    }
}

/// One rigid body, attached to the skeleton joint whose segment it represents.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyDef {
    pub name: String,
    /// Skeleton joint this body hangs from; the body follows that joint's frame.
    pub joint: usize,
    /// Centre of mass and principal axes, relative to the joint's frame.
    pub frame: Frame,
    pub mass: f32,
    /// Principal moments of inertia about `frame`'s axes.
    pub inertia: [f32; 3],
    pub shapes: Vec<ShapeDef>,
}

/// A collision shape on a body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ShapeDef {
    pub geometry: Geometry,
    /// Relative to the body frame.
    pub frame: Frame,
    pub material: String,
    pub collides_with_parent: bool,
    pub collides_with_siblings: bool,
    pub collides_with_ancestors: bool,
}

/// Capsules and cylinders run along their local Z axis; `length` excludes the caps.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Geometry {
    Sphere { radius: f32 },
    Capsule { radius: f32, length: f32 },
    Cylinder { radius: f32, length: f32 },
    Box { size: [f32; 3] },
}

impl Geometry {
    pub fn volume(&self) -> f32 {
        use std::f32::consts::PI;
        match *self {
            Geometry::Sphere { radius } => 4.0 / 3.0 * PI * radius * radius * radius,
            Geometry::Capsule { radius, length } => {
                PI * radius * radius * (length + 4.0 / 3.0 * radius)
            }
            Geometry::Cylinder { radius, length } => PI * radius * radius * length,
            Geometry::Box { size } => size[0] * size[1] * size[2],
        }
    }

    /// Principal moments of inertia for a solid of the given mass.
    pub fn inertia(&self, mass: f32) -> [f32; 3] {
        match *self {
            Geometry::Sphere { radius } => [0.4 * mass * radius * radius; 3],
            Geometry::Capsule { radius, length } | Geometry::Cylinder { radius, length } => {
                let side = mass * (3.0 * radius * radius + length * length) / 12.0;
                [side, side, 0.5 * mass * radius * radius]
            }
            Geometry::Box { size: [x, y, z] } => {
                let k = mass / 12.0;
                [
                    k * (y * y + z * z),
                    k * (x * x + z * z),
                    k * (x * x + y * y),
                ]
            }
        }
    }
}

/// A joint between a body and its parent body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointDef {
    pub name: String,
    /// Skeleton joint index.
    pub joint: usize,
    /// Child body (the one this joint moves).
    pub body: usize,
    pub parent_body: usize,
    /// Limit frame attached to the parent side, relative to the joint frame at rest.
    pub primary: [f32; 4],
    /// Limit frame attached to the child side, relative to the joint frame at rest.
    /// Twist is rotation about its X axis; muscles drive along its axes too.
    pub secondary: [f32; 4],
    pub hard: Limits,
    pub soft: Limits,
    pub soft_stiffness: f32,
    pub soft_damping: f32,
    /// Muscle (effector) name used to look up gains in a [`MuscleSet`].
    pub muscle: String,
}

/// A virtual muscle: a force-limited spring that pulls a point on `body` towards where
/// the target pose puts it relative to `parent_body`, and turns `body` towards the
/// target pose's orientation relative to it (docs/physics-design.md, virtual muscles).
/// It adds no joint and no degree of freedom: the two bodies stay connected only
/// through the skeleton between them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VirtualMuscleDef {
    /// Its name in the character file ("LeftArmVirtualMuscle"); active poses and
    /// animations name it to drive it.
    pub name: String,
    /// Muscle (effector) name, for its entry in a [`MuscleSet`].
    pub muscle: String,
    /// The body it pulls from (the chest, the pelvis).
    pub parent_body: usize,
    /// The body it pulls (a hand, a heel).
    pub body: usize,
    /// The point on `body` it pulls, in that body's frame (metres): where the helper
    /// joint sits at rest.
    pub point: [f32; 3],
    /// Whether it drives position, and orientation. Its gains are its entries in the
    /// muscle presets, like a joint's: `MuscleSet::linear` for position, the swing
    /// gains for orientation.
    pub position: bool,
    pub orientation: bool,
}

/// Angular limits, all in radians, measured between the limit frames.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Limits {
    pub twist_min: f32,
    pub twist_max: f32,
    /// Half-angles of the elliptical swing cone about the frame's Y and Z axes.
    pub swing1: f32,
    pub swing2: f32,
}

impl Limits {
    /// endorphin stores each range as a full width centred on zero.
    pub fn from_sizes(twist: f32, swing1: f32, swing2: f32) -> Self {
        Self {
            twist_min: -twist * 0.5,
            twist_max: twist * 0.5,
            swing1: swing1 * 0.5,
            swing2: swing2 * 0.5,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MuscleSet {
    pub name: String,
    /// Gains per muscle name; muscles not listed use `fallback`.
    pub muscles: BTreeMap<String, MuscleGains>,
    pub fallback: MuscleGains,
    /// Position-drive gains (N/m, N·s/m, N) per muscle name, for the muscles that drive
    /// a position: virtual muscles. A virtual muscle not listed doesn't drive position
    /// with this preset. Projects saved before them have none.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub linear: BTreeMap<String, Gains>,
}

impl MuscleSet {
    pub fn gains(&self, muscle: &str) -> MuscleGains {
        self.muscles.get(muscle).copied().unwrap_or(self.fallback)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MuscleGains {
    pub twist: Gains,
    pub swing: Gains,
}

/// A force-limited PD drive: torque = stiffness·error − damping·velocity, capped.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Gains {
    pub stiffness: f32,
    pub damping: f32,
    pub force_limit: f32,
}

/// Scene-wide physics settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhysicsSettings {
    /// Physics ticks per second (endorphin: 120).
    pub rate: f32,
    pub gravity: [f32; 3],
    /// Infinite ground plane at this height, if any.
    pub ground: Option<Ground>,
    pub contact: ContactModel,
    /// What the solver's muscle spring cap counts as the mass a muscle moves.
    #[serde(default)]
    pub spring_cap: SpringCap,
    /// The share of their preset's stiffness and damping active poses drive a joint's
    /// swing axes with (its bend, as opposed to the twist about the bone). endorphin's
    /// own runs bend their swing joints as if driven at a sixteenth of the preset, so
    /// imported scenes use 1/16 (docs/physics-design.md, lesson 46). NeuroMotion's own
    /// projects use 1. The twist axis (all a knee or elbow has) keeps the whole preset
    /// here, although endorphin's knees and elbows measure at about a quarter of it
    /// (lesson 49): with a quarter Back-Flip-To-Combo's knees bend as endorphin's but
    /// its jump falls 0.17 m short, so not yet (lesson 50).
    #[serde(default = "full_share")]
    pub pose_swing_share: f32,
    /// Natural frequency (Hz) of contacts as springs on the bodies they touch, capped at
    /// what the solver's substep allows (a quarter of its rate: 120 Hz at 120 ticks a
    /// second). A contact is a spring for its body's own mass, so a 1 kg foot carrying
    /// half the body sinks 4.5 mm at 60 Hz and tilts 1.3° about its locked toe, where
    /// endorphin's sinks 0.5 mm; imported scenes use the cap (lesson 48). NeuroMotion's
    /// own projects use 60.
    #[serde(default = "contact_hertz")]
    pub contact_hertz: f32,
    /// With `SpringCap::Limbs`, solve a character's contacts that carry a load against
    /// the ground and fixed scenery rigidly, with the rest of its body, instead of as
    /// springs on the body that touches: a planted foot then doesn't sink at all where
    /// at 120 Hz it still sank a millimetre or two and tipped the body back. Imported
    /// scenes use it (lesson 50); NeuroMotion's own projects don't (no whole-body
    /// solve).
    #[serde(default)]
    pub rigid_contacts: bool,
    /// The force limit of an active pose's twist muscles (a knee's or elbow's only
    /// axis), as a share of the pose preset's, whatever the pose's strength. endorphin's
    /// knees push about 400 N·m at strengths 0.63, 0.8 and 2.5 alike (preset 1000 N·m);
    /// imported scenes use 0.4 (lesson 50). None: the preset's times the strength, as
    /// for every other muscle (NeuroMotion's own projects).
    #[serde(default)]
    pub pose_twist_limit: Option<f32>,
    pub interactions: BTreeMap<String, Interaction>,
    /// `(material, material) -> interaction name`, order-insensitive.
    pub pairs: Vec<(String, String, String)>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ground {
    pub height: f32,
    pub material: String,
}

/// How contact friction is limited.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContactModel {
    /// Each contact's friction force is capped at the interaction's `friction` value in
    /// newtons, whatever the load. NeuroMotion's default until Balance and Stagger were
    /// made to work with Coulomb friction; projects saved then keep it (they store
    /// their contact model).
    ForceCapped,
    /// Coulomb friction: the cap is `friction / 100` times the normal force, so
    /// endorphin's Default (100) is μ = 1 and its Grippy (300) μ = 3. That is what
    /// endorphin's friction values mean (lesson 21): imported scenes and new projects
    /// use it.
    #[default]
    Coulomb,
}

/// What a muscle spring's 30 Hz cap (docs/physics-design.md, lesson 4) measures the
/// moved mass by.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpringCap {
    /// The two bodies the joint connects. NeuroMotion's own default for now: Balance
    /// and Stagger were tuned on it, and lose their stepping with stronger leg muscles.
    #[default]
    Bodies,
    /// The whole limb hanging from the joint against the rest of the body, as endorphin's
    /// muscles behave: a clavicle holds the arm up, a knee in a crouch the body above
    /// it. Imported scenes use it (lesson 27). Each character is then solved as one
    /// system, so the cap can be as high as the two-body one (lesson 38).
    Limbs,
}

fn full_share() -> f32 {
    1.0
}

/// Contacts' natural frequency in NeuroMotion's own projects, and in files saved before
/// it was stored.
pub const CONTACT_HERTZ: f32 = 60.0;

fn contact_hertz() -> f32 {
    CONTACT_HERTZ
}

/// How two materials behave when they touch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Interaction {
    pub restitution: f32,
    pub friction: f32,
    /// Extra pull holding surfaces together, in newtons.
    pub adhesion: f32,
    /// Contact compliance; 0 is rigid.
    pub softness: f32,
}

impl Default for PhysicsSettings {
    fn default() -> Self {
        let interaction = |restitution, friction, adhesion, softness| Interaction {
            restitution,
            friction,
            adhesion,
            softness,
        };
        let mut interactions = BTreeMap::new();
        interactions.insert("Default".to_owned(), interaction(0.2, 100.0, 0.0, 0.0));
        interactions.insert("Grippy".to_owned(), interaction(0.2, 300.0, 0.0, 0.0));
        Self {
            rate: 120.0,
            gravity: [0.0, -9.81, 0.0],
            ground: Some(Ground {
                height: 0.0,
                material: "tarmac".to_owned(),
            }),
            contact: ContactModel::Coulomb,
            spring_cap: SpringCap::Bodies,
            pose_swing_share: 1.0,
            contact_hertz: CONTACT_HERTZ,
            rigid_contacts: false,
            pose_twist_limit: None,
            interactions,
            pairs: vec![
                ("rubber".into(), "tarmac".into(), "Grippy".into()),
                ("skin".into(), "tarmac".into(), "Default".into()),
            ],
        }
    }
}

impl PhysicsSettings {
    /// The interaction for two materials, falling back to "Default".
    pub fn interaction(&self, a: &str, b: &str) -> Interaction {
        let name = self
            .pairs
            .iter()
            .find(|(x, y, _)| (x == a && y == b) || (x == b && y == a))
            .map(|(_, _, i)| i.as_str())
            .unwrap_or("Default");
        self.interactions
            .get(name)
            .or_else(|| self.interactions.get("Default"))
            .copied()
            .unwrap_or(Interaction {
                restitution: 0.2,
                friction: 100.0,
                adhesion: 0.0,
                softness: 0.0,
            })
    }
}
