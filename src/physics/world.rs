//! Rigid bodies, joints, muscles and contacts, advanced by a substepped
//! sequential-impulse solver with soft constraints (see docs/physics-design.md).
//!
//! Each substep: gravity → contacts → warm start → solve with position bias →
//! integrate positions → relax without bias. Joints keep their impulses between
//! substeps; contacts are matched by (shape, shape, feature) and locks by their bodies
//! to warm start. Bodies that locks tie together, characters that something much
//! heavier presses on, and characters with a joint come apart are also solved directly,
//! as one system per group, each iteration (see `LockGroup`).

use std::collections::HashMap;

use glam::{Mat3, Quat, Vec3};

use super::collide::{self, Placed};
use crate::document::physics::{ContactModel, Gains, Geometry, Interaction, Limits, SpringCap};

pub const SUBSTEPS: usize = 4;
const ITERATIONS: usize = 4;
const RELAX_ITERATIONS: usize = 1;
const CONTACT_HERTZ: f32 = 60.0;
const CONTACT_DAMPING_RATIO: f32 = 10.0;
/// Fraction of a joint's positional error corrected per substep. Joints are rigid
/// (not mass-normalised soft constraints) so light limbs on long levers don't stretch.
const JOINT_CORRECTION: f32 = 0.2;
/// Fastest speed at which overlap is pushed apart, m/s.
const MAX_PUSH_SPEED: f32 = 3.0;
/// Contacts are created this far before surfaces touch, m.
const MARGIN: f32 = 0.02;
/// Limits engage this far before their stop, rad.
const ANGULAR_MARGIN: f32 = 0.05;
const RESTITUTION_THRESHOLD: f32 = 0.5;
/// Highest natural frequency a spring (muscle, soft limit) may have, given the
/// mass it actually moves. endorphin's presets are tuned for whole limbs; on very light
/// parts (fingers, hands) the same gains would ring far faster than the solver can
/// follow. Capping here leaves torso and limb muscles untouched (they sit at 3-15 Hz).
/// A body resting on the ground or scenery counts as immovable for the cap: a planted
/// foot's ankle muscle moves the whole body above it, not the foot.
const MAX_SPRING_HERTZ: f32 = 30.0;

/// With `SpringCap::Limbs`, a muscle may also be as stiff as this natural frequency on the
/// whole limb it moves against the rest of the body, if that allows more: a clavicle
/// holding an arm out, a knee in a crouch holding the body above it. Sequential impulses
/// solve a muscle with only its joint's two bodies, so this stays well under
/// `MAX_SPRING_HERTZ`: at 30 Hz Car Hit's pedestrian, curled up on the bonnet, opened a
/// knee by 6.2 cm; at 10 Hz 2.7 cm, at 5 Hz 2.2 cm (2.9 cm with the two-body cap alone).
/// Back-Flip-To-Combo crouches as endorphin's does at anything from 3 Hz up.
const MAX_LIMB_HERTZ: f32 = 5.0;

/// Stiffest spring (N·m/rad) with a natural frequency of `hertz` on an effective inverse
/// mass `inv_mass`.
fn spring_ceiling(hertz: f32, inv_mass: f32) -> f32 {
    let omega = 2.0 * std::f32::consts::PI * hertz;
    omega * omega / inv_mass.max(1e-12)
}

/// Scales stiffness and damping down together so the spring stays under
/// `max_stiffness`.
fn cap_spring(stiffness: f32, damping: f32, max_stiffness: f32) -> (f32, f32) {
    if stiffness <= max_stiffness {
        return (stiffness, damping);
    }
    let scale = max_stiffness / stiffness;
    (max_stiffness, damping * scale)
}
const MAX_ANGULAR_SPEED: f32 = 100.0;
/// Regularisation of a lock group's velocity solve and of its position correction, as a
/// share of each row's own effective mass (see `LockGroup::factorize`).
const GROUP_SOFTNESS: f64 = 1e-4;
const GROUP_PUSH_SOFTNESS: f64 = 1e-2;
/// Share of last substep's joint anchor impulses applied up front. With all of it, a
/// chain of light links holding a heavy load (a rope or strap with a weight on it,
/// hanging straight) breathes: every link stretches and squeezes together, more each
/// time, until it explodes. Four fifths keeps it still. Locks and muscles start from
/// their full impulse; locks need it to hold heavy loads up.
const WARM_START: f32 = 0.8;
/// A jointed group touched (closer than `CRUSH_SEPARATION`, m) by a group this many times
/// heavier is solved directly (see `World::crushed_groups`).
const CRUSH_RATIO: f32 = 5.0;
const CRUSH_SEPARATION: f32 = 0.01;
/// A jointed group with a joint open wider than this, m, is solved directly that substep
/// (lesson 17). Joints normally sit within a few millimetres. Not lower: at 1 cm it also
/// caught Heli Leap's jumper landing at 12.5 m/s, whose ankles the ground's contacts open
/// within a substep, and there solving the joints directly opened them a little more.
const STRETCH_LIMIT: f32 = 0.015;
const STATIC: usize = usize::MAX;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Motion {
    Dynamic,
    /// Moved by the caller (its velocity says how); collides. `momentum` scales the
    /// velocity it hands to what it hits: at 0 it pushes and carries things without
    /// setting them moving.
    Kinematic {
        momentum: f32,
    },
    /// Moved by the caller and ignored by collisions.
    Ghost,
}

#[derive(Clone, Debug)]
pub struct Body {
    pub position: Vec3,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub inv_mass: f32,
    /// Inverse principal moments in body axes.
    pub inv_inertia: Vec3,
    pub motion: Motion,
    /// Bodies in one group follow its self-collision flags; different groups always collide.
    pub group: usize,
    pub parent: Option<usize>,
}

impl Body {
    /// Moved by the solver. A dynamic body without mass is fixed in place: it takes part
    /// in collisions like scenery but nothing moves it (endorphin's massless bodies).
    fn is_dynamic(&self) -> bool {
        self.motion == Motion::Dynamic && self.inv_mass > 0.0
    }

    fn inv_inertia_world(&self) -> Mat3 {
        let r = Mat3::from_quat(self.rotation);
        r * Mat3::from_diagonal(self.inv_inertia) * r.transpose()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CollisionFlags {
    pub parent: bool,
    pub siblings: bool,
    pub ancestors: bool,
}

#[derive(Clone, Debug)]
pub struct Shape {
    /// `None` for static scenery (then the frame is in world space).
    pub body: Option<usize>,
    pub geometry: Geometry,
    pub rotation: Quat,
    pub position: Vec3,
    pub material: usize,
    pub flags: CollisionFlags,
}

/// What a joint's muscle is trying to do this tick.
#[derive(Clone, Copy, Debug)]
pub struct Muscle {
    /// Target rotation of the child joint frame relative to the parent joint frame.
    pub target: Quat,
    pub twist: Gains,
    pub swing: Gains,
}

#[derive(Clone, Debug)]
pub struct Joint {
    pub parent: usize,
    pub child: usize,
    /// Joint position in each body's frame.
    pub anchor_parent: Vec3,
    pub anchor_child: Vec3,
    /// Limit frames in each body's frame; twist is about the child frame's X.
    pub limit_parent: Quat,
    pub limit_child: Quat,
    /// Skeleton joint frames in each body's frame; their relative rotation is the
    /// joint's local rotation, which muscles drive towards `Muscle::target`.
    pub drive_parent: Quat,
    pub drive_child: Quat,
    pub hard: Limits,
    pub soft: Limits,
    pub soft_stiffness: f32,
    pub soft_damping: f32,
    pub muscle: Option<Muscle>,
    pub enabled: bool,
    warm_point: Vec3,
    warm_muscle: [f32; 3],
}

impl Joint {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        parent: usize,
        child: usize,
        anchor_parent: Vec3,
        anchor_child: Vec3,
        limit_parent: Quat,
        limit_child: Quat,
        drive_parent: Quat,
        drive_child: Quat,
        hard: Limits,
        soft: Limits,
        soft_stiffness: f32,
        soft_damping: f32,
    ) -> Self {
        Self {
            parent,
            child,
            anchor_parent,
            anchor_child,
            limit_parent,
            limit_child,
            drive_parent,
            drive_child,
            hard,
            soft,
            soft_stiffness,
            soft_damping,
            muscle: None,
            enabled: true,
            warm_point: Vec3::ZERO,
            warm_muscle: [0.0; 3],
        }
    }

    /// Moves a muscle target (a joint-local rotation) inside the soft limit range, so
    /// muscles never pull the joint into its stops. Targets from motion capture often
    /// sit slightly outside a rig's limits.
    pub fn clamp_to_limits(&self, target: Quat) -> Quat {
        // Limit-frame rotation for a given joint-local rotation R is Lp * R * Lc.
        let lp = self.limit_parent.inverse() * self.drive_parent;
        let lc = self.drive_child.inverse() * self.limit_child;
        let (twist, swing) = swing_twist(lp * target * lc);
        let l = self.soft;
        let clamped_twist = twist.clamp(l.twist_min.min(l.twist_max), l.twist_max.max(l.twist_min));
        let s = glam::Vec2::new(swing.y, swing.z);
        let radius = s.length();
        let mut swing_vec = swing;
        if radius > 1e-6 {
            let dir = s / radius;
            let (y, z) = (l.swing1.max(1e-4), l.swing2.max(1e-4));
            let boundary = 1.0 / ((dir.x / y).powi(2) + (dir.y / z).powi(2)).sqrt();
            if radius > boundary {
                swing_vec = Vec3::new(0.0, dir.x * boundary, dir.y * boundary);
            }
        }
        if clamped_twist == twist && swing_vec == swing {
            return target;
        }
        let limited = Quat::from_scaled_axis(swing_vec) * Quat::from_rotation_x(clamped_twist);
        (lp.inverse() * limited * lc.inverse()).normalize()
    }

    /// Current rotation of the child joint frame relative to the parent joint frame.
    pub fn local_rotation(&self, bodies: &[Body]) -> Quat {
        let qa = bodies[self.parent].rotation * self.drive_parent;
        let qb = bodies[self.child].rotation * self.drive_child;
        (qa.inverse() * qb).normalize()
    }
}

/// Holds a body at a target pose relative to the world or another body (endorphin's
/// lock and join constraints). Like Karma's relative-position-relative-orientation
/// joint, it is rigid, but the force it may use on each axis is capped by that axis's
/// strength (N for position, N·m for orientation); 0 leaves the axis free. The axes are
/// the world's for a lock to the world and the anchor's for a join.
#[derive(Clone, Copy, Debug)]
pub struct Lock {
    pub anchor: Option<usize>,
    pub body: usize,
    /// Target rotation and centre-of-mass position of `body`, in `anchor`'s frame
    /// (world space when there's no anchor).
    pub target: (Quat, Vec3),
    pub linear: Vec3,
    pub angular: Vec3,
}

/// A force-limited Cartesian spring between two bodies of a character (endorphin's
/// virtual muscles, chest to hand and pelvis to heel): pulls `point` on `body` towards
/// `target_position` in `anchor`'s frame, and turns `body` towards `target_rotation`
/// relative to `anchor`. Unlike a joint it removes no freedom: it is a spring on the
/// two bodies, solved implicitly like a muscle (stable at any stiffness), and its force
/// (N) and torque (N·m) are capped by its gains' force limits, on the full 3D vector.
#[derive(Clone, Copy, Debug)]
pub struct VirtualMuscle {
    pub anchor: usize,
    pub body: usize,
    /// The pulled point, in `body`'s frame.
    pub point: Vec3,
    /// Where `point` is wanted, in `anchor`'s frame.
    pub target_position: Vec3,
    /// `body`'s wanted rotation relative to `anchor`'s.
    pub target_rotation: Quat,
    pub linear: Option<Gains>,
    pub angular: Option<Gains>,
}

pub struct World {
    pub bodies: Vec<Body>,
    pub shapes: Vec<Shape>,
    pub joints: Vec<Joint>,
    pub locks: Vec<Lock>,
    /// Set by the caller every tick; nothing is remembered between ticks.
    pub virtual_muscles: Vec<VirtualMuscle>,
    pub gravity: Vec3,
    /// Ground plane height and material.
    pub ground: Option<(f32, usize)>,
    pub contact_model: ContactModel,
    pub spring_cap: SpringCap,
    material_count: usize,
    /// `interactions[a * material_count + b]`.
    interactions: Vec<Interaction>,
    contact_cache: HashMap<(u32, u32, u8), [f32; 3]>,
    /// Keyed by (anchor, body, how many earlier locks hold the same pair).
    lock_cache: HashMap<(usize, usize, usize), [f32; 6]>,
    /// Whether each body was dynamic last substep, to spot switches.
    was_dynamic: Vec<bool>,
}

// ---------------------------------------------------------------------------------
// Solver internals

#[derive(Clone, Copy)]
struct SolverBody {
    v: Vec3,
    w: Vec3,
    inv_mass: f32,
    inv_i: Mat3,
    /// Share of a moved body's velocity it hands to what it touches (1 when dynamic).
    carry: f32,
}

#[derive(Clone, Copy)]
struct Softness {
    bias_rate: f32,
    mass_scale: f32,
    impulse_scale: f32,
}

/// Soft constraint coefficients for a given stiffness frequency and damping ratio.
fn soft(hertz: f32, damping_ratio: f32, h: f32) -> Softness {
    let omega = 2.0 * std::f32::consts::PI * hertz;
    let a1 = 2.0 * damping_ratio + h * omega;
    let a2 = h * omega * a1;
    let a3 = 1.0 / (1.0 + a2);
    Softness {
        bias_rate: omega / a1,
        mass_scale: a2 * a3,
        impulse_scale: a3,
    }
}

/// One angular row: a one-sided limit (rigid) or a spring (soft limit zone / muscle).
#[derive(Clone, Copy)]
struct AngularRow {
    axis: Vec3,
    /// Inverse of the row's effective inverse mass (plus softness for springs).
    mass: f32,
    /// Rigid rows: constraint value (≥ 0 wanted). Springs: velocity bias.
    value: f32,
    spring: bool,
    gamma: f32,
    lower: f32,
    upper: f32,
    impulse: f32,
}

struct JointRows {
    joint: usize,
    a: usize,
    b: usize,
    ra: Vec3,
    rb: Vec3,
    point_error: Vec3,
    point_mass: Mat3,
    point_impulse: Vec3,
    /// Coupling between the anchor and angular impulses, and the angular effective
    /// inverse mass with the anchor held (Schur complement).
    k_pa: Mat3,
    k_ap: Mat3,
    angular_schur: Mat3,
    /// The same with grounded bodies held still, for capping muscle springs.
    cap_schur: Mat3,
    /// With `SpringCap::Limbs`: the same for the whole limb against the rest of the body.
    limb_schur: Option<Mat3>,
    limits: Vec<AngularRow>,
    muscle: Option<MuscleBlock>,
}

/// A joint's muscle solved together with its anchor as one 6x6 system, so the muscle
/// "feels" the whole lever it moves rather than only its own body's inertia.
struct MuscleBlock {
    /// World-space drive axes as columns: twist, swing 1, swing 2.
    frame: Mat3,
    /// Spring softness (world space) and velocity bias.
    gamma: Mat3,
    bias: Vec3,
    /// Impulse bounds per drive axis (force limit × h).
    bound: Vec3,
    /// Accumulated angular impulse, world space.
    impulse: Vec3,
    schur_inv: Mat3,
}

/// A lock solved as one block: three position rows at the held point and three
/// orientation rows, along the lock's axes, each capped at its own strength.
struct LockBlock {
    a: usize,
    b: usize,
    /// Held point on A relative to A's centre: where B's centre is now.
    ra: Vec3,
    /// Lock axes (columns), world space.
    frame: Mat3,
    /// Position and orientation error along the axes.
    error: [f32; 6],
    /// Inverse effective mass of the block; free axes are decoupled.
    inv_k: [[f32; 6]; 6],
    /// Impulse caps (strength × h); 0 on free axes.
    bound: [f32; 6],
    impulse: [f32; 6],
    /// Axes that ended position correction at their strength. A lock that can't keep up
    /// pulls with all of it, so relaxing leaves them there (lesson 20).
    pulling: [bool; 6],
    /// √(2 × strength / effective mass) per held axis: an axis `e` off target can close
    /// at up to this × √e and still stop on target using its strength (lesson 20).
    brake: [f32; 6],
    key: (usize, usize, usize),
}

/// A virtual muscle's position or orientation spring this substep: three rows (world
/// axes) solved as one block, the impulse capped in length.
struct DriveBlock {
    a: usize,
    b: usize,
    /// Levers of the pulled point from A's and B's centres (zero for orientation).
    ra: Vec3,
    rb: Vec3,
    linear: bool,
    /// Spring softness and velocity bias.
    gamma: f32,
    bias: Vec3,
    /// (K + γ)⁻¹, K the block's effective inverse mass.
    mass: Mat3,
    /// Force limit × h.
    bound: f32,
    impulse: Vec3,
}

/// One equality row of a lock group: a joint's anchor along a world axis, or one held
/// axis of a lock.
struct GroupRow {
    a: usize,
    b: usize,
    /// Jacobian: linear and angular parts on A, then on B.
    jacobian: [Vec3; 4],
    /// Velocity bias while correcting positions.
    bias: f32,
    /// Upper cap on the accumulated impulse (locks); infinite for anchors and contacts.
    bound: f32,
    /// Lower cap: minus the strength for locks, minus the adhesion for contacts (they
    /// can only push), infinite for anchors.
    floor: f32,
    source: RowSource,
}

#[derive(Clone, Copy)]
enum RowSource {
    /// Joint rows index and world axis.
    Anchor(usize, usize),
    /// Lock block index and lock axis.
    Lock(usize, usize),
    /// A contact's normal.
    Contact(usize),
}

/// Bodies held together by locks, with every joint between them, solved as one linear
/// system. Sequential impulses converge slowly around loops and long levers: a gun
/// joined to both hands closes a loop through the arms, and a trapeze bar joined to a
/// rope hinged 6 m away swings on a lever hundreds of times its own inertia. Solving
/// those rows together, directly, after the per-joint and per-lock passes each
/// iteration, handles both. A character something much heavier presses on (a car
/// catching an arm against the ribs), or one with a joint come apart (an arm whipping
/// round against its own body), is solved the same way, joints only.
struct LockGroup {
    rows: Vec<GroupRow>,
    /// The rows' effective mass matrix J M⁻¹ Jᵀ (lower triangle, row-major).
    k: Vec<f64>,
    /// Columns left of the diagonal where each row of `k` isn't zero (rows sharing a
    /// moving body), ascending.
    k_pattern: Vec<Vec<usize>>,
    /// Where each row of the Cholesky factor isn't zero left of the diagonal. Joints
    /// come leaf first, so a skeleton factors without fill-in and only locks add any.
    pattern: Vec<Vec<usize>>,
    /// Cholesky factors (lower triangle, row-major) over the rows still solved, of `k`
    /// regularised for the velocity solve and for position correction.
    velocity_factor: Vec<f64>,
    push_factor: Vec<f64>,
    /// Lock axes left out of the direct solve because they reached their strength:
    /// they stay at it this substep, and the per-lock pass still sees them.
    out: Vec<bool>,
    /// Solved after the contacts, not before them (lesson 25).
    late: bool,
}

struct ContactRow {
    a: usize,
    b: usize,
    ra: Vec3,
    rb: Vec3,
    normal: Vec3,
    tangents: [Vec3; 2],
    separation: f32,
    normal_mass: f32,
    tangent_mass: [f32; 2],
    /// Normal velocity before solving, for restitution.
    approach: f32,
    interaction: Interaction,
    impulse: [f32; 3],
    key: (u32, u32, u8),
}

fn skew(v: Vec3) -> Mat3 {
    Mat3::from_cols(
        Vec3::new(0.0, v.z, -v.y),
        Vec3::new(-v.z, 0.0, v.x),
        Vec3::new(v.y, -v.x, 0.0),
    )
}

fn get(sb: &[SolverBody], i: usize) -> SolverBody {
    if i == STATIC {
        SolverBody {
            v: Vec3::ZERO,
            w: Vec3::ZERO,
            inv_mass: 0.0,
            inv_i: Mat3::ZERO,
            carry: 0.0,
        }
    } else {
        sb[i]
    }
}

fn relative_velocity(sb: &[SolverBody], a: usize, b: usize, ra: Vec3, rb: Vec3) -> Vec3 {
    let (ba, bb) = (get(sb, a), get(sb, b));
    (bb.v + bb.w.cross(rb)) - (ba.v + ba.w.cross(ra))
}

/// Relative velocity at a contact, with moved bodies counting only the velocity they
/// hand on (`carry`). Used once positions are integrated, so that collision-only bodies
/// push things along without setting them moving.
fn handed_velocity(sb: &[SolverBody], a: usize, b: usize, ra: Vec3, rb: Vec3) -> Vec3 {
    let (ba, bb) = (get(sb, a), get(sb, b));
    (bb.v + bb.w.cross(rb)) * bb.carry - (ba.v + ba.w.cross(ra)) * ba.carry
}

fn apply_impulse(sb: &mut [SolverBody], a: usize, b: usize, ra: Vec3, rb: Vec3, p: Vec3) {
    if a != STATIC {
        let body = &mut sb[a];
        body.v -= p * body.inv_mass;
        body.w -= body.inv_i * ra.cross(p);
    }
    if b != STATIC {
        let body = &mut sb[b];
        body.v += p * body.inv_mass;
        body.w += body.inv_i * rb.cross(p);
    }
}

fn apply_angular(sb: &mut [SolverBody], a: usize, b: usize, l: Vec3) {
    if a != STATIC {
        sb[a].w -= sb[a].inv_i * l;
    }
    if b != STATIC {
        sb[b].w += sb[b].inv_i * l;
    }
}

fn perpendicular(n: Vec3) -> Vec3 {
    let t = if n.x.abs() > 0.57 {
        Vec3::new(n.y, -n.x, 0.0)
    } else {
        Vec3::new(0.0, n.z, -n.y)
    };
    t.normalize()
}

/// Splits `q` into swing · twist with twist about X; returns (twist angle, swing vector).
pub fn swing_twist(q: Quat) -> (f32, Vec3) {
    let q = if q.w < 0.0 { -q } else { q };
    let twist_len = (q.x * q.x + q.w * q.w).sqrt();
    if twist_len < 1e-9 {
        return (0.0, q.to_scaled_axis());
    }
    let twist = Quat::from_xyzw(q.x / twist_len, 0.0, 0.0, q.w / twist_len);
    let angle = 2.0 * q.x.atan2(q.w);
    let angle =
        (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
    let swing = (q * twist.inverse()).normalize();
    (angle, swing.to_scaled_axis())
}

impl World {
    pub fn new(gravity: Vec3, contact_model: ContactModel) -> Self {
        Self {
            bodies: Vec::new(),
            shapes: Vec::new(),
            joints: Vec::new(),
            locks: Vec::new(),
            virtual_muscles: Vec::new(),
            gravity,
            ground: None,
            contact_model,
            spring_cap: SpringCap::default(),
            material_count: 0,
            interactions: Vec::new(),
            contact_cache: HashMap::new(),
            lock_cache: HashMap::new(),
            was_dynamic: Vec::new(),
        }
    }

    /// Sets the material interaction table; `lookup(a, b)` gives the interaction for
    /// material indices `a` and `b`.
    pub fn set_materials(&mut self, count: usize, lookup: impl Fn(usize, usize) -> Interaction) {
        self.material_count = count.max(1);
        self.interactions = (0..self.material_count * self.material_count)
            .map(|i| lookup(i / self.material_count, i % self.material_count))
            .collect();
    }

    fn interaction(&self, a: usize, b: usize) -> Interaction {
        let n = self.material_count;
        self.interactions
            .get(a.min(n - 1) * n + b.min(n - 1))
            .copied()
            .unwrap_or(Interaction {
                restitution: 0.2,
                friction: 100.0,
                adhesion: 0.0,
                softness: 0.0,
            })
    }

    /// Whether any of `bodies` pushed on something outside them (another body, scenery
    /// or the ground) in the last substep. Only answers yes or no, so the cache's order
    /// doesn't matter.
    pub fn pressing(&self, bodies: std::ops::Range<usize>) -> bool {
        let inside = |s: u32| {
            self.shapes
                .get(s as usize)
                .and_then(|s| s.body)
                .is_some_and(|b| bodies.contains(&b))
        };
        self.contact_cache
            .iter()
            .any(|(&(a, b, _), impulse)| impulse[0] > 0.0 && inside(a) != inside(b))
    }

    /// Advances the world by `dt` seconds.
    pub fn step(&mut self, dt: f32) {
        let h = dt / SUBSTEPS as f32;
        for _ in 0..SUBSTEPS {
            self.substep(h);
        }
    }

    fn substep(&mut self, h: f32) {
        self.forget_switched_bodies();
        // A dynamic body without mass is fixed in place, whatever velocity it was handed
        // (a force event, or its animation's before it went simulated). Kept, it would
        // drift forever, carrying whatever stands on it.
        for body in self
            .bodies
            .iter_mut()
            .filter(|b| b.motion == Motion::Dynamic && !b.is_dynamic())
        {
            body.linear_velocity = Vec3::ZERO;
            body.angular_velocity = Vec3::ZERO;
        }
        for body in self.bodies.iter_mut().filter(|b| b.is_dynamic()) {
            body.linear_velocity += self.gravity * h;
        }
        self.apply_lock_drives(h);
        let mut sb: Vec<SolverBody> = self
            .bodies
            .iter()
            .map(|b| {
                // Moved bodies keep their real velocity: joints and locks follow it, and
                // contacts use it while correcting positions so moving scenery can't pass
                // through things. `carry` decides how much of it they hand on.
                let (inv_mass, inv_i, carry) = match b.motion {
                    Motion::Dynamic => (b.inv_mass, b.inv_inertia_world(), 1.0),
                    Motion::Kinematic { momentum } => (0.0, Mat3::ZERO, momentum),
                    Motion::Ghost => (0.0, Mat3::ZERO, 0.0),
                };
                SolverBody {
                    v: b.linear_velocity,
                    w: b.angular_velocity,
                    inv_mass,
                    inv_i,
                    carry,
                }
            })
            .collect();

        let mut contacts = self.find_contacts(&sb, h);
        let mut grounded = vec![false; self.bodies.len()];
        for c in contacts.iter().filter(|c| c.separation < 0.01) {
            let held = |i: usize| i == STATIC || !self.bodies[i].is_dynamic();
            if held(c.a) && c.b != STATIC {
                grounded[c.b] = true;
            }
            if held(c.b) && c.a != STATIC {
                grounded[c.a] = true;
            }
        }
        let mut joints = self.prepare_joints(&sb, h, &grounded);
        let mut locks = self.prepare_locks(&sb, h);
        let mut drives = self.prepare_virtual_muscles(&sb, h);

        // Warm start.
        for c in &contacts {
            let p = c.normal * c.impulse[0]
                + c.tangents[0] * c.impulse[1]
                + c.tangents[1] * c.impulse[2];
            apply_impulse(&mut sb, c.a, c.b, c.ra, c.rb, p);
        }
        for j in &joints {
            apply_impulse(&mut sb, j.a, j.b, j.ra, j.rb, j.point_impulse);
            if let Some(m) = &j.muscle {
                apply_angular(&mut sb, j.a, j.b, m.impulse);
            }
        }
        for l in &locks {
            l.apply(&mut sb, l.impulse);
        }

        let contact_soft = soft(CONTACT_HERTZ.min(0.25 / h), CONTACT_DAMPING_RATIO, h);
        let joint_soft = Softness {
            bias_rate: JOINT_CORRECTION / h,
            mass_scale: 1.0,
            impulse_scale: 0.0,
        };
        let mut groups = self.prepare_groups(
            &sb,
            &joints,
            &locks,
            &contacts,
            joint_soft.bias_rate,
            contact_soft.bias_rate,
            h,
        );
        // Contacts a group holds in its direct solve (lesson 25): only their
        // friction is left to the contact pass. One that stops pushing goes back to it.
        let mut held = vec![false; contacts.len()];
        for group in &groups {
            for row in &group.rows {
                if let RowSource::Contact(c) = row.source {
                    held[c] = true;
                }
            }
        }
        // Direct solves of the groups, then the contacts, then the direct solves of the
        // groups whose locks close a loop: those have the last word (lesson 25).
        for _ in 0..ITERATIONS {
            solve_drives(&mut sb, &mut drives);
            self.solve_joints(&mut sb, &mut joints, joint_soft, h, true);
            solve_locks(&mut sb, &mut locks, joint_soft.bias_rate);
            for late in [false, true] {
                if late {
                    self.solve_contacts(&mut sb, &mut contacts, &held, contact_soft, h, true);
                }
                solve_groups(
                    &mut sb,
                    &mut groups,
                    &mut joints,
                    &mut locks,
                    &mut contacts,
                    &mut held,
                    late,
                    true,
                );
            }
        }
        // A lock axis still at its strength after position correction can't keep up: it
        // pulls with all of it, and relaxing leaves it there (lesson 20).
        for l in &mut locks {
            l.pulling =
                std::array::from_fn(|r| l.bound[r] > 0.0 && l.impulse[r].abs() >= l.bound[r]);
        }

        // Integrate positions with the biased velocities.
        self.write_velocities(&sb);
        for body in self.bodies.iter_mut() {
            if body.is_dynamic() {
                let speed = body.angular_velocity.length();
                if speed > MAX_ANGULAR_SPEED {
                    body.angular_velocity *= MAX_ANGULAR_SPEED / speed;
                }
            }
            body.position += body.linear_velocity * h;
            let w = body.angular_velocity;
            let dq = Quat::from_xyzw(w.x, w.y, w.z, 0.0) * body.rotation;
            body.rotation = Quat::from_xyzw(
                body.rotation.x + 0.5 * h * dq.x,
                body.rotation.y + 0.5 * h * dq.y,
                body.rotation.z + 0.5 * h * dq.z,
                body.rotation.w + 0.5 * h * dq.w,
            )
            .normalize();
        }

        // Relax: remove the velocity that position correction added.
        for _ in 0..RELAX_ITERATIONS {
            solve_drives(&mut sb, &mut drives);
            self.solve_joints(&mut sb, &mut joints, joint_soft, h, false);
            solve_locks(&mut sb, &mut locks, 0.0);
            for late in [false, true] {
                if late {
                    self.solve_contacts(&mut sb, &mut contacts, &held, contact_soft, h, false);
                }
                solve_groups(
                    &mut sb,
                    &mut groups,
                    &mut joints,
                    &mut locks,
                    &mut contacts,
                    &mut held,
                    late,
                    false,
                );
            }
        }
        apply_restitution(&mut sb, &mut contacts);
        self.write_velocities(&sb);

        // Keep impulses for warm starting the next substep.
        self.contact_cache.clear();
        for c in &contacts {
            self.contact_cache.insert(c.key, c.impulse);
        }
        for j in &joints {
            let joint = &mut self.joints[j.joint];
            joint.warm_point = j.point_impulse;
            joint.warm_muscle = j
                .muscle
                .as_ref()
                .map_or([0.0; 3], |m| (m.frame.transpose() * m.impulse).to_array());
        }
        self.lock_cache.clear();
        for l in &locks {
            self.lock_cache.insert(l.key, l.impulse);
        }
    }

    /// A lock axis with a negative strength drives instead of holding: a steady force (N)
    /// or torque (N·m) of that value along the lock's axis, on the locked body, and the
    /// opposite on its anchor. endorphin's files use it to spin a character into a flip
    /// or push a hand down (its help doesn't say so; the scenes do).
    fn apply_lock_drives(&mut self, h: f32) {
        for lock in &self.locks {
            if lock.linear.min_element() >= 0.0 && lock.angular.min_element() >= 0.0 {
                continue;
            }
            let frame = lock
                .anchor
                .map_or(Mat3::IDENTITY, |a| Mat3::from_quat(self.bodies[a].rotation));
            let force = frame * lock.linear.min(Vec3::ZERO) * h;
            let torque = frame * lock.angular.min(Vec3::ZERO) * h;
            for (body, sign) in [(Some(lock.body), 1.0), (lock.anchor, -1.0)] {
                let Some(body) = body.map(|b| &mut self.bodies[b]).filter(|b| b.is_dynamic())
                else {
                    continue;
                };
                body.linear_velocity += force * (sign * body.inv_mass);
                body.angular_velocity += body.inv_inertia_world() * (torque * sign);
            }
        }
    }

    /// Drops warm-start impulses of bodies that just became dynamic or stopped being so.
    /// What a moved body pushed with (against something it was crushing, say) would hit a
    /// light body like a hammer once it is simulated.
    fn forget_switched_bodies(&mut self) {
        let dynamic: Vec<bool> = self.bodies.iter().map(Body::is_dynamic).collect();
        let switched: Vec<bool> = (0..dynamic.len())
            .map(|i| self.was_dynamic.get(i).is_some_and(|&d| d != dynamic[i]))
            .collect();
        if switched.contains(&true) {
            let shapes = &self.shapes;
            let switched_shape = |s: u32| {
                shapes
                    .get(s as usize)
                    .and_then(|s| s.body)
                    .is_some_and(|b| switched[b])
            };
            self.contact_cache
                .retain(|&(a, b, _), _| !switched_shape(a) && !switched_shape(b));
            self.lock_cache
                .retain(|&(a, b, _), _| !(a != STATIC && switched[a]) && !switched[b]);
            for joint in self
                .joints
                .iter_mut()
                .filter(|j| switched[j.parent] || switched[j.child])
            {
                joint.warm_point = Vec3::ZERO;
                joint.warm_muscle = [0.0; 3];
            }
        }
        self.was_dynamic = dynamic;
    }

    fn write_velocities(&mut self, sb: &[SolverBody]) {
        for (body, s) in self.bodies.iter_mut().zip(sb) {
            if body.is_dynamic() {
                body.linear_velocity = s.v;
                body.angular_velocity = s.w;
            }
        }
    }

    // -----------------------------------------------------------------------------
    // Joints

    fn prepare_joints(&self, sb: &[SolverBody], h: f32, grounded: &[bool]) -> Vec<JointRows> {
        // Anchor effective mass, anchor-angle couplings and angular Schur complement.
        let blocks = |sa: SolverBody, sbb: SolverBody, ra: Vec3, rb: Vec3| {
            let k = Mat3::from_diagonal(Vec3::splat(sa.inv_mass + sbb.inv_mass))
                - skew(ra) * sa.inv_i * skew(ra)
                - skew(rb) * sbb.inv_i * skew(rb);
            let point_mass = if k.determinant().abs() > 1e-12 {
                k.inverse()
            } else {
                Mat3::ZERO
            };
            let (sra, srb) = (skew(ra), skew(rb));
            let k_pa = -(sra * sa.inv_i + srb * sbb.inv_i);
            let k_ap = sa.inv_i * sra + sbb.inv_i * srb;
            (
                point_mass,
                k_pa,
                k_ap,
                sa.inv_i + sbb.inv_i - k_ap * point_mass * k_pa,
            )
        };
        // Only for `SpringCap::Limbs`; the default caps by the joint's own two bodies.
        let limbs = (self.spring_cap == SpringCap::Limbs).then(|| Limbs::new(self, grounded));
        let mut out = Vec::new();
        for (index, joint) in self.joints.iter().enumerate() {
            let (a, b) = (joint.parent, joint.child);
            if !joint.enabled || (!self.bodies[a].is_dynamic() && !self.bodies[b].is_dynamic()) {
                continue;
            }
            let (ba, bb) = (&self.bodies[a], &self.bodies[b]);
            let ra = ba.rotation * joint.anchor_parent;
            let rb = bb.rotation * joint.anchor_child;
            let (sa, sbb) = (sb[a], sb[b]);
            let (point_mass, k_pa, k_ap, angular_schur) = blocks(sa, sbb, ra, rb);
            // The two bodies the joint connects, a grounded one held still unless both
            // are: a foot and toe both on the ground are still just a foot and a toe
            // (lesson 9).
            let still = |s: SolverBody| SolverBody {
                inv_mass: 0.0,
                inv_i: Mat3::ZERO,
                ..s
            };
            let cap_schur = match (grounded[a], grounded[b]) {
                (true, false) => blocks(still(sa), sbb, ra, rb).3,
                (false, true) => blocks(sa, still(sbb), ra, rb).3,
                _ => angular_schur,
            };
            // What the muscle moves as a whole: the limb hanging from the joint against
            // the rest of the body, a side resting on the ground or scenery held still.
            let limb_schur = limbs.as_ref().map(|limbs| {
                let (limb, rest) = limbs.sides(b);
                let anchor = ba.position + ra;
                blocks(
                    rest.as_body(),
                    limb.as_body(),
                    rest.lever(anchor),
                    limb.lever(anchor),
                )
                .3
            });
            let mut rows = JointRows {
                joint: index,
                a,
                b,
                ra,
                rb,
                point_error: (bb.position + rb) - (ba.position + ra),
                point_mass,
                point_impulse: joint.warm_point * WARM_START,
                k_pa,
                k_ap,
                angular_schur,
                cap_schur,
                limb_schur,
                limits: Vec::new(),
                muscle: None,
            };
            self.prepare_limits(joint, h, &mut rows);
            rows.muscle = self.prepare_muscle(joint, h, &rows);
            out.push(rows);
        }
        out
    }

    fn prepare_limits(&self, joint: &Joint, h: f32, rows: &mut JointRows) {
        let (a, b) = (rows.a, rows.b);
        let primary = self.bodies[a].rotation * joint.limit_parent;
        let secondary = self.bodies[b].rotation * joint.limit_child;
        let (twist, swing) = swing_twist(primary.inverse() * secondary);
        let twist_axis = secondary * Vec3::X;

        let schur = rows.angular_schur;
        let coupled_inv_mass = |axis: Vec3| axis.dot(schur * axis);
        let rigid = |axis: Vec3, value: f32| AngularRow {
            axis,
            mass: inverse(coupled_inv_mass(axis)),
            value,
            spring: false,
            gamma: 0.0,
            lower: 0.0,
            upper: f32::INFINITY,
            impulse: 0.0,
        };
        let spring = |axis: Vec3, error: f32| -> Option<AngularRow> {
            let (k, c) = cap_spring(
                joint.soft_stiffness,
                joint.soft_damping,
                spring_ceiling(MAX_SPRING_HERTZ, coupled_inv_mass(axis)),
            );
            (k + c > 0.0).then(|| {
                let denom = c + h * k;
                let gamma = 1.0 / (h * denom);
                AngularRow {
                    axis,
                    mass: inverse(coupled_inv_mass(axis) + gamma),
                    value: (h * k / denom) / h * error,
                    spring: true,
                    gamma,
                    lower: 0.0,
                    upper: f32::INFINITY,
                    impulse: 0.0,
                }
            })
        };

        // Twist: stops at both ends, soft springs between the soft and hard ranges.
        let (hard, soft_l) = (joint.hard, joint.soft);
        if twist < hard.twist_min + ANGULAR_MARGIN {
            rows.limits.push(rigid(twist_axis, twist - hard.twist_min));
        }
        if twist > hard.twist_max - ANGULAR_MARGIN {
            rows.limits.push(rigid(-twist_axis, hard.twist_max - twist));
        }
        if twist < soft_l.twist_min {
            rows.limits
                .extend(spring(twist_axis, twist - soft_l.twist_min));
        }
        if twist > soft_l.twist_max {
            rows.limits
                .extend(spring(-twist_axis, soft_l.twist_max - twist));
        }

        // Swing: elliptical cone in the primary frame's YZ plane.
        let s = glam::Vec2::new(swing.y, swing.z);
        let radius = s.length();
        if radius > 1e-6 {
            let dir = s / radius;
            let boundary = |l: &Limits| {
                let (y, z) = (l.swing1.max(1e-3), l.swing2.max(1e-3));
                (
                    1.0 / ((dir.x / y).powi(2) + (dir.y / z).powi(2)).sqrt(),
                    y,
                    z,
                )
            };
            let (hard_r, y, z) = boundary(&hard);
            let n = glam::Vec2::new(s.x / (y * y), s.y / (z * z)).normalize_or_zero();
            let outward = (primary * Vec3::new(0.0, n.x, n.y)).normalize_or_zero();
            if radius > hard_r - ANGULAR_MARGIN {
                rows.limits.push(rigid(-outward, hard_r - radius));
            }
            let (soft_r, ..) = boundary(&soft_l);
            if radius > soft_r {
                rows.limits.extend(spring(-outward, soft_r - radius));
            }
        }
    }

    fn prepare_muscle(&self, joint: &Joint, h: f32, rows: &JointRows) -> Option<MuscleBlock> {
        let muscle = joint.muscle?;
        let b = rows.b;
        let qb = self.bodies[b].rotation * joint.drive_child;
        let target = joint.clamp_to_limits(muscle.target);
        let mut error = joint.local_rotation(&self.bodies).inverse() * target;
        if error.w < 0.0 {
            error = -error;
        }
        let error_world = qb * error.to_scaled_axis();
        let frame = Mat3::from_quat(self.bodies[b].rotation * joint.limit_child);

        // Per drive axis: implicit spring softness, velocity bias and impulse bound.
        let mut gamma = Vec3::ZERO;
        let mut bias = Vec3::ZERO;
        let mut bound = Vec3::ZERO;
        for k in 0..3 {
            let gains = if k == 0 { muscle.twist } else { muscle.swing };
            if gains.stiffness <= 0.0 && gains.damping <= 0.0 {
                gamma[k] = 1e9; // an axis with no muscle can't push
                continue;
            }
            let axis = frame.col(k);
            let mut max_stiffness =
                spring_ceiling(MAX_SPRING_HERTZ, axis.dot(rows.cap_schur * axis));
            if let Some(limb) = rows.limb_schur {
                max_stiffness =
                    max_stiffness.max(spring_ceiling(MAX_LIMB_HERTZ, axis.dot(limb * axis)));
            }
            let (stiffness, damping) = cap_spring(gains.stiffness, gains.damping, max_stiffness);
            let denom = damping + h * stiffness;
            gamma[k] = 1.0 / (h * denom);
            bias[k] = -(stiffness / denom) * error_world.dot(axis);
            bound[k] = gains.force_limit.max(0.0) * h;
        }
        let gamma = frame * Mat3::from_diagonal(gamma) * frame.transpose();
        let schur = rows.angular_schur + gamma;
        if schur.determinant().abs() < 1e-18 {
            return None;
        }
        let warm = Vec3::from(joint.warm_muscle).clamp(-bound, bound);
        Some(MuscleBlock {
            frame,
            gamma,
            bias: frame * bias,
            bound,
            impulse: frame * warm,
            schur_inv: schur.inverse(),
        })
    }

    fn solve_joints(
        &self,
        sb: &mut [SolverBody],
        joints: &mut [JointRows],
        soft_params: Softness,
        h: f32,
        use_bias: bool,
    ) {
        for j in joints.iter_mut() {
            let (a, b) = (j.a, j.b);
            let point_bias = if use_bias {
                (j.point_error * soft_params.bias_rate).clamp_length_max(MAX_PUSH_SPEED)
            } else {
                Vec3::ZERO
            };
            // Muscle first, then the stops (which always win), then the anchor. Each is
            // solved together with the anchor so it acts on the whole limb.
            if let Some(m) = j.muscle.as_mut() {
                let rp = relative_velocity(sb, a, b, j.ra, j.rb) + point_bias;
                let dw = get(sb, b).w - get(sb, a).w;
                let ra_rhs = dw + m.bias + m.gamma * m.impulse;
                let d_ang = m.schur_inv * (-ra_rhs + j.k_ap * (j.point_mass * rp));
                // Respect each drive axis's force limit.
                let local = m.frame.transpose() * (m.impulse + d_ang);
                let clamped = local.clamp(-m.bound, m.bound);
                let d_ang = m.frame * clamped - m.impulse;
                m.impulse += d_ang;
                apply_angular(sb, a, b, d_ang);
                let d_lin = j.point_mass * (-rp - j.k_pa * d_ang);
                j.point_impulse += d_lin;
                apply_impulse(sb, a, b, j.ra, j.rb, d_lin);
            }
            for row in j.limits.iter_mut() {
                let rp = relative_velocity(sb, a, b, j.ra, j.rb) + point_bias;
                let coupling = row.axis.dot(j.k_ap * (j.point_mass * rp));
                let d = solve_angular(sb, a, b, row, soft_params, h, use_bias, coupling);
                if d != 0.0 {
                    let d_lin = j.point_mass * (-rp - j.k_pa * (row.axis * d));
                    j.point_impulse += d_lin;
                    apply_impulse(sb, a, b, j.ra, j.rb, d_lin);
                }
            }
            let impulse = -(j.point_mass * (relative_velocity(sb, a, b, j.ra, j.rb) + point_bias));
            j.point_impulse += impulse;
            apply_impulse(sb, a, b, j.ra, j.rb, impulse);
        }
    }

    // -----------------------------------------------------------------------------
    // Virtual muscles

    fn prepare_virtual_muscles(&self, sb: &[SolverBody], h: f32) -> Vec<DriveBlock> {
        let mut out = Vec::new();
        for vm in &self.virtual_muscles {
            let (a, b) = (vm.anchor, vm.body);
            if !self.bodies[a].is_dynamic() && !self.bodies[b].is_dynamic() {
                continue;
            }
            let (ba, bb) = (&self.bodies[a], &self.bodies[b]);
            let (sa, sbb) = (sb[a], sb[b]);
            let mut block = |linear: bool, gains: Gains, ra: Vec3, rb: Vec3, error: Vec3| {
                let k = if linear {
                    Mat3::from_diagonal(Vec3::splat(sa.inv_mass + sbb.inv_mass))
                        - skew(ra) * sa.inv_i * skew(ra)
                        - skew(rb) * sbb.inv_i * skew(rb)
                } else {
                    sa.inv_i + sbb.inv_i
                };
                // Like a muscle, no stiffer than the solver can follow on the lightest
                // direction the two bodies move in (a hand alone weighs 0.4 kg).
                let lightest = k.x_axis.x.max(k.y_axis.y).max(k.z_axis.z);
                let (stiffness, damping) = cap_spring(
                    gains.stiffness.max(0.0),
                    gains.damping.max(0.0),
                    spring_ceiling(MAX_SPRING_HERTZ, lightest),
                );
                let denom = damping + h * stiffness;
                if denom <= 0.0 || gains.force_limit <= 0.0 {
                    return;
                }
                let gamma = 1.0 / (h * denom);
                let soft = k + Mat3::from_diagonal(Vec3::splat(gamma));
                if soft.determinant().abs() < 1e-18 {
                    return;
                }
                out.push(DriveBlock {
                    a,
                    b,
                    ra,
                    rb,
                    linear,
                    gamma,
                    bias: error * (stiffness / denom),
                    mass: soft.inverse(),
                    bound: gains.force_limit * h,
                    impulse: Vec3::ZERO,
                });
            };
            if let Some(gains) = vm.linear {
                let rb = bb.rotation * vm.point;
                let ra = ba.rotation * vm.target_position;
                let error = (bb.position + rb) - (ba.position + ra);
                block(true, gains, ra, rb, error);
            }
            if let Some(gains) = vm.angular {
                let mut turn = bb.rotation * (ba.rotation * vm.target_rotation).inverse();
                if turn.w < 0.0 {
                    turn = -turn;
                }
                block(false, gains, Vec3::ZERO, Vec3::ZERO, turn.to_scaled_axis());
            }
        }
        out
    }

    // -----------------------------------------------------------------------------
    // Locks

    fn prepare_locks(&self, sb: &[SolverBody], h: f32) -> Vec<LockBlock> {
        let mut out = Vec::new();
        let mut seen: HashMap<(usize, usize), usize> = HashMap::new();
        for lock in &self.locks {
            let b = lock.body;
            let a = lock.anchor.unwrap_or(STATIC);
            let nth = seen.entry((a, b)).or_default();
            let key = (a, b, *nth);
            *nth += 1;
            if !self.bodies[b].is_dynamic()
                && lock.anchor.is_none_or(|a| !self.bodies[a].is_dynamic())
            {
                continue;
            }
            let (anchor_rot, anchor_pos) = match lock.anchor {
                Some(a) => (self.bodies[a].rotation, self.bodies[a].position),
                None => (Quat::IDENTITY, Vec3::ZERO),
            };
            let body = &self.bodies[b];
            // The lever is where the body is, not where it should be: a join with a free
            // or driving axis lets the body slide far from its target, and the held axes
            // must turn with the anchor about that point or they drift as it turns.
            let ra = body.position - anchor_pos;
            let frame = Mat3::from_quat(anchor_rot);
            let mut turn = body.rotation * (anchor_rot * lock.target.0).inverse();
            if turn.w < 0.0 {
                turn = -turn;
            }
            let position_error =
                frame.transpose() * (body.position - (anchor_pos + anchor_rot * lock.target.1));
            let rotation_error = frame.transpose() * turn.to_scaled_axis();

            // Effective mass in lock axes: [linear; angular] rows, B's centre held.
            let (ia, ib) = (get(sb, a), get(sb, b));
            let sra = skew(ra);
            let k_ll =
                Mat3::from_diagonal(Vec3::splat(ia.inv_mass + ib.inv_mass)) - sra * ia.inv_i * sra;
            let k_la = -(sra * ia.inv_i);
            let k_aa = ia.inv_i + ib.inv_i;
            let blocks = [
                [
                    frame.transpose() * k_ll * frame,
                    frame.transpose() * k_la * frame,
                ],
                [
                    frame.transpose() * k_la.transpose() * frame,
                    frame.transpose() * k_aa * frame,
                ],
            ];
            let strength = [lock.linear.to_array(), lock.angular.to_array()].concat();
            let mut k = [[0.0f32; 6]; 6];
            let mut bound = [0.0f32; 6];
            let mut brake = [0.0f32; 6];
            for r in 0..6 {
                for c in 0..6 {
                    k[r][c] = blocks[r / 3][c / 3].col(c % 3)[r % 3];
                }
            }
            for r in 0..6 {
                // Free axes, and axes neither body can move along, drop out of the block.
                if strength[r] > 0.0 && k[r][r] > 1e-9 {
                    bound[r] = strength[r] * h;
                    brake[r] = (2.0 * strength[r] * k[r][r]).sqrt();
                } else {
                    for c in 0..6 {
                        k[r][c] = 0.0;
                        k[c][r] = 0.0;
                    }
                    k[r][r] = 1.0;
                }
            }
            let Some(inv_k) = invert6(k) else { continue };
            let mut impulse = self.lock_cache.get(&key).copied().unwrap_or([0.0; 6]);
            for (i, bound) in impulse.iter_mut().zip(bound) {
                *i = i.clamp(-bound, bound);
            }
            let mut error = [0.0f32; 6];
            error[..3].copy_from_slice(&position_error.to_array());
            error[3..].copy_from_slice(&rotation_error.to_array());
            out.push(LockBlock {
                a,
                b,
                ra,
                frame,
                error,
                inv_k,
                bound,
                impulse,
                pulling: [false; 6],
                brake,
                key,
            });
        }
        out
    }

    /// Groups of moving bodies that locks tie together (with the joints between them),
    /// jointed groups something heavy presses on or with a joint come apart, and the rows
    /// each group solves directly. Nothing else is grouped.
    fn prepare_groups(
        &self,
        sb: &[SolverBody],
        joints: &[JointRows],
        locks: &[LockBlock],
        contacts: &[ContactRow],
        bias_rate: f32,
        contact_bias_rate: f32,
        h: f32,
    ) -> Vec<LockGroup> {
        let stretched = |j: &&JointRows| j.point_error.length() > STRETCH_LIMIT;
        if locks.is_empty() && contacts.is_empty() && !joints.iter().any(|j| stretched(&j)) {
            return Vec::new();
        }
        let moves = |i: usize| i != STATIC && self.bodies[i].is_dynamic();
        let mut root: Vec<usize> = (0..self.bodies.len()).collect();
        fn find(root: &mut [usize], mut i: usize) -> usize {
            while root[i] != i {
                root[i] = root[root[i]];
                i = root[i];
            }
            i
        }
        let pairs = joints
            .iter()
            .map(|j| (j.a, j.b))
            .chain(locks.iter().map(|l| (l.a, l.b)));
        for (a, b) in pairs.filter(|&(a, b)| moves(a) && moves(b)) {
            let (ra, rb) = (find(&mut root, a), find(&mut root, b));
            root[ra] = rb;
        }
        // The group a row belongs to: that of whichever of its bodies moves.
        let mut group_of = |a: usize, b: usize| {
            let body = if moves(a) { a } else { b };
            moves(body).then(|| find(&mut root, body))
        };
        let mut locked: Vec<usize> = locks.iter().filter_map(|l| group_of(l.a, l.b)).collect();
        // Groups a lock holds to the world or a keyed body can't be pushed anywhere the
        // lock doesn't allow.
        let pinned: Vec<usize> = locks
            .iter()
            .filter(|l| !moves(l.a) && l.bound.iter().any(|&b| b > 0.0))
            .filter_map(|l| group_of(l.b, l.b))
            .collect();
        // Free groups a lock closes a loop in (a prop held in both hands) are solved
        // after the contacts (lesson 25); the others, before them as always.
        let mut late_groups = locked.clone();
        late_groups.retain(|g| !pinned.contains(g));
        // Groups something presses on that won't give way (lesson 15).
        locked.extend(self.crushed_groups(contacts, locks, &mut |i| group_of(i, i)));
        // Groups with a joint come apart (lesson 17). A light limb whipping round
        // against its own body closes a loop of joints and contacts that four
        // iterations don't settle: the contacts, solved last, win, and the joint keeps
        // opening. Solved directly, the joints hold again within a substep or two.
        locked.extend(
            joints
                .iter()
                .filter(stretched)
                .filter_map(|j| group_of(j.a, j.b)),
        );
        locked.sort_unstable();
        locked.dedup();
        let mut rows: Vec<Vec<GroupRow>> = locked.iter().map(|_| Vec::new()).collect();

        for (ji, j) in joints.iter().enumerate() {
            let Some(g) = group_of(j.a, j.b).and_then(|g| locked.binary_search(&g).ok()) else {
                continue;
            };
            let bias = (j.point_error * bias_rate).clamp_length_max(MAX_PUSH_SPEED);
            for (axis, e) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
                rows[g].push(GroupRow {
                    a: j.a,
                    b: j.b,
                    jacobian: [-e, -j.ra.cross(e), e, j.rb.cross(e)],
                    bias: bias[axis],
                    bound: f32::INFINITY,
                    floor: f32::NEG_INFINITY,
                    source: RowSource::Anchor(ji, axis),
                });
            }
        }
        for (li, l) in locks.iter().enumerate() {
            let Some(g) = group_of(l.a, l.b).and_then(|g| locked.binary_search(&g).ok()) else {
                continue;
            };
            let biases = l.bias(bias_rate);
            for axis in (0..6).filter(|&r| l.bound[r] > 0.0) {
                let f = l.frame.col(axis % 3);
                let jacobian = if axis < 3 {
                    [-f, -l.ra.cross(f), f, Vec3::ZERO]
                } else {
                    [Vec3::ZERO, -f, Vec3::ZERO, f]
                };
                let bias = biases[axis];
                rows[g].push(GroupRow {
                    a: l.a,
                    b: l.b,
                    jacobian,
                    bias,
                    bound: l.bound[axis],
                    floor: -l.bound[axis],
                    source: RowSource::Lock(li, axis),
                });
            }
        }
        // Contacts that carry a load (lesson 25): a touching contact of a
        // group against the ground or massless scenery joins its direct solve as a
        // one-sided row. A load
        // that reaches a light body through a lock (a gun's weight, through a hand, onto
        // a finger) is then carried by the contact, not by the softness the contact pass
        // gives a body that weighs 0.39 kg. Not against another body: two bodies of one
        // group are held by joints and locks, and a contact between them settles with
        // those in huge opposing impulses. Not against a keyed body either, which can
        // move on. Only in the groups that are solved late: for pinned ones a contact row
        // against the lock that holds them only adds stress, and groups solved directly
        // for a crushing body or an open joint (lessons 15 and 17) stay as they were.
        for (ci, c) in contacts.iter().enumerate() {
            // Carrying a load: it pushed last substep, and touches now.
            let carrying = c.impulse[0] > 0.0 && c.separation < CRUSH_SEPARATION;
            if !carrying || (c.a == STATIC && c.b == STATIC) {
                continue;
            }
            let (ga, gb) = (group_of(c.a, c.a), group_of(c.b, c.b));
            let fixed = |i: usize| i == STATIC || self.bodies[i].motion == Motion::Dynamic;
            let g = match (ga, gb) {
                (Some(a), None) if fixed(c.b) => a,
                (None, Some(b)) if fixed(c.a) => b,
                _ => continue,
            };
            if !late_groups.contains(&g) {
                continue;
            }
            let Some(g) = locked.binary_search(&g).ok() else {
                continue;
            };
            let n = c.normal;
            rows[g].push(GroupRow {
                a: c.a,
                b: c.b,
                jacobian: [-n, -c.ra.cross(n), n, c.rb.cross(n)],
                bias: if c.separation < 0.0 {
                    (contact_bias_rate * c.separation).max(-MAX_PUSH_SPEED)
                } else {
                    0.0
                },
                bound: f32::INFINITY,
                floor: -c.interaction.adhesion * h,
                source: RowSource::Contact(ci),
            });
        }
        // Joints leaf first (deepest child body first), then locks.
        let depth = |mut body: usize| {
            let mut d = 0;
            while let Some(p) = self.bodies[body].parent {
                d += 1;
                body = p;
            }
            d
        };
        for rows in rows.iter_mut() {
            rows.sort_by_key(|r| match r.source {
                RowSource::Anchor(..) => (0, std::cmp::Reverse(depth(r.b))),
                RowSource::Lock(..) => (1, std::cmp::Reverse(0)),
                RowSource::Contact(..) => (2, std::cmp::Reverse(0)),
            });
        }
        rows.into_iter()
            .zip(&locked)
            .filter_map(|(rows, g)| {
                let mut group = LockGroup::new(sb, rows, locks)?;
                group.late = late_groups.contains(g);
                Some(group)
            })
            .collect()
    }

    /// Groups of jointed bodies (characters, mostly) pressed on by something that won't
    /// give way: a group at least `CRUSH_RATIO` times heavier, or one a lock holds to the
    /// world or to a keyed body (a car joined to its carrier). A limb caught between such
    /// a body and the ground, or the rest of its own character, can't get out of the way,
    /// and sequential impulses then let its joints come apart. These groups are solved
    /// directly, like locked ones, while it lasts (lesson 15).
    fn crushed_groups(
        &self,
        contacts: &[ContactRow],
        locks: &[LockBlock],
        group_of: &mut dyn FnMut(usize) -> Option<usize>,
    ) -> Vec<usize> {
        // Contacts between a jointed body and a body of another moving group.
        let mut jointed = vec![false; self.bodies.len()];
        for joint in self.joints.iter().filter(|j| j.enabled) {
            jointed[joint.parent] = true;
            jointed[joint.child] = true;
        }
        let mut pressed: Vec<(usize, usize)> = Vec::new();
        for c in contacts.iter().filter(|c| c.separation < CRUSH_SEPARATION) {
            for (mine, other) in [(c.a, c.b), (c.b, c.a)] {
                if mine == STATIC || other == STATIC || !jointed[mine] {
                    continue;
                }
                if let (Some(g), Some(h)) = (group_of(mine), group_of(other))
                    && g != h
                {
                    pressed.push((g, h));
                }
            }
        }
        if pressed.is_empty() {
            return Vec::new();
        }
        let mut mass: HashMap<usize, f32> = HashMap::new();
        for (i, body) in self.bodies.iter().enumerate() {
            if let Some(g) = group_of(i) {
                *mass.entry(g).or_default() += 1.0 / body.inv_mass;
            }
        }
        let held: Vec<usize> = locks
            .iter()
            .filter(|l| l.a == STATIC || !self.bodies[l.a].is_dynamic())
            .filter_map(|l| group_of(l.b))
            .collect();
        pressed
            .into_iter()
            .filter(|(g, h)| held.contains(h) || mass[h] >= CRUSH_RATIO * mass[g])
            .map(|(g, _)| g)
            .collect()
    }

    /// Body pairs held together by locks, which don't collide with each other: each
    /// locked body and its anchor, and bodies locked to the same anchor (two hands
    /// clasped around someone).
    fn locked_pairs(&self) -> Vec<(usize, usize)> {
        let pair = |a: usize, b: usize| (a.min(b), a.max(b));
        let mut out: Vec<(usize, usize)> = self
            .locks
            .iter()
            .filter_map(|l| Some(pair(l.anchor?, l.body)))
            .collect();
        for (i, l) in self.locks.iter().enumerate() {
            let Some(anchor) = l.anchor else { continue };
            for m in &self.locks[i + 1..] {
                if m.anchor == Some(anchor) && m.body != l.body {
                    out.push(pair(l.body, m.body));
                }
            }
        }
        out
    }

    // -----------------------------------------------------------------------------
    // Contacts

    fn find_contacts(&self, sb: &[SolverBody], h: f32) -> Vec<ContactRow> {
        let placed: Vec<Option<Placed>> = self
            .shapes
            .iter()
            .map(|s| match s.body {
                None => Some(Placed {
                    geometry: s.geometry,
                    rotation: s.rotation,
                    position: s.position,
                }),
                Some(b) => {
                    let body = &self.bodies[b];
                    (body.motion != Motion::Ghost).then(|| Placed {
                        geometry: s.geometry,
                        rotation: body.rotation * s.rotation,
                        position: body.position + body.rotation * s.position,
                    })
                }
            })
            .collect();
        let radius: Vec<f32> = self
            .shapes
            .iter()
            .map(|s| collide::bounding_radius(&s.geometry))
            .collect();
        let dynamic = |s: &Shape| s.body.is_some_and(|b| self.bodies[b].is_dynamic());
        let locked = self.locked_pairs();
        let held_together = |a: Option<usize>, b: Option<usize>| match (a, b) {
            (Some(a), Some(b)) => locked.contains(&(a.min(b), a.max(b))),
            _ => false,
        };

        let mut out = Vec::new();
        let mut found = Vec::new();
        if let Some((height, ground_material)) = self.ground {
            for (i, shape) in self.shapes.iter().enumerate() {
                let Some(p) = &placed[i] else { continue };
                if !dynamic(shape) || p.position.y - radius[i] > height + MARGIN {
                    continue;
                }
                found.clear();
                collide::with_ground(p, height, MARGIN, &mut found);
                for c in &found {
                    out.push(self.contact_row(
                        sb,
                        h,
                        None,
                        Some(i),
                        ground_material,
                        shape.material,
                        c,
                    ));
                }
            }
        }
        for i in 0..self.shapes.len() {
            let Some(pi) = &placed[i] else { continue };
            for j in (i + 1)..self.shapes.len() {
                let Some(pj) = &placed[j] else { continue };
                let (si, sj) = (&self.shapes[i], &self.shapes[j]);
                if !(dynamic(si) || dynamic(sj))
                    || si.body == sj.body
                    || held_together(si.body, sj.body)
                {
                    continue;
                }
                let reach = radius[i] + radius[j] + MARGIN;
                if pi.position.distance_squared(pj.position) > reach * reach
                    || !self.may_collide(i, j)
                {
                    continue;
                }
                found.clear();
                collide::between(pi, pj, MARGIN, &mut found);
                for c in &found {
                    out.push(self.contact_row(
                        sb,
                        h,
                        Some(i),
                        Some(j),
                        si.material,
                        sj.material,
                        c,
                    ));
                }
            }
        }
        out
    }

    /// Self-collision rules within one character, following each shape's flags.
    fn may_collide(&self, i: usize, j: usize) -> bool {
        let (si, sj) = (&self.shapes[i], &self.shapes[j]);
        let (Some(bi), Some(bj)) = (si.body, sj.body) else {
            return true;
        };
        let (a, b) = (&self.bodies[bi], &self.bodies[bj]);
        if a.group != b.group {
            return true;
        }
        if b.parent == Some(bi) {
            return sj.flags.parent;
        }
        if a.parent == Some(bj) {
            return si.flags.parent;
        }
        if a.parent.is_some() && a.parent == b.parent {
            return si.flags.siblings && sj.flags.siblings;
        }
        if self.is_ancestor(bi, bj) {
            return sj.flags.ancestors;
        }
        if self.is_ancestor(bj, bi) {
            return si.flags.ancestors;
        }
        true
    }

    fn is_ancestor(&self, ancestor: usize, mut body: usize) -> bool {
        while let Some(p) = self.bodies[body].parent {
            if p == ancestor {
                return true;
            }
            body = p;
        }
        false
    }

    #[allow(clippy::too_many_arguments)]
    fn contact_row(
        &self,
        sb: &[SolverBody],
        h: f32,
        shape_a: Option<usize>,
        shape_b: Option<usize>,
        material_a: usize,
        material_b: usize,
        c: &collide::Contact,
    ) -> ContactRow {
        let body_of = |s: Option<usize>| s.and_then(|i| self.shapes[i].body).unwrap_or(STATIC);
        let (a, b) = (body_of(shape_a), body_of(shape_b));
        let origin = |i: usize| {
            if i == STATIC {
                c.point
            } else {
                self.bodies[i].position
            }
        };
        let (ra, rb) = (c.point - origin(a), c.point - origin(b));
        let interaction = self.interaction(material_a, material_b);
        let row_mass = |dir: Vec3, softness: f32| {
            let (ba, bb) = (get(sb, a), get(sb, b));
            let k = ba.inv_mass
                + bb.inv_mass
                + ra.cross(dir).dot(ba.inv_i * ra.cross(dir))
                + rb.cross(dir).dot(bb.inv_i * rb.cross(dir));
            inverse(k + softness)
        };
        let t1 = perpendicular(c.normal);
        let tangents = [t1, c.normal.cross(t1)];
        let key = (
            shape_a.map_or(u32::MAX, |s| s as u32),
            shape_b.map_or(u32::MAX, |s| s as u32),
            c.feature,
        );
        let cached = self.contact_cache.get(&key).copied();
        ContactRow {
            a,
            b,
            ra,
            rb,
            normal: c.normal,
            tangents,
            separation: c.separation,
            normal_mass: row_mass(c.normal, interaction.softness / h),
            tangent_mass: [row_mass(tangents[0], 0.0), row_mass(tangents[1], 0.0)],
            approach: handed_velocity(sb, a, b, ra, rb).dot(c.normal),
            interaction,
            impulse: cached.unwrap_or([0.0; 3]),
            key,
        }
    }

    fn solve_contacts(
        &self,
        sb: &mut [SolverBody],
        contacts: &mut [ContactRow],
        held: &[bool],
        soft_params: Softness,
        h: f32,
        use_bias: bool,
    ) {
        // While correcting positions, moved bodies push with their full velocity; after,
        // only with what they hand on.
        let velocity = if use_bias {
            relative_velocity
        } else {
            handed_velocity
        };
        for (c, &held) in contacts.iter_mut().zip(held) {
            // Friction first, limited by the current normal impulse or a fixed force cap.
            // The cap only holds surfaces that touch: a contact found inside the
            // speculative margin, still apart and not pushing, has nothing to grip with.
            let touching = c.separation <= 0.0 || c.impulse[0] > 0.0;
            let limit = match self.contact_model {
                ContactModel::ForceCapped if !touching => 0.0,
                ContactModel::ForceCapped => c.interaction.friction * h,
                ContactModel::Coulomb => c.interaction.friction / 100.0 * c.impulse[0].max(0.0),
            };
            for k in 0..2 {
                let vt = velocity(sb, c.a, c.b, c.ra, c.rb).dot(c.tangents[k]);
                let old = c.impulse[k + 1];
                let new = (old - c.tangent_mass[k] * vt).clamp(-limit, limit);
                c.impulse[k + 1] = new;
                apply_impulse(sb, c.a, c.b, c.ra, c.rb, c.tangents[k] * (new - old));
            }
            // The normal is the group's, while it holds.
            if held {
                continue;
            }

            let vn = velocity(sb, c.a, c.b, c.ra, c.rb).dot(c.normal);
            let (bias, mass_scale, impulse_scale) = if c.separation > 0.0 {
                (c.separation / h, 1.0, 0.0)
            } else if use_bias {
                (
                    (soft_params.bias_rate * c.separation).max(-MAX_PUSH_SPEED),
                    soft_params.mass_scale,
                    soft_params.impulse_scale,
                )
            } else {
                (0.0, 1.0, 0.0)
            };
            let old = c.impulse[0];
            let impulse = -c.normal_mass * mass_scale * (vn + bias) - impulse_scale * old;
            let new = (old + impulse).max(-c.interaction.adhesion * h);
            c.impulse[0] = new;
            apply_impulse(sb, c.a, c.b, c.ra, c.rb, c.normal * (new - old));
        }
    }
}

impl LockBlock {
    /// Position-correcting velocity along the lock's rows; `bias_rate` is the share of
    /// the error corrected per second. An axis closes no faster than its strength could
    /// stop it on target (`brake`): faster, and a lock that has fallen far behind would
    /// arrive with more speed than its strength can take off, and swing past.
    fn bias(&self, bias_rate: f32) -> [f32; 6] {
        let push =
            (Vec3::from_slice(&self.error[..3]) * bias_rate).clamp_length_max(MAX_PUSH_SPEED);
        let turn =
            (Vec3::from_slice(&self.error[3..]) * bias_rate).clamp_length_max(MAX_PUSH_SPEED * 4.0);
        let mut bias = [0.0f32; 6];
        for r in 0..6 {
            let b = if r < 3 { push[r] } else { turn[r - 3] };
            let most = self.brake[r] * self.error[r].abs().sqrt();
            bias[r] = b.clamp(-most, most);
        }
        bias
    }

    /// Velocity along the lock's rows.
    fn velocity(&self, sb: &[SolverBody]) -> [f32; 6] {
        let linear =
            self.frame.transpose() * relative_velocity(sb, self.a, self.b, self.ra, Vec3::ZERO);
        let angular = self.frame.transpose() * (get(sb, self.b).w - get(sb, self.a).w);
        [
            linear.x, linear.y, linear.z, angular.x, angular.y, angular.z,
        ]
    }

    fn apply(&self, sb: &mut [SolverBody], impulse: [f32; 6]) {
        let linear = self.frame * Vec3::new(impulse[0], impulse[1], impulse[2]);
        let angular = self.frame * Vec3::new(impulse[3], impulse[4], impulse[5]);
        apply_impulse(sb, self.a, self.b, self.ra, Vec3::ZERO, linear);
        apply_angular(sb, self.a, self.b, angular);
    }
}

/// Solves each lock's block; `bias_rate` is the share of the error corrected per second.
/// Virtual muscles' springs: implicit, so `γ·impulse` stands in for the spring's give,
/// with the accumulated impulse kept within the force limit.
fn solve_drives(sb: &mut [SolverBody], drives: &mut [DriveBlock]) {
    for d in drives.iter_mut() {
        let velocity = if d.linear {
            relative_velocity(sb, d.a, d.b, d.ra, d.rb)
        } else {
            get(sb, d.b).w - get(sb, d.a).w
        };
        let total = (d.impulse - d.mass * (velocity + d.bias + d.gamma * d.impulse))
            .clamp_length_max(d.bound);
        let delta = total - d.impulse;
        d.impulse = total;
        if d.linear {
            apply_impulse(sb, d.a, d.b, d.ra, d.rb, delta);
        } else {
            apply_angular(sb, d.a, d.b, delta);
        }
    }
}

fn solve_locks(sb: &mut [SolverBody], locks: &mut [LockBlock], bias_rate: f32) {
    for l in locks.iter_mut() {
        let mut rhs = l.velocity(sb);
        for (r, b) in rhs.iter_mut().zip(l.bias(bias_rate)) {
            *r = -(*r + b);
        }
        let mut delta = [0.0f32; 6];
        for (r, d) in delta.iter_mut().enumerate() {
            // Relaxing takes back what position correction added, but not from an axis
            // that can't keep up: a real force-capped constraint would still pull.
            if bias_rate == 0.0 && l.pulling[r] {
                continue;
            }
            let wanted = l.impulse[r] + (0..6).map(|c| l.inv_k[r][c] * rhs[c]).sum::<f32>();
            let clamped = wanted.clamp(-l.bound[r], l.bound[r]);
            *d = clamped - l.impulse[r];
            l.impulse[r] = clamped;
        }
        l.apply(sb, delta);
    }
}

impl LockGroup {
    /// Builds the rows' effective mass matrix. Rows neither body can move along drop out.
    fn new(sb: &[SolverBody], rows: Vec<GroupRow>, locks: &[LockBlock]) -> Option<LockGroup> {
        let moves = |body: usize| {
            body != STATIC && (sb[body].inv_mass > 0.0 || sb[body].inv_i != Mat3::ZERO)
        };
        // M⁻¹ Jᵀ per row: change in A's and B's velocities per unit impulse.
        let response = |r: &GroupRow| {
            let (sa, sbb) = (get(sb, r.a), get(sb, r.b));
            [
                r.jacobian[0] * sa.inv_mass,
                sa.inv_i * r.jacobian[1],
                r.jacobian[2] * sbb.inv_mass,
                sbb.inv_i * r.jacobian[3],
            ]
        };
        let entry = |ri: &GroupRow, rj: &GroupRow, mj: &[Vec3; 4]| {
            let mut k = 0.0;
            for (si, bi) in [ri.a, ri.b].into_iter().enumerate() {
                for (sj, bj) in [rj.a, rj.b].into_iter().enumerate() {
                    if bi == bj && bi != STATIC {
                        k += ri.jacobian[2 * si].dot(mj[2 * sj])
                            + ri.jacobian[2 * si + 1].dot(mj[2 * sj + 1]);
                    }
                }
            }
            k as f64
        };
        let rows: Vec<GroupRow> = rows
            .into_iter()
            .filter(|r| entry(r, r, &response(r)) > 1e-9)
            .collect();
        let n = rows.len();
        if n == 0 {
            return None;
        }
        let responses: Vec<[Vec3; 4]> = rows.iter().map(response).collect();
        // Rows meet only where they share a moving body.
        let mut on_body: HashMap<usize, Vec<usize>> = HashMap::new();
        for (i, r) in rows.iter().enumerate() {
            for body in [r.a, r.b].into_iter().filter(|&b| moves(b)) {
                on_body.entry(body).or_default().push(i);
            }
        }
        let mut k = vec![0.0f64; n * n];
        let mut k_pattern = vec![Vec::new(); n];
        for (i, r) in rows.iter().enumerate() {
            let mut near: Vec<usize> = [r.a, r.b]
                .into_iter()
                .filter(|&b| moves(b))
                .flat_map(|b| on_body[&b].iter().copied())
                .filter(|&j| j < i)
                .collect();
            near.sort_unstable();
            near.dedup();
            for &j in &near {
                k[i * n + j] = entry(r, &rows[j], &responses[j]);
            }
            k[i * n + i] = entry(r, r, &responses[i]);
            k_pattern[i] = near;
        }
        // Lock axes that ended the last substep at their strength most likely stay there.
        let out = rows
            .iter()
            .map(|r| match r.source {
                RowSource::Lock(l, axis) => locks[l].impulse[axis].abs() >= r.bound,
                RowSource::Anchor(..) | RowSource::Contact(..) => false,
            })
            .collect();
        let mut group = LockGroup {
            rows,
            k,
            k_pattern,
            pattern: Vec::new(),
            velocity_factor: Vec::new(),
            push_factor: Vec::new(),
            out,
            late: false,
        };
        group.factorize();
        Some(group)
    }

    /// Sparse Cholesky factors over the rows still solved. Rows that nearly repeat each
    /// other (a ring hanging on a straight cable can barely move up or down, and a lock
    /// on it says how far) would turn small disagreements into enormous opposing
    /// impulses, so each row's diagonal is raised by a share of itself: a little for the
    /// velocity solve, which stays close to exact, and more for position correction,
    /// which only has to head the right way (damped least squares).
    fn factorize(&mut self) {
        const NONE: usize = usize::MAX;
        let n = self.rows.len();
        // Where each row isn't zero: K's entries and everything above them in the
        // elimination tree (the first row below each column that isn't zero in it).
        let mut pattern: Vec<Vec<usize>> = vec![Vec::new(); n];
        let mut parent = vec![NONE; n];
        let mut mark = vec![NONE; n];
        for i in (0..n).filter(|&i| !self.out[i]) {
            let mut row = Vec::new();
            mark[i] = i;
            for &start in self.k_pattern[i].iter().filter(|&&j| !self.out[j]) {
                let mut j = start;
                while j < i && mark[j] != i {
                    row.push(j);
                    mark[j] = i;
                    j = parent[j];
                }
            }
            row.sort_unstable();
            for &j in &row {
                if parent[j] == NONE {
                    parent[j] = i;
                }
            }
            pattern[i] = row;
        }
        self.pattern = pattern;
        self.velocity_factor = self.cholesky(GROUP_SOFTNESS);
        self.push_factor = self.cholesky(GROUP_PUSH_SOFTNESS);
    }

    fn cholesky(&self, softness: f64) -> Vec<f64> {
        let n = self.rows.len();
        let k = &self.k;
        let mut l = vec![0.0f64; n * n];
        for i in 0..n {
            if self.out[i] {
                l[i * n + i] = 1.0;
                continue;
            }
            let mut pivot = k[i * n + i] * (1.0 + softness);
            for &j in &self.pattern[i] {
                let mut sum = k[i * n + j];
                for &p in &self.pattern[j] {
                    sum -= l[i * n + p] * l[j * n + p];
                }
                let v = sum / l[j * n + j];
                l[i * n + j] = v;
                pivot -= v * v;
            }
            // At least `softness` × the diagonal in exact arithmetic.
            l[i * n + i] = pivot.max(softness * k[i * n + i]).sqrt();
        }
        l
    }

    /// Impulses that make `velocity_scale` × every solved row's relative velocity plus
    /// `bias_scale` × its bias zero, with factor `l`.
    fn solve(
        &self,
        l: &[f64],
        sb: &[SolverBody],
        velocity_scale: f64,
        bias_scale: f64,
    ) -> Vec<f64> {
        let n = self.rows.len();
        let mut x: Vec<f64> = self
            .rows
            .iter()
            .zip(&self.out)
            .map(|(r, &out)| {
                if out {
                    return 0.0;
                }
                let (sa, sbb) = (get(sb, r.a), get(sb, r.b));
                let v = r.jacobian[0].dot(sa.v)
                    + r.jacobian[1].dot(sa.w)
                    + r.jacobian[2].dot(sbb.v)
                    + r.jacobian[3].dot(sbb.w);
                -(v as f64 * velocity_scale + r.bias as f64 * bias_scale)
            })
            .collect();
        for i in 0..n {
            let sum: f64 = self.pattern[i].iter().map(|&p| l[i * n + p] * x[p]).sum();
            x[i] = (x[i] - sum) / l[i * n + i];
        }
        for i in (0..n).rev() {
            x[i] /= l[i * n + i];
            for &p in &self.pattern[i] {
                x[p] -= l[i * n + p] * x[i];
            }
        }
        for (x, &out) in x.iter_mut().zip(&self.out) {
            if out {
                *x = 0.0;
            }
        }
        x
    }

    /// Impulses for one pass: the velocity solve, plus position correction, scaled down
    /// in the unlikely case it would move a body faster than a correction may.
    fn impulses(&self, sb: &[SolverBody], use_bias: bool) -> Vec<f64> {
        let mut x = self.solve(&self.velocity_factor, sb, 1.0, 0.0);
        if use_bias {
            let push = self.solve(&self.push_factor, sb, 0.0, 1.0);
            let mut moved = vec![(Vec3::ZERO, Vec3::ZERO); sb.len()];
            for (r, &d) in self.rows.iter().zip(&push) {
                for (side, body) in [r.a, r.b]
                    .into_iter()
                    .enumerate()
                    .filter(|&(_, b)| b != STATIC)
                {
                    let s = sb[body];
                    moved[body].0 += r.jacobian[2 * side] * (s.inv_mass * d as f32);
                    moved[body].1 += s.inv_i * (r.jacobian[2 * side + 1] * d as f32);
                }
            }
            let fastest = moved.iter().fold(0.0f32, |m, (v, w)| {
                m.max(v.length() / MAX_PUSH_SPEED)
                    .max(w.length() / MAX_ANGULAR_SPEED)
            });
            let scale = if fastest > 1.0 {
                1.0 / fastest as f64
            } else {
                1.0
            };
            for (x, p) in x.iter_mut().zip(push) {
                *x += p * scale;
            }
        }
        x
    }

    /// Applies impulse `d` along row `r` to its bodies' velocities.
    fn apply(r: &GroupRow, sb: &mut [SolverBody], d: f32) {
        if r.a != STATIC {
            let body = &mut sb[r.a];
            body.v += r.jacobian[0] * (body.inv_mass * d);
            body.w += body.inv_i * (r.jacobian[1] * d);
        }
        if r.b != STATIC {
            let body = &mut sb[r.b];
            body.v += r.jacobian[2] * (body.inv_mass * d);
            body.w += body.inv_i * (r.jacobian[3] * d);
        }
    }
}

/// Solves each lock group directly, adding the impulses to the joints' and locks' own so
/// they warm start. A lock axis that would pass its strength is held at it and the rest
/// solved again without it.
fn solve_groups(
    sb: &mut [SolverBody],
    groups: &mut [LockGroup],
    joints: &mut [JointRows],
    locks: &mut [LockBlock],
    contacts: &mut [ContactRow],
    held: &mut [bool],
    late: bool,
    use_bias: bool,
) {
    for group in groups.iter_mut().filter(|g| g.late == late) {
        // Relaxing leaves axes that are pulling with their full strength alone.
        if !use_bias {
            let mut changed = false;
            for (r, out) in group.rows.iter().zip(group.out.iter_mut()) {
                if let RowSource::Lock(l, axis) = r.source
                    && locks[l].pulling[axis]
                    && !*out
                {
                    *out = true;
                    changed = true;
                }
            }
            if changed {
                group.factorize();
            }
        }
        loop {
            let x = group.impulses(sb, use_bias);
            // The row's accumulated impulse so far.
            let so_far = |r: &GroupRow| match r.source {
                RowSource::Anchor(j, axis) => joints[j].point_impulse[axis],
                RowSource::Lock(l, axis) => locks[l].impulse[axis],
                RowSource::Contact(c) => contacts[c].impulse[0],
            };
            let over = group.rows.iter().zip(&x).position(|(r, &dx)| {
                let total = so_far(r) + dx as f32;
                !matches!(r.source, RowSource::Anchor(..)) && (total > r.bound || total < r.floor)
            });
            if let Some(i) = over {
                let r = &group.rows[i];
                let before = so_far(r);
                let kept = (before + x[i] as f32).clamp(r.floor, r.bound);
                LockGroup::apply(r, sb, kept - before);
                match r.source {
                    RowSource::Lock(l, axis) => locks[l].impulse[axis] = kept,
                    RowSource::Contact(c) => {
                        contacts[c].impulse[0] = kept;
                        // Not pushing any more: the contact pass has it again.
                        held[c] = false;
                    }
                    RowSource::Anchor(..) => unreachable!(),
                }
                group.out[i] = true;
                group.factorize();
                continue;
            }
            for (r, &dx) in group.rows.iter().zip(&x) {
                let d = dx as f32;
                match r.source {
                    RowSource::Anchor(j, axis) => joints[j].point_impulse[axis] += d,
                    RowSource::Lock(l, axis) => locks[l].impulse[axis] += d,
                    RowSource::Contact(c) => contacts[c].impulse[0] += d,
                }
                LockGroup::apply(r, sb, d);
            }
            break;
        }
    }
}

/// Inverse of a 6x6 matrix by Gauss-Jordan elimination with partial pivoting.
fn invert6(mut m: [[f32; 6]; 6]) -> Option<[[f32; 6]; 6]> {
    let mut inv = [[0.0f32; 6]; 6];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for col in 0..6 {
        let pivot = (col..6).max_by(|&x, &y| m[x][col].abs().total_cmp(&m[y][col].abs()))?;
        if m[pivot][col].abs() < 1e-12 {
            return None;
        }
        m.swap(col, pivot);
        inv.swap(col, pivot);
        let scale = 1.0 / m[col][col];
        for c in 0..6 {
            m[col][c] *= scale;
            inv[col][c] *= scale;
        }
        for r in 0..6 {
            if r != col {
                let f = m[r][col];
                if f != 0.0 {
                    for c in 0..6 {
                        m[r][c] -= f * m[col][c];
                        inv[r][c] -= f * inv[col][c];
                    }
                }
            }
        }
    }
    Some(inv)
}

/// Bounces contacts that were approaching fast and are now touching.
fn apply_restitution(sb: &mut [SolverBody], contacts: &mut [ContactRow]) {
    for c in contacts.iter_mut() {
        let e = c.interaction.restitution;
        if e <= 0.0 || c.approach > -RESTITUTION_THRESHOLD || c.impulse[0] <= 0.0 {
            continue;
        }
        let vn = handed_velocity(sb, c.a, c.b, c.ra, c.rb).dot(c.normal);
        let old = c.impulse[0];
        let new = (old - c.normal_mass * (vn + e * c.approach)).max(0.0);
        c.impulse[0] = new;
        apply_impulse(sb, c.a, c.b, c.ra, c.rb, c.normal * (new - old));
    }
}

/// Solves one angular row and returns the impulse it applied. `coupling` removes the
/// part of the row's velocity the joint anchor is about to correct anyway.
#[allow(clippy::too_many_arguments)]
fn solve_angular(
    sb: &mut [SolverBody],
    a: usize,
    b: usize,
    row: &mut AngularRow,
    soft_params: Softness,
    h: f32,
    use_bias: bool,
    coupling: f32,
) -> f32 {
    let (ba, bb) = (get(sb, a), get(sb, b));
    let jv = row.axis.dot(bb.w - ba.w) - coupling;
    let old = row.impulse;
    let impulse = if row.spring {
        -row.mass * (jv + row.value + row.gamma * old)
    } else {
        let (bias, mass_scale, impulse_scale) = if row.value > 0.0 {
            (row.value / h, 1.0, 0.0)
        } else if use_bias {
            (
                (soft_params.bias_rate * row.value).max(-MAX_PUSH_SPEED * 4.0),
                soft_params.mass_scale,
                soft_params.impulse_scale,
            )
        } else {
            (0.0, 1.0, 0.0)
        };
        -row.mass * mass_scale * (jv + bias) - impulse_scale * old
    };
    let new = (old + impulse).clamp(row.lower, row.upper);
    row.impulse = new;
    apply_angular(sb, a, b, row.axis * (new - old));
    new - old
}

fn inverse(k: f32) -> f32 {
    if k > 1e-12 { 1.0 / k } else { 0.0 }
}

/// Mass properties of a set of bodies, summed about the world origin in f64 (positions
/// are metres from the origin, inertias can be grams times square centimetres).
#[derive(Clone, Copy, Default)]
struct Moments {
    mass: f64,
    first: glam::DVec3,
    second: glam::DMat3,
    /// Bodies the solver doesn't move (keyed or massless) in the set.
    fixed: usize,
    /// Bodies resting on the ground or scenery in the set.
    grounded: usize,
}

impl Moments {
    fn of(body: &Body, grounded: bool) -> Self {
        let mut m = Self {
            grounded: grounded as usize,
            ..Self::default()
        };
        if !body.is_dynamic() {
            m.fixed = 1;
            return m;
        }
        let mass = 1.0 / body.inv_mass as f64;
        let x = body.position.as_dvec3();
        let r = glam::DMat3::from_quat(body.rotation.as_dquat());
        let i = body.inv_inertia.as_dvec3().map(|v| 1.0 / v.max(1e-12));
        let own = r * glam::DMat3::from_diagonal(i) * r.transpose();
        m.mass = mass;
        m.first = x * mass;
        m.second =
            own + (glam::DMat3::from_diagonal(glam::DVec3::splat(x.dot(x))) - outer(x, x)) * mass;
        m
    }

    fn add(self, o: Self) -> Self {
        Self {
            mass: self.mass + o.mass,
            first: self.first + o.first,
            second: self.second + o.second,
            fixed: self.fixed + o.fixed,
            grounded: self.grounded + o.grounded,
        }
    }

    fn sub(self, o: Self) -> Self {
        Self {
            mass: self.mass - o.mass,
            first: self.first - o.first,
            second: self.second - o.second,
            fixed: self.fixed - o.fixed,
            grounded: self.grounded - o.grounded,
        }
    }

    fn centre(&self) -> glam::DVec3 {
        if self.mass > 0.0 {
            self.first / self.mass
        } else {
            glam::DVec3::ZERO
        }
    }

    /// Offset from the set's centre of mass to `point`.
    fn lever(&self, point: Vec3) -> Vec3 {
        (point.as_dvec3() - self.centre()).as_vec3()
    }
}

fn outer(a: glam::DVec3, b: glam::DVec3) -> glam::DMat3 {
    glam::DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}

/// A set of bodies as one rigid body, for the spring cap: immovable if it holds a fixed
/// body or `still` says so.
fn limb_body(set: Side) -> SolverBody {
    let m = set.moments;
    if set.still || m.fixed > 0 || m.mass <= 0.0 {
        return SolverBody {
            v: Vec3::ZERO,
            w: Vec3::ZERO,
            inv_mass: 0.0,
            inv_i: Mat3::ZERO,
            carry: 0.0,
        };
    }
    let c = m.centre();
    let about_centre = m.second
        - (glam::DMat3::from_diagonal(glam::DVec3::splat(c.dot(c))) - outer(c, c)) * m.mass;
    SolverBody {
        v: Vec3::ZERO,
        w: Vec3::ZERO,
        inv_mass: (1.0 / m.mass) as f32,
        inv_i: about_centre.inverse().as_mat3(),
        carry: 1.0,
    }
}

/// One side of a joint, for the spring cap.
#[derive(Clone, Copy)]
struct Side {
    moments: Moments,
    still: bool,
}

impl Side {
    fn lever(&self, point: Vec3) -> Vec3 {
        self.moments.lever(point)
    }

    fn as_body(&self) -> SolverBody {
        limb_body(*self)
    }
}

/// For every joint, the bodies hanging from it (its child and everything jointed below
/// that) and the rest of the jointed group it belongs to.
struct Limbs {
    /// Per body: the moments of the body and everything below it.
    below: Vec<Moments>,
    /// Per body: the moments of its whole jointed group.
    group: Vec<Moments>,
}

impl Limbs {
    fn new(world: &World, grounded: &[bool]) -> Self {
        let n = world.bodies.len();
        let own: Vec<Moments> = world
            .bodies
            .iter()
            .zip(grounded)
            .map(|(b, &g)| Moments::of(b, g))
            .collect();
        // Parent body of each body through an enabled joint (a tree per character).
        let mut up = vec![usize::MAX; n];
        for j in world.joints.iter().filter(|j| j.enabled) {
            if up[j.child] == usize::MAX && j.parent != j.child {
                up[j.child] = j.parent;
            }
        }
        // Depth, to sum leaves first; a cycle (not expected) stops at n steps.
        let depth = |mut b: usize| {
            let mut d = 0;
            while up[b] != usize::MAX && d < n {
                b = up[b];
                d += 1;
            }
            (d, b)
        };
        let info: Vec<(usize, usize)> = (0..n).map(depth).collect();
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by_key(|&b| std::cmp::Reverse(info[b].0));
        let mut below = own.clone();
        for &b in &order {
            if up[b] != usize::MAX && info[b].0 > 0 {
                let m = below[b];
                below[up[b]] = below[up[b]].add(m);
            }
        }
        let group = (0..n).map(|b| below[info[b].1]).collect();
        Self { below, group }
    }

    /// The limb hanging from a joint whose child body is `child`, and the rest of its group.
    /// A side resting on the ground or scenery is held there (lesson 9), whichever it is:
    /// a toe and the foot above it both on the ground are held by it, as is a lying
    /// character's limb, so neither needs a soft muscle to stay put.
    fn sides(&self, child: usize) -> (Side, Side) {
        let limb = self.below[child];
        let rest = self.group[child].sub(limb);
        (
            Side {
                moments: limb,
                still: limb.grounded > 0,
            },
            Side {
                moments: rest,
                still: rest.grounded > 0,
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::document::physics::Interaction;

    fn world() -> World {
        let mut w = World::new(Vec3::new(0.0, -9.81, 0.0), ContactModel::ForceCapped);
        w.set_materials(1, |_, _| Interaction {
            restitution: 0.0,
            friction: 100.0,
            adhesion: 0.0,
            softness: 0.0,
        });
        w.ground = Some((0.0, 0));
        w
    }

    fn ball(w: &mut World, at: Vec3, radius: f32, mass: f32) -> usize {
        let i = 0.4 * mass * radius * radius;
        w.bodies.push(Body {
            position: at,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 1.0 / mass,
            inv_inertia: Vec3::splat(1.0 / i),
            motion: Motion::Dynamic,
            group: 0,
            parent: None,
        });
        let b = w.bodies.len() - 1;
        w.shapes.push(Shape {
            body: Some(b),
            geometry: Geometry::Sphere { radius },
            rotation: Quat::IDENTITY,
            position: Vec3::ZERO,
            material: 0,
            flags: CollisionFlags::default(),
        });
        b
    }

    fn run(w: &mut World, seconds: f32) {
        for _ in 0..(seconds * 120.0) as usize {
            w.step(1.0 / 120.0);
        }
    }

    #[test]
    fn free_fall_matches_gravity() {
        let mut w = world();
        w.ground = None;
        let b = ball(&mut w, Vec3::new(0.0, 10.0, 0.0), 0.1, 1.0);
        run(&mut w, 1.0);
        let fallen = 10.0 - w.bodies[b].position.y;
        assert!((fallen - 0.5 * 9.81).abs() < 0.05, "fell {fallen} m");
    }

    #[test]
    fn ball_comes_to_rest_on_ground() {
        let mut w = world();
        let b = ball(&mut w, Vec3::new(0.0, 1.0, 0.0), 0.1, 2.0);
        run(&mut w, 3.0);
        let body = &w.bodies[b];
        assert!(
            (body.position.y - 0.1).abs() < 0.01,
            "rests at {}",
            body.position.y
        );
        assert!(body.linear_velocity.length() < 0.05);
    }

    /// Coulomb friction stops a sliding box in v² / (2 μ g) whatever it weighs: 0.82 m
    /// from 4 m/s at μ = 1 (endorphin's Default, 100). Force-capped friction, 100 N per
    /// contact, stops a light box in a fraction of that and lets a heavy one slide on
    /// (lesson 21).
    #[test]
    fn coulomb_friction_scales_with_the_load() {
        let slide = |model: ContactModel, mass: f32| {
            let mut w = world();
            w.contact_model = model;
            let b = ball(&mut w, Vec3::new(0.0, 0.2, 0.0), 0.2, mass);
            let size = 0.4;
            w.shapes[b].geometry = Geometry::Box { size: [size; 3] };
            w.bodies[b].inv_inertia = Vec3::splat(6.0 / (mass * size * size));
            w.bodies[b].linear_velocity = Vec3::new(4.0, 0.0, 0.0);
            run(&mut w, 1.5);
            w.bodies[b].position.x
        };
        let [light, heavy] = [10.0, 100.0].map(|m| slide(ContactModel::Coulomb, m));
        let [capped_light, capped_heavy] =
            [10.0, 100.0].map(|m| slide(ContactModel::ForceCapped, m));
        println!(
            "slid from 4 m/s: Coulomb {light:.2} m (10 kg), {heavy:.2} m (100 kg); \
             force-capped {capped_light:.2} m, {capped_heavy:.2} m"
        );
        for d in [light, heavy] {
            assert!((d - 0.82).abs() < 0.08, "slid {d} m");
        }
    }

    /// A ball gliding 1 cm above the ground (inside the contact margin, weightless)
    /// isn't touching it, so friction mustn't hold it; one rolling on it still slows.
    #[test]
    fn friction_grips_only_what_touches() {
        let mut w = world();
        w.gravity = Vec3::ZERO;
        let gliding = ball(&mut w, Vec3::new(0.0, 0.11, 0.0), 0.1, 2.0);
        w.bodies[gliding].linear_velocity = Vec3::X;
        run(&mut w, 0.5);
        let glided = w.bodies[gliding].position.x;
        assert!((glided - 0.5).abs() < 0.01, "glided {glided} m");

        let mut w = world();
        let sliding = ball(&mut w, Vec3::new(0.0, 0.1, 0.0), 0.1, 2.0);
        w.bodies[sliding].linear_velocity = Vec3::X;
        run(&mut w, 0.5);
        let slid = w.bodies[sliding].position.x;
        assert!(slid < 0.45, "slid {slid} m on the ground without friction");
    }

    #[test]
    fn restitution_bounces() {
        let mut w = world();
        w.set_materials(1, |_, _| Interaction {
            restitution: 0.8,
            friction: 100.0,
            adhesion: 0.0,
            softness: 0.0,
        });
        let b = ball(&mut w, Vec3::new(0.0, 1.0, 0.0), 0.1, 1.0);
        let mut peak_after_bounce = 0.0f32;
        let mut bounced = false;
        for _ in 0..240 {
            w.step(1.0 / 120.0);
            let body = &w.bodies[b];
            if body.linear_velocity.y > 0.5 {
                bounced = true;
            }
            if bounced {
                peak_after_bounce = peak_after_bounce.max(body.position.y);
            }
        }
        assert!(bounced);
        assert!(peak_after_bounce > 0.4, "bounced to {peak_after_bounce}");
    }

    /// Two balls on a joint: the anchor holds and a stiff muscle holds the pose.
    #[test]
    fn joint_holds_and_muscle_holds_pose() {
        let mut w = world();
        w.ground = None;
        let a = ball(&mut w, Vec3::ZERO, 0.05, 5.0);
        let b = ball(&mut w, Vec3::new(0.5, 0.0, 0.0), 0.05, 1.0);
        w.bodies[a].motion = Motion::Kinematic { momentum: 1.0 };
        w.bodies[b].parent = Some(a);
        let wide = Limits {
            twist_min: -3.0,
            twist_max: 3.0,
            swing1: 3.0,
            swing2: 3.0,
        };
        let mut joint = Joint::new(
            a,
            b,
            Vec3::ZERO,
            Vec3::new(-0.5, 0.0, 0.0),
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            wide,
            wide,
            0.0,
            0.0,
        );
        let gains = Gains {
            stiffness: 2000.0,
            damping: 50.0,
            force_limit: 500.0,
        };
        joint.muscle = Some(Muscle {
            target: Quat::IDENTITY,
            twist: gains,
            swing: gains,
        });
        w.joints.push(joint);
        run(&mut w, 2.0);
        let tip = w.bodies[b].position;
        assert!((tip.length() - 0.5).abs() < 0.01, "anchor drifted: {tip}");
        // Gravity torque 1 kg × 9.81 × 0.5 m ≈ 4.9 N·m sags a 2000 N·m/rad muscle by ~0.15°.
        assert!(tip.y > -0.01, "sagged to {tip}");

        // Without the muscle it swings down (and back up the other side).
        w.joints[0].muscle = None;
        let mut lowest = f32::MAX;
        for _ in 0..120 {
            w.step(1.0 / 120.0);
            lowest = lowest.min(w.bodies[b].position.y);
        }
        assert!(lowest < -0.45, "lowest {lowest}");
    }

    #[test]
    fn force_limit_caps_muscle() {
        let mut w = world();
        w.ground = None;
        let a = ball(&mut w, Vec3::ZERO, 0.05, 5.0);
        let b = ball(&mut w, Vec3::new(0.5, 0.0, 0.0), 0.05, 1.0);
        w.bodies[a].motion = Motion::Kinematic { momentum: 1.0 };
        let wide = Limits {
            twist_min: -3.0,
            twist_max: 3.0,
            swing1: 3.0,
            swing2: 3.0,
        };
        let mut joint = Joint::new(
            a,
            b,
            Vec3::ZERO,
            Vec3::new(-0.5, 0.0, 0.0),
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            wide,
            wide,
            0.0,
            0.0,
        );
        // Very stiff but only 1 N·m: can't hold up a 4.9 N·m load.
        let gains = Gains {
            stiffness: 1e5,
            damping: 100.0,
            force_limit: 1.0,
        };
        joint.muscle = Some(Muscle {
            target: Quat::IDENTITY,
            twist: gains,
            swing: gains,
        });
        w.joints.push(joint);
        let mut lowest = f32::MAX;
        for _ in 0..180 {
            w.step(1.0 / 120.0);
            lowest = lowest.min(w.bodies[b].position.y);
        }
        assert!(lowest < -0.3, "lowest {lowest}");
    }

    /// A virtual muscle pulls a free 1 kg ball 0.5 m to its target beside a fixed
    /// anchor and holds it there against gravity, as a stiff spring; capped at 5 N it
    /// can't hold up the 9.8 N ball, which falls.
    #[test]
    fn virtual_muscle_pulls_to_its_target_within_its_force_limit() {
        let run = |force_limit: f32| {
            let mut w = world();
            w.ground = None;
            let a = ball(&mut w, Vec3::ZERO, 0.05, 5.0);
            let b = ball(&mut w, Vec3::new(0.5, 0.0, 0.0), 0.05, 1.0);
            w.bodies[a].motion = Motion::Kinematic { momentum: 1.0 };
            let target = Vec3::new(0.0, 0.0, 0.5);
            for _ in 0..240 {
                w.virtual_muscles = vec![VirtualMuscle {
                    anchor: a,
                    body: b,
                    point: Vec3::ZERO,
                    target_position: target,
                    target_rotation: Quat::IDENTITY,
                    linear: Some(Gains {
                        stiffness: 2000.0,
                        damping: 100.0,
                        force_limit,
                    }),
                    angular: None,
                }];
                w.step(1.0 / 120.0);
            }
            w.bodies[b].position - target
        };
        let held = run(1000.0);
        assert!(held.length() < 0.01, "{held}");
        let weak = run(5.0);
        assert!(weak.y < -1.0, "{weak}");
    }

    #[test]
    fn twist_limit_stops_rotation() {
        let mut w = world();
        w.ground = None;
        w.gravity = Vec3::ZERO;
        let a = ball(&mut w, Vec3::ZERO, 0.05, 5.0);
        let b = ball(&mut w, Vec3::new(0.5, 0.0, 0.0), 0.05, 1.0);
        w.bodies[a].motion = Motion::Kinematic { momentum: 1.0 };
        let limits = Limits {
            twist_min: -0.5,
            twist_max: 0.5,
            swing1: 1.0,
            swing2: 1.0,
        };
        w.joints.push(Joint::new(
            a,
            b,
            Vec3::ZERO,
            Vec3::new(-0.5, 0.0, 0.0),
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            limits,
            limits,
            0.0,
            0.0,
        ));
        w.bodies[b].angular_velocity = Vec3::new(5.0, 0.0, 0.0);
        run(&mut w, 1.0);
        let (twist, swing) = swing_twist(w.bodies[b].rotation);
        assert!(twist.abs() < 0.55, "twisted to {twist}");
        assert!(swing.length() < 0.05);
    }

    #[test]
    fn swing_cone_stops_bending() {
        let mut w = world();
        w.ground = None;
        let a = ball(&mut w, Vec3::ZERO, 0.05, 5.0);
        let b = ball(&mut w, Vec3::new(0.5, 0.0, 0.0), 0.05, 1.0);
        w.bodies[a].motion = Motion::Kinematic { momentum: 1.0 };
        // Gravity bends the arm about Z; the cone allows 0.3 rad.
        let limits = Limits {
            twist_min: -1.0,
            twist_max: 1.0,
            swing1: 0.3,
            swing2: 0.3,
        };
        w.joints.push(Joint::new(
            a,
            b,
            Vec3::ZERO,
            Vec3::new(-0.5, 0.0, 0.0),
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            limits,
            limits,
            0.0,
            0.0,
        ));
        run(&mut w, 2.0);
        let (_, swing) = swing_twist(w.bodies[b].rotation);
        assert!(swing.length() < 0.36, "swung {}", swing.length());
    }

    #[test]
    fn muscle_targets_are_clamped_into_limits() {
        let limits = Limits {
            twist_min: -0.5,
            twist_max: 0.5,
            swing1: 0.3,
            swing2: 0.3,
        };
        let joint = Joint::new(
            0,
            1,
            Vec3::ZERO,
            Vec3::ZERO,
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            limits,
            limits,
            0.0,
            0.0,
        );
        let inside = Quat::from_rotation_x(0.2);
        assert_eq!(joint.clamp_to_limits(inside), inside);
        let (twist, _) = swing_twist(joint.clamp_to_limits(Quat::from_rotation_x(1.2)));
        assert!((twist - 0.5).abs() < 1e-4);
        let (_, swing) = swing_twist(joint.clamp_to_limits(Quat::from_rotation_z(1.0)));
        assert!((swing.length() - 0.3).abs() < 1e-4);
    }

    #[test]
    fn lock_holds_a_body_in_place_and_leaves_free_axes_free() {
        let mut w = world();
        w.ground = None;
        let b = ball(&mut w, Vec3::new(0.0, 2.0, 0.0), 0.1, 3.0);
        // Held in Y only: it should stay up but drift sideways when pushed.
        w.locks.push(Lock {
            anchor: None,
            body: b,
            target: (Quat::IDENTITY, Vec3::new(0.0, 2.0, 0.0)),
            linear: Vec3::new(0.0, 1e5, 0.0),
            angular: Vec3::ZERO,
        });
        w.bodies[b].linear_velocity = Vec3::new(1.0, 0.0, 0.0);
        run(&mut w, 1.0);
        let p = w.bodies[b].position;
        assert!((p.y - 2.0).abs() < 0.01, "sagged to {p}");
        assert!(p.x > 0.8, "free axis was held: {p}");
    }

    /// A lock's strength caps its force: 50 N can't hold up a 10 kg ball.
    #[test]
    fn lock_force_is_capped_by_strength() {
        let mut w = world();
        w.ground = None;
        let b = ball(&mut w, Vec3::new(0.0, 2.0, 0.0), 0.1, 10.0);
        w.locks.push(Lock {
            anchor: None,
            body: b,
            target: (Quat::IDENTITY, Vec3::new(0.0, 2.0, 0.0)),
            linear: Vec3::new(0.0, 50.0, 0.0),
            angular: Vec3::ZERO,
        });
        run(&mut w, 1.0);
        // Net acceleration (98.1 - 50) N / 10 kg ≈ 4.8 m/s², so ≈ 2.4 m in a second.
        let fallen = 2.0 - w.bodies[b].position.y;
        assert!((fallen - 2.4).abs() < 0.1, "fell {fallen} m");
    }

    /// A lock that has fallen behind pulls with its full strength until it catches up, as
    /// a force-capped constraint would, and brakes in time not to swing past (lesson 20).
    /// 50 N on a weightless 10 kg ball held 1 m from its target: 5 m/s² to half way and
    /// back down, 0.9 s in all. It used to creep in at strength × substep / mass, 1 cm/s.
    #[test]
    fn a_lock_that_falls_behind_pulls_with_its_full_strength() {
        let mut w = world();
        w.ground = None;
        w.gravity = Vec3::ZERO;
        let b = ball(&mut w, Vec3::ZERO, 0.1, 10.0);
        w.locks.push(Lock {
            anchor: None,
            body: b,
            target: (Quat::IDENTITY, Vec3::new(1.0, 0.0, 0.0)),
            linear: Vec3::splat(50.0),
            angular: Vec3::ZERO,
        });
        run(&mut w, 0.5);
        let half = w.bodies[b].position.x;
        run(&mut w, 0.5);
        let gap = (1.0 - w.bodies[b].position.x).abs();
        assert!(half > 0.5, "moved {half} m in 0.5 s");
        assert!(gap < 0.01, "{gap} m from the target after 1 s");
    }

    /// A negative linear strength pushes with that many newtons: -10 N down on 2 kg adds
    /// 5 m/s² to gravity.
    #[test]
    fn negative_linear_strength_pushes_with_a_steady_force() {
        let mut w = world();
        w.ground = None;
        let b = ball(&mut w, Vec3::new(0.0, 10.0, 0.0), 0.1, 2.0);
        w.locks.push(Lock {
            anchor: None,
            body: b,
            target: (Quat::IDENTITY, Vec3::new(0.0, 10.0, 0.0)),
            linear: Vec3::new(0.0, -10.0, 0.0),
            angular: Vec3::ZERO,
        });
        run(&mut w, 1.0);
        let v = w.bodies[b].linear_velocity;
        let expected = -9.81 - 10.0 / 2.0;
        assert!(
            (v.y - expected).abs() < 1e-3,
            "{v}, expected {expected} m/s down"
        );
        assert!(v.x.abs() < 1e-6 && v.z.abs() < 1e-6, "{v}");
    }

    /// A negative angular strength turns with that many newton-metres about the world
    /// axis: -100 N·m about X spins a ball of 1 kg·m² at -100 rad/s².
    #[test]
    fn negative_angular_strength_turns_with_a_steady_torque() {
        let mut w = world();
        w.ground = None;
        let b = ball(&mut w, Vec3::new(0.0, 10.0, 0.0), 0.5, 10.0);
        let inertia = 0.4 * 10.0 * 0.5 * 0.5;
        w.locks.push(Lock {
            anchor: None,
            body: b,
            target: (Quat::IDENTITY, Vec3::new(0.0, 10.0, 0.0)),
            linear: Vec3::ZERO,
            angular: Vec3::new(-100.0, 0.0, 0.0),
        });
        run(&mut w, 0.2);
        let spin = w.bodies[b].angular_velocity;
        let expected = -100.0 * 0.2 / inertia;
        assert!(
            (spin.x - expected).abs() < 1e-3,
            "{spin}, expected {expected} rad/s about X"
        );
        assert!(spin.y.abs() < 1e-5 && spin.z.abs() < 1e-5, "{spin}");
        // Half of -100 rad/s² times (0.2 s)²: -2 rad about X.
        let off = w.bodies[b]
            .rotation
            .angle_between(Quat::from_rotation_x(-2.0));
        assert!(off < 0.05, "{off} rad away from a -2 rad turn about X");
    }

    /// Driving axes and holding axes on one lock: X and Y-rotation hold, the rest push.
    /// The ball stays at x = 0 and stops turning about Y, while it falls faster than
    /// gravity and spins about X.
    #[test]
    fn negative_axes_drive_while_positive_axes_of_the_same_lock_hold() {
        let mut w = world();
        w.ground = None;
        let b = ball(&mut w, Vec3::new(0.0, 10.0, 0.0), 0.5, 10.0);
        w.bodies[b].linear_velocity = Vec3::new(1.0, 0.0, 0.0);
        w.bodies[b].angular_velocity = Vec3::new(0.0, 2.0, 0.0);
        w.locks.push(Lock {
            anchor: None,
            body: b,
            target: (Quat::IDENTITY, Vec3::new(0.0, 10.0, 0.0)),
            linear: Vec3::new(1e5, -20.0, 0.0),
            angular: Vec3::new(-100.0, 1e5, 0.0),
        });
        run(&mut w, 0.5);
        let body = &w.bodies[b];
        assert!(
            body.position.x.abs() < 0.01,
            "held axis drifted: {}",
            body.position
        );
        assert!(
            (body.linear_velocity.y - (-9.81 - 2.0) * 0.5).abs() < 1e-2,
            "{}",
            body.linear_velocity
        );
        assert!(
            body.angular_velocity.y.abs() < 0.05,
            "held rotation still turning: {}",
            body.angular_velocity
        );
        assert!(
            (body.angular_velocity.x + 50.0).abs() < 0.5,
            "{}",
            body.angular_velocity
        );
    }

    /// A join's negative axis pushes the body and its anchor apart with equal and
    /// opposite force, along the anchor's own axis (here turned to world Y), so their
    /// momentum stays what it was. Its positive axes keep holding.
    #[test]
    fn negative_join_axis_pushes_body_and_anchor_apart_equally() {
        let mut w = world();
        w.ground = None;
        w.gravity = Vec3::ZERO;
        let anchor = ball(&mut w, Vec3::new(0.0, 5.0, 0.0), 0.1, 2.0);
        let body = ball(&mut w, Vec3::new(0.0, 4.5, 0.0), 0.1, 5.0);
        let turn = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
        w.bodies[anchor].rotation = turn;
        // The anchor's X is world Y: the push on the body is down, away from the anchor.
        let offset = turn.inverse() * (w.bodies[body].position - w.bodies[anchor].position);
        w.locks.push(Lock {
            anchor: Some(anchor),
            body,
            target: (Quat::IDENTITY, offset),
            linear: Vec3::new(-10.0, 1e5, 1e5),
            angular: Vec3::ZERO,
        });
        run(&mut w, 0.5);
        let (va, vb) = (
            w.bodies[anchor].linear_velocity,
            w.bodies[body].linear_velocity,
        );
        assert!((vb.y - (-10.0 * 0.5 / 5.0)).abs() < 1e-3, "body {vb}");
        assert!((va.y - (10.0 * 0.5 / 2.0)).abs() < 1e-3, "anchor {va}");
        let momentum = va * 2.0 + vb * 5.0;
        assert!(momentum.length() < 1e-4, "momentum {momentum}");

        // Knocked sideways, the held axes drag the anchor along and turn it while the
        // body slides metres along the drive: still no momentum gained or lost, and no
        // gap across the anchor's Y and Z.
        w.bodies[body].linear_velocity.x += 1.0;
        let before = w.bodies[anchor].linear_velocity * 2.0 + w.bodies[body].linear_velocity * 5.0;
        run(&mut w, 0.5);
        let (a, b) = (&w.bodies[anchor], &w.bodies[body]);
        let after = a.linear_velocity * 2.0 + b.linear_velocity * 5.0;
        assert!(
            (after - before).length() < 1e-3,
            "momentum {before} -> {after}"
        );
        let gap = a.rotation.inverse() * (b.position - a.position) - offset;
        assert!(
            gap.y.abs() < 0.01 && gap.z.abs() < 0.01,
            "held axes opened {gap}"
        );
        assert!(gap.x < -0.5, "drive axis didn't push apart: {gap}");
    }

    /// A light body joined to a moving anchor holds a heavy load hanging from it by a
    /// joint, and follows the anchor.
    #[test]
    fn join_on_a_light_body_holds_a_heavy_load() {
        let mut w = world();
        w.ground = None;
        let anchor = ball(&mut w, Vec3::new(0.0, 2.0, 0.0), 0.1, 5.0);
        w.bodies[anchor].motion = Motion::Kinematic { momentum: 0.0 };
        w.bodies[anchor].linear_velocity = Vec3::new(1.0, 0.0, 0.0);
        let handle = ball(&mut w, Vec3::new(0.0, 1.9, 0.0), 0.02, 0.05);
        let load = ball(&mut w, Vec3::new(0.0, 1.4, 0.0), 0.2, 50.0);
        let free = Limits {
            twist_min: -3.0,
            twist_max: 3.0,
            swing1: 3.0,
            swing2: 3.0,
        };
        w.joints.push(Joint::new(
            handle,
            load,
            Vec3::ZERO,
            Vec3::new(0.0, 0.5, 0.0),
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            Quat::IDENTITY,
            free,
            free,
            0.0,
            0.0,
        ));
        w.locks.push(Lock {
            anchor: Some(anchor),
            body: handle,
            target: (Quat::IDENTITY, Vec3::new(0.0, -0.1, 0.0)),
            linear: Vec3::splat(1e5),
            angular: Vec3::splat(1e5),
        });
        run(&mut w, 1.0);
        let gap =
            w.bodies[handle].position - (w.bodies[anchor].position + Vec3::new(0.0, -0.1, 0.0));
        assert!(gap.length() < 0.01, "handle drifted {gap}");
        assert!((w.bodies[anchor].position.x - 1.0).abs() < 1e-3);
    }

    /// endorphin's Trapeze: a light bar joined to a light rope end that is hinged 6 m
    /// away, on a lever far beyond the rope end's own inertia. Solving the two locks
    /// together must neither stiffen nor feed the swing: the bar moves like a pendulum.
    #[test]
    fn bar_joined_far_from_its_hinge_swings_like_a_pendulum() {
        let mut w = world();
        w.ground = None;
        let (length, angle) = (6.0f32, 1.0f32);
        let pivot = Vec3::new(0.0, 10.0, 0.0);
        let top = ball(&mut w, pivot, 0.03, 2.7);
        let bar = ball(
            &mut w,
            pivot + Vec3::new(0.0, -angle.cos(), angle.sin()) * length,
            0.03,
            2.7,
        );
        // As the scene lists them: the join, then the hinge.
        let offset = w.bodies[top].position - w.bodies[bar].position;
        w.locks.push(Lock {
            anchor: Some(bar),
            body: top,
            target: (Quat::IDENTITY, offset),
            linear: Vec3::splat(1e5),
            angular: Vec3::splat(1e5),
        });
        w.locks.push(Lock {
            anchor: None,
            body: top,
            target: (Quat::IDENTITY, pivot),
            linear: Vec3::splat(1e5),
            angular: Vec3::new(0.0, 1e5, 1e5),
        });
        let mut fastest = 0.0f32;
        for _ in 0..240 {
            w.step(1.0 / 120.0);
            fastest = fastest.max(w.bodies[bar].linear_velocity.length());
        }
        let expected = (2.0 * 9.81 * length * (1.0 - angle.cos())).sqrt();
        assert!(
            (fastest - expected).abs() < 0.05 * expected,
            "fastest {fastest}, a pendulum reaches {expected}"
        );
        assert!(
            w.bodies[top].position.distance(pivot) < 0.01,
            "hinge moved to {}",
            w.bodies[top].position
        );
        assert!(
            (w.bodies[bar].position.distance(pivot) - length).abs() < 0.01,
            "bar at {}",
            w.bodies[bar].position.distance(pivot)
        );
    }

    /// endorphin's Walk With Gun: a 550 kg prop joined to both hands closes a loop
    /// through two arms of half-kilo to two-kilo bodies. Solved one constraint at a time
    /// that tore the arms apart; the arms must carry it (here as it swings down).
    #[test]
    fn heavy_prop_held_in_both_hands_keeps_the_arms_together() {
        let mut w = world();
        w.ground = None;
        let free = Limits {
            twist_min: -3.0,
            twist_max: 3.0,
            swing1: 3.0,
            swing2: 3.0,
        };
        let torso = ball(&mut w, Vec3::new(0.0, 1.5, 0.0), 0.1, 30.0);
        w.bodies[torso].motion = Motion::Kinematic { momentum: 0.0 };
        let joint = |w: &mut World, parent: usize, at_parent: Vec3, at: Vec3, mass: f32| {
            let child = ball(w, at, 0.04, mass);
            w.bodies[child].group = child;
            let anchor = w.bodies[parent].position + at_parent;
            w.joints.push(Joint::new(
                parent,
                child,
                at_parent,
                anchor - at,
                Quat::IDENTITY,
                Quat::IDENTITY,
                Quat::IDENTITY,
                Quat::IDENTITY,
                free,
                free,
                0.0,
                0.0,
            ));
            child
        };
        let mut hands = Vec::new();
        for side in [-1.0f32, 1.0] {
            let x = 0.2 * side;
            let upper = joint(
                &mut w,
                torso,
                Vec3::new(x, 0.0, 0.0),
                Vec3::new(x, 1.5, 0.15),
                2.0,
            );
            let fore = joint(
                &mut w,
                upper,
                Vec3::new(0.0, 0.0, 0.15),
                Vec3::new(x, 1.5, 0.45),
                1.5,
            );
            hands.push(joint(
                &mut w,
                fore,
                Vec3::new(0.0, 0.0, 0.15),
                Vec3::new(x, 1.5, 0.65),
                0.5,
            ));
        }
        let gun = ball(&mut w, Vec3::new(0.0, 1.5, 0.8), 0.15, 550.0);
        w.bodies[gun].group = gun;
        // JoinBodies [gun, right hand, left hand]: each held to the last.
        let left = hands[0];
        for body in [gun, hands[1]] {
            let offset = w.bodies[body].position - w.bodies[left].position;
            w.locks.push(Lock {
                anchor: Some(left),
                body,
                target: (Quat::IDENTITY, offset),
                linear: Vec3::splat(1e5),
                angular: Vec3::splat(1e4),
            });
        }
        let gap = |w: &World| {
            w.joints
                .iter()
                .map(|j| {
                    let (a, b) = (&w.bodies[j.parent], &w.bodies[j.child]);
                    (a.position + a.rotation * j.anchor_parent)
                        .distance(b.position + b.rotation * j.anchor_child)
                })
                .fold(0.0f32, f32::max)
        };
        let mut widest = 0.0f32;
        for _ in 0..240 {
            w.step(1.0 / 120.0);
            widest = widest.max(gap(&w));
        }
        assert!(widest < 0.02, "a joint opened by {widest} m");
        let held = w.bodies[gun].position - w.bodies[left].position;
        assert!(
            held.is_finite() && held.length() < 1.0,
            "gun came loose: {held}"
        );
    }

    /// A hand a lock holds to a 550 kg prop, with its forearm pulled hard against it.
    /// The forearm's contact pushes the hand about as the half kilo it sees, and with
    /// the contacts solved last the lock, which holds it easily, was left short by
    /// whatever they did: in Walk With Gun the hand turned 20° away from the prop. The
    /// lock's direct solve now has the last word (lesson 25).
    #[test]
    fn contacts_do_not_pull_a_hand_off_the_prop_it_is_held_to() {
        for pull in [0.8f32, 1.0, 1.25] {
            let mut w = world();
            let free = Limits {
                twist_min: -3.0,
                twist_max: 3.0,
                swing1: 3.0,
                swing2: 3.0,
            };
            let gun = ball(&mut w, Vec3::new(0.0, 0.15, 0.0), 0.15, 550.0);
            let hand = ball(&mut w, Vec3::new(0.3, 0.15, 0.0), 0.05, 0.5);
            let fore = ball(&mut w, Vec3::new(0.36, 0.2, 0.0), 0.05, 0.7);
            for (group, body) in [gun, hand, fore].into_iter().enumerate() {
                w.bodies[body].group = group;
            }
            // The wrist: a ball joint between them, nearer than their shapes allow.
            let wrist = Vec3::new(0.33, 0.175, 0.0);
            w.joints.push(Joint::new(
                hand,
                fore,
                wrist - w.bodies[hand].position,
                wrist - w.bodies[fore].position,
                Quat::IDENTITY,
                Quat::IDENTITY,
                Quat::IDENTITY,
                Quat::IDENTITY,
                free,
                free,
                0.0,
                0.0,
            ));
            w.bodies[fore].parent = Some(hand);
            for shape in w.shapes.iter_mut().filter(|s| s.body != Some(gun)) {
                shape.flags.parent = true;
            }
            // JoinBodies: the hand held to the prop, position and angle.
            w.locks.push(Lock {
                anchor: Some(gun),
                body: hand,
                target: (Quat::IDENTITY, Vec3::new(0.3, 0.0, 0.0)),
                linear: Vec3::splat(1e5),
                angular: Vec3::splat(1e4),
            });
            // The rest of the arm pulls the forearm in: 2.5 kN along the line between
            // them (a negative strength drives).
            w.locks.push(Lock {
                anchor: None,
                body: fore,
                target: (Quat::IDENTITY, Vec3::ZERO),
                linear: Vec3::new(-1500.0, -2000.0, 0.0) * pull,
                angular: Vec3::ZERO,
            });
            let (mut off, mut turn) = (0.0f32, 0.0f32);
            for _ in 0..240 {
                w.step(1.0 / 120.0);
                // As the prop sees it: it may roll about.
                let (g, h) = (&w.bodies[gun], &w.bodies[hand]);
                turn = turn.max(
                    (g.rotation.inverse() * h.rotation)
                        .to_scaled_axis()
                        .length(),
                );
                off = off.max(
                    (g.rotation.inverse() * (h.position - g.position) - Vec3::new(0.3, 0.0, 0.0))
                        .length(),
                );
            }
            println!(
                "pull x{pull}: hand {:.2} cm off the prop, turned {:.1}°",
                off * 100.0,
                turn.to_degrees()
            );
            assert!(
                off < 0.005,
                "the hand came {off} m off the prop (pull x{pull})"
            );
            assert!(turn < 0.05, "the hand turned {turn} rad (pull x{pull})");
        }
    }

    /// Collision-only scenery moving into a body pushes it along without launching it,
    /// and with momentum it hands its velocity on.
    #[test]
    fn moving_scenery_pushes_with_its_momentum() {
        for momentum in [0.0, 1.0] {
            let mut w = world();
            w.ground = None;
            w.gravity = Vec3::ZERO;
            let wall = ball(&mut w, Vec3::ZERO, 0.2, 100.0);
            w.bodies[wall].motion = Motion::Kinematic { momentum };
            w.bodies[wall].linear_velocity = Vec3::new(5.0, 0.0, 0.0);
            let b = ball(&mut w, Vec3::new(0.35, 0.0, 0.0), 0.1, 1.0);
            run(&mut w, 0.2);
            let (wall, body) = (&w.bodies[wall], &w.bodies[b]);
            // Touching at 0.3 m; soft contacts let a pushed body overlap a little.
            assert!(
                body.position.x > wall.position.x + 0.27,
                "passed through at momentum {momentum}"
            );
            let speed = body.linear_velocity.x;
            if momentum == 0.0 {
                assert!(speed < 0.5, "launched at {speed} m/s");
            } else {
                assert!(speed > 4.5, "only got {speed} m/s");
            }
        }
    }

    #[test]
    fn massless_bodies_stay_put_and_hold_things_up() {
        let mut w = world();
        w.ground = None;
        let table = ball(&mut w, Vec3::ZERO, 0.5, 1.0);
        w.bodies[table].inv_mass = 0.0;
        w.bodies[table].inv_inertia = Vec3::ZERO;
        let cup = ball(&mut w, Vec3::new(0.0, 0.8, 0.0), 0.1, 0.3);
        w.bodies[cup].group = 1;
        run(&mut w, 2.0);
        assert_eq!(
            w.bodies[table].position,
            Vec3::ZERO,
            "a massless body must not move"
        );
        let rest = w.bodies[cup].position.y;
        assert!(
            (rest - 0.6).abs() < 0.01,
            "should rest on the fixed body, at {rest}"
        );
    }

    #[test]
    fn simulation_is_deterministic() {
        let make = || {
            let mut w = world();
            for i in 0..5 {
                ball(
                    &mut w,
                    Vec3::new(i as f32 * 0.15, 0.5 + i as f32 * 0.3, 0.0),
                    0.1,
                    1.0,
                );
            }
            run(&mut w, 2.0);
            w.bodies.iter().map(|b| b.position).collect::<Vec<_>>()
        };
        assert_eq!(make(), make());
    }
}

#[cfg(test)]
mod chain_tests {
    use super::*;
    use crate::document::physics::Interaction;

    /// Forearm → hand → fingers style chain: light bodies, tiny inertias, tight stops,
    /// stiff force-limited muscles holding the rest pose under gravity.
    fn arm(tight: bool, stiffness: f32) -> f32 {
        let mut w = World::new(Vec3::new(0.0, -9.81, 0.0), ContactModel::ForceCapped);
        w.set_materials(1, |_, _| Interaction {
            restitution: 0.2,
            friction: 100.0,
            adhesion: 0.0,
            softness: 0.0,
        });
        let specs = [
            (5.0, 0.05, 0.0),
            (0.7, 0.03, 0.27),
            (0.5, 0.025, 0.13),
            (0.4, 0.02, 0.09),
        ];
        let mut x = 0.0;
        for (i, (mass, r, len)) in specs.iter().enumerate() {
            x += len;
            let inertia = 0.4 * mass * r * r;
            w.bodies.push(Body {
                position: Vec3::new(x, 1.0, 0.0),
                rotation: Quat::IDENTITY,
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
                inv_mass: 1.0 / mass,
                inv_inertia: Vec3::splat(1.0 / inertia),
                motion: if i == 0 {
                    Motion::Kinematic { momentum: 1.0 }
                } else {
                    Motion::Dynamic
                },
                group: 0,
                parent: if i == 0 { None } else { Some(i - 1) },
            });
        }
        for i in 1..specs.len() {
            let len = specs[i].2;
            let limits = if tight {
                Limits {
                    twist_min: -0.03,
                    twist_max: 0.03,
                    swing1: 0.01,
                    swing2: 0.5,
                }
            } else {
                Limits {
                    twist_min: -1.0,
                    twist_max: 1.0,
                    swing1: 1.0,
                    swing2: 1.0,
                }
            };
            let mut j = Joint::new(
                i - 1,
                i,
                Vec3::ZERO,
                Vec3::new(-len, 0.0, 0.0),
                Quat::IDENTITY,
                Quat::IDENTITY,
                Quat::IDENTITY,
                Quat::IDENTITY,
                limits,
                limits,
                10.0,
                10.0,
            );
            let g = Gains {
                stiffness,
                damping: stiffness * 0.5,
                force_limit: stiffness,
            };
            j.muscle = Some(Muscle {
                target: Quat::IDENTITY,
                twist: g,
                swing: g,
            });
            w.joints.push(j);
        }
        let mut fastest = 0.0f32;
        for _ in 0..240 {
            w.step(1.0 / 120.0);
            for b in &w.bodies[1..] {
                fastest = fastest.max(b.linear_velocity.length());
            }
        }
        fastest
    }

    /// Light links hanging straight down under a heavy load: every link stretches and
    /// squeezes in step if joint warm starts overshoot.
    #[test]
    fn loaded_vertical_chain_stays_still() {
        for load in [10.0, 80.0] {
            let mut w = World::new(Vec3::new(0.0, -9.81, 0.0), ContactModel::ForceCapped);
            w.set_materials(1, |_, _| Interaction {
                restitution: 0.0,
                friction: 100.0,
                adhesion: 0.0,
                softness: 0.0,
            });
            let free = Limits {
                twist_min: -0.5,
                twist_max: 0.5,
                swing1: 1.2,
                swing2: 1.2,
            };
            for k in 0..8usize {
                let mass = if k == 7 { load } else { 0.1 };
                let inertia = 0.4 * mass * 0.09 * 0.09;
                w.bodies.push(Body {
                    position: Vec3::new(0.0, 5.0 - 0.2 * k as f32, 0.0),
                    rotation: Quat::IDENTITY,
                    linear_velocity: Vec3::ZERO,
                    angular_velocity: Vec3::ZERO,
                    inv_mass: 1.0 / mass,
                    inv_inertia: Vec3::splat(1.0 / inertia),
                    motion: if k == 0 {
                        Motion::Kinematic { momentum: 0.0 }
                    } else {
                        Motion::Dynamic
                    },
                    group: 0,
                    parent: k.checked_sub(1),
                });
                if k > 0 {
                    w.joints.push(Joint::new(
                        k - 1,
                        k,
                        Vec3::new(0.0, -0.1, 0.0),
                        Vec3::new(0.0, 0.1, 0.0),
                        Quat::IDENTITY,
                        Quat::IDENTITY,
                        Quat::IDENTITY,
                        Quat::IDENTITY,
                        free,
                        free,
                        0.0,
                        0.0,
                    ));
                }
            }
            let mut fastest = 0.0f32;
            for _ in 0..360 {
                w.step(1.0 / 120.0);
                fastest = w
                    .bodies
                    .iter()
                    .map(|b| b.linear_velocity.length())
                    .fold(fastest, f32::max);
            }
            let bottom = w.bodies[7].position.y;
            assert!(
                fastest < 1.0 && (bottom - 3.6).abs() < 0.1,
                "load {load}: fastest {fastest}, bottom at {bottom}"
            );
        }
    }

    #[test]
    fn light_chain_with_stiff_muscles_and_tight_stops_is_stable() {
        for (tight, k) in [
            (false, 1000.0),
            (true, 50.0),
            (true, 1000.0),
            (true, 5000.0),
        ] {
            let fastest = arm(tight, k);
            eprintln!("tight stops {tight}, stiffness {k}: fastest {fastest:.2} m/s");
            assert!(
                fastest < 2.0,
                "chain blew up (tight {tight}, k {k}): {fastest} m/s"
            );
        }
    }
}
