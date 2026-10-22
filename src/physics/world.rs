//! Rigid bodies, joints, muscles and contacts, advanced by a substepped
//! sequential-impulse solver with soft constraints (see docs/physics-design.md).
//!
//! Each substep: gravity → contacts → warm start → solve with position bias →
//! integrate positions → relax without bias. Joints keep their impulses between
//! substeps; contacts are matched by (shape, shape, feature) and locks by their bodies
//! to warm start. Bodies that locks tie together are also solved directly, as one
//! system per group, each iteration (see `LockGroup`).

use std::collections::HashMap;

use glam::{Mat3, Quat, Vec3};

use super::collide::{self, Placed};
use crate::document::physics::{ContactModel, Gains, Geometry, Interaction, Limits};

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

/// Scales stiffness and damping down together so the spring's natural frequency on an
/// effective inverse mass `inv_mass` stays under `MAX_SPRING_HERTZ`.
fn cap_spring(stiffness: f32, damping: f32, inv_mass: f32) -> (f32, f32) {
    let omega = 2.0 * std::f32::consts::PI * MAX_SPRING_HERTZ;
    let max_stiffness = omega * omega / inv_mass.max(1e-12);
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
const STATIC: usize = usize::MAX;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Motion {
    Dynamic,
    /// Moved by the caller (its velocity says how); collides. `momentum` scales the
    /// velocity it hands to what it hits: at 0 it pushes and carries things without
    /// setting them moving.
    Kinematic { momentum: f32 },
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

pub struct World {
    pub bodies: Vec<Body>,
    pub shapes: Vec<Shape>,
    pub joints: Vec<Joint>,
    pub locks: Vec<Lock>,
    pub gravity: Vec3,
    /// Ground plane height and material.
    pub ground: Option<(f32, usize)>,
    pub contact_model: ContactModel,
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
    Softness { bias_rate: omega / a1, mass_scale: a2 * a3, impulse_scale: a3 }
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
    /// Held point on A relative to A's centre; on B it's B's centre.
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
    key: (usize, usize, usize),
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
    /// Impulse cap (locks); infinite for anchors.
    bound: f32,
    source: RowSource,
}

#[derive(Clone, Copy)]
enum RowSource {
    /// Joint rows index and world axis.
    Anchor(usize, usize),
    /// Lock block index and lock axis.
    Lock(usize, usize),
}

/// Bodies held together by locks, with every joint between them, solved as one linear
/// system. Sequential impulses converge slowly around loops and long levers: a gun
/// joined to both hands closes a loop through the arms, and a trapeze bar joined to a
/// rope hinged 6 m away swings on a lever hundreds of times its own inertia. Solving
/// those rows together, directly, after the per-joint and per-lock passes each
/// iteration, handles both.
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
    Mat3::from_cols(Vec3::new(0.0, v.z, -v.y), Vec3::new(-v.z, 0.0, v.x), Vec3::new(v.y, -v.x, 0.0))
}

fn get(sb: &[SolverBody], i: usize) -> SolverBody {
    if i == STATIC {
        SolverBody { v: Vec3::ZERO, w: Vec3::ZERO, inv_mass: 0.0, inv_i: Mat3::ZERO, carry: 0.0 }
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
    let t = if n.x.abs() > 0.57 { Vec3::new(n.y, -n.x, 0.0) } else { Vec3::new(0.0, n.z, -n.y) };
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
    let angle = (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
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
            gravity,
            ground: None,
            contact_model,
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
        self.interactions.get(a.min(n - 1) * n + b.min(n - 1)).copied().unwrap_or(Interaction {
            restitution: 0.2,
            friction: 100.0,
            adhesion: 0.0,
            softness: 0.0,
        })
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
        for body in self.bodies.iter_mut().filter(|b| b.is_dynamic()) {
            body.linear_velocity += self.gravity * h;
        }
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
                SolverBody { v: b.linear_velocity, w: b.angular_velocity, inv_mass, inv_i, carry }
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

        // Warm start.
        for c in &contacts {
            let p = c.normal * c.impulse[0] + c.tangents[0] * c.impulse[1] + c.tangents[1] * c.impulse[2];
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
        let joint_soft = Softness { bias_rate: JOINT_CORRECTION / h, mass_scale: 1.0, impulse_scale: 0.0 };
        let mut groups = self.prepare_groups(&sb, &joints, &locks, joint_soft.bias_rate);
        for _ in 0..ITERATIONS {
            self.solve_joints(&mut sb, &mut joints, joint_soft, h, true);
            solve_locks(&mut sb, &mut locks, joint_soft.bias_rate);
            solve_groups(&mut sb, &mut groups, &mut joints, &mut locks, true);
            self.solve_contacts(&mut sb, &mut contacts, contact_soft, h, true);
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
            self.solve_joints(&mut sb, &mut joints, joint_soft, h, false);
            solve_locks(&mut sb, &mut locks, 0.0);
            solve_groups(&mut sb, &mut groups, &mut joints, &mut locks, false);
            self.solve_contacts(&mut sb, &mut contacts, contact_soft, h, false);
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
            joint.warm_muscle = j.muscle.as_ref().map_or([0.0; 3], |m| (m.frame.transpose() * m.impulse).to_array());
        }
        self.lock_cache.clear();
        for l in &locks {
            self.lock_cache.insert(l.key, l.impulse);
        }
    }

    /// Drops warm-start impulses of bodies that just became dynamic or stopped being so.
    /// What a moved body pushed with (against something it was crushing, say) would hit a
    /// light body like a hammer once it is simulated.
    fn forget_switched_bodies(&mut self) {
        let dynamic: Vec<bool> = self.bodies.iter().map(Body::is_dynamic).collect();
        let switched: Vec<bool> = (0..dynamic.len()).map(|i| self.was_dynamic.get(i).is_some_and(|&d| d != dynamic[i])).collect();
        if switched.contains(&true) {
            let shapes = &self.shapes;
            let switched_shape = |s: u32| shapes.get(s as usize).and_then(|s| s.body).is_some_and(|b| switched[b]);
            self.contact_cache.retain(|&(a, b, _), _| !switched_shape(a) && !switched_shape(b));
            self.lock_cache.retain(|&(a, b, _), _| !(a != STATIC && switched[a]) && !switched[b]);
            for joint in self.joints.iter_mut().filter(|j| switched[j.parent] || switched[j.child]) {
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
            let point_mass = if k.determinant().abs() > 1e-12 { k.inverse() } else { Mat3::ZERO };
            let (sra, srb) = (skew(ra), skew(rb));
            let k_pa = -(sra * sa.inv_i + srb * sbb.inv_i);
            let k_ap = sa.inv_i * sra + sbb.inv_i * srb;
            (point_mass, k_pa, k_ap, sa.inv_i + sbb.inv_i - k_ap * point_mass * k_pa)
        };
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
            // Only when one side is grounded: a foot and toe both on the ground are still
            // just a foot and a toe.
            let still = |s: SolverBody| SolverBody { inv_mass: 0.0, inv_i: Mat3::ZERO, ..s };
            let cap_schur = match (grounded[a], grounded[b]) {
                (true, false) => blocks(still(sa), sbb, ra, rb).3,
                (false, true) => blocks(sa, still(sbb), ra, rb).3,
                _ => angular_schur,
            };
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
            let (k, c) = cap_spring(joint.soft_stiffness, joint.soft_damping, coupled_inv_mass(axis));
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
            rows.limits.extend(spring(twist_axis, twist - soft_l.twist_min));
        }
        if twist > soft_l.twist_max {
            rows.limits.extend(spring(-twist_axis, soft_l.twist_max - twist));
        }

        // Swing: elliptical cone in the primary frame's YZ plane.
        let s = glam::Vec2::new(swing.y, swing.z);
        let radius = s.length();
        if radius > 1e-6 {
            let dir = s / radius;
            let boundary = |l: &Limits| {
                let (y, z) = (l.swing1.max(1e-3), l.swing2.max(1e-3));
                (1.0 / ((dir.x / y).powi(2) + (dir.y / z).powi(2)).sqrt(), y, z)
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
            let (stiffness, damping) = cap_spring(gains.stiffness, gains.damping, axis.dot(rows.cap_schur * axis));
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

    fn solve_joints(&self, sb: &mut [SolverBody], joints: &mut [JointRows], soft_params: Softness, h: f32, use_bias: bool) {
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
            if !self.bodies[b].is_dynamic() && lock.anchor.is_none_or(|a| !self.bodies[a].is_dynamic()) {
                continue;
            }
            let (anchor_rot, anchor_pos) = match lock.anchor {
                Some(a) => (self.bodies[a].rotation, self.bodies[a].position),
                None => (Quat::IDENTITY, Vec3::ZERO),
            };
            let body = &self.bodies[b];
            let ra = anchor_rot * lock.target.1;
            let frame = Mat3::from_quat(anchor_rot);
            let mut turn = body.rotation * (anchor_rot * lock.target.0).inverse();
            if turn.w < 0.0 {
                turn = -turn;
            }
            let position_error = frame.transpose() * (body.position - (anchor_pos + ra));
            let rotation_error = frame.transpose() * turn.to_scaled_axis();

            // Effective mass in lock axes: [linear; angular] rows, B's centre held.
            let (ia, ib) = (get(sb, a), get(sb, b));
            let sra = skew(ra);
            let k_ll = Mat3::from_diagonal(Vec3::splat(ia.inv_mass + ib.inv_mass)) - sra * ia.inv_i * sra;
            let k_la = -(sra * ia.inv_i);
            let k_aa = ia.inv_i + ib.inv_i;
            let blocks = [
                [frame.transpose() * k_ll * frame, frame.transpose() * k_la * frame],
                [frame.transpose() * k_la.transpose() * frame, frame.transpose() * k_aa * frame],
            ];
            let strength = [lock.linear.to_array(), lock.angular.to_array()].concat();
            let mut k = [[0.0f32; 6]; 6];
            let mut bound = [0.0f32; 6];
            for r in 0..6 {
                for c in 0..6 {
                    k[r][c] = blocks[r / 3][c / 3].col(c % 3)[r % 3];
                }
            }
            for r in 0..6 {
                // Free axes, and axes neither body can move along, drop out of the block.
                if strength[r] > 0.0 && k[r][r] > 1e-9 {
                    bound[r] = strength[r] * h;
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
            out.push(LockBlock { a, b, ra, frame, error, inv_k, bound, impulse, key });
        }
        out
    }

    /// Groups of moving bodies that locks tie together (with the joints between them),
    /// and the rows each group solves directly. Nothing without a lock is grouped.
    fn prepare_groups(&self, sb: &[SolverBody], joints: &[JointRows], locks: &[LockBlock], bias_rate: f32) -> Vec<LockGroup> {
        if locks.is_empty() {
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
        let pairs = joints.iter().map(|j| (j.a, j.b)).chain(locks.iter().map(|l| (l.a, l.b)));
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
        locked.sort_unstable();
        locked.dedup();
        let mut rows: Vec<Vec<GroupRow>> = locked.iter().map(|_| Vec::new()).collect();

        for (ji, j) in joints.iter().enumerate() {
            let Some(g) = group_of(j.a, j.b).and_then(|g| locked.binary_search(&g).ok()) else { continue };
            let bias = (j.point_error * bias_rate).clamp_length_max(MAX_PUSH_SPEED);
            for (axis, e) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
                rows[g].push(GroupRow {
                    a: j.a,
                    b: j.b,
                    jacobian: [-e, -j.ra.cross(e), e, j.rb.cross(e)],
                    bias: bias[axis],
                    bound: f32::INFINITY,
                    source: RowSource::Anchor(ji, axis),
                });
            }
        }
        for (li, l) in locks.iter().enumerate() {
            let Some(g) = group_of(l.a, l.b).and_then(|g| locked.binary_search(&g).ok()) else { continue };
            let push = (Vec3::from_slice(&l.error[..3]) * bias_rate).clamp_length_max(MAX_PUSH_SPEED);
            let turn = (Vec3::from_slice(&l.error[3..]) * bias_rate).clamp_length_max(MAX_PUSH_SPEED * 4.0);
            for axis in (0..6).filter(|&r| l.bound[r] > 0.0) {
                let f = l.frame.col(axis % 3);
                let jacobian = if axis < 3 { [-f, -l.ra.cross(f), f, Vec3::ZERO] } else { [Vec3::ZERO, -f, Vec3::ZERO, f] };
                let bias = if axis < 3 { push[axis] } else { turn[axis - 3] };
                rows[g].push(GroupRow { a: l.a, b: l.b, jacobian, bias, bound: l.bound[axis], source: RowSource::Lock(li, axis) });
            }
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
            });
        }
        rows.into_iter().filter_map(|rows| LockGroup::new(sb, rows, locks)).collect()
    }

    /// Body pairs held together by locks, which don't collide with each other: each
    /// locked body and its anchor, and bodies locked to the same anchor (two hands
    /// clasped around someone).
    fn locked_pairs(&self) -> Vec<(usize, usize)> {
        let pair = |a: usize, b: usize| (a.min(b), a.max(b));
        let mut out: Vec<(usize, usize)> = self.locks.iter().filter_map(|l| Some(pair(l.anchor?, l.body))).collect();
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
                None => Some(Placed { geometry: s.geometry, rotation: s.rotation, position: s.position }),
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
        let radius: Vec<f32> = self.shapes.iter().map(|s| collide::bounding_radius(&s.geometry)).collect();
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
                    out.push(self.contact_row(sb, h, None, Some(i), ground_material, shape.material, c));
                }
            }
        }
        for i in 0..self.shapes.len() {
            let Some(pi) = &placed[i] else { continue };
            for j in (i + 1)..self.shapes.len() {
                let Some(pj) = &placed[j] else { continue };
                let (si, sj) = (&self.shapes[i], &self.shapes[j]);
                if !(dynamic(si) || dynamic(sj)) || si.body == sj.body || held_together(si.body, sj.body) {
                    continue;
                }
                let reach = radius[i] + radius[j] + MARGIN;
                if pi.position.distance_squared(pj.position) > reach * reach || !self.may_collide(i, j) {
                    continue;
                }
                found.clear();
                collide::between(pi, pj, MARGIN, &mut found);
                for c in &found {
                    out.push(self.contact_row(sb, h, Some(i), Some(j), si.material, sj.material, c));
                }
            }
        }
        out
    }

    /// Self-collision rules within one character, following each shape's flags.
    fn may_collide(&self, i: usize, j: usize) -> bool {
        let (si, sj) = (&self.shapes[i], &self.shapes[j]);
        let (Some(bi), Some(bj)) = (si.body, sj.body) else { return true };
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
        let origin = |i: usize| if i == STATIC { c.point } else { self.bodies[i].position };
        let (ra, rb) = (c.point - origin(a), c.point - origin(b));
        let interaction = self.interaction(material_a, material_b);
        let row_mass = |dir: Vec3, softness: f32| {
            let (ba, bb) = (get(sb, a), get(sb, b));
            let k = ba.inv_mass + bb.inv_mass + ra.cross(dir).dot(ba.inv_i * ra.cross(dir)) + rb.cross(dir).dot(bb.inv_i * rb.cross(dir));
            inverse(k + softness)
        };
        let t1 = perpendicular(c.normal);
        let tangents = [t1, c.normal.cross(t1)];
        let key = (shape_a.map_or(u32::MAX, |s| s as u32), shape_b.map_or(u32::MAX, |s| s as u32), c.feature);
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

    fn solve_contacts(&self, sb: &mut [SolverBody], contacts: &mut [ContactRow], soft_params: Softness, h: f32, use_bias: bool) {
        // While correcting positions, moved bodies push with their full velocity; after,
        // only with what they hand on.
        let velocity = if use_bias { relative_velocity } else { handed_velocity };
        for c in contacts.iter_mut() {
            // Friction first, limited by the current normal impulse or a fixed force cap.
            let limit = match self.contact_model {
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

            let vn = velocity(sb, c.a, c.b, c.ra, c.rb).dot(c.normal);
            let (bias, mass_scale, impulse_scale) = if c.separation > 0.0 {
                (c.separation / h, 1.0, 0.0)
            } else if use_bias {
                ((soft_params.bias_rate * c.separation).max(-MAX_PUSH_SPEED), soft_params.mass_scale, soft_params.impulse_scale)
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
    /// Velocity along the lock's rows.
    fn velocity(&self, sb: &[SolverBody]) -> [f32; 6] {
        let linear = self.frame.transpose() * relative_velocity(sb, self.a, self.b, self.ra, Vec3::ZERO);
        let angular = self.frame.transpose() * (get(sb, self.b).w - get(sb, self.a).w);
        [linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
    }

    fn apply(&self, sb: &mut [SolverBody], impulse: [f32; 6]) {
        let linear = self.frame * Vec3::new(impulse[0], impulse[1], impulse[2]);
        let angular = self.frame * Vec3::new(impulse[3], impulse[4], impulse[5]);
        apply_impulse(sb, self.a, self.b, self.ra, Vec3::ZERO, linear);
        apply_angular(sb, self.a, self.b, angular);
    }
}

/// Solves each lock's block; `bias_rate` is the share of the error corrected per second.
fn solve_locks(sb: &mut [SolverBody], locks: &mut [LockBlock], bias_rate: f32) {
    for l in locks.iter_mut() {
        let mut rhs = l.velocity(sb);
        let push = Vec3::from_slice(&l.error[..3]) * bias_rate;
        let turn = Vec3::from_slice(&l.error[3..]) * bias_rate;
        let bias = [push.clamp_length_max(MAX_PUSH_SPEED).to_array(), turn.clamp_length_max(MAX_PUSH_SPEED * 4.0).to_array()].concat();
        for (r, b) in rhs.iter_mut().zip(bias) {
            *r = -(*r + b);
        }
        let mut delta = [0.0f32; 6];
        for (r, d) in delta.iter_mut().enumerate() {
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
        let moves = |body: usize| body != STATIC && (sb[body].inv_mass > 0.0 || sb[body].inv_i != Mat3::ZERO);
        // M⁻¹ Jᵀ per row: change in A's and B's velocities per unit impulse.
        let response = |r: &GroupRow| {
            let (sa, sbb) = (get(sb, r.a), get(sb, r.b));
            [r.jacobian[0] * sa.inv_mass, sa.inv_i * r.jacobian[1], r.jacobian[2] * sbb.inv_mass, sbb.inv_i * r.jacobian[3]]
        };
        let entry = |ri: &GroupRow, rj: &GroupRow, mj: &[Vec3; 4]| {
            let mut k = 0.0;
            for (si, bi) in [ri.a, ri.b].into_iter().enumerate() {
                for (sj, bj) in [rj.a, rj.b].into_iter().enumerate() {
                    if bi == bj && bi != STATIC {
                        k += ri.jacobian[2 * si].dot(mj[2 * sj]) + ri.jacobian[2 * si + 1].dot(mj[2 * sj + 1]);
                    }
                }
            }
            k as f64
        };
        let rows: Vec<GroupRow> = rows.into_iter().filter(|r| entry(r, r, &response(r)) > 1e-9).collect();
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
            let mut near: Vec<usize> = [r.a, r.b].into_iter().filter(|&b| moves(b)).flat_map(|b| on_body[&b].iter().copied()).filter(|&j| j < i).collect();
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
                RowSource::Anchor(..) => false,
            })
            .collect();
        let mut group = LockGroup { rows, k, k_pattern, pattern: Vec::new(), velocity_factor: Vec::new(), push_factor: Vec::new(), out };
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
    fn solve(&self, l: &[f64], sb: &[SolverBody], velocity_scale: f64, bias_scale: f64) -> Vec<f64> {
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
                let v = r.jacobian[0].dot(sa.v) + r.jacobian[1].dot(sa.w) + r.jacobian[2].dot(sbb.v) + r.jacobian[3].dot(sbb.w);
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
                for (side, body) in [r.a, r.b].into_iter().enumerate().filter(|&(_, b)| b != STATIC) {
                    let s = sb[body];
                    moved[body].0 += r.jacobian[2 * side] * (s.inv_mass * d as f32);
                    moved[body].1 += s.inv_i * (r.jacobian[2 * side + 1] * d as f32);
                }
            }
            let fastest = moved.iter().fold(0.0f32, |m, (v, w)| m.max(v.length() / MAX_PUSH_SPEED).max(w.length() / MAX_ANGULAR_SPEED));
            let scale = if fastest > 1.0 { 1.0 / fastest as f64 } else { 1.0 };
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
fn solve_groups(sb: &mut [SolverBody], groups: &mut [LockGroup], joints: &mut [JointRows], locks: &mut [LockBlock], use_bias: bool) {
    for group in groups.iter_mut() {
        loop {
            let x = group.impulses(sb, use_bias);
            let over = group.rows.iter().zip(&x).position(|(r, &dx)| match r.source {
                RowSource::Lock(l, axis) => (locks[l].impulse[axis] + dx as f32).abs() > r.bound,
                RowSource::Anchor(..) => false,
            });
            if let Some(i) = over {
                let r = &group.rows[i];
                let RowSource::Lock(l, axis) = r.source else { unreachable!() };
                let held = (locks[l].impulse[axis] + x[i] as f32).clamp(-r.bound, r.bound);
                LockGroup::apply(r, sb, held - locks[l].impulse[axis]);
                locks[l].impulse[axis] = held;
                group.out[i] = true;
                group.factorize();
                continue;
            }
            for (r, &dx) in group.rows.iter().zip(&x) {
                let d = dx as f32;
                match r.source {
                    RowSource::Anchor(j, axis) => joints[j].point_impulse[axis] += d,
                    RowSource::Lock(l, axis) => locks[l].impulse[axis] += d,
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
fn solve_angular(sb: &mut [SolverBody], a: usize, b: usize, row: &mut AngularRow, soft_params: Softness, h: f32, use_bias: bool, coupling: f32) -> f32 {
    let (ba, bb) = (get(sb, a), get(sb, b));
    let jv = row.axis.dot(bb.w - ba.w) - coupling;
    let old = row.impulse;
    let impulse = if row.spring {
        -row.mass * (jv + row.value + row.gamma * old)
    } else {
        let (bias, mass_scale, impulse_scale) = if row.value > 0.0 {
            (row.value / h, 1.0, 0.0)
        } else if use_bias {
            ((soft_params.bias_rate * row.value).max(-MAX_PUSH_SPEED * 4.0), soft_params.mass_scale, soft_params.impulse_scale)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::document::physics::Interaction;

    fn world() -> World {
        let mut w = World::new(Vec3::new(0.0, -9.81, 0.0), ContactModel::ForceCapped);
        w.set_materials(1, |_, _| Interaction { restitution: 0.0, friction: 100.0, adhesion: 0.0, softness: 0.0 });
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
        assert!((body.position.y - 0.1).abs() < 0.01, "rests at {}", body.position.y);
        assert!(body.linear_velocity.length() < 0.05);
    }

    #[test]
    fn restitution_bounces() {
        let mut w = world();
        w.set_materials(1, |_, _| Interaction { restitution: 0.8, friction: 100.0, adhesion: 0.0, softness: 0.0 });
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
        let wide = Limits { twist_min: -3.0, twist_max: 3.0, swing1: 3.0, swing2: 3.0 };
        let mut joint = Joint::new(
            a, b, Vec3::ZERO, Vec3::new(-0.5, 0.0, 0.0),
            Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY,
            wide, wide, 0.0, 0.0,
        );
        let gains = Gains { stiffness: 2000.0, damping: 50.0, force_limit: 500.0 };
        joint.muscle = Some(Muscle { target: Quat::IDENTITY, twist: gains, swing: gains });
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
        let wide = Limits { twist_min: -3.0, twist_max: 3.0, swing1: 3.0, swing2: 3.0 };
        let mut joint = Joint::new(
            a, b, Vec3::ZERO, Vec3::new(-0.5, 0.0, 0.0),
            Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY,
            wide, wide, 0.0, 0.0,
        );
        // Very stiff but only 1 N·m: can't hold up a 4.9 N·m load.
        let gains = Gains { stiffness: 1e5, damping: 100.0, force_limit: 1.0 };
        joint.muscle = Some(Muscle { target: Quat::IDENTITY, twist: gains, swing: gains });
        w.joints.push(joint);
        let mut lowest = f32::MAX;
        for _ in 0..180 {
            w.step(1.0 / 120.0);
            lowest = lowest.min(w.bodies[b].position.y);
        }
        assert!(lowest < -0.3, "lowest {lowest}");
    }

    #[test]
    fn twist_limit_stops_rotation() {
        let mut w = world();
        w.ground = None;
        w.gravity = Vec3::ZERO;
        let a = ball(&mut w, Vec3::ZERO, 0.05, 5.0);
        let b = ball(&mut w, Vec3::new(0.5, 0.0, 0.0), 0.05, 1.0);
        w.bodies[a].motion = Motion::Kinematic { momentum: 1.0 };
        let limits = Limits { twist_min: -0.5, twist_max: 0.5, swing1: 1.0, swing2: 1.0 };
        w.joints.push(Joint::new(
            a, b, Vec3::ZERO, Vec3::new(-0.5, 0.0, 0.0),
            Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY,
            limits, limits, 0.0, 0.0,
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
        let limits = Limits { twist_min: -1.0, twist_max: 1.0, swing1: 0.3, swing2: 0.3 };
        w.joints.push(Joint::new(
            a, b, Vec3::ZERO, Vec3::new(-0.5, 0.0, 0.0),
            Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY,
            limits, limits, 0.0, 0.0,
        ));
        run(&mut w, 2.0);
        let (_, swing) = swing_twist(w.bodies[b].rotation);
        assert!(swing.length() < 0.36, "swung {}", swing.length());
    }

    #[test]
    fn muscle_targets_are_clamped_into_limits() {
        let limits = Limits { twist_min: -0.5, twist_max: 0.5, swing1: 0.3, swing2: 0.3 };
        let joint = Joint::new(
            0, 1, Vec3::ZERO, Vec3::ZERO,
            Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY,
            limits, limits, 0.0, 0.0,
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
        w.locks.push(Lock { anchor: None, body: b, target: (Quat::IDENTITY, Vec3::new(0.0, 2.0, 0.0)), linear: Vec3::new(0.0, 1e5, 0.0), angular: Vec3::ZERO });
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
        w.locks.push(Lock { anchor: None, body: b, target: (Quat::IDENTITY, Vec3::new(0.0, 2.0, 0.0)), linear: Vec3::new(0.0, 50.0, 0.0), angular: Vec3::ZERO });
        run(&mut w, 1.0);
        // Net acceleration (98.1 - 50) N / 10 kg ≈ 4.8 m/s², so ≈ 2.4 m in a second.
        let fallen = 2.0 - w.bodies[b].position.y;
        assert!((fallen - 2.4).abs() < 0.1, "fell {fallen} m");
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
        let free = Limits { twist_min: -3.0, twist_max: 3.0, swing1: 3.0, swing2: 3.0 };
        w.joints.push(Joint::new(
            handle, load, Vec3::ZERO, Vec3::new(0.0, 0.5, 0.0),
            Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY,
            free, free, 0.0, 0.0,
        ));
        w.locks.push(Lock { anchor: Some(anchor), body: handle, target: (Quat::IDENTITY, Vec3::new(0.0, -0.1, 0.0)), linear: Vec3::splat(1e5), angular: Vec3::splat(1e5) });
        run(&mut w, 1.0);
        let gap = w.bodies[handle].position - (w.bodies[anchor].position + Vec3::new(0.0, -0.1, 0.0));
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
        let bar = ball(&mut w, pivot + Vec3::new(0.0, -angle.cos(), angle.sin()) * length, 0.03, 2.7);
        // As the scene lists them: the join, then the hinge.
        let offset = w.bodies[top].position - w.bodies[bar].position;
        w.locks.push(Lock { anchor: Some(bar), body: top, target: (Quat::IDENTITY, offset), linear: Vec3::splat(1e5), angular: Vec3::splat(1e5) });
        w.locks.push(Lock { anchor: None, body: top, target: (Quat::IDENTITY, pivot), linear: Vec3::splat(1e5), angular: Vec3::new(0.0, 1e5, 1e5) });
        let mut fastest = 0.0f32;
        for _ in 0..240 {
            w.step(1.0 / 120.0);
            fastest = fastest.max(w.bodies[bar].linear_velocity.length());
        }
        let expected = (2.0 * 9.81 * length * (1.0 - angle.cos())).sqrt();
        assert!((fastest - expected).abs() < 0.05 * expected, "fastest {fastest}, a pendulum reaches {expected}");
        assert!(w.bodies[top].position.distance(pivot) < 0.01, "hinge moved to {}", w.bodies[top].position);
        assert!((w.bodies[bar].position.distance(pivot) - length).abs() < 0.01, "bar at {}", w.bodies[bar].position.distance(pivot));
    }

    /// endorphin's Walk With Gun: a 550 kg prop joined to both hands closes a loop
    /// through two arms of half-kilo to two-kilo bodies. Solved one constraint at a time
    /// that tore the arms apart; the arms must carry it (here as it swings down).
    #[test]
    fn heavy_prop_held_in_both_hands_keeps_the_arms_together() {
        let mut w = world();
        w.ground = None;
        let free = Limits { twist_min: -3.0, twist_max: 3.0, swing1: 3.0, swing2: 3.0 };
        let torso = ball(&mut w, Vec3::new(0.0, 1.5, 0.0), 0.1, 30.0);
        w.bodies[torso].motion = Motion::Kinematic { momentum: 0.0 };
        let joint = |w: &mut World, parent: usize, at_parent: Vec3, at: Vec3, mass: f32| {
            let child = ball(w, at, 0.04, mass);
            w.bodies[child].group = child;
            let anchor = w.bodies[parent].position + at_parent;
            w.joints.push(Joint::new(parent, child, at_parent, anchor - at, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, free, free, 0.0, 0.0));
            child
        };
        let mut hands = Vec::new();
        for side in [-1.0f32, 1.0] {
            let x = 0.2 * side;
            let upper = joint(&mut w, torso, Vec3::new(x, 0.0, 0.0), Vec3::new(x, 1.5, 0.15), 2.0);
            let fore = joint(&mut w, upper, Vec3::new(0.0, 0.0, 0.15), Vec3::new(x, 1.5, 0.45), 1.5);
            hands.push(joint(&mut w, fore, Vec3::new(0.0, 0.0, 0.15), Vec3::new(x, 1.5, 0.65), 0.5));
        }
        let gun = ball(&mut w, Vec3::new(0.0, 1.5, 0.8), 0.15, 550.0);
        w.bodies[gun].group = gun;
        // JoinBodies [gun, right hand, left hand]: each held to the last.
        let left = hands[0];
        for body in [gun, hands[1]] {
            let offset = w.bodies[body].position - w.bodies[left].position;
            w.locks.push(Lock { anchor: Some(left), body, target: (Quat::IDENTITY, offset), linear: Vec3::splat(1e5), angular: Vec3::splat(1e4) });
        }
        let gap = |w: &World| {
            w.joints.iter().map(|j| {
                let (a, b) = (&w.bodies[j.parent], &w.bodies[j.child]);
                (a.position + a.rotation * j.anchor_parent).distance(b.position + b.rotation * j.anchor_child)
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
        assert!(held.is_finite() && held.length() < 1.0, "gun came loose: {held}");
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
            assert!(body.position.x > wall.position.x + 0.27, "passed through at momentum {momentum}");
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
        assert_eq!(w.bodies[table].position, Vec3::ZERO, "a massless body must not move");
        let rest = w.bodies[cup].position.y;
        assert!((rest - 0.6).abs() < 0.01, "should rest on the fixed body, at {rest}");
    }

    #[test]
    fn simulation_is_deterministic() {
        let make = || {
            let mut w = world();
            for i in 0..5 {
                ball(&mut w, Vec3::new(i as f32 * 0.15, 0.5 + i as f32 * 0.3, 0.0), 0.1, 1.0);
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
        w.set_materials(1, |_, _| Interaction { restitution: 0.2, friction: 100.0, adhesion: 0.0, softness: 0.0 });
        let specs = [(5.0, 0.05, 0.0), (0.7, 0.03, 0.27), (0.5, 0.025, 0.13), (0.4, 0.02, 0.09)];
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
                motion: if i == 0 { Motion::Kinematic { momentum: 1.0 } } else { Motion::Dynamic },
                group: 0,
                parent: if i == 0 { None } else { Some(i - 1) },
            });
        }
        for i in 1..specs.len() {
            let len = specs[i].2;
            let limits = if tight {
                Limits { twist_min: -0.03, twist_max: 0.03, swing1: 0.01, swing2: 0.5 }
            } else {
                Limits { twist_min: -1.0, twist_max: 1.0, swing1: 1.0, swing2: 1.0 }
            };
            let mut j = Joint::new(i - 1, i, Vec3::ZERO, Vec3::new(-len, 0.0, 0.0), Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, limits, limits, 10.0, 10.0);
            let g = Gains { stiffness, damping: stiffness * 0.5, force_limit: stiffness };
            j.muscle = Some(Muscle { target: Quat::IDENTITY, twist: g, swing: g });
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
            w.set_materials(1, |_, _| Interaction { restitution: 0.0, friction: 100.0, adhesion: 0.0, softness: 0.0 });
            let free = Limits { twist_min: -0.5, twist_max: 0.5, swing1: 1.2, swing2: 1.2 };
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
                    motion: if k == 0 { Motion::Kinematic { momentum: 0.0 } } else { Motion::Dynamic },
                    group: 0,
                    parent: k.checked_sub(1),
                });
                if k > 0 {
                    w.joints.push(Joint::new(k - 1, k, Vec3::new(0.0, -0.1, 0.0), Vec3::new(0.0, 0.1, 0.0), Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY, free, free, 0.0, 0.0));
                }
            }
            let mut fastest = 0.0f32;
            for _ in 0..360 {
                w.step(1.0 / 120.0);
                fastest = w.bodies.iter().map(|b| b.linear_velocity.length()).fold(fastest, f32::max);
            }
            let bottom = w.bodies[7].position.y;
            assert!(fastest < 1.0 && (bottom - 3.6).abs() < 0.1, "load {load}: fastest {fastest}, bottom at {bottom}");
        }
    }

    #[test]
    fn light_chain_with_stiff_muscles_and_tight_stops_is_stable() {
        for (tight, k) in [(false, 1000.0), (true, 50.0), (true, 1000.0), (true, 5000.0)] {
            let fastest = arm(tight, k);
            eprintln!("tight stops {tight}, stiffness {k}: fastest {fastest:.2} m/s");
            assert!(fastest < 2.0, "chain blew up (tight {tight}, k {k}): {fastest} m/s");
        }
    }
}
