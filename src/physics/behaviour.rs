//! Behaviours: controllers that read a simulated character's state every physics tick
//! and set its muscle targets and strengths, like endorphin's balance, stagger and catch
//! fall.
//! endorphin's own behaviours are vendor-protected; these are original designs, run for
//! imported behaviour events of the same name. Design notes: docs/behaviours.md.
//!
//! Controllers are stateless (everything comes from the bodies each tick), so they keep
//! the simulation deterministic.

use glam::{Quat, Vec3};

use super::sim::rig_bodies;
use super::world::{Body, Joint, World};
use crate::document::physics::{Gains, Geometry, MuscleGains, PhysicsRig};
use crate::document::{Pose, SkeletonRef, world_transforms};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Behaviour {
    /// Keep the centre of mass over the feet (ankle strategy), stepping when they
    /// can't hold it; pelvis upright.
    Balance,
    /// Reach both arms towards where the chest will land; let the legs give.
    CatchFall,
    /// Curl up on the side of the trunk: spine, hips and knees folded, arms over the head.
    BodyFoetal,
    /// Struggle weakly around a half-curled pose, with slowly wandering targets.
    Writhe,
    /// Balance that steps to keep up with a push, with a looser trunk.
    Stagger,
}

impl Behaviour {
    /// The behaviour for an event's name (endorphin's `behaviourName`), ignoring case,
    /// spaces and punctuation; `None` if NeuroMotion has none of that name.
    ///
    /// endorphin's scenes name a behaviour after the set it comes from and its version,
    /// e.g. `Bob2: Balance 2.5` or `Bob2: BodyWrithe`: the part before the last `:` and
    /// a trailing version number are dropped before matching.
    pub fn from_name(name: &str) -> Option<Self> {
        let name = name.rsplit(':').next().unwrap_or(name);
        let name =
            name.trim_end_matches(|c: char| c.is_ascii_digit() || c == '.' || c.is_whitespace());
        let key: String = name
            .chars()
            .filter(|c| c.is_alphanumeric())
            .flat_map(char::to_lowercase)
            .collect();
        match key.as_str() {
            "balance" => Some(Behaviour::Balance),
            "catchfall" => Some(Behaviour::CatchFall),
            "bodyfoetal" | "bodyfetal" | "foetal" | "fetal" => Some(Behaviour::BodyFoetal),
            "writhe" | "bodywrithe" => Some(Behaviour::Writhe),
            "stagger" => Some(Behaviour::Stagger),
            _ => None,
        }
    }

    pub fn drive(self, d: &mut Drive) {
        match self {
            Behaviour::Balance => balance(d),
            Behaviour::CatchFall => catch_fall(d),
            Behaviour::BodyFoetal => body_foetal(d),
            Behaviour::Writhe => writhe(d),
            Behaviour::Stagger => stagger(d),
        }
    }
}

/// Rig bodies of a limb, from the trunk outwards: thigh, shin, foot or upper arm,
/// forearm, hand.
#[derive(Clone, Copy, Debug)]
struct Limb {
    upper: usize,
    lower: usize,
    end: usize,
}

/// Where a character's pelvis, trunk and limbs are in its rig.
#[derive(Clone, Debug)]
pub struct Layout {
    pelvis: usize,
    /// The pelvis's up and sideways directions in its own frame (world up and X at rest).
    pelvis_up: Vec3,
    pelvis_side: Vec3,
    /// The direction the character faces, in the pelvis's frame (where the toes point).
    pelvis_forward: Vec3,
    /// The trunk body the arms hang from.
    chest: Option<usize>,
    legs: Vec<Limb>,
    /// The sole of each leg's foot, in the same order.
    feet: Vec<Foot>,
    /// Each leg's side, flat, in the pelvis's frame: from the pelvis out to its hip.
    outward: Vec<Vec3>,
    arms: Vec<Limb>,
    /// Rig joint moving each body.
    joint_of: Vec<Option<usize>>,
    /// The trunk chain from the pelvis's trunk branch upwards: spine, chest, neck, head.
    spine: Vec<usize>,
    /// Each body's rest rotation relative to the body its joint hangs from.
    rest_rel: Vec<Quat>,
    /// Each body's bend axis, up and forward directions in its own frame at rest. Turning
    /// about the bend axis by +θ moves up towards forward: it bends a trunk forwards, and
    /// bends a hanging shin's knee; -θ swings a hanging thigh forwards (hip flexion).
    bend_local: Vec<Vec3>,
    up_local: Vec<Vec3>,
    fwd_local: Vec<Vec3>,
}

impl Layout {
    /// Finds the limbs from the rig's shape at rest rather than from names, so it works
    /// on any biped: the legs are the two branches from the pelvis reaching lowest, the
    /// trunk the one reaching highest, and the arms the two branches off the trunk
    /// reaching furthest sideways. A leg's thigh and shin are the first two bodies of
    /// its branch; an arm's upper arm and forearm the consecutive pair reaching furthest
    /// to its tip, split most evenly at the elbow (skipping collarbones, hands, fingers
    /// and a forearm's second body).
    pub fn new(rig: &PhysicsRig, skeleton: &SkeletonRef) -> Option<Self> {
        let n = rig.bodies.len();
        let rest_joints = world_transforms(skeleton, &Pose::rest(skeleton.joints.len()));
        let rest = rig_bodies(rig, &rest_joints);
        let mut joint_of = vec![None; n];
        let mut children = vec![Vec::new(); n];
        for (j, jd) in rig.joints.iter().enumerate() {
            if jd.body < n
                && jd.parent_body < n
                && jd.body != jd.parent_body
                && jd.joint < rest_joints.len()
            {
                joint_of[jd.body] = Some(j);
                children[jd.parent_body].push(jd.body);
            }
        }
        let pelvis = (0..n).find(|&b| joint_of[b].is_none() && !children[b].is_empty())?;
        let at = |b: usize| rest[b].1;
        // Where each body's joint is at rest (its origin for the pelvis).
        let pivot = |b: usize| joint_of[b].map_or(at(b), |j| rest_joints[rig.joints[j].joint].1);
        // Extremes of a branch at rest, by some measure of body position. `depth` stops a
        // malformed rig whose joints loop from recursing forever.
        fn extreme(
            children: &[Vec<usize>],
            b: usize,
            measure: &dyn Fn(usize) -> f32,
            depth: usize,
        ) -> f32 {
            let below = children[b]
                .iter()
                .filter(|_| depth > 0)
                .map(|&c| extreme(children, c, measure, depth - 1));
            below.fold(measure(b), f32::max)
        }
        let extreme = |b: usize, measure: &dyn Fn(usize) -> f32| extreme(&children, b, measure, n);
        // Follows a branch towards its extreme, one body at a time.
        let follow = |start: usize, measure: &dyn Fn(usize) -> f32| {
            let mut chain = vec![start];
            while let Some(&next) = children[*chain.last().unwrap()]
                .iter()
                .max_by(|&&a, &&b| extreme(a, measure).total_cmp(&extreme(b, measure)))
            {
                if chain.contains(&next) {
                    break;
                }
                chain.push(next);
            }
            chain
        };
        let low = |b: usize| -at(b).y;
        let high = |b: usize| at(b).y;
        let mut branches: Vec<usize> = children[pelvis].clone();
        branches.sort_by(|&a, &b| extreme(b, &low).total_cmp(&extreme(a, &low)));
        let legs: Vec<Limb> = branches
            .iter()
            .take(2)
            .filter_map(|&thigh| {
                let chain = follow(thigh, &low);
                (chain.len() >= 3).then(|| Limb {
                    upper: chain[0],
                    lower: chain[1],
                    end: chain[2],
                })
            })
            .collect();

        let trunk = branches
            .iter()
            .skip(2)
            .copied()
            .max_by(|&a, &b| extreme(a, &high).total_cmp(&extreme(b, &high)));
        let mut arms = Vec::new();
        let mut chest = None;
        let mut trunk_chain = Vec::new();
        if let Some(trunk) = trunk {
            let spine = follow(trunk, &high);
            trunk_chain = spine.clone();
            let axis = at(pelvis);
            let side = |b: usize| (at(b) - axis).with_y(0.0).length();
            let mut off: Vec<(f32, usize, usize)> = spine
                .iter()
                .flat_map(|&s| {
                    children[s]
                        .iter()
                        .filter(|c| !spine.contains(c))
                        .map(move |&c| (s, c))
                })
                .map(|(s, c)| (extreme(c, &side), s, c))
                .collect();
            off.sort_by(|a, b| b.0.total_cmp(&a.0));
            for &(_, s, c) in off.iter().take(2) {
                let start = pivot(c);
                let chain = follow(c, &|b| at(b).distance(start));
                // The arm's tip: the far end of its last body, taken as its centre
                // mirrored about its joint (a hand or finger hangs from its joint).
                let last = *chain.last().unwrap();
                let tip = 2.0 * at(last) - pivot(last);
                // Shoulder and elbow are the consecutive pair of joints reaching furthest
                // to the tip, weighed by how evenly the elbow splits that reach (shorter
                // part over longer). The longest pair of segments isn't enough: a
                // forearm in two bodies (endorphin's) loses to collarbone and upper arm.
                let score = |i: usize| {
                    let (shoulder, elbow) = (pivot(chain[i]), pivot(chain[i + 1]));
                    let (a, b) = (shoulder.distance(elbow), elbow.distance(tip));
                    shoulder.distance(tip) * a.min(b) / a.max(b).max(1e-6)
                };
                let best = (0..chain.len().saturating_sub(2))
                    .max_by(|&a, &b| score(a).total_cmp(&score(b)));
                if let Some(i) = best {
                    arms.push(Limb {
                        upper: chain[i],
                        lower: chain[i + 1],
                        end: chain[i + 2],
                    });
                    chest.get_or_insert(s);
                }
            }
        }
        let pelvis_up = rest[pelvis].0.inverse() * Vec3::Y;
        let pelvis_side = rest[pelvis].0.inverse() * Vec3::X;

        // Each foot's sole: the lowest points of its collision shapes and those of the
        // bodies beyond it (toes), within 2 cm of the lowest, in the foot's own frame.
        let (mut feet, toes): (Vec<Foot>, Vec<Vec3>) = legs
            .iter()
            .map(|leg| {
                let mut bodies = vec![leg.end];
                let mut i = 0;
                while i < bodies.len() && bodies.len() <= n {
                    for &c in &children[bodies[i]] {
                        if !bodies.contains(&c) {
                            bodies.push(c);
                        }
                    }
                    i += 1;
                }
                let mut points: Vec<Vec3> = bodies
                    .iter()
                    .flat_map(|&b| {
                        let (r, p) = rest[b];
                        rig.bodies[b].shapes.iter().flat_map(move |s| {
                            let sr = r * Quat::from_array(s.frame.rotation).normalize();
                            let sp = p + r * Vec3::from(s.frame.position);
                            shape_bottom(s.geometry, sr, sp)
                        })
                    })
                    .collect();
                if points.is_empty() {
                    points.push(at(leg.end));
                }
                let lowest = points.iter().map(|p| p.y).fold(f32::MAX, f32::min);
                points.retain(|p| p.y < lowest + 0.02);
                let (fr, fp) = rest[leg.end];
                let ankle = pivot(leg.end);
                let centre = points.iter().sum::<Vec3>() / points.len() as f32;
                let foot = Foot {
                    sole: points.iter().map(|&q| fr.inverse() * (q - fp)).collect(),
                    bodies,
                    up: fr.inverse() * Vec3::Y,
                    ankle_height: ankle.y - lowest,
                    forward: Vec3::ZERO,
                };
                (foot, flat(centre - ankle))
            })
            .unzip();
        // Which way the character faces: where its toes point (both feet together), so
        // it doesn't depend on how the rig is turned at rest. Feet centred under their
        // ankles don't say; then the rig is taken to face +Z with its left at +X, as
        // the pelvis's sideways direction already assumes.
        let toes = toes.iter().sum::<Vec3>();
        let forward = if toes.length() > 0.02 {
            toes.normalize()
        } else {
            (rest[pelvis].0 * pelvis_side)
                .cross(Vec3::Y)
                .normalize_or(Vec3::Z)
        };
        let pelvis_forward = rest[pelvis].0.inverse() * forward;
        for (foot, leg) in feet.iter_mut().zip(&legs) {
            foot.forward = rest[leg.end].0.inverse() * forward;
        }
        let outward = legs
            .iter()
            .map(|l| rest[pelvis].0.inverse() * flat(pivot(l.upper) - at(pelvis)))
            .collect();
        let fwd = forward;
        let bend = Vec3::Y.cross(fwd);
        let rest_rel = (0..n)
            .map(|b| {
                joint_of[b].map_or(Quat::IDENTITY, |j| {
                    rest[rig.joints[j].parent_body].0.inverse() * rest[b].0
                })
            })
            .collect();
        let local = |v: Vec3| (0..n).map(|b| rest[b].0.inverse() * v).collect::<Vec<_>>();
        Some(Layout {
            spine: trunk_chain,
            rest_rel,
            bend_local: local(bend),
            up_local: local(Vec3::Y),
            fwd_local: local(fwd),
            pelvis,
            pelvis_up,
            pelvis_side,
            pelvis_forward,
            chest,
            legs,
            feet,
            outward,
            arms,
            joint_of,
        })
    }

    /// Every body of the rig except the legs and feet (toes included): pelvis, trunk,
    /// head and arms.
    fn upper_body(&self, n: usize) -> impl Iterator<Item = usize> + '_ {
        (0..n).filter(|b| {
            !self.legs.iter().any(|l| [l.upper, l.lower].contains(b))
                && !self.feet.iter().any(|f| f.bodies.contains(b))
        })
    }
}

/// The points of a collision shape that could touch the ground below it (at rest).
fn shape_bottom(geometry: Geometry, rotation: Quat, at: Vec3) -> Vec<Vec3> {
    match geometry {
        Geometry::Sphere { radius } => vec![at - Vec3::Y * radius],
        Geometry::Capsule { radius, length } | Geometry::Cylinder { radius, length } => {
            let half = rotation * Vec3::Z * (length * 0.5);
            vec![at + half - Vec3::Y * radius, at - half - Vec3::Y * radius]
        }
        Geometry::Box { size } => {
            let h = Vec3::from(size) * 0.5;
            let mut corners = Vec::with_capacity(8);
            for x in [-h.x, h.x] {
                for y in [-h.y, h.y] {
                    for z in [-h.z, h.z] {
                        corners.push(at + rotation * Vec3::new(x, y, z));
                    }
                }
            }
            corners
        }
    }
}

/// A foot's shape, for stepping: where it touches the ground, measured at rest.
#[derive(Clone, Debug)]
struct Foot {
    /// Sole points in the foot body's frame.
    sole: Vec<Vec3>,
    /// The foot body and those beyond it (toes).
    bodies: Vec<usize>,
    /// World up in the foot's frame at rest (the foot is flat then).
    up: Vec3,
    /// Height of the ankle above the sole, standing flat, m.
    ankle_height: f32,
    /// The way the character faces, in the foot's frame (at rest, it stands with its
    /// feet pointing that way).
    forward: Vec3,
}

/// A behaviour's view of one character for one tick: its bodies and joints in the world,
/// and the muscle targets and gains it may change (one per rig joint), which start out
/// as the timeline asks (active pose, animation or the held pose).
pub struct Drive<'a> {
    pub world: &'a World,
    pub first_body: usize,
    pub first_joint: usize,
    pub rig: &'a PhysicsRig,
    pub layout: &'a Layout,
    pub targets: &'a mut [Quat],
    pub gains: &'a mut [MuscleGains],
    /// Seconds since the behaviour event started.
    pub elapsed: f32,
}

impl Drive<'_> {
    fn body(&self, b: usize) -> &Body {
        &self.world.bodies[self.first_body + b]
    }

    fn joint(&self, b: usize) -> Option<(usize, &Joint)> {
        let j = self.layout.joint_of[b]?;
        Some((j, self.world.joints.get(self.first_joint + j)?))
    }

    /// Where the joint moving body `b` is.
    fn pivot(&self, b: usize) -> Vec3 {
        let body = self.body(b);
        self.joint(b).map_or(body.position, |(_, j)| {
            body.position + body.rotation * j.anchor_child
        })
    }

    /// Rotation of the body the joint moving `b` hangs from.
    fn parent_rotation(&self, b: usize) -> Quat {
        self.layout.joint_of[b].map_or(Quat::IDENTITY, |j| {
            self.body(self.rig.joints[j].parent_body).rotation
        })
    }

    /// Sets the muscle moving `b` to bring its body and its parent to these world rotations.
    fn aim(&mut self, b: usize, parent: Quat, child: Quat) {
        if let Some((j, joint)) = self.joint(b) {
            let target =
                ((parent * joint.drive_parent).inverse() * (child * joint.drive_child)).normalize();
            self.targets[j] = target;
        }
    }

    /// Gains from a named muscle preset (falling back to the rig's first), scaled.
    fn preset(&mut self, name: &str, scale: f32, bodies: impl Iterator<Item = usize>) {
        self.preset_of(&[name], scale, bodies);
    }

    /// Like `preset`, with the first of several names the rig has.
    fn preset_of(&mut self, names: &[&str], scale: f32, bodies: impl Iterator<Item = usize>) {
        let set = names
            .iter()
            .find_map(|n| self.rig.muscle_set(n))
            .or_else(|| self.rig.muscle_sets.first());
        let scaled = |g: Gains| Gains {
            stiffness: g.stiffness * scale,
            damping: g.damping * scale,
            force_limit: g.force_limit * scale,
        };
        for b in bodies {
            if let Some(j) = self.layout.joint_of[b] {
                let g = set.map_or(STEADY, |s| s.gains(&self.rig.joints[j].muscle));
                self.gains[j] = MuscleGains {
                    twist: scaled(g.twist),
                    swing: scaled(g.swing),
                };
            }
        }
    }

    /// Centre of mass and its velocity.
    fn centre_of_mass(&self) -> (Vec3, Vec3) {
        let (mut mass, mut at, mut vel) = (0.0, Vec3::ZERO, Vec3::ZERO);
        for (b, def) in self.rig.bodies.iter().enumerate() {
            let body = self.body(b);
            mass += def.mass;
            at += body.position * def.mass;
            vel += body.linear_velocity * def.mass;
        }
        (at / mass.max(1e-6), vel / mass.max(1e-6))
    }

    /// The middle of the feet, on the ground plane (the pelvis without legs).
    fn support(&self) -> Vec3 {
        let legs = &self.layout.legs;
        if legs.is_empty() {
            return flat(self.body(self.layout.pelvis).position);
        }
        legs.iter()
            .map(|l| flat(self.body(l.end).position))
            .sum::<Vec3>()
            / legs.len() as f32
    }

    /// Ground height under the character: the ground plane, else its lowest foot.
    fn ground(&self) -> f32 {
        let feet = self
            .layout
            .legs
            .iter()
            .map(|l| self.body(l.end).position.y)
            .fold(f32::MAX, f32::min);
        self.world.ground.map_or(feet, |(h, _)| h)
    }

    fn all_bodies(&self) -> std::ops::Range<usize> {
        0..self.rig.bodies.len()
    }

    /// Where leg `i`'s sole points are in the world.
    fn sole(&self, i: usize) -> Vec<Vec3> {
        let foot = self.body(self.layout.legs[i].end);
        self.layout.feet[i]
            .sole
            .iter()
            .map(|&p| foot.position + foot.rotation * p)
            .collect()
    }

    /// The upright pelvis's facing and leg `i`'s outward direction, flat.
    fn facing(&self, upright: Quat, i: usize) -> (Vec3, Vec3) {
        let forward = flat(upright * self.layout.pelvis_forward).normalize_or_zero();
        let outward = flat(upright * self.layout.outward[i]);
        (forward, outward)
    }

    fn upright_pelvis(&self) -> Quat {
        let pelvis = self.body(self.layout.pelvis).rotation;
        Quat::from_rotation_arc((pelvis * self.layout.pelvis_up).normalize(), Vec3::Y) * pelvis
    }

    /// The pelvis upright and facing the way foot `i` points. On one leg nothing else
    /// holds the pelvis's heading: kept as it is, it spins about the stance leg as the
    /// other swings.
    fn upright_over_foot(&self, i: usize) -> Quat {
        let upright = self.upright_pelvis();
        let foot = self.body(self.layout.legs[i].end).rotation;
        let toes = flat(foot * self.layout.feet[i].forward);
        let facing = flat(upright * self.layout.pelvis_forward);
        match (toes.try_normalize(), facing.try_normalize()) {
            (Some(toes), Some(facing)) => Quat::from_rotation_arc(facing, toes) * upright,
            _ => upright,
        }
    }

    /// Where leg `i` should put the middle of its foot down (flat), standing on the
    /// other foot (`stance`, its sole flat), with its own foot's middle now at `from`.
    ///
    /// On one foot the body tips about the edge of that foot nearest the capture point,
    /// and the capture point runs away from that edge exponentially, with time constant
    /// `fall` (√(height / g)). So the foot aims where the capture point will be when it
    /// gets there, estimated from how far it has to go, and a little past it. Never
    /// across the other foot (at least 0.6 hips' width outside it), and within reach.
    fn step_target(
        &self,
        i: usize,
        stance: &[Vec3],
        capture: Vec3,
        from: Vec3,
        fall: f32,
        gait: &Gait,
    ) -> Vec3 {
        let (_, outward) = self.facing(self.upright_pelvis(), i);
        let hips = 2.0 * outward.length();
        let outward = outward.normalize_or_zero();
        let middle = stance.iter().sum::<Vec3>() / stance.len().max(1) as f32;
        let edge = closest(stance, capture);
        let mut target = capture;
        for _ in 0..2 {
            let eta = (STEP_TIME + from.distance(target) / FOOT_SPEED).min(MAX_AHEAD);
            let ahead = edge + (capture - edge) * (eta / fall.max(0.05)).exp();
            target = ahead + (ahead - middle).normalize_or_zero() * gait.overshoot;
        }
        let across = (target - middle).dot(outward);
        target += outward * (0.6 * hips - across).max(0.0);
        let hip = flat(self.pivot(self.layout.legs[i].upper));
        hip + (target - hip).clamp_length_max(0.6 * self.leg_length(i))
    }

    /// World rotations of leg `i`'s thigh and shin that put its ankle at `ankle_at`, as
    /// near as the leg reaches from the hip, the knee bending forwards: the triangle of
    /// thigh, shin and hip-to-ankle.
    fn bend_leg(&self, i: usize, upright: Quat, ankle_at: Vec3) -> (Quat, Quat) {
        let leg = self.layout.legs[i];
        let (hip, knee, ankle) = (
            self.pivot(leg.upper),
            self.pivot(leg.lower),
            self.pivot(leg.end),
        );
        let (thigh_len, shin_len) = (hip.distance(knee), knee.distance(ankle));
        let reach = ankle_at - hip;
        let span = reach
            .length()
            .clamp(0.3 * (thigh_len + shin_len), thigh_len + shin_len);
        let along = reach.normalize_or(Vec3::NEG_Y);
        let cos = ((thigh_len.powi(2) + span.powi(2) - shin_len.powi(2))
            / (2.0 * thigh_len * span))
            .clamp(-1.0, 1.0);
        let (forward, _) = self.facing(upright, i);
        let bend = (forward - along * forward.dot(along)).normalize_or(forward);
        let knee_at = hip + (along * cos + bend * (1.0 - cos * cos).sqrt()) * thigh_len;
        let ankle_at = hip + along * span;
        let thigh = arc(knee - hip, knee_at - hip) * self.body(leg.upper).rotation;
        let shin = arc(ankle - knee, ankle_at - knee_at) * self.body(leg.lower).rotation;
        (thigh, shin)
    }

    fn leg_length(&self, i: usize) -> f32 {
        let leg = self.layout.legs[i];
        let (hip, knee, ankle) = (
            self.pivot(leg.upper),
            self.pivot(leg.lower),
            self.pivot(leg.end),
        );
        hip.distance(knee) + knee.distance(ankle)
    }

    /// Damps the muscle moving `b` critically for the limb it swings (`limb`, the bodies
    /// beyond the joint). The rig's presets are tuned to hold poses against the ground,
    /// and are several times overdamped for a leg in the air, which then swings too
    /// slowly to step.
    fn damp_for_swing(&mut self, b: usize, limb: &[usize]) {
        let Some(j) = self.layout.joint_of[b] else {
            return;
        };
        let pivot = self.pivot(b);
        let inertia: f32 = limb
            .iter()
            .map(|&l| self.rig.bodies[l].mass * self.body(l).position.distance_squared(pivot))
            .sum::<f32>()
            .max(1e-3);
        let critical = |g: Gains| Gains {
            damping: g.damping.min(2.0 * (g.stiffness * inertia).sqrt()),
            ..g
        };
        self.gains[j] = MuscleGains {
            twist: critical(self.gains[j].twist),
            swing: critical(self.gains[j].swing),
        };
    }

    /// Swings leg `i` to put the middle of its foot (now at `sole`, flat) down at
    /// `target` (flat), on the ground at height `floor`. The foot is lifted
    /// `CLEARANCE` while it's away from the target and lowered as it gets there, kept
    /// level. The hip aims the thigh with the pelvis `upright`.
    fn swing(
        &mut self,
        i: usize,
        upright: Quat,
        target: Vec3,
        floor: f32,
        sole: Vec3,
        phase: Phase,
    ) {
        let leg = self.layout.legs[i];
        if matches!(phase, Phase::Lift) {
            // Snatched up hard: the body often still leans on this foot, and a gentle
            // lift only sinks the hips onto it.
            self.preset("Strong", 1.0, [leg.upper, leg.lower, leg.end].into_iter());
        }
        self.damp_for_swing(leg.upper, &[leg.upper, leg.lower, leg.end]);
        self.damp_for_swing(leg.lower, &[leg.lower, leg.end]);
        self.damp_for_swing(leg.end, &[leg.end]);
        let foot = &self.layout.feet[i];
        let (hip, ankle) = (self.pivot(leg.upper), self.pivot(leg.end));
        let length = self.leg_length(i);
        // Where the ankle goes for the middle of the foot to be over the target.
        let mut goal = target - (sole - flat(ankle));
        let away = flat(ankle).distance(goal);
        let mut lift = CLEARANCE * length * ((away - 0.02) / 0.06).clamp(0.0, 1.0);
        match phase {
            // Still on the ground: lift it first, straight up, rather than drag it.
            Phase::Lift => {
                goal = flat(ankle);
                lift = LIFT_OFF * length;
            }
            Phase::Carry => {}
            Phase::Land => {
                goal = flat(ankle);
                lift = 0.0;
            }
        }
        // Heights are to the lowest point of the sole, so a tilted foot clears the
        // ground too; the foot is aimed a centimetre into it so it lands firmly.
        let low = self.sole(i).iter().map(|p| p.y).fold(f32::MAX, f32::min);
        let y = floor + (ankle.y - low).max(foot.ankle_height) + lift - 0.01;
        if matches!(phase, Phase::Land) {
            // Landing, no further from the hip than the leg reaches the ground, or the
            // foot hangs in the air short of it.
            let reach = (length * length - (hip.y - y).powi(2)).max(0.0).sqrt();
            goal = flat(hip) + (goal - flat(hip)).clamp_length_max(reach);
        }
        let (thigh, shin) = self.bend_leg(i, upright, goal.with_y(y));
        // The foot level, aimed from the shin as it is: aimed from where the shin should
        // be, the toes drop while the shin lags behind and catch the ground.
        let (shin_now, foot_now) = (self.body(leg.lower).rotation, self.body(leg.end).rotation);
        let level = arc(foot_now * foot.up, Vec3::Y) * foot_now;
        self.aim(leg.upper, upright, thigh);
        self.aim(leg.lower, thigh, shin);
        self.aim(leg.end, shin_now, level);
    }
}

/// The shortest rotation taking direction `from` to direction `to`.
fn arc(from: Vec3, to: Vec3) -> Quat {
    Quat::from_rotation_arc(from.normalize_or(Vec3::NEG_Y), to.normalize_or(Vec3::NEG_Y))
}

/// Used when a rig has no muscle presets.
const STEADY: MuscleGains = MuscleGains {
    twist: Gains {
        stiffness: 1000.0,
        damping: 500.0,
        force_limit: 1000.0,
    },
    swing: Gains {
        stiffness: 1000.0,
        damping: 500.0,
        force_limit: 1000.0,
    },
};

fn flat(v: Vec3) -> Vec3 {
    v.with_y(0.0)
}

/// Furthest the centre of mass is asked to lean away from straight over the feet, m.
const MAX_SHIFT: f32 = 0.15;

/// What a swinging leg is doing.
#[derive(Clone, Copy, Debug)]
enum Phase {
    /// The foot is still on the ground: lift it.
    Lift,
    /// Carry it towards where it should go.
    Carry,
    /// Put it down where it is.
    Land,
}

/// Least time a step takes, s: lifting the foot and putting it down.
const STEP_TIME: f32 = 0.1;
/// How fast a swinging foot travels, m/s, to estimate when it will land.
const FOOT_SPEED: f32 = 2.0;
/// Furthest ahead (s) a step looks for where the capture point will be. Looking
/// further makes long steps that the body then sways back from.
const MAX_AHEAD: f32 = 0.25;

/// How a balancing behaviour steps.
struct Gait {
    /// How far (m) the capture point may leave the planted feet before a step starts.
    margin: f32,
    /// How far past where the capture point will be the swing foot is aimed, m.
    overshoot: f32,
    /// Strength of the trunk, head and arms, as a fraction of the "Normal" preset.
    trunk: f32,
}

/// Balance stands its ground while the ankles can hold it, and steps a little past
/// the capture point when they can't.
const BALANCE: Gait = Gait {
    margin: 0.02,
    overshoot: 0.05,
    trunk: 1.0,
};

/// Stagger steps as soon as the capture point leaves the feet, only as far as it, so
/// it keeps stepping to keep up with a push, with a looser trunk.
const STAGGER: Gait = Gait {
    margin: 0.0,
    overshoot: 0.0,
    trunk: 0.35,
};

/// A foot whose sole is within this height (m) of the lowest sole carries weight.
const PLANTED: f32 = 0.02;

/// How high a swing foot is carried, as a fraction of the leg's length.
const CLEARANCE: f32 = 0.1;

/// How high a foot still on the ground is asked to lift, as a fraction of the leg's
/// length: higher than it is carried, so it comes off the ground quickly.
const LIFT_OFF: f32 = 0.15;

/// A swinging foot whose step would leave the capture point more than this far (m)
/// outside the feet can't help: it is put straight down, so the other leg can step.
const USELESS: f32 = 0.05;

/// Balance: the ankle strategy while the capture point stays over the feet, stepping
/// when it doesn't (`keep_balance`).
fn balance(d: &mut Drive) {
    keep_balance(d, &BALANCE);
}

/// Stagger: balance whose support moves. It steps whenever the capture point leaves
/// the feet, puts the foot down just where the capture point will be, and lets the
/// trunk sway.
fn stagger(d: &mut Drive) {
    keep_balance(d, &STAGGER);
}

/// Balance on the capture point (position + velocity × √(height / g)), the point where
/// the centre of mass would come to rest given its speed.
///
/// While it is over the planted feet, the ankle strategy (`ankle_strategy`) pulls the
/// body back over them, exactly as Balance did before it could step. When it leaves
/// them by more than the gait's margin, the leg whose step would best cover it swings
/// (`step_target`, `swing`), while the other leg keeps the ankle strategy over its own
/// foot. A swinging foot is put down where it is as soon as that would hold the
/// capture point, or if its step can't. Then it starts again, with another step if the
/// capture point is still outside the feet. Standing safely on one foot, the other is
/// put down beside it.
///
/// Nothing is remembered between ticks: a foot more than `PLANTED` above the other is
/// in the air, and the leg in the air is the one stepping. Where a step goes, and
/// whether one is needed, is worked out afresh from the bodies each tick.
fn keep_balance(d: &mut Drive, gait: &Gait) {
    let layout = d.layout;
    let legs = layout.legs.clone();
    if legs.len() < 2 || layout.feet.len() < 2 {
        return;
    }
    d.preset("Normal", 1.0, d.all_bodies());
    if gait.trunk != 1.0 {
        d.preset("Normal", gait.trunk, layout.upper_body(d.rig.bodies.len()));
    }

    let soles: Vec<Vec<Vec3>> = (0..2).map(|i| d.sole(i)).collect();
    let low = |s: &[Vec3]| s.iter().map(|p| p.y).fold(f32::MAX, f32::min);
    let floor = low(&soles[0]).min(low(&soles[1]));
    let lift = [low(&soles[0]) - floor, low(&soles[1]) - floor];
    let planted = lift.map(|l| l < PLANTED);
    let flat_sole = |i: usize| flats(&soles[i]);
    let centre = |i: usize| flat(soles[i].iter().sum::<Vec3>() / soles[i].len() as f32);

    let g = d.world.gravity.length().max(1.0);
    let (com, vel) = d.centre_of_mass();
    let fall = ((com.y - floor).max(0.3) / g).sqrt();
    let capture = flat(com + vel * fall);
    let support: Vec<Vec3> = (0..2).filter(|&i| planted[i]).flat_map(flat_sole).collect();

    if outside(&support, capture) <= gait.margin {
        if planted[0] == planted[1] {
            ankle_strategy(d, &legs, d.support());
            return;
        }
        // Safe on one foot: put the other down beside it, hips' width apart.
        let (swing, stance) = if planted[0] { (1, 0) } else { (0, 1) };
        let upright = d.upright_over_foot(stance);
        let (_, out) = d.facing(upright, swing);
        lean_legs_to(
            d,
            &legs[stance..=stance],
            hip_shift(d, centre(stance)),
            upright,
        );
        let home = centre(stance) + out * 2.0;
        d.swing(swing, upright, home, floor, centre(swing), Phase::Carry);
        return;
    }

    // How far outside the feet the capture point would be after leg `i` stepped to
    // `target`, the other standing.
    let miss = |i: usize, target: Vec3| {
        let moved = target - centre(i);
        let after: Vec<Vec3> = flat_sole(1 - i)
            .into_iter()
            .chain(flat_sole(i).into_iter().map(|p| p + moved))
            .collect();
        outside(&after, capture)
    };
    let target = |i: usize| d.step_target(i, &flat_sole(1 - i), capture, centre(i), fall, gait);
    let swing = if planted[0] != planted[1] {
        usize::from(planted[0])
    } else {
        // The leg whose step would cover the capture point; between legs that both
        // would (to within 3 cm), the one carrying less weight (further from the centre
        // of mass) or already lifting.
        let cost = |i: usize| {
            10.0 * (miss(i, target(i)) - 0.03).max(0.0)
                - 0.2 * centre(i).distance(flat(com))
                - lift[i]
        };
        usize::from(cost(1) < cost(0))
    };
    let stance = 1 - swing;
    let to = target(swing);
    let phase = if planted[swing] {
        Phase::Lift
    } else {
        let here: Vec<Vec3> = flat_sole(0).into_iter().chain(flat_sole(1)).collect();
        if outside(&here, capture) < 0.0 || miss(swing, to) > USELESS {
            Phase::Land
        } else {
            Phase::Carry
        }
    };
    let upright = d.upright_over_foot(stance);
    lean_legs_to(
        d,
        &legs[stance..=stance],
        hip_shift(d, centre(stance)),
        upright,
    );
    d.swing(swing, upright, to, floor, centre(swing), phase);
}

/// How far a point (flat) lies outside the convex hull of some points (flat), m;
/// negative inside.
fn outside(points: &[Vec3], p: Vec3) -> f32 {
    let hull = hull(points);
    match hull.len() {
        0 => f32::MAX,
        1 => p.distance(hull[0]),
        2 => {
            let (a, b) = (hull[0], hull[1]);
            let t = ((p - a).dot(b - a) / (b - a).length_squared()).clamp(0.0, 1.0);
            p.distance(a + (b - a) * t)
        }
        n => (0..n)
            .map(|i| {
                let (a, b) = (hull[i], hull[(i + 1) % n]);
                // Anticlockwise in (x, z), the outside is to the right of each edge.
                let e = b - a;
                let out = Vec3::new(e.z, 0.0, -e.x).normalize_or_zero();
                (p - a).dot(out)
            })
            .fold(f32::MIN, f32::max),
    }
}

/// The point of the convex hull of some points (flat) nearest `p` (flat): `p` itself
/// if it is inside.
fn closest(points: &[Vec3], p: Vec3) -> Vec3 {
    let hull = hull(points);
    let on_segment = |a: Vec3, b: Vec3| {
        let t = ((p - a).dot(b - a) / (b - a).length_squared().max(1e-12)).clamp(0.0, 1.0);
        a + (b - a) * t
    };
    match hull.len() {
        0 => p,
        1 => hull[0],
        2 => on_segment(hull[0], hull[1]),
        _ if outside(&hull, p) <= 0.0 => p,
        n => (0..n)
            .map(|i| on_segment(hull[i], hull[(i + 1) % n]))
            .min_by(|a, b| a.distance(p).total_cmp(&b.distance(p)))
            .unwrap_or(p),
    }
}

fn flats(points: &[Vec3]) -> Vec<Vec3> {
    points.iter().map(|&p| flat(p)).collect()
}

/// Convex hull of flat points, in order around it (Andrew's monotone chain).
fn hull(points: &[Vec3]) -> Vec<Vec3> {
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.z.total_cmp(&b.z)));
    sorted.dedup_by(|a, b| a.distance(*b) < 1e-5);
    if sorted.len() < 3 {
        return sorted;
    }
    let turn = |o: Vec3, a: Vec3, b: Vec3| (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
    let mut hull: Vec<Vec3> = Vec::with_capacity(sorted.len() * 2);
    for pass in [sorted.clone(), sorted.into_iter().rev().collect()] {
        let start = hull.len();
        for p in pass {
            while hull.len() >= start + 2
                && turn(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0
            {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    hull
}

/// The ankle strategy: both legs (or the stance leg) are swung about the ankles to
/// carry the hips back over `support`, twice as hard against where the body is heading
/// as against where it is, and the ankle muscles push the body back. The pelvis is held
/// upright. Sideways, the two legs and the pelvis make a parallelogram, so the hips aim
/// for the same lean as the ankles to keep it from folding; fore and aft they only keep
/// the pelvis level, or they'd tip it. The knees keep the pose the timeline asks for.
/// Returns the upright pelvis rotation it aims for.
fn ankle_strategy(d: &mut Drive, legs: &[Limb], support: Vec3) -> Quat {
    let shift = hip_shift(d, support);
    lean_legs(d, legs, shift)
}

/// How far the ankle strategy wants to move the hips to bring the body to rest over
/// `support`.
fn hip_shift(d: &Drive, support: Vec3) -> Vec3 {
    let g = d.world.gravity.length().max(1.0);
    let (com, vel) = d.centre_of_mass();
    let height = (com.y - d.ground()).max(0.3);
    let heading = flat(vel) * (height / g).sqrt();
    (support - flat(com) - heading * 2.0).clamp_length_max(MAX_SHIFT)
}

/// Leans the legs about the ankles to move the hips by `shift`, pelvis upright.
fn lean_legs(d: &mut Drive, legs: &[Limb], shift: Vec3) -> Quat {
    let upright = d.upright_pelvis();
    lean_legs_to(d, legs, shift, upright);
    upright
}

/// Leans the legs about the ankles to move the hips by `shift`, the hips aiming the
/// pelvis at `upright`.
fn lean_legs_to(d: &mut Drive, legs: &[Limb], shift: Vec3, upright: Quat) {
    let side = flat(upright * d.layout.pelvis_side).normalize_or_zero();
    // On one leg there is no parallelogram, and leaning the thigh would only roll the
    // pelvis.
    let sideways = if legs.len() > 1 {
        side * shift.dot(side)
    } else {
        Vec3::ZERO
    };
    for &leg in legs {
        let (hip, ankle) = (d.pivot(leg.upper), d.pivot(leg.end));
        let lean = |by: Vec3| {
            Quat::from_rotation_arc((hip - ankle).normalize(), (hip + by - ankle).normalize())
        };
        let shin = lean(shift) * d.body(leg.lower).rotation;
        let thigh = lean(sideways) * d.body(leg.upper).rotation;
        let foot = d.body(leg.end).rotation;
        d.aim(leg.end, shin, foot);
        d.aim(leg.upper, upright, thigh);
    }
}

/// How far ahead of the feet catch fall reaches for the ground, in chest heights: about
/// where the head would land. Out of reach at first, so the arms point forwards and
/// down at it, and meet the ground braced well ahead of the head as the body pivots.
const REACH: f32 = 1.5;

/// Catch fall. The fall direction is the centre of mass's horizontal velocity (or its
/// lean when nearly still). Both arms straighten towards the ground `REACH` chest
/// heights out that way, shoulder-width apart, at the rig's "Strong" preset, while the
/// legs drop to a fifth of "Normal" so the knees give and the body comes down onto
/// the hands.
fn catch_fall(d: &mut Drive) {
    let layout = d.layout;
    let (arms, legs) = (layout.arms.clone(), layout.legs.clone());
    let Some(chest) = layout.chest.filter(|_| !arms.is_empty()) else {
        return;
    };
    d.preset("Normal", 1.0, d.all_bodies());
    d.preset(
        "Normal",
        0.2,
        legs.iter().flat_map(|l| [l.upper, l.lower, l.end]),
    );
    d.preset("Strong", 1.0, arms.iter().flat_map(|a| [a.upper, a.lower]));

    let ground = d.ground();
    let (com, vel) = d.centre_of_mass();
    let support = d.support();
    let chest_at = d.body(chest).position;
    let lean = flat(com) - support;
    let heading = if flat(vel).length() > 0.3 {
        flat(vel)
    } else {
        lean
    };
    let Some(dir) = heading.try_normalize() else {
        return;
    };
    let landing = support + dir * (chest_at.y - ground).max(0.3) * REACH;

    for arm in arms {
        let shoulder = d.pivot(arm.upper);
        // Keep the hands shoulder-width apart, across the fall direction.
        let side = flat(shoulder - chest_at);
        let side = side - dir * side.dot(dir);
        let hand = (landing + side).with_y(ground + 0.05);
        let want = (hand - shoulder).normalize();
        let elbow = d.pivot(arm.lower);
        let upper = Quat::from_rotation_arc((elbow - shoulder).normalize(), want)
            * d.body(arm.upper).rotation;
        let lower = Quat::from_rotation_arc((d.pivot(arm.end) - elbow).normalize(), want)
            * d.body(arm.lower).rotation;
        let parent = d.parent_rotation(arm.upper);
        d.aim(arm.upper, parent, upper);
        d.aim(arm.lower, upper, lower);
    }
}

/// How far the trunk is bent forwards when curled up, in total over all of its joints
/// (the neck and head included, so the head bows), rad.
const CURL_SPINE: f32 = 0.9;
/// Hip flexion and knee bend when curled up, rad (the engine clamps them to the limits).
const CURL_HIP: f32 = -1.6;
const CURL_KNEE: f32 = 2.0;

/// How one joint of a chain is turned from its rest pose, in rad: about the character's
/// bend axis (fore and aft), its forward axis (sideways) and its up axis (twist).
#[derive(Clone, Copy, Default)]
struct Turn {
    bend: f32,
    side: f32,
    twist: f32,
}

impl Drive<'_> {
    /// Aims a chain of bodies (each hanging from the one before, the first from a body
    /// with rotation `parent`) at their rest pose turned by `turn` at each joint,
    /// measured about the rest bend, forward and up axes. Works from the rig's rest
    /// frames, so it doesn't assume any rest pose.
    fn pose_chain(&mut self, mut parent: Quat, links: &[(usize, Turn)]) {
        let layout = self.layout;
        for &(s, turn) in links {
            let Some(j) = layout.joint_of[s] else { break };
            let p = self.rig.joints[j].parent_body;
            let about = |local: Vec3, angle: f32| {
                Quat::from_axis_angle((parent * local).normalize_or_zero(), angle)
            };
            let child = about(layout.bend_local[p], turn.bend)
                * about(layout.fwd_local[p], turn.side)
                * about(layout.up_local[p], turn.twist)
                * parent
                * layout.rest_rel[s];
            self.aim(s, parent, child);
            parent = child;
        }
    }

    /// Aims an arm's segments along directions given as (up, forward, out) weights in
    /// the chest's frame, one for the upper arm and one for the forearm.
    fn point_arm(&mut self, arm: Limb, frame: (Vec3, Vec3, Vec3), upper: Vec3, lower: Vec3) {
        let (up, fwd, out) = frame;
        let dir = |w: Vec3| (up * w.x + fwd * w.y + out * w.z).try_normalize();
        let (shoulder, elbow) = (self.pivot(arm.upper), self.pivot(arm.lower));
        let upper = dir(upper).map_or(self.body(arm.upper).rotation, |want| {
            Quat::from_rotation_arc((elbow - shoulder).normalize(), want)
                * self.body(arm.upper).rotation
        });
        let lower = dir(lower).map_or(self.body(arm.lower).rotation, |want| {
            Quat::from_rotation_arc((self.pivot(arm.end) - elbow).normalize(), want)
                * self.body(arm.lower).rotation
        });
        let parent = self.parent_rotation(arm.upper);
        self.aim(arm.upper, parent, upper);
        self.aim(arm.lower, upper, lower);
    }

    /// The chest's up, forward and outwards (towards `arm`'s side) directions now.
    fn chest_frame(&self, chest: usize, arm: Limb) -> (Vec3, Vec3, Vec3) {
        let rot = self.body(chest).rotation;
        let up = (rot * self.layout.up_local[chest]).normalize();
        let fwd = (rot * self.layout.fwd_local[chest]).normalize();
        let left = up.cross(fwd);
        let sign = (self.pivot(arm.upper) - self.body(chest).position).dot(left);
        (up, fwd, if sign < 0.0 { -left } else { left })
    }

    /// Aims the whole body at the curled-up pose scaled by `k` (1: fully curled), with
    /// `wander` added to the targets.
    fn curl(&mut self, k: f32, wander: Option<Wander>) {
        let layout = self.layout;
        let w = |i: usize, amount: f32| wander.map_or(0.0, |w| w.at(i) * amount);
        let pelvis = self.body(layout.pelvis).rotation;
        let spine: Vec<(usize, Turn)> = layout
            .spine
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let turn = Turn {
                    bend: k * CURL_SPINE / layout.spine.len() as f32 + w(i, 0.25),
                    side: w(i + 8, 0.3),
                    twist: w(i + 16, 0.2),
                };
                (s, turn)
            })
            .collect();
        self.pose_chain(pelvis, &spine);
        for (l, leg) in layout.legs.iter().enumerate() {
            let at = 24 + 8 * l;
            let hip = Turn {
                bend: k * CURL_HIP + w(at, 0.5),
                side: w(at + 1, 0.35),
                twist: 0.0,
            };
            let knee = Turn {
                bend: k * CURL_KNEE + w(at + 2, 0.5),
                ..Turn::default()
            };
            let links = [
                (leg.upper, hip),
                (leg.lower, knee),
                (leg.end, Turn::default()),
            ];
            self.pose_chain(pelvis, &links);
        }
        let Some(chest) = layout.chest else { return };
        for (a, &arm) in layout.arms.iter().enumerate() {
            let at = 48 + 8 * a;
            let frame = self.chest_frame(chest, arm);
            // Between hanging down (k = 0) and over the head with the forearm folded
            // back towards it (k = 1), as (up, forward, out) weights.
            let sway = |i: usize| Vec3::new(w(i, 0.5), w(i + 1, 0.5), w(i + 2, 0.5));
            let hang = Vec3::new(-1.0, 0.0, 0.0);
            let upper = hang.lerp(Vec3::new(0.7, 0.6, 0.4), k) + sway(at);
            let lower = hang.lerp(Vec3::new(0.2, -0.5, -0.6), k) + sway(at + 3);
            self.point_arm(arm, frame, upper, lower);
        }
    }
}

/// Slow, deterministic wandering: sines of 0.5 to 1.5 rad/s, a different rate and phase
/// for each numbered joint motion.
#[derive(Clone, Copy)]
struct Wander {
    elapsed: f32,
}

impl Wander {
    fn at(self, i: usize) -> f32 {
        let rate = 0.5 + ((i * 7) % 11) as f32 * 0.1;
        (rate * self.elapsed + i as f32 * 2.4).sin()
    }
}

/// Body foetal: curl up. The trunk bends forwards by `CURL_SPINE` in all, shared by its
/// joints so the head bows too; the hips flex by `CURL_HIP` and the knees by `CURL_KNEE`;
/// the arms go over the head, upper arms forwards, up and out, forearms folded back
/// towards it. The trunk and hips use the rig's "Strong" preset (else "Normal"), the
/// rest "Normal". Joint limits clamp the targets, so a rig that can't curl so far
/// curls as far as it can.
fn body_foetal(d: &mut Drive) {
    let layout = d.layout;
    d.preset("Normal", 1.0, d.all_bodies());
    d.preset_of(
        &["Strong", "Normal"],
        1.0,
        layout
            .spine
            .iter()
            .copied()
            .chain(layout.legs.iter().map(|l| l.upper)),
    );
    d.curl(1.0, None);
}

/// Writhe: low-strength targets wandering around a half-curled pose. Every joint motion
/// follows a slow sine of its own rate and phase of the time since the behaviour began,
/// so it keeps moving without any randomness. The trunk uses 0.6 of "Normal" and the
/// limbs 0.3, so the trunk leads and the limbs flail along.
fn writhe(d: &mut Drive) {
    let layout = d.layout;
    d.preset("Normal", 0.3, d.all_bodies());
    d.preset("Normal", 0.6, layout.spine.iter().copied());
    let elapsed = d.elapsed;
    d.curl(0.5, Some(Wander { elapsed }));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::document::sample::{humanoid_rig, humanoid_skeleton};

    #[test]
    fn names_match_endorphins() {
        assert_eq!(Behaviour::from_name("Balance"), Some(Behaviour::Balance));
        assert_eq!(
            Behaviour::from_name("CatchFall"),
            Some(Behaviour::CatchFall)
        );
        assert_eq!(
            Behaviour::from_name("catch fall"),
            Some(Behaviour::CatchFall)
        );
        assert_eq!(Behaviour::from_name("Writhe"), Some(Behaviour::Writhe));
        assert_eq!(Behaviour::from_name("writhe"), Some(Behaviour::Writhe));
        for name in ["BodyFoetal", "body foetal", "Foetal", "fetal", "BodyFetal"] {
            assert_eq!(Behaviour::from_name(name), Some(Behaviour::BodyFoetal));
        }
        assert_eq!(Behaviour::from_name("Stagger"), Some(Behaviour::Stagger));
        // As endorphin's scenes name them: the set's name, then the version.
        assert_eq!(
            Behaviour::from_name("Bob2: Balance 2.5"),
            Some(Behaviour::Balance)
        );
        assert_eq!(
            Behaviour::from_name("Bob2: CatchFall"),
            Some(Behaviour::CatchFall)
        );
        assert_eq!(
            Behaviour::from_name("Bob2: Stagger 2.0"),
            Some(Behaviour::Stagger)
        );
        assert_eq!(
            Behaviour::from_name("Bob2: BodyFoetal"),
            Some(Behaviour::BodyFoetal)
        );
        assert_eq!(
            Behaviour::from_name("Bob2: BodyWrithe"),
            Some(Behaviour::Writhe)
        );
        assert_eq!(Behaviour::from_name("Bob2: Look and Reach 2.0"), None);
        assert_eq!(Behaviour::from_name("Bob2: Explosion 2.0"), None);
    }

    /// A rig whose joints loop (only possible from a malformed file) must not hang.
    #[test]
    fn looping_rig_does_not_hang() {
        let skeleton = humanoid_skeleton();
        let mut rig = humanoid_rig(&skeleton);
        let (knee, ankle) = (
            rig.body_index("l_knee").unwrap(),
            rig.body_index("l_ankle").unwrap(),
        );
        let mut back = rig.joints.iter().find(|j| j.body == ankle).unwrap().clone();
        (back.body, back.parent_body) = (knee, ankle);
        rig.joints.push(back);
        Layout::new(&rig, &skeleton);
    }

    #[test]
    fn finds_the_placeholders_feet() {
        let skeleton = humanoid_skeleton();
        let rig = humanoid_rig(&skeleton);
        let layout = Layout::new(&rig, &skeleton).unwrap();
        // Facing +Z with the left hip at +X: forward is side × up (in the pelvis's frame).
        let (up, side) = (layout.pelvis_up, layout.pelvis_side);
        assert!(layout.pelvis_forward.abs_diff_eq(side.cross(up), 1e-4));
        let left = layout
            .legs
            .iter()
            .position(|l| rig.bodies[l.upper].name == "l_hip")
            .unwrap();
        assert!(layout.outward[left].dot(side) > 0.09);
        assert!(layout.outward[1 - left].dot(side) < -0.09);
        for foot in &layout.feet {
            // The foot box's four bottom corners: heel 6 cm behind the ankle, toes
            // 18 cm ahead, 7 cm below it.
            assert_eq!(foot.sole.len(), 4);
            let (heel, toe) = foot
                .sole
                .iter()
                .fold((f32::MAX, f32::MIN), |(h, t), p| (h.min(p.z), t.max(p.z)));
            assert!((toe - heel - 0.24).abs() < 1e-4, "{heel} {toe}");
            assert!((foot.ankle_height - 0.07).abs() < 1e-4);
        }
    }

    #[test]
    fn finds_the_placeholders_limbs() {
        let skeleton = humanoid_skeleton();
        let rig = humanoid_rig(&skeleton);
        let layout = Layout::new(&rig, &skeleton).unwrap();
        let name = |b: usize| rig.bodies[b].name.as_str();
        assert_eq!(name(layout.pelvis), "hips");
        assert_eq!(layout.chest.map(name), Some("chest"));
        let mut legs: Vec<_> = layout
            .legs
            .iter()
            .map(|l| [name(l.upper), name(l.lower), name(l.end)])
            .collect();
        legs.sort();
        assert_eq!(
            legs,
            [
                ["l_hip", "l_knee", "l_ankle"],
                ["r_hip", "r_knee", "r_ankle"]
            ]
        );
        let mut arms: Vec<_> = layout
            .arms
            .iter()
            .map(|a| [name(a.upper), name(a.lower), name(a.end)])
            .collect();
        arms.sort();
        assert_eq!(
            arms,
            [
                ["l_shoulder", "l_elbow", "l_wrist"],
                ["r_shoulder", "r_elbow", "r_wrist"]
            ]
        );
    }

    /// The arms of rigs with collarbones, and with forearms in two bodies as on
    /// endorphin's characters: upper arm and forearm, not collarbone and upper arm,
    /// though the collarbone and upper arm make the longest pair of segments there.
    #[test]
    fn finds_the_arms_past_collarbones_and_split_forearms() {
        use crate::physics::scenes::{person_rig_with_clavicles, person_rig_with_split_forearms};
        let cases = [
            (person_rig_with_clavicles(), "wrist"),
            (person_rig_with_split_forearms(), "forearm_twist"),
        ];
        for ((skeleton, rig), end) in cases {
            let layout = Layout::new(&rig, &skeleton).unwrap();
            let name = |b: usize| rig.bodies[b].name.clone();
            let mut arms: Vec<_> = layout
                .arms
                .iter()
                .map(|a| [name(a.upper), name(a.lower), name(a.end)])
                .collect();
            arms.sort();
            assert_eq!(
                arms,
                ["l", "r"].map(|s| [
                    format!("{s}_shoulder"),
                    format!("{s}_elbow"),
                    format!("{s}_{end}")
                ]),
            );
        }
    }
}
