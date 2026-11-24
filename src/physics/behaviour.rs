//! Behaviours: controllers that read a simulated character's state every physics tick
//! and set its muscle targets and strengths, like endorphin's balance, stagger and catch
//! fall.
//! endorphin's own behaviours are vendor-protected; these are original designs, run for
//! imported behaviour events of the same name. Design notes: docs/behaviours.md.
//!
//! Controllers are stateless (everything comes from the bodies each tick), so they keep
//! the simulation deterministic.

use std::collections::BTreeMap;

use glam::{Mat3, Quat, Vec3};

use super::sim::rig_bodies;
use super::world::{Body, Joint, World};
use crate::document::events;
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
    /// Reach for a target with either hand and turn the head to look at another
    /// (endorphin's Look and Reach, "Hands reach and look at"); the rest of the body
    /// holds its pose. Its targets and settings come from the event: [`LookAndReach`].
    LookAndReach,
    /// Writhe in mid-air: arms flap up and down from out to the sides, legs pedal,
    /// back arched, until the character nears the ground.
    Explosion(Explosion),
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
            "lookandreach" | "handsreachandlookat" => Some(Behaviour::LookAndReach),
            // endorphin 2.0 calls it Explosion, 2.5 WritheInMidAir; both show as
            // "Writhe in mid-air".
            "explosion" | "writheinmidair" => Some(Behaviour::Explosion(Explosion::default())),
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
            Behaviour::LookAndReach => look_and_reach(d),
            Behaviour::Explosion(settings) => explosion(d, settings),
        }
    }

    /// This behaviour with the settings an event carries (`clip.source`, endorphin's
    /// parameter names); missing or unreadable ones keep their defaults. Look and
    /// Reach's settings name bodies in the world, so it reads them once the world is
    /// built ([`LookAndReach::read`]), through the same [`setting`].
    pub fn with_settings(self, source: &BTreeMap<String, String>) -> Self {
        let number = |key: &str| setting(source, key);
        match self {
            Behaviour::Explosion(e) => Behaviour::Explosion(e.read(number)),
            other => other,
        }
    }
}

/// A number from a behaviour event's settings (`clip.source`: endorphin's parameter
/// names, as the importer keeps them); `None` when missing or unreadable, so the
/// behaviour keeps its default. Every behaviour reads its settings through this and
/// [`setting_flag`].
pub fn setting(source: &BTreeMap<String, String>, key: &str) -> Option<f32> {
    events::number(source, key).filter(|v| v.is_finite())
}

/// A yes/no setting (`true` or `false`, as endorphin writes them); `None` when missing.
pub fn setting_flag(source: &BTreeMap<String, String>, key: &str) -> Option<bool> {
    source
        .get(key)
        .map(|v| v.trim().eq_ignore_ascii_case("true"))
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
    /// What lies beyond each arm's forearm, in the same order as `arms`.
    hands: Vec<Hand>,
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
    /// reaching furthest sideways. Upper and lower limb segments are the longest
    /// consecutive pair along each branch (skipping collarbones, hands and fingers).
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
        let mut hand_chains = Vec::new();
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
                let mut chain = follow(c, &|b| at(b).distance(start));
                let whole = chain.clone();
                // A body that only twists about its own bone (endorphin's forearm twist)
                // continues the segment before it: without this, half forearms made the
                // collarbone and upper arm the longest pair.
                let twist_only = |b: usize, from: usize| {
                    let Some(j) = joint_of[b] else { return false };
                    let jd = &rig.joints[j];
                    let axis = rest_joints[jd.joint].0
                        * Quat::from_array(jd.secondary).normalize()
                        * Vec3::X;
                    let bone = (pivot(b) - pivot(from)).normalize_or_zero();
                    jd.hard.swing1.max(jd.hard.swing2) < 0.05 && axis.dot(bone).abs() > 0.9
                };
                let mut k = 1;
                while k < chain.len() {
                    if twist_only(chain[k], chain[k - 1]) {
                        chain.remove(k);
                    } else {
                        k += 1;
                    }
                }
                // Segment lengths, joint to next joint; pick the longest consecutive pair.
                let length = |i: usize| {
                    chain
                        .get(i + 1)
                        .map_or(0.0, |&next| pivot(chain[i]).distance(pivot(next)))
                };
                let best = (0..chain.len().saturating_sub(2)).max_by(|&a, &b| {
                    (length(a) + length(a + 1)).total_cmp(&(length(b) + length(b + 1)))
                });
                if let Some(i) = best {
                    arms.push(Limb {
                        upper: chain[i],
                        lower: chain[i + 1],
                        end: chain[i + 2],
                    });
                    hand_chains.push(whole);
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
        // Each hand, for reaching: the tip of the arm's furthest body (the fingers), and
        // the arm's upper arm and forearm (`arms`, past a collarbone and a forearm
        // twist) as two bones out to it. What comes before the upper arm is the
        // collarbone; what follows the forearm (a twist, the hand) is carried along.
        let hands = arms
            .iter()
            .zip(hand_chains)
            .map(|(arm, chain)| {
                let start = pivot(chain[0]);
                let tip = chain
                    .iter()
                    .map(|&b| at(b))
                    .max_by(|a, b| a.distance(start).total_cmp(&b.distance(start)))
                    .unwrap_or(start);
                let (upper, lower) = (arm.upper, arm.lower);
                let i = chain.iter().position(|&b| b == upper).unwrap_or(0);
                let after = chain
                    .iter()
                    .position(|&b| b == lower)
                    .map_or(chain.len(), |l| l + 1);
                let (fr, fp) = rest[lower];
                // Where the arm points at rest, shoulder to tip, seen from the trunk body
                // it hangs from.
                let trunk = joint_of[chain[0]].map_or(pelvis, |j| rig.joints[j].parent_body);
                Hand {
                    upper,
                    lower,
                    collar: chain[..i].to_vec(),
                    rest_reach: rest[trunk].0.inverse() * (tip - pivot(upper)).normalize_or_zero(),
                    chain: chain[after..].to_vec(),
                    tip: fr.inverse() * (tip - fp),
                    left: (pivot(arm.upper) - at(pelvis)).dot(Vec3::Y.cross(forward)) > 0.0,
                }
            })
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
            hands,
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

/// The end of an arm, for reaching.
#[derive(Clone, Debug)]
struct Hand {
    /// The upper arm and forearm, as reaching bends them.
    upper: usize,
    lower: usize,
    /// Bodies between the trunk and the upper arm (a collarbone).
    collar: Vec<usize>,
    /// The way the arm points at rest, shoulder to tip, in the frame of the trunk body
    /// it hangs from.
    rest_reach: Vec3,
    /// The bodies past the forearm, outwards: forearm twist, hand, fingers.
    chain: Vec<usize>,
    /// The hand's tip (the centre of its furthest body) in the forearm's frame, wrist
    /// straight as at rest.
    tip: Vec3,
    /// The character's left hand (left is up × the way it faces).
    left: bool,
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
    /// The event's targets and settings, for the behaviours that read them (Look and
    /// Reach); `None` runs a behaviour with its defaults and no target.
    pub settings: Option<&'a LookAndReach>,
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
            let eta = (STEP_TIME + from.distance(target) / FOOT_SPEED).min(gait.ahead);
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
    /// `target` (flat), on the ground at height `floor`. The foot is carried at the
    /// gait's clearance while it's far from the target and comes down along its glide
    /// slope as it gets there, kept level. The hip aims the thigh with the pelvis
    /// `upright`.
    #[allow(clippy::too_many_arguments)]
    fn swing(
        &mut self,
        i: usize,
        upright: Quat,
        target: Vec3,
        floor: f32,
        sole: Vec3,
        phase: Phase,
        gait: &Gait,
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
        // Carried lower when it swings forwards (see `Gait::clearance_forward`), and
        // brought down along the glide slope so it reaches the ground as it reaches
        // the target, 2 cm out.
        let (forward, _) = self.facing(upright, i);
        let ahead = (goal - flat(ankle))
            .normalize_or_zero()
            .dot(forward)
            .max(0.0);
        let clearance = gait.clearance + (gait.clearance_forward - gait.clearance) * ahead * ahead;
        let mut lift = length * clearance.min(gait.glide * (away - 0.02).max(0.0));
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

/// The rotation taking X to `along` and Y to `front` (made square to it), Z to their
/// cross product.
fn basis(along: Vec3, front: Vec3) -> Quat {
    let along = along.normalize_or(Vec3::X);
    let front = front
        .reject_from(along)
        .normalize_or(along.any_orthonormal_vector());
    Quat::from_mat3(&Mat3::from_cols(along, front, along.cross(front))).normalize()
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

/// How a balancing behaviour steps.
struct Gait {
    /// How far (m) the capture point may leave the planted feet before a step starts.
    margin: f32,
    /// Furthest ahead (s) a step looks for where the capture point will be.
    ahead: f32,
    /// How far past where the capture point will be the swing foot is aimed, m.
    overshoot: f32,
    /// How high a swing foot is carried, as a fraction of the leg's length.
    clearance: f32,
    /// The same for a foot swinging straight forwards (in between for steps at an
    /// angle).
    clearance_forward: f32,
    /// How steeply the swing foot comes down as it nears its target: its height, as a
    /// fraction of the leg's length, per metre it still has to go.
    glide: f32,
    /// Strength of the trunk, head and arms, as a fraction of the "Normal" preset.
    trunk: f32,
}

/// Balance stands its ground while the ankles can hold it, and steps a little past
/// the capture point when they can't. It looks 0.25 s ahead: looking further made
/// long steps that the body then swayed back from.
///
/// Its feet are carried high, a quarter of the leg's length, and brought down along a
/// gentle slope. Carried at a tenth and dropped over the last 6 cm, as they were, a
/// swing foot often sagged back to within 2 cm of the ground, counted as planted and
/// started lifting again, and the step came too late (docs/behaviours.md). Forward
/// steps are carried at 15%: carried as high as the others, about 15 of the sweep's
/// 21 forward pushes were held instead of 19, for a reason not yet pinned down.
const BALANCE: Gait = Gait {
    margin: 0.02,
    ahead: 0.25,
    overshoot: 0.05,
    clearance: 0.25,
    clearance_forward: 0.15,
    glide: 0.9,
    trunk: 1.0,
};

/// Stagger steps as soon as the capture point leaves the feet, aims where the capture
/// point will be only half as far ahead as Balance does, so its steps fall short and
/// it keeps stepping to keep up with a push, shuffles (feet carried low, as Balance's
/// were before), and has a looser trunk.
const STAGGER: Gait = Gait {
    margin: 0.0,
    ahead: 0.125,
    overshoot: 0.05,
    clearance: 0.1,
    clearance_forward: 0.1,
    glide: 0.1 / 0.06,
    trunk: 0.35,
};

/// A foot whose sole is within this height (m) of the lowest sole carries weight.
const PLANTED: f32 = 0.02;

/// How high a foot still on the ground is asked to lift, straight up, as a fraction of
/// the leg's length, so it comes off the ground quickly.
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
/// the feet, puts the foot down short of where the capture point is going, and lets
/// the trunk sway.
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
        d.swing(
            swing,
            upright,
            home,
            floor,
            centre(swing),
            Phase::Carry,
            gait,
        );
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
    d.swing(swing, upright, to, floor, centre(swing), phase, gait);
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

// ---------------------------------------------------------------------------------
// Look and Reach

/// A point to reach for or look at: a world body and a point in its frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aim {
    /// Index into the world's bodies.
    pub body: usize,
    /// The point in the body's frame (its centre, or its collision shape's).
    pub offset: Vec3,
}

/// How one hand reaches, from a Look and Reach event's `LeftHand_...` or
/// `RightHand_...` settings. A setting the event doesn't carry takes the value most of
/// endorphin's scenes give it.
#[derive(Clone, Debug, PartialEq)]
pub struct Reach {
    /// What to reach for (`...HandTarget`); `None`: the hand keeps its pose.
    pub target: Option<Aim>,
    /// Arm muscle strength, a fraction of the rig's "Strong" preset (`Strength`, 0.8).
    pub strength: f32,
    /// How fast the hand goes for the target, 0 to 1 (`Speed`, 0.8).
    pub speed: f32,
    /// Seconds after the event starts before the hand moves (`Delay`, 0).
    pub delay: f32,
    /// Seconds over which it then takes over from the pose (`Blend`, 0).
    pub blend: f32,
    /// Furthest the hand reaches, a fraction of the arm's length (`MaxExtension`, 1).
    pub max_extension: f32,
    /// Added to the target point, world axes, m (`CatchOffsetX/Y/Z`, 0).
    pub offset: Vec3,
    /// Aim where a moving target will be when the hand gets there (`UsePrediction`,
    /// false).
    pub predict: bool,
    /// How much the wrist and fingers are held straight along the forearm, 0 to 1:
    /// `Orientation` (0.5) when `applyHandOrientation` (true) is set, else 0.
    pub orientation: f32,
}

/// Where the head looks, from the event's `LookAt...` settings.
#[derive(Clone, Debug, PartialEq)]
pub struct Look {
    /// What to look at (`LookAtTarget`); `None`: the head keeps its pose.
    pub target: Option<Aim>,
    /// How fast the head turns, 0 to 1 (`LookAtSpeed`, 0.5).
    pub speed: f32,
    /// Seconds before it turns (`LookAtDelay`, 0) and over which it takes over
    /// (`LookAtBlend`, 0).
    pub delay: f32,
    pub blend: f32,
    /// The world direction the top of the head keeps to (`LookAt_HeadUpVectorX/Y/Z`,
    /// up) when `LookAt_UseUpVector` (true) is set; `None`: it keeps the roll it has.
    pub up: Option<Vec3>,
}

/// A Look and Reach event's settings and targets: [`Behaviour::LookAndReach`].
#[derive(Clone, Debug, PartialEq)]
pub struct LookAndReach {
    pub left: Reach,
    pub right: Reach,
    pub look: Look,
    /// Strength of the legs and trunk, which hold the timeline's pose, as a fraction of
    /// the rig's "Normal" preset (`LegTorsoStiffness`, 0.8).
    pub leg_torso: f32,
}

impl LookAndReach {
    /// Reads an event's settings, as imported from endorphin (`LeftHand_Speed`, ...).
    /// `aim` turns a target as the scene names it (`character.MetaBodies.Body`, or a
    /// collision surface's path) into a point on a body in the world.
    pub fn read(settings: &BTreeMap<String, String>, aim: &dyn Fn(&str) -> Option<Aim>) -> Self {
        let number = |key: &str, default: f32| setting(settings, key).unwrap_or(default);
        let flag = |key: &str, default: bool| setting_flag(settings, key).unwrap_or(default);
        // A selection may list several; the first that resolves is used.
        let target = |key: &str| {
            settings
                .get(key)
                .and_then(|list| list.split(',').find_map(|t| aim(t.trim())))
        };
        let hand = |side: &str| {
            let key = |name: &str| format!("{side}Hand_{name}");
            Reach {
                target: target(&format!("{side}HandTarget")),
                strength: number(&key("Strength"), 0.8).clamp(0.0, 2.0),
                speed: number(&key("Speed"), 0.8).clamp(0.0, 1.0),
                delay: number(&key("Delay"), 0.0).max(0.0),
                blend: number(&key("Blend"), 0.0).max(0.0),
                max_extension: number(&key("MaxExtension"), 1.0).clamp(0.1, 1.0),
                offset: Vec3::new(
                    number(&key("CatchOffsetX"), 0.0),
                    number(&key("CatchOffsetY"), 0.0),
                    number(&key("CatchOffsetZ"), 0.0),
                ),
                predict: flag(&key("UsePrediction"), false),
                orientation: if flag(&key("applyHandOrientation"), true) {
                    number(&key("Orientation"), 0.5).clamp(0.0, 1.0)
                } else {
                    0.0
                },
            }
        };
        let up = Vec3::new(
            number("LookAt_HeadUpVectorX", 0.0),
            number("LookAt_HeadUpVectorY", 1.0),
            number("LookAt_HeadUpVectorZ", 0.0),
        );
        LookAndReach {
            left: hand("Left"),
            right: hand("Right"),
            look: Look {
                target: target("LookAtTarget"),
                speed: number("LookAtSpeed", 0.5).clamp(0.0, 1.0),
                delay: number("LookAtDelay", 0.0).max(0.0),
                blend: number("LookAtBlend", 0.0).max(0.0),
                up: flag("LookAt_UseUpVector", true)
                    .then(|| up.try_normalize())
                    .flatten(),
            },
            leg_torso: number("LegTorsoStiffness", 0.8).clamp(0.0, 2.0),
        }
    }
}

/// How far ahead of the hand (m) the point its arm aims for is put, at full speed. The
/// muscles pull the hand towards a point at most this far ahead, which sets how fast
/// it travels. Fitted to endorphin's run of "Hands reach and look at" (behaviours.md).
const REACH_LEAD: f32 = 0.8;
/// How far ahead of the head (rad) its aim is turned, at full speed.
const LOOK_LEAD: f32 = 1.5;
/// The furthest the neck and head together turn from the chest to look, rad.
const MAX_LOOK: f32 = 1.4;
/// How far apart (m) two hands reaching for the same point are aimed: close, as
/// endorphin's hands come together on a target.
const HAND_GAP: f32 = 0.08;
/// The share of an arm's turn towards its aim that the collarbone takes.
const COLLAR_SHARE: f32 = 0.3;
/// Speed (m/s) a predicting hand is taken to travel at, to work out when it gets there.
const HAND_SPEED: f32 = 2.0;
/// Furthest ahead (s) a moving target is predicted.
const MAX_PREDICT: f32 = 0.5;

/// How far a part with a delay and a blend (s) has taken over at `elapsed` s, 0 to 1.
fn ramp(elapsed: f32, delay: f32, blend: f32) -> f32 {
    if blend > 0.0 {
        ((elapsed - delay) / blend).clamp(0.0, 1.0)
    } else if elapsed >= delay {
        1.0
    } else {
        0.0
    }
}

/// Look and Reach. Each hand with a target reaches for it, and the head turns to look
/// at its own target, while the legs and trunk hold the timeline's pose at
/// `leg_torso` × "Normal".
///
/// Reaching: the target point (plus the catch offset, and moved on to where a moving
/// target will be if the hand predicts) is brought within `max_extension` of the
/// arm's length from the shoulder. The arm bends to put the hand's tip there, the
/// elbow out and down from the chest (two bones: the upper arm, and the forearm out to
/// the tip). The point the arm aims for is never more than `REACH_LEAD` × speed ahead
/// of where the tip is, so the muscles carry the hand there at a pace `speed` sets.
/// The wrist and fingers are held straight along the forearm by `orientation`. Arm
/// muscles use the rig's "Strong" preset × strength.
///
/// Looking: the head turns to face its target, the top of the head towards the up
/// vector, at most `MAX_LOOK` from straight on the chest. The neck and head joints
/// share the turn, and the head is never aimed more than `LOOK_LEAD` × speed past
/// where it is.
///
/// Delay and blend ease each part in. Everything is worked out afresh from the bodies
/// each tick.
fn look_and_reach(d: &mut Drive) {
    let layout = d.layout;
    let Some(chest) = layout.chest else { return };
    let defaults;
    let settings = match d.settings {
        Some(s) => s,
        None => {
            defaults = LookAndReach::read(&Default::default(), &|_| None);
            &defaults
        }
    };
    // The legs and trunk hold their pose at `leg_torso`. Arms and neck keep what the
    // timeline gives them until a reach or the look takes them over: Heli Leap joins a
    // hand that only reaches later to another character's, and an arm held stiffly in
    // its old pose pulled the join 20 cm apart.
    let mut own: Vec<usize> = layout
        .hands
        .iter()
        .flat_map(|h| h.collar.iter().chain([&h.upper, &h.lower]).chain(&h.chain))
        .copied()
        .collect();
    if let Some(from) = layout.spine.iter().position(|&b| b == chest) {
        own.extend(&layout.spine[from + 1..]);
    }
    let trunk_and_legs: Vec<usize> = d.all_bodies().filter(|b| !own.contains(b)).collect();
    d.preset("Normal", settings.leg_torso, trunk_and_legs.into_iter());
    let reaches: Vec<(Limb, &Hand, &Reach)> = layout
        .arms
        .iter()
        .zip(&layout.hands)
        .map(|(&arm, hand)| {
            let reach = if hand.left {
                &settings.left
            } else {
                &settings.right
            };
            (arm, hand, reach)
        })
        .collect();
    // Where each hand is going, to keep two hands going for one point apart.
    let goals: Vec<Option<Vec3>> = reaches
        .iter()
        .map(|&(_, _, reach)| reach.target.map(|aim| d.reach_point(aim, reach)))
        .collect();
    for (i, &(arm, hand, reach)) in reaches.iter().enumerate() {
        let Some(goal) = goals[i] else { continue };
        let w = ramp(d.elapsed, reach.delay, reach.blend);
        if w <= 0.0 {
            continue;
        }
        let shared = goals
            .iter()
            .enumerate()
            .any(|(j, g)| j != i && g.is_some_and(|g| g.distance(goal) < HAND_GAP));
        d.reach(chest, arm, hand, reach, goal, shared, w);
    }
    d.look(chest, &settings.look);
}

impl Drive<'_> {
    /// Where a hand reaching for `aim` goes: the target point plus the catch offset,
    /// moved on to where the target will be when the hand gets there if it predicts.
    fn reach_point(&self, aim: Aim, reach: &Reach) -> Vec3 {
        let body = &self.world.bodies[aim.body];
        let mut at = body.position + body.rotation * aim.offset + reach.offset;
        if reach.predict {
            let chest = self.layout.chest.map_or(at, |c| self.body(c).position);
            let ahead = (at.distance(chest) / HAND_SPEED).min(MAX_PREDICT);
            at += body.linear_velocity * ahead;
        }
        at
    }

    /// Bends `arm` to put the hand's tip at `goal` (see `look_and_reach`), by `w`
    /// (0: the timeline's pose, 1: reaching). `shared`: the other hand goes for the
    /// same point, so the two are kept a little apart.
    #[allow(clippy::too_many_arguments)]
    fn reach(
        &mut self,
        chest: usize,
        arm: Limb,
        hand: &Hand,
        reach: &Reach,
        goal: Vec3,
        shared: bool,
        w: f32,
    ) {
        let layout = self.layout;
        let (up, _, out) = self.chest_frame(chest, arm);
        let shoulder = self.pivot(hand.upper);
        let elbow = self.pivot(hand.lower);
        let upper_rot = self.body(hand.upper).rotation;
        let forearm = self.body(hand.lower);
        let tip = forearm.position + forearm.rotation * hand.tip;
        let forearm_rot = forearm.rotation;
        let upper_len = elbow.distance(shoulder);
        let lower_len = tip.distance(elbow);
        let mut goal = goal;
        if shared {
            // Apart across the direction of reach, each hand on its own side.
            let dir = (goal - shoulder).normalize_or_zero();
            goal += (out - dir * out.dot(dir)).normalize_or_zero() * (HAND_GAP * 0.5);
        }
        // Within reach, and no closer than the arm can fold.
        let longest = reach.max_extension * (upper_len + lower_len);
        let shortest = ((upper_len - lower_len).abs() + 0.05).min(longest);
        let clamp = |p: Vec3| {
            let v = p - shoulder;
            shoulder + v.normalize_or(-up) * v.length().clamp(shortest, longest)
        };
        let goal = clamp(goal);
        let lead = REACH_LEAD * reach.speed.max(0.05);
        let aim = clamp(tip + (goal - tip).clamp_length_max(lead));

        // Two bones, the elbow out (and a little down) from the line to the aim. The
        // upper arm is turned whole, twist included, so that its front (the way the
        // character faces at rest, whether the arms hang or stretch out in a T-pose, the
        // side an elbow folds towards) faces the way the forearm goes: the elbow then
        // bends about its own hinge. Swinging the arm by the shortest arc instead left
        // the twist of the T-pose, and elbows that should bend down bent up. An elbow
        // pointing down needs more twist than endorphin's shoulder allows: there the
        // hands fell 15° short of their aim, 10° now (docs/behaviours.md).
        let to = aim - shoulder;
        let dist = to.length().max(1e-4);
        let dir = to / dist;
        let away = (out - up * 0.3)
            .reject_from(dir)
            .try_normalize()
            .unwrap_or_else(|| dir.any_orthonormal_vector());
        let cos = ((upper_len * upper_len + dist * dist - lower_len * lower_len)
            / (2.0 * upper_len * dist).max(1e-6))
        .clamp(-1.0, 1.0);
        let want_elbow = shoulder + (dir * cos + away * (1.0 - cos * cos).sqrt()) * upper_len;
        let along = (want_elbow - shoulder) / upper_len.max(1e-6);
        let front = (aim - want_elbow)
            .reject_from(along)
            .try_normalize()
            .unwrap_or_else(|| (-away).reject_from(along).normalize_or_zero());
        let local_along = (upper_rot.inverse() * (elbow - shoulder)).normalize_or(Vec3::NEG_Y);
        let local_front = layout.fwd_local[hand.upper]
            .reject_from(local_along)
            .normalize_or(local_along.any_orthonormal_vector());
        let upper = basis(along, front) * basis(local_along, local_front).inverse();
        // The forearm straight on the upper arm (as at rest), then folded at the elbow to
        // bring the tip to the aim.
        let on_upper = layout.joint_of[hand.lower]
            .is_some_and(|j| self.rig.joints[j].parent_body == hand.upper);
        let straight = if on_upper {
            upper * layout.rest_rel[hand.lower]
        } else {
            upper * upper_rot.inverse() * forearm_rot
        };
        let reach_local = forearm_rot.inverse() * (tip - elbow);
        let lower = arc(straight * reach_local, aim - want_elbow) * straight;

        let mut bodies = vec![hand.upper, hand.lower];
        // The collarbone takes a share of the turn from where the arm points at rest to
        // the aim, as a shoulder comes forwards or up to reach: the shoulder joint alone
        // can't bring a hand across the chest.
        if let Some(&first) = hand.collar.first() {
            let trunk = self.parent_rotation(first);
            let turn = arc(trunk * hand.rest_reach, aim - shoulder);
            let mut parent = trunk;
            let mut child = Quat::IDENTITY.slerp(turn, COLLAR_SHARE) * trunk;
            for &b in &hand.collar {
                child *= layout.rest_rel[b];
                if let Some(j) = layout.joint_of[b] {
                    let held = self.targets[j];
                    self.aim(b, parent, child);
                    self.targets[j] = held.slerp(self.targets[j], w);
                    bodies.push(b);
                }
                parent = child;
            }
        }
        let parent = self.parent_rotation(hand.upper);
        for (b, parent, child) in [(hand.upper, parent, upper), (hand.lower, upper, lower)] {
            if let Some(j) = layout.joint_of[b] {
                let held = self.targets[j];
                self.aim(b, parent, child);
                self.targets[j] = held.slerp(self.targets[j], w);
            }
        }
        // The wrist and fingers straight along the forearm, by `orientation`.
        let mut parent = lower;
        for &b in &hand.chain {
            let Some(j) = layout.joint_of[b] else { break };
            let held = self.targets[j];
            let child = parent * layout.rest_rel[b];
            self.aim(b, parent, child);
            self.targets[j] = held.slerp(self.targets[j], reach.orientation * w);
            bodies.push(b);
            parent = child;
        }
        self.preset_of(&["Strong", "Normal"], reach.strength, bodies.into_iter());
    }

    /// Turns the neck and head to look at `look`'s target (see `look_and_reach`).
    fn look(&mut self, chest: usize, look: &Look) {
        let layout = self.layout;
        let Some(aim) = look.target else { return };
        let w = ramp(self.elapsed, look.delay, look.blend);
        let Some(from) = layout.spine.iter().position(|&b| b == chest) else {
            return;
        };
        let neck = &layout.spine[from + 1..];
        let Some(&head) = neck.last() else { return };
        if w <= 0.0 {
            return;
        }
        let body = &self.world.bodies[aim.body];
        let target = body.position + body.rotation * aim.offset;
        let head_rot = self.body(head).rotation;
        let Some(dir) = (target - self.body(head).position).try_normalize() else {
            return;
        };
        // The head as it sits on the chest at rest, then turned to face the target with
        // its top towards the up vector.
        let chest_rot = self.body(chest).rotation;
        let straight = neck.iter().fold(chest_rot, |r, &b| r * layout.rest_rel[b]);
        let (front, top) = (layout.fwd_local[head], layout.up_local[head]);
        let facing = Quat::from_rotation_arc((straight * front).normalize(), dir);
        let up = look.up.unwrap_or(head_rot * top);
        let roll = {
            let now = facing * straight * top;
            let now = now - dir * now.dot(dir);
            let want = up - dir * up.dot(dir);
            match (now.try_normalize(), want.try_normalize()) {
                (Some(a), Some(b)) => Quat::from_rotation_arc(a, b),
                _ => Quat::IDENTITY,
            }
        };
        let mut turn = roll * facing;
        let (axis, angle) = turn.to_axis_angle();
        if angle > MAX_LOOK {
            turn = Quat::from_axis_angle(axis, MAX_LOOK);
        }
        // No further than the lead past where the head is now.
        let wanted = turn * straight;
        let step = head_rot.angle_between(wanted);
        let lead = LOOK_LEAD * look.speed.max(0.05);
        if step > lead {
            turn = head_rot.slerp(wanted, lead / step) * straight.inverse();
        }
        // Shared evenly by the joints from the chest to the head.
        let n = neck.len() as f32;
        let (mut parent, mut rest) = (chest_rot, chest_rot);
        for (i, &b) in neck.iter().enumerate() {
            rest *= layout.rest_rel[b];
            let child = Quat::IDENTITY.slerp(turn, (i + 1) as f32 / n) * rest;
            if let Some(j) = layout.joint_of[b] {
                let held = self.targets[j];
                self.aim(b, parent, child);
                self.targets[j] = held.slerp(self.targets[j], w);
            }
            parent = child;
        }
        self.preset_of(&["Strong", "Normal"], 1.0, neck.iter().copied());
    }
}

/// Explosion's settings, as endorphin's scenes name them (all 0 to 1 but `PhaseOffset`).
/// The defaults are what Car Hit and most other scenes use.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Explosion {
    /// How far the arms swing up and down (`ShoulderAmplitude`).
    pub shoulder: f32,
    /// How far the elbows bend (`ElbowAmplitude`).
    pub elbow: f32,
    /// How far the legs pedal (`LegAmplitude`).
    pub legs: f32,
    /// How fast it all cycles (`Speed`, endorphin's "Frequency").
    pub speed: f32,
    /// Muscle strength, a fraction of the rig's "Normal" preset (`Strength`).
    pub strength: f32,
    /// How far the head turns from side to side (`HeadTurn`).
    pub head_turn: f32,
    /// How far the back arches (`BackArch`).
    pub back_arch: f32,
    /// Where in the cycle it starts, in cycles (`PhaseOffset`).
    pub phase: f32,
}

impl Default for Explosion {
    fn default() -> Self {
        Self {
            shoulder: 0.7,
            elbow: 0.6,
            legs: 0.5,
            speed: 0.6,
            strength: 0.5,
            head_turn: 0.6,
            back_arch: 1.0,
            phase: 0.0,
        }
    }
}

impl Explosion {
    /// Reads the settings by endorphin's names. WritheInMidAir (endorphin 2.5) has one
    /// `Amplitude` for arms, elbows and legs where Explosion (2.0) has one each.
    fn read(self, number: impl Fn(&str) -> Option<f32>) -> Self {
        let amplitude = number("Amplitude");
        let get = |key: &str, default: f32| number(key).or(amplitude).unwrap_or(default);
        Self {
            shoulder: get("ShoulderAmplitude", self.shoulder),
            elbow: get("ElbowAmplitude", self.elbow),
            legs: get("LegAmplitude", self.legs),
            speed: number("Speed").unwrap_or(self.speed),
            strength: number("Strength").unwrap_or(self.strength),
            head_turn: number("HeadTurn").unwrap_or(self.head_turn),
            back_arch: number("BackArch").unwrap_or(self.back_arch),
            phase: number("PhaseOffset").unwrap_or(self.phase),
        }
    }
}

/// The cycle's rate per unit of `Speed`, rad/s: at Speed 0.6 (7.2 rad/s, 52 frames at
/// 60 fps) the arms, lagging their targets, are highest at frame 16-24 and lowest at
/// 40-50, where endorphin's are at 20-30 and 42-46.
const EXPLOSION_RATE: f32 = 12.0;
/// How far the arms swing above and below out to the sides per unit of amplitude, rad.
const EXPLOSION_SHOULDER: f32 = 1.1;
/// Elbow bend at its fullest per unit of amplitude, rad.
const EXPLOSION_ELBOW: f32 = 1.5;
/// Hip flexion and knee bend per unit of leg amplitude, rad.
const EXPLOSION_HIP: f32 = 0.6;
const EXPLOSION_KNEE: f32 = 1.0;
/// How far each leg is spread to its side, rad.
const EXPLOSION_SPREAD: f32 = 0.15;
/// The back's arch at `BackArch` 1, over the whole trunk (neck and head included), rad.
const EXPLOSION_ARCH: f32 = 0.4;
/// Head turn per unit of `HeadTurn`, rad, at half the limbs' rate.
const EXPLOSION_HEAD: f32 = 0.5;
/// Height of the lowest body's centre above the ground below which the character has
/// landed, and the height over which the behaviour fades in above that, m: endorphin's
/// characters lie still once they have landed.
const EXPLOSION_LANDED: f32 = 0.15;
const EXPLOSION_FADE: f32 = 0.35;
/// The trunk's tilt from upright, rad, over which a character on the ground counts as
/// lying down (and goes limp) rather than standing.
const EXPLOSION_LYING: (f32, f32) = (0.8, 1.3);
/// Seconds after the behaviour starts before legs near the ground give.
const EXPLOSION_SETTLE: f32 = 0.25;

/// Explosion (endorphin's "Writhe in mid-air"): the arms, held out to the sides, flap
/// up and down together, the elbows bending as they come up; the legs pedal, spread a
/// little; the back arches and the head turns from side to side. Everything follows
/// sines of the time since the event began, so it is deterministic. Moving the arms
/// together keeps the swing from turning the body: endorphin's characters fall
/// upright. While it touches anything the flailing stops (the pose stays); lying on
/// the ground it goes limp and lies still, as endorphin's characters do once landed.
/// Works from the chest's frame and the rig's rest frames, so any biped works, T-pose
/// or not.
fn explosion(d: &mut Drive, e: Explosion) {
    use std::f32::consts::{FRAC_PI_4, PI, TAU};
    let layout = d.layout;
    // How far above the ground it is: 1 well clear of it, 0 landed.
    let lowest = d
        .all_bodies()
        .map(|b| d.body(b).position.y)
        .fold(f32::MAX, f32::min);
    let clear = d.world.ground.map_or(1.0, |(h, _)| {
        ((lowest - h - EXPLOSION_LANDED) / EXPLOSION_FADE).clamp(0.0, 1.0)
    });
    // It only flails in the air. Flailing while Car Hit's pedestrian lay on the
    // bonnet pushed it along the car: it landed 0.5 m further forward than endorphin's,
    // 10 frames early.
    let first = d.first_body;
    let pressing = d.world.pressing(first..first + d.rig.bodies.len());
    let airborne = if pressing { 0.0 } else { clear };
    // Down on the ground it goes limp, as endorphin's characters lie still once they
    // have landed. Held, even at a third of `Strength`, ours sat up again or rolled
    // over for a second or more. Standing, it keeps its strength: Multi Martial starts
    // it on characters still on their feet as they are kicked.
    let chest = layout.chest.unwrap_or(layout.pelvis);
    let trunk = d.body(chest).position - d.body(layout.pelvis).position;
    let tilt = trunk.normalize_or(Vec3::Y).y.clamp(-1.0, 1.0).acos();
    let lying =
        ((tilt - EXPLOSION_LYING.0) / (EXPLOSION_LYING.1 - EXPLOSION_LYING.0)).clamp(0.0, 1.0);
    d.preset(
        "Normal",
        e.strength * (1.0 - (1.0 - clear) * lying),
        d.all_bodies(),
    );
    // Landing on its feet, its legs give: held, ours stood on landing in some runs and
    // went down on its back in others. Not in the first moments, though, so a character
    // it starts on while standing (Multi Martial's, as they are kicked) keeps its feet.
    if d.elapsed > EXPLOSION_SETTLE {
        let legs: Vec<usize> = layout
            .legs
            .iter()
            .flat_map(|l| [l.upper, l.lower, l.end])
            .collect();
        d.preset("Normal", e.strength * clear, legs.into_iter());
    }
    // The presets are tuned to hold poses against the ground; for arms swinging in the
    // air they are several times overdamped, and the arms then barely moved. The legs
    // keep them: damped critically, they kicked Car Hit's pedestrian off the bonnet
    // 0.45 m higher than endorphin's goes.
    for arm in &layout.arms {
        d.damp_for_swing(arm.upper, &[arm.upper, arm.lower, arm.end]);
        d.damp_for_swing(arm.lower, &[arm.lower, arm.end]);
    }
    let angle = EXPLOSION_RATE * e.speed * d.elapsed + e.phase * TAU;

    // The back arches, shared by the trunk's joints; the head turns.
    let pelvis = d.body(layout.pelvis).rotation;
    let n = layout.spine.len();
    let spine: Vec<(usize, Turn)> = layout
        .spine
        .iter()
        .enumerate()
        .map(|(i, &s)| {
            let turn = Turn {
                bend: -e.back_arch * EXPLOSION_ARCH / n as f32,
                twist: if i + 1 == n {
                    airborne * e.head_turn * EXPLOSION_HEAD * (0.5 * angle).sin()
                } else {
                    0.0
                },
                ..Turn::default()
            };
            (s, turn)
        })
        .collect();
    d.pose_chain(pelvis, &spine);

    // The legs pedal in turn, spread a little to their sides.
    let (fwd, up) = (
        layout.fwd_local[layout.pelvis],
        layout.up_local[layout.pelvis],
    );
    for (l, leg) in layout.legs.iter().enumerate() {
        // Turning about the forward axis by +θ swings a hanging leg towards fwd × -up.
        let side = fwd.cross(-up).dot(layout.outward[l]).signum();
        let cycle = angle + l as f32 * PI;
        let swing = airborne * e.legs;
        let hip = Turn {
            bend: -swing * EXPLOSION_HIP * cycle.sin(),
            side: side * EXPLOSION_SPREAD,
            twist: 0.0,
        };
        let knee = Turn {
            bend: swing * EXPLOSION_KNEE * (0.5 + 0.5 * cycle.cos()),
            ..Turn::default()
        };
        d.pose_chain(
            pelvis,
            &[
                (leg.upper, hip),
                (leg.lower, knee),
                (leg.end, Turn::default()),
            ],
        );
    }

    // The arms flap together about out to the sides; the elbows bend most just after
    // the arms are at their highest, bringing the hands in over the head.
    let Some(chest) = layout.chest else { return };
    let lift = airborne * e.shoulder * EXPLOSION_SHOULDER * angle.sin();
    let bend = airborne * e.elbow * EXPLOSION_ELBOW * (0.5 + 0.5 * (angle - FRAC_PI_4).sin());
    for &arm in &layout.arms {
        let frame = d.chest_frame(chest, arm);
        // (up, forward, out) weights: out to the side, turned up by `a`.
        let along = |a: f32| Vec3::new(a.sin(), 0.15, a.cos());
        d.point_arm(arm, frame, along(lift), along(lift + bend));
    }
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
        for name in [
            "Bob2: Look and Reach 2.0",
            "Bob2: Look and Reach 2.5",
            "Bob2: Look and Reach",
            "Hands reach and look at 2.5",
            "Hands reach and look at2.0",
            "LookAndReach",
        ] {
            assert_eq!(Behaviour::from_name(name), Some(Behaviour::LookAndReach));
        }
        // endorphin's "Writhe in mid-air", under its 2.0 and 2.5 names and its display
        // name; plain writhe stays Writhe.
        for name in [
            "Explosion",
            "Bob2: Explosion 2.0",
            "Bob2: Explosion",
            "Bob2: WritheInMidAir 2.5",
            "Writhe in mid-air 2.0",
            "Writhe in mid-air",
        ] {
            assert_eq!(
                Behaviour::from_name(name),
                Some(Behaviour::Explosion(Explosion::default())),
                "{name}"
            );
        }
        assert_eq!(Behaviour::from_name("Writhe"), Some(Behaviour::Writhe));
        assert_eq!(Behaviour::from_name("Bob2: JumpDive 2.0"), None);
    }

    #[test]
    fn explosion_reads_its_settings() {
        let source = |pairs: &[(&str, &str)]| -> BTreeMap<String, String> {
            pairs
                .iter()
                .map(|(k, v)| ((*k).to_owned(), (*v).to_owned()))
                .collect()
        };
        let explosion = |pairs: &[(&str, &str)]| match Behaviour::from_name("Bob2: Explosion 2.0")
            .unwrap()
            .with_settings(&source(pairs))
        {
            Behaviour::Explosion(e) => e,
            other => panic!("{other:?}"),
        };
        // Explosion 2.0's own names; anything missing or unreadable keeps the default.
        let e = explosion(&[
            ("ShoulderAmplitude", "0.300"),
            ("ElbowAmplitude", "0.2"),
            ("LegAmplitude", "1.000"),
            ("Speed", "0.8"),
            ("Strength", "1"),
            ("HeadTurn", "0"),
            ("BackArch", "0.000"),
            ("PhaseOffset", "0.25"),
            ("index", "27"),
        ]);
        assert_eq!(
            e,
            Explosion {
                shoulder: 0.3,
                elbow: 0.2,
                legs: 1.0,
                speed: 0.8,
                strength: 1.0,
                head_turn: 0.0,
                back_arch: 0.0,
                phase: 0.25,
            }
        );
        assert_eq!(explosion(&[("Speed", "fast")]), Explosion::default());
        // WritheInMidAir 2.5's one Amplitude sets all three, unless one is given.
        let e = explosion(&[("Amplitude", "0.4"), ("LegAmplitude", "0.9")]);
        assert_eq!((e.shoulder, e.elbow, e.legs), (0.4, 0.4, 0.9));
        // Other behaviours ignore settings.
        assert_eq!(
            Behaviour::Balance.with_settings(&source(&[("Speed", "1")])),
            Behaviour::Balance
        );
    }

    /// endorphin's characters have a clavicle before each upper arm and a twist body
    /// half way along each forearm. The upper arm and forearm are still the arm.
    #[test]
    fn finds_the_arms_past_clavicles_and_forearm_twists() {
        for t_pose in [false, true] {
            let (skeleton, rig) = super::super::scenes::person_rig_like_endorphins(t_pose);
            let layout = Layout::new(&rig, &skeleton).unwrap();
            let name = |b: usize| rig.bodies[b].name.as_str();
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
                ],
                "t-pose {t_pose}"
            );
            assert_eq!(layout.chest.map(name), Some("chest"));
        }
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

    /// The two bones reaching bends, and which hand is which.
    #[test]
    fn finds_the_placeholders_hands() {
        let skeleton = humanoid_skeleton();
        let rig = humanoid_rig(&skeleton);
        let layout = Layout::new(&rig, &skeleton).unwrap();
        let name = |b: usize| rig.bodies[b].name.as_str();
        for hand in &layout.hands {
            let side = if hand.left { "l" } else { "r" };
            assert_eq!(name(hand.upper), format!("{side}_shoulder"));
            assert_eq!(name(hand.lower), format!("{side}_elbow"));
            assert!(hand.collar.is_empty());
        }
        assert_eq!(layout.hands.iter().filter(|h| h.left).count(), 1);
    }

    /// Settings as an imported event carries them; missing ones take the defaults, and a
    /// target that doesn't resolve leaves the hand alone.
    #[test]
    fn reads_look_and_reach_settings() {
        let source: std::collections::BTreeMap<String, String> = [
            ("LeftHandTarget", "background.MetaBodies.Prop0"),
            ("RightHandTarget", "nobody.MetaBodies.Nothing"),
            ("LeftHand_Speed", "0.700"),
            ("LeftHand_CatchOffsetZ", "0.200"),
            ("LeftHand_applyHandOrientation", "false"),
            (
                "LookAtTarget",
                "nobody.MetaBodies.Nothing, background.MetaBodies.Prop0",
            ),
            ("LookAt_UseUpVector", "true"),
            ("LegTorsoStiffness", "0.5"),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_owned(), v.to_owned()))
        .collect();
        let aim = |name: &str| {
            (name == "background.MetaBodies.Prop0").then_some(Aim {
                body: 7,
                offset: Vec3::ZERO,
            })
        };
        let s = LookAndReach::read(&source, &aim);
        assert_eq!(s.left.target.map(|a| a.body), Some(7));
        assert_eq!(s.right.target, None);
        assert_eq!(s.left.speed, 0.7);
        assert_eq!(s.right.speed, 0.8);
        assert_eq!(s.left.offset, Vec3::new(0.0, 0.0, 0.2));
        assert_eq!(s.left.orientation, 0.0);
        assert_eq!(s.right.orientation, 0.5);
        assert_eq!(s.look.target.map(|a| a.body), Some(7));
        assert_eq!(s.look.up, Some(Vec3::Y));
        assert_eq!(s.leg_torso, 0.5);
    }
}
