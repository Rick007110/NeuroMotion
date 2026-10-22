//! Behaviours: controllers that read a simulated character's state every physics tick
//! and set its muscle targets and strengths, like endorphin's balance and catch fall.
//! endorphin's own behaviours are vendor-protected; these are original designs, run for
//! imported behaviour events of the same name. Design notes: docs/behaviours.md.
//!
//! Controllers are stateless (everything comes from the bodies each tick), so they keep
//! the simulation deterministic.

use glam::{Quat, Vec3};

use super::sim::rig_bodies;
use super::world::{Body, Joint, World};
use crate::document::physics::{Gains, MuscleGains, PhysicsRig};
use crate::document::{Pose, SkeletonRef, world_transforms};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Behaviour {
    /// Keep the centre of mass over the feet (ankle strategy), pelvis upright.
    Balance,
    /// Reach both arms towards where the chest will land; let the legs give.
    CatchFall,
}

impl Behaviour {
    /// The behaviour for an event's name (endorphin's `behaviourName`), ignoring case,
    /// spaces and punctuation; `None` if NeuroMotion has none of that name.
    pub fn from_name(name: &str) -> Option<Self> {
        let key: String = name.chars().filter(|c| c.is_alphanumeric()).flat_map(char::to_lowercase).collect();
        match key.as_str() {
            "balance" => Some(Behaviour::Balance),
            "catchfall" => Some(Behaviour::CatchFall),
            _ => None,
        }
    }

    pub fn drive(self, d: &mut Drive) {
        match self {
            Behaviour::Balance => balance(d),
            Behaviour::CatchFall => catch_fall(d),
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
    /// The trunk body the arms hang from.
    chest: Option<usize>,
    legs: Vec<Limb>,
    arms: Vec<Limb>,
    /// Rig joint moving each body.
    joint_of: Vec<Option<usize>>,
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
            if jd.body < n && jd.parent_body < n && jd.body != jd.parent_body && jd.joint < rest_joints.len() {
                joint_of[jd.body] = Some(j);
                children[jd.parent_body].push(jd.body);
            }
        }
        let pelvis = (0..n).find(|&b| joint_of[b].is_none() && !children[b].is_empty())?;
        let at = |b: usize| rest[b].1;
        // Where each body's joint is at rest (its origin for the pelvis).
        let pivot = |b: usize| joint_of[b].map_or(at(b), |j| rest_joints[rig.joints[j].joint].1);
        // Extremes of a branch at rest, by some measure of body position.
        fn extreme(children: &[Vec<usize>], b: usize, measure: &dyn Fn(usize) -> f32) -> f32 {
            children[b].iter().map(|&c| extreme(children, c, measure)).fold(measure(b), f32::max)
        }
        // Follows a branch towards its extreme, one body at a time.
        let follow = |start: usize, measure: &dyn Fn(usize) -> f32| {
            let mut chain = vec![start];
            while let Some(&next) = children[*chain.last().unwrap()].iter().max_by(|&&a, &&b| extreme(&children, a, measure).total_cmp(&extreme(&children, b, measure))) {
                chain.push(next);
            }
            chain
        };
        let low = |b: usize| -at(b).y;
        let high = |b: usize| at(b).y;
        let mut branches: Vec<usize> = children[pelvis].clone();
        branches.sort_by(|&a, &b| extreme(&children, b, &low).total_cmp(&extreme(&children, a, &low)));
        let legs: Vec<Limb> = branches
            .iter()
            .take(2)
            .filter_map(|&thigh| {
                let chain = follow(thigh, &low);
                (chain.len() >= 3).then(|| Limb { upper: chain[0], lower: chain[1], end: chain[2] })
            })
            .collect();

        let trunk = branches.iter().skip(2).copied().max_by(|&a, &b| extreme(&children, a, &high).total_cmp(&extreme(&children, b, &high)));
        let mut arms = Vec::new();
        let mut chest = None;
        if let Some(trunk) = trunk {
            let spine = follow(trunk, &high);
            let axis = at(pelvis);
            let side = |b: usize| (at(b) - axis).with_y(0.0).length();
            let mut off: Vec<(f32, usize, usize)> = spine
                .iter()
                .flat_map(|&s| children[s].iter().filter(|c| !spine.contains(c)).map(move |&c| (s, c)))
                .map(|(s, c)| (extreme(&children, c, &side), s, c))
                .collect();
            off.sort_by(|a, b| b.0.total_cmp(&a.0));
            for &(_, s, c) in off.iter().take(2) {
                let start = pivot(c);
                let chain = follow(c, &|b| at(b).distance(start));
                // Segment lengths, joint to next joint; pick the longest consecutive pair.
                let length = |i: usize| chain.get(i + 1).map_or(0.0, |&next| pivot(chain[i]).distance(pivot(next)));
                let best = (0..chain.len().saturating_sub(2)).max_by(|&a, &b| (length(a) + length(a + 1)).total_cmp(&(length(b) + length(b + 1))));
                if let Some(i) = best {
                    arms.push(Limb { upper: chain[i], lower: chain[i + 1], end: chain[i + 2] });
                    chest.get_or_insert(s);
                }
            }
        }
        let pelvis_up = rest[pelvis].0.inverse() * Vec3::Y;
        let pelvis_side = rest[pelvis].0.inverse() * Vec3::X;
        Some(Layout { pelvis, pelvis_up, pelvis_side, chest, legs, arms, joint_of })
    }
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
        self.joint(b).map_or(body.position, |(_, j)| body.position + body.rotation * j.anchor_child)
    }

    /// Rotation of the body the joint moving `b` hangs from.
    fn parent_rotation(&self, b: usize) -> Quat {
        self.layout.joint_of[b].map_or(Quat::IDENTITY, |j| self.body(self.rig.joints[j].parent_body).rotation)
    }

    /// Sets the muscle moving `b` to bring its body and its parent to these world rotations.
    fn aim(&mut self, b: usize, parent: Quat, child: Quat) {
        if let Some((j, joint)) = self.joint(b) {
            let target = ((parent * joint.drive_parent).inverse() * (child * joint.drive_child)).normalize();
            self.targets[j] = target;
        }
    }

    /// Gains from a named muscle preset (falling back to the rig's first), scaled.
    fn preset(&mut self, name: &str, scale: f32, bodies: impl Iterator<Item = usize>) {
        let set = self.rig.muscle_set(name).or_else(|| self.rig.muscle_sets.first());
        let scaled = |g: Gains| Gains { stiffness: g.stiffness * scale, damping: g.damping * scale, force_limit: g.force_limit * scale };
        for b in bodies {
            if let Some(j) = self.layout.joint_of[b] {
                let g = set.map_or(STEADY, |s| s.gains(&self.rig.joints[j].muscle));
                self.gains[j] = MuscleGains { twist: scaled(g.twist), swing: scaled(g.swing) };
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
        legs.iter().map(|l| flat(self.body(l.end).position)).sum::<Vec3>() / legs.len() as f32
    }

    /// Ground height under the character: the ground plane, else its lowest foot.
    fn ground(&self) -> f32 {
        let feet = self.layout.legs.iter().map(|l| self.body(l.end).position.y).fold(f32::MAX, f32::min);
        self.world.ground.map_or(feet, |(h, _)| h)
    }

    fn all_bodies(&self) -> std::ops::Range<usize> {
        0..self.rig.bodies.len()
    }
}

/// Used when a rig has no muscle presets.
const STEADY: MuscleGains = MuscleGains {
    twist: Gains { stiffness: 1000.0, damping: 500.0, force_limit: 1000.0 },
    swing: Gains { stiffness: 1000.0, damping: 500.0, force_limit: 1000.0 },
};

fn flat(v: Vec3) -> Vec3 {
    v.with_y(0.0)
}

/// Furthest the centre of mass is asked to lean away from straight over the feet, m.
const MAX_SHIFT: f32 = 0.15;

/// Balance: an ankle strategy on the capture point, the point where the centre of mass
/// would come to rest, given its speed (position + velocity × √(height / g)). Both
/// legs are swung about the ankles to carry the hips back over the middle of the feet,
/// twice as hard against where the body is heading as against where it is, and the
/// ankle muscles push the body back. The pelvis is held upright. Sideways, the two legs
/// and the pelvis make a parallelogram, so the hips aim for the same lean as the ankles
/// to keep it from folding; fore and aft they only keep the pelvis level, or they'd tip
/// it. Every muscle is tensed to the rig's "Normal" preset; the knees, trunk and arms
/// keep the pose the timeline asks for.
fn balance(d: &mut Drive) {
    let legs = d.layout.legs.clone();
    if legs.len() < 2 {
        return;
    }
    d.preset("Normal", 1.0, d.all_bodies());
    let g = d.world.gravity.length().max(1.0);
    let support = d.support();
    let (com, vel) = d.centre_of_mass();
    let height = (com.y - d.ground()).max(0.3);
    let heading = flat(vel) * (height / g).sqrt();
    let shift = (support - flat(com) - heading * 2.0).clamp_length_max(MAX_SHIFT);

    let pelvis = d.body(d.layout.pelvis).rotation;
    let upright = Quat::from_rotation_arc((pelvis * d.layout.pelvis_up).normalize(), Vec3::Y) * pelvis;
    let side = flat(upright * d.layout.pelvis_side).normalize_or_zero();
    let sideways = side * shift.dot(side);
    for leg in legs {
        let (hip, ankle) = (d.pivot(leg.upper), d.pivot(leg.end));
        let lean = |by: Vec3| Quat::from_rotation_arc((hip - ankle).normalize(), (hip + by - ankle).normalize());
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
    let Some(chest) = layout.chest.filter(|_| !arms.is_empty()) else { return };
    d.preset("Normal", 1.0, d.all_bodies());
    d.preset("Normal", 0.2, legs.iter().flat_map(|l| [l.upper, l.lower, l.end]));
    d.preset("Strong", 1.0, arms.iter().flat_map(|a| [a.upper, a.lower]));

    let ground = d.ground();
    let (com, vel) = d.centre_of_mass();
    let support = d.support();
    let chest_at = d.body(chest).position;
    let lean = flat(com) - support;
    let heading = if flat(vel).length() > 0.3 { flat(vel) } else { lean };
    let Some(dir) = heading.try_normalize() else { return };
    let landing = support + dir * (chest_at.y - ground).max(0.3) * REACH;

    for arm in arms {
        let shoulder = d.pivot(arm.upper);
        // Keep the hands shoulder-width apart, across the fall direction.
        let side = flat(shoulder - chest_at);
        let side = side - dir * side.dot(dir);
        let hand = (landing + side).with_y(ground + 0.05);
        let want = (hand - shoulder).normalize();
        let elbow = d.pivot(arm.lower);
        let upper = Quat::from_rotation_arc((elbow - shoulder).normalize(), want) * d.body(arm.upper).rotation;
        let lower = Quat::from_rotation_arc((d.pivot(arm.end) - elbow).normalize(), want) * d.body(arm.lower).rotation;
        let parent = d.parent_rotation(arm.upper);
        d.aim(arm.upper, parent, upper);
        d.aim(arm.lower, upper, lower);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::document::sample::{humanoid_rig, humanoid_skeleton};

    #[test]
    fn names_match_endorphins() {
        assert_eq!(Behaviour::from_name("Balance"), Some(Behaviour::Balance));
        assert_eq!(Behaviour::from_name("CatchFall"), Some(Behaviour::CatchFall));
        assert_eq!(Behaviour::from_name("catch fall"), Some(Behaviour::CatchFall));
        assert_eq!(Behaviour::from_name("Writhe"), None);
    }

    #[test]
    fn finds_the_placeholders_limbs() {
        let skeleton = humanoid_skeleton();
        let rig = humanoid_rig(&skeleton);
        let layout = Layout::new(&rig, &skeleton).unwrap();
        let name = |b: usize| rig.bodies[b].name.as_str();
        assert_eq!(name(layout.pelvis), "hips");
        assert_eq!(layout.chest.map(name), Some("chest"));
        let mut legs: Vec<_> = layout.legs.iter().map(|l| [name(l.upper), name(l.lower), name(l.end)]).collect();
        legs.sort();
        assert_eq!(legs, [["l_hip", "l_knee", "l_ankle"], ["r_hip", "r_knee", "r_ankle"]]);
        let mut arms: Vec<_> = layout.arms.iter().map(|a| [name(a.upper), name(a.lower), name(a.end)]).collect();
        arms.sort();
        assert_eq!(arms, [["l_shoulder", "l_elbow", "l_wrist"], ["r_shoulder", "r_elbow", "r_wrist"]]);
    }
}
