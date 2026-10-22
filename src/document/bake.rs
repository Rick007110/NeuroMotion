//! Baking: turning world-space joint motion (usually the simulation's) back into keys.

use glam::{EulerRot, Quat, Vec3};

use super::model::{Channel, Interp, Keyframe, Property, SkeletonRef};
use super::pose::{euler_near, local_transforms};

/// Offsets that move less than this aren't keyed: simulated joints stretch by fractions
/// of a millimetre, and keying that would only add noise.
const OFFSET_TOLERANCE: f32 = 1e-3;

/// Channels that reproduce `frames` (every joint's world rotation and position, one
/// entry per frame, keyed from clip frame 0): a rotation for every joint relative to its
/// rest orientation, the position of each root, and the offset of any other joint that
/// moves. Every frame is keyed, linearly, and rotations stay continuous frame to frame.
pub fn bake_channels(skeleton: &SkeletonRef, frames: &[Vec<(Quat, Vec3)>]) -> Vec<Channel> {
    let locals: Vec<Vec<(Quat, Vec3)>> = frames.iter().map(|world| local_transforms(skeleton, world)).collect();
    let rotations = [Property::RotX, Property::RotY, Property::RotZ];
    let positions = [Property::PosX, Property::PosY, Property::PosZ];
    let mut channels = Vec::new();
    for (j, joint) in skeleton.joints.iter().enumerate() {
        let rest = Quat::from_array(joint.rest_rotation).normalize();
        let mut previous = [0.0; 3];
        let euler: Vec<[f32; 3]> = locals
            .iter()
            .map(|l| {
                previous = euler_near(rest.inverse() * l[j].0, EulerRot::XYZ, previous);
                previous
            })
            .collect();
        for (axis, property) in rotations.into_iter().enumerate() {
            channels.push(keyed(&joint.name, property, euler.iter().map(|e| e[axis])));
        }

        let offsets: Vec<Vec3> = locals.iter().map(|l| l[j].1 - Vec3::from(joint.offset)).collect();
        let is_root = joint.parent.filter(|&p| p < j).is_none();
        if is_root || offsets.iter().any(|o| o.length() > OFFSET_TOLERANCE) {
            for (axis, property) in positions.into_iter().enumerate() {
                channels.push(keyed(&joint.name, property, offsets.iter().map(|o| o[axis])));
            }
        }
    }
    channels
}

fn keyed(joint: &str, property: Property, values: impl Iterator<Item = f32>) -> Channel {
    Channel {
        joint: joint.to_owned(),
        property,
        keys: values
            .enumerate()
            .map(|(frame, value)| Keyframe { frame: frame as i32, value, interp: Interp::Linear })
            .collect(),
    }
}
