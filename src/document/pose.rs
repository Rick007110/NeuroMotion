//! Pose evaluation: turning tracks, clips and keys into joint values at a frame.

use glam::{EulerRot, Quat, Vec3};

use super::model::{ClipId, Project, SkeletonRef};

/// Channel values per joint, indexed like the character's skeleton:
/// `[rot x, rot y, rot z (degrees), offset x, offset y, offset z (metres)]`.
#[derive(Clone, Debug, PartialEq)]
pub struct Pose {
    pub joints: Vec<[f32; 6]>,
}

impl Pose {
    pub fn rest(joint_count: usize) -> Self {
        Self {
            joints: vec![[0.0; 6]; joint_count],
        }
    }
}

impl Project {
    /// The blended pose of every character at `frame`, in `characters` order.
    /// Fractional frames are allowed for smooth playback.
    ///
    /// Per character, tracks apply bottom to top and clips within a track in start
    /// order. Each motion clip pulls the channels it animates towards its own values by
    /// its current influence, so a fully weighted clip on a higher track overrides what
    /// is below it and the blend ramps crossfade between them. Channels no clip touches
    /// stay at rest. Event clips don't affect the keyframed pose.
    pub fn evaluate(&self, frame: f32) -> Vec<Pose> {
        let mut poses: Vec<Pose> = self
            .characters
            .iter()
            .map(|c| Pose::rest(c.skeleton.joints.len()))
            .collect();
        for track in self.tracks.iter().filter(|t| !t.muted) {
            let Some(ci) = self.character_index(track.character) else { continue };
            let skeleton = &self.characters[ci].skeleton;
            let pose = &mut poses[ci];
            let mut clips: Vec<_> = self
                .clips_on_track(track.id)
                .filter(|c| c.kind.is_motion())
                .collect();
            clips.sort_by_key(|c| c.props.start);
            for clip in clips {
                let weight = clip.influence(frame);
                if weight <= 0.0 {
                    continue;
                }
                let local = frame - clip.props.start as f32;
                for channel in &clip.channels {
                    let (Some(joint), Some(value)) =
                        (skeleton.joint_index(&channel.joint), channel.sample(local))
                    else {
                        continue;
                    };
                    let slot = &mut pose.joints[joint][channel.property.index()];
                    *slot += (value - *slot) * weight;
                }
            }
        }
        poses
    }

    /// Clips contributing at `frame` with their influence, in track order.
    pub fn active_clips(&self, frame: f32) -> Vec<(ClipId, f32)> {
        let mut out = Vec::new();
        for track in self.tracks.iter().filter(|t| !t.muted) {
            for clip in self.clips_on_track(track.id) {
                let weight = clip.influence(frame);
                if weight > 0.0 {
                    out.push((clip.id, weight));
                }
            }
        }
        out
    }
}

/// Each joint's world rotation and position for a pose. A joint's rotation is its rest
/// rotation followed by the pose's local XYZ Euler angles; pose offsets add to the rest
/// offsets. Parents must precede children (any other parent index is treated as root).
pub fn world_transforms(skeleton: &SkeletonRef, pose: &Pose) -> Vec<(Quat, Vec3)> {
    let mut out: Vec<(Quat, Vec3)> = Vec::with_capacity(skeleton.joints.len());
    for (i, joint) in skeleton.joints.iter().enumerate() {
        let v = pose.joints.get(i).copied().unwrap_or([0.0; 6]);
        let local = local_rotation(joint.rest_rotation, v);
        let offset = Vec3::from(joint.offset) + Vec3::new(v[3], v[4], v[5]);
        let transform = match joint.parent.filter(|&p| p < i) {
            Some(p) => {
                let (parent_rot, parent_pos) = out[p];
                ((parent_rot * local).normalize(), parent_pos + parent_rot * offset)
            }
            None => (local, offset),
        };
        out.push(transform);
    }
    out
}

/// A joint's local rotation: rest orientation, then the pose's XYZ Euler degrees.
pub fn local_rotation(rest: [f32; 4], values: [f32; 6]) -> Quat {
    let euler = Quat::from_euler(
        EulerRot::XYZ,
        values[0].to_radians(),
        values[1].to_radians(),
        values[2].to_radians(),
    );
    (Quat::from_array(rest).normalize() * euler).normalize()
}
