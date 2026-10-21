//! The project data model: what a NeuroMotion project *is*, independent of any UI.
//!
//! * [`model`] – serializable types (skeleton reference, tracks, clips, channels, keys)
//! * [`pose`] – evaluating the blended pose at a frame
//! * [`edit`] – undoable edits and the [`Document`] that records them
//! * [`ops`] – higher-level edits (add/move/delete keys, clips and tracks; baking)
//! * [`bake`] – turning world-space motion (the simulation's) back into keys
//! * [`physics`] – bodies, joints, muscles and materials (data only)
//! * [`sample`] – built-in placeholder rig and demo animation

mod bake;
mod edit;
mod model;
mod ops;
pub mod physics;
mod pose;
pub mod sample;

#[cfg(test)]
mod tests;

pub use edit::{Document, Edit};
pub use model::*;
pub use physics::SimLevel;
pub use pose::{Pose, euler_near, local_rotation as pose_local_rotation, local_transforms, world_transforms};
