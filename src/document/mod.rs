//! The project data model: what a NeuroMotion project *is*, independent of any UI.
//!
//! * [`model`] – serializable types (skeleton reference, tracks, clips, channels, keys)
//! * [`pose`] – evaluating the blended pose at a frame
//! * [`edit`] – undoable edits and the [`Document`] that records them
//! * [`ops`] – higher-level edits (add/move/delete keys, delete clips)
//! * [`sample`] – built-in placeholder rig and demo animation

mod edit;
mod model;
mod ops;
mod pose;
pub mod sample;

#[cfg(test)]
mod tests;

pub use edit::{Document, Edit};
pub use model::*;
pub use pose::Pose;
