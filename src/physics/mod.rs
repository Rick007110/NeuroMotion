//! NeuroMotion's physics engine.
//!
//! * [`world`] – rigid bodies, joints, muscles and contacts, and the solver
//! * [`collide`] – shape-vs-shape contact generation
//! * [`sim`] – runs a project's characters through the world, frame by frame,
//!   following its simulation events, and caches the results for the timeline
//!
//! Design notes: docs/physics-design.md.

pub mod collide;
pub mod sim;
pub mod world;

pub use sim::{Simulation, rig_bodies};
