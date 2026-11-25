//! NeuroMotion's physics engine.
//!
//! * [`world`] – rigid bodies, joints, muscles and contacts, and the solver
//! * [`collide`] – shape-vs-shape contact generation
//! * [`behaviour`] – controllers that drive muscles (balance, catch fall)
//! * [`interp`] – blends cached frames for output between whole frames
//! * [`sim`] – runs a project's characters through the world, frame by frame,
//!   following its simulation events, and caches the results for the timeline
//!
//! Design notes: docs/physics-design.md.

pub mod behaviour;
pub mod collide;
pub mod interp;
#[cfg(test)]
mod reference;
#[cfg(test)]
mod scenes;
pub mod sim;
pub mod world;

pub use sim::{Simulation, rig_bodies};
