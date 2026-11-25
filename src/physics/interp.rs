//! Smooth output between cached frames.
//!
//! The simulation caches one state per scene frame (30 fps for most scenes), so
//! drawing the nearest whole frame makes playback on a 60 Hz display step. This
//! blends the two neighbouring cached frames instead: positions lerp, rotations
//! slerp along the shortest path. It only reads the cache; nothing here feeds back
//! into the simulation, so the cached data stays exactly as computed.

use glam::{Quat, Vec3};

use super::sim::{CharacterFrame, Frame, Simulation};

/// Blends two transforms: `t` = 0 gives `a`, 1 gives `b`.
pub fn blend_transform(a: (Quat, Vec3), b: (Quat, Vec3), t: f32) -> (Quat, Vec3) {
    // glam's slerp takes the short way round (it flips `b` when the dot is negative).
    (a.0.slerp(b.0, t).normalize(), a.1.lerp(b.1, t))
}

fn blend_all(a: &[(Quat, Vec3)], b: &[(Quat, Vec3)], t: f32) -> Vec<(Quat, Vec3)> {
    a.iter()
        .zip(b)
        .map(|(&a, &b)| blend_transform(a, b, t))
        .collect()
}

/// A character's state between two frames. The simulation level (and so the colour it
/// is drawn in) is the nearer frame's. If the two rigs don't match, `a` is returned.
pub fn blend_character(a: &CharacterFrame, b: &CharacterFrame, t: f32) -> CharacterFrame {
    if t <= 0.0 || a.joints.len() != b.joints.len() || a.bodies.len() != b.bodies.len() {
        return a.clone();
    }
    CharacterFrame {
        joints: blend_all(&a.joints, &b.joints, t),
        bodies: blend_all(&a.bodies, &b.bodies, t),
        level: if t < 0.5 { a.level } else { b.level },
    }
}

/// All characters between two frames.
pub fn blend_frames(a: &Frame, b: &Frame, t: f32) -> Frame {
    a.iter()
        .enumerate()
        .map(|(i, a)| match (a, b.get(i).and_then(Option::as_ref)) {
            (Some(a), Some(b)) => Some(blend_character(a, b, t)),
            (a, _) => a.clone(),
        })
        .collect()
}

/// The simulated state at a fractional `frame`, interpolated between the cached whole
/// frames around it. At a whole frame this is the cached frame itself, bit for bit.
/// Past the last computed frame (the simulation is still running) it holds the last
/// one. `None` if the frame before `frame` isn't computed.
pub fn frame_at(simulation: &Simulation, frame: f64) -> Option<Frame> {
    let base = frame.floor();
    let a = simulation.frame(base as i32)?;
    let t = (frame - base) as f32;
    if t <= 0.0 {
        return Some(a.clone());
    }
    Some(match simulation.frame(base as i32 + 1) {
        Some(b) => blend_frames(a, b, t),
        None => a.clone(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::document::physics::SimLevel;
    use std::time::{Duration, Instant};

    fn character(angle: f32, x: f32, level: SimLevel) -> CharacterFrame {
        let t = (Quat::from_rotation_y(angle), Vec3::new(x, 1.0, 0.0));
        CharacterFrame {
            joints: vec![t],
            bodies: vec![t],
            level,
        }
    }

    #[test]
    fn halfway_is_the_mid_pose() {
        let a = character(0.0, 0.0, SimLevel::Simulated);
        let b = character(1.0, 2.0, SimLevel::Simulated);
        let (r, p) = blend_character(&a, &b, 0.5).joints[0];
        assert!((p - Vec3::new(1.0, 1.0, 0.0)).length() < 1e-6);
        assert!(r.angle_between(Quat::from_rotation_y(0.5)) < 1e-5);
    }

    #[test]
    fn rotation_takes_the_short_way() {
        // 170 degrees to -170 degrees is 20 degrees apart, not 340.
        let a = character(170f32.to_radians(), 0.0, SimLevel::Simulated);
        let b = character((-170f32).to_radians(), 0.0, SimLevel::Simulated);
        let half = Quat::from_rotation_y(180f32.to_radians());
        let mid = blend_character(&a, &b, 0.5).joints[0].0;
        assert!(mid.angle_between(half) < 1e-5);
        // The same rotation written with the opposite sign must not swing the long way.
        let flipped = CharacterFrame {
            joints: vec![(-b.joints[0].0, b.joints[0].1)],
            ..b
        };
        let mid = blend_character(&a, &flipped, 0.5).joints[0].0;
        assert!(mid.angle_between(half) < 1e-5);
    }

    #[test]
    fn animation_to_simulation_has_no_pop() {
        // Frame 0 animated at x = 0, frame 1 simulated at x = 1 and turned.
        let a = character(0.0, 0.0, SimLevel::Animated);
        let b = character(0.4, 1.0, SimLevel::Simulated);
        let mut last = a.joints[0];
        for i in 1..=10 {
            let s = blend_character(&a, &b, i as f32 / 10.0).joints[0];
            assert!(s.1.distance(last.1) < 0.1001);
            assert!(s.0.angle_between(last.0) < 0.0401);
            last = s;
        }
        assert!(last.1.distance(b.joints[0].1) < 1e-6);
        // The level switches at the half way point, not before.
        assert_eq!(blend_character(&a, &b, 0.4).level, SimLevel::Animated);
        assert_eq!(blend_character(&a, &b, 0.6).level, SimLevel::Simulated);
    }

    #[test]
    fn mismatched_or_missing_characters_fall_back_to_the_first_frame() {
        let a = character(0.0, 0.0, SimLevel::Simulated);
        let mut b = character(1.0, 2.0, SimLevel::Simulated);
        b.bodies.push(b.bodies[0]);
        assert_eq!(blend_character(&a, &b, 0.5).joints, a.joints);
        let frames = blend_frames(&vec![Some(a.clone()), None], &vec![None, None], 0.5);
        assert_eq!(frames[0].as_ref().unwrap().joints, a.joints);
        assert!(frames[1].is_none());
    }

    #[test]
    fn whole_frames_are_the_cache_bit_for_bit() {
        let project = crate::document::sample::physics_demo_project();
        let mut sim = Simulation::default();
        sim.sync(1);
        while !sim.advance(&project, Instant::now() + Duration::from_secs(60)) {}
        for frame in [0, 1, 17, 45, 60] {
            let cached = sim.frame(frame).unwrap();
            let got = frame_at(&sim, frame as f64).unwrap();
            assert_eq!(cached.len(), got.len());
            for (c, g) in cached.iter().zip(&got) {
                let (c, g) = (c.as_ref().unwrap(), g.as_ref().unwrap());
                assert_eq!(c.joints, g.joints);
                assert_eq!(c.bodies, g.bodies);
                assert_eq!(c.level, g.level);
            }
        }
        // Half way between two real frames is the average of their positions.
        let a = sim.frame(50).unwrap()[0].as_ref().unwrap();
        let b = sim.frame(51).unwrap()[0].as_ref().unwrap();
        let mid = frame_at(&sim, 50.5).unwrap();
        let m = mid[0].as_ref().unwrap();
        for i in 0..m.joints.len() {
            let expect = (a.joints[i].1 + b.joints[i].1) * 0.5;
            assert!(m.joints[i].1.distance(expect) < 1e-5);
        }
        // Past the computed frames it holds the last one.
        let last = sim.progress().0 as i32 - 1;
        let held = frame_at(&sim, last as f64 + 0.5).unwrap();
        assert_eq!(
            held[0].as_ref().unwrap().joints,
            sim.frame(last).unwrap()[0].as_ref().unwrap().joints
        );
    }
}
