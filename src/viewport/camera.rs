//! Orbit camera around a target point.

use eframe::egui::{Pos2, Rect, Vec2, pos2};
use glam::{Mat4, Vec3};

#[derive(Clone, Debug)]
pub struct OrbitCamera {
    pub target: Vec3,
    /// Radians around the vertical axis; 0 looks at the character's front (+Z).
    pub yaw: f32,
    /// Radians above the ground plane.
    pub pitch: f32,
    pub distance: f32,
    pub fov_y: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: Vec3::new(0.0, 0.9, 0.0),
            yaw: 0.55,
            pitch: 0.18,
            distance: 4.2,
            fov_y: 38f32.to_radians(),
        }
    }
}

impl OrbitCamera {
    pub fn eye(&self) -> Vec3 {
        let dir = Vec3::new(
            self.yaw.sin() * self.pitch.cos(),
            self.pitch.sin(),
            self.yaw.cos() * self.pitch.cos(),
        );
        self.target + dir * self.distance
    }

    pub fn view(&self) -> Mat4 {
        Mat4::look_at_rh(self.eye(), self.target, Vec3::Y)
    }

    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        Mat4::perspective_rh(self.fov_y, aspect.max(0.1), 0.05, 200.0) * self.view()
    }

    /// Projects a world point into `rect`; `None` when it's behind the camera.
    pub fn project(view_proj: &Mat4, p: Vec3, rect: Rect) -> Option<Pos2> {
        let clip = *view_proj * p.extend(1.0);
        if clip.w <= 0.05 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        Some(pos2(
            rect.center().x + ndc.x * rect.width() * 0.5,
            rect.center().y - ndc.y * rect.height() * 0.5,
        ))
    }

    pub fn orbit(&mut self, drag: Vec2) {
        self.yaw -= drag.x * 0.008;
        self.pitch = (self.pitch + drag.y * 0.008).clamp(-1.45, 1.45);
    }

    /// Slides the target in the view plane so the scene follows the pointer.
    pub fn pan(&mut self, drag: Vec2, viewport_height: f32) {
        let forward = (self.target - self.eye()).normalize();
        let right = forward.cross(Vec3::Y).normalize();
        let up = right.cross(forward);
        let world_per_px = 2.0 * self.distance * (self.fov_y * 0.5).tan() / viewport_height.max(1.0);
        self.target += (-right * drag.x + up * drag.y) * world_per_px;
    }

    /// Aims at the bounding box of `points` and backs off far enough to see all of it.
    /// Keeps the current viewing angle; does nothing for an empty set.
    pub fn frame(&mut self, points: impl Iterator<Item = Vec3>) {
        let (mut min, mut max) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
        for p in points {
            min = min.min(p);
            max = max.max(p);
        }
        if min.x > max.x {
            return;
        }
        let radius = ((max - min).length() * 0.5).max(0.5);
        self.target = (min + max) * 0.5;
        self.distance = (radius / (self.fov_y * 0.5).tan() * 1.15).clamp(0.4, 60.0);
    }

    pub fn zoom(&mut self, factor: f32) {
        self.distance = (self.distance * factor).clamp(0.4, 60.0);
    }
}
