//! Character preview: an orbit camera, a ground grid and the posed skeleton.
//!
//! Phase 1 draws a stick figure with egui's 2D painter after projecting joints on the
//! CPU. That keeps the shell dependency-light; a GPU mesh renderer can replace the
//! drawing later without touching the document or timeline code.

mod camera;

pub use camera::OrbitCamera;

use eframe::egui::{
    self, Align2, Color32, CursorIcon, FontId, Mesh, PointerButton, Pos2, Rect, Sense, Stroke,
    vec2,
};
use glam::{Mat4, Quat, Vec3};

use crate::document::physics::{Geometry, PhysicsRig};
use crate::document::{CharacterKind, Pose, Project, SkeletonRef, world_transforms};

const BG_TOP: Color32 = Color32::from_rgb(44, 48, 56);
const BG_BOTTOM: Color32 = Color32::from_rgb(22, 24, 28);
const GRID_MINOR: Color32 = Color32::from_rgba_premultiplied(14, 14, 14, 14);
const GRID_MAJOR: Color32 = Color32::from_rgba_premultiplied(34, 34, 34, 34);
const SHADOW: Color32 = Color32::from_rgba_premultiplied(0, 0, 0, 70);
const BONE_CENTER: Color32 = Color32::from_rgb(222, 224, 228);
const BONE_LEFT: Color32 = Color32::from_rgb(110, 178, 255);
const BONE_RIGHT: Color32 = Color32::from_rgb(255, 150, 112);
const TEXT: Color32 = Color32::from_rgb(200, 204, 210);
const PROP: Color32 = Color32::from_rgb(196, 170, 120);
const BODY_SIMULATED: Color32 = Color32::from_rgba_premultiplied(70, 40, 14, 70);
const BODY_ANIMATED: Color32 = Color32::from_rgba_premultiplied(30, 40, 52, 60);
const BODY_SCENERY: Color32 = Color32::from_rgba_premultiplied(40, 40, 40, 70);

/// What to draw for one character this frame.
pub struct CharacterView {
    /// World position of every skeleton joint.
    pub joints: Vec<Vec3>,
    /// World transform of every physics body, if the character has a rig.
    pub bodies: Option<Vec<(Quat, Vec3)>>,
    /// Whether it is being simulated right now (drawn in a warmer colour).
    pub simulated: bool,
}

impl CharacterView {
    /// The animated view of a character: forward kinematics on its pose.
    pub fn animated(skeleton: &SkeletonRef, rig: Option<&PhysicsRig>, pose: &Pose) -> Self {
        let transforms = world_transforms(skeleton, pose);
        Self {
            joints: transforms.iter().map(|t| t.1).collect(),
            bodies: rig.map(|rig| crate::physics::rig_bodies(rig, &transforms)),
            simulated: false,
        }
    }
}

pub struct Viewport {
    pub camera: OrbitCamera,
    pub show_grid: bool,
    pub show_joint_names: bool,
    pub show_bodies: bool,
    /// Re-aim the camera at everything in the scene on the next frame.
    pub frame_requested: bool,
}

impl Default for Viewport {
    fn default() -> Self {
        Self {
            camera: OrbitCamera::default(),
            show_grid: true,
            show_joint_names: false,
            show_bodies: true,
            frame_requested: true,
        }
    }
}

impl Viewport {
    /// Draws every character of `project` as described by `views` (one per character, in
    /// order) into the remaining space of `ui`. `hud` lines are overlaid top-left.
    pub fn show(&mut self, ui: &mut egui::Ui, project: &Project, views: &[CharacterView], hud: &[String]) {
        let rect = ui.available_rect_before_wrap();
        let response = ui.allocate_rect(rect, Sense::click_and_drag());
        self.handle_input(ui, &response, rect);

        if std::mem::take(&mut self.frame_requested) {
            // Frame the characters, not the (often huge) scenery.
            let points = project
                .characters
                .iter()
                .zip(views)
                .filter(|(c, _)| c.kind != CharacterKind::Environment)
                .flat_map(|(_, v)| v.joints.iter().copied());
            self.camera.frame(points);
        }

        let painter = ui.painter_at(rect);
        paint_background(&painter, rect);

        let view_proj = self.camera.view_proj(rect.width() / rect.height().max(1.0));
        let project_point = |p: Vec3| OrbitCamera::project(&view_proj, p, rect);

        if self.show_grid {
            paint_grid(&painter, &project_point);
        }

        if self.show_bodies {
            let eye = self.camera.eye();
            for (character, view) in project.characters.iter().zip(views) {
                let (Some(rig), Some(bodies)) = (&character.physics, &view.bodies) else { continue };
                let color = match (character.kind, view.simulated) {
                    (CharacterKind::Environment, _) => BODY_SCENERY,
                    (_, true) => BODY_SIMULATED,
                    (_, false) => BODY_ANIMATED,
                };
                paint_bodies(&painter, &project_point, eye, rig, bodies, color);
            }
        }
        for (character, view) in project.characters.iter().zip(views) {
            if character.kind == CharacterKind::Environment {
                continue;
            }
            let is_prop = character.kind == CharacterKind::Prop;
            self.paint_skeleton(&painter, &project_point, &character.skeleton, &view.joints, is_prop);
        }

        paint_axis_gizmo(&painter, rect, &self.camera.view());

        let mut y = rect.top() + 10.0;
        for line in hud {
            painter.text(
                Pos2::new(rect.left() + 12.0, y),
                Align2::LEFT_TOP,
                line,
                FontId::monospace(12.0),
                TEXT,
            );
            y += 17.0;
        }
        painter.text(
            rect.right_bottom() + vec2(-10.0, -8.0),
            Align2::RIGHT_BOTTOM,
            "drag orbit · right-drag pan · wheel zoom · double-click reset",
            FontId::proportional(11.0),
            TEXT.gamma_multiply(0.5),
        );
    }


    fn paint_skeleton(
        &self,
        painter: &egui::Painter,
        project: &impl Fn(Vec3) -> Option<Pos2>,
        skeleton: &SkeletonRef,
        world: &[Vec3],
        is_prop: bool,
    ) {
        let color_of = |name: &str| {
            let lower = name.to_ascii_lowercase();
            if is_prop {
                PROP
            } else if lower.starts_with("l_") || lower.starts_with("left") {
                BONE_LEFT
            } else if lower.starts_with("r_") || lower.starts_with("right") {
                BONE_RIGHT
            } else {
                BONE_CENTER
            }
        };
        // Ground shadow first so the figure sits on top of it.
        for (i, joint) in skeleton.joints.iter().enumerate() {
            let Some(parent) = joint.parent else { continue };
            let flat = |p: Vec3| Vec3::new(p.x, 0.0, p.z);
            if let (Some(a), Some(b)) = (project(flat(world[parent])), project(flat(world[i]))) {
                painter.line_segment([a, b], Stroke::new(5.0, SHADOW));
            }
        }
        for (i, joint) in skeleton.joints.iter().enumerate() {
            let Some(parent) = joint.parent else { continue };
            if let (Some(a), Some(b)) = (project(world[parent]), project(world[i])) {
                painter.line_segment([a, b], Stroke::new(3.0, color_of(&joint.name)));
            }
        }
        for (i, joint) in skeleton.joints.iter().enumerate() {
            let Some(p) = project(world[i]) else { continue };
            let radius = if joint.parent.is_none() { 5.5 } else { 3.5 };
            painter.circle_filled(p, radius, color_of(&joint.name));
            painter.circle_stroke(p, radius, Stroke::new(1.0, BG_BOTTOM));
            if self.show_joint_names {
                painter.text(
                    p + vec2(7.0, -7.0),
                    Align2::LEFT_BOTTOM,
                    &joint.name,
                    FontId::proportional(11.0),
                    TEXT,
                );
            }
        }
    }

    fn handle_input(&mut self, ui: &egui::Ui, response: &egui::Response, rect: Rect) {
        if response.dragged_by(PointerButton::Primary) {
            self.camera.orbit(response.drag_delta());
            ui.ctx().set_cursor_icon(CursorIcon::Grabbing);
        }
        if response.dragged_by(PointerButton::Secondary) || response.dragged_by(PointerButton::Middle) {
            self.camera.pan(response.drag_delta(), rect.height());
            ui.ctx().set_cursor_icon(CursorIcon::AllScroll);
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.camera.zoom((-scroll * 0.0015).exp());
            }
        }
        if response.double_clicked() {
            self.camera = OrbitCamera::default();
        }
    }
}

/// World-space joint positions for a pose.
#[cfg_attr(not(test), allow(dead_code))]
pub fn forward_kinematics(skeleton: &SkeletonRef, pose: &Pose) -> Vec<Vec3> {
    world_transforms(skeleton, pose).into_iter().map(|(_, p)| p).collect()
}

/// Draws collision shapes as translucent silhouettes.
fn paint_bodies(
    painter: &egui::Painter,
    project: &impl Fn(Vec3) -> Option<Pos2>,
    eye: Vec3,
    rig: &PhysicsRig,
    bodies: &[(Quat, Vec3)],
    color: Color32,
) {
    // On-screen radius of a world-space radius at a point, using a sideways offset.
    let screen_radius = |p: Vec3, r: f32| -> Option<(Pos2, f32)> {
        let side = (p - eye).cross(Vec3::Y).normalize_or_zero();
        let side = if side == Vec3::ZERO { Vec3::X } else { side };
        let c = project(p)?;
        let e = project(p + side * r)?;
        Some((c, c.distance(e)))
    };
    let edge = Stroke::new(1.0, color.gamma_multiply(1.8));
    for (def, &(rotation, position)) in rig.bodies.iter().zip(bodies) {
        for shape in &def.shapes {
            let r = rotation * Quat::from_array(shape.frame.rotation).normalize();
            let p = position + rotation * Vec3::from(shape.frame.position);
            match shape.geometry {
                Geometry::Sphere { radius } => {
                    if let Some((c, sr)) = screen_radius(p, radius) {
                        painter.circle(c, sr, color, edge);
                    }
                }
                Geometry::Capsule { radius, length } | Geometry::Cylinder { radius, length } => {
                    let half = if matches!(shape.geometry, Geometry::Capsule { .. }) { length * 0.5 } else { (length * 0.5 - radius).max(0.0) };
                    let axis = r * Vec3::Z * half;
                    if let (Some((a, ra)), Some((b, _))) = (screen_radius(p - axis, radius), screen_radius(p + axis, radius)) {
                        painter.line_segment([a, b], Stroke::new(ra * 2.0, color));
                        painter.circle(a, ra, color, Stroke::NONE);
                        painter.circle(b, ra, color, Stroke::NONE);
                    }
                }
                Geometry::Box { size } => {
                    let h = Vec3::from(size) * 0.5;
                    let corner = |i: usize| {
                        let s = Vec3::new(
                            if i & 1 == 0 { -1.0 } else { 1.0 },
                            if i & 2 == 0 { -1.0 } else { 1.0 },
                            if i & 4 == 0 { -1.0 } else { 1.0 },
                        );
                        project(p + r * (h * s))
                    };
                    for (i, j) in [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)] {
                        if let (Some(a), Some(b)) = (corner(i), corner(j)) {
                            painter.line_segment([a, b], edge);
                        }
                    }
                }
            }
        }
    }
}

fn paint_background(painter: &egui::Painter, rect: Rect) {
    let mut mesh = Mesh::default();
    mesh.colored_vertex(rect.left_top(), BG_TOP);
    mesh.colored_vertex(rect.right_top(), BG_TOP);
    mesh.colored_vertex(rect.right_bottom(), BG_BOTTOM);
    mesh.colored_vertex(rect.left_bottom(), BG_BOTTOM);
    mesh.add_triangle(0, 1, 2);
    mesh.add_triangle(0, 2, 3);
    painter.add(mesh);
}

fn paint_grid(painter: &egui::Painter, project: &impl Fn(Vec3) -> Option<Pos2>) {
    const HALF: i32 = 12;
    const SPACING: f32 = 0.25;
    let extent = HALF as f32 * SPACING;
    for i in -HALF..=HALF {
        let t = i as f32 * SPACING;
        let color = if i == 0 {
            Color32::from_rgba_premultiplied(60, 60, 60, 60)
        } else if i % 4 == 0 {
            GRID_MAJOR
        } else {
            GRID_MINOR
        };
        for (a, b) in [
            (Vec3::new(t, 0.0, -extent), Vec3::new(t, 0.0, extent)),
            (Vec3::new(-extent, 0.0, t), Vec3::new(extent, 0.0, t)),
        ] {
            if let (Some(a), Some(b)) = (project(a), project(b)) {
                painter.line_segment([a, b], Stroke::new(1.0, color));
            }
        }
    }
}

/// Small X/Y/Z indicator in the bottom-left corner showing the camera's orientation.
fn paint_axis_gizmo(painter: &egui::Painter, rect: Rect, view: &Mat4) {
    let origin = rect.left_bottom() + vec2(36.0, -36.0);
    let axes = [
        (Vec3::X, "X", Color32::from_rgb(230, 92, 92)),
        (Vec3::Y, "Y", Color32::from_rgb(120, 210, 110)),
        (Vec3::Z, "Z", Color32::from_rgb(96, 150, 240)),
    ];
    for (axis, label, color) in axes {
        let v = view.transform_vector3(axis);
        let tip = origin + vec2(v.x, -v.y) * 22.0;
        painter.line_segment([origin, tip], Stroke::new(2.0, color));
        painter.text(tip, Align2::CENTER_CENTER, label, FontId::proportional(11.0), color);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::document::sample::humanoid_skeleton;

    #[test]
    fn rest_pose_stands_on_ground() {
        let skeleton = humanoid_skeleton();
        let world = forward_kinematics(&skeleton, &Pose::rest(skeleton.joints.len()));
        let lowest = world.iter().map(|p| p.y).fold(f32::MAX, f32::min);
        assert!(lowest.abs() < 0.05, "feet should touch the ground, lowest joint at {lowest}");
    }

    #[test]
    fn parent_rotation_moves_children() {
        let skeleton = humanoid_skeleton();
        let shoulder = skeleton.joint_index("r_shoulder").unwrap();
        let wrist = skeleton.joint_index("r_wrist").unwrap();
        let mut pose = Pose::rest(skeleton.joints.len());
        let rest = forward_kinematics(&skeleton, &pose)[wrist];
        pose.joints[shoulder][2] = -170.0;
        let raised = forward_kinematics(&skeleton, &pose)[wrist];
        assert!(raised.y > rest.y + 0.5);
    }
}
