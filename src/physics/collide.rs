//! Narrow-phase collision between primitive shapes.
//!
//! Spheres, capsules and cylinders are all handled as a line segment with a radius
//! (a cylinder collides as a capsule of the same overall length). Boxes are oriented
//! boxes. The ground is an infinite plane. Contacts are generated up to `margin`
//! before touching so the solver can stop fast bodies before they tunnel.

use glam::{Quat, Vec3};

use crate::document::physics::Geometry;

/// A shape placed in the world.
#[derive(Clone, Copy, Debug)]
pub struct Placed {
    pub geometry: Geometry,
    pub rotation: Quat,
    pub position: Vec3,
}

/// One contact between shape A and shape B.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    /// Midway between the two surfaces.
    pub point: Vec3,
    /// Unit vector pointing from A towards B.
    pub normal: Vec3,
    /// Gap between the surfaces; negative when they overlap.
    pub separation: f32,
    /// Stable id of the feature that produced it (for warm starting).
    pub feature: u8,
}

enum Form {
    /// Segment from `a` to `b` swept by `radius` (a sphere has a == b).
    Swept { a: Vec3, b: Vec3, radius: f32 },
    Cuboid { rotation: Quat, center: Vec3, half: Vec3 },
}

fn form(shape: &Placed) -> Form {
    let axis = shape.rotation * Vec3::Z;
    let swept = |radius: f32, half_len: f32| Form::Swept {
        a: shape.position - axis * half_len,
        b: shape.position + axis * half_len,
        radius,
    };
    match shape.geometry {
        Geometry::Sphere { radius } => swept(radius, 0.0),
        Geometry::Capsule { radius, length } => swept(radius, length * 0.5),
        Geometry::Cylinder { radius, length } => swept(radius, (length * 0.5 - radius).max(0.0)),
        Geometry::Box { size } => Form::Cuboid {
            rotation: shape.rotation,
            center: shape.position,
            half: Vec3::from(size) * 0.5,
        },
    }
}

/// Radius of a sphere around `shape.position` that contains the whole shape.
pub fn bounding_radius(geometry: &Geometry) -> f32 {
    match *geometry {
        Geometry::Sphere { radius } => radius,
        Geometry::Capsule { radius, length } => radius + length * 0.5,
        Geometry::Cylinder { radius, length } => (radius * radius + length * length * 0.25).sqrt(),
        Geometry::Box { size } => Vec3::from(size).length() * 0.5,
    }
}

/// Contacts between `shape` and the plane `y = height` (plane is A, normal +Y).
pub fn with_ground(shape: &Placed, height: f32, margin: f32, out: &mut Vec<Contact>) {
    let mut push = |p: Vec3, radius: f32, feature: u8| {
        let separation = p.y - height - radius;
        if separation < margin {
            out.push(Contact {
                point: Vec3::new(p.x, height + separation * 0.5, p.z),
                normal: Vec3::Y,
                separation,
                feature,
            });
        }
    };
    match form(shape) {
        Form::Swept { a, b, radius } => {
            push(a, radius, 0);
            if a.distance_squared(b) > 1e-12 {
                push(b, radius, 1);
            }
        }
        Form::Cuboid { rotation, center, half } => {
            for (i, corner) in corners(rotation, center, half).into_iter().enumerate() {
                push(corner, 0.0, i as u8);
            }
        }
    }
}

/// Contacts between two shapes (normals from A to B). Box-box isn't supported yet.
pub fn between(a: &Placed, b: &Placed, margin: f32, out: &mut Vec<Contact>) {
    match (form(a), form(b)) {
        (Form::Swept { a: a0, b: a1, radius: ra }, Form::Swept { a: b0, b: b1, radius: rb }) => {
            let (pa, pb) = closest_between_segments(a0, a1, b0, b1);
            push_spheres(pa, ra, pb, rb, margin, 0, out);
            // Parallel-ish capsules resting on each other need a second point to stop rolling.
            if a0.distance_squared(a1) > 1e-8 && b0.distance_squared(b1) > 1e-8 {
                let far_a = if pa.distance_squared(a0) > pa.distance_squared(a1) { a0 } else { a1 };
                let qb = closest_on_segment(far_a, b0, b1);
                if far_a.distance_squared(pa) > 1e-6 {
                    push_spheres(far_a, ra, qb, rb, margin, 1, out);
                }
            }
        }
        (Form::Swept { a: s0, b: s1, radius }, Form::Cuboid { rotation, center, half }) => {
            let mut flipped = Vec::new();
            swept_vs_box(s0, s1, radius, rotation, center, half, margin, &mut flipped);
            out.extend(flipped.into_iter().map(|c| Contact { normal: -c.normal, ..c }));
        }
        (Form::Cuboid { rotation, center, half }, Form::Swept { a: s0, b: s1, radius }) => {
            swept_vs_box(s0, s1, radius, rotation, center, half, margin, out);
        }
        (Form::Cuboid { .. }, Form::Cuboid { .. }) => {}
    }
}

fn push_spheres(pa: Vec3, ra: f32, pb: Vec3, rb: f32, margin: f32, feature: u8, out: &mut Vec<Contact>) {
    let delta = pb - pa;
    let distance = delta.length();
    let separation = distance - ra - rb;
    if separation >= margin {
        return;
    }
    let normal = if distance > 1e-6 { delta / distance } else { Vec3::Y };
    out.push(Contact {
        point: pa + normal * (ra + separation * 0.5),
        normal,
        separation,
        feature,
    });
}

/// Box is A, swept shape is B. Normals point from the box towards the swept shape.
#[allow(clippy::too_many_arguments)]
fn swept_vs_box(s0: Vec3, s1: Vec3, radius: f32, rotation: Quat, center: Vec3, half: Vec3, margin: f32, out: &mut Vec<Contact>) {
    let inv = rotation.inverse();
    let to_local = |p: Vec3| inv * (p - center);
    let (l0, l1) = (to_local(s0), to_local(s1));

    let mut candidates = vec![(l0, 0u8)];
    if l0.distance_squared(l1) > 1e-12 {
        candidates.push((l1, 1));
        // The segment point nearest the box, found by alternating projections.
        let mut t = 0.5;
        for _ in 0..4 {
            let p = l0.lerp(l1, t);
            let q = p.clamp(-half, half);
            t = segment_param(q, l0, l1);
        }
        candidates.push((l0.lerp(l1, t), 2));
    }
    for (p, feature) in candidates {
        let q = p.clamp(-half, half);
        let (normal_local, separation) = if p == q {
            // Centre inside the box: push out through the nearest face.
            let depth = half - p.abs();
            let axis = if depth.x < depth.y && depth.x < depth.z {
                0
            } else if depth.y < depth.z {
                1
            } else {
                2
            };
            let mut n = Vec3::ZERO;
            n[axis] = p[axis].signum();
            (n, -depth[axis] - radius)
        } else {
            let d = p - q;
            let len = d.length();
            (d / len, len - radius)
        };
        if separation < margin {
            let normal = rotation * normal_local;
            let surface = center + rotation * q;
            out.push(Contact {
                point: surface + normal * (separation * 0.5).max(0.0),
                normal,
                separation,
                feature,
            });
        }
    }
}

fn corners(rotation: Quat, center: Vec3, half: Vec3) -> [Vec3; 8] {
    let mut out = [Vec3::ZERO; 8];
    for (i, c) in out.iter_mut().enumerate() {
        let sign = Vec3::new(
            if i & 1 == 0 { -1.0 } else { 1.0 },
            if i & 2 == 0 { -1.0 } else { 1.0 },
            if i & 4 == 0 { -1.0 } else { 1.0 },
        );
        *c = center + rotation * (half * sign);
    }
    out
}

fn segment_param(p: Vec3, a: Vec3, b: Vec3) -> f32 {
    let ab = b - a;
    let len2 = ab.length_squared();
    if len2 < 1e-12 { 0.0 } else { ((p - a).dot(ab) / len2).clamp(0.0, 1.0) }
}

fn closest_on_segment(p: Vec3, a: Vec3, b: Vec3) -> Vec3 {
    a.lerp(b, segment_param(p, a, b))
}

/// Closest points between segments `p0-p1` and `q0-q1`.
pub fn closest_between_segments(p0: Vec3, p1: Vec3, q0: Vec3, q1: Vec3) -> (Vec3, Vec3) {
    let d1 = p1 - p0;
    let d2 = q1 - q0;
    let r = p0 - q0;
    let a = d1.length_squared();
    let e = d2.length_squared();
    let f = d2.dot(r);
    let (s, t) = if a <= 1e-12 && e <= 1e-12 {
        (0.0, 0.0)
    } else if a <= 1e-12 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= 1e-12 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s = if denom > 1e-12 { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    (p0 + d1 * s, q0 + d2 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sphere(r: f32, at: Vec3) -> Placed {
        Placed { geometry: Geometry::Sphere { radius: r }, rotation: Quat::IDENTITY, position: at }
    }

    #[test]
    fn sphere_on_ground() {
        let mut out = Vec::new();
        with_ground(&sphere(0.1, Vec3::new(0.0, 0.08, 0.0)), 0.0, 0.02, &mut out);
        assert_eq!(out.len(), 1);
        assert!((out[0].separation + 0.02).abs() < 1e-6);
        assert_eq!(out[0].normal, Vec3::Y);
    }

    #[test]
    fn capsule_lying_on_ground_has_two_contacts() {
        let capsule = Placed {
            geometry: Geometry::Capsule { radius: 0.05, length: 0.4 },
            rotation: Quat::from_rotation_y(1.2),
            position: Vec3::new(0.0, 0.05, 0.0),
        };
        let mut out = Vec::new();
        with_ground(&capsule, 0.0, 0.01, &mut out);
        assert_eq!(out.len(), 2);
    }

    #[test]
    fn overlapping_spheres_push_apart() {
        let mut out = Vec::new();
        between(&sphere(0.1, Vec3::ZERO), &sphere(0.1, Vec3::new(0.15, 0.0, 0.0)), 0.0, &mut out);
        assert_eq!(out.len(), 1);
        assert!((out[0].separation + 0.05).abs() < 1e-5);
        assert!((out[0].normal - Vec3::X).length() < 1e-5);
    }

    #[test]
    fn sphere_resting_on_box_points_up() {
        let table = Placed { geometry: Geometry::Box { size: [1.0, 0.2, 1.0] }, rotation: Quat::IDENTITY, position: Vec3::ZERO };
        let mut out = Vec::new();
        between(&table, &sphere(0.1, Vec3::new(0.2, 0.19, 0.0)), 0.02, &mut out);
        assert_eq!(out.len(), 1);
        assert!((out[0].normal - Vec3::Y).length() < 1e-5);
        assert!((out[0].separation + 0.01).abs() < 1e-5);
    }

    #[test]
    fn segment_closest_points() {
        let (p, q) = closest_between_segments(
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, -1.0),
            Vec3::new(0.0, 1.0, 1.0),
        );
        assert!(p.length() < 1e-6 && (q - Vec3::Y).length() < 1e-6);
    }
}
