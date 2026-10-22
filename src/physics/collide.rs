//! Narrow-phase collision between primitive shapes.
//!
//! Spheres, capsules and long cylinders are handled as a line segment with a radius (a
//! long cylinder collides as a capsule of the same overall length). Flat cylinders
//! (discs, wheels, turntables) are a flat disc with a rounded rim of half their height.
//! Boxes are oriented boxes. The ground is an infinite plane. Contacts are generated up
//! to `margin` before touching so the solver can stop fast bodies before they tunnel.

use glam::{Mat3, Quat, Vec3};

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
    /// A flat disc of radius `core` across the plane through `center` normal to the
    /// rotation's Z, swept by `radius` (a cylinder shorter than it is wide).
    Disc { rotation: Quat, center: Vec3, core: f32, radius: f32 },
}

/// Points spread around a disc's rim, used to touch it against flat things.
const RIM_POINTS: usize = 6;

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
        Geometry::Cylinder { radius, length } if length >= 2.0 * radius => swept(radius, length * 0.5 - radius),
        Geometry::Cylinder { radius, length } => Form::Disc {
            rotation: shape.rotation,
            center: shape.position,
            core: radius - length * 0.5,
            radius: length * 0.5,
        },
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
        Form::Disc { rotation, center, core, radius } => {
            for (i, p) in rim(rotation, center, core).into_iter().enumerate() {
                push(p, radius, i as u8);
            }
            // A tilted or upright disc (a wheel) rests on the lowest point of its rim,
            // which usually falls between the samples.
            let axis = rotation * Vec3::Z;
            let down = axis * axis.y - Vec3::Y;
            if down.length() > 0.1 {
                push(center + down.normalize() * core, radius, RIM_POINTS as u8);
            }
        }
    }
}

/// Contacts between two shapes (normals from A to B).
pub fn between(a: &Placed, b: &Placed, margin: f32, out: &mut Vec<Contact>) {
    let flip = |found: Vec<Contact>, out: &mut Vec<Contact>| out.extend(found.into_iter().map(|c| Contact { normal: -c.normal, ..c }));
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
        (Form::Cuboid { rotation: ra, center: ca, half: ha }, Form::Cuboid { rotation: rb, center: cb, half: hb }) => {
            box_vs_box(Mat3::from_quat(ra), ca, ha, Mat3::from_quat(rb), cb, hb, margin, out);
        }
        (Form::Disc { rotation, center, core, radius: rd }, Form::Swept { a: s0, b: s1, radius }) => {
            swept_vs_disc(s0, s1, radius, rotation, center, core, rd, margin, out);
        }
        (Form::Swept { a: s0, b: s1, radius }, Form::Disc { rotation, center, core, radius: rd }) => {
            let mut found = Vec::new();
            swept_vs_disc(s0, s1, radius, rotation, center, core, rd, margin, &mut found);
            flip(found, out);
        }
        (Form::Disc { rotation, center, core, radius }, Form::Cuboid { rotation: br, center: bc, half }) => {
            let mut found = Vec::new();
            disc_vs_box(rotation, center, core, radius, br, bc, half, margin, &mut found);
            flip(found, out);
        }
        (Form::Cuboid { rotation: br, center: bc, half }, Form::Disc { rotation, center, core, radius }) => {
            disc_vs_box(rotation, center, core, radius, br, bc, half, margin, out);
        }
        (Form::Disc { rotation: ra, center: ca, core: ka, radius: rad_a }, Form::Disc { rotation: rb, center: cb, core: kb, radius: rad_b }) => {
            // Each rim against the other disc, and the closest points of the two cores.
            for (i, p) in rim(rb, cb, kb).into_iter().enumerate() {
                point_vs_disc(p, rad_b, ra, ca, ka, rad_a, margin, i as u8, out);
            }
            let mut q = cb;
            for _ in 0..6 {
                q = closest_on_disc(closest_on_disc(q, ra, ca, ka), rb, cb, kb);
            }
            point_vs_disc(q, rad_b, ra, ca, ka, rad_a, margin, (2 * RIM_POINTS) as u8, out);
            let mut found = Vec::new();
            for (i, p) in rim(ra, ca, ka).into_iter().enumerate() {
                point_vs_disc(p, rad_a, rb, cb, kb, rad_b, margin, (RIM_POINTS + i) as u8, &mut found);
            }
            flip(found, out);
        }
    }
}

/// Points around a disc's core rim, spread evenly from its local X axis.
fn rim(rotation: Quat, center: Vec3, core: f32) -> [Vec3; RIM_POINTS] {
    std::array::from_fn(|i| {
        let angle = i as f32 * std::f32::consts::TAU / RIM_POINTS as f32;
        center + rotation * Vec3::new(angle.cos() * core, angle.sin() * core, 0.0)
    })
}

/// The point of a disc's core nearest `p`.
fn closest_on_disc(p: Vec3, rotation: Quat, center: Vec3, core: f32) -> Vec3 {
    let axis = rotation * Vec3::Z;
    let in_plane = (p - center) - axis * (p - center).dot(axis);
    center + in_plane.clamp_length_max(core)
}

/// A sphere of radius `r` at `p` against a disc (A). Normals point from the disc.
#[allow(clippy::too_many_arguments)]
fn point_vs_disc(p: Vec3, r: f32, rotation: Quat, center: Vec3, core: f32, radius: f32, margin: f32, feature: u8, out: &mut Vec<Contact>) {
    let q = closest_on_disc(p, rotation, center, core);
    let d = p - q;
    let distance = d.length();
    let normal = if distance > 1e-6 {
        d / distance
    } else {
        // On the core itself: out through the nearer face.
        let axis = rotation * Vec3::Z;
        if (p - center).dot(axis) < 0.0 { -axis } else { axis }
    };
    let separation = distance - radius - r;
    if separation < margin {
        out.push(Contact { point: q + normal * (radius + separation * 0.5), normal, separation, feature });
    }
}

/// Disc is A, swept shape is B.
#[allow(clippy::too_many_arguments)]
fn swept_vs_disc(s0: Vec3, s1: Vec3, r: f32, rotation: Quat, center: Vec3, core: f32, radius: f32, margin: f32, out: &mut Vec<Contact>) {
    let mut candidates = vec![(s0, 0u8)];
    if s0.distance_squared(s1) > 1e-12 {
        candidates.push((s1, 1));
        // The segment point nearest the disc, by alternating projections.
        let mut t = 0.5;
        for _ in 0..4 {
            let q = closest_on_disc(s0.lerp(s1, t), rotation, center, core);
            t = segment_param(q, s0, s1);
        }
        candidates.push((s0.lerp(s1, t), 2));
    }
    for (p, feature) in candidates {
        point_vs_disc(p, r, rotation, center, core, radius, margin, feature, out);
    }
}

/// Box is A, disc is B: the box's corners against the disc, and the disc's rim, centre
/// and point nearest the box against the box.
#[allow(clippy::too_many_arguments)]
fn disc_vs_box(rotation: Quat, center: Vec3, core: f32, radius: f32, box_rotation: Quat, box_center: Vec3, half: Vec3, margin: f32, out: &mut Vec<Contact>) {
    let mut found = Vec::new();
    for (i, corner) in corners(box_rotation, box_center, half).into_iter().enumerate() {
        point_vs_disc(corner, 0.0, rotation, center, core, radius, margin, i as u8, &mut found);
    }
    out.extend(found.into_iter().map(|c| Contact { normal: -c.normal, ..c }));
    // The disc point nearest the box, by alternating projections (both are convex):
    // catches a box passing under the disc away from the rim samples and the centre.
    let mut nearest = center;
    for _ in 0..6 {
        let local = box_rotation.inverse() * (nearest - box_center);
        nearest = closest_on_disc(box_center + box_rotation * local.clamp(-half, half), rotation, center, core);
    }
    let points = rim(rotation, center, core).into_iter().chain([center, nearest]);
    for (i, p) in points.enumerate() {
        let mut one = Vec::new();
        swept_vs_box(p, p, radius, box_rotation, box_center, half, margin, &mut one);
        out.extend(one.into_iter().map(|c| Contact { feature: 8 + i as u8, ..c }));
    }
}

/// Oriented boxes A and B (rotation matrices, centres, half sizes): the separating axis
/// with the least overlap decides the normal. A face axis clips the other box's most
/// opposed face against the reference face (up to 8 points); an edge pair touches at
/// one point.
#[allow(clippy::too_many_arguments)]
fn box_vs_box(ra: Mat3, ca: Vec3, ha: Vec3, rb: Mat3, cb: Vec3, hb: Vec3, margin: f32, out: &mut Vec<Contact>) {
    let d = cb - ca;
    let project = |r: Mat3, h: Vec3, axis: Vec3| (0..3).map(|k| h[k] * r.col(k).dot(axis).abs()).sum::<f32>();
    let separation = |axis: Vec3| d.dot(axis).abs() - project(ra, ha, axis) - project(rb, hb, axis);

    // Best face axis: (separation, on A, index).
    let mut face = (f32::MIN, true, 0);
    for (on_a, r) in [(true, ra), (false, rb)] {
        for k in 0..3 {
            let s = separation(r.col(k));
            if s > margin {
                return;
            }
            if s > face.0 {
                face = (s, on_a, k);
            }
        }
    }
    let mut edge = (f32::MIN, 0, 0, Vec3::ZERO);
    for i in 0..3 {
        for j in 0..3 {
            let axis = ra.col(i).cross(rb.col(j));
            let len = axis.length();
            if len < 1e-4 {
                continue;
            }
            let axis = axis / len;
            let s = separation(axis);
            if s > margin {
                return;
            }
            if s > edge.0 {
                edge = (s, i, j, axis);
            }
        }
    }

    // Prefer faces unless an edge pair is clearly shallower: face contacts are stabler.
    if edge.0 > face.0 + 0.01 {
        let (s, i, j, axis) = edge;
        let n = if d.dot(axis) < 0.0 { -axis } else { axis };
        // The edge of each box nearest the other.
        let edge_of = |r: Mat3, c: Vec3, h: Vec3, k: usize, toward: Vec3| {
            let mut mid = c;
            for m in (0..3).filter(|&m| m != k) {
                mid += r.col(m) * h[m] * r.col(m).dot(toward).signum();
            }
            (mid - r.col(k) * h[k], mid + r.col(k) * h[k])
        };
        let (a0, a1) = edge_of(ra, ca, ha, i, n);
        let (b0, b1) = edge_of(rb, cb, hb, j, -n);
        let (pa, pb) = closest_between_segments(a0, a1, b0, b1);
        out.push(Contact { point: (pa + pb) * 0.5, normal: n, separation: s, feature: 16 + (i * 3 + j) as u8 });
        return;
    }

    // Reference face on one box, incident face on the other.
    let (_, on_a, k) = face;
    let (rr, cr, hr, ri, ci, hi) = if on_a { (ra, ca, ha, rb, cb, hb) } else { (rb, cb, hb, ra, ca, ha) };
    let towards = if on_a { d } else { -d };
    let nr = rr.col(k) * towards.dot(rr.col(k)).signum();
    let face_center = cr + nr * hr[k];
    // Incident face: the other box's face most opposed to the reference normal.
    let m = (0..3).max_by(|&x, &y| ri.col(x).dot(nr).abs().total_cmp(&ri.col(y).dot(nr).abs())).unwrap();
    let (u, v) = ((m + 1) % 3, (m + 2) % 3);
    let incident_center = ci - ri.col(m) * hi[m] * ri.col(m).dot(nr).signum();
    let mut polygon: Vec<(Vec3, u8)> = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        .iter()
        .enumerate()
        .map(|(i, &(su, sv))| (incident_center + ri.col(u) * hi[u] * su + ri.col(v) * hi[v] * sv, i as u8))
        .collect();
    // Clip against the reference face's four side planes.
    let (ru, rv) = ((k + 1) % 3, (k + 2) % 3);
    let sides = [(rr.col(ru), hr[ru]), (-rr.col(ru), hr[ru]), (rr.col(rv), hr[rv]), (-rr.col(rv), hr[rv])];
    for (plane, (axis, limit)) in (0u8..).zip(sides) {
        let outside = |p: Vec3| (p - face_center).dot(axis) - limit;
        let mut clipped = Vec::with_capacity(polygon.len() + 1);
        for (i, &(p, f)) in polygon.iter().enumerate() {
            let q = polygon[(i + 1) % polygon.len()].0;
            let (dp, dq) = (outside(p), outside(q));
            if dp <= 0.0 {
                clipped.push((p, f));
            }
            if (dp <= 0.0) != (dq <= 0.0) {
                // A convex polygon crosses each plane at most twice: leaving and entering.
                clipped.push((p + (q - p) * (dp / (dp - dq)), 4 + 2 * plane + u8::from(dp <= 0.0)));
            }
        }
        polygon = clipped;
        if polygon.is_empty() {
            return;
        }
    }
    let normal = if on_a { nr } else { -nr };
    for (p, feature) in polygon {
        let s = (p - face_center).dot(nr);
        if s < margin {
            out.push(Contact { point: p - nr * (s * 0.5), normal, separation: s, feature });
        }
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

    fn cuboid(size: [f32; 3], rotation: Quat, at: Vec3) -> Placed {
        Placed { geometry: Geometry::Box { size }, rotation, position: at }
    }

    #[test]
    fn box_resting_on_box_touches_at_its_four_corners() {
        let floor = cuboid([4.0, 0.2, 4.0], Quat::IDENTITY, Vec3::new(0.0, -0.1, 0.0));
        let foot = cuboid([0.1, 0.06, 0.24], Quat::from_rotation_y(0.4), Vec3::new(0.3, 0.025, 0.2));
        let mut out = Vec::new();
        between(&floor, &foot, 0.02, &mut out);
        assert_eq!(out.len(), 4, "{out:?}");
        for c in &out {
            assert!((c.normal - Vec3::Y).length() < 1e-5);
            assert!((c.separation + 0.005).abs() < 1e-5, "{}", c.separation);
        }
        // Same contacts, flipped, the other way round.
        let mut back = Vec::new();
        between(&foot, &floor, 0.02, &mut back);
        assert_eq!(back.len(), 4);
        assert!(back.iter().all(|c| (c.normal + Vec3::Y).length() < 1e-5));
    }

    #[test]
    fn crossed_box_edges_touch_at_one_point() {
        // Two long bars crossed at right angles, each rotated 45° about its length so
        // they meet edge to edge.
        let a = cuboid([2.0, 0.1, 0.1], Quat::from_rotation_x(std::f32::consts::FRAC_PI_4), Vec3::ZERO);
        let b = cuboid([0.1, 0.1, 2.0], Quat::from_rotation_z(std::f32::consts::FRAC_PI_4), Vec3::new(0.0, 0.14, 0.0));
        let mut out = Vec::new();
        between(&a, &b, 0.02, &mut out);
        assert_eq!(out.len(), 1, "{out:?}");
        let gap = 0.14 - 0.1 * std::f32::consts::SQRT_2;
        assert!((out[0].separation - gap).abs() < 1e-4, "{}", out[0].separation);
        assert!((out[0].normal - Vec3::Y).length() < 1e-4);
    }

    #[test]
    fn separated_boxes_do_not_touch() {
        let a = cuboid([1.0, 1.0, 1.0], Quat::IDENTITY, Vec3::ZERO);
        let b = cuboid([1.0, 1.0, 1.0], Quat::from_rotation_z(0.3), Vec3::new(1.8, 0.0, 0.0));
        let mut out = Vec::new();
        between(&a, &b, 0.02, &mut out);
        assert!(out.is_empty());
    }

    /// A flat cylinder is a disc, not a sphere of its radius.
    #[test]
    fn flat_cylinder_is_a_disc() {
        let disc = Placed {
            geometry: Geometry::Cylinder { radius: 2.0, length: 0.2 },
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            position: Vec3::new(0.0, -0.1, 0.0),
        };
        // A ball resting on top, well away from the centre.
        let mut out = Vec::new();
        between(&disc, &sphere(0.1, Vec3::new(1.5, 0.09, 0.0)), 0.02, &mut out);
        assert_eq!(out.len(), 1);
        assert!((out[0].normal - Vec3::Y).length() < 1e-5);
        assert!((out[0].separation + 0.01).abs() < 1e-5);
        // Nothing just past the rim.
        out.clear();
        between(&disc, &sphere(0.1, Vec3::new(2.15, 0.09, 0.0)), 0.02, &mut out);
        assert!(out.is_empty());
        // A box foot on it touches at its four bottom corners (and the disc point nearest
        // it); the disc lying on the ground touches around its rim.
        out.clear();
        between(&disc, &cuboid([0.1, 0.06, 0.24], Quat::IDENTITY, Vec3::new(0.5, 0.025, 0.0)), 0.02, &mut out);
        assert_eq!(out.len(), 5);
        assert!(out.iter().all(|c| (c.normal - Vec3::Y).length() < 1e-4));
        out.clear();
        with_ground(&Placed { position: Vec3::new(0.0, 0.1, 0.0), ..disc }, 0.0, 0.01, &mut out);
        assert_eq!(out.len(), RIM_POINTS);
    }

    /// An upright disc (a wheel) rests on the lowest point of its rim, between samples,
    /// and a disc lying across a long beam away from its centre still lands on it.
    #[test]
    fn discs_touch_between_rim_samples() {
        let wheel = Placed {
            geometry: Geometry::Cylinder { radius: 0.5, length: 0.1 },
            rotation: Quat::from_rotation_y(0.3),
            position: Vec3::new(0.0, 0.5, 0.0),
        };
        let mut out = Vec::new();
        with_ground(&wheel, 0.0, 0.01, &mut out);
        assert!(out.iter().any(|c| c.separation.abs() < 1e-4), "{out:?}");

        let disc = Placed {
            geometry: Geometry::Cylinder { radius: 2.0, length: 0.2 },
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            position: Vec3::new(0.0, 0.09, 0.0),
        };
        let beam = cuboid([10.0, 0.2, 0.2], Quat::IDENTITY, Vec3::new(0.0, -0.1, 1.2));
        out.clear();
        between(&beam, &disc, 0.02, &mut out);
        assert!(out.iter().any(|c| (c.normal - Vec3::Y).length() < 1e-4 && c.separation < 0.0), "{out:?}");
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
