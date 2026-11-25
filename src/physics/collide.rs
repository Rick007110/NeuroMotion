//! Narrow-phase collision between primitive shapes.
//!
//! Spheres and capsules are handled as a line segment with a radius. Cylinders are
//! exact: flat caps with a sharp rim, long ones (logs, forearms) and flat ones (discs,
//! wheels, turntables) alike. Boxes are oriented boxes. The ground is an infinite
//! plane. Contacts are generated up to `margin` before touching so the solver can stop
//! fast bodies before they tunnel. Flat faces resting on each other touch at several
//! points (a box's corners, a cap's rim as a polygon) so they rest without rocking.

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
    Swept {
        a: Vec3,
        b: Vec3,
        radius: f32,
    },
    Cuboid {
        rotation: Quat,
        center: Vec3,
        half: Vec3,
    },
    Cylinder(Cyl),
}

/// Points spread around a cylinder's cap rim, used to touch it against flat things.
const RIM_POINTS: usize = 6;

/// A cap rests on its rim points only while it faces what it touches within 60°
/// (the cosine). Lying on its side, a cylinder rests on its side line instead: rim
/// points near the bottom then turn with it as it rolls, and pushing up behind the
/// line they would roll it further, faster and faster.
const CAP_FACING: f32 = 0.5;

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
        Geometry::Cylinder { radius, length } => Form::Cylinder(Cyl {
            rotation: shape.rotation,
            center: shape.position,
            radius,
            half: length * 0.5,
        }),
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
        Form::Cuboid {
            rotation,
            center,
            half,
        } => {
            for (i, corner) in corners(rotation, center, half).into_iter().enumerate() {
                push(corner, 0.0, i as u8);
            }
        }
        Form::Cylinder(c) => {
            // A cap down rests on its rim; a cylinder on its side, or tilted onto its
            // rim, on the lowest point of each cap's rim, which falls between samples.
            if let Some((first, rim)) = c.facing_rim(-Vec3::Y) {
                for (i, p) in (first..).zip(rim) {
                    push(p, 0.0, i);
                }
            }
            if let Some((p0, p1)) = c.side_line(-Vec3::Y) {
                push(p0, 0.0, 2 * RIM_POINTS as u8);
                push(p1, 0.0, 2 * RIM_POINTS as u8 + 1);
            }
        }
    }
}

/// Contacts between two shapes (normals from A to B).
pub fn between(a: &Placed, b: &Placed, margin: f32, out: &mut Vec<Contact>) {
    let flip = |found: Vec<Contact>, out: &mut Vec<Contact>| {
        out.extend(found.into_iter().map(|c| Contact {
            normal: -c.normal,
            ..c
        }))
    };
    match (form(a), form(b)) {
        (
            Form::Swept {
                a: a0,
                b: a1,
                radius: ra,
            },
            Form::Swept {
                a: b0,
                b: b1,
                radius: rb,
            },
        ) => {
            let (pa, pb) = closest_between_segments(a0, a1, b0, b1);
            push_spheres(pa, ra, pb, rb, margin, 0, out);
            // Parallel-ish capsules resting on each other need a second point to stop rolling.
            if a0.distance_squared(a1) > 1e-8 && b0.distance_squared(b1) > 1e-8 {
                let far_a = if pa.distance_squared(a0) > pa.distance_squared(a1) {
                    a0
                } else {
                    a1
                };
                let qb = closest_on_segment(far_a, b0, b1);
                if far_a.distance_squared(pa) > 1e-6 {
                    push_spheres(far_a, ra, qb, rb, margin, 1, out);
                }
            }
        }
        (
            Form::Swept {
                a: s0,
                b: s1,
                radius,
            },
            Form::Cuboid {
                rotation,
                center,
                half,
            },
        ) => {
            let mut flipped = Vec::new();
            swept_vs_box(s0, s1, radius, rotation, center, half, margin, &mut flipped);
            out.extend(flipped.into_iter().map(|c| Contact {
                normal: -c.normal,
                ..c
            }));
        }
        (
            Form::Cuboid {
                rotation,
                center,
                half,
            },
            Form::Swept {
                a: s0,
                b: s1,
                radius,
            },
        ) => {
            swept_vs_box(s0, s1, radius, rotation, center, half, margin, out);
        }
        (
            Form::Cuboid {
                rotation: ra,
                center: ca,
                half: ha,
            },
            Form::Cuboid {
                rotation: rb,
                center: cb,
                half: hb,
            },
        ) => {
            box_vs_box(
                Mat3::from_quat(ra),
                ca,
                ha,
                Mat3::from_quat(rb),
                cb,
                hb,
                margin,
                out,
            );
        }
        (
            Form::Cylinder(c),
            Form::Swept {
                a: s0,
                b: s1,
                radius,
            },
        ) => {
            swept_vs_cylinder(s0, s1, radius, &c, margin, out);
        }
        (
            Form::Swept {
                a: s0,
                b: s1,
                radius,
            },
            Form::Cylinder(c),
        ) => {
            let mut found = Vec::new();
            swept_vs_cylinder(s0, s1, radius, &c, margin, &mut found);
            flip(found, out);
        }
        (
            Form::Cylinder(c),
            Form::Cuboid {
                rotation,
                center,
                half,
            },
        ) => {
            let mut found = Vec::new();
            cylinder_vs_box(&c, rotation, center, half, margin, &mut found);
            flip(found, out);
        }
        (
            Form::Cuboid {
                rotation,
                center,
                half,
            },
            Form::Cylinder(c),
        ) => {
            cylinder_vs_box(&c, rotation, center, half, margin, out);
        }
        (Form::Cylinder(ca), Form::Cylinder(cb)) => {
            cylinder_vs_cylinder(&ca, &cb, margin, out);
        }
    }
}

/// A placed cylinder: its axis is the rotation's Z.
#[derive(Clone, Copy)]
struct Cyl {
    rotation: Quat,
    center: Vec3,
    radius: f32,
    half: f32,
}

impl Cyl {
    fn axis(&self) -> Vec3 {
        self.rotation * Vec3::Z
    }

    /// Centre of cap `side` (-1 or +1 along the axis).
    fn cap(&self, side: f32) -> Vec3 {
        self.center + self.axis() * (self.half * side)
    }

    /// Points around the rim of cap `side`, spread evenly from the local X axis: the
    /// cap as a polygon, to rest it on flat things.
    fn rim(&self, side: f32) -> [Vec3; RIM_POINTS] {
        std::array::from_fn(|i| {
            let angle = i as f32 * std::f32::consts::TAU / RIM_POINTS as f32;
            let local = Vec3::new(
                angle.cos() * self.radius,
                angle.sin() * self.radius,
                self.half * side,
            );
            self.center + self.rotation * local
        })
    }

    /// The rim of the cap facing `dir` (a unit vector), and the feature id of its
    /// first point, if it faces `dir` within the `CAP_FACING` angle.
    fn facing_rim(&self, dir: Vec3) -> Option<(u8, [Vec3; RIM_POINTS])> {
        let along = self.axis().dot(dir);
        (along.abs() >= CAP_FACING).then(|| {
            let side = along.signum();
            (
                if side < 0.0 { 0 } else { RIM_POINTS as u8 },
                self.rim(side),
            )
        })
    }

    /// The point furthest along `dir` on each cap's rim (the line of the side facing
    /// `dir`), or nothing when the caps face `dir` square on and every rim point is
    /// as far along it as the others.
    fn side_line(&self, dir: Vec3) -> Option<(Vec3, Vec3)> {
        let axis = self.axis();
        let across = dir - axis * dir.dot(axis);
        let len = across.length();
        (len > 1e-4 * dir.length()).then(|| {
            let out = across * (self.radius / len);
            (self.cap(-1.0) + out, self.cap(1.0) + out)
        })
    }

    /// Half the cylinder's extent along the unit vector `n`.
    fn extent(&self, n: Vec3) -> f32 {
        let along = self.axis().dot(n).abs();
        self.half * along + self.radius * (1.0 - along * along).max(0.0).sqrt()
    }

    /// The point of the (solid) cylinder nearest `p`.
    fn closest(&self, p: Vec3) -> Vec3 {
        let local = self.rotation.inverse() * (p - self.center);
        let across = local.truncate().clamp_length_max(self.radius);
        self.center + self.rotation * across.extend(local.z.clamp(-self.half, self.half))
    }
}

/// A sphere of radius `r` at `p` against cylinder `c` (A): the exact nearest points, or
/// when `p` is inside, out through the nearer of the caps and the side. Normals point
/// from the cylinder.
fn point_vs_cylinder(p: Vec3, r: f32, c: &Cyl, margin: f32, feature: u8, out: &mut Vec<Contact>) {
    let local = c.rotation.inverse() * (p - c.center);
    let across = local.truncate();
    let from_axis = across.length();
    let (surface, normal_local, separation) = if from_axis <= c.radius && local.z.abs() <= c.half {
        let to_cap = c.half - local.z.abs();
        let to_side = c.radius - from_axis;
        if to_cap <= to_side {
            let side = if local.z < 0.0 { -1.0 } else { 1.0 };
            (across.extend(c.half * side), Vec3::Z * side, -to_cap - r)
        } else {
            let out_dir = if from_axis > 1e-6 {
                across / from_axis
            } else {
                glam::Vec2::X
            };
            (
                (out_dir * c.radius).extend(local.z),
                out_dir.extend(0.0),
                -to_side - r,
            )
        }
    } else {
        let q = across
            .clamp_length_max(c.radius)
            .extend(local.z.clamp(-c.half, c.half));
        let d = local - q;
        let len = d.length();
        if len < 1e-9 {
            return;
        }
        (q, d / len, len - r)
    };
    if separation < margin {
        let normal = c.rotation * normal_local;
        out.push(Contact {
            point: c.center + c.rotation * surface + normal * (separation * 0.5),
            normal,
            separation,
            feature,
        });
    }
}

/// A point of another cylinder against cylinder `c` (A), which it touches along `n`
/// (a unit vector from `c`): against the cap facing `n` if one does within the
/// `CAP_FACING` angle and `p` is over it, else against the side if `p` is beside it,
/// else the exact nearest points. A point on the rim's very edge (where a disc
/// stacked on one as wide, or a log beside one as long, has its corners) then pushes
/// the way they touch, not off the cap or side it is just as near.
fn point_toward_cylinder(
    p: Vec3,
    c: &Cyl,
    n: Vec3,
    margin: f32,
    feature: u8,
    out: &mut Vec<Contact>,
) {
    // A millimetre's grace past the rim or the cap, for edges that line up.
    const GRACE: f32 = 1e-3;
    let local = c.rotation.inverse() * (p - c.center);
    let across = local.truncate();
    let from_axis = across.length();
    let along = c.axis().dot(n);
    let (normal_local, separation) = if along.abs() >= CAP_FACING {
        if from_axis > c.radius + GRACE || local.z.abs() > c.half + c.half.max(margin) {
            return point_vs_cylinder(p, 0.0, c, margin, feature, out);
        }
        let side = along.signum();
        (Vec3::Z * side, local.z * side - c.half)
    } else {
        if local.z.abs() > c.half + GRACE || from_axis < 1e-6 {
            return point_vs_cylinder(p, 0.0, c, margin, feature, out);
        }
        ((across / from_axis).extend(0.0), from_axis - c.radius)
    };
    if separation < margin {
        let normal = c.rotation * normal_local;
        out.push(Contact {
            point: p - normal * (separation * 0.5),
            normal,
            separation,
            feature,
        });
    }
}

/// Alternating projections between two convex shapes, given each one's nearest-point
/// map: the pair of nearest points when they are apart, a point in both when they
/// overlap.
fn nearest_pair(
    start: Vec3,
    onto_a: impl Fn(Vec3) -> Vec3,
    onto_b: impl Fn(Vec3) -> Vec3,
) -> (Vec3, Vec3) {
    let mut b = onto_b(start);
    let mut a = onto_a(b);
    for _ in 0..6 {
        b = onto_b(a);
        a = onto_a(b);
    }
    (a, b)
}

/// Cylinder is A, swept shape (sphere or capsule) is B: the segment's ends and its
/// point nearest the cylinder against the cylinder, and the cylinder's side facing
/// the segment against the swept shape (a cylinder lying along a capsule).
fn swept_vs_cylinder(s0: Vec3, s1: Vec3, r: f32, c: &Cyl, margin: f32, out: &mut Vec<Contact>) {
    point_vs_cylinder(s0, r, c, margin, 0, out);
    if s0.distance_squared(s1) <= 1e-12 {
        return;
    }
    point_vs_cylinder(s1, r, c, margin, 1, out);
    let (on_cylinder, on_segment) = nearest_pair(
        c.center,
        |p| c.closest(p),
        |p| closest_on_segment(p, s0, s1),
    );
    point_vs_cylinder(on_segment, r, c, margin, 2, out);
    let toward = on_segment - on_cylinder;
    if toward.length_squared() > 1e-12
        && let Some((p0, p1)) = c.side_line(toward)
    {
        for (p, feature) in [(p0, 3), (p1, 4)] {
            let on = closest_on_segment(p, s0, s1);
            push_spheres(p, 0.0, on, r, margin, feature, out);
        }
    }
}

/// Box is A, cylinder is B (normals from the box). As between two boxes, the box face
/// that separates them most is the reference face; the cylinder's cap facing it (as a
/// polygon) and its side line facing it are clipped to that face, so a cylinder
/// standing or lying on a box rests on several points, and on the box's edge where it
/// hangs over. The box's corners touch the cylinder's caps and side (a foot on a
/// turntable), and the nearest points catch edges crossing.
fn cylinder_vs_box(
    c: &Cyl,
    box_rotation: Quat,
    box_center: Vec3,
    half: Vec3,
    margin: f32,
    out: &mut Vec<Contact>,
) {
    let rb = Mat3::from_quat(box_rotation);
    let d = c.center - box_center;
    let box_extent = |n: Vec3| {
        (0..3)
            .map(|k| half[k] * rb.col(k).dot(n).abs())
            .sum::<f32>()
    };
    // Reference face (separation, axis); the box's faces and the cylinder's axis are
    // separating axes.
    let mut face = (f32::MIN, 0);
    for k in 0..3 {
        let n = rb.col(k);
        let s = d.dot(n).abs() - half[k] - c.extent(n);
        if s > margin {
            return;
        }
        if s > face.0 {
            face = (s, k);
        }
    }
    let axis = c.axis();
    if d.dot(axis).abs() - c.half - box_extent(axis) > margin {
        return;
    }

    let mut found = Vec::new();
    for (i, corner) in corners(box_rotation, box_center, half)
        .into_iter()
        .enumerate()
    {
        point_vs_cylinder(corner, 0.0, c, margin, i as u8, &mut found);
    }

    let k = face.1;
    let n = rb.col(k) * d.dot(rb.col(k)).signum();
    let face_center = box_center + n * half[k];
    let (u, v) = ((k + 1) % 3, (k + 2) % 3);
    let push = |p: Vec3, feature: u8, out: &mut Vec<Contact>| {
        let s = (p - face_center).dot(n);
        if s < margin {
            out.push(Contact {
                point: p - n * (s * 0.5),
                normal: n,
                separation: s,
                feature,
            });
        }
    };

    // The cap facing the face, clipped to it.
    let mut polygon: Vec<(Vec3, u8)> = match c.facing_rim(-n) {
        Some((_, rim)) => (0u8..).zip(rim).map(|(i, p)| (p, i)).collect(),
        None => Vec::new(),
    };
    let sides = [
        (rb.col(u), half[u]),
        (-rb.col(u), half[u]),
        (rb.col(v), half[v]),
        (-rb.col(v), half[v]),
    ];
    for (plane, (dir, limit)) in (0u8..).zip(sides) {
        let outside = |p: Vec3| (p - face_center).dot(dir) - limit;
        let mut clipped = Vec::with_capacity(polygon.len() + 1);
        for (i, &(p, f)) in polygon.iter().enumerate() {
            let q = polygon[(i + 1) % polygon.len()].0;
            let (dp, dq) = (outside(p), outside(q));
            if dp <= 0.0 {
                clipped.push((p, f));
            }
            if (dp <= 0.0) != (dq <= 0.0) {
                clipped.push((
                    p + (q - p) * (dp / (dp - dq)),
                    RIM_POINTS as u8 + 2 * plane + u8::from(dp <= 0.0),
                ));
            }
        }
        polygon = clipped;
        if polygon.is_empty() {
            break;
        }
    }
    for (p, f) in polygon {
        // A corner of the face over the cap is the box's corner, touched below.
        let l = rb.transpose() * (p - box_center);
        let at_corner = |m: usize| (l[m].abs() - half[m]).abs() < 1e-4;
        if !(at_corner(u) && at_corner(v)) {
            push(p, 8 + f, out);
        }
    }

    // The side line facing the face, clipped to it (a cylinder lying on the box).
    let side_line = c.side_line(-n);
    if let Some((p0, p1)) = side_line {
        let to_local = |p: Vec3| rb.transpose() * (p - box_center);
        let (l0, l1) = (to_local(p0), to_local(p1));
        let (mut t0, mut t1) = (0.0f32, 1.0f32);
        for m in [u, v] {
            let (start, delta) = (l0[m], l1[m] - l0[m]);
            if delta.abs() < 1e-9 {
                if start.abs() > half[m] {
                    t1 = -1.0;
                }
                continue;
            }
            let (ta, tb) = ((-half[m] - start) / delta, (half[m] - start) / delta);
            t0 = t0.max(ta.min(tb));
            t1 = t1.min(ta.max(tb));
        }
        if t0 <= t1 {
            push(p0.lerp(p1, t0), 24 + RIM_POINTS as u8, out);
            push(p0.lerp(p1, t1), 25 + RIM_POINTS as u8, out);
        }
    }

    // Edges crossing: the box point nearest the cylinder's side line facing the face,
    // or, caps facing it, nearest the cylinder (its axis when they overlap, the
    // deepest). The nearest points of the whole cylinder are anywhere along a line
    // contact, or anywhere in both overlapping, and beside a lying cylinder's line of
    // support they roll it (see `CAP_FACING`).
    let onto_box =
        |p: Vec3| box_center + rb * (rb.transpose() * (p - box_center)).clamp(-half, half);
    let on_box = match side_line {
        Some((p0, p1)) => nearest_pair(c.center, |p| closest_on_segment(p, p0, p1), onto_box).1,
        None => {
            let (on_cylinder, on_box) = nearest_pair(c.center, |p| c.closest(p), onto_box);
            if on_cylinder.distance_squared(on_box) > 1e-12 {
                on_box
            } else {
                let (end0, end1) = (c.cap(-1.0), c.cap(1.0));
                nearest_pair(c.center, |p| closest_on_segment(p, end0, end1), onto_box).1
            }
        }
    };
    point_vs_cylinder(on_box, 0.0, c, margin, 26 + RIM_POINTS as u8, &mut found);
    out.extend(found.into_iter().map(|c| Contact {
        normal: -c.normal,
        ..c
    }));
}

/// Two cylinders (normals from A to B): each one's cap rims against the other (caps
/// stacked), each one's side line facing the other against it (logs side by side),
/// and the nearest points (crossed).
fn cylinder_vs_cylinder(a: &Cyl, b: &Cyl, margin: f32, out: &mut Vec<Contact>) {
    let d = b.center - a.center;
    let (on_a, on_b) = nearest_pair(a.center, |p| a.closest(p), |p| b.closest(p));
    // The direction they touch along: from the nearest points when apart, else the
    // candidate axis that separates them most.
    let separation = |n: Vec3| n.dot(d) - a.extent(n) - b.extent(n);
    let gap = on_b - on_a;
    let n = if gap.length_squared() > 1e-10 {
        gap.normalize()
    } else {
        let (aa, ab) = (a.axis(), b.axis());
        let across = |axis: Vec3, v: Vec3| (v - axis * v.dot(axis)).normalize_or_zero();
        [
            aa,
            ab,
            aa.cross(ab).normalize_or_zero(),
            across(aa, d),
            across(ab, d),
        ]
        .into_iter()
        .filter(|n| n.length_squared() > 0.5)
        .map(|n| if n.dot(d) < 0.0 { -n } else { n })
        .max_by(|x, y| separation(*x).total_cmp(&separation(*y)))
        .unwrap_or(Vec3::Y)
    };
    if separation(n) > margin {
        return;
    }
    let (b_line, a_line) = (b.side_line(-n), a.side_line(n));

    if let Some((first, rim)) = b.facing_rim(-n) {
        for (i, p) in (first..).zip(rim) {
            point_toward_cylinder(p, a, n, margin, i, out);
        }
    }
    let lines = 4 * RIM_POINTS as u8;
    if let Some((p0, p1)) = b_line {
        point_toward_cylinder(p0, a, n, margin, lines, out);
        point_toward_cylinder(p1, a, n, margin, lines + 1, out);
    }
    // Crossed, or a cap on a side: where each one's side line facing the other comes
    // nearest the other's axis, the deepest point. (The nearest points found by
    // alternating projections are anywhere along a line contact, or anywhere in both
    // overlapping, and beside a lying cylinder's line of support they roll it; see
    // `CAP_FACING`.) Caps facing each other: the nearest points.
    let deepest = |(p0, p1): (Vec3, Vec3), other: &Cyl| {
        closest_between_segments(p0, p1, other.cap(-1.0), other.cap(1.0)).0
    };
    match b_line {
        Some(line) => point_toward_cylinder(deepest(line, a), a, n, margin, lines + 4, out),
        None if a_line.is_none() => point_toward_cylinder(on_b, a, n, margin, lines + 4, out),
        None => {}
    }
    let mut found = Vec::new();
    if let Some((first, rim)) = a.facing_rim(n) {
        for (i, p) in (first..).zip(rim) {
            let feature = 2 * RIM_POINTS as u8 + i;
            point_toward_cylinder(p, b, -n, margin, feature, &mut found);
        }
    }
    if let Some((p0, p1)) = a_line {
        point_toward_cylinder(p0, b, -n, margin, lines + 2, &mut found);
        point_toward_cylinder(p1, b, -n, margin, lines + 3, &mut found);
        point_toward_cylinder(deepest((p0, p1), b), b, -n, margin, lines + 5, &mut found);
    }
    out.extend(found.into_iter().map(|c| Contact {
        normal: -c.normal,
        ..c
    }));
}

/// Oriented boxes A and B (rotation matrices, centres, half sizes): the separating axis
/// with the least overlap decides the normal. A face axis clips the other box's most
/// opposed face against the reference face (up to 8 points); an edge pair touches at
/// one point.
#[allow(clippy::too_many_arguments)]
fn box_vs_box(
    ra: Mat3,
    ca: Vec3,
    ha: Vec3,
    rb: Mat3,
    cb: Vec3,
    hb: Vec3,
    margin: f32,
    out: &mut Vec<Contact>,
) {
    let d = cb - ca;
    let project =
        |r: Mat3, h: Vec3, axis: Vec3| (0..3).map(|k| h[k] * r.col(k).dot(axis).abs()).sum::<f32>();
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
        out.push(Contact {
            point: (pa + pb) * 0.5,
            normal: n,
            separation: s,
            feature: 16 + (i * 3 + j) as u8,
        });
        return;
    }

    // Reference face on one box, incident face on the other.
    let (_, on_a, k) = face;
    let (rr, cr, hr, ri, ci, hi) = if on_a {
        (ra, ca, ha, rb, cb, hb)
    } else {
        (rb, cb, hb, ra, ca, ha)
    };
    let towards = if on_a { d } else { -d };
    let nr = rr.col(k) * towards.dot(rr.col(k)).signum();
    let face_center = cr + nr * hr[k];
    // Incident face: the other box's face most opposed to the reference normal.
    let m = (0..3)
        .max_by(|&x, &y| ri.col(x).dot(nr).abs().total_cmp(&ri.col(y).dot(nr).abs()))
        .unwrap();
    let (u, v) = ((m + 1) % 3, (m + 2) % 3);
    let incident_center = ci - ri.col(m) * hi[m] * ri.col(m).dot(nr).signum();
    let mut polygon: Vec<(Vec3, u8)> = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        .iter()
        .enumerate()
        .map(|(i, &(su, sv))| {
            (
                incident_center + ri.col(u) * hi[u] * su + ri.col(v) * hi[v] * sv,
                i as u8,
            )
        })
        .collect();
    // Clip against the reference face's four side planes.
    let (ru, rv) = ((k + 1) % 3, (k + 2) % 3);
    let sides = [
        (rr.col(ru), hr[ru]),
        (-rr.col(ru), hr[ru]),
        (rr.col(rv), hr[rv]),
        (-rr.col(rv), hr[rv]),
    ];
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
                clipped.push((
                    p + (q - p) * (dp / (dp - dq)),
                    4 + 2 * plane + u8::from(dp <= 0.0),
                ));
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
            out.push(Contact {
                point: p - nr * (s * 0.5),
                normal,
                separation: s,
                feature,
            });
        }
    }
}

fn push_spheres(
    pa: Vec3,
    ra: f32,
    pb: Vec3,
    rb: f32,
    margin: f32,
    feature: u8,
    out: &mut Vec<Contact>,
) {
    let delta = pb - pa;
    let distance = delta.length();
    let separation = distance - ra - rb;
    if separation >= margin {
        return;
    }
    let normal = if distance > 1e-6 {
        delta / distance
    } else {
        Vec3::Y
    };
    out.push(Contact {
        point: pa + normal * (ra + separation * 0.5),
        normal,
        separation,
        feature,
    });
}

/// Box is A, swept shape is B. Normals point from the box towards the swept shape.
#[allow(clippy::too_many_arguments)]
fn swept_vs_box(
    s0: Vec3,
    s1: Vec3,
    radius: f32,
    rotation: Quat,
    center: Vec3,
    half: Vec3,
    margin: f32,
    out: &mut Vec<Contact>,
) {
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
    if len2 < 1e-12 {
        0.0
    } else {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    }
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
            let mut s = if denom > 1e-12 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
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
        Placed {
            geometry: Geometry::Sphere { radius: r },
            rotation: Quat::IDENTITY,
            position: at,
        }
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
            geometry: Geometry::Capsule {
                radius: 0.05,
                length: 0.4,
            },
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
        between(
            &sphere(0.1, Vec3::ZERO),
            &sphere(0.1, Vec3::new(0.15, 0.0, 0.0)),
            0.0,
            &mut out,
        );
        assert_eq!(out.len(), 1);
        assert!((out[0].separation + 0.05).abs() < 1e-5);
        assert!((out[0].normal - Vec3::X).length() < 1e-5);
    }

    #[test]
    fn sphere_resting_on_box_points_up() {
        let table = Placed {
            geometry: Geometry::Box {
                size: [1.0, 0.2, 1.0],
            },
            rotation: Quat::IDENTITY,
            position: Vec3::ZERO,
        };
        let mut out = Vec::new();
        between(
            &table,
            &sphere(0.1, Vec3::new(0.2, 0.19, 0.0)),
            0.02,
            &mut out,
        );
        assert_eq!(out.len(), 1);
        assert!((out[0].normal - Vec3::Y).length() < 1e-5);
        assert!((out[0].separation + 0.01).abs() < 1e-5);
    }

    fn cuboid(size: [f32; 3], rotation: Quat, at: Vec3) -> Placed {
        Placed {
            geometry: Geometry::Box { size },
            rotation,
            position: at,
        }
    }

    #[test]
    fn box_resting_on_box_touches_at_its_four_corners() {
        let floor = cuboid([4.0, 0.2, 4.0], Quat::IDENTITY, Vec3::new(0.0, -0.1, 0.0));
        let foot = cuboid(
            [0.1, 0.06, 0.24],
            Quat::from_rotation_y(0.4),
            Vec3::new(0.3, 0.025, 0.2),
        );
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
        let a = cuboid(
            [2.0, 0.1, 0.1],
            Quat::from_rotation_x(std::f32::consts::FRAC_PI_4),
            Vec3::ZERO,
        );
        let b = cuboid(
            [0.1, 0.1, 2.0],
            Quat::from_rotation_z(std::f32::consts::FRAC_PI_4),
            Vec3::new(0.0, 0.14, 0.0),
        );
        let mut out = Vec::new();
        between(&a, &b, 0.02, &mut out);
        assert_eq!(out.len(), 1, "{out:?}");
        let gap = 0.14 - 0.1 * std::f32::consts::SQRT_2;
        assert!(
            (out[0].separation - gap).abs() < 1e-4,
            "{}",
            out[0].separation
        );
        assert!((out[0].normal - Vec3::Y).length() < 1e-4);
    }

    #[test]
    fn separated_boxes_do_not_touch() {
        let a = cuboid([1.0, 1.0, 1.0], Quat::IDENTITY, Vec3::ZERO);
        let b = cuboid(
            [1.0, 1.0, 1.0],
            Quat::from_rotation_z(0.3),
            Vec3::new(1.8, 0.0, 0.0),
        );
        let mut out = Vec::new();
        between(&a, &b, 0.02, &mut out);
        assert!(out.is_empty());
    }

    /// A flat cylinder is a disc, not a sphere of its radius.
    #[test]
    fn flat_cylinder_is_a_disc() {
        let disc = Placed {
            geometry: Geometry::Cylinder {
                radius: 2.0,
                length: 0.2,
            },
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            position: Vec3::new(0.0, -0.1, 0.0),
        };
        // A ball resting on top, well away from the centre.
        let mut out = Vec::new();
        between(
            &disc,
            &sphere(0.1, Vec3::new(1.5, 0.09, 0.0)),
            0.02,
            &mut out,
        );
        assert_eq!(out.len(), 1);
        assert!((out[0].normal - Vec3::Y).length() < 1e-5);
        assert!((out[0].separation + 0.01).abs() < 1e-5);
        // Nothing just past the rim.
        out.clear();
        between(
            &disc,
            &sphere(0.1, Vec3::new(2.15, 0.09, 0.0)),
            0.02,
            &mut out,
        );
        assert!(out.is_empty());
        // A box foot on it touches at its four bottom corners (and the disc point nearest
        // it); the disc lying on the ground touches around its rim.
        out.clear();
        between(
            &disc,
            &cuboid(
                [0.1, 0.06, 0.24],
                Quat::IDENTITY,
                Vec3::new(0.5, 0.025, 0.0),
            ),
            0.02,
            &mut out,
        );
        assert_eq!(out.len(), 5);
        assert!(out.iter().all(|c| (c.normal - Vec3::Y).length() < 1e-4));
        out.clear();
        with_ground(
            &Placed {
                position: Vec3::new(0.0, 0.1, 0.0),
                ..disc
            },
            0.0,
            0.01,
            &mut out,
        );
        assert_eq!(out.len(), RIM_POINTS);
    }

    /// An upright disc (a wheel) rests on the lowest point of its rim, between samples,
    /// and a disc lying across a long beam away from its centre still lands on it.
    #[test]
    fn discs_touch_between_rim_samples() {
        let wheel = Placed {
            geometry: Geometry::Cylinder {
                radius: 0.5,
                length: 0.1,
            },
            rotation: Quat::from_rotation_y(0.3),
            position: Vec3::new(0.0, 0.5, 0.0),
        };
        let mut out = Vec::new();
        with_ground(&wheel, 0.0, 0.01, &mut out);
        assert!(out.iter().any(|c| c.separation.abs() < 1e-4), "{out:?}");

        let disc = Placed {
            geometry: Geometry::Cylinder {
                radius: 2.0,
                length: 0.2,
            },
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            position: Vec3::new(0.0, 0.09, 0.0),
        };
        let beam = cuboid([10.0, 0.2, 0.2], Quat::IDENTITY, Vec3::new(0.0, -0.1, 1.2));
        out.clear();
        between(&beam, &disc, 0.02, &mut out);
        assert!(
            out.iter()
                .any(|c| (c.normal - Vec3::Y).length() < 1e-4 && c.separation < 0.0),
            "{out:?}"
        );
    }

    fn cylinder(radius: f32, length: f32, rotation: Quat, at: Vec3) -> Placed {
        Placed {
            geometry: Geometry::Cylinder { radius, length },
            rotation,
            position: at,
        }
    }

    /// A long cylinder has a flat cap and a sharp rim, not a capsule's round end.
    #[test]
    fn cylinder_has_flat_caps_and_a_sharp_rim() {
        let up = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        let pillar = cylinder(0.2, 1.0, up, Vec3::new(0.0, 0.5, 0.0));
        let touch = |at: Vec3| {
            let mut out = Vec::new();
            between(&pillar, &sphere(0.05, at), 0.02, &mut out);
            assert_eq!(out.len(), 1, "{out:?}");
            out[0]
        };
        // On the cap 3 cm in from the rim: straight up.
        let c = touch(Vec3::new(0.17, 1.04, 0.0));
        assert!((c.normal - Vec3::Y).length() < 1e-5 && (c.separation + 0.01).abs() < 1e-5);
        // Over the rim's edge: out from the edge.
        let c = touch(Vec3::new(0.22, 1.02, 0.0));
        let gap = Vec3::new(0.02, 0.02, 0.0);
        assert!((c.normal - gap.normalize()).length() < 1e-4, "{c:?}");
        assert!((c.separation - (gap.length() - 0.05)).abs() < 1e-5, "{c:?}");
        // Beside the side.
        let c = touch(Vec3::new(0.24, 0.5, 0.0));
        assert!((c.normal - Vec3::X).length() < 1e-5 && (c.separation + 0.01).abs() < 1e-5);

        // Standing on the ground it rests on its rim; lying, on its side's two ends.
        let mut out = Vec::new();
        with_ground(&pillar, 0.0, 0.01, &mut out);
        assert_eq!(out.len(), RIM_POINTS);
        assert!(out.iter().all(|c| c.separation.abs() < 1e-5));
        let lying = cylinder(
            0.2,
            1.0,
            Quat::from_rotation_y(0.3),
            Vec3::new(0.0, 0.2, 0.0),
        );
        out.clear();
        with_ground(&lying, 0.0, 0.01, &mut out);
        assert_eq!(out.len(), 2, "{out:?}");
        assert!(out.iter().all(|c| c.separation.abs() < 1e-5));
    }

    /// Discs as wide stacked push each other up, not sideways off their lined-up
    /// rims; logs as long side by side push apart along the line between them.
    #[test]
    fn cylinders_touch_along_the_way_they_face() {
        let up = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        let below = cylinder(0.3, 0.06, up, Vec3::new(0.0, 0.03, 0.0));
        for tilt in [0.0, 0.001] {
            let above = cylinder(0.3, 0.06, Quat::from_rotation_z(tilt) * up, Vec3::Y * 0.089);
            let mut out = Vec::new();
            between(&below, &above, 0.02, &mut out);
            assert!(out.len() >= RIM_POINTS, "{out:?}");
            assert!(out.iter().all(|c| c.normal.y > 0.999), "{out:?}");
        }
        let along = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        let left = cylinder(0.1, 0.8, along, Vec3::new(0.0, 0.1, -0.1));
        let right = cylinder(0.1, 0.8, along, Vec3::new(0.0, 0.1, 0.099));
        let mut out = Vec::new();
        between(&left, &right, 0.02, &mut out);
        assert!(out.len() >= 2, "{out:?}");
        assert!(
            out.iter().all(
                |c| (c.normal - Vec3::Z).length() < 1e-4 && (c.separation + 0.001).abs() < 1e-5
            ),
            "{out:?}"
        );
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
