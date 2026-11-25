//! Characters solved whole: every joint's anchor, muscle and stops of a jointed group
//! as one linear system, each iteration (docs/physics-design.md, lesson 38).
//!
//! Sequential impulses solve a muscle with only its joint's two bodies, so a stiff
//! muscle on a light body (a finger, a clavicle, a 4.5 kg pelvis between two 9 kg
//! thighs) overshoots and tears its joint open, and muscles had to be capped far below
//! what endorphin's presets ask for. Solved together, a muscle is an implicit spring
//! on the whole articulated body: the anchors hold exactly, each muscle feels every
//! body it moves, and a stiff spring stays stable.
//!
//! The system is the bodies' velocity changes and the rows' impulses together,
//! `[M Jᵀ; J −Γ] [Δv; −Δλ] = [0; r]`, with Γ the springs' softness. Bodies and joints
//! form a tree, so eliminated leaf first it factors without fill-in, in blocks of 6
//! (a body) or a joint's few rows: linear time in the number of bodies (Baraff,
//! "Linear-time dynamics using Lagrange multipliers", 1996).
//!
//! Force limits and one-sided stops are bounds on a row's accumulated impulse. Which
//! rows are at a bound is guessed when the substep starts, from the last one: those
//! rows stay out of the factor, at their bound. A row whose guess turns out wrong
//! during a pass is held at its bound, or let back in, through the factor's own
//! response to it (a small dense system the size of the rows that changed).

use glam::Vec3;

use super::{ContactRow, JointRows, MAX_PUSH_SPEED, STATIC, SolverBody};

/// Most rounds of switching rows onto and off their bounds, per pass. A pass that runs
/// out ends with its rows clamped, not solved, and the next pass starts from that
/// guess: at 8, a kick in Multi Martial left a hand's muscles flipping between their
/// limits from pass to pass, and the hand spun at 1,870 rad/s for a substep (lesson
/// 45). Most passes settle in 0-3 rounds; over their whole ranges the 17 sample scenes
/// have 323 passes that need more than 8 and 13 that need more than 24.
const PIVOTS: usize = 24;
/// More rounds per contact row in the system: a character landing on a floor puts
/// dozens of contact rows onto or off their bounds at once, and with 24 rounds a hand
/// landing in Catch Rope ran out with its finger contacts at no impulse while they
/// closed at 7 m/s, and the finger joint opened 20 cm (lesson 50).
const PIVOTS_PER_CONTACT_ROW: usize = 2;
/// Rows off the factor's guess beyond which it is factored again.
const REFACTOR: usize = 6;
/// Further rounds that only put rows past their bounds onto them.
const SETTLE: usize = 4;
const NONE: usize = usize::MAX;
/// Softness of a contact row solved with its character, as a share of its body's own
/// inverse mass along it. A foot's corners on a floor repeat each other (four normals
/// on a body its ankle leaves only three ways to move), so rigid rows couldn't be
/// solved; this little give (0.2 µm a substep under a planted foot's load) settles how
/// they share the load.
const CONTACT_SOFTNESS: f32 = 1e-4;
/// Most rows a joint's block may have: a body's contacts beyond it are left to the
/// contact pass.
const MAX_BLOCK: usize = 48;

#[derive(Clone, Copy, PartialEq)]
enum Source {
    /// A joint's anchor along a world axis.
    Anchor(usize),
    /// A muscle's drive axis (twist, swing 1, swing 2).
    Muscle(usize),
    /// One of the joint's limit rows (a stop or a soft-zone spring).
    Limit(usize),
    /// A contact against the ground or fixed scenery: its normal (0) or a tangent (1, 2).
    Contact(usize, usize),
}

struct Row {
    /// Index into the substep's joint rows.
    joint: usize,
    source: Source,
    a: usize,
    b: usize,
    /// Jacobian: linear and angular parts on A, then on B.
    jacobian: [Vec3; 4],
    /// Spring softness (muscles and soft-zone springs), 0 for rigid rows.
    gamma: f32,
    /// Bounds on the accumulated impulse.
    lower: f32,
    upper: f32,
    /// Velocity bias while correcting positions, and while relaxing.
    bias: [f32; 2],
    /// Where the row sits in the solution vector, if it is in the factor.
    slot: Option<usize>,
    /// Where each of its moving bodies' velocity changes sit in the solution vector, and
    /// its Jacobian there.
    parts: [(usize, [f64; 6]); 2],
    /// A Coulomb friction row's normal row and friction coefficient: its bounds are ±μ
    /// times the normal impulse.
    friction: Option<(usize, f32)>,
}

impl Row {
    /// The row's Jacobian on `body` (linear, then angular), if it acts on it.
    fn on(&self, body: usize) -> Option<[Vec3; 2]> {
        if body == self.a {
            Some([self.jacobian[0], self.jacobian[1]])
        } else if body == self.b {
            Some([self.jacobian[2], self.jacobian[3]])
        } else {
            None
        }
    }

    /// Whether an impulse of `total` sits at the row's upper bound rather than its lower.
    fn near_upper(&self, total: f64) -> bool {
        match (self.lower.is_finite(), self.upper.is_finite()) {
            (true, true) => total > 0.5 * (self.lower as f64 + self.upper as f64),
            (lower, _) => !lower,
        }
    }
}

/// A body or a joint in the elimination tree.
struct Node {
    /// The body, for a body's node.
    body: Option<usize>,
    /// Size of the block: 6 for a body, the rows in the factor for a joint.
    n: usize,
    /// Offset in the solution vector.
    at: usize,
    parent: usize,
    /// D⁻¹ (n × n) and W = D⁻¹ H(this, parent) (n × parent's n), row-major.
    dinv: Vec<f64>,
    w: Vec<f64>,
    /// Wᵀ, for the forward pass.
    wt: Vec<f64>,
}

/// One jointed group's rows and their factor.
pub(super) struct Articulation {
    rows: Vec<Row>,
    /// Root first, every node after its parent.
    nodes: Vec<Node>,
    /// (body, node) of each moving body of the group, by body.
    body_node: Vec<(usize, usize)>,
    size: usize,
    /// Responses to a unit impulse on a row (in the factor: on its own equation; out of
    /// it: on its bodies), cached for the substep.
    columns: Vec<Option<Vec<f64>>>,
    /// Per row: the accumulated impulse it is held at, if it sits at a bound.
    at: Vec<Option<f64>>,
    /// The tree, root first: (a joint?, body, parent node), and each joint node's rows.
    order: Vec<(bool, usize, usize)>,
    row_range: Vec<(usize, usize)>,
}

impl Articulation {
    /// Builds the articulations of every jointed group among `joints`, and marks the
    /// joints they solve. A joint that would close a loop, or a second joint holding a
    /// group to something that doesn't move, is left to sequential impulses. With
    /// `ground`, a group's contacts against the ground and fixed scenery (bodies `fixed`
    /// says don't move) are solved with it too, rigid (lesson 50), and marked in
    /// `articulated`.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn build(
        sb: &[SolverBody],
        joints: &mut [JointRows],
        bias_rate: f32,
        h: f32,
        ground: Option<&GroundContacts>,
        contacts: &[ContactRow],
        articulated: &mut [bool],
    ) -> Vec<Articulation> {
        let moves = |i: usize| i != STATIC && sb[i].inv_mass > 0.0;
        // Jointed groups: union-find over joints whose bodies both move.
        let mut root: Vec<usize> = (0..sb.len()).collect();
        fn find(root: &mut [usize], mut i: usize) -> usize {
            while root[i] != i {
                root[i] = root[root[i]];
                i = root[i];
            }
            i
        }
        for j in joints.iter() {
            if moves(j.a) && moves(j.b) {
                let (ra, rb) = (find(&mut root, j.a), find(&mut root, j.b));
                root[ra] = rb;
            }
        }
        let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
        for (ji, j) in joints.iter().enumerate() {
            let body = if moves(j.b) { j.b } else { j.a };
            if !moves(body) {
                continue;
            }
            let g = find(&mut root, body);
            match groups.iter_mut().find(|(r, _)| *r == g) {
                Some((_, list)) => list.push(ji),
                None => groups.push((g, vec![ji])),
            }
        }
        // Only groups with a muscle: ropes and props made of jointed links have nothing
        // stiff to hold and stay with sequential impulses.
        groups.retain(|(_, list)| list.iter().any(|&j| joints[j].muscle.is_some()));
        let articulations: Vec<Articulation> = groups
            .into_iter()
            .filter_map(|(_, list)| {
                Articulation::new(sb, joints, &list, bias_rate, h, ground, contacts)
            })
            .collect();
        for a in &articulations {
            for r in &a.rows {
                if let Source::Contact(c, _) = r.source {
                    articulated[c] = true;
                }
            }
        }
        articulations
    }

    #[allow(clippy::too_many_arguments)]
    fn new(
        sb: &[SolverBody],
        joints: &mut [JointRows],
        list: &[usize],
        bias_rate: f32,
        h: f32,
        ground: Option<&GroundContacts>,
        contacts: &[ContactRow],
    ) -> Option<Articulation> {
        let moves = |i: usize| i != STATIC && sb[i].inv_mass > 0.0;
        let mut bodies: Vec<usize> = list
            .iter()
            .flat_map(|&j| [joints[j].a, joints[j].b])
            .filter(|&b| moves(b))
            .collect();
        bodies.sort_unstable();
        bodies.dedup();
        let body_index = |b: usize| bodies.binary_search(&b).ok();
        // The tree, rooted at the first joint holding the group to something that
        // doesn't move if there is one (a leaf there would be a rigid row on its own),
        // else at the first joint's moving body.
        let fixed = list
            .iter()
            .copied()
            .find(|&j| !moves(joints[j].a) || !moves(joints[j].b));
        let mut adjacent: Vec<Vec<usize>> = vec![Vec::new(); bodies.len()];
        for &j in list {
            if let (Some(a), Some(b)) = (body_index(joints[j].a), body_index(joints[j].b)) {
                adjacent[a].push(j);
                adjacent[b].push(j);
            }
        }
        // Contacts against the ground and fixed scenery, by body: each body's go into the
        // block of the joint above it (lesson 50).
        let mut touching: Vec<Vec<usize>> = vec![Vec::new(); bodies.len()];
        if let Some(ground) = ground {
            for (ci, c) in contacts.iter().enumerate() {
                let (body, other) = match (body_index(c.a), body_index(c.b)) {
                    (Some(a), None) => (a, c.b),
                    (None, Some(b)) => (b, c.a),
                    _ => continue,
                };
                // Only contacts that carry a load: they pushed at the end of the last
                // substep and still touch. A new one (a hand or foot landing) goes to the
                // contact pass first, as before: taken rigid at first touch, a landing
                // finger or a foot wedged against a ledge got impulses of hundreds of
                // N·s, and limbs came apart (lesson 50).
                let carrying = c.impulse[0] > 0.0 && c.separation < ground.touching;
                if carrying && (other == STATIC || ground.fixed[other]) {
                    touching[body].push(ci);
                }
            }
        }
        // Breadth first: (joint, or else body; parent node). A joint of `NONE` holds only
        // the root body's contacts.
        let mut order: Vec<(Option<usize>, usize, usize)> = Vec::new();
        let mut seen = vec![false; bodies.len()];
        let mut used = vec![false; joints.len()];
        let start = match fixed {
            Some(j) => {
                used[j] = true;
                order.push((Some(j), NONE, NONE));
                let b = if moves(joints[j].a) {
                    joints[j].a
                } else {
                    joints[j].b
                };
                order.push((None, b, 0));
                b
            }
            None => {
                let j = &joints[list[0]];
                let b = if moves(j.a) { j.a } else { j.b };
                if touching[body_index(b)?].is_empty() {
                    order.push((None, b, NONE));
                } else {
                    order.push((Some(NONE), NONE, NONE));
                    order.push((None, b, 0));
                }
                b
            }
        };
        seen[body_index(start)?] = true;
        let mut next = 0;
        while next < order.len() {
            let (joint, b, _) = order[next];
            if joint.is_none() {
                for &j in &adjacent[body_index(b)?] {
                    let other = if joints[j].a == b {
                        joints[j].b
                    } else {
                        joints[j].a
                    };
                    let oi = body_index(other)?;
                    // A joint closing a loop stays with sequential impulses.
                    if used[j] || seen[oi] {
                        continue;
                    }
                    used[j] = true;
                    seen[oi] = true;
                    order.push((Some(j), NONE, next));
                    order.push((None, other, order.len() - 1));
                }
            }
            next += 1;
        }

        // Rows of the joints in the tree, each with its child body's contacts; those out
        // of the factor start at the bound they sit at.
        let mut rows = Vec::new();
        let mut row_range = vec![(0, 0); order.len()];
        for (k, &(joint, _, _)) in order.iter().enumerate() {
            if let Some(ji) = joint {
                let start = rows.len();
                if ji != NONE {
                    push_rows(&mut rows, sb, &joints[ji], ji, bias_rate, h);
                    joints[ji].direct = true;
                }
                // The joint's child body comes right after it.
                if let (Some(ground), Some(child)) =
                    (ground, order.get(k + 1).and_then(|o| body_index(o.1)))
                {
                    for &ci in &touching[child] {
                        if rows.len() - start + 3 > MAX_BLOCK {
                            break;
                        }
                        push_contact_rows(&mut rows, sb, &contacts[ci], ci, ground, h);
                    }
                }
                row_range[k] = (start, rows.len());
            }
        }
        let at: Vec<Option<f64>> = rows
            .iter()
            .map(|r| {
                r.slot.is_none().then(|| {
                    let lambda = Self::accumulated(r, joints, contacts) as f64;
                    if r.near_upper(lambda) {
                        r.upper as f64
                    } else {
                        r.lower as f64
                    }
                })
            })
            .collect();
        let mut articulation = Articulation {
            at,
            columns: Vec::new(),
            rows,
            order: order.iter().map(|&(j, b, p)| (j.is_some(), b, p)).collect(),
            row_range,
            nodes: Vec::new(),
            body_node: Vec::new(),
            size: 0,
        };
        articulation.factor(sb)?;
        Some(articulation)
    }

    /// Factors the system with the rows not at a bound.
    fn factor(&mut self, sb: &[SolverBody]) -> Option<()> {
        let mut nodes: Vec<Node> = Vec::with_capacity(self.order.len());
        let mut body_node = Vec::new();
        let mut size = 0;
        for (k, &(joint, body, parent)) in self.order.iter().enumerate() {
            let n = if joint {
                let (s, e) = self.row_range[k];
                let mut n = 0;
                for (r, at) in self.rows[s..e].iter_mut().zip(&self.at[s..e]) {
                    r.slot = at.is_none().then_some(size + n);
                    n += usize::from(at.is_none());
                }
                n
            } else {
                body_node.push((body, k));
                6
            };
            nodes.push(Node {
                body: (!joint).then_some(body),
                n,
                at: size,
                parent,
                dinv: Vec::new(),
                w: Vec::new(),
                wt: Vec::new(),
            });
            size += n;
        }
        body_node.sort_unstable();
        let rows = &self.rows;
        let row_range = &self.row_range;

        // H(joint's rows in the factor, body), n × 6.
        let block = |range: (usize, usize), body: usize| -> Vec<f64> {
            let mut m = Vec::new();
            for r in rows[range.0..range.1].iter().filter(|r| r.slot.is_some()) {
                let [lin, ang] = r.on(body).unwrap_or([Vec3::ZERO; 2]);
                m.extend(
                    lin.to_array()
                        .into_iter()
                        .chain(ang.to_array())
                        .map(f64::from),
                );
            }
            m
        };
        let mut d: Vec<Vec<f64>> = nodes
            .iter()
            .enumerate()
            .map(|(k, node)| match node.body {
                Some(b) => {
                    let s = sb[b];
                    let inertia = s.inv_i.inverse();
                    let mut m = vec![0.0f64; 36];
                    for i in 0..3 {
                        m[i * 6 + i] = 1.0 / s.inv_mass as f64;
                        for j in 0..3 {
                            m[(3 + i) * 6 + 3 + j] = inertia.col(j)[i] as f64;
                        }
                    }
                    m
                }
                None => {
                    let (s, e) = row_range[k];
                    let n = node.n;
                    let mut m = vec![0.0f64; n * n];
                    for (i, r) in rows[s..e].iter().filter(|r| r.slot.is_some()).enumerate() {
                        m[i * n + i] = -(r.gamma as f64);
                    }
                    m
                }
            })
            .collect();
        // Leaf first: D = H(node, node) − Σ over children H(node, child) D⁻¹ H(child, node).
        for k in (0..nodes.len()).rev() {
            let n = nodes[k].n;
            // A body's block is positive definite, a joint's negative definite.
            let dinv = if n == 0 {
                Vec::new()
            } else {
                invert_definite(&d[k], n, nodes[k].body.is_none())?
            };
            let p = nodes[k].parent;
            if p != NONE {
                let np = nodes[p].n;
                // H(node, parent), n × np.
                let up = match nodes[k].body {
                    Some(b) => transpose(&block(row_range[p], b), np, 6),
                    None => block(row_range[k], nodes[p].body.unwrap_or(STATIC)),
                };
                let w = multiply(&dinv, &up, n, n, np);
                for i in 0..np {
                    for j in 0..=i {
                        let mut s = 0.0;
                        for t in 0..n {
                            s += up[t * np + i] * w[t * np + j];
                        }
                        d[p][i * np + j] -= s;
                        if j != i {
                            d[p][j * np + i] -= s;
                        }
                    }
                }
                nodes[k].wt = transpose(&w, n, np);
                nodes[k].w = w;
            }
            nodes[k].dinv = dinv;
        }
        for r in &mut self.rows {
            for (k, body) in [r.a, r.b].into_iter().enumerate() {
                let node = body_node
                    .binary_search_by_key(&body, |&(b, _)| b)
                    .ok()
                    .map(|i| body_node[i].1);
                r.parts[k] = match (node, r.on(body)) {
                    (Some(node), Some([lin, ang])) => {
                        let mut j = [0.0; 6];
                        for (j, v) in j
                            .iter_mut()
                            .zip(lin.to_array().into_iter().chain(ang.to_array()))
                        {
                            *j = v as f64;
                        }
                        (nodes[node].at, j)
                    }
                    _ => (NONE, [0.0; 6]),
                };
            }
        }
        self.columns = (0..self.rows.len()).map(|_| None).collect();
        self.nodes = nodes;
        self.body_node = body_node;
        self.size = size;
        Some(())
    }

    /// The solution with every row in the factor free and every other at its bound.
    fn free(&self, wanted: &[f64]) -> Vec<f64> {
        let mut free = vec![0.0f64; self.size];
        for (r, &w) in self.rows.iter().zip(wanted) {
            if let Some(slot) = r.slot {
                free[slot] = w;
            }
        }
        self.solve_system(&mut free);
        free
    }

    /// Solves the factored system for right-hand side `x`, in place.
    fn solve_system(&self, x: &mut [f64]) {
        let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>();
        let mut z = [0.0f64; MAX_BLOCK];
        // Forward: each node's part, less what its children pass up.
        for node in self.nodes.iter().rev() {
            if node.parent == NONE || node.n == 0 {
                continue;
            }
            let p = &self.nodes[node.parent];
            z[..node.n].copy_from_slice(&x[node.at..node.at + node.n]);
            for (x, wt) in x[p.at..p.at + p.n]
                .iter_mut()
                .zip(node.wt.chunks_exact(node.n))
            {
                *x -= dot(wt, &z[..node.n]);
            }
        }
        for node in self.nodes.iter().filter(|n| n.n > 0) {
            let n = node.n;
            z[..n].copy_from_slice(&x[node.at..node.at + n]);
            for (x, d) in x[node.at..node.at + n]
                .iter_mut()
                .zip(node.dinv.chunks_exact(n))
            {
                *x = dot(d, &z[..n]);
            }
        }
        // Backward: each node's part, less what its parent's solution takes.
        for node in &self.nodes {
            if node.parent == NONE || node.n == 0 {
                continue;
            }
            let p = &self.nodes[node.parent];
            if p.n == 0 {
                continue;
            }
            z[..p.n].copy_from_slice(&x[p.at..p.at + p.n]);
            for (x, w) in x[node.at..node.at + node.n]
                .iter_mut()
                .zip(node.w.chunks_exact(p.n))
            {
                *x -= dot(w, &z[..p.n]);
            }
        }
    }

    /// The row's velocity change in solution `y`: its Jacobian times its bodies' Δv.
    fn velocity_in(&self, r: &Row, y: &[f64]) -> f64 {
        let mut v = 0.0;
        for (at, j) in r.parts.iter().filter(|p| p.0 != NONE) {
            for (j, y) in j.iter().zip(&y[*at..*at + 6]) {
                v += j * y;
            }
        }
        v
    }

    /// The solution for a unit impulse on row `i`: on its own equation if it is in the
    /// factor, on its bodies if it isn't. Cached for the substep.
    fn column(&mut self, i: usize) -> &[f64] {
        if self.columns[i].is_none() {
            let mut e = vec![0.0; self.size];
            let r = &self.rows[i];
            match r.slot {
                Some(slot) => e[slot] = 1.0,
                None => {
                    for (at, j) in r.parts.iter().filter(|p| p.0 != NONE) {
                        e[*at..*at + 6].copy_from_slice(j);
                    }
                }
            }
            self.solve_system(&mut e);
            self.columns[i] = Some(e);
        }
        self.columns[i].as_deref().unwrap_or(&[])
    }

    /// The row's accumulated impulse so far.
    fn accumulated(r: &Row, joints: &[JointRows], contacts: &[ContactRow]) -> f32 {
        match r.source {
            Source::Anchor(axis) => joints[r.joint].point_impulse[axis],
            Source::Muscle(k) => joints[r.joint]
                .muscle
                .as_ref()
                .map_or(0.0, |m| m.frame.col(k).dot(m.impulse)),
            Source::Limit(l) => joints[r.joint].limits[l].impulse,
            Source::Contact(c, k) => contacts[c].impulse[k],
        }
    }

    /// One pass: the impulses that satisfy every row at once, each within its bounds.
    pub(super) fn solve(
        &mut self,
        sb: &mut [SolverBody],
        joints: &mut [JointRows],
        contacts: &mut [ContactRow],
        use_bias: bool,
    ) {
        let pass = if use_bias { 0 } else { 1 };
        let so_far: Vec<f64> = self
            .rows
            .iter()
            .map(|r| Self::accumulated(r, joints, contacts) as f64)
            .collect();
        // Coulomb friction: ±μ times the normal impulse so far. A row that was sliding
        // stays at the new bound, on the same side.
        for i in 0..self.rows.len() {
            let Some((normal, mu)) = self.rows[i].friction else {
                continue;
            };
            let limit = mu * (so_far[normal] as f32).max(0.0);
            let r = &mut self.rows[i];
            r.lower = -limit;
            r.upper = limit;
            self.at[i] = match self.at[i] {
                _ if limit == 0.0 => Some(0.0),
                Some(at) if at > 0.0 => Some(limit as f64),
                Some(at) if at < 0.0 => Some(-limit as f64),
                _ => None,
            };
        }
        // What each row's equation asks for: −(J v + bias + γ λ).
        let wanted: Vec<f64> = self
            .rows
            .iter()
            .zip(&so_far)
            .map(|(r, &lambda)| {
                let v: f32 = [r.a, r.b]
                    .into_iter()
                    .filter(|&b| b != STATIC)
                    .map(|b| {
                        let [lin, ang] = r.on(b).unwrap_or([Vec3::ZERO; 2]);
                        lin.dot(sb[b].v) + ang.dot(sb[b].w)
                    })
                    .sum();
                -((v + r.bias[pass]) as f64 + r.gamma as f64 * lambda)
            })
            .collect();
        let mut free = self.free(&wanted);
        let mut refactored = 0;

        // Which bounded rows sit at a bound is the last pass's answer (at first, the
        // guess the factor was built on). Each round solves with it, then switches every
        // row that is wrong: a free row past a bound goes to it, a row at a bound whose
        // equation would take it off goes free (block principal pivoting, Júdice and
        // Pires 1994). If that doesn't make fewer rows wrong, only the last one switches,
        // which always ends (Murty 1974).
        let mut changed: Vec<(usize, Option<f64>)> = Vec::new();
        let mut u: Vec<f64> = Vec::new();
        let mut y = free.clone();
        let mut fewest = usize::MAX;
        let contact_rows = self
            .rows
            .iter()
            .filter(|r| matches!(r.source, Source::Contact(..)))
            .count();
        let pivots = PIVOTS + PIVOTS_PER_CONTACT_ROW * contact_rows;
        for round in 0..=pivots + SETTLE {
            // Rows held at a bound in the factor, let in from out of it, or put on a
            // bound from off it.
            changed.clear();
            for (i, r) in self.rows.iter().enumerate() {
                match (r.slot, self.at[i]) {
                    (Some(_), Some(at)) => changed.push((i, Some(at))),
                    (None, None) => changed.push((i, None)),
                    (None, Some(at)) if (at - so_far[i]).abs() > 1e-6 * (1.0 + at.abs()) => {
                        changed.push((i, Some(at)))
                    }
                    _ => {}
                }
            }
            u = self.adjust(&changed, &free, &so_far, &wanted);
            y.clone_from(&free);
            for (&(c, _), &uk) in changed.iter().zip(&u) {
                for (y, col) in y.iter_mut().zip(self.column(c)) {
                    *y += col * uk;
                }
            }
            // The last round only solves with what the rounds before it settled on.
            if round == pivots + SETTLE {
                break;
            }
            let mut wrong: Vec<(usize, Option<f64>)> = Vec::new();
            for (i, r) in self.rows.iter().enumerate() {
                // Unbounded rows are never wrong; a row with no room (friction with no
                // load) stays at its one value.
                if (r.lower == f32::NEG_INFINITY && r.upper == f32::INFINITY) || r.lower == r.upper
                {
                    continue;
                }
                let d = match r.slot {
                    Some(slot) => -y[slot],
                    None => changed.iter().position(|c| c.0 == i).map_or(0.0, |k| u[k]),
                };
                let total = so_far[i] + d;
                match self.at[i] {
                    None if total > r.upper as f64 => wrong.push((i, Some(r.upper as f64))),
                    None if total < r.lower as f64 => wrong.push((i, Some(r.lower as f64))),
                    Some(at) => {
                        // Its equation's shortfall: > 0 wants more impulse, < 0 less.
                        let short = wanted[i] - self.velocity_in(r, &y) - r.gamma as f64 * d;
                        let tol = 1e-9 * (1.0 + wanted[i].abs());
                        let upper = at >= r.upper as f64;
                        if (upper && short < -tol) || (!upper && short > tol) {
                            wrong.push((i, None));
                        }
                    }
                    None => {}
                }
            }
            // Out of rounds: only rows past their bounds still switch, onto them, so the
            // impulses stay consistent (clamping one row alone would leave the joints
            // around it to take up the difference).
            let settle = round >= pivots;
            if settle {
                wrong.retain(|w| w.1.is_some());
            }
            if wrong.is_empty() {
                break;
            }
            if settle || wrong.len() < fewest {
                fewest = wrong.len();
                for (i, at) in wrong {
                    self.at[i] = at;
                }
            } else if let Some(&(i, at)) = wrong.last() {
                self.at[i] = at;
            }
            // Many rows off the guess (a body coming off or onto its stops this
            // substep): factoring again is cheaper than carrying them all.
            let off = self
                .rows
                .iter()
                .zip(&self.at)
                .filter(|(r, at)| r.slot.is_some() == at.is_some())
                .count();
            if off > REFACTOR && refactored < 2 && self.factor(sb).is_some() {
                refactored += 1;
                free = self.free(&wanted);
            }
        }

        for (i, r) in self.rows.iter().enumerate() {
            let d = match r.slot {
                Some(slot) => -y[slot],
                None => changed.iter().position(|c| c.0 == i).map_or(0.0, |k| u[k]),
            };
            // What the rounds didn't settle is clamped.
            let total = ((so_far[i] + d) as f32).clamp(r.lower, r.upper);
            let d = total - so_far[i] as f32;
            if d == 0.0 || !d.is_finite() {
                continue;
            }
            match r.source {
                Source::Anchor(axis) => joints[r.joint].point_impulse[axis] += d,
                Source::Muscle(k) => {
                    if let Some(m) = joints[r.joint].muscle.as_mut() {
                        m.impulse += m.frame.col(k) * d;
                    }
                }
                Source::Limit(l) => joints[r.joint].limits[l].impulse += d,
                Source::Contact(c, k) => contacts[c].impulse[k] += d,
            }
            if r.a != STATIC {
                let body = &mut sb[r.a];
                body.v += r.jacobian[0] * (body.inv_mass * d);
                body.w += body.inv_i * (r.jacobian[1] * d);
            }
            if r.b != STATIC {
                let body = &mut sb[r.b];
                body.v += r.jacobian[2] * (body.inv_mass * d);
                body.w += body.inv_i * (r.jacobian[3] * d);
            }
        }
    }

    /// The impulses on the changed rows on top of the free solution: a row in the
    /// factor held at its bound, a row out of it let in to satisfy its equation.
    fn adjust(
        &mut self,
        changed: &[(usize, Option<f64>)],
        free: &[f64],
        so_far: &[f64],
        wanted: &[f64],
    ) -> Vec<f64> {
        let m = changed.len();
        let columns: Vec<Vec<f64>> = changed.iter().map(|c| self.column(c.0).to_vec()).collect();
        let mut g = vec![0.0f64; m * m];
        let mut rhs = vec![0.0f64; m];
        for (k, &(i, at)) in changed.iter().enumerate() {
            let r = &self.rows[i];
            match (r.slot, at) {
                (Some(slot), Some(at)) => {
                    // −y(slot) = the impulse that puts it at its bound.
                    for (c, column) in columns.iter().enumerate() {
                        g[k * m + c] = -column[slot];
                    }
                    rhs[k] = at - so_far[i] + free[slot];
                }
                (None, Some(at)) => {
                    g[k * m + k] = 1.0;
                    rhs[k] = at - so_far[i];
                }
                (_, None) => {
                    // J Δv + γ u = what its equation asks for.
                    for (c, column) in columns.iter().enumerate() {
                        g[k * m + c] = self.velocity_in(r, column);
                    }
                    g[k * m + k] += r.gamma as f64;
                    rhs[k] = wanted[i] - self.velocity_in(r, free);
                }
            }
        }
        dense_solve(g, rhs, m)
    }
}

/// The rows of one joint: its anchor, its muscle's drive axes and its limit rows. A row
/// out of the factor starts at a bound: a muscle that ended the last substep at its
/// force limit, a stop not reached and not being closed this substep.
fn push_rows(
    rows: &mut Vec<Row>,
    sb: &[SolverBody],
    j: &JointRows,
    ji: usize,
    bias_rate: f32,
    h: f32,
) {
    let (a, b) = (j.a, j.b);
    let spin = |i: usize| if i == STATIC { Vec3::ZERO } else { sb[i].w };
    let push = (j.point_error * bias_rate).clamp_length_max(MAX_PUSH_SPEED);
    for (axis, e) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
        rows.push(Row {
            joint: ji,
            source: Source::Anchor(axis),
            a,
            b,
            jacobian: [-e, -j.ra.cross(e), e, j.rb.cross(e)],
            gamma: 0.0,
            lower: f32::NEG_INFINITY,
            upper: f32::INFINITY,
            bias: [push[axis], 0.0],
            slot: Some(0),
            parts: [(NONE, [0.0; 6]); 2],
            friction: None,
        });
    }
    if let Some(m) = &j.muscle {
        for k in 0..3 {
            let axis = m.frame.col(k);
            let gamma = axis.dot(m.gamma * axis);
            // An axis without a muscle can't push.
            if gamma >= 1e8 {
                continue;
            }
            let bias = axis.dot(m.bias);
            let warm = axis.dot(m.impulse);
            rows.push(Row {
                joint: ji,
                source: Source::Muscle(k),
                a,
                b,
                jacobian: [Vec3::ZERO, -axis, Vec3::ZERO, axis],
                gamma,
                lower: -m.bound[k],
                upper: m.bound[k],
                bias: [bias, bias],
                slot: (warm.abs() < m.bound[k] * (1.0 - 1e-6)).then_some(0),
                parts: [(NONE, [0.0; 6]); 2],
                friction: None,
            });
        }
    }
    // Two stops on one axis (a twist range narrower than the margin) would repeat each
    // other: only the nearer one.
    let mut skip = None;
    for (x, l) in j.limits.iter().enumerate() {
        for (y, o) in j.limits.iter().enumerate().skip(x + 1) {
            if !l.spring && !o.spring && l.axis.dot(o.axis) < -0.999 {
                skip = Some(if l.value <= o.value { y } else { x });
            }
        }
    }
    for (li, l) in j.limits.iter().enumerate() {
        if skip == Some(li) {
            continue;
        }
        let bias = if l.spring {
            [l.value, l.value]
        } else if l.value > 0.0 {
            // Not at the stop yet: it may close the gap this substep.
            [l.value / h, l.value / h]
        } else {
            [(bias_rate * l.value).max(-MAX_PUSH_SPEED * 4.0), 0.0]
        };
        // In the factor if it pushed at the end of last substep; a new one if the
        // joint's own motion pushes into it.
        let [seen, pushed] = j.warm_limits.map(|bits| bits & 1 << l.id != 0);
        let active = if seen {
            pushed
        } else {
            l.axis.dot(spin(b) - spin(a)) + bias[0] < 0.0
        };
        rows.push(Row {
            joint: ji,
            source: Source::Limit(li),
            a,
            b,
            jacobian: [Vec3::ZERO, -l.axis, Vec3::ZERO, l.axis],
            gamma: if l.spring { l.gamma } else { 0.0 },
            lower: l.lower,
            upper: l.upper,
            bias,
            slot: active.then_some(0),
            parts: [(NONE, [0.0; 6]); 2],
            friction: None,
        });
    }
}

/// What a character's contacts against the ground and fixed scenery need to be solved
/// with it (lesson 50).
pub(super) struct GroundContacts {
    /// Per body: it doesn't move and isn't keyed (massless scenery).
    pub fixed: Vec<bool>,
    /// Share of a contact's overlap pushed apart per second.
    pub bias_rate: f32,
    /// Coulomb friction (bounded by the normal impulse), else a fixed force cap.
    pub coulomb: bool,
    /// A contact this close (m) that pushed in the last substep carries a load.
    pub touching: f32,
}

/// A contact's three rows: its normal, which only pushes (or pulls up to its adhesion),
/// and two friction rows. The normal starts in the factor if it pushed at the end of the
/// last substep, friction if it did and didn't slide; the others at their bounds.
fn push_contact_rows(
    rows: &mut Vec<Row>,
    sb: &[SolverBody],
    c: &ContactRow,
    ci: usize,
    ground: &GroundContacts,
    h: f32,
) {
    let normal = rows.len();
    // Its body's own inverse mass along `dir`, for the rows' little give.
    let own = |dir: Vec3| {
        [(c.a, c.ra), (c.b, c.rb)]
            .into_iter()
            .filter(|&(i, _)| i != STATIC)
            .map(|(i, r)| sb[i].inv_mass + r.cross(dir).dot(sb[i].inv_i * r.cross(dir)))
            .sum::<f32>()
    };
    let row = |k: usize, dir: Vec3| Row {
        joint: NONE,
        source: Source::Contact(ci, k),
        a: c.a,
        b: c.b,
        jacobian: [-dir, -c.ra.cross(dir), dir, c.rb.cross(dir)],
        gamma: CONTACT_SOFTNESS * own(dir),
        lower: f32::NEG_INFINITY,
        upper: f32::INFINITY,
        bias: [0.0; 2],
        slot: Some(0),
        parts: [(NONE, [0.0; 6]); 2],
        friction: None,
    };
    // Still apart: it may close the gap this substep, no more. Overlapping: pushed apart
    // while correcting positions, as the contact pass does.
    let bias = if c.separation > 0.0 {
        [c.separation / h; 2]
    } else {
        [(ground.bias_rate * c.separation).max(-MAX_PUSH_SPEED), 0.0]
    };
    let pushing = c.impulse[0] > 0.0;
    let r = row(0, c.normal);
    rows.push(Row {
        gamma: r.gamma,
        lower: -c.interaction.adhesion * h,
        bias,
        slot: pushing.then_some(0),
        ..r
    });
    let touching = c.separation <= 0.0 || pushing;
    for k in 0..2 {
        let mut r = row(k + 1, c.tangents[k]);
        let limit = if ground.coulomb {
            r.friction = Some((normal, c.interaction.friction / 100.0));
            c.interaction.friction / 100.0 * c.impulse[0].max(0.0)
        } else if touching {
            c.interaction.friction * h
        } else {
            0.0
        };
        r.lower = -limit;
        r.upper = limit;
        r.slot = (pushing && c.impulse[k + 1].abs() < limit * (1.0 - 1e-3)).then_some(0);
        rows.push(r);
    }
}

/// `a` (n × k) times `b` (k × m), row-major.
fn multiply(a: &[f64], b: &[f64], n: usize, k: usize, m: usize) -> Vec<f64> {
    let mut c = vec![0.0f64; n * m];
    for i in 0..n {
        for t in 0..k {
            let x = a[i * k + t];
            if x == 0.0 {
                continue;
            }
            for j in 0..m {
                c[i * m + j] += x * b[t * m + j];
            }
        }
    }
    c
}

/// Transpose of an n × m matrix, row-major.
fn transpose(a: &[f64], n: usize, m: usize) -> Vec<f64> {
    let mut t = vec![0.0f64; n * m];
    for i in 0..n {
        for j in 0..m {
            t[j * n + i] = a[i * m + j];
        }
    }
    t
}

/// Inverse of a symmetric definite n × n matrix (row-major; negative definite if
/// `negative`) by Cholesky factorisation; `None` if it is singular.
fn invert_definite(a: &[f64], n: usize, negative: bool) -> Option<Vec<f64>> {
    let sign = if negative { -1.0 } else { 1.0 };
    let scale = (0..n)
        .map(|i| (sign * a[i * n + i]).abs())
        .fold(0.0, f64::max);
    // L Lᵀ = sign × a.
    let mut l = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut s = sign * a[i * n + j];
            for k in 0..j {
                s -= l[i * n + k] * l[j * n + k];
            }
            if i == j {
                if s <= 1e-14 * scale {
                    return None;
                }
                l[i * n + i] = s.sqrt();
            } else {
                l[i * n + j] = s / l[j * n + j];
            }
        }
    }
    // L⁻¹, lower triangular.
    let mut li = vec![0.0f64; n * n];
    for i in 0..n {
        li[i * n + i] = 1.0 / l[i * n + i];
        for j in 0..i {
            let mut s = 0.0;
            for k in j..i {
                s -= l[i * n + k] * li[k * n + j];
            }
            li[i * n + j] = s / l[i * n + i];
        }
    }
    // a⁻¹ = sign × L⁻ᵀ L⁻¹.
    let mut inv = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut s = 0.0;
            for k in i..n {
                s += li[k * n + i] * li[k * n + j];
            }
            inv[i * n + j] = sign * s;
            inv[j * n + i] = sign * s;
        }
    }
    Some(inv)
}

/// Solves the m × m system `a x = b` (row-major) by Gaussian elimination with partial
/// pivoting; a vanishing pivot leaves its unknown at zero.
fn dense_solve(mut a: Vec<f64>, mut b: Vec<f64>, m: usize) -> Vec<f64> {
    for col in 0..m {
        let pivot = (col..m)
            .max_by(|&x, &y| a[x * m + col].abs().total_cmp(&a[y * m + col].abs()))
            .unwrap_or(col);
        if pivot != col {
            for k in 0..m {
                a.swap(col * m + k, pivot * m + k);
            }
            b.swap(col, pivot);
        }
        let p = a[col * m + col];
        if p.abs() < 1e-300 {
            continue;
        }
        for r in col + 1..m {
            let f = a[r * m + col] / p;
            if f == 0.0 {
                continue;
            }
            for k in col..m {
                a[r * m + k] -= f * a[col * m + k];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = vec![0.0f64; m];
    for r in (0..m).rev() {
        let p = a[r * m + r];
        if p.abs() < 1e-300 {
            continue;
        }
        let sum: f64 = (r + 1..m).map(|k| a[r * m + k] * x[k]).sum();
        x[r] = (b[r] - sum) / p;
    }
    x
}

#[cfg(test)]
mod tests {
    use glam::{Quat, Vec3};

    use super::super::{Body, Joint, Motion, Muscle, World};
    use crate::document::physics::{ContactModel, Gains, Interaction, Limits, SpringCap};

    /// Multi Martial's punch: an arm (0.4 kg clavicle, 2.2 kg upper arm, 1.3 kg forearm,
    /// 0.5 kg hand, 0.06 kg fingers) held out from a keyed chest by endorphin-strength
    /// muscles (3000 N·m/rad), whose clavicle target jumps 0.6 rad and elbow 1.2 rad at
    /// 0.2 s. Returns how far the clavicle still lags its target 0.1 s later (rad) and
    /// the widest any joint opens (m).
    fn punch(cap: SpringCap) -> (f32, f32) {
        let mut w = World::new(Vec3::new(0.0, -9.81, 0.0), ContactModel::Coulomb);
        w.set_materials(1, |_, _| Interaction {
            restitution: 0.0,
            friction: 100.0,
            adhesion: 0.0,
            softness: 0.0,
        });
        w.spring_cap = cap;
        // (mass kg, length m); the chest is keyed.
        let specs: [(f32, f32); 6] = [
            (10.0, 0.0),
            (0.4, 0.15),
            (2.2, 0.3),
            (1.3, 0.27),
            (0.5, 0.1),
            (0.06, 0.08),
        ];
        let mut x = 0.0;
        for (i, &(mass, len)) in specs.iter().enumerate() {
            x += len;
            let across = mass * len.max(0.1) * len.max(0.1) / 12.0;
            let axial = mass * 0.04 * 0.04;
            w.bodies.push(Body {
                position: Vec3::new(x - len / 2.0, 1.5, 0.0),
                rotation: Quat::IDENTITY,
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
                inv_mass: 1.0 / mass,
                inv_inertia: Vec3::new(1.0 / axial, 1.0 / across, 1.0 / across),
                motion: if i == 0 {
                    Motion::Kinematic { momentum: 1.0 }
                } else {
                    Motion::Dynamic
                },
                group: 0,
                parent: i.checked_sub(1),
            });
        }
        let wide = Limits {
            twist_min: -1.5,
            twist_max: 1.5,
            swing1: 2.0,
            swing2: 2.0,
        };
        for i in 1..specs.len() {
            let (parent_len, len) = (specs[i - 1].1, specs[i].1);
            let mut j = Joint::new(
                i - 1,
                i,
                Vec3::new(parent_len / 2.0, 0.0, 0.0),
                Vec3::new(-len / 2.0, 0.0, 0.0),
                Quat::IDENTITY,
                Quat::IDENTITY,
                Quat::IDENTITY,
                Quat::IDENTITY,
                wide,
                wide,
                0.0,
                0.0,
            );
            let g = Gains {
                stiffness: 3000.0,
                damping: 150.0,
                force_limit: 5000.0,
            };
            j.muscle = Some(Muscle {
                target: Quat::IDENTITY,
                twist: g,
                swing: g,
            });
            w.joints.push(j);
        }
        let (mut lag, mut widest) = (0.0, 0.0f32);
        for frame in 0..120 {
            if frame >= 24 {
                for (j, angle) in w.joints.iter_mut().zip([0.6, 0.0, 1.2]) {
                    if let Some(m) = j.muscle.as_mut() {
                        m.target = Quat::from_rotation_z(angle);
                    }
                }
            }
            w.step(1.0 / 120.0);
            if frame == 35 {
                let turned = w.joints[0].local_rotation(&w.bodies);
                lag = turned.angle_between(Quat::from_rotation_z(0.6));
            }
            for j in &w.joints {
                let (a, b) = (&w.bodies[j.parent], &w.bodies[j.child]);
                let gap = (b.position + b.rotation * j.anchor_child)
                    - (a.position + a.rotation * j.anchor_parent);
                widest = widest.max(gap.length());
            }
        }
        (lag, widest)
    }

    /// The clavicle moves the whole arm, so it can be as stiff as the arm needs, and the
    /// joints hold while the hand and fingers whip round (lesson 38). Before, solved by
    /// sequential impulses with the 5 Hz whole-limb cap: 0.085 rad behind, joints 6.4 mm
    /// open; with that cap at 30 Hz, 0.165 rad and 9.3 mm. Capped for its own two bodies
    /// (NeuroMotion's own projects), the clavicle lags 0.61 rad.
    #[test]
    fn a_stiff_arm_punches_without_coming_apart() {
        let (lag, widest) = punch(SpringCap::Limbs);
        let (lag_bodies, _) = punch(SpringCap::Bodies);
        eprintln!("lag {lag:.3} rad (two-body cap {lag_bodies:.3}), widest joint {widest:.4} m");
        assert!(lag < 0.08, "the clavicle lags {lag} rad");
        assert!(widest < 0.003, "a joint opened {widest} m");
    }
}
