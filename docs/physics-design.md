# NeuroMotion physics: design

Goal: imported endorphin scenes should behave as close to endorphin as possible, and
then be improved on. Exact frame-for-frame equality isn't achievable even between
endorphin installs (endorphin warns that results differ across processor types), so the
target is the same model, the same parameters, and the same behaviour.

## Same model as endorphin

| endorphin (Karma) | NeuroMotion |
|---|---|
| One rigid body per bone; mass from inertial shape × density | Same, imported per body (`document::physics::BodyDef`) |
| Spherical inertia by default | Same (`isotropic_inertia`), full tensors available |
| Collision spheres, boxes, cylinders, capsules ("sphyls") with self-collision flags | Same shapes and flags. Cylinders collide as capsules; box-box isn't handled yet |
| Ball joints: twist range + swing cone between a parent-side and a child-side limit frame; soft zone inside a hard stop | Same frames, ranges and soft-zone springs (elliptical cone) |
| Effectors: force-limited PD on twist and swing; calibration presets | Same gains and presets, solved as implicit force-capped springs |
| Velocity-level constraint solver at 120 Hz | Velocity-level sequential-impulse solver at the scene's rate (120 Hz), substepped |
| Material pairs → restitution, friction, adhesion, softness | Same table. Friction defaults to endorphin-style force caps (newtons per contact) |
| Simulation levels; momentum carried into physics | Same four levels; kinematic bodies carry animation velocity into the switch |
| Force events are velocity changes on bodies | Same |
| Transitions blend simulation back to animation | Same, linear window, then the required level |

## Solver

Each tick (1/rate s) runs 4 substeps. Each substep:

1. Apply gravity to dynamic bodies.
2. Detect contacts at the current pose, with a 2 cm speculative margin so fast bodies
   don't tunnel.
3. Warm-start joint, muscle and contact impulses from the previous substep.
4. Iterate, resolving errors through *soft* constraints (frequency and damping ratio):
   - joint anchor points (3-row block);
   - twist and swing limits (one-sided rows; springs in the soft zone);
   - muscles;
   - contact normals;
   - friction.
5. Integrate positions.
6. Relax: one pass without position bias, so error correction doesn't add energy.
7. Restitution, for impacts faster than 0.5 m/s.

Muscles are springs with physical stiffness *k* and damping *c*, solved implicitly:
softness γ = 1/(h(c + hk)), bias β = hk/(c + hk). Their accumulated impulse is clamped
to ±(force limit × h). This makes them unconditionally stable at any gain, which is
what makes endorphin's 1000–6000 N·m/rad presets usable.

Everything is single-threaded with a fixed order, so a scene always simulates the same
way. That determinism is what allows automatic re-simulation.

## Muscle targets (what the muscles try to reach)

In priority order, for a simulated character at a given frame:

1. **Active pose event** covering the frame: the pose stored in the event, at the
   event's strength, eased in over its entry blend. Uses the `DesiredPoseCalibration`
   preset.
2. **Active animation** covering the frame: the animated pose at the clip's strength
   (imported `strength`, default 1 for NeuroMotion clips). Uses the `Normal` preset.
3. **Otherwise**: hold the pose the character had when it went into simulation, at the
   `Default` preset. This gives the slightly-tensed fall endorphin characters have.

The preset names come from endorphin's characters; this priority order is my reading
of how the events interact, not something the files state outright.

## Lessons from simulating endorphin's own characters

Making endorphin's sample scenes simulate cleanly needed four solver choices that
plain sequential impulses get wrong:

1. **Rigid joints, not mass-normalised soft joints.** Soft-step joints stretch
   when a light body sits on a long lever, because the effective mass is tiny.
   Joint anchors and limit stops use full-strength position correction instead.
2. **Each joint's muscle and stops are solved together with its anchor.** The
   muscle is a 6x6 block via a Schur complement, and each limit row uses the
   anchor-coupled effective mass. On its own, a muscle row only "feels" its body's
   own inertia, not the limb it moves, and ends up far too weak.
3. **Stops get the final say.** Order per joint: muscle, then stops, then anchor.
   If the muscle goes last, it pushes through stops and light end bodies blow up.
4. **Springs are capped at 30 Hz of natural frequency, for the mass they move.**
   endorphin's presets tuned for whole limbs would ring at 140-325 Hz on hands and
   fingers. Torso and limb muscles (3-15 Hz) are untouched.

Muscle targets are also clamped into each joint's soft range, because motion
capture often sits slightly outside a rig's limits.

## Status against endorphin's sample scenes

`cargo test --release -- --ignored --nocapture` simulates the first two seconds of
every bundled scene. It checks for:

- non-finite values;
- bodies sinking more than 10 cm below the ground;
- joints stretching more than 5 cm;
- simulated bodies moving faster than 40 m/s.

| Result | Scenes |
|---|---|
| Clean | Arrows, Car Crash, Catch Run, Multi Martial, Stagger Walk, Sword Play, Trapeze, Walk With Gun |
| Close | Car Hit (6 cm stretch) |
| Open | Balcony (four characters), Tackle (characters joined together), Bendy Stick, Catch Rope, Gymnast Rings, Heli Leap (props hung from constraints still drop), Rotating Floor (moving floor) |

Behaviours are not recreated, so scenes that depend on them (e.g. Stagger Walk's
stagger) simulate as if the character went limp.

## Not yet

- Behaviours (stagger, catch fall, …). endorphin's are vendor-protected, so these need
  original designs.
- Sever events.
- Virtual-muscle reach springs.
- Box-box contacts.
- Interpolated output between frames.

## Improvements planned beyond endorphin

- Re-simulate automatically from the edited frame onward, instead of from the start.
- Muscle strength as keyable curves per body region.
- Pose-matched hand-back from simulation to animation.
- Coulomb friction mode for physically scaled grip (available now as
  `ContactModel::Coulomb`).
