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
| Collision spheres, boxes, cylinders, capsules ("sphyls") with self-collision flags | Same shapes and flags. Long cylinders collide as capsules, flat ones (wider than long) as discs with a rounded rim |
| Ball joints: twist range + swing cone between a parent-side and a child-side limit frame; soft zone inside a hard stop | Same frames, ranges and soft-zone springs (elliptical cone) |
| Effectors: force-limited PD on twist and swing; calibration presets | Same gains and presets, solved as implicit force-capped springs |
| Velocity-level constraint solver at 120 Hz | Velocity-level sequential-impulse solver at the scene's rate (120 Hz), substepped |
| Material pairs → restitution, friction, adhesion, softness | Same table. Friction defaults to endorphin-style force caps (newtons per contact) |
| Simulation levels; momentum carried into physics | Same four levels; kinematic bodies carry animation velocity into the switch |
| Force events are velocity changes on bodies | Same |
| Lock and join constraints, with per-axis position and orientation strengths | Rigid constraints whose force on each axis is capped by its strength (read as Karma's relative-position-relative-orientation joint; 0 = free) |
| Transitions blend simulation back to animation | Same, linear window, then the required level |

## Solver

Each tick (1/rate s) runs 4 substeps. Each substep:

1. Apply gravity to dynamic bodies.
2. Detect contacts at the current pose, with a 2 cm speculative margin so fast bodies
   don't tunnel. Bodies held together by a lock don't collide with each other.
3. Warm-start joint, muscle, lock and contact impulses from the previous substep
   (joint anchors from four fifths of it; nothing for bodies that just switched
   between keyed and simulated).
4. Iterate:
   - joint anchor points (3-row block);
   - twist and swing limits (one-sided rows; springs in the soft zone);
   - muscles;
   - locks (one 6-row block each, each row capped at its strength);
   - contact normals, as *soft* constraints (frequency and damping ratio);
   - friction.
5. Integrate positions.
6. Relax: one pass without position bias, so error correction doesn't add energy.
7. Restitution, for impacts faster than 0.5 m/s.

Keyed bodies (animated, collision-only, collision with momentum) are moved by the
caller with the animation's velocity. Joints and locks always see that velocity.
Contacts see it while correcting positions (step 4), so moving scenery can't pass
through anything, and only the momentum-scaled share in the relax pass: a collision-only
body pushes and carries things without setting them moving, and one with momentum 1
hands on its full velocity. When any keyed body of a character moves more than 0.5 m
or turns more than 1 rad in one tick (a cut, or snapping back from simulation), the
character is placed instead, without velocity.

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

The scenes with props, several characters and moving scenery taught four more:

5. **Locks are rigid, force-capped constraints, solved as one block.** They used to be
   a critically damped spring per axis, capped at 30 Hz for the locked body's own
   mass, and not warm-started. A rope's top segment, a ring or a hand weighs a few
   hundred grams, so the cap made the lock a soft spring while it carried a whole
   character, and props sagged by decimetres to metres. Like Karma's
   relative-position-relative-orientation joint, a lock now holds rigidly unless an
   axis needs more than its strength (newtons, or newton-metres for orientation;
   endorphin's 1e5 is effectively unbreakable). Position (at the held point) and
   orientation are one 6x6 block, so the lock feels the whole lever it moves. It is
   warm-started, and the bodies it joins don't collide: a join captured while bodies
   overlap would otherwise fight the contact forever. World locks use world axes;
   joins use the anchor's axes. That is a reading of Karma, not something the files
   state; a strength of 0 frees the axis either way. JoinBodies holds every listed
   body to the last one; LockWithParent does the same, and with a single body holds it
   to its parent body in its own rig (it used to create no lock at all).
6. **Keyed bodies show the solver their real velocity; momentum only decides what they
   hand on.** Collision-only bodies used to look stationary to the solver, so a moving
   floor or a hand swung through things, and props joined to an animated hand lagged.
7. **Forget warm starts when a body switches between keyed and simulated.** A keyed
   body crushing something against scenery or the ground builds up enormous contact
   impulses, because it can't give way. Replayed on a 60 g finger once the character
   goes simulated, that was 2,800 m/s in one substep and joints stretched by up to
   0.9 m. This is the likely cause of the Tackle failure (a collision-with-momentum
   character going fully simulated while colliding).
8. **Box-box contacts and flat cylinders are needed for scenery.** Character feet are
   often boxes, and so are floors and balconies, so without box-box contacts feet sank
   into them. A flat cylinder (a turntable) collided as a sphere of its radius.

Making characters balance (docs/behaviours.md) added a ninth:

9. **The spring cap counts grounded bodies as immovable.** The 30 Hz cap estimates the
   mass a muscle moves from the two bodies it joins. For an ankle that is a 1.1 kg
   foot, so its muscle was capped at about 270 N·m/rad, too weak to hold a standing
   body up (each ankle needs ~370). A planted foot is held by the ground, and its ankle
   moves the body above it. Bodies touching the ground or scenery now count as
   immovable for the cap (only for the cap; the solve itself is unchanged), unless
   both of a joint's bodies are grounded (a foot and its toe). Standing characters
   with strong presets now hold their stance.

A review of these changes turned up a tenth, older than all of them:

10. **Joint anchors warm-start from four fifths of last substep's impulse.** A chain of
    light links holding a heavy load and hanging straight (a rope or strap with a
    weight on it, before anything swings it) breathed: every link stretched and
    squeezed in step, a little more each substep, until it exploded (a 20 kg weight
    on an 8-link rope: 0.36 m stretch and 63 m/s within two seconds). It's the light
    chain/heavy load case sequential impulses converge worst on, and full warm starts
    overshoot it. Four fifths keeps it still (0.4 cm, 0.3 m/s). Locks and muscles keep
    their full warm start: cutting it for locks made loaded ropes sag. Extreme ratios
    still stretch while swinging: a 0.1 kg link locked at the top of seven, with 80 kg
    hanging from them and swung at 0.5-2 m/s, stretches 12-19 cm (2 cm with a keyed top;
    a 0.15 kg-link rope swinging a 75 kg character stays at 2 cm). An exact tree solve
    for joint anchors would remove the problem at its root.

## Status against endorphin's sample scenes

`cargo test --release -- --ignored --nocapture` simulates the first two seconds of
every bundled scene. It checks for:

- non-finite values;
- bodies (other than scenery) sinking more than 10 cm below the ground;
- joints stretching more than 5 cm;
- simulated bodies moving faster than 40 m/s.

It also prints lock drift and bodies buried in scenery, says where each worst value
happened, and for failing scenes and scenes with constraints prints the physics set-up
(ground, rigs, events, constraint targets and settings).

The open failures can't be reproduced in the cloud (the scenes ship only with
endorphin), so each has a stand-in built in code (`src/physics/scenes.rs`) with the
same pattern. All stand-ins pass now; the real scenes need re-checking on a machine
with endorphin.

| Result | Scenes |
|---|---|
| Clean before lessons 5-9; re-check, and look at anyone standing with a strong preset or running Balance or CatchFall | Arrows, Car Crash, Catch Run, Multi Martial, Stagger Walk, Sword Play, Trapeze, Walk With Gun |
| Re-check: stand-in passes | Tackle (lessons 5, 7), Balcony (7, 8), Catch Rope, Gymnast Rings, Bendy Stick, Heli Leap (5, 6), Rotating Floor (6, 8) |
| Re-check: not reproduced | Car Hit (6 cm stretch): the stand-in takes a 15 m/s hit with 3 cm |

What each stand-in reproduces:

| Stand-in | Pattern | Before | After |
|---|---|---|---|
| Tackle | Join (1e5) captured while colliding; the tackler drives the victim into the ground, then goes from collision with momentum to fully simulated | 0.36 m stretch | 1.2 cm |
| Balcony | Four characters shoulder to shoulder on a box slab; a fifth barges in keyed, then goes limp | feet 13 cm into the slab; then 0.91 m stretch, 77 m/s | 0.5 cm, 5 m/s |
| Catch Rope | A 1.5 kg rope locked at the top (position 1e5, 1e5, 0), caught by a running, jumping character | 0.74 m stretch, 1.4 m lock drift, 37 m/s | 2.2 cm, 0.8 cm |
| Gymnast Rings | Light straps locked at the top, a gymnast hanging by LockWithParent with hands overlapping the rings | 11 cm drift | 0.1 cm |
| Bendy Stick | A springy stick joined to the hand of a collision-only character swinging it | 10 cm drift | 0.2 cm |
| Heli Leap | A 1.7 t helicopter (fuselage, tail, skids) held by a Lock; a character jumps off | 17 cm drift | 0 |
| Rotating Floor | No ground; a thin collision-only slab spinning and rocking | sinks through | rides it |
| Turntable | No ground; a flat cylinder spinning | floats 0.9 m above it | rests on it |
| Hanging weight | A 20-80 kg weight on a rope locked at the top, hanging straight (lesson 10) | 0.36 m stretch, 63 m/s | 1.5 cm, 0.6 m/s |

Of endorphin's behaviours, Balance and CatchFall now have controllers of their own
(docs/behaviours.md). Scenes that depend on others (e.g. Stagger Walk's stagger)
simulate as if the character went limp.

## Not yet

- Behaviours beyond Balance and CatchFall (stagger, writhe, …), and stepping.
  endorphin's are vendor-protected, so these need original designs.
- Sever events.
- Virtual-muscle reach springs.
- Exact cylinders: long ones have rounded ends; flat ones a rounded rim, touching
  flat things at six rim points, the lowest rim point and the point nearest them.
- An exact (tree) solve for joint anchors, for chains with big mass ratios.
- Interpolated output between frames.

## Improvements planned beyond endorphin

- Re-simulate automatically from the edited frame onward, instead of from the start.
- Muscle strength as keyable curves per body region.
- Pose-matched hand-back from simulation to animation.
- Coulomb friction mode for physically scaled grip (available now as
  `ContactModel::Coulomb`).
