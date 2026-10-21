# Research notes: how endorphin's physics works

These notes come from a static read of endorphin 2.5.2's shipped files: the XML schema
in `Application/NaturalMotion.cmf`, the standard character, the sample scenes, strings in
the executable, and the help file. Nothing was executed. They describe *concepts* for
NeuroMotion to build on in its own way; no code or text is reused. Where something is a
guess rather than read from a file, it says so.

## Engine

- endorphin is built on the MathEngine **Karma** rigid-body toolkit. The executable
  contains Karma's collision and dynamics function names and its "Kea" constraint
  solver. That means a velocity-level constraint solver for joints, limits and contacts,
  not reduced-coordinate (Featherstone) dynamics.
- Physics runs at a fixed **120 Hz**. Output is recorded at 60 Hz (30 Hz in a few
  scenes), so each frame is 2–4 physics ticks.
- Gravity defaults to −9.8 to −9.81 m/s² on Y. Units are metres, kilograms and seconds.
- Simulation is a separate, explicit step. You press *Simulate* and frames are cached;
  editing means clearing frames and simulating again. A scene can cap the number of
  ticks to run.

## Bodies

- One rigid body per bone segment, 28 on the standard character.
- Each body has one *inertial* shape plus a density, which gives its mass. It also has
  separate, slightly smaller *collision* shapes: spheres, boxes, cylinders and capsules.
- Densities are tuning values, not tissue densities. They are chosen so segment masses
  come out realistic, about 88 kg total for a 1.8 m figure. For example, the head is a
  4.55 g/cm³ sphere of r 0.076 m, the thigh a 1.5 g/cm³ cylinder, and the toes a
  15 g/cm³ box.
- Inertia is treated as spherical (isotropic) by default.
- Feet touch the ground through small "rubber" spheres. Most bodies don't collide with
  their parent.

## Joints and limits

- Ball joints with **twist plus a two-axis swing cone**. Knees and elbows are ball
  joints with swing locked to ±0.02 rad, which makes them hinges. Fingers are true
  hinges. The root is free.
- Each joint has a soft limit about 0.05 rad inside a hard limit.
  - Typical hard ranges: shoulder twist 2.7 rad, knee twist 2.83 rad, spine around
    0.65–0.85 rad.
  - Left-side limits are mirrored copies of the right.

## Muscles (effectors)

- Every joint has a **PD servo** that drives it toward a desired orientation. It has
  separate stiffness, damping and **force limit** for twist and for swing, and drive
  modes for position, velocity or both.
- Characters carry named calibration sets:

  | Set | Stiffness | Damping | Force limit |
  |-----|-----------|---------|-------------|
  | Limp default | 50 | 100 | 50 |
  | Normal | 1000–3000 (knee/elbow twist 5000) | 500 | 1000 |
  | Pose holding | 1000 (hips/spine 5000) | 100 | 500 |

  An event-level *strength* multiplier scales them over time.
- Gains this stiff are stable at 120 Hz because the solver treats each drive as a
  force-limited constraint row instead of an explicit spring. That is inferred from
  the drive modes and Karma's design.
- "Virtual muscles" link the chest to the hands and the pelvis to the feet. They act
  as Cartesian springs for reaching and planting.

## Animation versus simulation (the timeline)

- **Simulation level per character**, switched by single-frame events:

  | Level | What it means |
  |-------|---------------|
  | No simulation | keyframes drive the character |
  | Collision only | keyed, but pushes other objects |
  | Collision with momentum | keyed, and transfers momentum to what it hits, scaled 0–1 |
  | Full simulation | fully dynamic |

  On switching to full simulation, the body carries the animation's position *and
  velocity* into physics.
- **Animation events** can be *passive* (played kinematically) or *active* (the muscles
  try to follow the animation), with a strength and a blend-in time.
- **Active pose events** set a target pose for the muscles, with a strength (often
  1–12) and a blend-in.
- **Transition events** blend from simulation back to animation over a frame window,
  linearly, optionally holding the root in place.
- **Force events** are one-frame *velocity changes* (m/s) on chosen bodies, not forces.
- **Constraint events** lock bodies to the world or to each other. **Sever events** cut
  joints or switch their stiffness off.
- **Behaviours** (stagger, catch fall, writhe, …) are controllers that read sensors
  (contacts, velocities, centre of mass) and write muscle targets and strengths. Their
  definitions are vendor-protected and were not examined.

## Where NeuroMotion can do better

These are the improvement targets for NeuroMotion's own physics:

1. **No manual simulate/clear cycle.** Re-simulate automatically from the first frame
   an edit affects, and cache the rest, so the timeline always shows current results.
2. **Muscle strength as a curve.** Strength, stiffness and damping per body region,
   keyed over time like any channel, instead of one value per event.
3. **Stable stiff muscles by construction.** Use an implicit or constraint-based drive
   so high gains can't explode, whatever the timestep.
4. **Smooth hand-back to animation.** Pose-match on the way out of simulation, so
   transitions don't need hand-tuned windows.
5. **Inspectable.** Show the simulated and animated pose together, and plot per-joint
   muscle effort on the timeline.
