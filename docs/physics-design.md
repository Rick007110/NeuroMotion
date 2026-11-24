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
| Material pairs → restitution, friction, adhesion, softness | Same table. Imported friction is Coulomb with μ = friction / 100 (lesson 21); NeuroMotion's own projects still cap friction at that many newtons per contact |
| Simulation levels; momentum carried into physics | Same four levels; kinematic bodies carry animation velocity into the switch |
| Force events are velocity changes on bodies | Same |
| Lock and join constraints, with per-axis position and orientation strengths | Rigid constraints whose force on each axis is capped by its strength (read as Karma's relative-position-relative-orientation joint; 0 = free; a negative strength pushes or turns instead, lesson 13); one that has fallen behind pulls with its full strength until it catches up (lesson 20) |
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
   - locks (one 6-row block each, each row capped at its strength, closing a gap no
     faster than that strength could stop it, lesson 20);
   - bodies that locks tie together, with the joints between them, solved directly as
     one system (lesson 12), and the same for characters something much heavier
     presses on (lesson 15) or with a joint open more than 1.5 cm (lesson 17);
   - contact normals, as *soft* constraints (frequency and damping ratio);
   - friction.
5. Integrate positions.
6. Relax: one pass without position bias, so error correction doesn't add energy.
   Lock axes still at their strength after step 4 are left as they are (lesson 20).
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
way with the same build on the same machine. That determinism is what allows
automatic re-simulation. It isn't bit-identical across platforms: `sin`, `cos`,
`atan2`, `acos` and `powf` come from the platform's maths library, which can round
differently in the last bit (glibc even picks FMA variants by CPU at run time).
Usually that stays in the last bits, but a chaotic moment (a foot landing near the
edge of a recovery, a light limb whipping round) amplifies it into a visibly different
outcome. Tests therefore check statements that hold when the scene is nudged slightly
(`nudge_survey`), not one exact number near a limit.

## Muscle targets (what the muscles try to reach)

In priority order, for a simulated character at a given frame:

1. **Active pose events** covering the frame, each on the joints it names (all of
   them if it names none): the pose stored in the event, at the event's strength, with
   the `DesiredPoseCalibration` preset. Over its entry blend an event moves the target
   and the gains over from what the joint had before it; where several drive one
   joint, the latest to start wins (lesson 19).
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
    a 0.15 kg-link rope swinging a 75 kg character stays at 2 cm). A rope locked at
    the top is now also solved directly (lesson 12); chains without a lock still
    rely on the four fifths.

Simulating the real scenes with props being props added two more:

11. **The environment is simulated too; only its massless pieces are fixed.** Scenery
    used to start collision-only, as if endorphin keyed it. The files say otherwise:
    every scene's background has no inertial body, but every prop has one, scenes
    lock or join props to hold them in place, and Hands Reach and Look At switches its
    environment to NoPhysics so its reach target (a prop) doesn't fall. So the
    environment starts simulated like everything else, bodies without mass stay where
    they are, and props are real objects: a trapeze bar swings from its rope, a gun
    joined to both hands is carried by them. Invisible props without collision shapes
    fall through everything, in endorphin too, so the sweep ignores their height.
12. **Bodies held together by locks are solved directly, as one system.** With props
    simulated, sequential impulses met the cases they handle worst. A trapeze bar
    joined to a rope end hinged 6 m away swings on a lever hundreds of times the rope
    end's own inertia: it shook itself to 177 m/s instead of swinging at 7.6. A 549 kg
    gun (Walk With Gun sets the prop's density to 15) joined to both hands closes a
    loop through the arms with a thousand-to-one mass ratio: the arms came apart.
    Now, each iteration, after the per-joint and per-lock passes, every group of
    moving bodies that locks tie together is solved as one linear system: the locks'
    held axes and the anchors of every joint between those bodies. The system is
    factored once per substep with a sparse Cholesky factorisation (joints ordered
    leaf first, so a skeleton adds no fill-in). A lock axis that would pass its
    strength is held at it, and the rest is solved again without it. Three details
    make it robust:
    - Velocity and position correction are solved separately. Around a loop, joint
      and lock errors never quite agree, and an exact solve settles the disagreement
      with huge opposing impulses that move nothing but break locks. Position
      correction uses damped least squares: each row's diagonal is raised by 1%.
    - The velocity solve is regularised too, by 0.01%. Rows that nearly repeat
      each other (a ring hanging on a straight cable can barely move up or down, yet
      a lock says how far) would otherwise amplify small velocity mismatches, for
      example when a tackler switches from keyed to simulated while joined.
    - Lock axes that ended the last substep at their strength start out of the
      solve, so a weak lock (Gymnast Rings' 500 N) doesn't force a refactor every
      iteration.
    Scenes without locks are unchanged. Scenes with them simulate 1.3-3 times slower,
    still faster than real time.

A user's own scene, Back-Flip-To-Combo, added one more. In endorphin the character
crouches on a box, backflips off it and lands; in NeuroMotion it floated above the box.

13. **A negative lock strength drives instead of holding, and a character's massless
    body moves.** Two things held it up:
    - The flip comes from two locks on pelvis and upper spine with an angular strength
      of (-100, 0, 0). endorphin's help doesn't mention negative strengths, but the
      scene only works if they push: such an axis is no longer held (it drops out of
      the lock's block like a free one) and instead gets a steady force (N) or torque
      (N·m) of that value along the lock's axis, each substep, before the solve:
      world axes for world locks, the anchor's axes for joins. The locked body gets
      it and its anchor the opposite, so a join's drive conserves momentum. Axes
      with a positive strength on the same lock still hold. -100 about X spins a body
      backwards about the world's X axis (head towards -Z).
    - The character has a helper body, NormalMass, whose inertial body is a
      zero-sized Sphyl, so its mass comes out as 0. Lesson 11's rule (no mass: fixed
      in place) made it a fixed point inside the character, holding it in the air.
      That rule is for the environment's background now; a character's or prop's
      body without mass gets a token 1 kg (inertia 0.004 kg·m² on each axis).
    With both, the character leaves the box, but not yet when it does in endorphin.
    The stand-in shows one reason: left alone, the placeholder doesn't stand. 25 N
    toe locks can't hold it and the pelvis lock leaves pitch free, so it tips
    forwards from about frame 70, leans about 27° when the drive starts at 246, and
    the 200 N·m only slows that fall. Kept standing until the drive (Balance, in the
    stand-in), it flips backwards off the box as in endorphin.

Writing the test for a join's drive showed one more.

14. **A join's held axes turn with the anchor about where the body is, not where it
    should be.** A lock's rows used the target offset captured when it switched on
    as the anchor's lever. That is right while the body sits at the target, but a
    join with a free or driving position axis lets the body slide far along it. When
    the anchor turned, the body's point moved sideways at ω × (actual offset) while
    the rows counted only ω × (target offset), so the held axes drifted: a body
    driven 4 m along a join's axis, knocked sideways so the anchor turned, opened
    16 cm across a held axis. The lever is now the body's actual offset from the
    anchor, as a prismatic joint uses the current separation; the position error is
    measured from the target as before. The same gap now stays under 1 mm. For a
    held lock the two levers differ only by the lock's error, so every join's rows
    change by about that much.

Car Hit, the last sample scene failing the sweep, added one more.

15. **A character something much heavier presses on is solved directly, like a locked
    group.** In Car Hit a 747 kg car, joined to a keyed carrier, drives into a
    pedestrian at 8 m/s, carries them on the bonnet and stops dead, and a clavicle
    came apart by 10 cm. The Car carry stand-in sets it up the same way, with
    clavicles on the placeholder: the bonnet's edge catches the arm hanging at the
    pedestrian's side and drives it into their ribs, and the shoulder opened 5.8 cm
    and stayed open for about ten frames, right after the hit (whether the scene's
    clavicle goes at the hit or at the stop isn't known without its files). The car
    can't give way (its join holds it to the carrier) and the ribs only as the whole
    body does, so the arm is caught between them,
    and what keeps it on the body is a chain of joints, 2.2 kg upper arm, 0.4 kg
    clavicle, through to a 14 kg chest, under a contact load from something hundreds
    of times heavier than any of its links. Sequential impulses pass that load along
    the chain one link per iteration, and the contacts, solved last, win: the arm is
    pushed where the car wants it and the shoulder lets go. It is lesson 12's mass
    ratio problem without a lock in the chain.
    So the direct solve of lesson 12 now also covers, for the substep, every group of
    jointed bodies with a contact (closer than 1 cm) from a moving group that is at
    least 5 times heavier than it, or that a lock holds to the world or to a keyed
    body. The whole character's joint anchors are then solved as one system (no lock
    rows), and the load reaches the chest in one go. The shoulder now stays within
    1 cm (0.96 cm worst, a toe). The same rule closes two other ways a car came
    through the placeholder in the experiments that led to the stand-in: run over
    lying head first, the neck stretched 6.6 cm (now 0.4); lying on the bonnet with
    an arm under the body when the car stopped, the shoulder reached 4.9 cm (now
    2.5 cm, all of it in that pose's first frames).
    Characters that nothing heavy touches are unchanged, and every other stand-in
    gives the same numbers. Car carry takes about 110 ms instead of 75 for its 2 s
    (the character is direct-solved for most of the scene); scenes without such a
    contact don't pay for it.
    Keyed bodies don't trigger it. Counting a keyed body moving faster than 2 m/s
    as heavy helped the old Car Hit stand-in, whose car is keyed (3.0 to 1.5 cm), but
    nudged Balcony and Tackle (0.50 to 0.55 cm, 0.59 to 0.65 cm). Counting slow ones
    too (moving floors) helped Turntable (0.9 to 0.2 cm) but made it and Rotating
    Floor two to three times slower. Solving every character directly all the time
    slowed single characters about twice and made Balance lose its footing, so the
    direct solve stays where a heavy body needs it.
    Since lesson 16, nudging the car's speed by up to ±1e-3 over 201 runs
    (`nudge_survey`) shows how much of this holds: up to the stop (frame 99) no joint
    of the body, shoulders and clavicles included, opens more than 0.5 cm (an ankle);
    the toes' and fingers' hinges, which the bonnet strikes directly, open 0.5 to
    2.7 cm at the hit. After the stop was another matter (#23). Friction no longer
    holds the pedestrian on the bonnet (lesson 16), so they slide off, and the arm
    whips round (fingers at 11-15 m/s) with nothing heavy on it, so this lesson's
    direct solve doesn't apply. The shoulder then opened by 1.3 cm in a typical run,
    over 2 cm in a third of them and over the sweep's 5 cm in 11 of the 201 (7.0 cm
    worst), always between frames 113 and 120; on CI it reached 3.9 cm (Linux), 4.6
    (macOS) and 3.8 (Windows), against 2.1 here. Lesson 17 found the cause and fixed
    it.

Teaching Balance to step (docs/behaviours.md) showed one more.

16. **Friction only grips surfaces that touch.** Contacts are found 2 cm before
    surfaces meet (step 2 of the solver), so a fast body can't pass through in one
    substep. With the default force-capped friction, every such contact gripped with
    its full cap (100 N per contact by default) whether or not the surfaces
    touched. A foot swinging 1-2 cm above the floor was held as if planted: its hip
    and knee muscles, wanting 0.2 rad more, couldn't move it, and steps never landed.
    Now a contact grips only while its surfaces touch or it pushes them apart (it has a
    normal impulse). Coulomb friction was already right, being in proportion to that
    impulse. A weightless ball gliding at 1 m/s 1 cm above the ground went 0.37 m in
    0.5 s; it now goes 0.50 m (test `friction_grips_only_what_touches`). Of the
    stand-ins only the back flip changed: standing, it leaves the box at frame 324
    instead of 329 and lands at 387 instead of 393; left alone, it falls off the
    front at 335 instead of 348. Car carry changed after its stop: its pedestrian now
    slides off the bonnet (lesson 15).

An arm whipping round in Car carry after the car stops showed one more (#23).

17. **A character with a joint come apart is solved directly too.** After Car carry's
    stop, the shoulder opened by up to 7 cm. Tracing the solver substep by substep
    showed why, and that it was worse than the sweep said:
    - The stand-in's clavicles put a 0.4 kg body between chest and upper arm. That
      makes the chest the upper arm's grandparent and the neck an unrelated body of the
      same character, so by the placeholder's collision flags (limbs collide with their
      ancestors; unrelated bodies always collide) the upper arm now collides with
      both, right beside the shoulder. Without clavicles they were parent and sibling,
      and didn't.
    - By frame 111 the car no longer touches the pedestrian, so lesson 15's direct
      solve ends. The left arm is jammed up against chest and neck, 0.2-0.44 rad past
      its shoulder's swing stop, turning at 30-50 rad/s. Upper arm, clavicle and chest
      form a loop of joints that those contacts close, and four iterations of
      sequential impulses don't settle it: the contacts, solved last, win, and the
      shoulder opened by 5 mm every substep (2.4 m/s) for 40 substeps, 14 cm in the
      unnudged run, until the arm came off the chest. Then it closed again, slowly.
    - The sweep measures how much a bone's length changes, and most of that gap was
      across the clavicle, so it read 2.1 cm. Measured at the joint anchors, the
      shoulder opened 9.7 cm in a typical nudged run and 22 cm in the worst.
    Nothing else was involved: the arm touched neither the bonnet nor the ground in
    those frames, and of the joints that opened, only a finger and a toe reached the
    100 rad/s cap on angular speed.
    Eight iterations instead of four also fix it (worst 1.1 cm) but make every scene
    1.3-1.6 times slower; turning off collisions between a character's bodies that
    aren't parent and child also fixes it (worst 2.5 cm), which confirms the cause but
    would change what collides.
    So any character with a joint open more than 1.5 cm at the start of a substep is
    now solved directly for that substep, like lesson 15's crushed ones: all its joint
    anchors as one system. The shoulder then holds within a substep or two. At 1 cm the
    rule also caught Heli Leap's jumper landing at 12.5 m/s, whose ankles the ground's
    contacts open within one substep, and there it made them open a little more (2.6
    to 3.0 cm); at 2 cm the shoulder's anchors still opened 2.7 cm. Over the same 201
    nudged runs, after the stop:

    | | Before | After |
    |---|---|---|
    | Sweep (bone length), any joint | 1.3 cm typical, 7.0 worst; over 5 cm in 11 runs | 0.2 cm typical, 2.0 worst (a finger) |
    | Sweep, body without toes and fingers | 1.3 cm typical, 7.0 worst | 0.7 cm worst |
    | Joint anchors | 9.7 cm typical, 22 cm worst | 1.9 cm typical, 2.2 worst |

    The hit and carry don't change (0.5-2.7 cm for the toes and fingers, 0.4-0.5 cm for
    the body). Of the other stand-ins, Car Hit goes from 3.0 cm (its shoulder, at the
    hit) to 1.4 cm and Turntable from 0.17 to 0.16 cm; every other stand-in's joints
    stretch as much as before (Heli Leap's jumper lands a little differently, its
    lowest body 2.5 cm above the ground instead of 2.2). Balance's stepping gives the same result in every run of the
    survey, and Stagger the same step counts, with the feet moving a little
    differently in 21 of 201 runs (2.3-4.2 times Balance's, as before). Timings don't
    change beyond run-to-run noise: the rule only acts while a joint is open.
    `car_carry_keeps_joints_together` now holds the whole scene to 3.5 cm and the body
    after the stop to 1.5 cm (it failed that at 2.1 cm before the fix). The sweep's
    bone-length measure still under-reports a joint that opens sideways.

The real Car Hit, swept with endorphin installed, still failed after lessons 15-17,
and not where Car carry does.

18. **A transition's blend reads each bone in its own pose's parent frame.** The
    real Car Hit failed the sweep with a clavicle 7.9 cm out at frame 106, with the
    car long stopped (it stops at frame 102). The trace put the stretch inside the
    scene's 90-112 transition back to animation: it grew from frame 95, peaked at
    106 and was gone by 112, smoothly, as a blend weight would. Recording the
    simulated pose without the blend, nothing stretched more than 1 cm in those
    frames, and the solver never sees the blended pose, so the bodies were whole and
    only the drawn skeleton came apart. The blend slerps each joint's rotation, then
    rebuilds positions down the hierarchy by lerping each bone's offset from its
    parent between the simulated and the animated pose. It read the simulated offset
    in the *blended* parent's frame instead of the simulated parent's. Both offsets
    are the bone's rest offset in their own parent's frame; read in the wrong frame,
    the simulated one is turned away from the animated one by however far the
    blended parent is from the simulated one, and a lerp between two vectors at an
    angle is shorter than either (cos(θ/2) at the half-way point). Car Hit's
    character lies on the road after the car stops while its animation stands, so
    the angles were large. The offset is now read in the simulated parent's frame.
    The solver and the bodies are untouched: only the joints a transition records
    change, and no other stand-in has a transition, so their numbers, the nudge
    survey's and the timings are as before.

    The Curl hand-back stand-in curls a character with clavicles up on the ground
    with BodyFoetal and then transitions it back to standing over 0.75 s,
    overlapping the curl's end as Car Hit's transition overlaps BodyFoetal's. Its
    bones stretched 16.6 cm in the transition (a knee); now 0.09 cm, and
    `transition_keeps_bones_whole` holds them to 2 cm.

    | Sample scene | Before | After |
    |---|---|---|
    | Car Hit | 7.9 cm (clavicle, frame 106, mid-transition): failing | 0.93 cm (a foot, frame 53, while simulated): clean |
    | Stagger Walk | 1.9 cm (ankle, frame 120, in its 106-144 transition) | 0.07 cm |
    | The other 14 | | unchanged |

    Imported behaviour names ("Bob2: BodyFoetal") don't match a controller yet, so
    in these runs the real Car Hit's character goes limp where BodyFoetal should
    curl it (frames 60-105). Matching the name locally, just to measure, BodyFoetal
    runs and the scene still failed before this fix, the elbow 9.4 cm out at frame
    106 before this fix, 0.93 cm (the same foot, frame 53) after.

    Car Hit still reports 0.38 m of drift on the car's join to its 4 g carrier from
    frame 5. The sweep doesn't check drift, and the carrier is lighter than the car
    by a factor of about 180 000; that was a separate question (lesson 20).

A frame-by-frame comparison of Car Hit with a recording of endorphin's showed two
general mismatches. The first was in locks.

20. **A lock that has fallen behind pulls with its full strength until it catches
    up.** In Car Hit a 747 kg car is joined (1e5 N) to a 4 g keyed carrier, which
    switches to its keys at frame 2 and drives off at 8.2 m/s (16.5 m/s for that first
    frame, as it catches up with its keys). 1e5 N moves the car at 134 m/s², so it
    can't keep up at once: it fell 0.38 m behind by frame 5. Then it stayed behind,
    closing the gap at 0.3 m/s, 0.3 m behind still at frame 100. In endorphin the car
    follows the carrier from the start; in the recording it is 0.4-0.6 m ahead of
    NeuroMotion's at frames 8-30.
    The force cap wasn't the problem; how a capped lock corrects position was.
    Position correction adds velocity that the relax pass (step 6) takes back, so
    each substep the lock could close the gap by at most strength × h² / mass and
    keep none of the speed: the 0.3 m/s is 1e5 N × (1/480 s) / 747 kg. A force-capped
    constraint in Karma, which corrects errors in the same velocity solve that it
    caps, pulls with its full strength while it can't keep up, and the body gains
    speed. Now, too: a lock axis still at its strength when the position-correcting
    iterations end keeps that impulse through the relax pass, in the per-lock block
    and in the direct solve. Pulling with all its strength, a lock would arrive
    faster than it can stop and swing past (a weak lock round and round its target),
    so each axis also closes no faster than its strength could stop it on target:
    √(2 × strength × e / effective mass) for a gap e, as well as the usual 3 m/s.
    Strong locks never meet that limit at the gaps they keep (1e5 N on 747 kg allows
    1.6 m/s at 1 cm, where position correction asks for 1 m/s).

    | Stand-in or scene | Before | After |
    |---|---|---|
    | 50 N lock on a weightless 10 kg ball, 1 m from its target (`a_lock_that_falls_behind_pulls_with_its_full_strength`) | moved 0.5 cm in 0.5 s | 61 cm in 0.5 s, on target (within 0.01 mm) at 0.9 s without swinging past |
    | Heavy join: Car Hit's join alone, the carrier driving off at 8 m/s and stopping dead at 1 s (`heavy_join_catches_up`) | 23 cm behind at frame 4, still 18 cm at frame 15 and 15-16 cm for the rest of the drive and the second after the stop | 23 cm behind at frame 4, on its carrier (under 1 mm) from frame 11; 23 cm past it at the stop (frame 63), back by frame 71 |
    | Car Hit, the car's join | 12.0 cm mean drift, 0.3 m from frame 5 to the stop | 2.1 cm mean: 0.38 m at frame 5 as before, on the carrier from frame 14 |
    | Car Crash, the car caught by "Hold car in place" (full range) | 6.2 cm mean drift | 2.0 cm |
    | Car Hit's pedestrian, centre of mass x at frames 68/78/88 (endorphin 0.02/-0.11/-0.11 m) | -0.61/-0.91/-1.21 m | -0.55/-0.84/-1.12 m (friction is the rest, lesson 21) |

    The largest drift of each lock, what the sweep prints, is unchanged: Car Hit's
    0.38 m at frame 5 is the car falling behind as the carrier drives off, which a
    1e5 N join allows. Every other lock in the sample scenes drifts as before (mean
    and largest, over each scene's full range). In the 2 s sample sweep only scenes
    with weak locks move, a little: Rotating Floor (50-2000 N·m) stretches 0.36 cm
    instead of 0.30, Multi Martial (a 300 N·m guide) is unchanged but for its lowest
    body (1.4 cm instead of 1.3), and Car Hit stretches 1.12 cm instead of 0.93. Over
    the full ranges Rotating Floor, which already fails, sinks to -4.73 m instead of
    -4.55; Catch Rope and Heli Leap fail as before. Of the stand-ins, the back flip
    standing is unchanged and left alone leaves the box at frame 333 instead of 335;
    Car carry's car catches up before the hit, which moves its joints by a millimetre
    or so (over the nudge survey's 9 runs: 0.7-1.6 cm in the hit, 0.4-0.5 without toes
    and fingers, 0.1-0.5 after the stop). Balance's stepping and Stagger give the same
    result in every nudged run. Back-Flip-To-Combo, whose toe locks are 25 N, leaves
    the box at frame 168 instead of 167 and lands at 223 instead of 222. Timings don't
    change.

The second was friction.

21. **endorphin's friction is Coulomb friction: μ is its friction value / 100.** The
    second mismatch in Car Hit was the pedestrian. Running at about -4 m/s into the
    car, which drives at +8, endorphin's is stopped dead: its centre of mass is at
    x = 0.02, -0.11 and -0.11 m at frames 68, 78 and 88, and it lands at -0.3 to
    -0.44 m around frame 96. NeuroMotion's kept -1.8 m/s through the air
    (-0.55, -0.84, -1.12 m with lesson 20) and landed 1.2 m further, at frame 98.
    NeuroMotion read a material interaction's friction as a force cap per contact:
    100 N for endorphin's Default, whatever the load. Grip then depends on how many
    contacts the collision code happens to make rather than on how hard bodies
    press: a 10 kg box sliding from 4 m/s stops in 0.7 m, a 100 kg one in 2.0 m, and
    a body landing on a bonnet at several metres a second gets the same 100 N a
    resting one does. The files' values read better as Coulomb coefficients × 100:
    Icy 1 (μ = 0.01), Default 100 (μ = 1), Grippy 300 (rubber feet on tarmac, μ = 3),
    Sticky 700 with 700 N of adhesion; Karma's friction is Coulomb-style, limited in
    proportion to the normal force (the friction "box" is Karma's square
    approximation of the cone, which NeuroMotion's two tangent rows also make). So
    imported scenes now use `ContactModel::Coulomb`, the cap being friction / 100 ×
    the normal impulse, which already existed. Car Hit:

    | Car Hit (60 fps) | endorphin | Before (force caps, lesson 20) | Coulomb |
    |---|---|---|---|
    | Pedestrian's speed along the road at frame 70 | about 0 | -1.7 m/s | -0.1 m/s |
    | Centre of mass x at frames 68 / 78 / 88 | 0.02 / -0.11 / -0.11 m | -0.55 / -0.84 / -1.12 m | -0.14 / -0.15 / -0.17 m |
    | First touches the ground | around frame 96, at -0.3 to -0.44 m | frame 98, at -1.12 m | frame 90, at -0.17 m |

    μ is not tuned: Coulomb at μ = 1 without lesson 20 overshot (0.21 m at landing),
    and half of it (μ = 0.5) kept -1.6 m/s. The new stand-ins pin the difference down:
    `coulomb_friction_scales_with_the_load` (both boxes stop in 0.82 m, v²/2μg) and
    Bonnet slide, a 75 kg box landing on Heavy join's car at 4 m/s against it, which
    is stopped dead (-0.33 m/s three frames after landing, against -3.9 with force
    caps) as endorphin's pedestrian is.

    Two stand-ins needed a look, and one change to the scene:
    - Car carry, given Coulomb friction like the scene it stands in for, opened an
      ankle by 1.8 cm after the stop, and by up to 7.8 cm in nudged runs. Its car
      cleared the ground by 7 cm, and a foot caught under the chassis was dragged
      along the road at 8 m/s under the car's weight: in proportion to that load,
      friction held it back while the car carried the body on. Car Hit's car clears
      the ground by about 15 cm, and so does the stand-in's now. Over the 201 nudged
      runs the body then opens 0.3-0.7 cm up to the stop and 0.1-1.1 cm after it (the
      test holds it to 2 and 1.5 cm), toes and fingers 0.5-2.2 cm (3.5). A foot
      crushed under a car, with friction to match, still tears an ankle open; the
      direct solve covers joints, not contacts (see *Not yet*).
    - The back flip with Coulomb friction: standing, it leaves the box at frame 319
      instead of 324 and lands at 385 instead of 387, -281° in all. Left alone, its
      feet grip the box longer: it falls off the front at 338 and lands at 412
      instead of 399, still settling at frame 480, where the stand-in ended; it now
      runs 5 s.

    NeuroMotion's own projects, and stand-ins other than those built from Car Hit and
    Back-Flip-To-Combo, keep force-capped friction for now. Balance and Stagger were
    tuned on it, and their margins shrink with grip in proportion to load: a
    lightly loaded foot landing a step slides where a force cap held it. Over the
    nudge survey with Coulomb friction everywhere, Balance survives 4-7 of its 8 hard
    backward pushes (6-8 before; the test wants 4) and falls after 2.9-3.0 m/s
    forward pushes it used to stand through, so Stagger's feet go only 0.9-1.9 times
    as far as Balance's (2.3-4.0; the test wants 1.5). Imported scenes run Balance
    and Stagger on Coulomb friction already.

    The sample sweep stays 16 of 16 clean (2 s each):

    | Scene | Before (lesson 20) | Coulomb |
    |---|---|---|
    | Car Hit | 1.12 cm stretch, lowest body 2.0 cm | 2.94 cm (a knee, frame 93, landing), 0.6 cm |
    | Catch Rope | 0.15 cm | 0.95 cm |
    | Walk With Gun | 1.50 cm stretch, hands' join 2.6 cm drift | 0.78 cm; 6.6 cm drift and a finger 2.4 cm into the ground at frame 37, as the character falls with its 549 kg gun |
    | Trapeze | lowest body 3.08 m | 2.12 m (it swings differently) |
    | Sword Play | 12.6 cm drift (sword to hand) | 5.9 cm |
    | Heli Leap | lowest body -4.1 cm | 0.0 cm |
    | The others | | within a few millimetres |

    Over each scene's full range, the 13 clean scenes stay clean, and the three that
    already failed still fail: Catch Rope stretches 17.6 m instead of 18.0 after 2 s,
    Heli Leap's lowest body sinks to -2.16 m instead of -3.42 and Rotating Floor's to
    -4.69 m instead of -4.55 (-4.73 with lesson 20 alone). Back-Flip-To-Combo is no
    nearer endorphin's: its character still falls off the back of the box long
    before the flip (lesson 13), at frame 174 instead of 168, but its rubber feet,
    gripping at μ = 3, now catch on the box's edge and it hangs there until about
    frame 450, landing at 488 instead of 223 (endorphin: crouched on the box until
    the flip at about 280, landed behind it by 340). Timings don't change.

Sweeping the sample scenes over their whole range instead of their first two seconds
failed three of them: Catch Rope (a joint stretched 18.0 m), Heli Leap (a body 3.42 m
below y = 0) and Rotating Floor (4.55 m below). Tracing them found one bug in the
simulation and two in the sweep's own measures.

22. **A force pushes the bodies it names, and a
massless body never moves.** Catch Rope's environment drifted at 1 m/s from its first
frame, carrying the roof and the character standing on it. The scene's "Start rope
moving" force, a 1 m/s push on the rope's end, sits on the environment's track but
names the rope's body (`FBXImport03.MetaBodies.joint13MB`). Force targets were looked
up by their last name segment in the clip's own character; none matched, an empty list
meant "every body", and every body of the environment got the push, its massless root
included. A massless body is fixed (lesson 11), so the solver never writes its velocity
back, but nothing reset it either, and the integrator moved it at 1 m/s for the rest of
the scene. The rope never got its push. Two changes:
- A force pushes the bodies it names on whichever character they are, as constraints
  already did (a bare body name still means the clip's own character, and no targets
  still means every body of it); one naming only bodies that don't exist pushes
  nothing.
- The world zeroes a massless dynamic body's velocity every substep, whatever handed
  it one: a force, or its animation's velocity before it went simulated.

The Cut rope stand-in reproduces it: a rope locked at the top beside a roof that is the
environment's massless piece, pushed by a force on the environment's track. Before,
the roof drifted 1.98 m in 2 s, the climber standing on it fell off, and the rope's
end didn't move; after, the roof stays put and the end swings 0.14 m in 0.5 s. In
the real scene the character, carried along by the roof before, now rolls off its edge,
hangs from the rope's end by the hands' join and falls 16 m with it when the rope is
cut. Every other scene and stand-in, and `nudge_survey`, give the same numbers as
before: no other one has a force on another character's track or a moving massless
body.

23. **Without a ground plane, measure from the
scenery; a cut joint is no joint.** The other failures were the sweep's:
- Heli Leap and Rotating Floor set `useDefaultGround="false"`, and the sweep failed
  any body 10 cm below y = 0 all the same. Heli Leap's roof top is at y = 0, with
  nothing modelled under its 8.5 m building; the jumper leaves the roof's edge, is
  joined to the hand reaching down from the helicopter (frame 77) and dangles 3.4 m
  below roof level, beside the building. Rotating Floor's slab tilts by up to 37°;
  once its hands let go of the rail, the character slides down the slab to the wall at
  its low end, 4.5 m below y = 0 and on the slab all the while (its bodies 0.19-0.58 m
  above the slab's centre line, the slab being 0.3 m thick). Neither falls through
  anything. The sweep now measures the lowest body from the ground plane and only
  checks it when there is one. Without one, it checks that no body centre is more than
  10 cm under the underside of fixed scenery it is beneath, "up" being the shape's own
  axis closest to vertical, so a tilting floor keeps its top and underside. With a
  ground that check is off: bodies under a balcony or a beam are meant to be there
  (Balcony and Gymnast Rings have them, 2.8 and 4.9 m under).
- Catch Rope's 18 m is the rope's last link, which the scene's own Sever event ("Cut
  rope", frame 160) cuts so the character falls with it. A joint a sever event cuts no
  longer counts from the frame it is cut, as free props' missing joints never did.

The Tilting floor (a slab tilting 20° carries a character 1.9 m below y = 0) and Roof
edge (a character runs off a roof with no ground under it and falls 10.6 m) stand-ins
failed the old measure and pass the new one; Rotating Floor's dancer started 1 m under
its slab fails it, as it should. Imported BodyWrithe runs in all three scenes and isn't
involved: with it switched off they pass too (Catch Rope 2.4 cm, Heli Leap 0.29 cm,
Rotating Floor 0.45 cm).

| Sample scene, full range | Before | After |
|---|---|---|
| Catch Rope | 18.0 m stretch (the cut link), environment drifting at 1 m/s: failing | 0.90 cm, environment still: clean |
| Heli Leap | lowest body -3.42 m: failing | -3.42 m with no ground, nothing under scenery: clean |
| Rotating Floor | lowest body -4.55 m: failing | -4.55 m with no ground, nothing under scenery: clean |
| The other 13 | clean | unchanged |

Both scenes still differ from endorphin before the sweep can see it: their catches are
made by behaviours NeuroMotion doesn't have yet (JumpDive, Look and Reach), so the
joins switch on with the hands 1.8 m (Catch Rope) and 2 m (Heli Leap) from what they
catch, and hold that offset. Heli Leap's jump (a squat, downward pushes on the pelvis,
then a legs-out pose at frames 58-62) doesn't lift its jumper either: the pelvis sinks
from 0.90 m to 0.52 m over frames 54-62 and the jumper steps off the roof's edge.

24. **A transition's blend
    includes its last frame.** In Car Hit's 90-112 transition the drawn pose jumped
    for exactly one frame: the root was at x = -0.65 m at frame 111 (blended), -1.94 m
    at 112 (the raw simulated pose, lying on the road) and -0.62 m at 113 (animated).
    The blend covered `start <= frame < end`, but the level only changes to animated
    after frame `end` is recorded (the step from `end` to `end + 1` is the first
    with the level animated), so frame `end` was recorded with neither: the full
    simulated pose, between a blend that had nearly finished and the animation.
    The blend now covers `start <= frame <= end`; its weight is 1 at `end`, so that
    frame is the animation, and the bodies still switch one frame later as before.
    Only recorded joints change, so what undo, the timeline, the viewport and the
    exports (`io/motion.rs` reads the recorded joints) show is the fixed frames; the
    solver, the bodies and every other number are untouched.

    `transition_ends_on_the_animation_without_a_pop` uses the Curl hand-back stand-in
    (a curled character against a standing animation, so a pop is metres): the
    largest joint move from frame 179 to 180 was 2.35 m against a blend pace of 6.4
    cm per frame, now 6.5 cm; from 180 to 181, now 0.0, and frame 180 is within 1 mm
    of the animation. `transition_keeps_bones_whole` still holds (0.04 cm in the
    transition).

Tracing Back-Flip-To-Combo itself (#6), with the user's scene file, showed two more.

19. **Each active pose drives only the joints it names, and a shape already inside
    its parent at rest doesn't collide with it.** The scene has no behaviours: its 42
    active poses (41 enabled) and 83 forces do everything, with the constraints of
    lesson 13. Its poses split the body: one event drives the legs (`Objects`
    `character01.Joints.RThigh`, ...), another, overlapping it, the spine, head and
    arms, a third the feet, and so on, each at its own strength (0.05 to 5) and ease-in.
    Every event stores a whole-body pose. The simulation took the one event that came
    last in the file, applied its whole pose to every joint at its strength, and
    ignored the others; joints the pose left at rest have no channel (the importer
    drops those) and were held where the character started. Now each event drives
    the joints it names, or all of them if it names none, at rest where it has no
    channel. Where several drive one joint, they apply in the order they start, each
    moving the target (slerp) and the gains (lerp) over from what the joint had before
    over its ease-in, so the latest to start wins once it has eased in. A lone event
    used to ease its strength in from zero; it now eases in from the character's own
    muscles (`Default`, or `Normal` under an active animation).

    The second: the character's head has four collision shapes that may collide with
    their parent (`collidesWithParent`, on no other body), and they sit inside the
    neck at rest. Contacts push overlap apart at up to 3 m/s, so the head was thrown
    50° forward against the neck in the first ten frames of every run. In endorphin's
    video it stays upright. A shape that already overlaps one of its parent body's
    shapes in the rig's rest pose now doesn't collide with that parent; the head
    turns 0.5° in those frames. A shape on a fixed parent keeps colliding: Heli Leap's
    and Bendy Stick's background props sit inside the background body they belong to,
    and without that exception they fell through it.

    Stand-ins: Split poses hangs the placeholder by the hips, bends its knees by 60° in
    one event from frame 10 and its elbows by 60° in another from frame 20, each
    storing the other at rest. Before: knees 0° and elbows 59° at frame 150; after:
    59° and 60° (`overlapping_active_poses_each_drive_their_own_joints`). Head on
    neck grows the placeholder's head sphere into the neck and lets it collide with
    its parent: it turned 44° against the neck in the first second, now 2.6°
    (`a_shape_overlapping_its_parent_at_rest_does_not_push_it`). Every other stand-in
    gives the same numbers, and `nudge_survey`'s output is identical. endorphin's
    sample sweep stays 16 of 16 clean; the scenes with active poses or such shapes
    change a little: Catch Run's lowest body 1.6 to 0.4 cm, Gymnast Rings' stretch
    0.22 to 0.51 cm, Rotating Floor's 0.30 to 0.36 cm, Walk With Gun's 1.50 to
    1.25 cm (its drift 2.6 to 11.3 cm), Sword Play's drift 12.6 to 2.1 cm, Heli Leap's
    lowest body -4.1 to 0.0 cm, Bendy Stick's fastest body 9.2 to 8.7 m/s; the other
    nine are unchanged.

    This doesn't make the scene flip yet. endorphin's video (it plays the 726 frames
    in 7.05 s, about 103 a second) shows the character holding a T-pose, bringing its
    arms forward from frame ~100, crouching from ~185, springing back off the box
    around 250-260 as the drive (246-297) runs, turning over and landing feet first
    behind the box around 340. In NeuroMotion, before and after, it starts tipping
    backwards within the first 20 frames and falls off the back of the box head first
    before the drive starts:

    | | main | lesson 19 |
    |---|---|---|
    | Head in the first 10 frames | thrown 50° forward | 0.5° |
    | Leaves the box | frame 190 | frame 199 |
    | First touches the floor | frame 219, a finger | frame 228, the head |
    | Torso pitch at 246 / 297 (pelvis to head, 0 = upright, negative = backwards) | -253° / -309° | -242° / -263° |
    | Flips feet first and lands | no | no |

    What tips it: its centre of mass starts 4.4 cm in front of its heels' back edge
    (feet on the box from z = -0.034 to 0.231 m, centre of mass at 0.010 m), and the
    first pose swings its arms about 90° forward and down in half a second. The
    reaction leans the trunk back; by frame 40 the centre of mass is behind the heels
    and the 25 N toe locks can't hold it. In an experiment with the toe locks made
    unbreakable (and the head not yet fixed) it stood until the crouch but went over
    backwards between frames 200 and 210, before the drive. In the video the arms stay
    out until about frame 100, so either endorphin applies
    that first pose (frames 0-102, strength 0.55, 0.5 s ease-in) differently, or the
    timing reading of the video is off. Still open: how endorphin eases a pose in,
    whether a pose holds its joints after it ends (here the ankles are driven by
    nothing between frames 102 and 199), and how `strength` scales the preset.

## Status against endorphin's sample scenes

`cargo test --release -- --ignored --nocapture` simulates every bundled scene over
its whole frame range (about 12 s in all). It checks for:

- non-finite values;
- simulated bodies sinking more than 10 cm below the ground (fixed scenery, and bodies
  without collision shapes, aside; keyed bodies sit where their animation puts them);
  in a scene without a ground plane, bodies more than 10 cm under the underside of
  fixed scenery they are beneath instead;
- joints stretching more than 5 cm (free props have no joint to stretch, and a joint a
  sever event cuts none from then on);
- simulated bodies moving faster than 40 m/s.

It also prints lock drift and bodies buried in fixed scenery, says where each worst
value happened, and for failing scenes and scenes with constraints prints the physics
set-up (ground, rigs, events, constraint targets and settings).
`NM_TRACE=<path to an .ens scene> cargo test --release trace_stand_in -- --ignored
--nocapture` prints a scene frame by frame (its first two seconds, or `NM_SECONDS`).

Each failure also gets a stand-in built in code (`src/physics/scenes.rs`) with the
same pattern, so it can be worked on without endorphin's files.

Last run, over each scene's whole range, with lessons 1-24 and
imported behaviours actually running (until the behaviour names were matched,
`docs/behaviours.md`, none of them ran in these scenes); 16 scenes, Bridge Escape is
skipped for size:

| Result | Scenes |
|---|---|
| Clean | All 16: Arrows, Balcony, Bendy Stick, Car Crash, Car Hit, Catch Rope, Catch Run, Gymnast Rings, Heli Leap, Multi Martial, Rotating Floor, Stagger Walk, Sword Play, Tackle, Trapeze, Walk With Gun |
| Failing | None |

Max joint stretch per scene over its whole range: Arrows 1.27 cm, Balcony 1.16,
Bendy Stick 0.61, Car Crash 1.26, Car Hit 2.94, Catch Rope 1.12, Catch Run 1.41,
Gymnast Rings 0.40, Heli Leap 0.29, Multi Martial 1.74, Rotating Floor 0.41, Stagger
Walk 0.46, Sword Play 0.46, Tackle 0.66, Trapeze 0.82, Walk With Gun 0.78 (the whole
sweep takes about 12 s). Heli Leap's lowest body is 2.16 m and Rotating Floor's 4.69 m
below y = 0, in scenes without a ground (lesson 23). Over the first two seconds only,
as the sweep used to run, Car Hit was the last to fail: a clavicle 7.9 cm out, which
lessons 15 and 17 (fixed in the Car carry stand-in) didn't cure because the real
failure was in its transition back to animation (lesson 18).

Four of the five scenes still failing after lesson 12 weren't solver problems at all:
the importer set them up wrongly. Looking at Balcony in the app showed it: one
character sitting in mid-air, with no chairs and no balcony.

- **Characters weren't placed.** A character's own `TransformMatrix` (where it sits in
  the world) was ignored, and only its start pose's root offset used. So everyone
  started near the origin: Catch Rope's runner on the ground instead of a roof
  15.8 m up, the gymnast standing on the ground under the rings it should hang from,
  Rotating Floor's character inside the floor (it fell through), Car Hit's car on top of
  the pedestrian.
- **Props had no collision.** A collision surface without geometry of its own uses the
  mass shape ("environment mass objects have a collision object by default", says
  endorphin's help). It was read as no collision, so Balcony's chairs and table,
  Trapeze's bar, Catch Rope's rope and the gymnast's cables touched nothing.
- **The scene's own keys were dropped.** The scene's `<Animation>` section drives
  Car Hit's car and Car Crash's carrier box, the rotating floor and Heli Leap's rigs,
  outside any event. Without it, the cars never moved.

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
| Car carry | Car Hit: a 747 kg car joined (1e5) to a keyed carrier drives into a standing character with clavicles at 8 m/s, catches its arm against its ribs, carries it on the bonnet and stops dead at 1.65 s (lesson 15). The join gives way by 23 cm as the carrier stops, by design. After the stop the pedestrian slides off and the arm whips round against its own chest and neck. Coulomb friction, and the car 15 cm off the ground, as Car Hit's is (lesson 21) | 5.8 cm (shoulder) at the hit; 7.0 cm after the stop (#23) | Over 201 nudged runs, up to the stop: 0.7 cm at most for the body (shoulder included), toes and fingers 0.5-2.2 cm at the hit. After the stop: 1.1 cm at most (lessons 17 and 21) |
| Back flip | A character on a 2.5 m box: weak toe locks, a pelvis lock holding X and turning about Y and Z, then a -100 N·m drive about X on pelvis and upper spine (lesson 13); 120 fps, Coulomb friction (lesson 21) | floats above the box | standing (Balance until the drive): flips backwards, 1° at frame 246, -76° at 297, leaves the box at 319, lands behind it at 385, -281° in all. Left alone: tips forwards and falls off the front at 338, landing at 412 (changed by lessons 16 and 21) |
| Bonnet slide | Car Hit's pedestrian landing on the bonnet: a 75 kg box thrown at 4 m/s against Heavy join's car drops onto it (lesson 21) | slides on at 3.7-3.9 m/s | stopped dead: -0.33 m/s three frames after landing |
| Heavy join | Car Hit's join alone: a 747 kg car joined (1e5) to a light keyed carrier that drives off at 8 m/s and stops dead 1 s later (lesson 20) | 15-18 cm behind its carrier for the whole drive and after the stop | 23 cm behind at frame 4, on the carrier from frame 11; 23 cm past it at the stop, back 8 frames later |
| Split poses | Two overlapping active poses on different joints (knees from frame 10, elbows from 20), each storing the other joints at rest (lesson 19) | knees 0°, elbows 59° at frame 150 | knees 59°, elbows 60° |
| Head on neck | A head shape that may collide with its parent, inside the neck at rest (lesson 19) | turned 44° against the neck in 1 s | 2.6° |
| Curl hand-back | Car Hit's hand-back: a character with clavicles curls up on the ground (BodyFoetal), then a 0.75 s transition, overlapping the curl's end, blends it back onto its standing animation (lesson 18) | 16.6 cm (knee) mid-transition | 0.09 cm |
| Tilting floor | Rotating Floor's end: no ground; a long collision-only slab with a wall at its low end tilts 20°, carrying a character 1.9 m below y = 0 | fails the sweep (lowest -1.9 m) | rides it (nothing under the slab) |
| Roof edge | Heli Leap's jumper: no ground; a character runs off a roof whose top is at y = 0 and falls past the building's side | fails the sweep (lowest -10.6 m) | falls 10.6 m beside the building |
| Cut rope | Catch Rope: a force on the environment's track names a rope's end; the environment's roof is massless; the rope's last link is cut at 1 s | roof drifts 1.98 m in 2 s, rope end not pushed; the cut link counts as 0.83 m stretch | roof still, end swings 0.14 m; cut link not counted |

Of endorphin's behaviours, Balance (which now steps), CatchFall and Stagger have
controllers of their own (docs/behaviours.md). Scenes that depend on others simulate
as if the character went limp. Stagger Walk now runs the Stagger controller; it
hasn't been re-run against endorphin's scene since.

## Not yet

- Behaviours beyond Balance, Stagger, CatchFall, BodyFoetal and Writhe, and stepping
  that holds sideways pushes. endorphin's are vendor-protected, so these need original
  designs.
- Sever events are in (cut a joint, or switch its muscle off; `events.md`), but
  endorphin's own option names for them are unverified (`endorphin-formats.md`), and
  a severed joint can't be re-attached later.
- Virtual-muscle reach springs.
- Exact cylinders: long ones have rounded ends; flat ones a rounded rim, touching
  flat things at six rim points, the lowest rim point and the point nearest them.
- An exact solve for joint anchors where no lock holds the chain, nothing heavy
  presses on it and no joint has opened 1.5 cm yet (lessons 12, 15 and 17 cover
  those). Keyed bodies pressing on a character don't count as heavy yet: lesson 15
  says what that cost. Joints, limits and contacts are still solved one after
  another, contacts last, so a joint can open up to the 1.5 cm before the direct
  solve steps in, and a hard landing opens ankles by 2-3 cm for a substep or two.
- A sweep measure for joints that open sideways: it compares bone lengths, which
  read Car carry's 14 cm shoulder gap as 2.1 cm (lesson 17).
- Bit-identical results across platforms. Results are identical run to run with the
  same build on the same machine, but `sin`, `cos`, `atan2`, `acos` and `powf` come
  from the platform's maths library and can differ in the last bit, which a chaotic
  moment can grow into a different outcome. If identical results everywhere become a
  goal, the physics could use pure-Rust maths that rounds the same everywhere (the
  `libm` crate, say) instead of the platform's.
- Interpolated output between frames.
- Contacts in the direct solve. A foot crushed under a car, with Coulomb friction
  holding it to the road, tears its ankle open (lesson 21): the direct solve holds
  the joints, but contacts are solved after it.
- Saved projects keep the friction model they were saved with: one imported before
  lesson 21 keeps force-capped friction until it is imported again.

## Improvements planned beyond endorphin

- Re-simulate automatically from the edited frame onward, instead of from the start.
- Muscle strength as keyable curves per body region.
- Pose-matched hand-back from simulation to animation.
- Coulomb friction for NeuroMotion's own projects too (imported scenes have it,
  lesson 21), once Balance and Stagger are tuned for it.
