# Roadmap to endorphin level

The goal for NeuroMotion **1.0**: anything you did in endorphin 2.5, you can do in
NeuroMotion. That means the same physics results, the same kinds of events and
behaviours, your own characters and motion, and an editor that's at least as quick to
work in. After that comes the part where NeuroMotion goes further
([Beyond endorphin](#beyond-endorphin)).

This page lists what's done and what's missing, grouped into milestones that build on
each other, and [what's next](#next-up). Each open item is, or will become, a GitHub
[issue](https://github.com/Rick007110/NeuroMotion/issues). Pick one up; see
[CONTRIBUTING.md](CONTRIBUTING.md).

✅ done · 🟡 partly done · ⬜ not started

## Where we are

| Area | endorphin 2.5 | NeuroMotion today |
|---|---|---|
| Open endorphin files | – | ✅ `.ens`, `.nmc`, `.nma` (159 of the 161 bundled; the 2 password-protected tutorial scenes don't) |
| Physics engine | Karma, 120 Hz, stiff muscles | ✅ own engine, 120 Hz, deterministic on one machine; all 16 sample scenes checked clean over their whole range |
| Event types | 9 | 🟡 7 of 9 (no motion transfer; behaviours partly) |
| Behaviours | a library of them | 🟡 7 (Balance with stepping, Stagger, CatchFall, BodyFoetal, Writhe, Look and Reach, Explosion) |
| Your own characters | yes (rigs, motion transfer) | ⬜ only endorphin's own `.nmc` characters |
| Import motion capture | yes | ⬜ |
| Export | FBX, BVH | 🟡 FBX and BVH |
| Viewport | shaded 3D, several views | 🟡 bodies and bones drawn flat, one view |
| Editing in the viewport | posing, picking bodies | ⬜ |
| Re-simulating | press Simulate | ✅ automatic, in the background |
| Same results as endorphin | – | 🟡 eight scenes compared frame by frame with endorphin's own runs (exported as FBX); Car Hit close, Back-Flip-To-Combo now flips |

## Milestone 1: Physics that matches endorphin

The foundation. Every scene that works in endorphin should give the same result in
NeuroMotion, and we need numbers to prove it.

- ✅ Rigid bodies, joint limits with soft zones, force-limited muscles, contacts, materials
- ✅ Simulation levels, momentum carried from animation, forces, transitions, active
  poses, active animations, constraints (lock and join, including negative
  "drive" strengths)
- ✅ Scenery props, characters placed in the world, the scene's own keys
- ✅ Sample scenes clean, 17 of 17: the real Car Hit's clavicle came apart in its
  transition back to animation, which shortened the bones it drew; transitions now
  blend each bone in its own pose's frame (lesson 18,
  [#7](https://github.com/Rick007110/NeuroMotion/issues/7)). Lessons 15 and 17 fixed
  the car's hit and the arm whipping round after it stops
  ([#23](https://github.com/Rick007110/NeuroMotion/issues/23)) in the Car carry
  stand-in. Bridge Escape, skipped for size until the large-scenes item, was clean by
  luck: its 28 t truck cab, tumbling on its wheels, gained speed from its ground
  contact, and one run in five failed (flying off at 47.7 m/s, or buried 17 cm). The
  cause was the truck's hitch: at a large swing, its twist and swing stops pushed
  along axes that weren't their gradients and fought each other through the cab's
  one wheel on the ground. They now push along the true gradients (lesson 45), and
  Bridge Escape is clean in 9 of 9 runs with gravity nudged by up to ±4e-4
- ✅ Heavy bodies pressing on a character, friction only where surfaces touch, and
  characters with a joint come apart solved directly (lessons 15, 16 and 17)
- ✅ Car Hit measured against a recording of endorphin's: a lock that falls behind
  now pulls with its full strength, and imported friction is Coulomb, μ = friction
  / 100 (lessons 20 and 21). NeuroMotion's own projects use it too now; projects
  saved before keep force-capped friction
- 🟡 Free fall and thrown bodies follow g·t²/2 to the integrator's 1 cm a second;
  nothing damps a fast body (lesson 32). A level switch now acts on the step into its
  frame, as endorphin's does (lesson 42), and Car Hit's pedestrian flies to 1.83 m
  where endorphin's own run, exported as FBX, reaches 1.79 m (1.73 m before). The car
  still carries it along the road and sideways: its left foot sticks to the
  windscreen's face where endorphin's slides up it (lesson 44)
- ✅ A force pushes the bodies it names, a massless body never moves, and a
  transition ends exactly on the animation (lessons 22-24); scenes run as long as
  endorphin runs them, and the sweep checks each scene's whole range
- 🟡 Back-Flip-To-Combo ([#6](https://github.com/Rick007110/NeuroMotion/issues/6)):
  measured against endorphin's own run, exported as FBX (lesson 40), which showed the
  recording misread: endorphin's crouches, springs 1.1 m up off the box (centre of
  mass 3.745 m at frame 242), turns over and lands on the floor at 330. Lock strengths
  are now read as endorphin's kilograms-force (lesson 41): its 25 toe locks hold the
  crouch (centre of mass 3.19 m up and 0.155 m behind the heels at frame 192,
  endorphin 3.16 m and 0.164 m; it used to fall back off the box at 213) and it
  springs up and flips, its trunk turning as endorphin's (469° by frame 297, endorphin
  523°). Measured joint by joint, endorphin's active poses drive swing axes (measured
  on the spine, neck and head) as if at a sixteenth of their preset and hinges
  (knees, elbows) at all of it; imported scenes now do the same (lesson 46): the lean,
  arms, crouch, hip and spine bends come within endorphin's uncertainty (10 of 18
  rows, 2 before) and it lands on the floor at 329 (330). Its flip turns as
  endorphin's to frame 297 (527°, endorphin 523°): endorphin's drive fades as the body
  turns and is shared by the bodies it names (lesson 47). Imported scenes' contacts
  are now as stiff as the solver's substep allows, so its feet no longer sink into the
  box and tip it back (lesson 48): crouched 0.17 m behind the heels (0.16), it jumps
  to 3.69 m at 236 (3.745 m at 242), passes the box top at 292 (296) and lands at 328
    (330). Its contacts that carry a load are now rigid, solved with the rest of the
  character, and its knees push at most 400 N·m whatever the pose's strength, as
  endorphin's (lesson 50): it jumps to 3.74 m at 239 (3.745 m at 242), passes the box
  top at 294 (296) and lands at 332 (330), all within endorphin's uncertainty, with
  14 of 21 rows within. Measured step by step, endorphin's flip drive drives on again
  33° short of a whole turn where NeuroMotion's pushed back (lesson 51): the trunk is
  now 21° short by frame 330 (35° before). Still short: the trunk turns 18° too far by
  frame 260 and 21° too little by 330 (it is less tucked than endorphin's), so #6
  stays open; NeuroMotion's relax pass leaks angular momentum in flight, which hides
  a take-off that spins too fast (lesson 51); its knees bend 6.6° less in the crouch (endorphin's
  hold a quarter of their preset, which here loses the jump); its unnamed ankles still
  follow the legs' pose where endorphin's are limp (limp, it jumps 3.67 m); the
  crouch's height guard is near its edge (3.186 m, bound 3.188); and its combo jumps
  reach 1.07 m (1.67 m)
- ✅ **Reference measurements**: scenes measured in endorphin (frames, positions,
  angles) and scored against NeuroMotion by a test, `reference_scores`, so "behaves
  like endorphin" is checked, not eyeballed. 71 measurements in eight scenes: Car Hit,
  Back-Flip-To-Combo, Tackle, Stagger Walk, Trapeze, Balcony and the Look and Reach
  and Explosion behaviour scenes; 32 are within endorphin's uncertainty, 14 of them
  guarded against regressions, and the known gaps print their error so progress shows
  ([details](docs/reference-measurements.md)). 69 are read from endorphin's own runs
  exported as FBX (every joint at every frame, lesson 40; `tools/fbx_motion.py`
  reads an export), which corrected several recording-based rows; two still come from
  a recording. Anyone with endorphin can add rows, using the *Reference measurement*
  issue template
- ✅ **Walk With Gun**: since Coulomb friction its hands' join drifted up to 0.15 m and
  a finger sat 2.5 cm in the ground as the character fell with its 549 kg gun. Locks
  now get the last word over contacts in groups nothing pins, and the ground contacts
  that carry such a group's load join its direct solve: 0.7 cm and 1.4 cm above the
  ground over the whole scene. Not compared with endorphin's own run yet
- ✅ **Sever events**: cut a joint or switch its muscle off
  ([#13](https://github.com/Rick007110/NeuroMotion/issues/13)); endorphin's own setting
  names for them still need checking against a real scene
- ✅ **Exact cylinders** ([#14](https://github.com/Rick007110/NeuroMotion/issues/14)):
  flat caps and a sharp rim against every shape, resting still on a cap or a side
  (lesson 36). A cap rests on six rim points, not the whole circle
- ✅ **Interpolated output** between cached frames, for smooth 60 fps playback: the
  viewport blends the two neighbouring simulated frames at the fractional playhead, and
  export can write 2x or 4x the project's frame rate (the simulation itself still runs
  at 120 Hz and caches one state per scene frame; scrubbing clicks still snap to whole
  frames)
- ✅ **Large scenes**: Bridge Escape and other big scenes simulate at a usable speed.
  Measured, Bridge Escape was never slow: its file is large for its animation keys,
  and it simulates its 10.5 s in about 3 s on one core, so it is now in the sweep (17
  of 17 clean). Finding contacts no longer tests every pair of shapes (a sweep and
  prune, bit for bit the same results): Balcony, the slowest sample, 4.5 to 2.9 s for
  its 5 s. The rest of the time is the solver, mostly its direct solves
  ([physics-design.md](docs/physics-design.md), lesson 37). Solving imported
  characters whole (lesson 38) costs about twice that again (Balcony 5 s, Bridge
  Escape 6 s for its 10.5 s, Gymnast Rings 7 s for its 10 s, one core): every scene is
  still faster than real time, but making it cheaper is open
- ✅ Legacy poses that store muscle targets (`dp.nma`): imported as a held pose and
  usable as an active pose, mapped onto the character's joints and turned from a
  T-pose's arms to the pose's rest first
  ([endorphin-formats.md](docs/endorphin-formats.md#muscle-target-poses)). On the
  Standard character its spine bends less than the pose's: its 46° and 34° are past
  that spine's limits, and the middle spine joint gets no share
- ✅ Bit-identical results across platforms: the physics' `sin`, `cos`, `atan2`,
  `acos`, `exp` and the like come from the pure-Rust `libm` crate, and glam's from its
  `libm` and `scalar-math` features, so Linux, Windows and macOS simulate every scene
  bit for bit the same (CI checks a hash of three stand-ins on all three). No
  measurable cost in speed

## Milestone 2: Behaviours

Behaviours are what make endorphin characters feel alive. endorphin's definitions are
protected, so every one is an original design that shares only the name. Imported
scenes then run something sensible.

- ✅ Balance (ankle strategy), CatchFall
- ✅ **Balance and Stagger on Coulomb friction**, so NeuroMotion's own projects use
  endorphin's friction too (lessons 21 and 26): Balance's swing foot
  now clears the ground, and Stagger shuffles and steps short
- 🟡 **Virtual muscles**: the chest-to-hand and pelvis-to-heel springs endorphin uses for
  reaching and planting. Imported into the rig (not the skeleton or exported motion)
  and simulated as force-limited springs pulling the hand or heel to where the active
  poses that name them put it (docs/physics-design.md, lesson 35); the sweep stays 16 of
  16 clean, five scenes move a little. Still open: active animations don't drive them
  yet (doing so tipped Multi Martial's Stagger landing over), and endorphin's own gains
  (1 in every preset field) make them weak, more brake than pull; how endorphin scales
  them isn't known. endorphin's own runs of Trapeze and Balcony, whose poses name them,
  don't settle it either (lesson 43): Trapeze's acrobat is elsewhere by the time its
  poses drive them (it needs JumpDive), and Balcony's export is a character no such
  pose drives. Moved here from Milestone 1: settling their gains needs JumpDive first
- ✅ **Inventory**: the behaviour names and settings that endorphin scenes use,
  gathered from scenes (not from the protected `.nmb` files), to plan the rest:
  [docs/behaviour-inventory.md](docs/behaviour-inventory.md)
  ([#24](https://github.com/Rick007110/NeuroMotion/issues/24)). It found that
  imported names like `Bob2: Balance 2.5` matched none of our controllers, so no
  imported scene ran a behaviour; that is fixed
- 🟡 **Stepping** when Balance can't hold ([#9](https://github.com/Rick007110/NeuroMotion/issues/9)):
  on the placeholder it now stands after 68 of 84 test pushes (29 without
  stepping, 50 before its swing foot was raised), up to 3 m/s forwards; sideways
  is still weakest, because the foot on the falling side carries the weight and
  can't lift ([details](docs/behaviours.md#not-yet)). On endorphin's own character
  (Stagger Walk) it didn't step at all, the engine's spring cap weakening that rig's
  hips, knees and ankles several times over; with imported characters solved whole and
  their muscles as stiff as their limbs need (lesson 38), it steps back about seven
  times (endorphin about five) and stays up in every nudged run, though not as far back
  as endorphin's ([details](docs/behaviours.md#stagger-walk))
- 🟡 **Stagger** ([#8](https://github.com/Rick007110/NeuroMotion/issues/8)): Balance
  that keeps stepping with a loose trunk; its event settings aren't read and it
  hasn't been compared with endorphin
- ✅ **Writhe** and **Body foetal** ([#10](https://github.com/Rick007110/NeuroMotion/issues/10)):
  tested on the placeholder; not yet compared with endorphin
- ✅ **Look and Reach** (11 scenes): hands reach for a body or prop, the head looks
  at one, the rest holds its pose. Its targets and settings are imported, and it was
  compared with endorphin's run of "Hands reach and look at": arms on the target at
  the same pace (frame 20-24), hands within 24 px of endorphin's on average where
  the T-pose was 81 px off ([details](docs/behaviours.md#look-and-reach)). The hands
  come in about 10° low, and some settings aren't read yet
- ✅ **Explosion** ("Writhe in mid-air", 8 scenes with its 2.5 name WritheInMidAir):
  arms flapping, legs pedalling, back arched while in the air, limp once landed;
  matched to a recording of endorphin's behaviour scene and reads its event settings
  ([details](docs/behaviours.md#explosion-writhe-in-mid-air)). In Car Hit the
  pedestrian's centre of mass now rises to 1.62 m at frame 65 (endorphin 1.81 m at 68)
  and it comes down feet first at frame 89 (endorphin about 96)
- 🟡 **Behaviour settings imported**: every behaviour event now carries its settings
  and targets; Look and Reach and Explosion read them, the others don't yet
- ✅ **Arms found on endorphin's characters**: the limb finder took the collarbone for
  the upper arm on every imported character (their forearms are split in two), so
  CatchFall, BodyFoetal and Writhe moved the collarbones; fixed
- ⬜ The rest of the library, in the inventory's order by scenes using them:
  JumpDive, Jump and LandCrouch (5), Legs Kick and Arms
  Windmill (4 each), Tackle and Fall Back and Twist (3 each), the arm poses (8 scenes
  between them), then stiffness, leg straightening and velocity damping
- ⬜ **Behaviour settings in the inspector**, so animators can tune them, as with other
  events
- ⬜ A **guide to writing a behaviour**, so the community can add them in parallel

## Milestone 3: Your own characters, props and motion

endorphin wasn't only for its bundled characters: animators brought in their own rigs
and motion capture. Without this, NeuroMotion only works for people who already have
endorphin scenes.

- ⬜ **Import motion**: BVH and FBX animation as motion clips, to simulate on top of
  ([#25](https://github.com/Rick007110/NeuroMotion/issues/25))
- ⬜ **Your own skeleton**: import a character from FBX or BVH and map it to a physics
  rig (endorphin's motion transfer), so results come back onto your own skeleton
- ⬜ **Motion transfer events**
- ⬜ **Character editor**: body shapes, masses, joint limits and muscle strengths,
  edited in the inspector and shown in the viewport
- ⬜ **Props and environment**: add the ground, boxes, ramps, stairs and other props
  from within NeuroMotion, and set gravity and friction. endorphin's `.nmw` world
  files are protected, so this replaces them.
- ⬜ A **NeuroMotion character** of our own, so new users can start without endorphin

## Milestone 4: An editor animators want to work in

- 🟡 **Physics settings panel**: gravity, simulation rate and length, looping, the
  material interactions (restitution, friction, adhesion, softness) and which pair
  of surface materials uses which, all editable and undoable (in progress)
- 🟡 **Scene tree**: characters, props and the environment with their bodies,
  selectable, the inspector showing a body's mass, shapes and material (in progress)
- ⬜ **Toolbar**: select, move and rotate, add shapes (box, sphere, cylinder, capsule)
  and props, place constraints and forces; builds on the 3D viewport
- ⬜ **3D viewport**: shaded bodies on the GPU, lighting and shadows, and front, side, top
  and perspective views ([#26](https://github.com/Rick007110/NeuroMotion/issues/26))
- ⬜ **Work in the viewport**: click bodies to select them; drag to aim a force; place
  constraints; pose with rotate gizmos and IK
- ⬜ **Pose tools**: mirror left↔right, a pose library, key the current pose
- ⬜ **Ghosting**: show the animated and simulated pose together, and motion trails
- ⬜ **Timeline**: an undo history window and separate playback and export ranges
- ✅ **Physics settings panel**: simulation rate, gravity, contact model, muscle cap,
  material interactions and material pairs, editable on the project page of the
  inspector and undoable ([docs/layout.md](docs/layout.md)); the frame range sets the
  length
- ✅ **Scene tree**: a left panel listing characters, props and the environment with
  their physics bodies; a body shows its mass, shapes and material in the inspector,
  read-only for now (editing bodies, shapes and materials is still open)
- ⬜ **Preferences and keyboard profiles**, editable in the app
- ⬜ **Playblast**: export the viewport as a video or an image sequence
- ✅ Window and app icon ([#17](https://github.com/Rick007110/NeuroMotion/issues/17))
- ⬜ Recent files ([#16](https://github.com/Rick007110/NeuroMotion/issues/16))

## Milestone 5: Ready for everyone (1.0)

- ⬜ **Signed installers** for Windows and macOS
  ([#15](https://github.com/Rick007110/NeuroMotion/issues/15))
- ⬜ **User manual and tutorials**: a knock-down, a fall from a height, a catch, moving
  from endorphin
- ⬜ **Sample scenes** of our own, to learn from and to test against
- ⬜ **Performance**: several characters simulate faster than real time on an ordinary
  laptop
- ⬜ **Crash safety**: autosave, and reporting a problem without losing work

## Beyond endorphin

Already done:
- automatic re-simulation in the background;
- deterministic results (on the same machine);
- stiff muscles that can't explode;
- locked bodies solved exactly.

Planned once 1.0 is in sight:

- Re-simulate from the edited frame instead of from the start
  ([#11](https://github.com/Rick007110/NeuroMotion/issues/11))
- Muscle strength as keyable curves per body region
  ([#12](https://github.com/Rick007110/NeuroMotion/issues/12))
- Pose-matched hand-back from simulation to animation
- Muscle effort shown on the timeline and in the viewport
- Export to game engines (Unity, Unreal) and USD

## Next up

What moves NeuroMotion closest to endorphin quality now, in order:

1. **Compare more scenes with endorphin, by numbers.** The Car Hit comparison found
   five general differences in one go, and `reference_scores` now scores five scenes
   (docs/reference-measurements.md). Next: exports (BVH/FBX) of endorphin's own
   results instead of recordings, and more scenes (Tackle, the other sample and
   behaviour scenes).
2. **Back-Flip-To-Combo's jump** ([#6](https://github.com/Rick007110/NeuroMotion/issues/6)): it now holds its poses as endorphin's does up to
   the take-off (lesson 46), its planted feet are rigid (lesson 50) and its jump,
   the frames it passes the box top and lands are within endorphin's uncertainty.
   Next: the take-off's spin with the relax pass's angular momentum leak fixed
   (lesson 51), the flip's tuck (18° too far by 260, 21° short by 330), then the
   knees at a quarter of their preset and limp unnamed joints without losing the
   jump (lesson 50).
3. **The other missing behaviours** in the inventory's order (JumpDive, Jump,
   LandCrouch; Look and Reach and Explosion are done).
4. **Stepping on endorphin's characters** (Stagger Walk steps and stays up since
   lesson 38, but not as far as endorphin's, `docs/behaviours.md#stagger-walk`), and
   sideways stepping
   ([#9](https://github.com/Rick007110/NeuroMotion/issues/9)).
5. **Import BVH and FBX** ([#25](https://github.com/Rick007110/NeuroMotion/issues/25)) and the **3D viewport**
   ([#26](https://github.com/Rick007110/NeuroMotion/issues/26)).

## Releases along the way

There will be releases before 1.0. Each is a beta (pre-release) with whatever is done
by then:

- ✅ **0.1: first public beta** (released). Downloads for Windows, macOS and Linux. Opens and
  simulates endorphin scenes with five behaviours, sever events, and exports FBX and
  BVH.
- **0.2: physics parity.** Milestone 1 done: every sample scene clean, the first
  reference measurements passing.
- **0.3: behaviours.** The inventory's most used behaviours, sideways stepping,
  behaviour settings in the inspector.
- **0.4: your own motion.** Import of BVH and FBX animation, your own characters, props.
- **0.5–0.9: the editor.** The 3D viewport, editing in the viewport, pose tools.
- **1.0: endorphin level.** Milestones 1–5 done.
