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
| Open endorphin files | – | ✅ `.ens`, `.nmc`, `.nma` (160 of 163 bundled files) |
| Physics engine | Karma, 120 Hz, stiff muscles | ✅ own engine, 120 Hz, deterministic on one machine; all 16 sample scenes checked clean over their whole range |
| Event types | 9 | 🟡 7 of 9 (no motion transfer; behaviours partly) |
| Behaviours | a library of them | 🟡 6 (Balance with stepping, Stagger, CatchFall, BodyFoetal, Writhe, Explosion) |
| Your own characters | yes (rigs, motion transfer) | ⬜ only endorphin's own `.nmc` characters |
| Import motion capture | yes | ⬜ |
| Export | FBX, BVH | 🟡 FBX and BVH |
| Viewport | shaded 3D, several views | 🟡 bodies and bones drawn flat, one view |
| Editing in the viewport | posing, picking bodies | ⬜ |
| Re-simulating | press Simulate | ✅ automatic, in the background |
| Same results as endorphin | – | 🟡 Car Hit compared frame by frame with endorphin's own run and close; other scenes not compared yet |

## Milestone 1: Physics that matches endorphin

The foundation. Every scene that works in endorphin should give the same result in
NeuroMotion, and we need numbers to prove it.

- ✅ Rigid bodies, joint limits with soft zones, force-limited muscles, contacts, materials
- ✅ Simulation levels, momentum carried from animation, forces, transitions, active
  poses, active animations, constraints (lock and join, including negative
  "drive" strengths)
- ✅ Scenery props, characters placed in the world, the scene's own keys
- ✅ All 16 sample scenes clean: the real Car Hit's clavicle came apart in its
  transition back to animation, which shortened the bones it drew; transitions now
  blend each bone in its own pose's frame (lesson 18,
  [#7](https://github.com/Rick007110/NeuroMotion/issues/7)). Lessons 15 and 17 fixed
  the car's hit and the arm whipping round after it stops
  ([#23](https://github.com/Rick007110/NeuroMotion/issues/23)) in the Car carry
  stand-in. Bridge Escape is skipped for size
- ✅ Heavy bodies pressing on a character, friction only where surfaces touch, and
  characters with a joint come apart solved directly (lessons 15, 16 and 17)
- ✅ Car Hit measured against a recording of endorphin's: a lock that falls behind
  now pulls with its full strength, and imported friction is Coulomb, μ = friction
  / 100 (lessons 20 and 21). NeuroMotion's own projects keep force-capped friction
  until Balance and Stagger are tuned for Coulomb
- ✅ A force pushes the bodies it names, a massless body never moves, and a
  transition ends exactly on the animation (lessons 22-24); scenes run as long as
  endorphin runs them, and the sweep checks each scene's whole range
- 🟡 Back-Flip-To-Combo ([#6](https://github.com/Rick007110/NeuroMotion/issues/6)):
  overlapping active poses now each drive their own joints, and the head no longer
  starts inside its neck (lesson 19), but the character still leaves the box around
  frame 170, long before the flip's drive at 246, and with Coulomb friction its feet
  catch on the box's edge. endorphin's character stands, crouches and flips feet
  first. Open: how endorphin eases a pose in and what holds a joint after its pose
  ends
- 🟡 **Reference measurements**: scenes measured in endorphin (frames, positions,
  angles) and compared with NeuroMotion, so "behaves like endorphin" is checked, not
  eyeballed. Car Hit is the first: measured from a recording of endorphin's own run
  with its frame counter, against `dump_scene_motion`'s CSV of ours, which found
  lessons 20-24. Next: more scenes, and exports of endorphin's results (BVH/FBX)
  instead of recordings, turned into tests. Anyone with endorphin can help, using the
  *Reference measurement* issue template
- ⬜ **Walk With Gun**: since Coulomb friction its hands' join drifts up to 0.15 m and
  a finger sits 2.5 cm in the ground as the character falls with its 549 kg gun
  (within the sweep's checks)
- ✅ **Sever events**: cut a joint or switch its muscle off
  ([#13](https://github.com/Rick007110/NeuroMotion/issues/13)); endorphin's own setting
  names for them still need checking against a real scene
- ⬜ **Virtual muscles**: the chest-to-hand and pelvis-to-foot springs endorphin uses for
  reaching and planting (now skipped on import)
- ⬜ **Exact cylinders** ([#14](https://github.com/Rick007110/NeuroMotion/issues/14))
- ⬜ **Interpolated output** between physics ticks, for smooth 60 fps playback
- ⬜ **Large scenes**: Bridge Escape and other big scenes simulate at a usable speed
- ⬜ Legacy poses that store muscle targets (`dp.nma`)
- ⬜ Bit-identical results across platforms (optional): today the same build on the
  same machine always gives the same result, but other platforms' maths libraries
  round in the last bit, which chaotic moments can turn into a different outcome.
  Pure-Rust maths (e.g. the `libm` crate) would make every platform identical.

## Milestone 2: Behaviours

Behaviours are what make endorphin characters feel alive. endorphin's definitions are
protected, so every one is an original design that shares only the name. Imported
scenes then run something sensible.

- ✅ Balance (ankle strategy), CatchFall
- ⬜ **Balance and Stagger tuned for Coulomb friction**, so NeuroMotion's own projects
  can use endorphin's friction too (lesson 21)
- ✅ **Inventory**: the behaviour names and settings that endorphin scenes use,
  gathered from scenes (not from the protected `.nmb` files), to plan the rest:
  [docs/behaviour-inventory.md](docs/behaviour-inventory.md)
  ([#24](https://github.com/Rick007110/NeuroMotion/issues/24)). It found that
  imported names like `Bob2: Balance 2.5` matched none of our controllers, so no
  imported scene ran a behaviour; that is fixed
- 🟡 **Stepping** when Balance can't hold ([#9](https://github.com/Rick007110/NeuroMotion/issues/9)):
  forwards and backwards it now stands after pushes up to about twice as hard
  (50 of 84 test pushes, from 29); sideways it barely helps yet, because the foot on
  the falling side carries the weight and can't lift
  ([details](docs/behaviours.md#not-yet))
- 🟡 **Stagger** ([#8](https://github.com/Rick007110/NeuroMotion/issues/8)): Balance
  that keeps stepping with a loose trunk; its event settings aren't read and it
  hasn't been compared with endorphin
- ✅ **Writhe** and **Body foetal** ([#10](https://github.com/Rick007110/NeuroMotion/issues/10)):
  tested on the placeholder; not yet compared with endorphin
- ✅ **Explosion** ("Writhe in mid-air", 8 scenes with its 2.5 name WritheInMidAir):
  arms flapping, legs pedalling, back arched while in the air, limp once landed;
  matched to a recording of endorphin's behaviour scene and reads its event settings.
  In Car Hit the pedestrian now flies as high as endorphin's (1.75 m at frame 68,
  endorphin 1.81) and lands where it does
  ([details](docs/behaviours.md#explosion-writhe-in-mid-air))
- ✅ **Behaviour settings imported**: each behaviour event keeps its parameters
  (`Speed`, `Strength`, ...) on its clip; Explosion reads them, the others don't yet
- ✅ **Arms found on endorphin's characters**: the limb finder took the collarbone for
  the upper arm on every imported character (their forearms are split in two), so
  CatchFall, BodyFoetal and Writhe moved the collarbones; fixed
- ⬜ The rest of the library, in the inventory's order by scenes using them: Look and
  Reach (11), JumpDive, Jump and LandCrouch (5), Legs Kick and Arms
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

- ⬜ **3D viewport**: shaded bodies on the GPU, lighting and shadows, and front, side, top
  and perspective views ([#26](https://github.com/Rick007110/NeuroMotion/issues/26))
- ⬜ **Work in the viewport**: click bodies to select them; drag to aim a force; place
  constraints; pose with rotate gizmos and IK
- ⬜ **Pose tools**: mirror left↔right, a pose library, key the current pose
- ⬜ **Ghosting**: show the animated and simulated pose together, and motion trails
- ⬜ **Timeline**: an undo history window, separate playback and export ranges, and the
  simulation rate in the settings
- ⬜ **Preferences and keyboard profiles**, editable in the app
- ⬜ **Playblast**: export the viewport as a video
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
   five general differences in one go. Next: recordings with the frame counter, or
   better exports (BVH/FBX) of endorphin's own results, for a spread of scenes
   (Tackle, Stagger Walk, a few Behaviour Scenes, Back-Flip-To-Combo), turned into
   tests that score how close NeuroMotion is.
2. **Active poses as endorphin applies them** ([#6](https://github.com/Rick007110/NeuroMotion/issues/6)): how a pose eases in, what
   holds a joint after its pose ends and how strength scales the muscles. This is
   what keeps Back-Flip-To-Combo from standing until its flip.
3. **The other missing behaviours** in the inventory's order (Look and Reach,
   JumpDive). Explosion is done; Car Hit's flight now matches endorphin's height and
   landing place, turning over a few frames early.
4. **Balance and Stagger on Coulomb friction**, and sideways stepping
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
