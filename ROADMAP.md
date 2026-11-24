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
| Physics engine | Karma, 120 Hz, stiff muscles | ✅ own engine, 120 Hz, deterministic on one machine; 15 of 16 sample scenes clean, Car Hit fixed in its stand-in |
| Event types | 9 | 🟡 7 of 9 (no motion transfer; behaviours partly) |
| Behaviours | a library of them | 🟡 5 (Balance with stepping, Stagger, CatchFall, BodyFoetal, Writhe) |
| Your own characters | yes (rigs, motion transfer) | ⬜ only endorphin's own `.nmc` characters |
| Import motion capture | yes | ⬜ |
| Export | FBX, BVH | 🟡 FBX and BVH |
| Viewport | shaded 3D, several views | 🟡 bodies and bones drawn flat, one view |
| Editing in the viewport | posing, picking bodies | ⬜ |
| Re-simulating | press Simulate | ✅ automatic, in the background |

## Milestone 1: Physics that matches endorphin

The foundation. Every scene that works in endorphin should give the same result in
NeuroMotion, and we need numbers to prove it.

- ✅ Rigid bodies, joint limits with soft zones, force-limited muscles, contacts, materials
- ✅ Simulation levels, momentum carried from animation, forces, transitions, active
  poses, active animations, constraints (lock and join, including negative
  "drive" strengths)
- ✅ Scenery props, characters placed in the world, the scene's own keys
- 🟡 All 16 sample scenes clean: Car Hit's cause is fixed in a stand-in (a heavy body
  pressing on a character now gets an exact joint solve, lesson 15, and so does a
  character with a joint come apart, lesson 17, which stops the arm whipping round
  after the car stops from opening the shoulder
  ([#23](https://github.com/Rick007110/NeuroMotion/issues/23))), and the real scene
  still needs confirming on a machine with endorphin
  ([#7](https://github.com/Rick007110/NeuroMotion/issues/7)); Bridge Escape is skipped
  for size
- ✅ Heavy bodies pressing on a character, friction only where surfaces touch, and
  characters with a joint come apart solved directly (lessons 15, 16 and 17)
- 🟡 Back-Flip-To-Combo: the character tips forward before the flip
  ([#6](https://github.com/Rick007110/NeuroMotion/issues/6))
- ⬜ **Reference measurements**: a set of scenes measured in endorphin (frames, angles,
  heights) turned into tests, so "behaves like endorphin" is checked, not eyeballed.
  Anyone with endorphin can help, using the *Reference measurement* issue template.
  Nothing NeuroMotion does has been compared with endorphin by numbers yet.
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
- ✅ **Inventory**: the behaviour names and settings that endorphin scenes use,
  gathered from scenes (not from the protected `.nmb` files), to plan the rest:
  [docs/behaviour-inventory.md](docs/behaviour-inventory.md)
  ([#24](https://github.com/Rick007110/NeuroMotion/issues/24)). It found that
  imported names like `Bob2: Balance 2.5` don't match our controllers yet
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
- ⬜ The rest of the library, in the inventory's order by scenes using them: Look and
  Reach (11), Explosion (7), JumpDive, Jump and LandCrouch (5), Legs Kick and Arms
  Windmill (4 each), Tackle and Fall Back and Twist (3 each), the arm poses (8 scenes
  between them), then stiffness, leg straightening and velocity damping. First, match
  imported names (`Bob2: ` prefix, version suffix, `BodyWrithe`) so the controllers we
  have run in imported scenes
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
- ⬜ Window and app icon ([#17](https://github.com/Rick007110/NeuroMotion/issues/17)),
  recent files ([#16](https://github.com/Rick007110/NeuroMotion/issues/16))

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

1. **Measure endorphin** (needs someone with endorphin installed): run
   `cargo test --release -- --ignored` (every sample scene, including the real Car
   Hit, [#7](https://github.com/Rick007110/NeuroMotion/issues/7)), the behaviour inventory ([#24](https://github.com/Rick007110/NeuroMotion/issues/24)), and record
   reference measurements of a few scenes. Every later physics and behaviour change
   can then be checked against real numbers.
2. **Back-Flip-To-Combo** ([#6](https://github.com/Rick007110/NeuroMotion/issues/6)): needs the scene file to trace.
3. **Physics robustness**: the arm whip in Car carry ([#23](https://github.com/Rick007110/NeuroMotion/issues/23)) and sideways
   stepping ([#9](https://github.com/Rick007110/NeuroMotion/issues/9)).
4. **Import BVH and FBX** ([#25](https://github.com/Rick007110/NeuroMotion/issues/25)): the first step to people's own motion,
   so NeuroMotion is useful without endorphin scenes.
5. **3D viewport** ([#26](https://github.com/Rick007110/NeuroMotion/issues/26)): the biggest step in how it feels to use.

## Releases along the way

There will be releases before 1.0. Each is a beta (pre-release) with whatever is done
by then:

- **0.1: first public beta.** Downloads for Windows, macOS and Linux. Opens and
  simulates endorphin scenes with five behaviours, sever events, and exports FBX and
  BVH.
- **0.2: physics parity.** Milestone 1 done: every sample scene clean, the first
  reference measurements passing.
- **0.3: behaviours.** The inventory's most used behaviours, sideways stepping,
  behaviour settings in the inspector.
- **0.4: your own motion.** Import of BVH and FBX animation, your own characters, props.
- **0.5–0.9: the editor.** The 3D viewport, editing in the viewport, pose tools.
- **1.0: endorphin level.** Milestones 1–5 done.
