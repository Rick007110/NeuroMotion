# Opening endorphin files

NeuroMotion is meant to take over from endorphin, so it opens endorphin's own files
directly: **File → Open**, drag-and-drop onto the window, or pass a path on the command
line (`neuromotion.exe "Car Hit.ens"`). Because of the command-line path, Windows
"Open with…" works too. Imported files are saved as NeuroMotion projects (`.nmproj`);
NeuroMotion never writes endorphin files.

| Extension | What it is | Support |
|-----------|-----------|---------|
| `.ens` | Scene | **Opens.** All characters, props and the environment; animation as editable keys; every timed event on the timeline |
| `.nmc` | Character, prop, or motion-transfer rig | **Opens** as a project with that character and an empty track |
| `.nma` | Pose | **File → Import Pose** (or drop it on the window): placed on the matching character at the playhead as a 30-frame held clip |
| `.nmb` | Behaviour | Not readable: endorphin stores these in a protected format |
| `.nmw` | World settings | Not readable: protected format |

Tested against every file bundled with endorphin 2.5.2 (`imports_every_bundled_file`):
of its 161 `.ens`, `.nmc` and `.nma` files (67 scenes, 42 characters, props and rigs,
52 poses), 159 import, the legacy muscle-target pose `BehaviourPoses/dp.nma` included (see
[Muscle-target poses](#muscle-target-poses)). Two files don't: password-protected
tutorial scenes. Their contents are encrypted; re-export them from endorphin without a
password.

The sweep runs with `cargo test -- --ignored`. Set `ENDORPHIN_DIR` if endorphin isn't
installed in the default location.

## What gets converted, and how

**Scene settings.** The frame rate is the scene's output rate (usually 60 fps, sometimes
30). The frame range starts at the scene's playback range start and ends at the later
of its playback range end and the end of the run endorphin simulates: the engine's
`numTicksToRun` physics ticks at its simulation rate, converted to frames
(`ticks / simulation rate * fps`). A scene that sets no run length carries
`numTicksToRun="10000"`, which is the default and not a length (83 s at 120 Hz), so
those scenes keep their playback range. If the playback range is empty, the range is
taken from the events. The engine block also holds a viewable range (always 0-0 in the
samples) and a save range (always equal to the playback range, except Bridge Escape,
which saves from frame 0); they aren't used.

The playback range is only what the timeline's loop shows; endorphin simulates and plays
on to the end of the run. Car Hit's character finishes standing up between frames 162
and 200, so a range cut at the playback end loses the last 0.63 s of it. Range by sample
scene (old: playback range; new: with the run length; fps and ticks as stored):

| Sample scene | `numTicksToRun` | Old range | New range |
|---|---|---|---|
| Arrows | 10000 (default) | 0-178 | 0-178 |
| Balcony | 10000 (default) | 0-302 (from the events: playback range empty) | 0-302 |
| Bendy Stick | 10000 (default) | 0-24 | 0-24 |
| Bridge Escape (Large File) | 1256 | 1-618 | 1-628 |
| Car Crash | 640 | 0-119 | 0-160 |
| Car Hit | 400 | 0-162 | 0-200 |
| Catch Rope | 10000 (default) | 0-322 | 0-322 |
| Catch Run | 10000 (default) | 0-148 | 0-148 |
| Gymnast Rings | 1200 | 0-500 | 0-600 |
| Heli Leap | 10000 (default) | 0-88 | 0-88 |
| Multi Martial | 850 | 2-228 | 2-425 |
| Rotating Floor | 10000 (default) | 0-370 | 0-370 |
| Stagger Walk | 600 | 0-208 | 0-300 |
| Sword Play | 440 | 0-160 | 0-220 |
| Tackle | 250 | 0-104 | 0-125 |
| Trapeze | 1200 | 0-488 | 0-600 |
| Walk With Gun | 600 | 0-218 | 0-300 |

Seen on endorphin 2.5.2's sample scenes (Bendy Stick, Car Crash and Heli Leap run at 30 fps, the rest at 60; all simulate at 120 Hz). Car Crash's 640 ticks are 160 frames at 30 fps. The two password-protected tutorials aren't readable.

**Characters.** Each character, prop and the environment becomes a NeuroMotion
character, with its endorphin name kept (`character01`, `prop01`, `background`) because
events refer to characters by name. Joints keep their real names, offsets and rest
orientations. Virtual-muscle helper joints are left out: they are solver links between
distant body parts, not bones, and the import report says how many were skipped.
Each character's own `TransformMatrix` is where it sits in the world (Catch Rope's
runner starts on a roof 15.8 m up, the gymnast hangs 1.7 m up); see *Starting poses*.

**Animation events** become motion clips on an *Animation* track:

- Event start and end frames become the clip's start and duration.
- `frameOffset` shifts the keys.
- The entry blend period becomes the clip's blend-in.
- Per-joint quaternion keys are converted to XYZ Euler curves relative to each joint's
  rest orientation. Angles are unwrapped so curves don't jump at ±180°.
- Translation keys become offset curves.
- The event's placement matrix (where the animation sits in the scene) is baked into
  the root joint.

**Starting poses.** Each character's placement and pose at the start of the scene
(the scene's `<Poses>` section) becomes a held *Start pose* clip on the bottom track.
The pose's root is relative to the character's placement, so the two are combined.
It isn't used as a muscle target.

**Scene animation.** The scene's own `<Animation>` section keys characters outside
any event: a prop driven along a path (Car Hit's car, Car Crash's carrier box), a
rotating floor, a run-up, poses set by hand. Its channels are named after the
character they key, and are relative to its placement like the start pose. Each
character with keys gets one held *Scene animation* clip for the whole scene, above
its start pose and below its animation events; it isn't a muscle target either.
Camera channels are left out.

**Physics.** Each character's bodies, collision shapes, joint limits and muscle
presets are imported into its physics rig. A collision surface with no geometry of
its own collides with the body's mass shape: that is how props ("environment mass
objects have a collision object by default"), ropes and other rigs built in endorphin
get collision. The scene's simulation rate, gravity,
ground and material table become the project's physics settings. Characters and the
environment start *simulated*, as endorphin does. The environment's pieces without an
inertial body (the background) have no mass and stay fixed; its props with one are
real objects that fall, swing and get pushed, which is why scenes lock or join props
to hold them, or switch the environment to NoPhysics (Hands Reach and Look At, whose
reach target is a prop).
A joint's `SoftLimits` and `HardLimits` give full widths: `twistSize` about the limit
frame's X axis, `swing1Size` for the turn about its Z axis and `swing2Size` about its Y
axis (the active poses endorphin's scenes store sit inside the cones read this way, and
far outside them read the other way: physics-design.md, lesson 27).
Imported scenes also get Coulomb friction (lesson 21) and the whole-limb muscle cap.

**Every other event** becomes an *event clip* on a track named after its kind:
Forces, Behaviours, Constraints, Simulations, Active poses, Transitions, Motion
transfers, Severs. Their settings are kept on the clip and listed in the inspector under
*Imported settings*. That covers strength, physics level, momentum scale, behaviour
name and settings (see [behaviour settings](#behaviour-parameters)), impulse matrix,
target bodies and constraint strengths. The physics engine acts on simulation levels,
forces, transitions, active poses (whose target pose is stored as channels on the
clip) and constraints. Behaviours named Balance, Stagger, CatchFall, BodyFoetal,
BodyWrithe, Look and Reach and Explosion (Writhe in mid-air) run NeuroMotion's own
controllers (see `behaviours.md`); others are kept but not simulated.

Overlapping events of the same kind go onto extra tracks ("Animation 2").

## Muscle-target poses

<a id="muscle-target-poses"></a>One bundled pose, `BehaviourPoses/dp.nma` (made for
endorphin's older `character01`), stores muscle targets instead of joint transforms:
a `<DesiredPose>` with one `<EffectorParameters name="LKneeZ">` per single-axis
effector, each holding a `<DesiredAngle>` in radians. No scene refers to it, and no
other bundled file stores targets this way (`DesiredPose` elsewhere is only the
`DesiredPoseEvent` of active poses). It imports like any pose: a held clip on the
character, which an active pose event then drives the muscles towards.

**What's in the file.** Besides the targets, the pose carries its own skeleton
(`<Skeleton>`): the pelvis, then a chain of single-axis `<Hinge>`s per joint, named
`<parent body>_<child body>` (`LUpLegYMB_LKneeZMB`), each with its frame in the world
(`refFrame="World"`), at rest; `<MetaBodies>`, the bodies' world frames, also at rest;
and `<Relationships>`, reach targets (`RPROMagnet`) the importer doesn't use. The
posed result isn't stored anywhere, so the convention below comes from the skeleton
and the targets themselves:

- **Which hinge.** An effector drives the hinge into the body named after it
  (`LKneeZ` → `…_LKneeZMB`). The hip's third axis has no body of its own: `LHipY`
  turns the thigh, the hinge `LHipZMB_LUpLegYMB`.
- **About which axis.** A hinge turns about the first column (`Vi`) of its frame.
  That column is always the axis the effector's letter names, in the character's
  own terms: for every `…X` effector the body's forward axis (roll), for every `…Y`
  one the bone's length (twist: `LUpLegY`'s `Vi` runs down the thigh, `LLoArmY`'s
  along the forearm), for every `…Z` one the side axis (bend).
- **Which way.** Right-handed about `Vi`, from the rest frame at zero. With that sign
  every non-zero target of dp.nma bends its joint the way the joint bends: knees
  -0.99 rad bend backwards, hips (0.64) and elbows (0.87) forwards, the spine
  (0.80 + 0.60) forwards, ankles (-0.33) point the toes down, and the hips' `X` and
  `Y` targets, mirrored left and right, spread the legs symmetrically. With the
  opposite sign the knees would bend 57° forwards and the elbows 50° backwards.
  The pose is a deep crouch, hunched forwards, arms raised in front.

**Onto a character.** None of the bundled characters has dp.nma's skeleton (endorphin
2.x characters have one three-axis joint where it has two or three hinges), so the
importer maps it, in `io/endorphin.rs`:

1. A hinge's target goes to the character's joint of the same name if it has one,
   else to the joint whose muscle is the effector (`LKneeZ`) or its joint (`LKnee`).
   Joints later characters merged are renamed: the outer shoulder (`LOutShoulder`)
   and the upper arm's twist (`LUpArm`) are the shoulder's (`LShoulder`), the
   forearm's twist (`LLoArm`) is `LLoArmTwist` (else the elbow), the shin's twist
   the knee's and the thigh's the hip's. All of a joint's hinge turns combine,
   parent first, into one rotation.
2. The pose faces +X, right side towards +Z; it's turned about the vertical so a
   left-right pair of its joints lines up with the character's.
3. Each turn is about the hinge's axis in the world at rest, converted into the
   character's joint frame, so it doesn't matter how the character's own joint
   axes are set up.
4. Where the character rests differently from the pose's skeleton by more than
   0.5 rad (a T-pose's arms against dp.nma's hanging ones), the joint is first turned
   to the pose's rest by the shortest turn, which its children inherit, so the limb
   ends up where the pose puts it. Smaller differences are left alone: the Standard
   Simulation Character's straight forearm comes out about 12° from dp.nma's, which
   rests its forearm bent 12° forwards.

On the Standard Simulation Character every target but `BaseNeckZ` (zero, and on no
hinge) lands on a joint; the shoulders are turned down from its T-pose first. The
import's notes say what was left out or turned. Held as an active pose with the hips
locked, the character's knees reach the pose to within 2 cm and its feet to within
5 cm. Its spine bends far less than the pose's: dp.nma bends the lower spine 46° and
the upper 34° on two joints, past the Standard character's spine limits (19-24°),
which muscle targets are kept inside, and its middle spine joint gets no target. The
head ends up 0.8 m from where the pose puts it, and the arms, which hang from the
spine, follow it. Spreading a bend over the character's extra joints is still to do.

## Format notes (from the files themselves)

These are what the importer relies on:

- Files are zlib-compressed UTF-8 XML. An uncompressed file is also accepted.
- An attribute whose name starts with `b` and whose value is 8 hex digits is a float32
  bit pattern (`bstrength="3dcccccd"` = 0.1).
- `TransformMatrix` holds `Vi`, `Vj` and `Vk`, the columns of the rotation, and `Vp`,
  the position. It is always relative to the parent joint.
- Skeletons nest joints in `<ChildJoints>`. Rig templates nest `<RigNode>`s in
  `<ChildNodes>`.
- Animation channels are named `<character>.Joints.<joint>.<quat|tx|ty|tz>` (or
  `<character>.<joint>.<...>`, mostly in the scene's own `<Animation>`).
- A character's `TransformMatrix` places it in the world; its start pose's root (in
  `<Poses>`, `refFrame="Parent"`) and its keys in the scene's `<Animation>`
  (`refFrame="parent"`) are relative to it. An animation event's matrix places that
  event's animation in the world. Car Crash's carrier box shows the composition is
  real: placed 14.4 m along Z, its start pose sits 14.4 m back along Z (it ends up
  below the ground, where it isn't simulated).
  - Key time is in seconds (`bt` on quaternion keys, `bx` on scalar keys).
  - Quaternion keys store w, x, y and z in `bx`, `by`, `bz` and `bw`. Each is the
    joint's full local orientation.
- Event elements live in `Events`, `ForceEvents`, `SecondaryBehaviours`,
  `TertiaryBehaviours` and `SeverBehaviourEvents`. Each has `startFrameNumber`,
  `endFrameNumber`, `priority`, `enabled` and `character`.
- <a id="behaviour-parameters"></a>**Behaviour settings** (`SecondaryBehaviour`): a
  behaviour event names its behaviour in `behaviourName` (`Bob2: Explosion 2.0`) and
  shows `displayName` (`Writhe in mid-air 2.0`). Its settings are in its `Parameters`:
  one `Variable` per setting, its name in `fk` and its value in `ja`
  (`LeftHand_Speed` = `0.700`; `de`, when present, is the name endorphin shows).
  `kg` gives the kind: 0 a number, 1 a whole number, 2 a flag (`true`/`false`), 5 a
  selection or a helper graphic. A variable without `ja` keeps endorphin's default,
  which the scene doesn't record. A target is a selection,
  `ja="MetaBodyAndCollisionSelection"`, listing what was picked as `ValuePtr`s: a body
  (`character02.MetaBodies.HeadMass`) or a collision surface by its path through the
  skeleton (`character02.root.LowerSpineJoint....HeadMass.Sphere1`: the body, then the
  surface's name). `ja="HelperGraphic"` variables are the locators endorphin draws
  (target crosses, look-at lines), not settings. The importer keeps every setting with
  a value under its `fk` name, and a selection as its picks, comma separated; it
  leaves out defaults, empty selections and helper graphics, and a name the event's
  own attributes already use keeps the attribute. Behaviours read their settings from
  there through one reader (`behaviour::setting`): Look and Reach
  ([behaviours.md](behaviours.md#look-and-reach)), where a surface target resolves to
  its body, and to the surface's centre only when the body has a single shape, since
  NeuroMotion doesn't keep shape names; and Explosion
  ([behaviours.md](behaviours.md#explosion-writhe-in-mid-air)).
- <a id="sever-events"></a>**Sever events** (`SeverEvent`, in `SeverBehaviourEvents`) are
  **a best guess**: we have no scene containing one. The importer keeps every
  attribute and its `Objects` (as `targets`), and the simulation reads `severType`
  (`Cut` or `MuscleOff`; anything else cuts) and `targets` (`character.MetaBodies.Body`,
  the body whose joint to its parent is cut), the convention of the other events. endorphin's
  own names for the type and the target may differ. Then an imported sever
  event is kept and shown, but it cuts every joint it names that resolves (a missing type
  means cut) or, if no target resolves, does nothing. To verify
  against a real scene: look at the clip's *Other imported settings* in the inspector
  and rename the keys in `physics/sim.rs` and `document/events.rs`.
- Active pose events (`DesiredPoseEvent`) store a whole-body `Pose` (every joint,
  relative to its parent), a `strength`, an entry blend (`bentryBlendPeriod`,
  seconds) and `Objects`, the joints the event drives (`character.Joints.Joint`).
  Scenes split the body between overlapping events: Back-Flip-To-Combo drives its
  legs, feet, spine and arms from separate events with their own strengths. The
  importer keeps `Objects` as `targets`, and the simulation drives only those joints
  (physics-design.md, lesson 19).
- `CollisionSurface`s carry `collidesWithParent`, `collidesWithSiblings` and
  `collidesWithAncestors`. Back-Flip-To-Combo's head sets `collidesWithParent` on
  shapes that sit inside the neck at rest; NeuroMotion doesn't let a shape collide
  with a moving parent it already overlaps at rest (lesson 19).
- Constraint events give a strength per axis for position and for orientation:
  the most force (N) or torque (N·m) the lock may use on that axis, as Karma's
  relative-position-relative-orientation joint has; 0 leaves the axis free. The
  values the scenes use read sensibly as forces on a human or a car: 25 N toe locks,
  a 500 N ring hold, 100-1000 N·m "guides" to turn a character, and 1e5, which
  carries a 747 kg car at up to 134 m/s² (13.6 g) and is effectively unbreakable for
  a character. A lock that can't hold pulls towards its target with all of it
  (`physics-design.md`, lesson 20). The files don't say how fast endorphin closes
  a gap; that a strong join follows its keyed carrier, as Car Hit's car does, is
  what was checked.
  endorphin's help doesn't document negative strengths; scenes use them as drives.
  Back-Flip-To-Combo spins its character into the flip with an orientation strength
  of (-100, 0, 0) on pelvis and spine. NeuroMotion reads a negative axis as a steady
  force or torque of that value along the axis (world axes for locks, the anchor's
  for joins), on the locked body and the opposite on its anchor, instead of a hold.
- `MaterialInteraction`s give `friction`, `restitution`, `maxAdhesiveForce` (N) and
  `softness`; `MaterialPair`s name the interaction for two materials. The friction
  value is a Coulomb coefficient × 100: the sample scenes use 1 (Icy), 100
  (Default), 300 (Grippy, rubber on tarmac) and 700 (Sticky, with 700 N of
  adhesion), and Car Hit only behaves as endorphin's does read that way
  (`physics-design.md`, lesson 21). NeuroMotion used to read it as a force cap per
  contact in newtons.
- A body's mass is its inertial shape's volume times its density (g/cm³). A
  character's body can come out with none: Back-Flip-To-Combo's NormalMass is a
  zero-sized Sphyl. Only the environment's massless pieces stay fixed in place;
  a character's or prop's get a token 1 kg, or one fixed body would hold the whole
  character still.

The conventions were checked by forward kinematics on real data. With these
conventions, the standard character's rest pose is a T-pose with its feet on the ground
at y ≈ 0. A walk frame stands upright with the head at about 1.8 m. The other
interpretations put limbs at impossible angles.
