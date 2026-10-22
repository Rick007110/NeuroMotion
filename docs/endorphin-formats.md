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

Tested against every file bundled with endorphin 2.5.2. 160 scenes, characters, rigs
and poses import. Three files don't:

- 2 password-protected tutorial scenes. Their contents are encrypted; re-export them
  from endorphin without a password.
- 1 legacy pose (`BehaviourPoses/dp.nma`), which stores muscle targets instead of joint
  transforms. It will be handled once physics exists.

The sweep runs with `cargo test -- --ignored`. Set `ENDORPHIN_DIR` if endorphin isn't
installed in the default location.

## What gets converted, and how

**Scene settings.** The frame rate is the scene's output rate (usually 60 fps, sometimes
30). The frame range is the scene's playback range; if that is empty, the range is
taken from the events.

**Characters.** Each character, prop and the environment becomes a NeuroMotion
character, with its endorphin name kept (`character01`, `prop01`, `background`) because
events refer to characters by name. Joints keep their real names, offsets and rest
orientations. Virtual-muscle helper joints are left out: they are solver links between
distant body parts, not bones, and the import report says how many were skipped.

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
It isn't used as a muscle target.

**Physics.** Each character's bodies, collision shapes, joint limits and muscle
presets are imported into its physics rig. The scene's simulation rate, gravity,
ground and material table become the project's physics settings. Characters start
*simulated*, as endorphin does; scenery starts as collision-only.

**Every other event** becomes an *event clip* on a track named after its kind:
Forces, Behaviours, Constraints, Simulations, Active poses, Transitions, Motion
transfers, Severs. Their settings are kept on the clip and listed in the inspector under
*Imported settings*. That covers strength, physics level, momentum scale, behaviour
name, impulse matrix, target bodies and constraint strengths. The physics engine
acts on simulation levels, forces, transitions, active poses (whose target pose is
stored as channels on the clip) and constraints. Behaviours named Balance and
CatchFall run NeuroMotion's own controllers (see `behaviours.md`); others are kept
but not simulated.

Overlapping events of the same kind go onto extra tracks ("Animation 2").

## Format notes (from the files themselves)

These are what the importer relies on:

- Files are zlib-compressed UTF-8 XML. An uncompressed file is also accepted.
- An attribute whose name starts with `b` and whose value is 8 hex digits is a float32
  bit pattern (`bstrength="3dcccccd"` = 0.1).
- `TransformMatrix` holds `Vi`, `Vj` and `Vk`, the columns of the rotation, and `Vp`,
  the position. It is always relative to the parent joint.
- Skeletons nest joints in `<ChildJoints>`. Rig templates nest `<RigNode>`s in
  `<ChildNodes>`.
- Animation channels are named `<character>.Joints.<joint>.<quat|tx|ty|tz>`.
  - Key time is in seconds (`bt` on quaternion keys, `bx` on scalar keys).
  - Quaternion keys store w, x, y and z in `bx`, `by`, `bz` and `bw`. Each is the
    joint's full local orientation.
- Event elements live in `Events`, `ForceEvents`, `SecondaryBehaviours`,
  `TertiaryBehaviours` and `SeverBehaviourEvents`. Each has `startFrameNumber`,
  `endFrameNumber`, `priority`, `enabled` and `character`.

The conventions were checked by forward kinematics on real data. With these
conventions, the standard character's rest pose is a T-pose with its feet on the ground
at y ≈ 0. A walk frame stands upright with the head at about 1.8 m. The other
interpretations put limbs at impossible angles.
