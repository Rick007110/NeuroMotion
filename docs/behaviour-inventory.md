# Behaviour inventory

Which behaviours endorphin 2.5.2's own scenes use, how often, with which settings, and
which of them NeuroMotion can already run. It is the plan for the rest of the library
([ROADMAP.md](../ROADMAP.md), milestone 2; issue
[#24](https://github.com/Rick007110/NeuroMotion/issues/24)).

It comes only from the scenes (`.ens`) of a local install: the behaviour events on
their timelines and the settings those events carry. endorphin's protected behaviour
files were not read, so this says what scenes ask for, not how endorphin does it.

The numbers below are from the 65 scenes that import (of 67; two are
password-protected): the sample scenes, the one-scene-per-behaviour demos and the
tutorial scenes. Demo scenes count like any other, so a behaviour with its own demo
scene always shows at least one scene.

## Event types

Every event in every scene, by endorphin's type:

| Type | Events | Scenes | What NeuroMotion calls it |
|---|---:|---:|---|
| `SecondaryBehaviour` | 135 | 53 | Behaviour (this page) |
| `TertiaryBehaviour` | 111 | 36 | Constraint |
| `ForceEvent` | 79 | 23 | Force |
| `DesiredPoseEvent` | 67 | 15 | Active pose |
| `IncarnationEvent` | 55 | 20 | Simulation switch |
| `TransitionEvent` | 7 | 5 | Transition |
| `RigEvent` | 6 | 5 | Rig |
| `SeverEvent` | 2 | 2 | Sever |

Behaviours appear in 53 of the 65 scenes and constraints in 36. The
rest of this page is about the 135 behaviour events.

## Behaviours

Grouped by family: endorphin's names carry a library prefix and a version
(`Bob2: Balance 2.5`), and the 2.0 and 2.5 versions of a behaviour take the same
settings, so they are counted together. Sorted by scenes, then events. "Have" means
`Behaviour::from_name` knows the family name, so imported events of that family run
our controller (see [the name problem](#the-name-problem)).

| Family | Scenes | Events | Have | Settings the events carry (besides `index`, `priority`, `targets`) |
|---|---:|---:|:---:|---|
| Look and Reach | 11 | 19 | yes | Per hand (`LeftHand_`, `RightHand_`): Strength 0.6-1, Speed 0.3-1, Orientation 0-1, Blend, Delay, DistanceScale 0.1-0.5, MaxExtension 0.9-1, CatchOffset X/Y/Z (to -10 m), UsePrediction, PredictionRate, applyHandOrientation, FingerAngle; look-at: LookAtSpeed 0.5-1, Blend, Delay, HeadUpVector, UseUpVector; LegTorsoStiffness 0.8. The target is chosen in the scene (a body or collision surface), not a number |
| BodyWrithe | 10 | 14 | yes (our `Writhe`) | Amplitude 0.1-0.6, Frequency -0.8-0.6, Strength 0.2-1, PhaseOffset 0-0.2, NoiseLevel, Seed (98, 111), ApplyHipOpening false |
| Stagger | 7 | 11 | yes | MaxSteps 5 or 50, ArmsWindmill, StaggerSquarely, PredictGroundHeight, GroundHeight (default, 0, 3), ArmsSpeed 0.6-0.9, ArmsStrength 0.6-1 |
| Explosion | 7 | 10 | yes | Speed 0.3-0.8, Strength 0.5-1, BackArch 0-1, HeadTurn 0.6, ShoulderAmplitude 0.3-0.8, ElbowAmplitude 0.3-0.6, LegAmplitude 0.3-1, PhaseOffset 0 |
| BodyFoetal | 6 | 8 | yes | ArmSpeed 0.08-0.7, ArmStrength 0.5-1, LegSpeed 0.3-0.8, LegStrength 0.5-1, TorsoStrength 0.5-1 |
| CatchFall | 6 | 7 | yes | ArmsApart 0.5-0.7, ArmsSpeed 0.7-1, ArmsStrength 0.7-1, PredictGroundHeight, GroundHeight (default, 0) |
| JumpDive | 5 | 6 | no | JumpTiming 0.3-0.9, Strength 0.3-1 |
| Balance | 4 | 7 | yes | pelvisHeight 0-1, useArms true, failureThreshold 0.15-0.2, desiredCOM_pitchOffset 0-0.09, desiredCOM X/Z offsets (default) |
| LegsKick | 4 | 6 | no | Frequency 0.3-0.8, HipAmplitude 0.6-0.8, KneeAmplitude 0.6, Strength 0.5-0.8, NoiseLevel 0.5, Seed 333, ArmStiffness 0.2, TorsoStiffness 0.5 |
| Arms Windmill | 4 | 5 | no | Frequency 0.4-0.85, ShoulderAmplitude 0.6, ElbowAmplitude 0.4-0.8, Strength 0.5, SyncOffset 0.8, yxPlaneOffset 0.1, zxPlaneOffset, LegTorsoStiffness 0.5 |
| Tackle | 3 | 3 | no | ArmsSpeed 0.6-1, ArmsStrength 0.6-1, ReachFwdDuration 0-0.07, TackleHeight, WidenArmsDuration, PredictGroundHeight false, LegTorsoStiffness 0.5; targets pelvis and upper spine of another body |
| Fall Back, Twist to Land on Front (Arms to Head, or Catch Fall variants) | 3 | 6 | no | TwistUrgency 0.3-0.8, ArmsSpeed 0.3-0.8, ArmsStrength 0.5-0.8, ArmsApart 0.6 (catch variant), LegTorsoStiffness 0.5 |
| ArmsToHead | 2 | 3 | no | Speed 0.3-0.8, Strength 0.5, NoiseLevel 0.2, Seed 222, LegTorsoStiffness 0.5 |
| LegsStraighten | 2 | 3 | no | Per leg: Speed 0.3-0.8, Strength 0.6-0.8, Straighten on/off; ArmTorsoStiffness 0.3 |
| MetaBodyVelocityDamping | 2 | 3 | no | Linear and angular damping for each body part, 0, 5, 10 or 50 |
| ArmsCrossedOnChest | 1 | 2 | no | Speed 0.3-0.8, Strength 0.6, NoiseLevel, Seed, LegTorsoStiffness 0.5 |
| ArmsSurrender | 1 | 2 | no | Speed 0.3-0.8, Strength 0.5, NoiseLevel 0.17, Seed, LegTorsoStiffness 0.5 |
| ArmsToGroin | 1 | 2 | no | Speed 0.3-0.8, Strength 0.5, LegTorsoStiffness 0.8 |
| ArmsWideOfHead | 1 | 2 | no | Speed 0.3-0.8, Strength 0.5, NoiseLevel, Seed, LegTorsoStiffness 0.5 |
| ArmsZombie | 1 | 2 | no | Speed 0.3-0.6, Strength 0.6, NoiseLevel, Seed, LegTorsoStiffness 0.5 |
| FreeHandsBehindBack | 1 | 2 | no | Speed 0.3-0.8, Strength 0.4, HipWidthScale 0.4-0.8, LegTorsoStiffness 0.5 |
| BodyStiffness | 1 | 2 | no | Stiffness 0.3-0.8 for head and neck, each arm, each leg and torso |
| BodyStiffnessFull | 1 | 2 | no | One Stiffness, 0.3-0.8 |
| FootReach | 1 | 2 | no | Like Look and Reach, per foot; ArmTorsoStiffness 0.8 |
| Jump | 1 | 2 | no | JumpTiming 0.3-0.8, Strength 0.3-0.8 |
| LandCrouch | 1 | 2 | no | ArmsUrgency 0.8, LegStrength 0.5, GroundHeight (default) |
| WritheInMidAir | 1 | 2 | yes (Explosion) | Like BodyWrithe, plus Speed 0.6, BackArch 1, HeadTurn 0.6. endorphin 2.5's Explosion: both show as "Writhe in mid-air" |

Reading the settings:

- Strengths, speeds and stiffnesses are fractions from 0 to 1. Many events take the
  scene's value, so the range shown is what the scenes use, not what is allowed.
- `Seed` and `NoiseLevel` appear on the rhythmic and pose behaviours (writhe, kick,
  the arm poses). A seed is fixed in the scene, so these can stay deterministic.
- Nearly every event also has a `priority` (0-7, mostly 0 or 1) that orders
  overlapping behaviours, and an `index` that identifies the behaviour in endorphin's
  own library; NeuroMotion needs neither for the controllers it has.
- Reach, look-at and tackle behaviours take a target chosen in the scene (a body,
  a collision surface or a prop). The importer keeps the selection as the setting's
  value, `character.MetaBodies.Body` or a collision surface's path
  (`character.root.<joints>.Body.Surface`), comma separated; Look and Reach finds it in
  the world ([behaviours.md](behaviours.md#look-and-reach)).
- `MetaBodyVelocityDamping` is not a controller but a set of damping values per body
  part. It maps to the rig's damping rather than to muscles.

### The name problem

Scenes name behaviours with a library prefix and often a version: `Bob2: Balance`,
`Bob2: Balance 2.5`, `Bob2: CatchFall 2.0`, `Bob2: BodyWrithe 2.0`. `Behaviour::from_name`
ignores case, spaces and punctuation but not words, so it turns `Bob2: Balance 2.5`
into `bob2balance25` and found nothing: no imported behaviour event matched a
controller, Balance, CatchFall, Stagger and BodyFoetal included; only the stand-in
scenes, which use bare names, reached them. Likewise `BodyWrithe` is our `Writhe` under
another name. This inventory found it, and it is fixed: `from_name` now drops the part
up to the last `:` and a trailing version, and accepts `BodyWrithe`. The "Have" column
is what imported scenes now run.

The parameters (the settings above) are now imported with the event: each `Variable`
of its `Parameters` becomes a setting of the same name
([endorphin-formats.md](endorphin-formats.md#behaviour-parameters)). Look and Reach and
Explosion read theirs; the other controllers don't read theirs yet (Stagger's, for
one). A setting a scene leaves at endorphin's default carries no value and isn't
listed here.

## Which missing behaviours matter most

By number of scenes that use them, counting families we do not have yet:

1. **Look and Reach**, 11 scenes, 19 events: by far the most used. Done: its targets
   and settings are imported and it runs NeuroMotion's own controller, compared with
   endorphin's run of "Hands reach and look at"
   ([behaviours.md](behaviours.md#look-and-reach)). Foot Reach (1 scene) is the same
   idea for a foot and could share it.
2. **Writhe, under its scene name** (BodyWrithe, 10 scenes; WritheInMidAir is
   Explosion's 2.5 name, below): the names match our Writhe now; its extra settings
   (Seed, Frequency, ApplyHipOpening) aren't read yet.
3. ~~**Explosion**~~, 7 scenes (8 with WritheInMidAir, its 2.5 name): done, an
   arms-flapping flail in mid-air with per-limb amplitudes, BackArch and HeadTurn,
   matched to a recording of endorphin's behaviour scene
   ([behaviours.md](behaviours.md#explosion-writhe-in-mid-air)).
4. **JumpDive** (5 scenes) and **Jump** (1) with **LandCrouch** (1): takeoff timing
   and landing. Need ground prediction, as Stagger's PredictGroundHeight does.
5. **LegsKick** (4) and **Arms Windmill** (4): rhythmic limb motion with
   amplitude, frequency and phase. One design can cover both, and Windmill's
   settings already show up as Stagger's arms. Cheap once the rhythm generator exists.
6. **Tackle** (3 scenes) and the combined **Fall Back, Twist to Land on Front**
   (3 scenes): both build on CatchFall and Balance plus a target or a twist.
7. **The arm and hand poses**: ArmsToHead, ArmsCrossedOnChest, ArmsSurrender,
   ArmsToGroin, ArmsWideOfHead, ArmsZombie and FreeHandsBehindBack. Eight scenes in
   all, all with the same settings (Speed, Strength, stiffness of the rest of the
   body, a little noise). They are poses held by the arms, so they can share one
   design with the pose as a table.
8. **BodyStiffness** and **BodyStiffnessFull** (1 scene each), **LegsStraighten** (2)
   and **MetaBodyVelocityDamping** (2): simple muscle-strength and damping
   switches, quick to add.

The behaviours we do have are well covered by scenes: Stagger (7), BodyFoetal (6),
CatchFall (6) and Balance (4), so their remaining work is matching names and reading
their settings.

## Re-running it

The inventory is an `#[ignore]`d test next to `imports_every_bundled_file`. It needs a
local endorphin 2.5.2 install (set `ENDORPHIN_DIR` if it is not in the default
location), imports every scene, skips the password-protected ones, and prints the
event types, the behaviour names as they appear and the families with their scenes and
settings:

```bash
ENDORPHIN_DIR="/path/to/endorphin 2.5.2" \
  cargo test --release behaviour_inventory -- --ignored --nocapture
```

The "Have" column is `Behaviour::from_name`, so the output changes as controllers are
added. Re-run it after adding a behaviour or changing name matching, and update this
page.
