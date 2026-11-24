# Physics events

Events are clips that tell the physics what to do and when: go limp, take a push, hold
a pose, return to the animation. They sit on a character's tracks next to its motion
clips and are drawn as outlined bands. They don't change the keyed animation itself.

## Adding events

- **Right-click a track lane** on the timeline: the menu adds an empty motion clip or any
  event at the clicked frame, on that track. The playhead moves there too.
- **Edit → Add**: adds at the playhead for the selected character (or the first one).
  With a track selected it goes on that track; otherwise each kind gets its own track
  (Simulations, Forces, Transitions, Active poses, Constraints, Severs), created when
  needed.

Events need a character with a physics body; for others the menu items are disabled.
Every addition is one undo step.

Tracks: right-click a character's name for **Add Track**, a track's name or lane for
**Add Track** / **Delete Track**. The character inspector also has **Add track**, the
track inspector **Delete track**, and Delete removes the selected track with its clips.
Right-click a clip to duplicate, disable or delete it.

## The events

Select an event to edit it in the inspector. All changes are undoable; a drag is one
step.

| Event | What it does | Settings |
|---|---|---|
| **Simulation** | At its first frame, switches the character's simulation level. It stays there until the next simulation or transition event. | *Switch to*: Animated, Collision only, Collision with momentum, Simulated. *Momentum kept* (0–1): how much of the animation's velocity carries into physics. |
| **Force** | A one-frame push at its first frame: a sudden change of velocity on one body (or every body). Only acts while the character is simulated. | *Strength* (m/s), *Heading* (degrees about the up axis: 0° = +Z, the way NeuroMotion's characters face at rest; 90° = +X; 180° = −Z), *Elevation* (degrees above horizontal), *Body*. New forces push the heaviest body (usually the torso) backwards at 3 m/s. |
| **Transition** | Blends from the simulation back to the animation over the clip, then switches level. The blend weight goes from 0 at the first frame to 1 at the last, so the drawn pose is the animation from the last frame on (the simulated bodies switch one frame later). | *Then*: the level at the end (Animated by default). |
| **Active pose** | While simulated, the character's muscles pull towards a pose. Overlapping active poses each drive their own joints (an imported one only the joints it names); where two drive the same joint, the one that started last wins once it has eased in. | *Strength* (multiplies muscle strength), *Ease in* (seconds to move the muscles over from what they were doing), and the pose itself: captured from what the viewport shows at the playhead when the event is added; **Capture pose at playhead** re-records it. |
| **Constraint** | From its first frame to its last, holds bodies: each where it was when the clip started, or all of them to the last body listed. | *Type*: Lock in place, Lock position only, Lock orientation only, Join bodies together. *Position* / *Rotation strength*: the most force (N) or torque (N·m) the hold may use along each axis (the world's for a lock, the anchor's for a join); a hold that can't keep up pulls with all of it (physics-design.md, lessons 5 and 20); 0 leaves an axis free, and a negative value drives instead (lesson 13). *Bodies*: any body of any character. |

### Sever

From its first frame to the end of the scene, the joint between the target body and
its parent body is cut, or only its muscle is switched off (*Type*). *Joint of* picks
the body whose joint to its parent is meant: a forearm body cuts the elbow. The root
body has no joint, so targeting it does nothing.

- **Cut the joint**: the joint stops acting altogether (anchor, limits, soft limits and
  muscle), so the two parts move on separately. The body's parts below it, the hand
  say, stay attached to it. A cut body no longer counts as its old parent's child, so
  it collides with it like any unjointed body, instead of being excluded as joined
  neighbours are. (This is our choice: we don't know which of those endorphin does.)
- **Muscle off**: the joint, its limits and its collisions stay; the muscle no longer
  pulls, so the part hangs limp from the joint.

It acts while the character is simulated. The simulation restarts from the first
frame on every edit and works out what is severed at each frame from the events
themselves, so removing, moving or disabling the event takes effect on the next run.
The viewport and exports show the severed part where its body is: the joint follows its
own body, so exports key a translation on it. In an export the bone between the cut
joints stretches, as for any simulated joint that comes away.

The endorphin key names for sever events are a best guess; see
[endorphin-formats.md](endorphin-formats.md#sever-events).

Imported endorphin behaviours show their behaviour name. Balance and CatchFall run
NeuroMotion's own controllers ([behaviours.md](behaviours.md)); the others aren't
simulated yet.
Settings an event carries that the controls don't cover (imported from endorphin) are
listed read-only under **Other imported settings**.

A typical knock-down: a Simulation event (Simulated) and a Force event on the same
frame, then optionally a Transition a second or two later to get back to the keys.

## How they're stored

Settings live in the clip's `source` map under the same keys endorphin scenes are
imported with, which is also what the simulation reads, so created and imported
events behave identically and project files stay format v2:

| Event (`kind`) | Keys |
|---|---|
| `Simulation` | `physicsLevel` (`NoPhysics`, `CollisionOnly`, `CollisionOnlyWithInertia`, `FullPhysics`), `momentumScale` |
| `Force` | `matrix.Vi` (`"x, y, z"` velocity change, m/s), `targets` |
| `Transition` | `requiredPhysicsLevel` |
| `Active pose` | `strength`, `entryBlendPeriod` (seconds), `targets` (the joints it drives as `character.Joints.Joint`; none = all); the pose is the clip's channels (one key each), and a driven joint without a channel is held at rest |
| `Constraint` | `lockType` (`Lock`, `LockPosition`, `LockOrientation`, `JoinBodies`), `targets`, `positionStrength`, `angularStrength` (`"x, y, z"`) |
| `Behaviour` | `behaviourName` |
| `Sever` | `severType` (`Cut`, `MuscleOff`), `targets` (guessed, see below) |

`targets` lists bodies as `character.MetaBodies.Body` (joints, for active poses),
comma separated. A force with no targets pushes every body of its character. A force
pushes the bodies it names whichever character's track holds it (Catch Rope keeps its
push on the rope on the environment's track), and one naming only bodies that don't
exist pushes nothing.

`src/document/tests.rs` rebuilds the physics demo's events through the editor's own
operations and checks the simulation is identical to the built-in demo's, and that
transitions, active poses and constraints change the simulation as described.
