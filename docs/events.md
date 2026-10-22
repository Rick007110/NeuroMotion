# Physics events

Events are clips that tell the physics what to do and when: go limp, take a push, hold
a pose, return to the animation. They sit on a character's tracks next to its motion
clips and are drawn as outlined bands. They don't change the keyed animation itself.

## Adding events

- **Right-click a track lane** on the timeline: the menu adds an empty motion clip or any
  event at the clicked frame, on that track. The playhead moves there too.
- **Edit → Add**: adds at the playhead for the selected character (or the first one).
  With a track selected it goes on that track; otherwise each kind gets its own track
  (Simulations, Forces, Transitions, Active poses, Constraints), created when needed.

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
| **Transition** | Blends from the simulation back to the animation over the clip, then switches level. | *Then*: the level at the end (Animated by default). |
| **Active pose** | While simulated, the character's muscles pull towards a pose. | *Strength* (multiplies muscle strength), *Ease in* (seconds), and the pose itself: captured from what the viewport shows at the playhead when the event is added; **Capture pose at playhead** re-records it. |
| **Constraint** | From its first frame to its last, holds bodies: each where it was when the clip started, or all of them to the last body listed. | *Type*: Lock in place, Lock position only, Lock orientation only, Join bodies together. *Position* / *Rotation strength*: spring stiffness along each world axis (N/m, N·m/rad); 0 leaves an axis free. *Bodies*: any body of any character. |

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
| `Active pose` | `strength`, `entryBlendPeriod` (seconds); the pose is the clip's channels (one key each) |
| `Constraint` | `lockType` (`Lock`, `LockPosition`, `LockOrientation`, `JoinBodies`), `targets`, `positionStrength`, `angularStrength` (`"x, y, z"`) |
| `Behaviour` | `behaviourName` |

`targets` lists bodies as `character.MetaBodies.Body`, comma separated. A force with no
targets pushes every body of its character.

`src/document/tests.rs` rebuilds the physics demo's events through the editor's own
operations and checks the simulation is identical to the built-in demo's, and that
transitions, active poses and constraints change the simulation as described.
