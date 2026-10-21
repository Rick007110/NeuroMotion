# Exporting motion

**File → Export** writes one character's motion to a file other tools can read. A small
window asks which character (the selected one by default) and which length unit; the
frame range and frame rate are the project's (Project in the inspector).

## Where the motion comes from

- If the project uses physics (any character is ever simulated), the export is the
  **simulation**: exactly what the viewport shows, joint by joint. If the simulation is
  still running in the background, the export finishes it first.
- Characters without a physics body, and projects that never simulate, export their
  **keys** (the blended animation of all their tracks).

The status line says which one was written.

## BVH (`.bvh`)

| Part | What NeuroMotion writes |
|---|---|
| Hierarchy | The character's own skeleton. Each `OFFSET` is the joint's rest offset in its parent's frame. Roots sit at the origin (`OFFSET 0 0 0`). |
| Rotation channels | The joint's **full local rotation**: rest orientation times the animated rotation. Importers don't need to know about rest orientations, and the motion plays back exactly. |
| Rotation order | `X Y Z` (the order NeuroMotion keys in) for every joint whose motion stays clear of gimbal lock; otherwise, per joint, whichever order keeps the middle angle furthest from ±90°. Curves are made continuous frame to frame (no 360° jumps or flips). |
| Position channels | On the root, holding its full position. Other joints only get them when their offset is animated by more than 1 mm (simulated joints stretch by fractions of a millimetre; that is left out). Position channels hold the joint's full local position, replacing its `OFFSET`, which is how Blender and MotionBuilder read them. |
| End sites | Leaves get a short tip along their own bone so importers can draw them. |
| Names | Joint names with spaces or `" : { } ,` get `_` instead, and repeated names get a number. |
| Units | Metres by default (Blender's unit). Centimetres for tools that expect them (Maya, MotionBuilder). |

Because the rotations include the rest orientation, a BVH importer's "rest pose" (all
channels zero) is the skeleton with every rest rotation removed. For NeuroMotion's own
characters that's the normal T-pose-like rest; for imported endorphin skeletons it can
look folded up. The animation itself is unaffected.

### Opening in Blender

File → Import → Motion Capture (.bvh). Keep **Scale** at 1 for a metres export, tick
**Scene FPS** to take the frame rate from the file, and **Update Scene Duration**. The
defaults for Forward and Up (`-Z`, `Y`) turn NeuroMotion's Y-up space into Blender's
Z-up space.

## Tests

`src/io/bvh.rs` exports the placeholder project, the physics demo and a hand-made
skeleton with rotated rest frames (one joint sitting exactly at XYZ gimbal lock),
reads each file back with an independent parser and checks forward-kinematics joint
positions against the source. Errors are under 0.001 mm for keys and under 0.5 mm for
the simulation (the simulation's tiny joint stretch is the difference).
