# Exporting motion

**File → Export** writes one character's motion as BVH or FBX for other tools. A small
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

## FBX (`.fbx`)

FBX 7.4 (FBX 2014/2015), readable by Blender, Maya, MotionBuilder, 3ds Max, Unity and
Unreal. The window offers two encodings:

- **Binary** (default). Blender's FBX importer only reads binary files.
- **ASCII**, the same content as readable text, for diffing or debugging.

| Part | What NeuroMotion writes |
|---|---|
| Skeleton | One `LimbNode` model (with a `Skeleton` node attribute) per joint, parented like the skeleton. Each model's static transform is the joint's **rest pose** (rest offset and rest rotation), so importers show the character's own rest pose. |
| Animation | One take ("Take 001") with one layer. Every joint has a rotation curve per axis (`Lcl Rotation`, full local rotation in degrees, one linear key per frame at the frame's time); roots, and joints whose offset is animated, also have translation curves. |
| Rotation order | Per joint, as for BVH, written as the model's `RotationOrder` with `RotationActive` on (without it, importers ignore the order). |
| Axes and units | Declared in `GlobalSettings`: Y up, Z front, X across (NeuroMotion's space), and `UnitScaleFactor` (centimetres per unit: 100 for a metres export, 1 for centimetres), so importers scale and rotate it into their own conventions. Either unit imports at the right size. |
| Frame rate | `TimeMode` for standard rates (24, 25, 30, 48, 50, 60, ...), otherwise a custom rate. |

### Opening in Blender

File → Import → FBX (.fbx) with the default options. Blender shifts imported
animation by one frame by default (**Animation Offset** = 1), so the project's frame 0
lands on Blender's frame 1.

## Checking exports in Blender

`tools/blender_check.py` imports exports into Blender and checks every joint against
the positions NeuroMotion computed:

```bash
cargo test --release -- --ignored export_samples
blender --background --python tools/blender_check.py -- target/export-samples
```

The first command writes BVH (metres and centimetres) and binary FBX (metres and
centimetres) files of the placeholder project, the physics demo and a skeleton with
rotated rest frames, plus a `.json` of expected joint positions for each, to
`target/export-samples`. Set `NEUROMOTION_SAMPLE_SCENE` to an endorphin `.ens` file to
also export every character of that scene (simulated). With Blender 5.2 all of them
match to within 0.15 mm; the physics demo's residual is the simulation's own joint
stretch. The script also runs with Blender's Python module (`pip install bpy`).

## Baking the simulation to keys

**Edit → Bake Simulation** (with a character, or one of its tracks or clips, selected)
turns that character's simulated motion over the project range into an ordinary motion
clip, so it can be edited, layered and exported like any other animation:

- A new track named **Baked** goes on top of the character's other tracks (the highest
  blend layer, listed last under the character), with one clip covering the project
  range, keyed on every frame: a rotation for every joint (relative to its rest
  orientation, X/Y/Z like hand-made keys), the root's position, and the offset of any
  other joint that moves by more than 1 mm.
- The character then plays its keys: its starting level becomes *Animated* and its
  event clips (simulation switches, forces, active poses, transitions, constraints, ...)
  are disabled. Nothing is deleted; turn an event back on with its **Enabled** box.
- The whole bake is a single undo step.

Other characters no longer collide with a baked (animated) character. To keep it
pushing them around, set its **Starts as** level (character inspector, Physics) to
*Collision only* or *Collision with momentum*.

## Tests

`src/io/bvh.rs` and `src/io/fbx.rs` export the placeholder project, the physics demo
and a hand-made skeleton with rotated rest frames (one joint sitting exactly at XYZ
gimbal lock), read each file back with independent readers (BVH; binary FBX, checking
every record offset and the footer; ASCII FBX, which must say exactly what the binary
says) and check forward-kinematics joint positions against the source at every frame.
Errors are under 0.001 mm for keys and under 0.5 mm for the simulation (the
simulation's tiny joint stretch is the difference).

`src/document/tests.rs` bakes the placeholder animation and the physics demo's
simulation, checks the keys alone reproduce every joint within 0.1 mm (keys) and 2 mm
(simulation), and that one undo restores the project exactly.
