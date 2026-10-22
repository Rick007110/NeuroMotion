"""Imports NeuroMotion's sample exports into Blender and checks that every joint lands
where NeuroMotion put it.

Write the samples, then run this script with Blender:

    cargo test -- --ignored export_samples
    blender --background --python tools/blender_check.py -- target/export-samples

(It also runs with the `bpy` module from PyPI: `python tools/blender_check.py DIR`.)

Each `NAME.json` lists joint positions (metres, NeuroMotion's Y-up space) at a few
frames; `NAME.bvh`, `NAME-cm.bvh`, `NAME.fbx` and `NAME-cm.fbx` are imported in turn
with Blender's default options (the centimetre BVH at scale 0.01) and their bones
compared. ASCII FBX files are skipped: Blender only imports binary FBX.
"""

import json
import pathlib
import sys

import bpy

TOLERANCE = 0.001  # metres


def samples_dir():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    return pathlib.Path(argv[0] if argv else "target/export-samples")


def import_file(path):
    """Imports into an empty scene; returns the armature and the scene frame of the
    motion's first frame."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    if path.suffix == ".bvh":
        scale = 0.01 if path.stem.endswith("-cm") else 1.0
        bpy.ops.import_anim.bvh(filepath=str(path), global_scale=scale, frame_start=1, update_scene_fps=True)
        first = 1
    else:
        bpy.ops.import_scene.fbx(filepath=str(path))
        first = None  # keys sit at their own times, shifted by the importer's frame offset
    armatures = [o for o in bpy.context.scene.objects if o.type == "ARMATURE"]
    if len(armatures) != 1:
        raise RuntimeError(f"expected one armature, got {len(armatures)}")
    return armatures[0], first


def check(path, reference):
    armature, first = import_file(path)
    scene = bpy.context.scene
    if first is None:
        # Blender's FBX importer offsets animation by one frame by default.
        first = reference["start"] + 1
    worst = (0.0, "", 0)
    for index, frame in enumerate(reference["frames"]):
        scene.frame_set(first + frame)
        for name, positions in reference["joints"].items():
            bone = armature.pose.bones.get(name)
            if bone is None:
                raise RuntimeError(f"no bone named {name}")
            head = armature.matrix_world @ bone.head
            # Blender is Z up; NeuroMotion is Y up with Blender's -Y as its +Z.
            x, y, z = head.x, head.z, -head.y
            expected = positions[index]
            error = sum((a - b) ** 2 for a, b in zip((x, y, z), expected)) ** 0.5
            if error > worst[0]:
                worst = (error, name, reference["start"] + frame)
    fps = scene.render.fps / scene.render.fps_base
    if abs(fps - reference["fps"]) > 1e-3:
        raise RuntimeError(f"scene runs at {fps} fps, expected {reference['fps']}")
    return worst


def main():
    directory = samples_dir()
    failures = []
    references = sorted(directory.glob("*.json"))
    if not references:
        raise SystemExit(f"no samples in {directory}; run `cargo test -- --ignored export_samples` first")
    for reference_path in references:
        reference = json.loads(reference_path.read_text())
        base = reference_path.stem
        for suffix in (".bvh", "-cm.bvh", ".fbx", "-cm.fbx"):
            path = directory / f"{base}{suffix}"
            if not path.exists():
                continue
            try:
                error, joint, frame = check(path, reference)
            except Exception as e:  # report and carry on with the other files
                failures.append(path.name)
                print(f"FAIL {path.name}: {e}")
                continue
            ok = error <= TOLERANCE
            if not ok:
                failures.append(path.name)
            print(f"{'ok  ' if ok else 'FAIL'} {path.name:<40} worst {error * 1000:7.3f} mm ({joint} at frame {frame})")
    print(f"{len(failures)} failure(s)" if failures else "all exports match")
    if failures:
        raise SystemExit(1)


main()
