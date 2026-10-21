# NeuroMotion

A timeline-based motion editor for character animation, built to pick up where
NaturalMotion's endorphin left off. Endorphin hasn't been updated since 2005;
NeuroMotion opens your existing endorphin scenes, characters and poses, and is building
toward a modern physics engine that behaves like endorphin's and then goes further.

> **Status: early development.** The editor, timeline, undo/redo and endorphin import
> work. The physics engine runs imported endorphin characters with their own bodies,
> joint limits and muscles; 8 of the 15 bundled sample scenes already simulate cleanly.
> endorphin's *behaviours* (stagger, catch fall, ...) aren't recreated yet, so scenes
> that rely on them behave like a limp character instead.

## What works

- **Editor**: viewport with an orbit camera, a timeline with tracks, clips, keys,
  scrubbing and transport, an inspector for whatever is selected, and a menu bar.
- **Animation**: keyframed clips layered per track with weights and blend-in/out
  ramps; step, linear and smooth curves.
- **Undo/redo** on the document model, with each drag recorded as one step.
- **Projects** saved as readable JSON (`.nmproj`), with versioned migrations.
- **Export**: File → Export writes a character's motion (simulated or keyed) as BVH.
  **Edit → Bake Simulation** turns a character's simulation into ordinary keys.
  Details: [docs/export.md](docs/export.md).
- **Opening endorphin files**:

  | File | Support |
  |---|---|
  | `.ens` scenes | Characters, props and environment; animation as editable keys; every event on the timeline |
  | `.nmc` characters, props and rigs | Open as a project |
  | `.nma` poses | Import onto a character at the playhead |

  160 of the files bundled with endorphin 2.5.2 import. The exceptions are
  password-protected scenes and one legacy pose format. Details:
  [docs/endorphin-formats.md](docs/endorphin-formats.md).

## Physics

NeuroMotion has its own physics engine, modelled on endorphin's so imported scenes
behave the same way, then improved:

- One rigid body per bone, with endorphin's masses and collision shapes; twist/swing
  joint limits with soft zones; force-limited muscles with endorphin's presets
  (Default, Normal, DesiredPoseCalibration, Strong, Weak); material pairs for friction
  and bounce.
- Per-character simulation levels (animated, collision only, collision with momentum,
  simulated), momentum carried from animation into physics, push events, active poses,
  active animations, lock/join constraints, and transitions back to animation.
- **Improvements over endorphin:** no simulate/clear step (the timeline re-simulates
  automatically, in the background, whenever you edit); deterministic results;
  stiff muscles that can't explode; muscle targets kept inside joint limits.

Try **File → Open Physics Demo** and press Space. Design and status:
[docs/physics-design.md](docs/physics-design.md); research notes:
[docs/endorphin-physics.md](docs/endorphin-physics.md).

## Roadmap

1. Finish the sample-scene sweep: rope and ring scenes, multi-character joins, moving
   scenery.
2. **Behaviours**: original controllers for balance, stagger, catch-fall and more.
3. **Beyond endorphin**: strength curves per body region, smoother hand-back to
   animation, visual feedback on muscle effort, re-simulating only from the edited
   frame.

## Building and running

You need [Rust](https://rustup.rs) (stable).

```bash
cargo run --release
```

Open an endorphin scene directly:

```bash
cargo run --release -- "path/to/scene.ens"
```

Or use **File → Open**, or drag a file onto the window. Press **F1** in the app for
keyboard shortcuts (Space plays, ←/→ step frames, Ctrl+Z/Ctrl+Y undo/redo).

Run the tests:

```bash
cargo test
```

If endorphin 2.5.2 is installed, the ignored tests import every file it ships with
and simulate every sample scene. Set `ENDORPHIN_DIR` if it isn't in the default
location.

```bash
cargo test -- --ignored
```

## Layout of the code

| Module | Purpose |
|---|---|
| `document` | Project data model, pose evaluation, undo/redo. No UI. |
| `timeline` | Transport and time-to-pixel mapping. No drawing. |
| `io` | Project files, endorphin importers and motion exporters |
| `physics` | Rigid-body solver, collisions, and the timeline-driven simulation |
| `viewport` | Camera and character preview |
| `ui` | egui application shell, panels, menus, keyboard shortcuts (`ui/shortcuts.rs`) |

Design notes: [docs/layout.md](docs/layout.md),
[docs/project-format.md](docs/project-format.md), [docs/export.md](docs/export.md).

## License

GPL-3.0-or-later. See [LICENSE](LICENSE).

NeuroMotion is an independent project and is not affiliated with or endorsed by
NaturalMotion. "endorphin" is a trademark of its owner and is used here only to
describe file compatibility. NeuroMotion contains no endorphin code or data; it reads
files you already have.
