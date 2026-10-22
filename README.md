# NeuroMotion

A timeline-based motion editor for character animation, built to pick up where
NaturalMotion's endorphin left off. Endorphin hasn't been updated since 2005;
NeuroMotion opens your existing endorphin scenes, characters and poses, and is building
toward a modern physics engine that behaves like endorphin's and then goes further.

> **Status: early development.** The editor, timeline, undo/redo and endorphin import
> work. The physics engine runs imported endorphin characters with their own bodies,
> joint limits and muscles; 11 of the 16 bundled sample scenes checked simulate
> cleanly. Of endorphin's *behaviours*, balance and catch fall have original
> controllers; scenes that rely on the others (stagger, writhe, ...) behave like a limp
> character instead.

## Not affiliated with NaturalMotion or endorphin

NeuroMotion is an independent, from-scratch project. It is **not** affiliated with,
endorsed by, sponsored by or otherwise connected to NaturalMotion, endorphin, or any
company that owns or has owned them.

It is not a clone and contains no endorphin code. We research how endorphin's features
work, then design and write our own implementation: the code, physics engine,
interface, icons and wording are NeuroMotion's own. No endorphin source code, program
files, behaviours, characters, scenes or other assets are copied into or shipped with
NeuroMotion.

NeuroMotion can open endorphin's file formats so that people can keep working with
scenes, characters and poses they already have. It reads those files from your own
endorphin install or projects; it doesn't include them. "NaturalMotion" and "endorphin"
are trademarks of their respective owners and are used here only to describe that
compatibility.

## What works

- **Editor**: viewport with an orbit camera, a timeline with tracks, clips, keys,
  scrubbing and transport, an inspector for whatever is selected, and a menu bar.
- **Animation**: keyframed clips layered per track with weights and blend-in/out
  ramps; step, linear and smooth curves.
- **Physics events**: add simulation switches, pushes, transitions back to animation,
  active poses and constraints from the timeline (right-click a track) or Edit → Add,
  and edit them in the inspector. Details: [docs/events.md](docs/events.md).
- **Undo/redo** on the document model, with each drag recorded as one step.
- **Projects** saved as readable JSON (`.nmproj`), with versioned migrations.
- **Export**: File → Export writes a character's motion (simulated or keyed) as BVH or
  FBX, checked against Blender's importers.
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
- Behaviours: balance (capture-point ankle strategy) and catch fall (arms reach for
  the ground), run by endorphin's behaviour events of the same name
  ([docs/behaviours.md](docs/behaviours.md)).
- **Improvements over endorphin:** no simulate/clear step (the timeline re-simulates
  automatically, in the background, whenever you edit); deterministic results;
  stiff muscles that can't explode; muscle targets kept inside joint limits; bodies
  that constraints hold together (a heavy prop in both hands, a trapeze on a long rope)
  solved exactly as one system.

Try **File → Open Physics Demo** and press Space, or knock the placeholder character
over yourself: right-click a track at some frame and add a Simulation event, then a
Force event at the same frame. Design and status:
[docs/physics-design.md](docs/physics-design.md); research notes:
[docs/endorphin-physics.md](docs/endorphin-physics.md).

## Roadmap

1. The five sample scenes that still fail: Balcony, Bendy Stick, Car Hit, Heli Leap
   and Rotating Floor ([status](docs/physics-design.md#status-against-endorphins-sample-scenes)).
2. **Behaviours**: stepping, stagger, writhe and more, after balance and catch fall.
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

To check exports in Blender, see [docs/export.md](docs/export.md#checking-exports-in-blender).

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
[docs/project-format.md](docs/project-format.md), [docs/export.md](docs/export.md),
[docs/events.md](docs/events.md).

## License

GPL-3.0-or-later. See [LICENSE](LICENSE). NeuroMotion is not affiliated with
NaturalMotion or endorphin; see
[Not affiliated with NaturalMotion or endorphin](#not-affiliated-with-naturalmotion-or-endorphin).
