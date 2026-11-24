# NeuroMotion

A timeline-based motion editor for character animation, built to pick up where
NaturalMotion's endorphin left off. Endorphin hasn't been updated since 2005;
NeuroMotion opens your existing endorphin scenes, characters and poses, and is building
toward a modern physics engine that behaves like endorphin's and then goes further.

> **Status: early development.** The editor, timeline, undo/redo and endorphin import
> work. The physics engine runs imported endorphin characters with their own bodies,
> joint limits and muscles; 15 of the 16 bundled sample scenes checked simulate
> cleanly. Of endorphin's *behaviours*, balance and catch fall have original
> controllers; scenes that rely on the others (stagger, writhe, ...) behave like a limp
> character instead.

> **Want to help build it?** NeuroMotion is open to everyone: animators, programmers,
> testers and writers. See [Contributing](#contributing) below, or go straight to
> [CONTRIBUTING.md](CONTRIBUTING.md).

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

1. Car Hit, the one sample scene that still fails: a pedestrian's arm crushed under a
   car that stops dead ([status](docs/physics-design.md#status-against-endorphins-sample-scenes)).
2. **Behaviours**: stepping, stagger, writhe and more, after balance and catch fall.
3. **Beyond endorphin**: strength curves per body region, smoother hand-back to
   animation, visual feedback on muscle effort, re-simulating only from the edited
   frame.

Every item is an [issue](https://github.com/Rick007110/NeuroMotion/issues) you can pick
up; see [Contributing](#contributing).

## Download

Builds for Windows, macOS and Linux are attached to each
[release](https://github.com/Rick007110/NeuroMotion/releases). They aren't signed yet,
so Windows SmartScreen and macOS Gatekeeper will warn the first time you open them.

## Building and running

You need [Rust](https://rustup.rs) (stable, 1.88 or newer).

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

## Contributing

NeuroMotion is meant to be built by its community, not by one person. Help of every
kind is welcome, and you don't need to write Rust to help.

**How to contribute:**

1. **Read [CONTRIBUTING.md](CONTRIBUTING.md).** It explains how to build and test,
   how to send a pull request, and the ground rules every contribution follows:
   - **Clean-room only:** no endorphin code, files or assets, and no decompiling.
     Describing how endorphin *behaves* is welcome.
   - **Signed-off commits** (`git commit -s`), certifying the
     [Developer Certificate of Origin](https://developercertificate.org).
   - Contributions are released under the project's licence, GPL-3.0-or-later.
2. **Pick something to do:**
   - **Report a bug** or suggest a feature through
     [a new issue](https://github.com/Rick007110/NeuroMotion/issues/new/choose).
   - **Measure endorphin.** If you have it, record how a scene behaves (frames, angles,
     timings) with the *Reference measurement* issue template. These numbers become
     tests that keep NeuroMotion's physics close to endorphin's.
   - **Take an issue.** Those labelled
     [good first issue](https://github.com/Rick007110/NeuroMotion/labels/good%20first%20issue)
     are small and self-contained;
     [help wanted](https://github.com/Rick007110/NeuroMotion/labels/help%20wanted) ones
     are bigger. Comment on the issue first so two people don't do the same work.
   - **Improve the docs** in [docs/](docs).
3. **Find your way in the code** with [ARCHITECTURE.md](ARCHITECTURE.md), a map of
   the modules and where to start for common changes.
4. **Open a pull request.** CI checks formatting, clippy and the tests on Windows,
   macOS and Linux, and a maintainer reviews it.

Everyone taking part follows the [Code of Conduct](CODE_OF_CONDUCT.md). Security
problems go privately through [SECURITY.md](SECURITY.md), not in a public issue.

## License

GPL-3.0-or-later. See [LICENSE](LICENSE). NeuroMotion is not affiliated with
NaturalMotion or endorphin; see
[Not affiliated with NaturalMotion or endorphin](#not-affiliated-with-naturalmotion-or-endorphin).
