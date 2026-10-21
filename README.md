# NeuroMotion

A timeline-based motion editor for character animation, built to pick up where
NaturalMotion's endorphin left off. Endorphin hasn't been updated since 2005;
NeuroMotion opens your existing endorphin scenes, characters and poses, and is building
toward a modern physics engine that behaves like endorphin's and then goes further.

> **Status: early development.** The editor, timeline, undo/redo and endorphin import
> work today. Physics simulation is being built now: imported physics events (forces,
> behaviours, simulation switches, transitions) show on the timeline with all their
> settings, but don't simulate yet.

## What works

- **Editor**: viewport with an orbit camera, a timeline with tracks, clips, keys,
  scrubbing and transport, an inspector for whatever is selected, and a menu bar.
- **Animation**: keyframed clips layered per track with weights and blend-in/out
  ramps; step, linear and smooth curves.
- **Undo/redo** on the document model, with each drag recorded as one step.
- **Projects** saved as readable JSON (`.nmproj`), with versioned migrations.
- **Opening endorphin files**:

  | File | Support |
  |---|---|
  | `.ens` scenes | Characters, props and environment; animation as editable keys; every event on the timeline |
  | `.nmc` characters, props and rigs | Open as a project |
  | `.nma` poses | Import onto a character at the playhead |

  160 of the files bundled with endorphin 2.5.2 import. The exceptions are
  password-protected scenes and one legacy pose format. Details:
  [docs/endorphin-formats.md](docs/endorphin-formats.md).

## Roadmap

1. **Physics core**: rigid bodies per bone, twist/swing joint limits, contacts and
   friction, at a fixed 120 Hz, deterministic. Modelled on endorphin so imported
   characters keep their masses, limits and muscle strengths.
2. **Muscles**: force-limited joint drives that follow an animation or a target pose,
   with endorphin's strength presets.
3. **Timeline physics**: per-character simulation levels, momentum carried from
   animation into physics, push events, transitions back to animation, and automatic
   re-simulation when you edit (no manual simulate/clear step).
4. **Behaviours**: controllers such as balance, stagger and catch-fall.
5. **Beyond endorphin**: strength curves per body region, smoother hand-back to
   animation, and visual feedback on muscle effort.

Research notes on how endorphin's physics works:
[docs/endorphin-physics.md](docs/endorphin-physics.md).

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

If endorphin 2.5.2 is installed, this test imports every file it ships with. Set
`ENDORPHIN_DIR` if it isn't in the default location.

```bash
cargo test -- --ignored
```

## Layout of the code

| Module | Purpose |
|---|---|
| `document` | Project data model, pose evaluation, undo/redo. No UI. |
| `timeline` | Transport and time-to-pixel mapping. No drawing. |
| `io` | Project files and endorphin importers |
| `viewport` | Camera and character preview |
| `ui` | egui application shell, panels, menus, keyboard shortcuts (`ui/shortcuts.rs`) |

Design notes: [docs/layout.md](docs/layout.md),
[docs/project-format.md](docs/project-format.md).

## License

GPL-3.0-or-later. See [LICENSE](LICENSE).

NeuroMotion is an independent project and is not affiliated with or endorsed by
NaturalMotion. "endorphin" is a trademark of its owner and is used here only to
describe file compatibility. NeuroMotion contains no endorphin code or data; it reads
files you already have.
