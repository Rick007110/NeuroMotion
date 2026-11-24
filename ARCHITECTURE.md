# Architecture

A map of NeuroMotion's code for people who want to change it. For *why* things are the
way they are, see the design notes in [docs/](docs).

NeuroMotion is a single Rust binary: a desktop app built with
[egui](https://github.com/emilk/egui) (immediate-mode UI), its own rigid-body physics
engine, and importers and exporters for animation files.

## The big picture

```
            edits (undoable)                       revision number
  ui  ───────────────────────────►  document  ─────────────────────────►  physics::sim
  ▲                                  │  Project: characters, tracks,       │  re-simulates in the
  │                                  │  clips, keys, physics events        │  background, caches
  │                                  ▼                                     ▼  every frame
  └──────── viewport + timeline draw the pose at the playhead ◄─── simulated frames
                                     ▲
                       io: open .nmproj / endorphin files, save, export BVH / FBX
```

1. **Everything the user makes lives in a `Project`** (`document::model`): characters
   with their skeletons and physics rigs, tracks, clips (animation and physics events)
   and keys. It is plain serializable data with no UI in it.
2. **Every change goes through `Document::apply`** (`document::edit`) as an `Edit` that
   stores the before and after state. That one path gives undo/redo, the dirty flag and
   a revision number that increases with every change.
3. **The simulation follows the revision.** `physics::Simulation` notices a new
   revision, throws away its cache and simulates again from the start. It runs in
   small time slices each UI frame (`ui::app::advance_simulation`), so editing never
   blocks. There is no "simulate" button.
4. **Drawing reads, never writes.** The viewport and the timeline draw the project
   and the cached simulation at the playhead; user input becomes `Edit`s.

## Modules

| Module | What it does | Depends on |
|---|---|---|
| `document` | The data model, pose evaluation (blending clips), undoable edits, baking simulation to keys | nothing in the crate |
| `timeline` | Transport (play, step, loop) and the frame ↔ pixel mapping. Draws nothing. | `document` |
| `physics` | The rigid-body engine and the simulation of a project over time | `document` |
| `io` | `.nmproj` project files, endorphin importers, BVH and FBX exporters | `document`, `physics` (export of simulated motion) |
| `viewport` | Orbit camera and drawing the characters | `document` |
| `ui` | The egui app: window layout, menus, panels, inspector, shortcuts | everything |

Keep these boundaries: `document`, `timeline` and `physics` must never depend on
`ui` or egui, so they stay testable and could run without a window.

### `document`

- `model.rs`: `Project`, `Character`, `Track`, `Clip`, `Keyframe`, ids. Serialized
  with serde; the file format is described in [docs/project-format.md](docs/project-format.md).
- `edit.rs`: `Document` (project + undo stack + revision) and the `Edit` enum. A new
  kind of change usually means a new `Edit` variant with `before`/`after`.
- `ops.rs`: higher-level operations (add or move keys, clips, tracks) built from edits.
- `pose.rs`: evaluates a character's blended pose at a frame.
- `events.rs`: typed settings for physics events (simulation, force, active pose,
  constraints, behaviours). See [docs/events.md](docs/events.md).
- `physics.rs`: physics rig data: bodies, shapes, joints, limits, muscle gains,
  materials. Data only; the engine is in `physics`.
- `bake.rs`: turns simulated world-space motion back into keys.
- `sample.rs`: the built-in placeholder character and demo animation.

### `physics`

- `world.rs`: bodies, joints, muscles, locks and contacts, and the solver (substepped
  sequential impulses with soft constraints; locked groups, and characters something
  heavy presses on, solved directly).
- `collide.rs`: contact generation between shapes (spheres, capsules, boxes,
  cylinders).
- `behaviour.rs`: controllers that drive muscles while a character is simulated
  (Balance, CatchFall, Stagger). New behaviours go here; see [docs/behaviours.md](docs/behaviours.md).
- `sim.rs`: runs a project through the world frame by frame, following its physics
  events, and caches the results for the timeline.
- `scenes.rs` (tests only): small test scenes ("stand-ins") that pin down physical
  behaviour, plus sweeps over endorphin's sample scenes when it is installed.

The design, the numbers behind it, the lessons learned from endorphin's scenes and
what is still missing are in [docs/physics-design.md](docs/physics-design.md).
Research on how endorphin behaves is in [docs/endorphin-physics.md](docs/endorphin-physics.md).

### `io`

- `mod.rs`: opening and saving `.nmproj` (JSON with a format version and migrations).
- `endorphin.rs`: importers for `.ens`, `.nmc` and `.nma`. Format notes:
  [docs/endorphin-formats.md](docs/endorphin-formats.md).
- `motion.rs`, `bvh.rs`, `fbx.rs`: export a character's motion. See [docs/export.md](docs/export.md).

### `ui` and `viewport`

- `ui/app.rs`: the app: layout, action dispatch, running the simulation each frame.
- `ui/actions.rs` and `ui/shortcuts.rs`: every command and its key binding.
- `ui/timeline_panel.rs`, `ui/inspector.rs`, `ui/event_inspector.rs`, `ui/menu.rs`,
  `ui/export.rs`: the panels.
- `viewport/`: the orbit camera and drawing with egui's painter. A GPU renderer can
  replace it without touching the rest. Layout notes: [docs/layout.md](docs/layout.md).

### Other folders

- `assets/`: the app icon. `icon.svg` is the source; `icon-256.png` (the window
  icon) and `icon.ico` (embedded in the Windows executable by `build.rs`) are made
  from it. `logo-transparent.png` is the logo without its tile, with a dark outline
  so it reads on any background (the Discord webhooks' avatar).
- `docs/discord-roadmap.md`: the short roadmap the Discord roadmap workflow keeps
  posted in the community's #roadmap channel, and `docs/roadmap-timeline.json`, the
  releases drawn as a timeline image under it (by `tools/roadmap_timeline.py`); keep
  both in step with `ROADMAP.md`.
- `tools/`: helper scripts, such as checking exports in Blender.

## Principles

- **Deterministic simulation.** The same project gives the same motion every time
  with the same build on the same machine (other platforms can differ in the last
  bits: docs/physics-design.md).
  Behaviours are stateless; nothing in the physics reads the clock or randomness.
- **Undo covers everything.** If the user can change it, it's an `Edit`.
- **Tests pin down behaviour.** Physics changes come with a stand-in scene in
  `physics/scenes.rs` that checks the numbers (angles, heights, frames).
- **Original work only.** NeuroMotion reads endorphin's file formats but contains
  none of its code or assets. See [CONTRIBUTING.md](CONTRIBUTING.md#ground-rules).

## Common changes

| You want to... | Start in |
|---|---|
| Add a behaviour (stagger, ...) | `physics/behaviour.rs`, `docs/behaviours.md`, a stand-in test in `physics/scenes.rs` |
| Add a physics event type | `document/events.rs`, `physics/sim.rs`, `ui/event_inspector.rs`, `docs/events.md` |
| Fix how an endorphin file imports | `io/endorphin.rs`, `docs/endorphin-formats.md` |
| Add an export format | `io/motion.rs` and a new module next to `bvh.rs` |
| Add a command or shortcut | `ui/actions.rs`, `ui/shortcuts.rs`, `ui/menu.rs` |
| Change what's stored in a project | `document/model.rs`, a migration in `io/mod.rs`, `docs/project-format.md` |
