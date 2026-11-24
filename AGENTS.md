# Instructions for AI coding agents

This file is for AI assistants (Claude Code, Codex, Copilot, Cursor and others)
working on NeuroMotion. Humans: everything here is also in
[CONTRIBUTING.md](CONTRIBUTING.md) and [ARCHITECTURE.md](ARCHITECTURE.md), which are
the real source; this file is the short version with the points agents get wrong.
Read both before your first change.

NeuroMotion is a desktop motion editor in Rust (egui) with its own rigid-body physics
engine. It opens endorphin 2.5 scenes and aims to match endorphin's results, then go
further ([ROADMAP.md](ROADMAP.md)).

## Non-negotiable rules

1. **Clean-room.** Never decompile, disassemble or otherwise read endorphin's program
   code or its protected behaviour files (`.nmb`), and never use code derived from
   them. Never add endorphin's files (scenes, characters, poses, help text, icons,
   screenshots) to the repository. You may describe endorphin's observed behaviour and
   read the structure of a user's own files to import them. If a task seems to need
   any of this, stop and ask the human.
2. **Don't fake results.** Never loosen, skip, `#[ignore]` or delete a test to get
   green, and never change expected numbers without explaining why in the commit.
   If something doesn't work yet, say so in the code, docs and your report.
3. **Deterministic physics.** The same project must simulate identically on every run
   of the same build on the same machine. No randomness, no wall-clock time, no
   iteration over `HashMap` where order matters. Behaviours are stateless: they read
   the bodies every tick. Other platforms aren't bit-identical: their maths libraries
   round `sin`, `atan2` and the like differently in the last bit, and a chaotic moment
   (a foot landing, a limb whipping round) can turn that into a different outcome. So
   tests must not hang on a knife edge: check how far a number moves when the scene
   is nudged (`nudge_survey` in `src/physics/scenes.rs`) and keep bounds clear of it.
4. **Module boundaries.** `document`, `timeline` and `physics` never depend on `ui` or
   egui. Every user-visible change to a project goes through `Document::apply` as an
   `Edit`, so it can be undone.
5. **Sign-off belongs to a human.** Every commit needs a `Signed-off-by` line (DCO),
   which certifies that the submitter has the right to contribute the change. An AI
   can't make that certification. Add a sign-off only in the name of the human you
   work for, and only if they've told you to; otherwise leave it to them
   (`git commit -s`).
6. **Say that AI helped.** End commit messages with a `Co-Authored-By:` trailer naming
   the assistant, and mention AI assistance in pull request descriptions.

## Before you push

Run all of these; CI runs the same on Linux, Windows and macOS:

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --release
```

Physics changes: if the human has endorphin installed, ask them to run
`cargo test --release -- --ignored` (it simulates every sample scene) and report what
changed.

## How work is done here

- **Physics changes come with a stand-in.** Reproduce the situation as a small scene
  built in code in `src/physics/scenes.rs`, show it failing, then fix the engine.
  Assert on numbers: angles, heights, frames, joint stretch. Trace with
  `NM_TRACE=<stand-in or .ens path> cargo test --release trace_stand_in -- --ignored --nocapture`.
- **Measure, don't assume.** Report before and after numbers in commit messages and
  docs. A change that fixes one scene must not regress the others: the existing
  tests and the stand-ins' table in `docs/physics-design.md` are the baseline.
- **Behaviours are original designs** that only share endorphin's names; see
  `docs/behaviours.md` and `src/physics/behaviour.rs`. They must work on any biped
  `Layout` finds, including characters that rest in a T-pose.
- **Docs move with the code.** Update the relevant file in `docs/`, the README's
  status and `ROADMAP.md` when behaviour changes (and its short Discord version,
  `docs/discord-roadmap.md`, when an item moves between done, in progress and
  planned). A physics lesson learned gets a
  numbered entry in `docs/physics-design.md`.
- **Units in names or comments:** metres, kilograms, seconds, radians, N, N·m, frames.
- **Match the surrounding code:** naming, comment density, idioms. Comments explain why.
- **Keep changes focused:** one topic per pull request. Don't reformat or refactor
  code you weren't asked to touch.

## Commits and pull requests

- Subject: the area first, then what changed, under ~70 characters:
  `Physics: ...`, `Behaviours: ...`, `Timeline: ...`, `Import: ...`, `Docs: ...`.
- Body: why the change was needed, what it does, measured results, what's still open.
- Pull requests target `main`, fill in the template, and need green CI. Never push to
  `main` directly, rewrite shared history, or merge without the maintainer's say-so.

## Where to look

| Task | Start in |
|---|---|
| Physics engine | `src/physics/world.rs`, `docs/physics-design.md` |
| Simulation over time, events | `src/physics/sim.rs`, `src/document/events.rs`, `docs/events.md` |
| Behaviours | `src/physics/behaviour.rs`, `docs/behaviours.md` |
| endorphin import | `src/io/endorphin.rs`, `docs/endorphin-formats.md` |
| Export | `src/io/motion.rs`, `bvh.rs`, `fbx.rs`, `docs/export.md` |
| UI | `src/ui/`, `docs/layout.md` |
| Data model, undo | `src/document/model.rs`, `src/document/edit.rs` |
