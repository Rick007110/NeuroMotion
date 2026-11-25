# Contributing to NeuroMotion

Thanks for helping build NeuroMotion. Bug reports, test scenes, reference measurements
from endorphin, documentation and code are all welcome. This page explains how to get
started and the few rules every contribution has to follow.

- [Ground rules](#ground-rules): clean-room, sign-off, licence
- [Ways to help](#ways-to-help)
- [Building and testing](#building-and-testing)
- [Making a pull request](#making-a-pull-request)
- [Code style](#code-style)

By taking part you agree to follow the [Code of Conduct](CODE_OF_CONDUCT.md).

## Ground rules

### 1. Clean-room: no endorphin code, files or assets

NeuroMotion is an independent project and must stay that way (see the README's
[Not affiliated](README.md#not-affiliated-with-naturalmotion-or-endorphin) section).
Every contribution must be your own work:

- **Do not** decompile, disassemble or debug endorphin's program files, or read code
  derived from them, and don't contribute anything based on that.
- **Do not** add endorphin's files to the repository: no scenes, characters, poses,
  environments, behaviours, icons, help text or screenshots of its interface.
  Tests that need endorphin's files read them from a local install and are
  `#[ignore]`d (see [Building and testing](#building-and-testing)).
- **Do not** read or use endorphin's vendor-protected behaviour definitions.
- **You may** describe how endorphin *behaves*: what it does with a scene, which
  settings exist, measurements you take from it ("the pelvis turns -74° by frame
  297"). Studying the file formats of your own files so NeuroMotion can open them is
  fine too. Write the implementation yourself.

If you're unsure whether something is allowed, ask in the issue or pull request
before writing the code.

### 2. Sign off your commits (DCO)

Every commit must carry a `Signed-off-by` line. It certifies the
[Developer Certificate of Origin](https://developercertificate.org): that you wrote
the change, or otherwise have the right to submit it under the project's licence.
Git adds it for you with `-s`:

```bash
git commit -s -m "Physics: ..."
```

That adds `Signed-off-by: Your Name <you@example.com>` using your git `user.name`
and `user.email`. Use your real name or the name you're known by on GitHub. To sign
off commits you already made on your branch:

```bash
git rebase --signoff main
git push --force-with-lease
```

A check on every pull request reports commits without a sign-off.

### 3. Licence

NeuroMotion is licensed under the [GNU GPL v3 or later](LICENSE). By contributing you
agree that your contribution is released under the same licence.

## Ways to help

You don't need to write Rust to help.

- **Use it and report bugs.** Open an issue with the steps to reproduce, what you
  expected, and what happened. If an endorphin scene behaves differently, say which
  scene and what endorphin does.
- **Measure endorphin.** If you have endorphin, record how a scene behaves (frame
  numbers, angles, when a character leaves the ground, a screen recording you made
  yourself) and open a *Reference measurement* issue. These numbers become tests that
  keep NeuroMotion's physics honest. Don't attach endorphin's own files.
- **Write documentation** for the parts you've learned.
- **Pick up an issue.** Issues labelled
  [`good first issue`](https://github.com/Rick007110/NeuroMotion/labels/good%20first%20issue)
  are small and self-contained; [`help wanted`](https://github.com/Rick007110/NeuroMotion/labels/help%20wanted)
  ones are bigger. Comment on the issue to say you're working on it, so two people
  don't do the same work.
- **Propose something bigger** (a new behaviour, a renderer, a file format) in an issue
  first. Agreeing on the design before writing much code saves everyone time.

## Building and testing

You need [Rust](https://rustup.rs) (stable, 1.88 or newer). On Linux you may also need
the usual windowing development packages (X11 or Wayland).

```bash
cargo run --release              # the app
cargo run --release -- scene.ens # open a file on startup
cargo test                       # all tests that don't need endorphin
```

CI runs these on Linux, Windows and macOS, and they must pass before a pull request
is merged:

```bash
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test
```

Some tests are `#[ignore]`d because they need something not in the repository:

| Test | Needs | Run with |
|---|---|---|
| `imports_every_bundled_file`, `sample_scenes_simulate_stably` | a local endorphin 2.5.2 install (`ENDORPHIN_DIR` if not in the default location); `NM_SCENES=Car,Rope` sweeps only scenes whose names contain one of those | `cargo test --release -- --ignored` |
| `behaviour_inventory` | a local endorphin 2.5.2 install (`ENDORPHIN_DIR`); prints which behaviours and event types the scenes use, with their settings ([docs/behaviour-inventory.md](docs/behaviour-inventory.md)) | `cargo test --release behaviour_inventory -- --ignored --nocapture` |
| `export_samples` | Blender, to check the written files | see [docs/export.md](docs/export.md#checking-exports-in-blender) |
| `trace_stand_in` | nothing; prints a frame-by-frame trace of a physics test scene or an `.ens` file | `NM_TRACE=<name or path> cargo test --release trace_stand_in -- --ignored --nocapture` |
| `push_sweep` | nothing; prints which chest pushes (1-3 m/s, four ways) a behaviour survives | `NM_SWEEP=<behaviour> cargo test --release push_sweep -- --ignored --nocapture` |
| `dump_scene_motion` | nothing (an `.ens` scene needs its files); simulates a stand-in or a scene and writes its motion as CSV for comparing with a reference such as endorphin's own run: per frame, every body's and every skeleton joint's world position and rotation with the simulation level (`bodies.csv`, `joints.csv`), plus each body's frame on its joint (`rig.csv`) and its collision shapes (`shapes.csv`) | `NM_DUMP=<name or .ens path> cargo test --release dump_scene_motion -- --ignored --nocapture` (`NM_SECONDS` to run longer or shorter than the scene's range, `NM_DUMP_OUT` for the folder, default `target/dump/<scene>`) |
| `nudge_survey` | nothing; prints how far the numbers the stepping, Car carry, Car Hit and Explosion tests assert on move when the scenes' speeds (or drop height) are nudged by about 1e-4 | `cargo test --release nudge_survey -- --ignored --nocapture` (`NM_SURVEY=car\|car_hit\|explosion\|balance\|stagger`, `NM_NUDGES`, `NM_NUDGE_STEP`) |

If you change the physics and have endorphin, please run the ignored tests and say in
your pull request what changed.

### Working with AI assistants

AI-assisted contributions are welcome under the same rules. [AGENTS.md](AGENTS.md)
tells AI coding agents how to work here (Claude Code reads it through
[CLAUDE.md](CLAUDE.md)). You remain responsible for what you submit: review it, and
sign off the commits yourself.

### Where things are

[ARCHITECTURE.md](ARCHITECTURE.md) is a map of the code. The design notes in
[docs/](docs) explain why things are the way they are; read the one for the area you're
changing, especially [docs/physics-design.md](docs/physics-design.md) before touching
the solver.

## Making a pull request

1. Fork the repository and create a branch from `main`.
2. Keep each pull request to one change. A focused PR is reviewed much faster than one
   that mixes a fix, a feature and a refactor.
3. Add or update tests. Physics changes need a test scene that shows the behaviour
   (see `src/physics/scenes.rs`); bug fixes need a test that fails without the fix.
4. Update the docs in `docs/` and the README if you change behaviour they describe.
5. Write commit messages like the existing ones: a short subject with the area first
   (`Physics: ...`, `Timeline: ...`, `Import: ...`), then a body explaining *why*.
   Sign off every commit.
6. Open the pull request and fill in the template. CI must be green; a maintainer
   will review it.

## Code style

- `cargo fmt` formats the code; don't fight it.
- Match the surrounding code: its naming, comment density and idioms.
- Keep the module boundaries in [ARCHITECTURE.md](ARCHITECTURE.md): `document` and
  `physics` never depend on `ui`.
- Comment *why*, not *what*. Units go in names or comments (N, N·m, kg, radians, frames).
- Simulation must stay deterministic: the same project gives the same result on every
  run of the same build on the same machine. No randomness, no wall-clock time in the
  physics, no iteration over hash maps where order matters. Other platforms can differ
  in the last bits (their maths libraries round differently), and a chaotic moment
  can make that a visibly different outcome, so a test mustn't depend on one exact
  number near a limit (see `nudge_survey` below).
- Text the user sees is plain, short English, in NeuroMotion's own words.
