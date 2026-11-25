# Reference measurements

How close NeuroMotion is to endorphin, by numbers. endorphin's runs of its own scenes
were measured (frame numbers, heights, positions, angles), and the test
`reference_scores` simulates the same scenes in NeuroMotion, computes the same
quantities and scores the difference. "Behaves like endorphin" is then checked, not
eyeballed, and every change to the physics or the behaviours shows which way it moved
NeuroMotion.

The measurements are in `src/physics/reference.rs`, one row each. They are numbers
only: endorphin's scenes, recordings and frames stay on the machine that has
endorphin installed.

## Running it

```bash
ENDORPHIN_DIR="C:\Program Files (x86)\NaturalMotion\endorphin 2.5.2" \
NM_REFERENCE_SCENES=path/to/your/own/scenes \
cargo test --release reference_scores -- --ignored --nocapture
```

- `ENDORPHIN_DIR`: a local endorphin 2.5.2 install (the default is its usual Windows
  folder). Scenes are looked for under its `Resources/Scenes`.
- `NM_REFERENCE_SCENES` (optional): a folder with scenes of your own that rows refer
  to by file name, such as `Back-Flip-To-Combo.ens` (#6). A scene that can't be found
  is skipped and listed at the end.
- `NM_SCENES=Car,Flip` scores only scenes whose file names contain one of those.
- `NM_NUDGES=2` also runs each scene with gravity scaled by 1 ± 1e-4 and 1 ± 2e-4
  (`nudges` in `src/physics/scenes.rs`; `NM_NUDGE_STEP` sets the step), prints the
  range each value spans, and requires guards to hold in every run.

It takes a couple of seconds. For each scene it prints a table like this (Car Hit,
shortened):

```
Sample Scenes/Car Hit.ens (a frame-by-frame recording of endorphin's run, ...)
  quantity                                  endorphin NeuroMotion   error  baseline  within?
  flight: centre of mass highest      1.81 m ± 0.06 m      1.62 m -0.19 m    1.62 m  no (known gap)
  flight: centre of mass x, frame 68  0.01 m ± 0.10 m      0.06 m  0.05 m    0.06 m  yes (guard)
  ...
  score: 5 of 15 within endorphin's uncertainty; mean error 2.0 times the uncertainty (each row counted up to 4)
```

- **within?** is whether NeuroMotion's value is within the measurement's uncertainty
  of endorphin's.
- **Guards** are rows NeuroMotion already matches with room to spare, over nudged runs
  too. The test fails if a guard falls outside its uncertainty: that is a regression.
- Every other row is a **known gap**: printed with its error, never failed. A gap row
  that comes within its uncertainty says `yes (not a guard)`; make it a guard once it
  holds with a margin over `NM_NUDGES` runs.
- **baseline** is NeuroMotion's value when the row was written or last reviewed, so a
  change shows against it. Update it (in the same commit, saying why) when a change
  moves the number on purpose.
- The **score** per scene is the number of rows within, and the mean error in units of
  each row's uncertainty (a row that never happens, or is more than 4 uncertainties
  off, counts 4), so a scene's progress shows even before a row crosses its tolerance.

## The format

A scene names its file and how it was measured, then lists rows:

```rust
Scene {
    file: "Sample Scenes/Car Hit.ens",   // under Resources/Scenes, or a file name
                                         // in NM_REFERENCE_SCENES for your own
    measured: "a frame-by-frame recording of endorphin's run, ...",
    rows: &[
        Row {
            what: "flight: centre of mass highest",          // for the table
            signal: Signal::Coord(Point::Com("character01"), Axis::Y),
            measure: Measure::Max(48, 100),                  // over frames 48-100
            endorphin: 1.81,                                 // m, degrees or frames
            uncertainty: 0.06,                               // the tolerance
            guard: false,                                    // a known gap
            baseline: 1.616,                                 // NeuroMotion's, then
            source: "recording, silhouette back-projected (lesson 28)",
        },
    ],
},
```

**Points** on a character, by the names the imported scene gives them
(`dump_scene_motion` writes every character's joints and bodies to CSV):
`Point::Joint(character, joint)`, `Point::Body(character, body)` (its centre of mass),
`Point::Com(character)` (the whole character's centre of mass) and
`Point::Mid(&[...])` (the middle of several points).

**Signals**, a number at every frame:

| Signal | Unit |
|---|---|
| `Coord(point, Axis::X / Y / Z)`: a world coordinate (Y is up) | m |
| `Above(a, b)`: how far `a` is above `b` | m |
| `Lowest(character)`: its lowest body centre above y = 0 | m |
| `Elevation(a, b)`: the line `a` → `b` above the horizontal | ° |
| `Pitch(a, b)`: the line `a` → `b` from vertical, positive towards +Z | ° |
| `AimOff { from, tip, target }`: how far `from` → `tip` points off `from` → `target` | ° |
| `FacingOff(character, body, target)`: how far a body's local +Z points off a target (the face, for endorphin's head bodies) | ° |
| `Mean(&[...])`, `Worst(&[...])`: the mean or the largest of several | as theirs |

**Measures**, one number from a signal (frames are the scene's own, endorphin's View
Frame):

| Measure | Gives |
|---|---|
| `At(f)` | the value at frame `f` |
| `Change(a, b)` | the value at `b` minus the value at `a` (angles the short way round) |
| `Max(a, b)`, `Min(a, b)` | the largest or smallest value over frames `a..=b` |
| `MaxFrame(a, b)`, `MinFrame(a, b)` | the frame it happens |
| `FirstBelow { from, to, below }` | the first frame the value is under `below` (when a body touches down, an arm comes within 20° of its target) |
| `LastBelow { from, to, below }` | the last frame of that first run under `below` (when it ends) |

A change (`Change`) is often better than an absolute value: how a camera fitted to a
recording scales heights cancels out, and so does where exactly a silhouette's centre
sits on the body.

## Adding a measurement

From a *Reference measurement* issue, or your own run of endorphin:

1. Find the scene in `SCENES`, or add one (its file, and how it was measured).
2. Write each measurement as a row: the closest signal and measure, endorphin's value,
   an honest uncertainty (how far the reading could be off: a frame or two read off a
   recording's frame counter at 60 fps, a few centimetres for a position fitted from a
   video, more for something read by eye), and in `source` who measured it and how.
3. Run the test, put NeuroMotion's value in `baseline`, and check the row reads what
   you meant (a `FirstBelow` that is `never`, or a value far off, usually means a
   wrong name, axis or sign; `dump_scene_motion` shows the run frame by frame).
4. Leave it a known gap (`guard: false`) unless NeuroMotion is within the uncertainty
   with a margin in every run of `NM_NUDGES=4`; only then make it a guard.

Never put endorphin's files, or images or text taken from them, in the repository.

## What is covered

Run on the integrated build (Coulomb friction, active poses with the whole-limb
cap, Look and Reach, Explosion; physics-design.md lessons up to 28), 2026-11-25:

| Scene | Rows | Within | Guards | Mean error (× uncertainty) |
|---|---|---|---|---|
| Car Hit | 15 | 5 | 5 | 2.0 |
| Back-Flip-To-Combo (#6, the user's own scene) | 9 | 2 | 1 | 2.4 |
| Hands reach and look at 2.5 (Look and Reach) | 3 | 2 | 2 | 1.2 |
| Writhe in mid-air 2.5 (Explosion) | 5 | 3 | 1 | 1.1 |
| Stagger Walk | 4 | 3 | 1 | 0.9 |
| In all | 36 | 15 | 10 | |

The same numbers again on 2026-11-25 with Back-Flip-To-Combo's take-off lesson added
(docs only), and over `NM_NUDGES=2`: every row's nudged range is within two frames, 0.1° or
1 cm of it, except Stagger Walk's. Ways of reading endorphin's numbers that were scored
on all five scenes (lock strength units, damping, strength as a speed, the drive mode,
the swing sizes, the limb spring cap) are compared in physics-design.md, the lesson after
Back-Flip-To-Combo's take-off; none is in yet, so the baselines stay.

The known gaps, with their errors on that run:

- **Car Hit**: the pedestrian flies 0.19 m lower (centre of mass 1.62 m at frame 65,
  endorphin 1.81 m at 68), turns upside down 14 frames early (62, endorphin 76) and
  comes down 7 frames early (89, endorphin 96), 0.42 m further along the road (x
  +0.08 m at frame 96, endorphin -0.34 m). Its centre of mass is 0.25 m low at frame
  78 and 0.44 m at 88, and drifts forward (+0.08 m at 78) where endorphin's goes back
  (-0.11 m).
- **Back-Flip-To-Combo**: it leans back 2-3° more than endorphin's in the first
  second, holds its left arm 19° above the horizontal at frame 96 (endorphin -7°),
  crouches 0.17 m deeper by frame 192 and, instead of springing up 0.65 m, goes over
  the back of the box: it leaves the box top at 242 (endorphin about 212) and reaches
  the floor at 294 (about 276).
- **Look and Reach**: character02's far arm points at the box by frame 31; endorphin's
  waits and gets there at about 55.
- **Explosion**: the arms are highest at frame 16 (endorphin about 25) and go down to
  -34° (endorphin -40 to -58°). The lowest arms' frame (42, endorphin 44) and the
  landing (54, endorphin 56-58) are within their uncertainty but too close to its edge
  to guard.
- **Stagger Walk**: the feet go back about as far as endorphin's by frame 36 (0.43 m,
  endorphin 0.46), but by 96 they have gone 0.66 m back and 0.11 m along +x where
  endorphin's went 0.88 m back and 0.51 m along -x. Standing is on a knife edge:
  with gravity nudged by up to 4e-4, the top of the neck is 0.02 to 0.51 m lower at
  frame 108 than at frame 0 (it stays up in most runs and sinks or falls in others),
  so "still standing" isn't a guard.

## What is still estimated

Every endorphin number here comes from screen recordings of endorphin's runs, read
with its View Frame counter: frames by eye (to a frame or two at 60 fps), positions
from silhouettes back-projected through a camera fitted to the view, angles read in
the image. None comes from an export of endorphin's results (BVH or FBX of the
simulated motion), which would give every joint at every frame to the millimetre;
those would replace these rows' uncertainties with far smaller ones. In particular:

- Car Hit's positions are the silhouette's centroid, not the centre of mass; where
  both follow the same animation (frames 30 and 190) they agree within 6 cm.
- Back-Flip-To-Combo's lean was fitted to the silhouette with the whole start pose
  turned about the heels; NeuroMotion's side measures the pelvis over the heel, which
  agrees while the legs are straight (to about frame 100). Its head heights are image
  pixels at about 86 px a metre.
- Stagger Walk's feet are the footprint of the lower silhouette, and its standing
  height the silhouette's top, which reads about 15% short of NeuroMotion's own run
  drawn the same way; only changes are compared.
- Explosion's arm angles were read in a front view, where an arm out to the side shows
  its elevation directly.

Not yet measured, or not yet usable as a row:

- **Tackle**: the recording is too tangled through the tackle to read the runner's
  hand or head, and the footprints of the two characters lying together can't be told
  apart from each other.
- **Stagger Walk**: endorphin takes about five steps and is reported to turn about 90°;
  how far it turns, which way and by which frame haven't been measured, so neither the
  steps nor the turn is a row yet.
- **Look and Reach**: where the hands are (24 px on average from endorphin's at 14
  points) was measured in the image, which a row can't read.
- The other sample scenes and behaviour scenes haven't been measured at all.
