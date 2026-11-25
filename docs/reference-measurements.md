# Reference measurements

How close NeuroMotion is to endorphin, by numbers. endorphin's runs of its own scenes
were measured (frame numbers, heights, positions, angles), and the test
`reference_scores` simulates the same scenes in NeuroMotion, computes the same
quantities and scores the difference. "Behaves like endorphin" is then checked, not
eyeballed, and every change to the physics or the behaviours shows which way it moved
NeuroMotion.

The measurements are in `src/physics/reference.rs`, one row each. Most are read from
endorphin's own runs exported as FBX (every joint at every frame; physics-design.md,
lesson 40); a few still come from screen recordings. They are numbers only:
endorphin's scenes, exports, recordings and frames stay on the machine that has
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
Sample Scenes/Car Hit.ens (endorphin's own run, exported as FBX (lesson 40))
  quantity                                  endorphin NeuroMotion   error  baseline  within?
  run-up: centre of mass x, frame 30  1.44 m ± 0.03 m      1.44 m -0.00 m    1.44 m  yes (guard)
  flight: centre of mass highest      1.79 m ± 0.03 m      1.73 m -0.06 m    1.73 m  no (known gap)
  ...
  score: 10 of 20 within endorphin's uncertainty; mean error 1.9 times the uncertainty (each row counted up to 4)
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
`Point::Mid(&[...])` (the middle of several points). Bodies are read off the pose the
character is shown in (each body's joint and its frame on it), as endorphin's export
has them: during a transition back to animation that is the blended pose, not where
the simulated bodies are.

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
| `Bend(a, b, c)`: how far the joint at `b` bends, 0 with the three in line (hip, knee, ankle: the knee) | ° |
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

1. Best, export endorphin's result as FBX (the simulated character) and read it with
   `python3 -I tools/fbx_motion.py run.fbx joints.csv`: every joint's world position
   and rotation at every frame, in metres, in the same axes and with the same joint
   names as NeuroMotion's import. Compare it with `dump_scene_motion`'s `joints.csv`;
   work out centres of mass from the joints with the rig in `rig.csv` (each body's
   frame on its joint and its mass), as the test does. Where the character starts
   animated, the export's first key is its bind pose and scene frame f is key f + 1
   (docs/endorphin-formats.md, "Exported motion"). Otherwise measure a recording.
2. Find the scene in `SCENES`, or add one (its file, and how it was measured).
3. Write each measurement as a row: the closest signal and measure, endorphin's value,
   an uncertainty, and in `source` who measured it and how. For an export the
   uncertainty is how close counts as the same (a few centimetres, a few degrees, two
   or three frames; more after a tumble); for a recording, how far the reading could
   be off (a frame or two at 60 fps, a few centimetres for a position fitted from a
   video, more for something read by eye).
4. Run the test, put NeuroMotion's value in `baseline`, and check the row reads what
   you meant (a `FirstBelow` that is `never`, or a value far off, usually means a
   wrong name, axis or sign; `dump_scene_motion` shows the run frame by frame).
5. Leave it a known gap (`guard: false`) unless NeuroMotion is within the uncertainty
   with a margin in every run of `NM_NUDGES=4`; only then make it a guard.

Never put endorphin's files, exports, or images or text taken from them, in the
repository.

## What is covered

Run with active poses bending swing joints at a sixteenth of their preset
(physics-design.md, lesson 46), drives fading as cos(θ/2) (lesson 47) and imported
scenes' contacts at 120 Hz (lesson 48), 2026-11-25, with `NM_NUDGES=2`:

| Scene | Rows | Within | Guards | Mean error (× uncertainty) |
|---|---|---|---|---|
| Car Hit | 20 | 9 | 5 | 1.8 |
| Back-Flip-To-Combo (#6, the user's own scene) | 21 | 10 | 5 | 1.3 |
| Tackle (the tackled character) | 9 | 4 | 0 | 2.1 |
| Hands reach and look at 2.5 (Look and Reach) | 3 | 2 | 1 | 1.4 |
| Writhe in mid-air 2.5 (Explosion) | 5 | 3 | 3 | 1.2 |
| Stagger Walk | 4 | 1 | 0 | 2.9 |
| Trapeze (lesson 43) | 6 | 0 | 0 | 4.0 |
| Balcony (character01, lesson 43) | 3 | 0 | 0 | 4.0 |
| In all | 71 | 29 | 14 | |

69 rows come from endorphin's own runs exported as FBX (lessons 40 and 43); character02's two
rows in Hands reach and look at still come from a recording (the export holds
character01 only). The rows used to be 36, all from recordings (18 within on the build
with lessons up to 39); the exports showed several of them were misread (lesson 40),
among them a Stagger Walk guard. With locks read in newtons the first 59 rows scored 21
within too, with Back-Flip-To-Combo 1 of 18 (3.2) and Tackle 4 of 9 (1.5); before
lesson 42, Car Hit 10 of 20 (1.9) and Tackle 3 of 9 (2.3). Summed over the six first
scenes, each row's error in uncertainties (up to 4) went from 129.7 to 123.2 with
lesson 42; over all eight, lesson 46 took it from 159.9 to 140.1 (21 to 29 rows
within, all of them Back-Flip-To-Combo's). Three rows on the flip's turn came after
it (71 rows, the sum 150.4; it would have been 171.0 before lesson 46, where the flip
turned too little: those rows count at most 4 each either way); lesson 47's drive
reading took it to 145.0, and lesson 48's stiffer contacts to 140.6. Ways of
reading endorphin's numbers scored on the recordings are in lessons 34 and 39. The
baselines are this run's values; every guard holds over `NM_NUDGES=2`, where every
row's nudged range is within a frame, 0.1° or 2 cm except Stagger Walk's feet (-0.67 to
-0.50 m along z and 0.06 to 0.11 m along x by frame 96) and the top of its neck (-0.13
to -0.02 m by 108), Tackle's lying x (0.79 to 0.82 m), Trapeze's landing and standing
(frames 362 to 433, 0.26 to 4.10 m high at 400, 0.19 to 0.34 m at 600: it stays on
the platform or falls off it) and Back-Flip-To-Combo's combo (0.62 to 0.70 m).

The known gaps, with their errors on that run:

- **Car Hit**: the car carries the pedestrian along the road and sideways (centre of
  mass x +0.19 m at frame 68, z -0.20 m; endorphin -0.07, -0.06), because its left
  foot sticks to the windscreen's face where endorphin's slides up it (lesson 44); it
  flies 5 cm higher (1.84 m at 70, endorphin 1.79 m at 68; 1.73 m before lesson 42)
  and folds 20° less at the hips.
- **Back-Flip-To-Combo**: since lesson 46 it holds its poses as endorphin's does
  until the take-off (its lean, arms, crouch height, hip and spine bends all within)
  and springs up off the box; since lesson 48 its feet stay on the box top and the
  crouch keeps its centre of mass 0.17 m behind the heels (endorphin 0.164 m; 0.21
  before). It jumps to 3.69 m at frame 236 where endorphin's reaches 3.745 m at 242
  (3.61 m at 233 before): its take-off pushes in a spike where endorphin's pushes a
  steady 2.9 kN, with the hips 15° and the arms 10° short of endorphin's reach, and
  its ankles push where endorphin's are limp (lesson 49). It falls past the box top at
  292 (296) and reaches the floor at 328 (330). Its knees bend 6.5° less in the
  crouch, and its combo jump reaches 0.70 m where endorphin's reaches 1.67 m (0.93 m
  before lesson 48, inside the nudged spread of 0.47 to 0.88 m that row had). With
  the flip's drive read as lesson 47 found it, the trunk turns -263 / -527 / -655°
  by frames 260 / 297 / 330 since 190 (endorphin -247 / -523 / -689; -270 / -655 /
  -863 after lesson 46, -216 / -466 / -593 before it): on time to frame 297, then
  34° short by 330, where endorphin's flip gains angular momentum with nothing
  touching it (lesson 49).
- **Tackle**: the tackled runner goes down later and slower (centre of mass 0.83 m at
  frame 55, endorphin 0.70; head 0.70 m over it at 60, 0.46) and ends up 0.40 m
  further along z (0.28 m before lesson 48's contacts, 0.53 m before lesson 42). The tackler runs endorphin's Tackle
  behaviour, which NeuroMotion doesn't have.
- **Look and Reach**: character01's arms reach the box at frame 17 (endorphin 14);
  character02's far arm points at the box by frame 29, where endorphin's waits and
  gets there at about 55 (recording).
- **Explosion**: the arms are highest at frame 16 (endorphin 28) and lowest at 42 (47).
- **Stagger Walk**: the feet go 0.41 m back by frame 36 (endorphin 0.26 m; 0.59 m
  before lesson 48) and by 96 only 0.65 m back and 0.07 m along +x (endorphin 1.03 m
  back, 0.58 m along -x).
  endorphin's steps one foot at a time, five steps from frame 12; NeuroMotion's
  shuffles both feet from the first frame (lesson 44).
- **Trapeze**: endorphin's acrobat jumps off the platform (JumpDive, which NeuroMotion
  doesn't have), swings on the bar and lands at 341; NeuroMotion's stays on the
  platform, then falls off it at 362.
- **Balcony**: endorphin's character01 goes over the railing and is on the ground at
  frame 96; NeuroMotion's falls on the balcony (its Arms Windmill isn't a behaviour
  NeuroMotion has). The export's first frames don't line up with NeuroMotion's
  (lesson 43), so its rows are changes with wide uncertainties.

## What is still estimated

- Character02 in Hands reach and look at: two rows from the recording (frames read by
  eye with endorphin's frame counter).
- The exports hold one character each: Tackle's tackler, Look and Reach's character02
  and Balcony's seated characters (the only ones there whose poses drive virtual
  muscles) aren't measured from them.
- Stagger Walk's turn and steps, and the hands' positions in Look and Reach, aren't
  rows yet; the other sample scenes and behaviour scenes haven't been measured.
