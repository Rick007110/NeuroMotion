//! Reference measurements: what endorphin does in its own scenes, measured from its
//! runs, and the test that scores NeuroMotion against them (`reference_scores`).
//!
//! Each [`Scene`] names an endorphin scene file and lists [`Row`]s: a quantity
//! ([`Signal`] measured per frame, reduced by a [`Measure`]), endorphin's value, how
//! uncertain that value is, and where it came from. The test simulates the scene as
//! `dump_scene_motion` does, computes each quantity on NeuroMotion's run and prints
//! endorphin's value, NeuroMotion's, the error and whether it is within the
//! uncertainty, with a score per scene.
//!
//! A row is a `guard` when NeuroMotion already matches it with room to spare (the
//! error plus how far the value moves when the scene is nudged stays inside the
//! uncertainty): the test fails if a guard falls outside. Every other row is a known
//! gap: printed with its error, never failed, so progress shows without faking a
//! pass. `baseline` is NeuroMotion's value when the row was written or last
//! reviewed, so a change can be read off against it.
//!
//! Adding a measurement (from a *Reference measurement* issue, say): find the scene
//! below or add one, write the quantity with the signals and measures here, put
//! endorphin's number and an honest uncertainty, say who measured it and how in
//! `source`, run the test and write NeuroMotion's value into `baseline`. Make it a
//! guard only if it is within its uncertainty with a margin over the nudged runs
//! (`NM_NUDGES=2`). docs/reference-measurements.md has the details.
//!
//! Only numbers go here: never endorphin's files, text or images.

use std::path::{Path, PathBuf};

use glam::{Quat, Vec3};

use super::scenes::{nudges, simulate};
use super::sim::{CharacterFrame, Simulation};
use crate::document::Project;

/// A point on a character (or prop) at a frame. Characters, joints and bodies go by
/// the names the imported scene gives them (`dump_scene_motion` lists them).
#[derive(Clone, Copy, Debug)]
pub enum Point {
    /// A skeleton joint's world position: (character, joint).
    Joint(&'static str, &'static str),
    /// A rig body's centre of mass: (character, body).
    Body(&'static str, &'static str),
    /// A character's centre of mass, over all its rig's bodies.
    Com(&'static str),
    /// The middle of several points.
    Mid(&'static [Point]),
}

#[derive(Clone, Copy, Debug)]
pub enum Axis {
    X,
    Y,
    Z,
}

/// A number NeuroMotion's run gives at every frame.
#[derive(Clone, Copy, Debug)]
pub enum Signal {
    /// A point's world coordinate, m (Y is up).
    Coord(Point, Axis),
    /// How far the first point is above the second, m.
    Above(Point, Point),
    /// A character's lowest body centre, m above y = 0.
    Lowest(&'static str),
    /// The line from the first point to the second, degrees above the horizontal.
    Elevation(Point, Point),
    /// The line from the first point to the second, degrees from vertical, positive
    /// leaning towards +Z (endorphin's characters mostly face +Z).
    Pitch(Point, Point),
    /// How far the line `from` → `tip` points off the line `from` → `target`, degrees.
    AimOff {
        from: Point,
        tip: Point,
        target: Point,
    },
    /// How far a body's local +Z axis points off the line to a target, degrees:
    /// (character, body, target). On endorphin's characters the head body's +Z is the
    /// way the face looks.
    FacingOff(&'static str, &'static str, Point),
    /// How far a joint bends, degrees: the angle between the line from the middle point
    /// back to the first and the line from it on to the third, 0 when they are in line
    /// (hip, knee, ankle gives the knee's bend).
    Bend(Point, Point, Point),
    /// The mean of several signals.
    Mean(&'static [Signal]),
    /// The largest of several signals (e.g. the arm furthest off a target).
    Worst(&'static [Signal]),
}

/// How a row turns a signal into one number. Frames are the scene's own frame
/// numbers (endorphin's View Frame).
#[derive(Clone, Copy, Debug)]
pub enum Measure {
    /// The value at a frame.
    At(i32),
    /// The value at the second frame minus the value at the first.
    Change(i32, i32),
    /// The largest value over frames `from..=to`.
    Max(i32, i32),
    /// The frame of the largest value over `from..=to`.
    MaxFrame(i32, i32),
    /// The smallest value over `from..=to`.
    Min(i32, i32),
    /// The frame of the smallest value over `from..=to`.
    MinFrame(i32, i32),
    /// The first frame in `from..=to` where the value is below `below`.
    FirstBelow { from: i32, to: i32, below: f32 },
    /// The last frame of the first run of frames in `from..=to` below `below`.
    LastBelow { from: i32, to: i32, below: f32 },
}

/// One measurement of endorphin's run.
#[derive(Clone, Copy, Debug)]
pub struct Row {
    /// What it is, in a few words, for the table.
    pub what: &'static str,
    pub signal: Signal,
    pub measure: Measure,
    /// endorphin's value, in the unit the measure gives (m, degrees or frames).
    pub endorphin: f32,
    /// How far endorphin's real value may be from `endorphin`, same unit: the
    /// tolerance NeuroMotion is judged by.
    pub uncertainty: f32,
    /// Fail the test when NeuroMotion falls outside the uncertainty.
    pub guard: bool,
    /// NeuroMotion's value when the row was written or last reviewed.
    pub baseline: f32,
    /// Who measured it and how.
    pub source: &'static str,
}

/// An endorphin scene and what was measured in it.
pub struct Scene {
    /// Its file: a path under endorphin's `Resources/Scenes` (`Sample Scenes/Car
    /// Hit.ens`), or for a user's own scene a file name looked for in the folder
    /// `NM_REFERENCE_SCENES` names.
    pub file: &'static str,
    /// How the scene was measured, for the table's heading.
    pub measured: &'static str,
    pub rows: &'static [Row],
}

// Most rows are read from endorphin 2.5.2's own runs, exported as FBX by a user with
// endorphin (docs/endorphin-formats.md, "Exported motion"; physics-design.md, lesson
// 40): every joint's world transform at every frame, to the file's float precision.
// The centre of mass and body centres are worked out from those joints with
// NeuroMotion's own rig (body frames and masses), exactly as on NeuroMotion's run, so
// the two compare like with like. The uncertainty is then not endorphin's measurement
// error but how close counts as the same: 3 cm, 1-4° and 2-3 frames where nothing
// chaotic happens, wider after a landing or a tumble. Rows still read from recordings
// say so in `source`.
const FBX: &str = "endorphin's own run, exported as FBX (lesson 40)";

/// A row read from endorphin's own run, exported as FBX.
#[allow(clippy::too_many_arguments)]
const fn fbx(
    what: &'static str,
    signal: Signal,
    measure: Measure,
    endorphin: f32,
    uncertainty: f32,
    guard: bool,
    baseline: f32,
) -> Row {
    Row {
        what,
        signal,
        measure,
        endorphin,
        uncertainty,
        guard,
        baseline,
        source: FBX,
    }
}

const fn com(axis: Axis) -> Signal {
    Signal::Coord(Point::Com("character01"), axis)
}

const fn joint(name: &'static str) -> Point {
    Point::Joint("character01", name)
}

/// The bend at a joint, degrees: 0 when the three joints are in line.
const fn bend(a: &'static str, b: &'static str, c: &'static str) -> Signal {
    Signal::Bend(joint(a), joint(b), joint(c))
}

const HEAD_OVER_COM: Signal = Signal::Above(
    Point::Body("character01", "HeadMass"),
    Point::Com("character01"),
);

/// What is measured, scene by scene. See the module's documentation to add to it.
pub const SCENES: &[Scene] = &[
    Scene {
        file: "Sample Scenes/Car Hit.ens",
        measured: "endorphin's own run, exported as FBX (lesson 40)",
        rows: &[
            fbx(
                "run-up: centre of mass x, frame 30",
                com(Axis::X),
                Measure::At(30),
                1.437,
                0.03,
                true,
                1.437,
            ),
            fbx(
                "run-up: centre of mass height, frame 30",
                com(Axis::Y),
                Measure::At(30),
                0.868,
                0.03,
                true,
                0.868,
            ),
            fbx(
                "flight: centre of mass highest",
                com(Axis::Y),
                Measure::Max(48, 100),
                1.790,
                0.03,
                false,
                1.83,
            ),
            fbx(
                "flight: frame it is highest",
                com(Axis::Y),
                Measure::MaxFrame(48, 100),
                68.0,
                2.0,
                false,
                70.0,
            ),
            fbx(
                "flight: centre of mass x, frame 68",
                com(Axis::X),
                Measure::At(68),
                -0.073,
                0.05,
                false,
                0.20,
            ),
            fbx(
                "flight: centre of mass x, frame 78",
                com(Axis::X),
                Measure::At(78),
                -0.114,
                0.05,
                false,
                0.31,
            ),
            fbx(
                "flight: centre of mass x, frame 88",
                com(Axis::X),
                Measure::At(88),
                -0.155,
                0.05,
                false,
                0.42,
            ),
            fbx(
                "flight: centre of mass z, frame 68",
                com(Axis::Z),
                Measure::At(68),
                -0.056,
                0.05,
                false,
                -0.21,
            ),
            fbx(
                "flight: centre of mass height, frame 78",
                com(Axis::Y),
                Measure::At(78),
                1.667,
                0.05,
                false,
                1.74,
            ),
            fbx(
                "flight: centre of mass height, frame 88",
                com(Axis::Y),
                Measure::At(88),
                1.270,
                0.05,
                false,
                1.37,
            ),
            fbx(
                "flight: left knee bend, frame 60",
                bend("LeftHipJoint", "LeftKneeJoint", "LeftAnkleJoint"),
                Measure::At(60),
                61.1,
                4.0,
                false,
                59.2,
            ),
            fbx(
                "flight: right knee bend, frame 60",
                bend("RightHipJoint", "RightKneeJoint", "RightAnkleJoint"),
                Measure::At(60),
                32.6,
                4.0,
                false,
                32.1,
            ),
            fbx(
                "flight: trunk to left thigh bend, frame 70",
                bend("UpperSpineJoint", "LeftHipJoint", "LeftKneeJoint"),
                Measure::At(70),
                96.5,
                4.0,
                false,
                76.4,
            ),
            fbx(
                "upside down (head under the centre of mass) from",
                HEAD_OVER_COM,
                Measure::FirstBelow {
                    from: 48,
                    to: 120,
                    below: 0.0,
                },
                60.0,
                2.0,
                false,
                63.0,
            ),
            fbx(
                "upside down until",
                HEAD_OVER_COM,
                Measure::LastBelow {
                    from: 48,
                    to: 120,
                    below: 0.0,
                },
                83.0,
                2.0,
                true,
                83.0,
            ),
            fbx(
                "first on the ground (a body centre under 6 cm)",
                Signal::Lowest("character01"),
                Measure::FirstBelow {
                    from: 60,
                    to: 140,
                    below: 0.06,
                },
                96.0,
                2.0,
                false,
                96.0,
            ),
            fbx(
                "landed: centre of mass x, frame 96",
                com(Axis::X),
                Measure::At(96),
                -0.358,
                0.05,
                false,
                0.20,
            ),
            fbx(
                "landed: centre of mass z, frame 96",
                com(Axis::Z),
                Measure::At(96),
                0.063,
                0.05,
                false,
                -0.26,
            ),
            fbx(
                "back on its feet: centre of mass x, frame 190",
                com(Axis::X),
                Measure::At(190),
                -1.517,
                0.03,
                true,
                -1.517,
            ),
            fbx(
                "back on its feet: centre of mass height, frame 190",
                com(Axis::Y),
                Measure::At(190),
                0.948,
                0.03,
                true,
                0.948,
            ),
        ],
    },
    Scene {
        file: "Back-Flip-To-Combo.ens",
        measured: "a user's own scene (#6): endorphin's own run, exported as FBX (lesson 40)",
        rows: &[
            fbx(
                "lean back about the heels since frame 0, frame 44",
                BACKFLIP_LEAN,
                Measure::Change(0, 44),
                -2.3,
                1.0,
                false,
                -3.7,
            ),
            fbx(
                "lean back about the heels since frame 0, frame 72",
                BACKFLIP_LEAN,
                Measure::Change(0, 72),
                -5.3,
                1.0,
                false,
                -6.7,
            ),
            fbx(
                "lean back about the heels since frame 0, frame 100",
                BACKFLIP_LEAN,
                Measure::Change(0, 100),
                -6.4,
                1.0,
                false,
                -7.9,
            ),
            fbx(
                "left arm (shoulder to hand) above horizontal, frame 96",
                Signal::Elevation(joint("LUpperArm"), joint("LHand")),
                Measure::At(96),
                -2.9,
                3.0,
                false,
                23.9,
            ),
            fbx(
                "crouching: centre of mass height, frame 168",
                com(Axis::Y),
                Measure::At(168),
                3.159,
                0.03,
                false,
                3.195,
            ),
            fbx(
                "crouched: centre of mass height, frame 192",
                com(Axis::Y),
                Measure::At(192),
                3.158,
                0.03,
                false,
                3.186,
            ),
            fbx(
                "crouched: centre of mass z (behind the heels < -0.03), frame 192",
                com(Axis::Z),
                Measure::At(192),
                -0.164,
                0.05,
                true,
                -0.155,
            ),
            fbx(
                "crouched: knee bend, frame 170",
                bend("LThigh", "LCalf", "LFoot"),
                Measure::At(170),
                111.8,
                4.0,
                false,
                106.4,
            ),
            fbx(
                "crouched: hip bend (trunk to thigh), frame 170",
                bend("Spine2", "LThigh", "LCalf"),
                Measure::At(170),
                90.3,
                4.0,
                false,
                74.8,
            ),
            fbx(
                "crouched: spine bend (pelvis, chest, head), frame 185",
                bend("Pelvis", "Spine2", "Head"),
                Measure::At(185),
                48.2,
                4.0,
                false,
                39.0,
            ),
            fbx(
                "take-off: hips straightest",
                bend("Spine2", "LThigh", "LCalf"),
                Measure::Min(195, 235),
                3.9,
                4.0,
                false,
                16.1,
            ),
            fbx(
                "take-off: arms overhead (shoulder bend least)",
                bend("Spine1", "LUpperArm", "LForeArm"),
                Measure::Min(200, 240),
                60.6,
                4.0,
                false,
                88.5,
            ),
            fbx(
                "jump: centre of mass highest",
                com(Axis::Y),
                Measure::Max(192, 300),
                3.745,
                0.05,
                false,
                3.543,
            ),
            fbx(
                "jump: frame it is highest",
                com(Axis::Y),
                Measure::MaxFrame(192, 300),
                242.0,
                3.0,
                false,
                224.0,
            ),
            fbx(
                "jump: lowest body centre highest (box top 2.52 m)",
                Signal::Lowest("character01"),
                Measure::Max(192, 300),
                3.602,
                0.05,
                false,
                3.387,
            ),
            fbx(
                "falls past the box top (a body centre 10 cm under it)",
                Signal::Lowest("character01"),
                Measure::FirstBelow {
                    from: 100,
                    to: 500,
                    below: 2.42,
                },
                296.0,
                3.0,
                false,
                270.0,
            ),
            fbx(
                "first on the floor (a body centre under 15 cm)",
                Signal::Lowest("character01"),
                Measure::FirstBelow {
                    from: 100,
                    to: 726,
                    below: 0.15,
                },
                330.0,
                3.0,
                false,
                314.0,
            ),
            fbx(
                "combo: centre of mass highest, frames 500-620",
                com(Axis::Y),
                Measure::Max(500, 620),
                1.667,
                0.1,
                false,
                0.560,
            ),
        ],
    },
    Scene {
        file: "Sample Scenes/Tackle.ens",
        measured: "endorphin's own run of the tackled character, exported as FBX (lesson 40)",
        rows: &[
            fbx(
                "hit: centre of mass height, frame 55",
                com(Axis::Y),
                Measure::At(55),
                0.698,
                0.03,
                false,
                0.83,
            ),
            fbx(
                "hit: centre of mass height, frame 65",
                com(Axis::Y),
                Measure::At(65),
                0.430,
                0.05,
                false,
                0.586,
            ),
            fbx(
                "hit: head over the centre of mass, frame 60",
                HEAD_OVER_COM,
                Measure::At(60),
                0.464,
                0.05,
                false,
                0.70,
            ),
            fbx(
                "down: centre of mass lowest, frames 60-100",
                com(Axis::Y),
                Measure::Min(60, 100),
                0.219,
                0.03,
                false,
                0.239,
            ),
            fbx(
                "down: frame it is lowest",
                com(Axis::Y),
                Measure::MinFrame(60, 100),
                76.0,
                3.0,
                false,
                76.0,
            ),
            fbx(
                "down: head under the centre of mass from",
                HEAD_OVER_COM,
                Measure::FirstBelow {
                    from: 41,
                    to: 100,
                    below: 0.0,
                },
                75.0,
                2.0,
                false,
                76.0,
            ),
            fbx(
                "carried: centre of mass z, frame 70",
                com(Axis::Z),
                Measure::At(70),
                1.514,
                0.05,
                false,
                1.58,
            ),
            fbx(
                "lying: centre of mass x, frame 100",
                com(Axis::X),
                Measure::At(100),
                0.785,
                0.1,
                false,
                0.75,
            ),
            fbx(
                "lying: centre of mass z, frame 100",
                com(Axis::Z),
                Measure::At(100),
                1.650,
                0.1,
                false,
                1.93,
            ),
        ],
    },
    Scene {
        file: "Behaviour Scenes/Hands reach and look at2.5.ens",
        measured: "character01: endorphin's own run, exported as FBX (lesson 40); \
                   character02: a recording of endorphin's demo scene with its frame \
                   counter, a camera fitted to the view (behaviours.md, Look and Reach)",
        rows: &[
            Row {
                what: "character01: both arms within 20° of the box from",
                signal: Signal::Worst(&[
                    reach_arm("character01", "Left"),
                    reach_arm("character01", "Right"),
                ]),
                measure: Measure::FirstBelow {
                    from: 0,
                    to: 150,
                    below: 20.0,
                },
                endorphin: 14.0,
                uncertainty: 3.0,
                guard: false,
                baseline: 17.0,
                source: FBX,
            },
            Row {
                what: "character02: far (left) arm within 20° of the box from",
                signal: reach_arm("character02", "Left"),
                measure: Measure::FirstBelow {
                    from: 0,
                    to: 150,
                    below: 20.0,
                },
                endorphin: 55.0,
                uncertainty: 8.0,
                guard: false,
                baseline: 29.0,
                source: "recording, by eye: out until 24-30, across the chest at 40, on \
                         the box by about 60",
            },
            Row {
                what: "character02: head faces within 20° of the box from",
                signal: Signal::FacingOff("character02", "HeadMass", REACH_TARGET),
                measure: Measure::FirstBelow {
                    from: 0,
                    to: 150,
                    below: 20.0,
                },
                endorphin: 15.0,
                uncertainty: 3.0,
                guard: true,
                baseline: 15.0,
                source: "recording, by eye: faces the box by frame 12-18",
            },
        ],
    },
    Scene {
        file: "Behaviour Scenes/Writhe in mid-air2.5.ens",
        measured: "endorphin's own run, exported as FBX (lesson 40)",
        rows: &[
            Row {
                what: "character01: upper arms highest (mean, above horizontal)",
                signal: WRITHE_ARMS,
                measure: Measure::Max(5, 35),
                endorphin: 36.7,
                uncertainty: 3.0,
                guard: true,
                baseline: 37.1,
                source: FBX,
            },
            Row {
                what: "character01: frame the arms are highest",
                signal: WRITHE_ARMS,
                measure: Measure::MaxFrame(5, 35),
                endorphin: 28.0,
                uncertainty: 3.0,
                guard: false,
                baseline: 16.0,
                source: FBX,
            },
            Row {
                what: "character01: upper arms lowest",
                signal: WRITHE_ARMS,
                measure: Measure::Min(30, 56),
                endorphin: -33.5,
                uncertainty: 3.0,
                guard: true,
                baseline: -33.0,
                source: FBX,
            },
            Row {
                what: "character01: frame the arms are lowest",
                signal: WRITHE_ARMS,
                measure: Measure::MinFrame(30, 56),
                endorphin: 47.0,
                uncertainty: 3.0,
                guard: false,
                baseline: 42.0,
                source: FBX,
            },
            Row {
                what: "character01: lands (a body centre under 10 cm)",
                signal: Signal::Lowest("character01"),
                measure: Measure::FirstBelow {
                    from: 10,
                    to: 120,
                    below: 0.1,
                },
                endorphin: 54.0,
                uncertainty: 2.0,
                guard: true,
                baseline: 54.0,
                source: FBX,
            },
        ],
    },
    Scene {
        file: "Sample Scenes/Stagger Walk.ens",
        measured: "endorphin's own run, exported as FBX (lesson 40)",
        rows: &[
            Row {
                what: "feet moved back (z), frames 0 to 36",
                signal: Signal::Coord(STAGGER_FEET, Axis::Z),
                measure: Measure::Change(0, 36),
                endorphin: -0.264,
                uncertainty: 0.05,
                guard: false,
                baseline: -0.589,
                source: FBX,
            },
            Row {
                what: "feet moved back (z), frames 0 to 96",
                signal: Signal::Coord(STAGGER_FEET, Axis::Z),
                measure: Measure::Change(0, 96),
                endorphin: -1.026,
                uncertainty: 0.1,
                guard: false,
                baseline: -0.557,
                source: FBX,
            },
            Row {
                what: "feet moved sideways (x), frames 0 to 96",
                signal: Signal::Coord(STAGGER_FEET, Axis::X),
                measure: Measure::Change(0, 96),
                endorphin: -0.577,
                uncertainty: 0.1,
                guard: false,
                baseline: 0.378,
                source: FBX,
            },
            Row {
                what: "still standing: top of the neck, change frames 0 to 108",
                signal: Signal::Coord(Point::Joint("character01", "UpperNeckJoint"), Axis::Y),
                measure: Measure::Change(0, 108),
                endorphin: -0.042,
                uncertainty: 0.03,
                guard: false,
                baseline: -0.042,
                source: FBX,
            },
        ],
    },
    Scene {
        file: "Sample Scenes/Trapeze.ens",
        measured: "endorphin's own run, exported as FBX (lesson 43)",
        rows: &[
            // endorphin's acrobat jumps off the platform (JumpDive, frames 50-108, a
            // behaviour NeuroMotion doesn't have yet), catches the bar, swings across
            // and lands on the far side, where Balance holds it up.
            Row {
                what: "jump: centre of mass along z, frames 0 to 160",
                signal: Signal::Coord(Point::Com("character01"), Axis::Z),
                measure: Measure::Change(0, 160),
                endorphin: 4.126,
                uncertainty: 0.3,
                guard: false,
                baseline: 0.15,
                source: FBX,
            },
            Row {
                what: "swing: centre of mass lowest, frames 150-260",
                signal: Signal::Coord(Point::Com("character01"), Axis::Y),
                measure: Measure::Min(150, 260),
                endorphin: 2.392,
                uncertainty: 0.1,
                guard: false,
                baseline: 4.37,
                source: FBX,
            },
            Row {
                what: "swing: centre of mass z, frame 300",
                signal: Signal::Coord(Point::Com("character01"), Axis::Z),
                measure: Measure::At(300),
                endorphin: 4.472,
                uncertainty: 0.2,
                guard: false,
                baseline: -9.03,
                source: FBX,
            },
            Row {
                what: "lands (a body centre under 15 cm)",
                signal: Signal::Lowest("character01"),
                measure: Measure::FirstBelow {
                    from: 300,
                    to: 600,
                    below: 0.15,
                },
                endorphin: 341.0,
                uncertainty: 3.0,
                guard: false,
                baseline: 369.0,
                source: FBX,
            },
            Row {
                what: "standing: centre of mass height, frame 400",
                signal: Signal::Coord(Point::Com("character01"), Axis::Y),
                measure: Measure::At(400),
                endorphin: 0.736,
                uncertainty: 0.05,
                guard: false,
                baseline: 0.48,
                source: FBX,
            },
            Row {
                what: "standing: centre of mass height, frame 600",
                signal: Signal::Coord(Point::Com("character01"), Axis::Y),
                measure: Measure::At(600),
                endorphin: 0.732,
                uncertainty: 0.05,
                guard: false,
                baseline: 0.20,
                source: FBX,
            },
        ],
    },
    Scene {
        file: "Sample Scenes/Balcony.ens",
        measured: "endorphin's own run of character01, exported as FBX (lesson 43)",
        rows: &[
            // endorphin's character01 goes over the railing and falls to the ground.
            // The export's first key is 0.26 m further along -z than NeuroMotion's
            // frame 0 and holds 309 keys for the scene's 303 frames, so frames are
            // read with a few frames' uncertainty and positions as changes.
            Row {
                what: "falls: first on the ground (a body centre under 15 cm)",
                signal: Signal::Lowest("character01"),
                measure: Measure::FirstBelow {
                    from: 0,
                    to: 302,
                    below: 0.15,
                },
                endorphin: 96.0,
                uncertainty: 5.0,
                guard: false,
                baseline: f32::NAN, // never on the ground: it stays on the balcony
                source: FBX,
            },
            Row {
                what: "lying: centre of mass height, frame 200",
                signal: Signal::Coord(Point::Com("character01"), Axis::Y),
                measure: Measure::At(200),
                endorphin: 0.132,
                uncertainty: 0.05,
                guard: false,
                baseline: 3.15,
                source: FBX,
            },
            Row {
                what: "lying: centre of mass along z, frames 0 to 200",
                signal: Signal::Coord(Point::Com("character01"), Axis::Z),
                measure: Measure::Change(0, 200),
                endorphin: -3.756,
                uncertainty: 0.3,
                guard: false,
                baseline: -1.16,
                source: FBX,
            },
        ],
    },
];

// Back-Flip-To-Combo's character: the pelvis over the left heel, how far its whole body
// leans back about the heels while its legs are straight (to about frame 100).
const BACKFLIP_LEAN: Signal = Signal::Pitch(
    Point::Joint("character01", "LFoot"),
    Point::Joint("character01", "Pelvis"),
);

const REACH_TARGET: Point = Point::Body("background", "Prop0");

/// How far an arm of the demo's characters (shoulder joint to finger tips) points off
/// the box.
const fn reach_arm(character: &'static str, side: &'static str) -> Signal {
    let (shoulder, fingers) = match side.as_bytes()[0] {
        b'L' => ("LeftShoulderJoint", "LeftFingersMass"),
        _ => ("RightShoulderJoint", "RightFingersMass"),
    };
    Signal::AimOff {
        from: Point::Joint(character, shoulder),
        tip: Point::Body(character, fingers),
        target: REACH_TARGET,
    }
}

// The left character, which starts in a T-pose: its upper arms (shoulder to elbow)
// above the horizontal, the two averaged; they swing together.
const WRITHE_ARMS: Signal = Signal::Mean(&[
    Signal::Elevation(
        Point::Joint("character01", "LeftShoulderJoint"),
        Point::Joint("character01", "LeftElbowJoint"),
    ),
    Signal::Elevation(
        Point::Joint("character01", "RightShoulderJoint"),
        Point::Joint("character01", "RightElbowJoint"),
    ),
]);

const STAGGER_FEET: Point = Point::Mid(&[
    Point::Joint("character01", "LeftAnkleJoint"),
    Point::Joint("character01", "RightAnkleJoint"),
]);

impl Measure {
    /// Whether it gives a frame number rather than the signal's value.
    fn is_frame(self) -> bool {
        matches!(
            self,
            Measure::MaxFrame(..)
                | Measure::MinFrame(..)
                | Measure::FirstBelow { .. }
                | Measure::LastBelow { .. }
        )
    }

    /// The last frame it reads.
    fn last_frame(self) -> i32 {
        match self {
            Measure::At(f) => f,
            Measure::Change(a, b) => a.max(b),
            Measure::Max(_, to)
            | Measure::MaxFrame(_, to)
            | Measure::Min(_, to)
            | Measure::MinFrame(_, to)
            | Measure::FirstBelow { to, .. }
            | Measure::LastBelow { to, .. } => to,
        }
    }
}

impl Signal {
    fn unit(self) -> &'static str {
        match self {
            Signal::Coord(..) | Signal::Above(..) | Signal::Lowest(_) => "m",
            Signal::Mean(s) | Signal::Worst(s) => s[0].unit(),
            _ => "°",
        }
    }
}

impl Row {
    fn unit(&self) -> &'static str {
        if self.measure.is_frame() {
            "frames"
        } else {
            self.signal.unit()
        }
    }

    /// A value in the row's unit, for the table.
    fn show(&self, value: f32) -> String {
        match self.unit() {
            "frames" => format!("{value:.0}"),
            "m" => format!("{value:.2} m"),
            _ => format!("{value:.1}°"),
        }
    }
}

/// A simulated scene, read by signals.
struct Run<'a> {
    project: &'a Project,
    sim: Simulation,
}

impl Run<'_> {
    fn character(&self, name: &str) -> usize {
        self.project
            .characters
            .iter()
            .position(|c| c.name == name)
            .unwrap_or_else(|| panic!("{}: no character {name:?}", self.project.settings.name))
    }

    fn state(&self, character: usize, frame: i32) -> &CharacterFrame {
        self.sim.frame(frame).unwrap()[character]
            .as_ref()
            .unwrap_or_else(|| {
                panic!(
                    "{}: {} has no physics rig to measure",
                    self.project.settings.name, self.project.characters[character].name
                )
            })
    }

    /// Every rig body's world transform on the pose the character is shown in: its joint
    /// and its frame on that joint. While simulated that is where the body is; during a
    /// transition back to animation it is the blended pose, which is what endorphin
    /// shows and exports, and so what its measurements were read on.
    fn bodies(&self, character: usize, frame: i32) -> Vec<(Quat, Vec3)> {
        let rig = self.project.characters[character].physics.as_ref().unwrap();
        let joints = &self.state(character, frame).joints;
        rig.bodies
            .iter()
            .map(|def| {
                let (jr, jp) = joints[def.joint];
                (
                    jr * Quat::from_array(def.frame.rotation).normalize(),
                    jp + jr * Vec3::from(def.frame.position),
                )
            })
            .collect()
    }

    fn body(&self, character: &str, body: &str, frame: i32) -> (Quat, Vec3) {
        let ci = self.character(character);
        let rig = self.project.characters[ci].physics.as_ref().unwrap();
        let bi = rig
            .body_index(body)
            .unwrap_or_else(|| panic!("{character} has no body {body:?}"));
        self.bodies(ci, frame)[bi]
    }

    fn point(&self, point: Point, frame: i32) -> Vec3 {
        match point {
            Point::Joint(character, joint) => {
                let ci = self.character(character);
                let ji = self.project.characters[ci]
                    .skeleton
                    .joint_index(joint)
                    .unwrap_or_else(|| panic!("{character} has no joint {joint:?}"));
                self.state(ci, frame).joints[ji].1
            }
            Point::Body(character, body) => self.body(character, body, frame).1,
            Point::Com(character) => {
                let ci = self.character(character);
                let rig = self.project.characters[ci].physics.as_ref().unwrap();
                let bodies = self.bodies(ci, frame);
                let (sum, mass) = rig
                    .bodies
                    .iter()
                    .zip(bodies)
                    .fold((Vec3::ZERO, 0.0), |(sum, mass), (def, at)| {
                        (sum + at.1 * def.mass, mass + def.mass)
                    });
                sum / mass
            }
            Point::Mid(points) => {
                points.iter().map(|&p| self.point(p, frame)).sum::<Vec3>() / points.len() as f32
            }
        }
    }

    fn value(&self, signal: Signal, frame: i32) -> f32 {
        let deg = |a: Vec3, b: Vec3| a.angle_between(b).to_degrees();
        match signal {
            Signal::Coord(p, axis) => {
                let at = self.point(p, frame);
                match axis {
                    Axis::X => at.x,
                    Axis::Y => at.y,
                    Axis::Z => at.z,
                }
            }
            Signal::Above(a, b) => self.point(a, frame).y - self.point(b, frame).y,
            Signal::Lowest(character) => {
                let ci = self.character(character);
                self.bodies(ci, frame)
                    .iter()
                    .map(|b| b.1.y)
                    .fold(f32::MAX, f32::min)
            }
            Signal::Elevation(a, b) => {
                let v = self.point(b, frame) - self.point(a, frame);
                libm::atan2f(v.y, v.with_y(0.0).length()).to_degrees()
            }
            Signal::Pitch(a, b) => {
                let v = self.point(b, frame) - self.point(a, frame);
                libm::atan2f(v.z, v.y).to_degrees()
            }
            Signal::AimOff { from, tip, target } => {
                let from = self.point(from, frame);
                deg(
                    self.point(tip, frame) - from,
                    self.point(target, frame) - from,
                )
            }
            Signal::Bend(a, b, c) => {
                let at = self.point(b, frame);
                180.0 - deg(self.point(a, frame) - at, self.point(c, frame) - at)
            }
            Signal::FacingOff(character, body, target) => {
                let (rotation, at) = self.body(character, body, frame);
                deg(rotation * Vec3::Z, self.point(target, frame) - at)
            }
            Signal::Mean(signals) => {
                signals.iter().map(|&s| self.value(s, frame)).sum::<f32>() / signals.len() as f32
            }
            Signal::Worst(signals) => signals
                .iter()
                .map(|&s| self.value(s, frame))
                .fold(f32::MIN, f32::max),
        }
    }

    /// A row's number on this run: `None` when it never happens (a `FirstBelow` that
    /// never goes below).
    fn measure(&self, row: &Row) -> Option<f32> {
        let end = self.project.settings.range.end;
        let v = |frame: i32| self.value(row.signal, frame);
        let frames = |from: i32, to: i32| from..=to.min(end);
        let extreme = |from: i32, to: i32, sign: f32| {
            frames(from, to).map(|f| (f, v(f))).reduce(|best, next| {
                if sign * next.1 > sign * best.1 {
                    next
                } else {
                    best
                }
            })
        };
        let first_below = |from: i32, to: i32, below: f32| frames(from, to).find(|&f| v(f) < below);
        match row.measure {
            Measure::At(f) => Some(v(f)),
            Measure::Change(a, b) => {
                let change = v(b) - v(a);
                // An angle's change is taken the short way round.
                Some(if matches!(row.signal, Signal::Pitch(..)) {
                    (change + 180.0).rem_euclid(360.0) - 180.0
                } else {
                    change
                })
            }
            Measure::Max(a, b) => extreme(a, b, 1.0).map(|e| e.1),
            Measure::MaxFrame(a, b) => extreme(a, b, 1.0).map(|e| e.0 as f32),
            Measure::Min(a, b) => extreme(a, b, -1.0).map(|e| e.1),
            Measure::MinFrame(a, b) => extreme(a, b, -1.0).map(|e| e.0 as f32),
            Measure::FirstBelow { from, to, below } => {
                first_below(from, to, below).map(|f| f as f32)
            }
            Measure::LastBelow { from, to, below } => {
                let first = first_below(from, to, below)?;
                let last = frames(first, to)
                    .take_while(|&f| v(f) < below)
                    .last()
                    .unwrap_or(first);
                Some(last as f32)
            }
        }
    }
}

/// Where a scene's file is: under endorphin's `Resources/Scenes`, or in the folder of
/// one's own scenes.
fn locate(file: &str, endorphin: &Path, own: Option<&Path>) -> Option<PathBuf> {
    let candidates = [
        Some(endorphin.join("Resources/Scenes").join(file)),
        own.map(|dir| dir.join(Path::new(file).file_name().unwrap())),
    ];
    candidates.into_iter().flatten().find(|p| p.is_file())
}

/// `ENDORPHIN_DIR=... cargo test --release reference_scores -- --ignored --nocapture`
///
/// Simulates every scene in [`SCENES`] that can be found and prints, per measurement,
/// endorphin's value, NeuroMotion's, the error and whether it is within the
/// measurement's uncertainty, with a score per scene. Fails if a guard row falls
/// outside. `NM_REFERENCE_SCENES` is a folder with one's own scenes (such as
/// Back-Flip-To-Combo.ens); `NM_SCENES=Car,Flip` scores only scenes whose file names
/// contain one of those; `NM_NUDGES=2` (see `nudges`) also runs each scene with
/// gravity nudged by that many steps of 1e-4 each way, prints the range NeuroMotion's
/// values span and requires guards to hold in every run.
#[test]
#[ignore = "needs a local endorphin 2.5 install"]
fn reference_scores() {
    let root = std::env::var("ENDORPHIN_DIR")
        .unwrap_or_else(|_| r"C:\Program Files (x86)\NaturalMotion\endorphin 2.5.2".to_owned());
    let own = std::env::var("NM_REFERENCE_SCENES").ok().map(PathBuf::from);
    let nudges = match std::env::var("NM_NUDGES") {
        Ok(_) => nudges(),
        Err(_) => vec![0.0],
    };
    let only = std::env::var("NM_SCENES").ok();
    let (mut failures, mut within, mut rows, mut skipped) = (vec![], 0, 0, vec![]);
    for scene in SCENES {
        if let Some(only) = &only
            && !only.split(',').any(|part| scene.file.contains(part.trim()))
        {
            continue;
        }
        let Some(path) = locate(scene.file, Path::new(&root), own.as_deref()) else {
            skipped.push(scene.file);
            continue;
        };
        let mut project = crate::io::endorphin::import_scene(&path).unwrap().project;
        let last = scene
            .rows
            .iter()
            .map(|r| r.measure.last_frame())
            .max()
            .unwrap();
        let range = &mut project.settings.range;
        range.end = range.end.min(last.max(range.start));
        // NeuroMotion's value of every row in every nudged run.
        let mut values: Vec<Vec<Option<f32>>> = vec![vec![]; scene.rows.len()];
        for &nudge in &nudges {
            let mut nudged = project.clone();
            for g in &mut nudged.physics.gravity {
                *g *= 1.0 + nudge;
            }
            let (sim, _) = simulate(&nudged);
            let run = Run {
                project: &nudged,
                sim,
            };
            for (row, values) in scene.rows.iter().zip(&mut values) {
                values.push(run.measure(row));
            }
        }
        println!("\n{} ({})", scene.file, scene.measured);
        println!(
            "  {:<56} {:>18} {:>11} {:>10} {:>11}  within?",
            "quantity", "endorphin", "NeuroMotion", "error", "baseline"
        );
        let mut scene_within = 0;
        let mut errors_in_uncertainties = 0.0;
        for (row, values) in scene.rows.iter().zip(&values) {
            let base = values[0];
            let error = base.map(|v| v - row.endorphin);
            let holds =
                |v: &Option<f32>| v.is_some_and(|v| (v - row.endorphin).abs() <= row.uncertainty);
            let ok = holds(&base);
            let all = values.iter().all(holds);
            scene_within += usize::from(ok);
            errors_in_uncertainties += error.map_or(4.0, |e| (e.abs() / row.uncertainty).min(4.0));
            let nudged = if values.len() > 1 {
                let found: Vec<f32> = values.iter().flatten().copied().collect();
                let (lo, hi) = found
                    .iter()
                    .fold((f32::MAX, f32::MIN), |(lo, hi), &v| (lo.min(v), hi.max(v)));
                format!(
                    "  nudged {} to {}{}",
                    row.show(lo),
                    row.show(hi),
                    if found.len() < values.len() {
                        ", never in some"
                    } else {
                        ""
                    }
                )
            } else {
                String::new()
            };
            let verdict = match (row.guard, ok && all) {
                (true, true) => "yes (guard)",
                (true, false) => "NO: GUARD FAILS",
                (false, _) if ok => "yes (not a guard)",
                (false, _) => "no (known gap)",
            };
            println!(
                "  {:<56} {:>18} {:>11} {:>10} {:>11}  {verdict}{nudged}",
                row.what,
                format!(
                    "{} ± {}",
                    row.show(row.endorphin),
                    row.show(row.uncertainty)
                ),
                base.map_or("never".to_owned(), |v| row.show(v)),
                error.map_or("-".to_owned(), |e| row.show(e)),
                row.show(row.baseline),
            );
            if row.guard && !(ok && all) {
                failures.push(format!("{}: {}", scene.file, row.what));
            }
        }
        println!(
            "  score: {scene_within} of {} within endorphin's uncertainty; mean error {:.1} \
             times the uncertainty (each row counted up to 4)",
            scene.rows.len(),
            errors_in_uncertainties / scene.rows.len() as f32
        );
        let mut sources: Vec<&str> = scene.rows.iter().map(|r| r.source).collect();
        sources.sort_unstable();
        sources.dedup();
        println!("  sources: {}", sources.join("; "));
        within += scene_within;
        rows += scene.rows.len();
    }
    println!("\nIn all: {within} of {rows} measurements within endorphin's uncertainty.");
    for file in &skipped {
        println!(
            "Skipped {file}: not under {root}/Resources/Scenes{}",
            if own.is_some() {
                " or in NM_REFERENCE_SCENES"
            } else {
                " (one's own scenes: set NM_REFERENCE_SCENES)"
            }
        );
    }
    assert!(
        failures.is_empty(),
        "guards outside endorphin's uncertainty: {failures:#?}"
    );
}
