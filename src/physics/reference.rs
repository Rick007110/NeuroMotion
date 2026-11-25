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

// Car Hit's recording was read with endorphin's View Frame field (to a frame or two at
// 60 fps), and positions by fitting a camera to the view and back-projecting the
// pedestrian's silhouette (background subtracted) onto the vertical plane through
// NeuroMotion's centre of mass, so x runs along the road. A silhouette's centroid is
// not quite the centre of mass: where endorphin's pedestrian and NeuroMotion's follow
// the same animation (frames 30 and 190), the two differ by up to 6 cm.
const CAR_HIT_VIDEO: &str =
    "recording of endorphin 2.5.2, silhouette back-projected (lessons 20-21, 28)";

const fn com(axis: Axis) -> Signal {
    Signal::Coord(Point::Com("character01"), axis)
}

/// What is measured, scene by scene. See the module's documentation to add to it.
pub const SCENES: &[Scene] = &[
    Scene {
        file: "Sample Scenes/Car Hit.ens",
        measured: "a frame-by-frame recording of endorphin's run, silhouette \
                   back-projected through a fitted camera (physics-design.md lessons \
                   20, 21 and 28)",
        rows: &[
            Row {
                what: "run-up: centre of mass x, frame 30",
                signal: com(Axis::X),
                measure: Measure::At(30),
                endorphin: 1.38,
                uncertainty: 0.12,
                guard: true,
                baseline: 1.437,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "run-up: centre of mass height, frame 30",
                signal: com(Axis::Y),
                measure: Measure::At(30),
                endorphin: 0.82,
                uncertainty: 0.12,
                guard: true,
                baseline: 0.868,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "flight: centre of mass highest",
                signal: com(Axis::Y),
                measure: Measure::Max(48, 100),
                endorphin: 1.81,
                uncertainty: 0.06,
                guard: false,
                baseline: 1.616,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "flight: frame it is highest",
                signal: com(Axis::Y),
                measure: Measure::MaxFrame(48, 100),
                endorphin: 68.0,
                uncertainty: 2.0,
                guard: false,
                baseline: 65.0,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "flight: centre of mass x, frame 68",
                signal: com(Axis::X),
                measure: Measure::At(68),
                endorphin: 0.01,
                uncertainty: 0.1,
                guard: true,
                baseline: 0.058,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "flight: centre of mass x, frame 78",
                signal: com(Axis::X),
                measure: Measure::At(78),
                endorphin: -0.11,
                uncertainty: 0.1,
                guard: false,
                baseline: 0.084,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "flight: centre of mass x, frame 88",
                signal: com(Axis::X),
                measure: Measure::At(88),
                endorphin: -0.12,
                uncertainty: 0.1,
                guard: false,
                baseline: 0.054,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "flight: centre of mass height, frame 78",
                signal: com(Axis::Y),
                measure: Measure::At(78),
                endorphin: 1.62,
                uncertainty: 0.08,
                guard: false,
                baseline: 1.369,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "flight: centre of mass height, frame 88",
                signal: com(Axis::Y),
                measure: Measure::At(88),
                endorphin: 1.26,
                uncertainty: 0.08,
                guard: false,
                baseline: 0.823,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "upside down (head under the centre of mass) from",
                signal: Signal::Above(
                    Point::Body("character01", "HeadMass"),
                    Point::Com("character01"),
                ),
                measure: Measure::FirstBelow {
                    from: 48,
                    to: 120,
                    below: 0.0,
                },
                endorphin: 76.0,
                uncertainty: 3.0,
                guard: false,
                baseline: 62.0,
                source: "recording, read by eye frame by frame (lesson 28)",
            },
            Row {
                what: "upside down until",
                signal: Signal::Above(
                    Point::Body("character01", "HeadMass"),
                    Point::Com("character01"),
                ),
                measure: Measure::LastBelow {
                    from: 48,
                    to: 120,
                    below: 0.0,
                },
                endorphin: 88.0,
                uncertainty: 3.0,
                guard: false,
                baseline: 83.0,
                source: "recording, read by eye frame by frame (lesson 28)",
            },
            Row {
                what: "first on the ground (a body centre under 6 cm)",
                signal: Signal::Lowest("character01"),
                measure: Measure::FirstBelow {
                    from: 60,
                    to: 140,
                    below: 0.06,
                },
                endorphin: 96.0,
                uncertainty: 3.0,
                guard: false,
                baseline: 89.0,
                source: "recording: feet first about frame 96, by eye (lesson 21)",
            },
            Row {
                what: "landed: centre of mass x, frame 96",
                signal: com(Axis::X),
                measure: Measure::At(96),
                endorphin: -0.34,
                uncertainty: 0.1,
                guard: false,
                baseline: 0.079,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "back on its feet: centre of mass x, frame 190",
                signal: com(Axis::X),
                measure: Measure::At(190),
                endorphin: -1.49,
                uncertainty: 0.12,
                guard: true,
                baseline: -1.517,
                source: CAR_HIT_VIDEO,
            },
            Row {
                what: "back on its feet: centre of mass height, frame 190",
                signal: com(Axis::Y),
                measure: Measure::At(190),
                endorphin: 0.91,
                uncertainty: 0.12,
                guard: true,
                baseline: 0.948,
                source: CAR_HIT_VIDEO,
            },
        ],
    },
    Scene {
        file: "Back-Flip-To-Combo.ens",
        measured: "a user's own scene (#6): a recording of endorphin's run with its frame \
                   counter, a camera fitted to the box and the start pose, silhouettes \
                   (physics-design.md lessons 19 and 27)",
        rows: &[
            Row {
                what: "lean back about the heels since frame 0, frame 44",
                signal: BACKFLIP_LEAN,
                measure: Measure::Change(0, 44),
                endorphin: -1.5,
                uncertainty: 2.0,
                guard: false,
                baseline: -3.4,
                source: BACKFLIP_LEAN_SOURCE,
            },
            Row {
                what: "lean back about the heels since frame 0, frame 72",
                signal: BACKFLIP_LEAN,
                measure: Measure::Change(0, 72),
                endorphin: -4.0,
                uncertainty: 2.0,
                guard: false,
                baseline: -6.5,
                source: BACKFLIP_LEAN_SOURCE,
            },
            Row {
                what: "lean back about the heels since frame 0, frame 100",
                signal: BACKFLIP_LEAN,
                measure: Measure::Change(0, 100),
                endorphin: -6.0,
                uncertainty: 2.0,
                guard: false,
                baseline: -8.8,
                source: BACKFLIP_LEAN_SOURCE,
            },
            Row {
                what: "left arm (shoulder to hand) above horizontal, frame 96",
                signal: Signal::Elevation(
                    Point::Joint("character01", "LUpperArm"),
                    Point::Joint("character01", "LHand"),
                ),
                measure: Measure::At(96),
                endorphin: -7.0,
                uncertainty: 5.0,
                guard: false,
                baseline: 19.0,
                source: "recording: the finger tip's pixel on a plane through the shoulder \
                         (lesson 27)",
            },
            Row {
                what: "crouching: head drop, frames 140 to 168",
                signal: BACKFLIP_HEAD,
                measure: Measure::Change(140, 168),
                endorphin: -0.20,
                uncertainty: 0.06,
                guard: true,
                baseline: -0.193,
                source: BACKFLIP_HEAD_SOURCE,
            },
            Row {
                what: "crouching: head drop, frames 140 to 192",
                signal: BACKFLIP_HEAD,
                measure: Measure::Change(140, 192),
                endorphin: -0.10,
                uncertainty: 0.06,
                guard: false,
                baseline: -0.268,
                source: BACKFLIP_HEAD_SOURCE,
            },
            Row {
                what: "springs up: head rise, frames 192 to 248",
                signal: BACKFLIP_HEAD,
                measure: Measure::Change(192, 248),
                endorphin: 0.65,
                uncertainty: 0.12,
                guard: false,
                baseline: -1.337,
                source: BACKFLIP_HEAD_SOURCE,
            },
            Row {
                what: "leaves the box top (a body centre 10 cm under it)",
                signal: Signal::Lowest("character01"),
                measure: Measure::FirstBelow {
                    from: 100,
                    to: 500,
                    below: 2.42,
                },
                endorphin: 212.0,
                uncertainty: 6.0,
                guard: false,
                baseline: 242.0,
                source: "recording, by eye: in the air above the box at 228 (lesson 27)",
            },
            Row {
                what: "first on the floor (a body centre under 15 cm)",
                signal: Signal::Lowest("character01"),
                measure: Measure::FirstBelow {
                    from: 100,
                    to: 726,
                    below: 0.15,
                },
                endorphin: 276.0,
                uncertainty: 6.0,
                guard: false,
                baseline: 294.0,
                source: "recording, by eye: feet first behind the box (lesson 27)",
            },
        ],
    },
    Scene {
        file: "Behaviour Scenes/Hands reach and look at2.5.ens",
        measured: "a recording of endorphin's demo scene with its frame counter, a camera \
                   fitted to the view (behaviours.md, Look and Reach)",
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
                endorphin: 20.0,
                uncertainty: 4.0,
                guard: true,
                baseline: 18.0,
                source: "recording, by eye: swinging in at 12, on the box by 18-24",
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
                baseline: 31.0,
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
        measured: "a front view recording of endorphin's behaviour scene with its frame \
                   counter, arm angles read in the image (behaviours.md, Explosion)",
        rows: &[
            Row {
                what: "character01: upper arms highest (mean, above horizontal)",
                signal: WRITHE_ARMS,
                measure: Measure::Max(5, 35),
                endorphin: 36.5,
                uncertainty: 5.0,
                guard: true,
                baseline: 38.3,
                source: WRITHE_SOURCE,
            },
            Row {
                what: "character01: frame the arms are highest",
                signal: WRITHE_ARMS,
                measure: Measure::MaxFrame(5, 35),
                endorphin: 25.0,
                uncertainty: 5.0,
                guard: false,
                baseline: 16.0,
                source: WRITHE_SOURCE,
            },
            Row {
                what: "character01: upper arms lowest",
                signal: WRITHE_ARMS,
                measure: Measure::Min(30, 56),
                endorphin: -49.0,
                uncertainty: 9.0,
                guard: false,
                baseline: -33.7,
                source: WRITHE_SOURCE,
            },
            Row {
                what: "character01: frame the arms are lowest",
                signal: WRITHE_ARMS,
                measure: Measure::MinFrame(30, 56),
                endorphin: 44.0,
                uncertainty: 3.0,
                guard: false,
                baseline: 42.0,
                source: WRITHE_SOURCE,
            },
            Row {
                what: "character01: lands (a body centre under 10 cm)",
                signal: Signal::Lowest("character01"),
                measure: Measure::FirstBelow {
                    from: 10,
                    to: 120,
                    below: 0.1,
                },
                endorphin: 57.0,
                uncertainty: 3.0,
                guard: false,
                baseline: 54.0,
                source: "recording, by eye: feet first at frame 56-58",
            },
        ],
    },
    Scene {
        file: "Sample Scenes/Stagger Walk.ens",
        measured: "a recording of endorphin's run with its frame counter, the feet's \
                   footprint and the silhouette's top back-projected through a fitted \
                   camera (behaviours.md, Stagger Walk)",
        rows: &[
            Row {
                what: "feet moved back (z), frames 0 to 36",
                signal: Signal::Coord(STAGGER_FEET, Axis::Z),
                measure: Measure::Change(0, 36),
                endorphin: -0.46,
                uncertainty: 0.2,
                guard: true,
                baseline: -0.43,
                source: STAGGER_SOURCE,
            },
            Row {
                what: "feet moved back (z), frames 0 to 96",
                signal: Signal::Coord(STAGGER_FEET, Axis::Z),
                measure: Measure::Change(0, 96),
                endorphin: -0.88,
                uncertainty: 0.25,
                guard: false,
                baseline: -0.66,
                source: STAGGER_SOURCE,
            },
            Row {
                what: "feet moved sideways (x), frames 0 to 96",
                signal: Signal::Coord(STAGGER_FEET, Axis::X),
                measure: Measure::Change(0, 96),
                endorphin: -0.51,
                uncertainty: 0.25,
                guard: false,
                baseline: 0.11,
                source: STAGGER_SOURCE,
            },
            Row {
                what: "still standing: top of the neck, change frames 0 to 108",
                signal: Signal::Coord(Point::Joint("character01", "UpperNeckJoint"), Axis::Y),
                measure: Measure::Change(0, 108),
                endorphin: -0.12,
                uncertainty: 0.1,
                guard: false,
                baseline: -0.11,
                source: "recording: the silhouette's top, 1.59 m at frame 0 and 1.47 m at \
                         108",
            },
        ],
    },
];

// Back-Flip-To-Combo's character: the pelvis over the left heel, standing in for the
// whole-body lean the recording was fitted with (the start pose turned about the heels
// to fit the legs' and pelvis' silhouette). The two agree while the legs are straight,
// to about frame 100; endorphin's fit at frame 0 was -1°, so its leans are given
// relative to that.
const BACKFLIP_LEAN: Signal = Signal::Pitch(
    Point::Joint("character01", "LFoot"),
    Point::Joint("character01", "Pelvis"),
);
const BACKFLIP_LEAN_SOURCE: &str = "recording: the start pose turned about the heels to fit \
     the legs' and pelvis' silhouette, in 0.5° steps (lesson 27)";
const BACKFLIP_HEAD: Signal = Signal::Coord(Point::Joint("character01", "Head"), Axis::Y);
const BACKFLIP_HEAD_SOURCE: &str = "recording: the head's height in the image, about 86 px \
     a metre at the box (lesson 27)";

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
const WRITHE_SOURCE: &str = "recording, by eye: up to +35° (+38° one arm) at 20-30, down \
     to -40 to -58° at 42-46";

const STAGGER_FEET: Point = Point::Mid(&[
    Point::Joint("character01", "LeftAnkleJoint"),
    Point::Joint("character01", "RightAnkleJoint"),
]);
const STAGGER_SOURCE: &str = "recording: the feet's footprint back-projected onto the \
     ground; two steps back by frame 36, about five in all";

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

    fn body(&self, character: &str, body: &str, frame: i32) -> (Quat, Vec3) {
        let ci = self.character(character);
        let rig = self.project.characters[ci].physics.as_ref().unwrap();
        let bi = rig
            .body_index(body)
            .unwrap_or_else(|| panic!("{character} has no body {body:?}"));
        self.state(ci, frame).bodies[bi]
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
                let bodies = &self.state(ci, frame).bodies;
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
                self.state(ci, frame)
                    .bodies
                    .iter()
                    .map(|b| b.1.y)
                    .fold(f32::MAX, f32::min)
            }
            Signal::Elevation(a, b) => {
                let v = self.point(b, frame) - self.point(a, frame);
                v.y.atan2(v.with_y(0.0).length()).to_degrees()
            }
            Signal::Pitch(a, b) => {
                let v = self.point(b, frame) - self.point(a, frame);
                v.z.atan2(v.y).to_degrees()
            }
            Signal::AimOff { from, tip, target } => {
                let from = self.point(from, frame);
                deg(
                    self.point(tip, frame) - from,
                    self.point(target, frame) - from,
                )
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
