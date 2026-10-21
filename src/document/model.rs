//! Core data types for a NeuroMotion project.
//!
//! Everything here is plain data with no UI or rendering types. Timeline frames are
//! integers in project time; keyframe frames are local to their clip (0 = clip start).

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use super::physics::{PhysicsRig, PhysicsSettings, SimLevel};

pub type CharacterId = u64;
pub type TrackId = u64;
pub type ClipId = u64;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Project {
    pub settings: ProjectSettings,
    pub characters: Vec<Character>,
    /// Each track animates one character. Per character, blend order is bottom to top:
    /// later tracks are layered over earlier ones.
    pub tracks: Vec<Track>,
    pub clips: Vec<Clip>,
    #[serde(default)]
    pub physics: PhysicsSettings,
    #[serde(default)]
    next_id: u64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProjectSettings {
    pub name: String,
    pub fps: f32,
    pub range: FrameRange,
}

/// An inclusive span of timeline frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FrameRange {
    pub start: i32,
    pub end: i32,
}

impl FrameRange {
    pub fn len(&self) -> i32 {
        self.end - self.start
    }
}

/// Something with a skeleton that tracks can animate: a character or a prop.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Character {
    pub id: CharacterId,
    /// Unique within the project; imported events refer to characters by this name.
    pub name: String,
    pub kind: CharacterKind,
    pub skeleton: SkeletonRef,
    /// Physical body; characters without one are always animated.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub physics: Option<PhysicsRig>,
    /// Simulation level before any simulation event switches it.
    #[serde(default)]
    pub default_level: SimLevel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CharacterKind {
    Character,
    Prop,
    /// Static scenery (ground, walls) carried over from an imported scene.
    Environment,
}

impl CharacterKind {
    pub fn label(self) -> &'static str {
        match self {
            CharacterKind::Character => "Character",
            CharacterKind::Prop => "Prop",
            CharacterKind::Environment => "Environment",
        }
    }
}

/// The rig a character uses.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SkeletonRef {
    pub name: String,
    /// Path of the rig file this skeleton came from; `None` for built-in rigs.
    pub source: Option<String>,
    /// Cached joint hierarchy, so a project still previews when the source is missing.
    /// Parents always come before their children.
    pub joints: Vec<Joint>,
}

impl SkeletonRef {
    pub fn joint_index(&self, name: &str) -> Option<usize> {
        self.joints.iter().position(|j| j.name == name)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    pub parent: Option<usize>,
    /// Rest offset from the parent joint, in the parent's frame, in metres.
    pub offset: [f32; 3],
    /// Rest orientation relative to the parent, as a quaternion `[x, y, z, w]`.
    /// Animated rotations are applied on top of it.
    #[serde(default = "identity_quat")]
    pub rest_rotation: [f32; 4],
}

fn identity_quat() -> [f32; 4] {
    [0.0, 0.0, 0.0, 1.0]
}

impl Joint {
    pub fn new(name: &str, parent: Option<usize>, offset: [f32; 3]) -> Self {
        Self {
            name: name.to_owned(),
            parent,
            offset,
            rest_rotation: identity_quat(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Track {
    pub id: TrackId,
    pub name: String,
    pub character: CharacterId,
    pub muted: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Clip {
    pub id: ClipId,
    pub props: ClipProps,
    #[serde(default)]
    pub kind: ClipKind,
    pub channels: Vec<Channel>,
    /// Settings carried over verbatim from an imported file (e.g. an endorphin event's
    /// strength or physics level) that NeuroMotion doesn't model yet.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub source: BTreeMap<String, String>,
}

/// What a clip does to its character.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClipKind {
    /// Keyframed joint channels, blended into the pose.
    #[default]
    Motion,
    /// A timed event carried over from an imported scene (force, behaviour, simulation
    /// switch, transition, ...). Its settings live in [`Clip::source`] for the physics
    /// system; it does not change the keyframed pose.
    Event { event: String },
}

impl ClipKind {
    pub fn is_motion(&self) -> bool {
        matches!(self, ClipKind::Motion)
    }
}

/// The editable scalar properties of a clip, kept together so one undo entry covers them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ClipProps {
    pub name: String,
    pub track: TrackId,
    /// Timeline frame where the clip's local frame 0 lands.
    pub start: i32,
    pub duration: i32,
    /// Frames spent fading in from the layers below.
    pub blend_in: i32,
    /// Frames spent fading back out at the end.
    pub blend_out: i32,
    /// Peak influence, 0..=1.
    pub weight: f32,
    pub enabled: bool,
}

impl ClipProps {
    pub fn end(&self) -> i32 {
        self.start + self.duration
    }

    pub fn contains(&self, frame: f32) -> bool {
        frame >= self.start as f32 && frame <= self.end() as f32
    }
}

impl Clip {
    /// How strongly this clip affects the pose at a timeline frame: its weight scaled by
    /// the eased blend-in / blend-out ramps, and zero outside the clip.
    pub fn influence(&self, frame: f32) -> f32 {
        let p = &self.props;
        if !p.enabled || !p.contains(frame) {
            return 0.0;
        }
        let ramp = |frames_inside: f32, blend: i32| {
            if blend <= 0 {
                1.0
            } else {
                smoothstep((frames_inside / blend as f32).clamp(0.0, 1.0))
            }
        };
        let fade_in = ramp(frame - p.start as f32, p.blend_in);
        let fade_out = ramp(p.end() as f32 - frame, p.blend_out);
        p.weight.clamp(0.0, 1.0) * fade_in.min(fade_out)
    }
}

/// One animated property of one joint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Channel {
    pub joint: String,
    pub property: Property,
    /// Sorted by frame, at most one key per frame.
    pub keys: Vec<Keyframe>,
}

impl Channel {
    pub fn key(&self, frame: i32) -> Option<&Keyframe> {
        self.keys.iter().find(|k| k.frame == frame)
    }

    /// Inserts a key, replacing any existing key on the same frame.
    pub fn set_key(&mut self, key: Keyframe) {
        match self.keys.binary_search_by_key(&key.frame, |k| k.frame) {
            Ok(i) => self.keys[i] = key,
            Err(i) => self.keys.insert(i, key),
        }
    }

    pub fn remove_key(&mut self, frame: i32) -> Option<Keyframe> {
        let i = self.keys.iter().position(|k| k.frame == frame)?;
        Some(self.keys.remove(i))
    }

    /// Value at a clip-local frame. Holds the first/last value outside the keyed range.
    pub fn sample(&self, frame: f32) -> Option<f32> {
        let keys = &self.keys;
        let first = keys.first()?;
        let last = keys.last()?;
        if frame <= first.frame as f32 {
            return Some(first.value);
        }
        if frame >= last.frame as f32 {
            return Some(last.value);
        }
        let i = keys.partition_point(|k| k.frame as f32 <= frame) - 1;
        let (a, b) = (&keys[i], &keys[i + 1]);
        let span = (b.frame - a.frame) as f32;
        let t = (frame - a.frame as f32) / span;
        Some(match a.interp {
            Interp::Step => a.value,
            Interp::Linear => a.value + (b.value - a.value) * t,
            Interp::Smooth => {
                // Catmull-Rom style tangents from the neighbouring keys; flat at the ends.
                let slope = |j: usize| {
                    if j == 0 || j + 1 >= keys.len() {
                        0.0
                    } else {
                        let (prev, next) = (&keys[j - 1], &keys[j + 1]);
                        (next.value - prev.value) / (next.frame - prev.frame) as f32
                    }
                };
                hermite(a.value, slope(i) * span, b.value, slope(i + 1) * span, t)
            }
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Property {
    RotX,
    RotY,
    RotZ,
    PosX,
    PosY,
    PosZ,
}

impl Property {
    /// Slot of this property in a [`crate::document::Pose`] joint entry.
    pub fn index(self) -> usize {
        self as usize
    }

    pub fn label(self) -> &'static str {
        match self {
            Property::RotX => "rotate X",
            Property::RotY => "rotate Y",
            Property::RotZ => "rotate Z",
            Property::PosX => "offset X",
            Property::PosY => "offset Y",
            Property::PosZ => "offset Z",
        }
    }

    pub fn unit(self) -> &'static str {
        if self.is_rotation() { "°" } else { " m" }
    }

    pub fn is_rotation(self) -> bool {
        matches!(self, Property::RotX | Property::RotY | Property::RotZ)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Keyframe {
    pub frame: i32,
    pub value: f32,
    /// How the curve travels from this key to the next one.
    pub interp: Interp,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Interp {
    Step,
    Linear,
    Smooth,
}

impl Interp {
    pub const ALL: [Interp; 3] = [Interp::Step, Interp::Linear, Interp::Smooth];

    pub fn label(self) -> &'static str {
        match self {
            Interp::Step => "Step",
            Interp::Linear => "Linear",
            Interp::Smooth => "Smooth",
        }
    }
}

impl Project {
    pub fn new(settings: ProjectSettings) -> Self {
        Self {
            settings,
            characters: Vec::new(),
            tracks: Vec::new(),
            clips: Vec::new(),
            physics: PhysicsSettings::default(),
            next_id: 1,
        }
    }

    pub(crate) fn alloc_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Makes sure freshly allocated ids can't collide with loaded ones.
    pub(crate) fn normalize_ids(&mut self) {
        let max_character = self.characters.iter().map(|c| c.id).max().unwrap_or(0);
        let max_track = self.tracks.iter().map(|t| t.id).max().unwrap_or(0);
        let max_clip = self.clips.iter().map(|c| c.id).max().unwrap_or(0);
        self.next_id = self.next_id.max(max_character.max(max_track).max(max_clip) + 1);
    }

    /// Construction helper for built-in content, importers and tests; editor changes go
    /// through [`crate::document::Document::apply`] so they can be undone.
    pub fn add_character(&mut self, name: &str, kind: CharacterKind, skeleton: SkeletonRef) -> CharacterId {
        let id = self.alloc_id();
        self.characters.push(Character {
            id,
            name: name.to_owned(),
            kind,
            skeleton,
            physics: None,
            default_level: SimLevel::Animated,
        });
        id
    }

    /// Construction helper, see [`Project::add_character`].
    pub fn add_track(&mut self, character: CharacterId, name: &str) -> TrackId {
        let id = self.alloc_id();
        self.tracks.push(Track {
            id,
            name: name.to_owned(),
            character,
            muted: false,
        });
        id
    }

    /// Construction helper, see [`Project::add_character`].
    pub fn add_clip(&mut self, props: ClipProps, kind: ClipKind, channels: Vec<Channel>) -> ClipId {
        let id = self.alloc_id();
        self.clips.push(Clip {
            id,
            props,
            kind,
            channels,
            source: BTreeMap::new(),
        });
        id
    }

    pub fn character(&self, id: CharacterId) -> Option<&Character> {
        self.characters.iter().find(|c| c.id == id)
    }

    pub fn character_index(&self, id: CharacterId) -> Option<usize> {
        self.characters.iter().position(|c| c.id == id)
    }

    pub fn character_by_name(&self, name: &str) -> Option<&Character> {
        self.characters.iter().find(|c| c.name == name)
    }

    /// The character a clip animates (through its track).
    pub fn clip_character(&self, clip: &Clip) -> Option<&Character> {
        self.track(clip.props.track).and_then(|t| self.character(t.character))
    }

    pub fn tracks_of(&self, character: CharacterId) -> impl Iterator<Item = &Track> {
        self.tracks.iter().filter(move |t| t.character == character)
    }

    pub fn track(&self, id: TrackId) -> Option<&Track> {
        self.tracks.iter().find(|t| t.id == id)
    }

    pub fn track_index(&self, id: TrackId) -> Option<usize> {
        self.tracks.iter().position(|t| t.id == id)
    }

    pub fn clip(&self, id: ClipId) -> Option<&Clip> {
        self.clips.iter().find(|c| c.id == id)
    }

    pub fn clip_mut(&mut self, id: ClipId) -> Option<&mut Clip> {
        self.clips.iter_mut().find(|c| c.id == id)
    }

    pub fn clip_index(&self, id: ClipId) -> Option<usize> {
        self.clips.iter().position(|c| c.id == id)
    }

    pub fn clips_on_track(&self, track: TrackId) -> impl Iterator<Item = &Clip> {
        self.clips.iter().filter(move |c| c.props.track == track)
    }
}

fn smoothstep(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

fn hermite(p0: f32, m0: f32, p1: f32, m1: f32, t: f32) -> f32 {
    let t2 = t * t;
    let t3 = t2 * t;
    (2.0 * t3 - 3.0 * t2 + 1.0) * p0
        + (t3 - 2.0 * t2 + t) * m0
        + (-2.0 * t3 + 3.0 * t2) * p1
        + (t3 - t2) * m1
}
