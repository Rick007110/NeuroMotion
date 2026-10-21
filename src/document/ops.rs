//! Higher-level edits built from [`Edit`]s, shared by every panel so the timeline and
//! the inspector change the project in exactly the same way.

use std::collections::BTreeMap;

use glam::{Quat, Vec3};

use super::bake::bake_channels;
use super::edit::{Document, Edit};
use super::model::{CharacterId, Channel, Clip, ClipId, ClipKind, ClipProps, Interp, Keyframe, Track, TrackId};
use super::physics::SimLevel;

impl Document {
    /// Keys a channel at a clip-local frame with the value its curve already has there.
    /// Returns false if the clip or channel doesn't exist.
    pub fn insert_key(&mut self, clip: ClipId, channel: usize, frame: i32) -> bool {
        let Some(ch) = self.project.clip(clip).and_then(|c| c.channels.get(channel)) else {
            return false;
        };
        let key = Keyframe {
            frame,
            value: ch.sample(frame as f32).unwrap_or(0.0),
            interp: ch.keys.last().map_or(Interp::Smooth, |k| k.interp),
        };
        let mut updated = ch.clone();
        updated.set_key(key);
        let edit = Edit::ChannelKeys {
            clip,
            channel,
            before: ch.keys.clone(),
            after: updated.keys,
        };
        self.apply("Add key", edit, false);
        true
    }

    pub fn remove_key(&mut self, clip: ClipId, channel: usize, frame: i32) -> bool {
        let Some(ch) = self.project.clip(clip).and_then(|c| c.channels.get(channel)) else {
            return false;
        };
        let mut updated = ch.clone();
        if updated.remove_key(frame).is_none() {
            return false;
        }
        let edit = Edit::ChannelKeys {
            clip,
            channel,
            before: ch.keys.clone(),
            after: updated.keys,
        };
        self.apply("Delete key", edit, false);
        true
    }

    /// Replaces the key at `frame` with `key` (which may sit on a different frame).
    /// Refuses to move a key onto a frame another key already occupies.
    pub fn replace_key(
        &mut self,
        clip: ClipId,
        channel: usize,
        frame: i32,
        key: Keyframe,
        label: &str,
        continuous: bool,
    ) -> bool {
        let Some(ch) = self.project.clip(clip).and_then(|c| c.channels.get(channel)) else {
            return false;
        };
        if key.frame != frame && ch.key(key.frame).is_some() {
            return false;
        }
        let mut updated = ch.clone();
        if updated.remove_key(frame).is_none() {
            return false;
        }
        updated.set_key(key);
        let edit = Edit::ChannelKeys {
            clip,
            channel,
            before: ch.keys.clone(),
            after: updated.keys,
        };
        self.apply(label, edit, continuous);
        true
    }

    /// Adds a new clip (as one undo step) and returns its id.
    pub fn insert_clip(&mut self, props: ClipProps, kind: ClipKind, channels: Vec<Channel>) -> ClipId {
        let clip = Clip {
            id: self.project.alloc_id(),
            props,
            kind,
            channels,
            source: BTreeMap::new(),
        };
        let id = clip.id;
        let index = self.project.clips.len();
        self.apply("Add clip", Edit::InsertClip { index, clip }, false);
        id
    }

    /// Copies a clip onto the same track, starting where the original ends.
    /// Returns the new clip's id.
    pub fn duplicate_clip(&mut self, clip: ClipId) -> Option<ClipId> {
        let index = self.project.clip_index(clip)?;
        let mut copy = self.project.clips[index].clone();
        copy.id = self.project.alloc_id();
        copy.props.start = copy.props.end();
        copy.props.name = format!("{} copy", copy.props.name);
        let id = copy.id;
        self.apply("Duplicate clip", Edit::InsertClip { index: index + 1, clip: copy }, false);
        Some(id)
    }

    pub fn remove_clip(&mut self, clip: ClipId) -> bool {
        let Some(index) = self.project.clip_index(clip) else {
            return false;
        };
        let clip = self.project.clips[index].clone();
        self.apply("Delete clip", Edit::RemoveClip { index, clip }, false);
        true
    }

    /// Adds an empty track for `character` on top of its others (the highest blend
    /// layer), as one undo step. Returns its id.
    pub fn add_track(&mut self, character: CharacterId, name: &str) -> TrackId {
        let edit = self.insert_track_edit(character, name);
        let Edit::InsertTrack { track, .. } = &edit else { unreachable!() };
        let id = track.id;
        self.apply("Add track", edit, false);
        id
    }

    /// Deletes a track and every clip on it, as one undo step.
    pub fn remove_track(&mut self, track: TrackId) -> bool {
        let Some(index) = self.project.track_index(track) else {
            return false;
        };
        // Clips go first, last one first, so undoing puts each back at its own index.
        let mut edits: Vec<Edit> = self
            .project
            .clips
            .iter()
            .enumerate()
            .rev()
            .filter(|(_, c)| c.props.track == track)
            .map(|(index, clip)| Edit::RemoveClip { index, clip: clip.clone() })
            .collect();
        edits.push(Edit::RemoveTrack { index, track: self.project.tracks[index].clone() });
        self.apply("Delete track", Edit::Batch(edits), false);
        true
    }

    /// Turns a character's motion (world joint transforms for every frame of the project
    /// range, usually from the simulation) into keys on a new top track named "Baked",
    /// then makes the character play its keys: it starts animated and its event clips
    /// (simulation switches, forces, ...) are disabled. One undo step.
    pub fn bake_motion(&mut self, character: CharacterId, frames: &[Vec<(Quat, Vec3)>]) -> Option<(TrackId, ClipId)> {
        let project = &self.project;
        let skeleton = &project.character(character)?.skeleton;
        if frames.is_empty() || frames.iter().any(|f| f.len() != skeleton.joints.len()) {
            return None;
        }
        let channels = bake_channels(skeleton, frames);
        let range = project.settings.range;
        let level = project.character(character)?.default_level;
        let tracks: Vec<TrackId> = project.tracks_of(character).map(|t| t.id).collect();
        let mut edits: Vec<Edit> = project
            .clips
            .iter()
            .filter(|c| !c.kind.is_motion() && c.props.enabled && tracks.contains(&c.props.track))
            .map(|c| Edit::ClipProps {
                clip: c.id,
                before: c.props.clone(),
                after: ClipProps { enabled: false, ..c.props.clone() },
            })
            .collect();
        edits.push(Edit::CharacterLevel { character, before: level, after: SimLevel::Animated });

        let name = (1..)
            .map(|n| if n == 1 { "Baked".to_owned() } else { format!("Baked {n}") })
            .find(|n| !project.tracks_of(character).any(|t| t.name == *n))
            .unwrap_or_default();
        let insert = self.insert_track_edit(character, &name);
        let Edit::InsertTrack { track: Track { id: track, .. }, .. } = &insert else { unreachable!() };
        let track = *track;
        let clip = Clip {
            id: self.project.alloc_id(),
            props: ClipProps {
                name: "Baked simulation".to_owned(),
                track,
                start: range.start,
                duration: (frames.len() as i32 - 1).max(1),
                blend_in: 0,
                blend_out: 0,
                weight: 1.0,
                enabled: true,
            },
            kind: ClipKind::Motion,
            channels,
            source: BTreeMap::new(),
        };
        let id = clip.id;
        edits.push(insert);
        edits.push(Edit::InsertClip { index: self.project.clips.len(), clip });
        self.apply("Bake simulation", Edit::Batch(edits), false);
        Some((track, id))
    }

    /// An edit inserting a new track right after the character's last one.
    fn insert_track_edit(&mut self, character: CharacterId, name: &str) -> Edit {
        let project = &self.project;
        let index = project
            .tracks
            .iter()
            .rposition(|t| t.character == character)
            .map_or(project.tracks.len(), |i| i + 1);
        let track = Track { id: self.project.alloc_id(), name: name.to_owned(), character, muted: false };
        Edit::InsertTrack { index, track }
    }
}
