//! Higher-level edits built from [`Edit`]s, shared by every panel so the timeline and
//! the inspector change the project in exactly the same way.

use std::collections::BTreeMap;

use super::edit::{Document, Edit};
use super::model::{Channel, Clip, ClipId, ClipKind, ClipProps, Interp, Keyframe};

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
}
