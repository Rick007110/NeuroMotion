//! Undoable edits and the document that owns the project plus its history.
//!
//! Every change the editor makes to a [`Project`] is expressed as an [`Edit`] holding
//! both the before and after state of what it touches, so undo and redo are the same
//! operation run in opposite directions. The UI never mutates the project directly.

use std::path::PathBuf;

use super::model::{CharacterId, Clip, ClipId, ClipProps, Keyframe, Project, ProjectSettings, Track, TrackId};
use super::physics::SimLevel;

pub const UNDO_LIMIT: usize = 500;

#[derive(Clone, Debug, PartialEq)]
pub enum Edit {
    Settings {
        before: ProjectSettings,
        after: ProjectSettings,
    },
    Track {
        id: TrackId,
        before: Track,
        after: Track,
    },
    ClipProps {
        clip: ClipId,
        before: ClipProps,
        after: ClipProps,
    },
    /// Replaces a channel's whole key list; covers adding, moving, editing and deleting keys.
    ChannelKeys {
        clip: ClipId,
        channel: usize,
        before: Vec<Keyframe>,
        after: Vec<Keyframe>,
    },
    InsertClip {
        index: usize,
        clip: Clip,
    },
    RemoveClip {
        index: usize,
        clip: Clip,
    },
    /// Adds a track at `index` in the project's track list. Its clips are separate edits.
    InsertTrack {
        index: usize,
        track: Track,
    },
    /// Removes an (empty) track; remove its clips first, in the same [`Edit::Batch`].
    RemoveTrack {
        index: usize,
        track: Track,
    },
    /// The simulation level a character starts at.
    CharacterLevel {
        character: CharacterId,
        before: SimLevel,
        after: SimLevel,
    },
    /// Several edits as one undo step: applied in order, undone in reverse order.
    Batch(Vec<Edit>),
}

impl Edit {
    fn apply(&self, project: &mut Project, forward: bool) {
        fn pick<'a, T>(before: &'a T, after: &'a T, forward: bool) -> &'a T {
            if forward { after } else { before }
        }
        match self {
            Edit::Settings { before, after } => {
                project.settings = pick(before, after, forward).clone();
            }
            Edit::Track { id, before, after } => {
                if let Some(i) = project.track_index(*id) {
                    project.tracks[i] = pick(before, after, forward).clone();
                }
            }
            Edit::ClipProps { clip, before, after } => {
                if let Some(c) = project.clip_mut(*clip) {
                    c.props = pick(before, after, forward).clone();
                }
            }
            Edit::ChannelKeys {
                clip,
                channel,
                before,
                after,
            } => {
                if let Some(ch) = project
                    .clip_mut(*clip)
                    .and_then(|c| c.channels.get_mut(*channel))
                {
                    ch.keys = pick(before, after, forward).clone();
                }
            }
            Edit::InsertClip { index, clip } | Edit::RemoveClip { index, clip } => {
                let inserting = matches!(self, Edit::InsertClip { .. }) == forward;
                if inserting {
                    let at = (*index).min(project.clips.len());
                    project.clips.insert(at, clip.clone());
                } else if let Some(i) = project.clip_index(clip.id) {
                    project.clips.remove(i);
                }
            }
            Edit::InsertTrack { index, track } | Edit::RemoveTrack { index, track } => {
                let inserting = matches!(self, Edit::InsertTrack { .. }) == forward;
                if inserting {
                    let at = (*index).min(project.tracks.len());
                    project.tracks.insert(at, track.clone());
                } else if let Some(i) = project.track_index(track.id) {
                    project.tracks.remove(i);
                }
            }
            Edit::CharacterLevel { character, before, after } => {
                if let Some(c) = project.characters.iter_mut().find(|c| c.id == *character) {
                    c.default_level = *pick(before, after, forward);
                }
            }
            Edit::Batch(edits) => {
                if forward {
                    edits.iter().for_each(|e| e.apply(project, true));
                } else {
                    edits.iter().rev().for_each(|e| e.apply(project, false));
                }
            }
        }
    }

    fn is_noop(&self) -> bool {
        match self {
            Edit::Settings { before, after } => before == after,
            Edit::Track { before, after, .. } => before == after,
            Edit::ClipProps { before, after, .. } => before == after,
            Edit::ChannelKeys { before, after, .. } => before == after,
            Edit::CharacterLevel { before, after, .. } => before == after,
            Edit::InsertClip { .. } | Edit::RemoveClip { .. } => false,
            Edit::InsertTrack { .. } | Edit::RemoveTrack { .. } => false,
            Edit::Batch(edits) => edits.iter().all(Edit::is_noop),
        }
    }

    /// Folds a newer edit of the same target into this one, keeping our `before`.
    /// Returns false (and changes nothing) when the edits touch different things.
    fn absorb(&mut self, newer: &Edit) -> bool {
        match (self, newer) {
            (Edit::Settings { after, .. }, Edit::Settings { after: next, .. }) => {
                *after = next.clone();
                true
            }
            (Edit::Track { id, after, .. }, Edit::Track { id: other, after: next, .. })
                if *id == *other =>
            {
                *after = next.clone();
                true
            }
            (
                Edit::ClipProps { clip, after, .. },
                Edit::ClipProps { clip: other, after: next, .. },
            ) if *clip == *other => {
                *after = next.clone();
                true
            }
            (
                Edit::ChannelKeys {
                    clip, channel, after, ..
                },
                Edit::ChannelKeys {
                    clip: other_clip,
                    channel: other_channel,
                    after: next,
                    ..
                },
            ) if *clip == *other_clip && *channel == *other_channel => {
                *after = next.clone();
                true
            }
            _ => false,
        }
    }
}

struct HistoryEntry {
    label: String,
    edit: Edit,
    /// Open entries may still absorb follow-up edits from the same gesture.
    open: bool,
}

/// A project plus everything needed to edit it safely: undo history, file path, dirty flag.
pub struct Document {
    pub project: Project,
    pub path: Option<PathBuf>,
    undo: Vec<HistoryEntry>,
    redo: Vec<HistoryEntry>,
    dirty: bool,
    /// Increases with every change, so caches (like the simulation) know to refresh.
    revision: u64,
}

impl Document {
    pub fn new(project: Project) -> Self {
        Self {
            project,
            path: None,
            undo: Vec::new(),
            redo: Vec::new(),
            dirty: false,
            revision: 0,
        }
    }

    pub fn opened(project: Project, path: PathBuf) -> Self {
        Self {
            path: Some(path),
            ..Self::new(project)
        }
    }

    /// Applies an edit and records it for undo.
    ///
    /// Pass `continuous = true` for every step of an interactive gesture (dragging a
    /// value, moving a clip, typing a name): consecutive steps on the same target fold
    /// into one undo entry until [`Document::end_gesture`] is called.
    pub fn apply(&mut self, label: &str, edit: Edit, continuous: bool) {
        if edit.is_noop() {
            return;
        }
        edit.apply(&mut self.project, true);
        self.redo.clear();
        self.dirty = true;
        self.revision += 1;

        if let Some(top) = self.undo.last_mut() {
            if continuous && top.open && top.edit.absorb(&edit) {
                return;
            }
            top.open = false;
        }
        self.undo.push(HistoryEntry {
            label: label.to_owned(),
            edit,
            open: continuous,
        });
        if self.undo.len() > UNDO_LIMIT {
            self.undo.remove(0);
        }
    }

    /// Closes the current gesture so the next edit starts a new undo entry.
    pub fn end_gesture(&mut self) {
        if let Some(top) = self.undo.last_mut() {
            top.open = false;
        }
    }

    /// Reverts the most recent edit, returning its label.
    pub fn undo(&mut self) -> Option<String> {
        let mut entry = self.undo.pop()?;
        entry.edit.apply(&mut self.project, false);
        entry.open = false;
        self.dirty = true;
        self.revision += 1;
        let label = entry.label.clone();
        self.redo.push(entry);
        Some(label)
    }

    /// Re-applies the most recently undone edit, returning its label.
    pub fn redo(&mut self) -> Option<String> {
        let entry = self.redo.pop()?;
        entry.edit.apply(&mut self.project, true);
        self.dirty = true;
        self.revision += 1;
        let label = entry.label.clone();
        self.end_gesture();
        self.undo.push(entry);
        Some(label)
    }

    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    pub fn undo_label(&self) -> Option<&str> {
        self.undo.last().map(|e| e.label.as_str())
    }

    pub fn redo_label(&self) -> Option<&str> {
        self.redo.last().map(|e| e.label.as_str())
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn undo_depth(&self) -> usize {
        self.undo.len()
    }

    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    pub fn mark_saved(&mut self, path: PathBuf) {
        self.path = Some(path);
        self.dirty = false;
    }
}
