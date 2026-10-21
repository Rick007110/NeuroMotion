//! What the user has selected. This is UI state, not part of the document, so it is
//! never saved and selecting things never creates undo entries.

use crate::document::{CharacterId, ClipId, Project, TrackId};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Selection {
    #[default]
    None,
    Character(CharacterId),
    Track(TrackId),
    Clip(ClipId),
    Channel { clip: ClipId, channel: usize },
    /// Keys are identified by their clip-local frame (unique within a channel).
    Key { clip: ClipId, channel: usize, frame: i32 },
}

impl Selection {
    /// The clip this selection belongs to, if any.
    pub fn clip(self) -> Option<ClipId> {
        match self {
            Selection::Clip(clip)
            | Selection::Channel { clip, .. }
            | Selection::Key { clip, .. } => Some(clip),
            _ => None,
        }
    }

    pub fn channel(self) -> Option<(ClipId, usize)> {
        match self {
            Selection::Channel { clip, channel } | Selection::Key { clip, channel, .. } => {
                Some((clip, channel))
            }
            _ => None,
        }
    }

    pub fn is_deletable(self) -> bool {
        matches!(self, Selection::Track(_) | Selection::Clip(_) | Selection::Key { .. })
    }

    /// Falls back to the nearest still-existing parent after undo/redo or deletion
    /// removed the selected thing.
    pub fn validate(&mut self, project: &Project) {
        loop {
            let valid = match *self {
                Selection::None => true,
                Selection::Character(id) => project.character(id).is_some(),
                Selection::Track(id) => project.track(id).is_some(),
                Selection::Clip(id) => project.clip(id).is_some(),
                Selection::Channel { clip, channel } => project
                    .clip(clip)
                    .is_some_and(|c| channel < c.channels.len()),
                Selection::Key { clip, channel, frame } => project
                    .clip(clip)
                    .and_then(|c| c.channels.get(channel))
                    .is_some_and(|ch| ch.key(frame).is_some()),
            };
            if valid {
                return;
            }
            *self = match *self {
                Selection::Key { clip, channel, .. } => Selection::Channel { clip, channel },
                Selection::Channel { clip, .. } => Selection::Clip(clip),
                _ => Selection::None,
            };
        }
    }
}
