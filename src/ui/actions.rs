//! Every user command the shell understands. Menus, toolbar buttons and keyboard
//! shortcuts all produce these, and the app runs them in one place.

use crate::document::events::NewClip;
use crate::document::{ClipId, TrackId};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    NewProject,
    OpenProject,
    OpenDemo,
    OpenPhysicsDemo,
    ImportPose,
    ExportBvh,
    ExportFbx,
    SaveProject,
    SaveProjectAs,
    Quit,
    Undo,
    Redo,
    AddKey,
    DuplicateClip,
    /// Adds a clip on a given track at a given frame, or (`None`) for the selected
    /// character at the playhead.
    AddClip { kind: NewClip, at: Option<(TrackId, i32)> },
    /// Re-records an active pose event's pose from the playhead.
    CapturePose(ClipId),
    BakeSimulation,
    Delete,
    Deselect,
    PlayPause,
    Stop,
    StepBackward,
    StepForward,
    JumpBackward,
    JumpForward,
    GoToStart,
    GoToEnd,
    ToggleLoop,
    FitTimeline,
    ResetCamera,
    ShowShortcuts,
    ShowAbout,
}

impl Action {
    pub fn label(self) -> &'static str {
        match self {
            Action::NewProject => "New project",
            Action::OpenProject => "Open project",
            Action::OpenDemo => "Open placeholder demo",
            Action::OpenPhysicsDemo => "Open physics demo",
            Action::ImportPose => "Import pose",
            Action::ExportBvh => "Export BVH",
            Action::ExportFbx => "Export FBX",
            Action::SaveProject => "Save",
            Action::SaveProjectAs => "Save as",
            Action::Quit => "Exit",
            Action::Undo => "Undo",
            Action::Redo => "Redo",
            Action::AddKey => "Add key at playhead",
            Action::DuplicateClip => "Duplicate clip",
            Action::AddClip { .. } => "Add clip",
            Action::CapturePose(_) => "Capture pose",
            Action::BakeSimulation => "Bake simulation to keys",
            Action::Delete => "Delete selection",
            Action::Deselect => "Select none",
            Action::PlayPause => "Play / pause",
            Action::Stop => "Stop and rewind",
            Action::StepBackward => "Previous frame",
            Action::StepForward => "Next frame",
            Action::JumpBackward => "Back 10 frames",
            Action::JumpForward => "Forward 10 frames",
            Action::GoToStart => "Go to start",
            Action::GoToEnd => "Go to end",
            Action::ToggleLoop => "Toggle looping",
            Action::FitTimeline => "Fit timeline to range",
            Action::ResetCamera => "Reset camera",
            Action::ShowShortcuts => "Keyboard shortcuts",
            Action::ShowAbout => "About NeuroMotion",
        }
    }

    /// Whether the shortcut should fire while a text field has keyboard focus.
    pub fn works_while_typing(self) -> bool {
        matches!(
            self,
            Action::NewProject | Action::OpenProject | Action::SaveProject | Action::SaveProjectAs
        )
    }

    /// Whether running this would throw away the current document.
    pub fn discards_document(self) -> bool {
        matches!(
            self,
            Action::NewProject | Action::OpenProject | Action::OpenDemo | Action::OpenPhysicsDemo | Action::Quit
        )
    }
}
