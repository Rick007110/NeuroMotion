//! Every user command the shell understands. Menus, toolbar buttons and keyboard
//! shortcuts all produce these, and the app runs them in one place.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    NewProject,
    OpenProject,
    OpenDemo,
    ImportPose,
    SaveProject,
    SaveProjectAs,
    Quit,
    Undo,
    Redo,
    AddKey,
    DuplicateClip,
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
            Action::ImportPose => "Import pose",
            Action::SaveProject => "Save",
            Action::SaveProjectAs => "Save as",
            Action::Quit => "Exit",
            Action::Undo => "Undo",
            Action::Redo => "Redo",
            Action::AddKey => "Add key at playhead",
            Action::DuplicateClip => "Duplicate clip",
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
            Action::NewProject | Action::OpenProject | Action::OpenDemo | Action::Quit
        )
    }
}
