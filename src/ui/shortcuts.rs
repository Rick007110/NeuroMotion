//! Keyboard shortcuts — the single table to edit when remapping keys.
//!
//! `BINDINGS` maps key chords to [`Action`]s. An action may be bound more than once;
//! its first entry is the one shown next to menu items and in the help window.
//! "Ctrl" below means Cmd on macOS.

use eframe::egui::{Context, Key, KeyboardShortcut, Modifiers};

use super::actions::Action;

pub struct Binding {
    pub action: Action,
    pub shortcut: KeyboardShortcut,
}

const fn bind(action: Action, modifiers: Modifiers, key: Key) -> Binding {
    Binding {
        action,
        shortcut: KeyboardShortcut::new(modifiers, key),
    }
}

const NONE: Modifiers = Modifiers::NONE;
const CTRL: Modifiers = Modifiers::COMMAND;
const SHIFT: Modifiers = Modifiers::SHIFT;
const CTRL_SHIFT: Modifiers = Modifiers {
    alt: false,
    ctrl: false,
    shift: true,
    mac_cmd: false,
    command: true,
};

#[rustfmt::skip]
pub const BINDINGS: &[Binding] = &[
    // File
    bind(Action::NewProject,    CTRL,       Key::N),
    bind(Action::OpenProject,   CTRL,       Key::O),
    bind(Action::SaveProject,   CTRL,       Key::S),
    bind(Action::SaveProjectAs, CTRL_SHIFT, Key::S),
    // Edit
    bind(Action::Undo,          CTRL,       Key::Z),
    bind(Action::Redo,          CTRL,       Key::Y),
    bind(Action::Redo,          CTRL_SHIFT, Key::Z),
    bind(Action::AddKey,        NONE,       Key::K),
    bind(Action::DuplicateClip, CTRL,       Key::D),
    bind(Action::Delete,        NONE,       Key::Delete),
    bind(Action::Deselect,      NONE,       Key::Escape),
    // Transport
    bind(Action::PlayPause,     NONE,       Key::Space),
    bind(Action::Stop,          SHIFT,      Key::Space),
    bind(Action::StepBackward,  NONE,       Key::ArrowLeft),
    bind(Action::StepForward,   NONE,       Key::ArrowRight),
    bind(Action::JumpBackward,  SHIFT,      Key::ArrowLeft),
    bind(Action::JumpForward,   SHIFT,      Key::ArrowRight),
    bind(Action::GoToStart,     NONE,       Key::Home),
    bind(Action::GoToEnd,       NONE,       Key::End),
    bind(Action::ToggleLoop,    NONE,       Key::L),
    // View / help
    bind(Action::FitTimeline,   NONE,       Key::F),
    bind(Action::ShowShortcuts, NONE,       Key::F1),
];

/// Consumes this frame's key presses that match a binding and returns their actions.
/// Called before any widget runs, so e.g. Space never also "clicks" a focused button.
pub fn collect(ctx: &Context) -> Vec<Action> {
    let typing = ctx.wants_keyboard_input();
    // Most specific chords first, so Shift+Space isn't swallowed by plain Space.
    let mut ordered: Vec<&Binding> = BINDINGS.iter().collect();
    ordered.sort_by_key(|b| std::cmp::Reverse(modifier_count(b.shortcut.modifiers)));

    let mut actions = Vec::new();
    ctx.input_mut(|input| {
        for binding in ordered {
            if typing && !binding.action.works_while_typing() {
                continue;
            }
            if input.consume_shortcut(&binding.shortcut) {
                actions.push(binding.action);
            }
        }
    });
    actions
}

/// Human-readable chord for an action's primary binding, or "" if it has none.
pub fn text_for(ctx: &Context, action: Action) -> String {
    BINDINGS
        .iter()
        .find(|b| b.action == action)
        .map(|b| ctx.format_shortcut(&b.shortcut))
        .unwrap_or_default()
}

fn modifier_count(m: Modifiers) -> u8 {
    m.alt as u8 + m.shift as u8 + (m.ctrl || m.command || m.mac_cmd) as u8
}
