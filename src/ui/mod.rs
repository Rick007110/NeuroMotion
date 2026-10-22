//! Everything egui: the application shell, panels, menus and keyboard handling.
//!
//! * [`app`] – window layout and action dispatch
//! * [`menu`] – File / Edit / View / Help
//! * [`export`] – the File → Export window
//! * [`timeline_panel`] – transport, ruler, tracks, clips, keys
//! * [`inspector`] – property editing for the current selection
//! * [`event_inspector`] – typed settings for physics events
//! * [`shortcuts`] – the keyboard binding table
//! * [`actions`], [`selection`], [`theme`], [`icons`] – shared pieces

mod actions;
mod app;
mod event_inspector;
mod export;
mod icons;
mod inspector;
mod menu;
mod selection;
mod shortcuts;
mod theme;
mod timeline_panel;

pub use app::NeuroMotionApp;
