//! Time-domain logic behind the timeline: transport state and the frame ↔ pixel mapping.
//!
//! Nothing in here draws. The egui panel that uses it lives in `ui::timeline_panel`.

mod playback;
mod view;

pub use playback::Playback;
pub use view::TimelineView;
