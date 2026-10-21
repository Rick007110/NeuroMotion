//! NeuroMotion — a timeline-based motion editor for character animation.
//!
//! Module map:
//! * `document` – project data model, pose evaluation, undo/redo (no UI)
//! * `timeline` – transport/playhead and frame↔pixel mapping (no drawing)
//! * `io`       – project file load/save
//! * `viewport` – camera and character preview drawing
//! * `ui`       – the egui application shell and panels

mod document;
mod io;
mod timeline;
mod ui;
mod viewport;

use eframe::egui;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("NeuroMotion")
            .with_inner_size([1440.0, 880.0])
            .with_min_inner_size([960.0, 600.0]),
        ..Default::default()
    };
    // A file given on the command line (or via "Open with") opens on startup.
    let startup = std::env::args_os().nth(1).map(std::path::PathBuf::from);
    eframe::run_native(
        "NeuroMotion",
        options,
        Box::new(|cc| Ok(Box::new(ui::NeuroMotionApp::new(cc, startup)))),
    )
}
