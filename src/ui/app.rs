//! The application shell: owns the document and UI state, lays out the panels and runs
//! actions coming from menus, buttons and shortcuts.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use eframe::egui::{self, Align2, RichText, ViewportCommand};

use super::actions::Action;
use super::selection::Selection;
use super::timeline_panel::{self, TimelineState};
use super::{inspector, shortcuts, theme};
use crate::document::{ClipKind, ClipProps, Document, Project, SimLevel, sample};
use crate::io;
use crate::physics::Simulation;
use crate::timeline::Playback;
use crate::viewport::{CharacterView, OrbitCamera, Viewport};

/// Time per UI frame spent advancing the simulation, so editing stays responsive.
const SIMULATION_BUDGET: Duration = Duration::from_millis(12);

/// How long an imported pose is held on the timeline, in frames.
const POSE_CLIP_FRAMES: i32 = 30;

pub struct NeuroMotionApp {
    pub(super) doc: Document,
    pub(super) selection: Selection,
    pub(super) viewport: Viewport,
    pub(super) show_inspector: bool,
    pub(super) show_timeline: bool,
    playback: Playback,
    timeline: TimelineState,
    simulation: Simulation,
    /// Whether the project needs physics, cached per document revision.
    physics_needed: Option<(u64, bool)>,
    status: String,
    show_about: bool,
    /// Notes from the last import, shown until dismissed.
    import_report: Option<(String, Vec<String>)>,
    /// A file to open instead of asking (drag-and-drop), consumed by `Action::OpenProject`.
    queued_open: Option<PathBuf>,
    show_shortcuts: bool,
    /// An action waiting for the user to confirm discarding unsaved changes.
    pending_discard: Option<Action>,
    close_confirmed: bool,
    title: String,
}

impl NeuroMotionApp {
    /// `startup` is a file to open immediately, e.g. from the command line.
    pub fn new(cc: &eframe::CreationContext<'_>, startup: Option<PathBuf>) -> Self {
        theme::apply(&cc.egui_ctx);
        let project = sample::placeholder_project();
        let mut app = Self {
            playback: Playback::new(project.settings.range),
            doc: Document::new(project),
            selection: Selection::None,
            viewport: Viewport::default(),
            show_inspector: true,
            show_timeline: true,
            timeline: TimelineState::default(),
            simulation: Simulation::default(),
            physics_needed: None,
            status: "Placeholder project loaded. Press Space to play.".to_owned(),
            show_about: false,
            import_report: None,
            queued_open: None,
            show_shortcuts: false,
            pending_discard: None,
            close_confirmed: false,
            title: String::new(),
        };
        if let Some(path) = startup {
            app.open_path(path);
        }
        app
    }

    fn load(&mut self, project: Project, path: Option<PathBuf>) {
        self.playback = Playback::new(project.settings.range);
        self.doc = match path {
            Some(path) => Document::opened(project, path),
            None => Document::new(project),
        };
        self.selection = Selection::None;
        self.timeline.fit_requested = true;
        self.viewport.frame_requested = true;
        self.simulation.reset();
        self.physics_needed = None;
    }

    /// Keeps the simulation in step with the document, computing a little more each frame.
    fn advance_simulation(&mut self, ctx: &egui::Context) {
        let revision = self.doc.revision();
        let needed = match self.physics_needed {
            Some((r, needed)) if r == revision => needed,
            _ => {
                let needed = Simulation::is_needed(&self.doc.project);
                self.physics_needed = Some((revision, needed));
                needed
            }
        };
        if !needed {
            return;
        }
        // Any edit bumps the revision, which throws away stale frames.
        self.simulation.sync(revision);
        if !self.simulation.advance(&self.doc.project, Instant::now() + SIMULATION_BUDGET) {
            ctx.request_repaint();
        }
    }

    fn physics_on(&self) -> bool {
        self.physics_needed.is_some_and(|(_, n)| n)
    }

    /// What the viewport shows for each character: the simulation where it has been
    /// computed, otherwise the animation.
    fn character_views(&self) -> Vec<CharacterView> {
        let project = &self.doc.project;
        let poses = project.evaluate(self.playback.playhead() as f32);
        let simulated = self.physics_on().then(|| self.simulation.frame(self.playback.frame())).flatten();
        project
            .characters
            .iter()
            .enumerate()
            .map(|(ci, character)| match simulated.and_then(|f| f.get(ci)).and_then(Option::as_ref) {
                Some(frame) => CharacterView {
                    joints: frame.joints.iter().map(|j| j.1).collect(),
                    bodies: Some(frame.bodies.clone()),
                    simulated: frame.level == SimLevel::Simulated,
                },
                None => CharacterView::animated(&character.skeleton, character.physics.as_ref(), &poses[ci]),
            })
            .collect()
    }

    fn run(&mut self, ctx: &egui::Context, action: Action) {
        if action.discards_document() && self.doc.is_dirty() && !self.close_confirmed {
            self.pending_discard = Some(action);
            return;
        }
        let range = self.doc.project.settings.range;
        match action {
            Action::NewProject => {
                self.load(sample::blank_project(), None);
                self.status = "New project.".into();
            }
            Action::OpenDemo => {
                self.load(sample::placeholder_project(), None);
                self.status = "Placeholder project loaded.".into();
            }
            Action::OpenPhysicsDemo => {
                self.load(sample::physics_demo_project(), None);
                self.status = "Physics demo loaded: press Space. The character is shoved at frame 45.".into();
            }
            Action::OpenProject => match self.queued_open.take() {
                Some(path) => self.open_path(path),
                None => self.open(),
            },
            Action::ImportPose => self.import_pose(None),
            Action::SaveProject => self.save(false),
            Action::SaveProjectAs => self.save(true),
            Action::Quit => {
                self.close_confirmed = true;
                ctx.send_viewport_cmd(ViewportCommand::Close);
            }
            Action::Undo => {
                self.doc.end_gesture();
                self.status = match self.doc.undo() {
                    Some(label) => format!("Undid: {label}"),
                    None => "Nothing to undo.".into(),
                };
            }
            Action::Redo => {
                self.status = match self.doc.redo() {
                    Some(label) => format!("Redid: {label}"),
                    None => "Nothing to redo.".into(),
                };
            }
            Action::AddKey => self.add_key_at_playhead(),
            Action::DuplicateClip => match self.selection.clip() {
                Some(clip) => {
                    if let Some(copy) = self.doc.duplicate_clip(clip) {
                        self.selection = Selection::Clip(copy);
                    }
                }
                None => self.status = "Select a clip to duplicate.".into(),
            },
            Action::Delete => self.delete_selection(),
            Action::Deselect => self.selection = Selection::None,
            Action::PlayPause => self.playback.toggle(range),
            Action::Stop => self.playback.stop(range),
            Action::StepBackward => self.playback.step(-1, range),
            Action::StepForward => self.playback.step(1, range),
            Action::JumpBackward => self.playback.step(-10, range),
            Action::JumpForward => self.playback.step(10, range),
            Action::GoToStart => self.playback.seek(range.start as f64, range),
            Action::GoToEnd => self.playback.seek(range.end as f64, range),
            Action::ToggleLoop => {
                self.playback.looping = !self.playback.looping;
                self.status = if self.playback.looping { "Looping on." } else { "Looping off." }.into();
            }
            Action::FitTimeline => self.timeline.fit_requested = true,
            Action::ResetCamera => {
                self.viewport.camera = OrbitCamera::default();
                self.viewport.frame_requested = true;
            }
            Action::ShowShortcuts => self.show_shortcuts = true,
            Action::ShowAbout => self.show_about = true,
        }
    }

    fn open(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("All supported files", &io::OPEN_EXTENSIONS)
            .add_filter("NeuroMotion project", &[io::FILE_EXTENSION])
            .add_filter("endorphin scene", &["ens"])
            .add_filter("endorphin character or prop", &["nmc"])
            .pick_file()
        else {
            return;
        };
        self.open_path(path);
    }

    /// Opens a NeuroMotion project, or imports an endorphin scene or character.
    fn open_path(&mut self, path: PathBuf) {
        match io::open(&path) {
            Ok(opened) if opened.native => {
                self.status = format!("Opened {}", path.display());
                self.load(opened.project, Some(path));
            }
            Ok(opened) => {
                self.status = format!("Imported {}. Save to keep it as a NeuroMotion project.", path.display());
                self.load(opened.project, None);
                self.import_report = Some((file_name(&path), opened.notes));
            }
            Err(e) => self.status = format!("Couldn't open {}: {e}", path.display()),
        }
    }

    /// Puts an endorphin pose on the best-matching character, held at the playhead.
    fn import_pose(&mut self, path: Option<PathBuf>) {
        let path = match path {
            Some(path) => path,
            None => match rfd::FileDialog::new().add_filter("endorphin pose", &["nma"]).pick_file() {
                Some(path) => path,
                None => return,
            },
        };
        let pose = match io::endorphin::import_pose(&path, &self.doc.project) {
            Ok(pose) => pose,
            Err(e) => {
                self.status = format!("Couldn't import {}: {e}", path.display());
                return;
            }
        };
        let Some(track) = self.doc.project.tracks_of(pose.character).next().map(|t| t.id) else {
            self.status = "That character has no track to put the pose on.".into();
            return;
        };
        let start = self.playback.frame();
        let props = ClipProps {
            name: pose.name.clone(),
            track,
            start,
            duration: POSE_CLIP_FRAMES,
            blend_in: 6,
            blend_out: 6,
            weight: 1.0,
            enabled: true,
        };
        let clip = self.doc.insert_clip(props, ClipKind::Motion, pose.channels);
        self.selection = Selection::Clip(clip);
        self.status = format!("Imported pose \u{201c}{}\u{201d} at frame {start}.", pose.name);
    }

    /// Files dropped on the window: poses go onto the timeline, anything else opens.
    fn handle_dropped_files(&mut self, ctx: &egui::Context, actions: &mut Vec<Action>) {
        let dropped: Vec<PathBuf> = ctx.input(|i| i.raw.dropped_files.iter().filter_map(|f| f.path.clone()).collect());
        for path in dropped {
            let is_pose = path.extension().is_some_and(|e| e.eq_ignore_ascii_case("nma"));
            if is_pose {
                self.import_pose(Some(path));
            } else {
                self.queued_open = Some(path);
                actions.push(Action::OpenProject);
            }
        }
    }

    fn save(&mut self, choose_path: bool) {
        let path = match (&self.doc.path, choose_path) {
            (Some(path), false) => path.clone(),
            _ => {
                let suggested = format!("{}.{}", self.doc.project.settings.name, io::FILE_EXTENSION);
                let Some(path) = rfd::FileDialog::new()
                    .add_filter("NeuroMotion project", &[io::FILE_EXTENSION])
                    .set_file_name(suggested)
                    .save_file()
                else {
                    return;
                };
                path
            }
        };
        match io::save(&path, &self.doc.project) {
            Ok(()) => {
                self.status = format!("Saved {}", path.display());
                self.doc.mark_saved(path);
            }
            Err(e) => self.status = format!("Couldn't save {}: {e}", path.display()),
        }
    }

    fn add_key_at_playhead(&mut self) {
        let Some((clip_id, channel)) = self.selection.channel() else {
            self.status = "Select a channel (or key) to add a key to.".into();
            return;
        };
        let Some(clip) = self.doc.project.clip(clip_id) else { return };
        let frame = self.playback.frame();
        if !clip.props.contains(frame as f32) {
            self.status = "The playhead is outside the selected clip.".into();
            return;
        }
        let local = frame - clip.props.start;
        if self.doc.insert_key(clip_id, channel, local) {
            self.selection = Selection::Key { clip: clip_id, channel, frame: local };
            self.status = format!("Keyed at frame {frame}.");
        }
    }

    fn delete_selection(&mut self) {
        match self.selection {
            Selection::Key { clip, channel, frame } => {
                if self.doc.remove_key(clip, channel, frame) {
                    self.selection = Selection::Channel { clip, channel };
                }
            }
            Selection::Clip(clip) => {
                if self.doc.remove_clip(clip) {
                    self.selection = Selection::None;
                }
            }
            _ => self.status = "Select a clip or key to delete.".into(),
        }
    }

    fn update_title(&mut self, ctx: &egui::Context) {
        let name = match &self.doc.path {
            Some(path) => path.file_name().map_or_else(String::new, |n| n.to_string_lossy().into_owned()),
            None => self.doc.project.settings.name.clone(),
        };
        let title = format!("{}{name} — NeuroMotion", if self.doc.is_dirty() { "• " } else { "" });
        if title != self.title {
            ctx.send_viewport_cmd(ViewportCommand::Title(title.clone()));
            self.title = title;
        }
    }

    fn status_bar(&self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let file = self
                .doc
                .path
                .as_ref()
                .map_or("unsaved project".to_owned(), |p| p.display().to_string());
            ui.weak(file);
            if self.doc.is_dirty() {
                ui.label(RichText::new("modified").color(theme::KEY).small());
            }
            ui.separator();
            ui.label(&self.status);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.weak(format!(
                    "{} · frame {} · {} undo steps",
                    self.playback.state().label(),
                    self.playback.frame(),
                    self.doc.undo_depth()
                ));
            });
        });
    }

    fn hud_lines(&self) -> Vec<String> {
        let project = &self.doc.project;
        let playhead = self.playback.playhead() as f32;
        let mut lines = vec![format!(
            "{}  ·  frame {:>4}  ·  {}",
            project.settings.name,
            self.playback.frame(),
            self.playback.state().label()
        )];
        for (id, weight) in project.active_clips(playhead) {
            let Some(clip) = project.clip(id) else { continue };
            let who = project.clip_character(clip).map_or("", |c| c.name.as_str());
            let kind = match &clip.kind {
                ClipKind::Motion => format!("{:>3.0}%", weight * 100.0),
                ClipKind::Event { event } => event.to_lowercase(),
            };
            lines.push(format!("  {who:<12} {:<20} {kind}", clip.props.name));
        }
        if self.physics_on() {
            let (done, total) = self.simulation.progress();
            if done < total {
                lines.push(format!("physics: simulating {done}/{total} frames"));
            } else {
                lines.push("physics: up to date".to_owned());
            }
            if let Some(frame) = self.simulation.frame(self.playback.frame()) {
                for (character, state) in project.characters.iter().zip(frame) {
                    if let Some(state) = state.as_ref().filter(|_| character.kind != crate::document::CharacterKind::Environment) {
                        lines.push(format!("  {:<12} {}", character.name, state.level.label().to_lowercase()));
                    }
                }
            }
        }
        lines
    }

    fn windows(&mut self, ctx: &egui::Context, actions: &mut Vec<Action>) {
        if let Some(action) = self.pending_discard {
            egui::Window::new("Unsaved changes")
                .collapsible(false)
                .resizable(false)
                .anchor(Align2::CENTER_CENTER, [0.0, 0.0])
                .show(ctx, |ui| {
                    ui.label(format!("“{}” has unsaved changes.", self.doc.project.settings.name));
                    ui.label(format!("{} anyway and lose them?", action.label()));
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        if ui.button("Discard changes").clicked() {
                            self.pending_discard = None;
                            self.close_confirmed = true;
                            actions.push(action);
                        }
                        if ui.button("Cancel").clicked() {
                            self.pending_discard = None;
                            self.queued_open = None;
                        }
                    });
                });
        }

        let mut close_report = false;
        if let Some((file, notes)) = &self.import_report {
            egui::Window::new("Import report")
                .collapsible(false)
                .resizable(false)
                .anchor(Align2::RIGHT_TOP, [-330.0, 40.0])
                .show(ctx, |ui| {
                    ui.label(RichText::new(file).strong());
                    for note in notes {
                        ui.label(format!("\u{2022} {note}"));
                    }
                    ui.add_space(4.0);
                    close_report = ui.button("OK").clicked();
                });
        }
        if close_report {
            self.import_report = None;
        }

        egui::Window::new("Keyboard shortcuts")
            .open(&mut self.show_shortcuts)
            .resizable(false)
            .collapsible(false)
            .show(ctx, |ui| {
                ui.weak("Defined in src/ui/shortcuts.rs (BINDINGS).");
                ui.add_space(4.0);
                egui::Grid::new("shortcut_table").striped(true).num_columns(2).show(ui, |ui| {
                    for binding in shortcuts::BINDINGS {
                        ui.label(binding.action.label());
                        ui.label(RichText::new(ctx.format_shortcut(&binding.shortcut)).monospace());
                        ui.end_row();
                    }
                });
            });

        egui::Window::new("About NeuroMotion")
            .open(&mut self.show_about)
            .resizable(false)
            .collapsible(false)
            .show(ctx, |ui| {
                ui.label(RichText::new("NeuroMotion").size(20.0).color(theme::ACCENT));
                ui.label(format!("Version {}", env!("CARGO_PKG_VERSION")));
                ui.label("A timeline-based motion editor for character animation.");
                ui.weak("Phase 1: editor shell.");
            });
    }
}

impl eframe::App for NeuroMotionApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Shortcuts first, so their keys never reach widgets.
        let mut actions = shortcuts::collect(ctx);
        self.handle_dropped_files(ctx, &mut actions);
        self.advance_simulation(ctx);

        if ctx.input(|i| i.viewport().close_requested()) && self.doc.is_dirty() && !self.close_confirmed {
            ctx.send_viewport_cmd(ViewportCommand::CancelClose);
            self.pending_discard = Some(Action::Quit);
        }

        let settings = &self.doc.project.settings;
        let dt = ctx.input(|i| i.unstable_dt).min(0.1) as f64;
        self.playback.advance(dt, settings.fps as f64, settings.range);
        if self.playback.is_playing() {
            ctx.request_repaint();
        }
        self.selection.validate(&self.doc.project);

        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| self.menu_bar(ui, &mut actions));
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| self.status_bar(ui));
        if self.show_timeline {
            egui::TopBottomPanel::bottom("timeline")
                .resizable(true)
                .default_height(270.0)
                .min_height(140.0)
                .show(ctx, |ui| {
                    timeline_panel::show(
                        ui,
                        &mut self.timeline,
                        &mut self.doc,
                        &mut self.playback,
                        &mut self.selection,
                        &mut actions,
                    )
                });
        }
        if self.show_inspector {
            egui::SidePanel::right("inspector")
                .resizable(true)
                .default_width(310.0)
                .min_width(240.0)
                .show(ctx, |ui| {
                    let playhead = self.playback.playhead() as f32;
                    inspector::show(ui, &mut self.doc, &mut self.selection, playhead)
                });
        }
        let hud = self.hud_lines();
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ctx, |ui| {
                let views = self.character_views();
                self.viewport.show(ui, &self.doc.project, &views, &hud);
            });

        self.windows(ctx, &mut actions);
        for action in actions {
            self.run(ctx, action);
        }
        self.update_title(ctx);
    }
}

fn file_name(path: &std::path::Path) -> String {
    path.file_name().map_or_else(|| path.display().to_string(), |n| n.to_string_lossy().into_owned())
}
