//! Bottom timeline: transport bar, frame ruler, track lanes with clips, and key rows
//! for the selected clip.
//!
//! Layout, left to right: a fixed header column naming each row, then the lanes where
//! time runs horizontally. The playhead spans ruler and lanes. Clicking or dragging on
//! the ruler or on empty lane space scrubs; clips and keys can be selected and dragged.

use eframe::egui::{
    self, Align2, Color32, CursorIcon, DragValue, FontId, Painter, Rect, Response, Sense, Shape,
    Stroke, StrokeKind, Ui, pos2, vec2,
};

use super::actions::Action;
use super::icons::{self, Icon};
use super::inspector::Gesture;
use super::menu::add_clip_items;
use super::selection::Selection;
use super::shortcuts;
use super::theme;
use crate::document::{CharacterId, ClipId, ClipProps, Document, Edit, Keyframe, TrackId};
use crate::timeline::{Playback, TimelineView};

const HEADER_W: f32 = 176.0;
const RULER_H: f32 = 24.0;
const CHARACTER_H: f32 = 22.0;
const TRACK_H: f32 = 34.0;
const CHANNEL_H: f32 = 20.0;
const KEY_SIZE: f32 = 5.0;

pub struct TimelineState {
    pub view: TimelineView,
    pub fit_requested: bool,
    /// After a fit, keep refitting when the panel resizes until the user zooms or pans.
    auto_fit: bool,
    last_width: f32,
    scroll_y: f32,
    drag: Option<Drag>,
    /// Timeline frame under the pointer when a lane's context menu opened.
    menu_frame: i32,
}

impl Default for TimelineState {
    fn default() -> Self {
        Self {
            view: TimelineView::default(),
            fit_requested: true,
            auto_fit: true,
            last_width: 0.0,
            scroll_y: 0.0,
            drag: None,
            menu_frame: 0,
        }
    }
}

/// An in-progress pointer drag. `travel` accumulates pixels so slow drags still move
/// by whole frames.
enum Drag {
    Clip { id: ClipId, origin: i32, travel: f32 },
    Key { clip: ClipId, channel: usize, origin: i32, current: i32, travel: f32 },
}

enum Row {
    Character { id: CharacterId },
    Track { index: usize, id: TrackId },
    Channel { clip: ClipId, channel: usize },
}

impl Row {
    fn height(&self) -> f32 {
        match self {
            Row::Character { .. } => CHARACTER_H,
            Row::Track { .. } => TRACK_H,
            Row::Channel { .. } => CHANNEL_H,
        }
    }
}

pub fn show(
    ui: &mut Ui,
    state: &mut TimelineState,
    doc: &mut Document,
    playback: &mut Playback,
    selection: &mut Selection,
    actions: &mut Vec<Action>,
) {
    transport_bar(ui, state, doc, playback, actions);
    ui.add_space(2.0);

    let (rect, _) = ui.allocate_exact_size(ui.available_size(), Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, theme::LANE_BG);

    let range = doc.project.settings.range;
    let lanes_left = rect.left() + HEADER_W;
    let lane_width = (rect.right() - lanes_left).max(1.0);
    let resized = (lane_width - state.last_width).abs() > 0.5;
    if state.fit_requested || (state.auto_fit && resized) {
        state.view.fit(range, lane_width);
        state.fit_requested = false;
        state.auto_fit = true;
    }
    state.last_width = lane_width;

    let ruler = Rect::from_min_max(pos2(lanes_left, rect.top()), pos2(rect.right(), rect.top() + RULER_H));
    let body = Rect::from_min_max(pos2(rect.left(), ruler.bottom()), rect.max);
    let lanes = Rect::from_min_max(pos2(lanes_left, body.top()), body.max);

    handle_wheel(ui, state, rect, lanes_left);
    if playback.is_playing() {
        state.view.follow(playback.playhead() as f32, lane_width);
    }

    let rows = build_rows(doc, *selection);
    let content_h: f32 = rows.iter().map(Row::height).sum();
    state.scroll_y = state.scroll_y.clamp(0.0, (content_h - body.height()).max(0.0));

    let view = state.view.clone();
    let to_x = |frame: f32| view.frame_to_x(frame, lanes_left);

    // Vertical frame grid behind everything in the lanes.
    let step = view.tick_step(64.0);
    let grid_painter = painter.with_clip_rect(lanes);
    for frame in visible_ticks(&view, lanes_left, lane_width, step) {
        let x = to_x(frame as f32);
        grid_painter.line_segment([pos2(x, lanes.top()), pos2(x, lanes.bottom())], Stroke::new(1.0_f32, theme::GRID));
    }

    let mut y = body.top() - state.scroll_y;
    let mut stripe = 0;
    for row in &rows {
        let row_rect = Rect::from_min_size(pos2(rect.left(), y), vec2(rect.width(), row.height()));
        y += row.height();
        if row_rect.bottom() < body.top() || row_rect.top() > body.bottom() {
            continue;
        }
        let header = Rect::from_min_max(row_rect.min, pos2(lanes_left, row_rect.bottom()));
        let lane = Rect::from_min_max(pos2(lanes_left, row_rect.top()), row_rect.max);
        let header_painter = painter.with_clip_rect(header.intersect(body));
        let lane_painter = painter.with_clip_rect(lane.intersect(body));

        // Empty lane space scrubs and clears the selection. Registered before the clips
        // and keys so those sit on top and win the pointer.
        let lane_id = match *row {
            Row::Character { id } => ui.id().with(("character_lane", id)),
            Row::Track { id, .. } => ui.id().with(("lane", id)),
            Row::Channel { clip, channel } => ui.id().with(("channel_lane", clip, channel)),
        };
        let lane_response = ui.interact(lane.intersect(body), lane_id, Sense::click_and_drag());
        scrub(&lane_response, &view, lanes_left, playback, range);

        match *row {
            Row::Character { id } => {
                lane_painter.rect_filled(lane, 0.0, theme::CHARACTER_BG);
                if lane_response.clicked() {
                    *selection = Selection::None;
                }
                character_header(ui, &header_painter, header.intersect(body), header, doc, selection, id);
            }
            Row::Track { index, id } => {
                let fill = if stripe % 2 == 0 { theme::LANE_BG } else { theme::LANE_ALT };
                stripe += 1;
                lane_painter.rect_filled(lane, 0.0, fill);
                if lane_response.clicked() {
                    *selection = Selection::None;
                }
                // Right-click: add a clip where the pointer is. The playhead moves there
                // too, so an active pose captures the pose at its own start.
                if lane_response.secondary_clicked() {
                    if let Some(pos) = lane_response.interact_pointer_pos() {
                        state.menu_frame = view.x_to_frame(pos.x, lanes_left).round() as i32;
                        playback.pause();
                        playback.seek(state.menu_frame as f64, range);
                    }
                }
                let frame = state.menu_frame;
                lane_response.context_menu(|ui| track_menu(ui, doc, selection, actions, id, Some(frame)));
                track_header(ui, &header_painter, header.intersect(body), header, doc, selection, actions, index, id);
                track_lane(ui, &lane_painter, lane, lanes, &view, lanes_left, state, doc, selection, actions, index, id);
            }
            Row::Channel { clip, channel } => {
                lane_painter.rect_filled(lane, 0.0, theme::CHANNEL_BG);
                channel_header(ui, &header_painter, header.intersect(body), header, doc, selection, clip, channel);
                channel_lane(ui, &lane_painter, &lane_response, lane, lanes, &view, lanes_left, state, doc, selection, clip, channel);
            }
        }
        painter.with_clip_rect(body).line_segment(
            [row_rect.left_bottom(), row_rect.right_bottom()],
            Stroke::new(1.0_f32, Color32::from_black_alpha(60)),
        );
    }

    // Header column divider, out-of-range shading, ruler, then the playhead on top.
    painter.line_segment([pos2(lanes_left, rect.top()), pos2(lanes_left, rect.bottom())], Stroke::new(1.0_f32, Color32::from_black_alpha(120)));
    for shade in [
        Rect::from_min_max(pos2(lanes_left, ruler.top()), pos2(to_x(range.start as f32).clamp(lanes_left, rect.right()), rect.bottom())),
        Rect::from_min_max(pos2(to_x(range.end as f32).clamp(lanes_left, rect.right()), ruler.top()), rect.max),
    ] {
        if shade.width() > 0.0 {
            painter.rect_filled(shade, 0.0, theme::OUT_OF_RANGE);
        }
    }
    draw_ruler(ui, &painter, rect, ruler, &view, lanes_left, lane_width, playback, range);
    draw_playhead(&painter, rect, ruler, to_x(playback.playhead() as f32), playback.frame());
}

/// Characters in project order, each followed by its tracks; the selected clip's
/// channels unfold under its track.
fn build_rows(doc: &Document, selection: Selection) -> Vec<Row> {
    let project = &doc.project;
    let selected_clip = selection.clip().and_then(|id| project.clip(id));
    let mut rows = Vec::new();
    for character in &project.characters {
        rows.push(Row::Character { id: character.id });
        for (index, track) in project.tracks.iter().enumerate() {
            if track.character != character.id {
                continue;
            }
            rows.push(Row::Track { index, id: track.id });
            if let Some(clip) = selected_clip.filter(|c| c.props.track == track.id) {
                rows.extend((0..clip.channels.len()).map(|channel| Row::Channel { clip: clip.id, channel }));
            }
        }
    }
    rows
}

fn transport_bar(
    ui: &mut Ui,
    state: &mut TimelineState,
    doc: &mut Document,
    playback: &mut Playback,
    actions: &mut Vec<Action>,
) {
    let range = doc.project.settings.range;
    let fps = doc.project.settings.fps;
    let ctx = ui.ctx().clone();
    let tip = |action: Action| {
        let keys = shortcuts::text_for(&ctx, action);
        if keys.is_empty() { action.label().to_owned() } else { format!("{} ({keys})", action.label()) }
    };

    ui.horizontal(|ui| {
        let playing = playback.is_playing();
        let buttons = [
            (Icon::ToStart, Action::GoToStart, false),
            (Icon::StepBack, Action::StepBackward, false),
            (if playing { Icon::Pause } else { Icon::Play }, Action::PlayPause, playing),
            (Icon::Stop, Action::Stop, false),
            (Icon::StepForward, Action::StepForward, false),
            (Icon::ToEnd, Action::GoToEnd, false),
        ];
        for (icon, action, active) in buttons {
            if icons::button(ui, icon, &tip(action), active).clicked() {
                actions.push(action);
            }
        }
        ui.separator();

        let mut frame = playback.frame();
        let field = ui.add(DragValue::new(&mut frame).range(range.start..=range.end).prefix("frame "));
        if field.changed() {
            playback.seek(frame as f64, range);
        }
        // The scene's last frame, editable here as well as on the project page.
        let mut end = range.end;
        let end_field = ui
            .add(DragValue::new(&mut end).range(range.start + 1..=100_000).prefix("/ "))
            .on_hover_text("Last frame of the scene: drag or type to change it");
        let mut gesture = Gesture::default();
        gesture.drag(&end_field);
        let edit = (end != range.end).then(|| {
            let before = doc.project.settings.clone();
            let mut after = before.clone();
            after.range.end = end;
            Edit::Settings { before, after }
        });
        if edit.is_some() && state.auto_fit {
            state.fit_requested = true;
        }
        gesture.commit(doc, "Change end frame", edit);
        ui.weak(timecode(playback.playhead(), fps));
        ui.separator();
        if icons::button(ui, Icon::Loop, &tip(Action::ToggleLoop), playback.looping).clicked() {
            actions.push(Action::ToggleLoop);
        }
        ui.weak(format!("{fps:.0} fps"));

        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.small_button("Fit").on_hover_text(tip(Action::FitTimeline)).clicked() {
                actions.push(Action::FitTimeline);
            }
            let anchor = playback.playhead() as f32;
            if ui.small_button("+").on_hover_text("Zoom in (Ctrl+wheel)").clicked() {
                state.view.zoom_at(1.25, anchor);
                state.auto_fit = false;
            }
            if ui.small_button("−").on_hover_text("Zoom out (Ctrl+wheel)").clicked() {
                state.view.zoom_at(0.8, anchor);
                state.auto_fit = false;
            }
            ui.weak(playback.state().label());
        });
    });
}

fn timecode(playhead: f64, fps: f32) -> String {
    let seconds = playhead / fps.max(1.0) as f64;
    format!("{:02}:{:05.2}", (seconds / 60.0).floor() as i64, seconds % 60.0)
}

/// Wheel: scroll rows. Shift+wheel: pan time. Ctrl+wheel: zoom around the pointer.
fn handle_wheel(ui: &Ui, state: &mut TimelineState, rect: Rect, lanes_left: f32) {
    if !ui.rect_contains_pointer(rect) {
        return;
    }
    let (scroll, modifiers, pointer) = ui.input(|i| (i.raw_scroll_delta, i.modifiers, i.pointer.hover_pos()));
    if scroll == egui::Vec2::ZERO {
        return;
    }
    if modifiers.command {
        let anchor = pointer.map_or(state.view.first_frame, |p| state.view.x_to_frame(p.x, lanes_left));
        state.view.zoom_at((scroll.y * 0.003).exp(), anchor);
        state.auto_fit = false;
    } else if modifiers.shift || scroll.x != 0.0 {
        state.view.pan_px(scroll.x + if modifiers.shift { scroll.y } else { 0.0 });
        state.auto_fit = false;
    } else {
        state.scroll_y -= scroll.y;
    }
}

fn scrub(response: &Response, view: &TimelineView, lanes_left: f32, playback: &mut Playback, range: crate::document::FrameRange) {
    if response.is_pointer_button_down_on() {
        if let Some(pos) = response.interact_pointer_pos() {
            playback.pause();
            playback.seek(view.x_to_frame(pos.x, lanes_left).round() as f64, range);
        }
    }
}

fn visible_ticks(view: &TimelineView, lanes_left: f32, width: f32, step: i32) -> impl Iterator<Item = i32> {
    let first = view.x_to_frame(lanes_left, lanes_left).floor() as i32;
    let last = view.x_to_frame(lanes_left + width, lanes_left).ceil() as i32;
    let start = first.div_euclid(step) * step;
    (start..=last).step_by(step as usize)
}

fn character_header(
    ui: &mut Ui,
    painter: &Painter,
    visible: Rect,
    header: Rect,
    doc: &mut Document,
    selection: &mut Selection,
    id: CharacterId,
) {
    let Some(character) = doc.project.character(id) else { return };
    let selected = *selection == Selection::Character(id);
    let title = format!("{}  ·  {}", character.name, character.kind.label().to_lowercase());
    painter.rect_filled(header, 0.0, if selected { theme::ACCENT.gamma_multiply(0.25) } else { theme::CHARACTER_BG });
    painter.text(
        pos2(header.left() + 8.0, header.center().y),
        Align2::LEFT_CENTER,
        title,
        FontId::proportional(11.5),
        theme::ACCENT,
    );
    let response = ui.interact(visible, ui.id().with(("character_header", id)), Sense::click());
    if response.clicked() {
        *selection = Selection::Character(id);
    }
    response.context_menu(|ui| {
        if ui.button("Add Track").clicked() {
            *selection = Selection::Track(add_track(doc, id));
            ui.close_menu();
        }
    });
}

/// A new top track for a character, named after how many it has.
fn add_track(doc: &mut Document, character: CharacterId) -> TrackId {
    let name = format!("Track {}", doc.project.tracks_of(character).count() + 1);
    doc.add_track(character, &name)
}

/// Right-click menu of a track: add a clip of any kind at `frame` (if given), add a
/// track, delete this one.
fn track_menu(ui: &mut Ui, doc: &mut Document, selection: &mut Selection, actions: &mut Vec<Action>, track: TrackId, frame: Option<i32>) {
    let Some(character) = doc.project.track(track).map(|t| t.character) else { return };
    if let Some(frame) = frame {
        ui.weak(format!("Add at frame {frame}"));
        add_clip_items(ui, actions, doc.project.character(character), Some((track, frame)));
        ui.separator();
    }
    if ui.button("Add Track").clicked() {
        *selection = Selection::Track(add_track(doc, character));
        ui.close_menu();
    }
    if ui.button("Delete Track").clicked() {
        *selection = Selection::Track(track);
        actions.push(Action::Delete);
        ui.close_menu();
    }
}

#[allow(clippy::too_many_arguments)]
fn track_header(
    ui: &mut Ui,
    painter: &Painter,
    visible: Rect,
    header: Rect,
    doc: &mut Document,
    selection: &mut Selection,
    actions: &mut Vec<Action>,
    index: usize,
    id: TrackId,
) {
    let Some(track) = doc.project.track(id).cloned() else { return };
    let selected = *selection == Selection::Track(id);
    painter.rect_filled(header, 0.0, if selected { theme::ACCENT.gamma_multiply(0.18) } else { theme::HEADER_BG });
    painter.rect_filled(Rect::from_min_size(header.min, vec2(4.0, header.height())), 0.0, theme::track_color(index));

    let mute_rect = Rect::from_center_size(pos2(header.right() - 18.0, header.center().y), vec2(20.0, 18.0));
    let header_response = ui.interact(visible, ui.id().with(("track_header", id)), Sense::click());
    let mute_response = ui
        .interact(mute_rect.intersect(visible), ui.id().with(("track_mute", id)), Sense::click())
        .on_hover_text(if track.muted { "Unmute track" } else { "Mute track" });

    let name_color = if track.muted { theme::DIM_TEXT } else { theme::TEXT };
    painter.text(pos2(header.left() + 16.0, header.center().y), Align2::LEFT_CENTER, &track.name, FontId::proportional(13.0), name_color);
    let mute_fill = if track.muted {
        theme::PLAYHEAD.gamma_multiply(0.8)
    } else if mute_response.hovered() {
        Color32::from_white_alpha(30)
    } else {
        Color32::from_white_alpha(10)
    };
    painter.rect_filled(mute_rect, 3.0, mute_fill);
    painter.text(mute_rect.center(), Align2::CENTER_CENTER, "M", FontId::proportional(11.0), theme::TEXT);

    if mute_response.clicked() {
        let mut after = track.clone();
        after.muted = !after.muted;
        let label = if after.muted { "Mute track" } else { "Unmute track" };
        doc.apply(label, Edit::Track { id, before: track, after }, false);
    } else if header_response.clicked() {
        *selection = Selection::Track(id);
    }
    header_response.context_menu(|ui| track_menu(ui, doc, selection, actions, id, None));
}

#[allow(clippy::too_many_arguments)]
fn track_lane(
    ui: &mut Ui,
    painter: &Painter,
    lane: Rect,
    lanes: Rect,
    view: &TimelineView,
    lanes_left: f32,
    state: &mut TimelineState,
    doc: &mut Document,
    selection: &mut Selection,
    actions: &mut Vec<Action>,
    track_index: usize,
    track: TrackId,
) {
    let clips: Vec<(ClipId, ClipProps, bool)> = doc
        .project
        .clips_on_track(track)
        .map(|c| (c.id, c.props.clone(), c.kind.is_motion()))
        .collect();
    let to_x = |frame: f32| view.frame_to_x(frame, lanes_left);

    for (id, props, is_motion) in clips {
        let x0 = to_x(props.start as f32);
        let x1 = to_x(props.end() as f32).max(x0 + 3.0);
        let r = Rect::from_x_y_ranges(x0..=x1, (lane.top() + 4.0)..=(lane.bottom() - 4.0));
        let hit = r.intersect(lanes);
        if !hit.is_positive() {
            continue;
        }
        let response = ui.interact(hit, ui.id().with(("clip", id)), Sense::click_and_drag());
        let selected = selection.clip() == Some(id);

        // Body, then the blend envelope: ramps up over blend-in, holds at weight, ramps down.
        // Motion clips are solid with their blend envelope; imported events are drawn as
        // an outlined band since they don't (yet) change the pose.
        let base = theme::track_color(track_index);
        let fill = if props.enabled { base } else { base.gamma_multiply(0.3) };
        if is_motion {
            painter.rect_filled(r, 4.0, fill.gamma_multiply(if selected { 0.95 } else { 0.7 }));
            let (top, bottom) = (r.top() + 3.0, r.bottom() - 2.0);
            let held = bottom - (bottom - top) * props.weight.clamp(0.0, 1.0);
            let envelope = vec![
                pos2(x0, bottom),
                pos2(to_x((props.start + props.blend_in) as f32), held),
                pos2(to_x((props.end() - props.blend_out) as f32), held),
                pos2(x1, bottom),
            ];
            painter.add(Shape::line(envelope, Stroke::new(1.5_f32, Color32::from_white_alpha(120))));
        } else {
            painter.rect_filled(r, 4.0, fill.gamma_multiply(0.22));
            painter.rect_stroke(r, 4.0, Stroke::new(1.0_f32, fill), StrokeKind::Inside);
        }
        painter.with_clip_rect(r.intersect(lanes).shrink(1.0)).text(
            pos2(r.left() + 7.0, r.center().y),
            Align2::LEFT_CENTER,
            &props.name,
            FontId::proportional(12.0),
            if props.enabled { Color32::WHITE } else { theme::DIM_TEXT },
        );
        if selected {
            painter.rect_stroke(r, 4.0, Stroke::new(2.0_f32, theme::SELECTED), StrokeKind::Inside);
        } else if response.hovered() {
            painter.rect_stroke(r, 4.0, Stroke::new(1.0_f32, Color32::from_white_alpha(80)), StrokeKind::Inside);
        }

        if response.hovered() {
            ui.ctx().set_cursor_icon(CursorIcon::Grab);
        }
        if response.clicked() || response.drag_started() {
            if !matches!(*selection, Selection::Clip(c) if c == id) {
                *selection = Selection::Clip(id);
            }
        }
        if response.drag_started() {
            state.drag = Some(Drag::Clip { id, origin: props.start, travel: 0.0 });
        }
        if response.dragged() {
            ui.ctx().set_cursor_icon(CursorIcon::Grabbing);
            if let Some(Drag::Clip { id: dragged, origin, travel }) = &mut state.drag {
                if *dragged == id {
                    *travel += response.drag_delta().x;
                    let start = *origin + (*travel / view.px_per_frame).round() as i32;
                    if start != props.start {
                        let after = ClipProps { start, ..props.clone() };
                        doc.apply("Move clip", Edit::ClipProps { clip: id, before: props.clone(), after }, true);
                    }
                }
            }
        }
        if response.drag_stopped() {
            state.drag = None;
            doc.end_gesture();
        }
        response.context_menu(|ui| {
            if ui.button("Duplicate Clip").clicked() {
                *selection = Selection::Clip(id);
                actions.push(Action::DuplicateClip);
                ui.close_menu();
            }
            if ui.button(if props.enabled { "Disable Clip" } else { "Enable Clip" }).clicked() {
                let after = ClipProps { enabled: !props.enabled, ..props.clone() };
                let label = if after.enabled { "Enable clip" } else { "Disable clip" };
                doc.apply(label, Edit::ClipProps { clip: id, before: props.clone(), after }, false);
                ui.close_menu();
            }
            if ui.button("Delete Clip").clicked() {
                *selection = Selection::Clip(id);
                actions.push(Action::Delete);
                ui.close_menu();
            }
        });
    }
}

#[allow(clippy::too_many_arguments)]
fn channel_header(
    ui: &mut Ui,
    painter: &Painter,
    visible: Rect,
    header: Rect,
    doc: &Document,
    selection: &mut Selection,
    clip: ClipId,
    channel: usize,
) {
    let Some(ch) = doc.project.clip(clip).and_then(|c| c.channels.get(channel)) else { return };
    let selected = selection.channel() == Some((clip, channel));
    painter.rect_filled(header, 0.0, if selected { theme::ACCENT.gamma_multiply(0.18) } else { theme::CHANNEL_BG });
    painter.text(
        pos2(header.left() + 22.0, header.center().y),
        Align2::LEFT_CENTER,
        format!("{} · {}", ch.joint, ch.property.label()),
        FontId::proportional(11.5),
        if selected { theme::TEXT } else { theme::DIM_TEXT },
    );
    if ui.interact(visible, ui.id().with(("channel_header", clip, channel)), Sense::click()).clicked() {
        *selection = Selection::Channel { clip, channel };
    }
}

#[allow(clippy::too_many_arguments)]
fn channel_lane(
    ui: &mut Ui,
    painter: &Painter,
    lane_response: &Response,
    lane: Rect,
    lanes: Rect,
    view: &TimelineView,
    lanes_left: f32,
    state: &mut TimelineState,
    doc: &mut Document,
    selection: &mut Selection,
    clip_id: ClipId,
    channel: usize,
) {
    let Some(clip) = doc.project.clip(clip_id) else { return };
    let Some(ch) = clip.channels.get(channel) else { return };
    let start = clip.props.start;
    let keys: Vec<Keyframe> = ch.keys.clone();
    let to_x = |frame: f32| view.frame_to_x(frame, lanes_left);

    // Faint band showing the clip's extent on this row.
    let band = Rect::from_x_y_ranges(to_x(start as f32)..=to_x(clip.props.end() as f32), (lane.top() + 3.0)..=(lane.bottom() - 3.0));
    painter.rect_filled(band, 2.0, Color32::from_white_alpha(8));

    if lane_response.double_clicked() {
        if let Some(pos) = lane_response.interact_pointer_pos() {
            let local = view.x_to_frame(pos.x, lanes_left).round() as i32 - start;
            if doc.insert_key(clip_id, channel, local) {
                *selection = Selection::Key { clip: clip_id, channel, frame: local };
            }
        }
    } else if lane_response.clicked() {
        *selection = Selection::Channel { clip: clip_id, channel };
    }

    for key in keys {
        let center = pos2(to_x((start + key.frame) as f32), lane.center().y);
        if !lanes.contains(center) {
            continue;
        }
        let hit = Rect::from_center_size(center, vec2(KEY_SIZE * 2.8, lane.height()));
        // A dragged key keeps the widget id it started with, so egui doesn't lose the
        // drag when the key's frame changes underneath it.
        let id_frame = match &state.drag {
            Some(Drag::Key { clip, channel: c, origin, current, .. })
                if *clip == clip_id && *c == channel && *current == key.frame => *origin,
            _ => key.frame,
        };
        let response = ui
            .interact(hit, ui.id().with(("key", clip_id, channel, id_frame)), Sense::click_and_drag())
            .on_hover_text(format!("frame {} · {:.3}", key.frame, key.value));
        let selected = *selection == Selection::Key { clip: clip_id, channel, frame: key.frame };
        let s = if response.hovered() || selected { KEY_SIZE + 1.0 } else { KEY_SIZE };
        let diamond = vec![center + vec2(0.0, -s), center + vec2(s, 0.0), center + vec2(0.0, s), center + vec2(-s, 0.0)];
        let fill = if selected { theme::SELECTED } else { theme::KEY };
        painter.add(Shape::convex_polygon(diamond, fill, Stroke::new(1.0_f32, Color32::from_black_alpha(160))));

        if response.clicked() || response.drag_started() {
            *selection = Selection::Key { clip: clip_id, channel, frame: key.frame };
        }
        if response.drag_started() {
            state.drag = Some(Drag::Key { clip: clip_id, channel, origin: key.frame, current: key.frame, travel: 0.0 });
        }
        if response.dragged() {
            if let Some(Drag::Key { clip, channel: ch_index, origin, current, travel }) = &mut state.drag {
                if *clip == clip_id && *ch_index == channel && *current == key.frame {
                    *travel += response.drag_delta().x;
                    let target = *origin + (*travel / view.px_per_frame).round() as i32;
                    let moved = Keyframe { frame: target, ..key.clone() };
                    if target != *current && doc.replace_key(clip_id, channel, *current, moved, "Move key", true) {
                        *current = target;
                        *selection = Selection::Key { clip: clip_id, channel, frame: target };
                    }
                }
            }
        }
        if response.drag_stopped() {
            state.drag = None;
            doc.end_gesture();
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_ruler(
    ui: &mut Ui,
    painter: &Painter,
    rect: Rect,
    ruler: Rect,
    view: &TimelineView,
    lanes_left: f32,
    lane_width: f32,
    playback: &mut Playback,
    range: crate::document::FrameRange,
) {
    let corner = Rect::from_min_max(rect.min, pos2(lanes_left, ruler.bottom()));
    painter.rect_filled(corner, 0.0, theme::RULER_BG);
    painter.text(pos2(corner.left() + 10.0, corner.center().y), Align2::LEFT_CENTER, "TRACKS", FontId::proportional(10.5), theme::DIM_TEXT);
    painter.rect_filled(ruler, 0.0, theme::RULER_BG);

    let ruler_painter = painter.with_clip_rect(ruler);
    let step = view.tick_step(64.0);
    let minor = if step >= 5 && step % 5 == 0 { step / 5 } else { (step / 2).max(1) };
    if minor as f32 * view.px_per_frame >= 5.0 {
        for frame in visible_ticks(view, lanes_left, lane_width, minor) {
            let x = view.frame_to_x(frame as f32, lanes_left);
            ruler_painter.line_segment([pos2(x, ruler.bottom() - 4.0), pos2(x, ruler.bottom())], Stroke::new(1.0_f32, theme::DIM_TEXT.gamma_multiply(0.5)));
        }
    }
    for frame in visible_ticks(view, lanes_left, lane_width, step) {
        let x = view.frame_to_x(frame as f32, lanes_left);
        ruler_painter.line_segment([pos2(x, ruler.bottom() - 9.0), pos2(x, ruler.bottom())], Stroke::new(1.0_f32, theme::DIM_TEXT));
        ruler_painter.text(pos2(x + 3.0, ruler.top() + 3.0), Align2::LEFT_TOP, frame.to_string(), FontId::monospace(10.5), theme::DIM_TEXT);
    }
    painter.line_segment([ruler.left_bottom(), ruler.right_bottom()], Stroke::new(1.0_f32, Color32::from_black_alpha(140)));

    let response = ui.interact(ruler, ui.id().with("ruler"), Sense::click_and_drag());
    if response.hovered() {
        ui.ctx().set_cursor_icon(CursorIcon::ResizeHorizontal);
    }
    scrub(&response, view, lanes_left, playback, range);
}

fn draw_playhead(painter: &Painter, rect: Rect, ruler: Rect, x: f32, frame: i32) {
    if x < ruler.left() || x > ruler.right() {
        return;
    }
    let painter = painter.with_clip_rect(Rect::from_min_max(pos2(ruler.left(), rect.top()), rect.max));
    painter.line_segment([pos2(x, ruler.top()), pos2(x, rect.bottom())], Stroke::new(1.5_f32, theme::PLAYHEAD));
    let label = frame.to_string();
    let width = 10.0 + 7.0 * label.len() as f32;
    let tag = Rect::from_min_size(pos2(x - width / 2.0, ruler.top() + 2.0), vec2(width, 15.0));
    painter.rect_filled(tag, 3.0, theme::PLAYHEAD);
    painter.text(tag.center(), Align2::CENTER_CENTER, label, FontId::monospace(10.5), Color32::BLACK);
}
