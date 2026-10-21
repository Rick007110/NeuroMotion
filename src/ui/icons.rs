//! Transport icons, drawn from simple shapes rather than a font or image set.

use eframe::egui::{Painter, Pos2, Rect, Response, Sense, Shape, Stroke, Ui, pos2, vec2};

use super::theme;

#[derive(Clone, Copy, Debug)]
pub enum Icon {
    ToStart,
    StepBack,
    Play,
    Pause,
    Stop,
    StepForward,
    ToEnd,
    Loop,
}

/// A compact icon button. `active` draws it highlighted (e.g. Play while playing).
pub fn button(ui: &mut Ui, icon: Icon, tooltip: &str, active: bool) -> Response {
    let (rect, response) = ui.allocate_exact_size(vec2(28.0, 22.0), Sense::click());
    let visuals = ui.style().interact_selectable(&response, active);
    if response.hovered() || active {
        ui.painter().rect_filled(rect, 4.0, visuals.bg_fill);
    }
    let color = if active { theme::ACCENT } else { visuals.fg_stroke.color };
    paint(ui.painter(), rect.shrink2(vec2(8.0, 6.0)), icon, color);
    response.on_hover_text(tooltip)
}

fn paint(p: &Painter, r: Rect, icon: Icon, color: eframe::egui::Color32) {
    let right = |r: Rect| vec![r.left_top(), pos2(r.right(), r.center().y), r.left_bottom()];
    let left = |r: Rect| vec![r.right_top(), pos2(r.left(), r.center().y), r.right_bottom()];
    let fill = |points: Vec<Pos2>| {
        p.add(Shape::convex_polygon(points, color, Stroke::NONE));
    };
    let bar = |x: f32, w: f32| {
        p.rect_filled(Rect::from_x_y_ranges(x..=x + w, r.y_range()), 0.0, color);
    };
    let split = |r: Rect, t: f32| r.left() + r.width() * t;

    match icon {
        Icon::Play => fill(right(r)),
        Icon::Pause => {
            bar(r.left() + 1.0, 3.5);
            bar(r.right() - 4.5, 3.5);
        }
        Icon::Stop => {
            p.rect_filled(r.shrink(0.5), 1.5, color);
        }
        Icon::StepForward => {
            fill(right(Rect::from_x_y_ranges(r.left()..=split(r, 0.7), r.y_range())));
            bar(r.right() - 2.5, 2.5);
        }
        Icon::StepBack => {
            fill(left(Rect::from_x_y_ranges(split(r, 0.3)..=r.right(), r.y_range())));
            bar(r.left(), 2.5);
        }
        Icon::ToEnd => {
            fill(right(Rect::from_x_y_ranges(r.left()..=split(r, 0.45), r.y_range())));
            fill(right(Rect::from_x_y_ranges(split(r, 0.4)..=split(r, 0.85), r.y_range())));
            bar(r.right() - 2.0, 2.0);
        }
        Icon::ToStart => {
            fill(left(Rect::from_x_y_ranges(split(r, 0.55)..=r.right(), r.y_range())));
            fill(left(Rect::from_x_y_ranges(split(r, 0.15)..=split(r, 0.6), r.y_range())));
            bar(r.left(), 2.0);
        }
        Icon::Loop => {
            let c = r.center();
            let radius = r.height() * 0.5;
            p.circle_stroke(c, radius, Stroke::new(1.6, color));
            let tip = pos2(c.x + radius, c.y);
            fill(vec![tip + vec2(-3.5, -1.0), tip + vec2(3.5, -1.0), tip + vec2(0.0, 3.5)]);
        }
    }
}
