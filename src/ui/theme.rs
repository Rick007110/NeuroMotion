//! NeuroMotion's colours and small shared layout helpers.

use eframe::egui::{self, Color32, RichText, Stroke, Ui};

pub const ACCENT: Color32 = Color32::from_rgb(72, 204, 184);
pub const PLAYHEAD: Color32 = Color32::from_rgb(255, 116, 72);
pub const KEY: Color32 = Color32::from_rgb(236, 198, 96);
pub const SELECTED: Color32 = Color32::from_rgb(245, 245, 245);

pub const HEADER_BG: Color32 = Color32::from_rgb(34, 36, 40);
pub const RULER_BG: Color32 = Color32::from_rgb(40, 42, 47);
pub const LANE_BG: Color32 = Color32::from_rgb(26, 27, 30);
pub const LANE_ALT: Color32 = Color32::from_rgb(30, 31, 35);
pub const CHANNEL_BG: Color32 = Color32::from_rgb(22, 23, 26);
pub const CHARACTER_BG: Color32 = Color32::from_rgb(38, 44, 46);
pub const GRID: Color32 = Color32::from_rgba_premultiplied(10, 10, 10, 10);
pub const OUT_OF_RANGE: Color32 = Color32::from_rgba_premultiplied(0, 0, 0, 80);
pub const DIM_TEXT: Color32 = Color32::from_rgb(140, 144, 152);
pub const TEXT: Color32 = Color32::from_rgb(214, 216, 220);

const TRACK_COLORS: [Color32; 5] = [
    Color32::from_rgb(52, 150, 140),
    Color32::from_rgb(122, 102, 196),
    Color32::from_rgb(196, 142, 60),
    Color32::from_rgb(70, 122, 196),
    Color32::from_rgb(186, 88, 112),
];

/// Colour for clips on the track at `index` (cycles through a fixed palette).
pub fn track_color(index: usize) -> Color32 {
    TRACK_COLORS[index % TRACK_COLORS.len()]
}

pub fn apply(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = Color32::from_rgb(32, 34, 38);
    visuals.window_fill = Color32::from_rgb(36, 38, 43);
    visuals.extreme_bg_color = Color32::from_rgb(20, 21, 24);
    visuals.selection.bg_fill = ACCENT.gamma_multiply(0.45);
    visuals.selection.stroke = Stroke::new(1.0_f32, ACCENT);
    visuals.hyperlink_color = ACCENT;
    ctx.set_visuals(visuals);
    ctx.style_mut(|style| {
        style.spacing.item_spacing = egui::vec2(8.0, 5.0);
        style.spacing.button_padding = egui::vec2(8.0, 3.0);
    });
}

/// A collapsible group of related controls, open by default.
pub fn section(ui: &mut Ui, title: &str, add_contents: impl FnOnce(&mut Ui)) {
    egui::CollapsingHeader::new(RichText::new(title).strong())
        .default_open(true)
        .show(ui, add_contents);
}

/// Two-column label/control grid used throughout the inspector.
pub fn grid(ui: &mut Ui, id: &str, add_contents: impl FnOnce(&mut Ui)) {
    egui::Grid::new(id)
        .num_columns(2)
        .spacing([14.0, 6.0])
        .min_col_width(84.0)
        .show(ui, add_contents);
}
