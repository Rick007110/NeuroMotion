//! Horizontal mapping between timeline frames and screen pixels (zoom and pan).

use crate::document::FrameRange;

#[derive(Clone, Debug)]
pub struct TimelineView {
    /// Frame shown at the left edge of the lanes.
    pub first_frame: f32,
    pub px_per_frame: f32,
}

impl Default for TimelineView {
    fn default() -> Self {
        Self {
            first_frame: -5.0,
            px_per_frame: 4.0,
        }
    }
}

impl TimelineView {
    pub const MIN_PX_PER_FRAME: f32 = 0.2;
    pub const MAX_PX_PER_FRAME: f32 = 80.0;

    pub fn frame_to_x(&self, frame: f32, left: f32) -> f32 {
        left + (frame - self.first_frame) * self.px_per_frame
    }

    pub fn x_to_frame(&self, x: f32, left: f32) -> f32 {
        self.first_frame + (x - left) / self.px_per_frame
    }

    /// Zooms and pans so the whole range fits in `width` pixels with a little margin.
    pub fn fit(&mut self, range: FrameRange, width: f32) {
        let len = range.len().max(1) as f32;
        let pad = (len * 0.03).max(2.0);
        self.px_per_frame =
            (width / (len + 2.0 * pad)).clamp(Self::MIN_PX_PER_FRAME, Self::MAX_PX_PER_FRAME);
        self.first_frame = range.start as f32 - pad;
    }

    /// Zooms by `factor`, keeping `anchor_frame` at the same screen position.
    pub fn zoom_at(&mut self, factor: f32, anchor_frame: f32) {
        let new_ppf =
            (self.px_per_frame * factor).clamp(Self::MIN_PX_PER_FRAME, Self::MAX_PX_PER_FRAME);
        self.first_frame =
            anchor_frame - (anchor_frame - self.first_frame) * self.px_per_frame / new_ppf;
        self.px_per_frame = new_ppf;
    }

    pub fn pan_px(&mut self, dx: f32) {
        self.first_frame -= dx / self.px_per_frame;
    }

    /// Pages the view forward or back so `frame` stays visible during playback.
    pub fn follow(&mut self, frame: f32, width: f32) {
        let visible = width / self.px_per_frame;
        if frame < self.first_frame || frame > self.first_frame + visible {
            self.first_frame = frame - visible * 0.1;
        }
    }

    /// Spacing (in frames) for labelled ruler ticks so labels stay at least `min_px` apart.
    pub fn tick_step(&self, min_px: f32) -> i32 {
        const STEPS: [i32; 13] = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000];
        STEPS
            .into_iter()
            .find(|&s| s as f32 * self.px_per_frame >= min_px)
            .unwrap_or(10000)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mapping_round_trips() {
        let v = TimelineView { first_frame: 10.0, px_per_frame: 3.0 };
        let x = v.frame_to_x(42.0, 100.0);
        assert!((v.x_to_frame(x, 100.0) - 42.0).abs() < 1e-4);
    }

    #[test]
    fn fit_shows_whole_range() {
        let mut v = TimelineView::default();
        let range = FrameRange { start: 0, end: 240 };
        v.fit(range, 800.0);
        assert!(v.frame_to_x(0.0, 0.0) >= 0.0);
        assert!(v.frame_to_x(240.0, 0.0) <= 800.0);
    }

    #[test]
    fn zoom_keeps_anchor_in_place() {
        let mut v = TimelineView { first_frame: 0.0, px_per_frame: 4.0 };
        let before = v.frame_to_x(50.0, 0.0);
        v.zoom_at(2.0, 50.0);
        assert!((v.frame_to_x(50.0, 0.0) - before).abs() < 1e-3);
        assert_eq!(v.px_per_frame, 8.0);
    }

    #[test]
    fn tick_step_grows_when_zoomed_out() {
        let near = TimelineView { first_frame: 0.0, px_per_frame: 20.0 };
        let far = TimelineView { first_frame: 0.0, px_per_frame: 0.5 };
        assert!(near.tick_step(60.0) < far.tick_step(60.0));
    }
}
