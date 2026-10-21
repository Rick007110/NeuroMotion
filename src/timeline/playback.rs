//! Transport: the playhead and whether it is moving.

use crate::document::FrameRange;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransportState {
    /// At rest on the range start.
    Stopped,
    Playing,
    /// Holding wherever the playhead was left.
    Paused,
}

impl TransportState {
    pub fn label(self) -> &'static str {
        match self {
            TransportState::Stopped => "Stopped",
            TransportState::Playing => "Playing",
            TransportState::Paused => "Paused",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Playback {
    state: TransportState,
    /// Fractional so playback stays smooth between whole frames.
    playhead: f64,
    pub looping: bool,
}

impl Playback {
    pub fn new(range: FrameRange) -> Self {
        Self {
            state: TransportState::Stopped,
            playhead: range.start as f64,
            looping: true,
        }
    }

    pub fn state(&self) -> TransportState {
        self.state
    }

    pub fn is_playing(&self) -> bool {
        self.state == TransportState::Playing
    }

    pub fn playhead(&self) -> f64 {
        self.playhead
    }

    /// The whole frame the playhead is on.
    pub fn frame(&self) -> i32 {
        self.playhead.floor() as i32
    }

    pub fn play(&mut self, range: FrameRange) {
        if self.playhead >= range.end as f64 {
            self.playhead = range.start as f64;
        }
        self.state = TransportState::Playing;
    }

    pub fn pause(&mut self) {
        if self.state == TransportState::Playing {
            self.state = TransportState::Paused;
        }
    }

    pub fn toggle(&mut self, range: FrameRange) {
        if self.is_playing() {
            self.pause();
        } else {
            self.play(range);
        }
    }

    /// Halts playback and rewinds to the start of the range.
    pub fn stop(&mut self, range: FrameRange) {
        self.state = TransportState::Stopped;
        self.playhead = range.start as f64;
    }

    /// Moves the playhead without changing whether it is playing.
    pub fn seek(&mut self, frame: f64, range: FrameRange) {
        self.playhead = frame.clamp(range.start as f64, range.end as f64);
        if self.state == TransportState::Stopped && self.playhead != range.start as f64 {
            self.state = TransportState::Paused;
        }
    }

    /// Pauses and moves by whole frames.
    pub fn step(&mut self, frames: i32, range: FrameRange) {
        self.pause();
        self.seek((self.frame() + frames) as f64, range);
        if self.state == TransportState::Stopped {
            self.state = TransportState::Paused;
        }
    }

    /// Advances a playing playhead by real elapsed time.
    pub fn advance(&mut self, dt_seconds: f64, fps: f64, range: FrameRange) {
        if !self.is_playing() {
            return;
        }
        let (start, end) = (range.start as f64, range.end as f64);
        self.playhead += dt_seconds * fps;
        if self.playhead >= end {
            if self.looping && end > start {
                self.playhead = start + (self.playhead - start) % (end - start);
            } else {
                self.playhead = end;
                self.state = TransportState::Paused;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RANGE: FrameRange = FrameRange { start: 0, end: 100 };

    #[test]
    fn plays_and_pauses() {
        let mut p = Playback::new(RANGE);
        assert_eq!(p.state(), TransportState::Stopped);
        p.toggle(RANGE);
        p.advance(1.0, 30.0, RANGE);
        assert_eq!(p.frame(), 30);
        p.toggle(RANGE);
        p.advance(1.0, 30.0, RANGE);
        assert_eq!(p.frame(), 30);
        assert_eq!(p.state(), TransportState::Paused);
    }

    #[test]
    fn stop_rewinds() {
        let mut p = Playback::new(RANGE);
        p.play(RANGE);
        p.advance(1.0, 30.0, RANGE);
        p.stop(RANGE);
        assert_eq!(p.frame(), 0);
        assert!(!p.is_playing());
    }

    #[test]
    fn loops_or_holds_at_end() {
        let mut p = Playback::new(RANGE);
        p.play(RANGE);
        p.advance(4.0, 30.0, RANGE);
        assert_eq!(p.frame(), 20);
        assert!(p.is_playing());

        p.looping = false;
        p.advance(4.0, 30.0, RANGE);
        assert_eq!(p.frame(), 100);
        assert_eq!(p.state(), TransportState::Paused);
        // Playing again from the end restarts.
        p.play(RANGE);
        assert_eq!(p.frame(), 0);
    }

    #[test]
    fn stepping_pauses_and_clamps() {
        let mut p = Playback::new(RANGE);
        p.play(RANGE);
        p.step(1, RANGE);
        assert_eq!(p.frame(), 1);
        assert_eq!(p.state(), TransportState::Paused);
        p.step(-5, RANGE);
        assert_eq!(p.frame(), 0);
        p.seek(1000.0, RANGE);
        assert_eq!(p.frame(), 100);
    }
}
