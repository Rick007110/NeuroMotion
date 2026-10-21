# NeuroMotion editor layout

The editor window has a menu bar, three working panels and a status line. The goal is
that people who already know 3D and animation tools can find their way around without
reading anything, while the details are designed for NeuroMotion.

```
┌───────────────────────────────────────────────────────────┐
│ File  Edit  View  Help                                     │  menu bar
├──────────────────────────────────────────┬────────────────┤
│                                          │                │
│              viewport                    │   inspector    │
│        (character preview)               │  (selection)   │
│                                          │                │
├──────────────────────────────────────────┴────────────────┤
│ ⏮ ◀ ▶ ■ ▶ ⏭ │ frame 42 / 240 │ ⟳ │            − + Fit     │  transport
│ TRACKS │ 0    20    40    60 ▼   80    100 ...             │  ruler
│ Base   │ [ Idle breathe ............................... ]  │
│ Layer  │          [ Wave /‾‾‾‾‾‾‾‾\ ]                     │
│   r_elbow · rotate Z  ◆   ◆   ◆   ◆                       │  key rows
├───────────────────────────────────────────────────────────┤
│ file · modified │ last action            Paused · frame 42 │  status line
└───────────────────────────────────────────────────────────┘
```

## Why each panel sits where it does

**Viewport in the centre.** The character is what you're actually judging, so it gets
the largest, most central area and everything else is arranged around it. It has no
toolbar of its own; camera controls are on the mouse (drag to orbit, right-drag to pan,
wheel to zoom, double-click to reset) so the picture isn't cluttered. A small text
overlay shows the current frame and which clips are contributing and how strongly, so
you can read the blend without looking away from the motion.

**Timeline along the bottom, full width.** Time reads left to right, so the timeline
benefits more from width than from height. Running it under *both* the viewport and the
inspector gives the longest possible time axis. Putting it below the viewport also
keeps the playhead visually close to the pose it is driving. Inside the panel:

- The transport controls sit at the top-left of the timeline, right next to the thing
  they move. The current frame field, timecode and loop toggle are on the same row;
  zoom and fit are pushed to the right edge because they're used less often.
- A fixed header column names each row. Tracks are colour-coded with a stripe, and the
  same colour is used for that track's clips so you can match them at a glance. Each
  track has a mute toggle at the end of its header.
- Clips show their blend envelope as a line: rising over the blend-in, flat at the clip
  weight, falling over the blend-out. Overlaps between layers are visible without
  opening the inspector.
- Selecting a clip unfolds one row per animated channel directly under its track, with
  keys drawn as diamonds. Keys only appear when you ask for them, so the default view
  stays clean.
- The ruler and empty lane space both scrub. Frames outside the project range are
  shaded so the working range is obvious.

**Inspector on the right.** Your eye and pointer are usually on the viewport or the
timeline, so the inspector sits beside them rather than between them. A vertical strip
also suits a list of labelled fields. It always shows exactly one thing (the project,
a track, a clip, a channel or a key) with a coloured kind label above the name, so it's
always clear what you're editing. Controls are grouped by what they affect:

| Selection | Groups |
|-----------|--------|
| Nothing   | General · Time (fps, range) · Skeleton · Contents |
| Track     | Track (name, mute) |
| Clip      | Clip (name, enabled, track) · Timing (start, duration) · Blending (weight, blend in/out, live influence) · Channels |
| Channel   | Channel (joint, property, live value) · Keys |
| Key       | Key (frame, value, curve) · Belongs to (links back up) |

Every edit in the inspector goes through the document's undo history; dragging a value
records one undo step per drag, not one per frame.

**Menu bar on top.** File, Edit, View and Help are the conventional home for commands
that don't belong to one panel: file handling, undo/redo, deleting, panel visibility.
Each menu item shows its keyboard shortcut, read from the same table the key handler
uses (`src/ui/shortcuts.rs`), so the menus never disagree with the keys.

**Status line at the very bottom.** One quiet line for the file path, an unsaved-changes
marker, the last action's result (e.g. "Undid: Move clip") and the transport state. It
confirms what just happened without dialogs.

## Visual choices

- Dark neutral greys so the character and coloured clips carry the attention.
- One accent colour (teal) for selection and active states, one warm colour (orange)
  reserved for the playhead, and amber for keys, so the three most important markers
  never get confused.
- In the viewport, left-side bones are blue and right-side bones are orange, which makes
  mirrored poses and facing direction easy to read on a stick figure.
- Transport icons are drawn from simple shapes in code rather than taken from an icon set.

## Keyboard

All shortcuts live in one table, `BINDINGS` in `src/ui/shortcuts.rs`; Help → Keyboard
Shortcuts lists them. Defaults: Ctrl+Z / Ctrl+Y undo/redo, Space play/pause, Shift+Space
stop, ←/→ step a frame (Shift for 10), Home/End jump to range ends, K key the selected
channel, Delete remove the selected clip or key, Ctrl+D duplicate a clip, F fit the
timeline. Single-key shortcuts are ignored while a text field has focus.
