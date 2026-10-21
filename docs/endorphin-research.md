# Research notes: how Endorphin 2.5.2 is organised

These notes come from a static look at a local Endorphin 2.5.2 install
(`C:\Program Files (x86)\NaturalMotion\endorphin 2.5.2`) to learn its *concepts*, so that
NeuroMotion's ideas are familiar to Endorphin users. Nothing here is copied into
NeuroMotion; layout, names, icons, file formats and code are designed separately.

## How the evidence was gathered

| Source | Method | What it showed |
|--------|--------|----------------|
| `bin/endorphin.exe` resources | REA `inspect_pe_resources` (static resource tree), then decoding the menu, dialog and string-table entries | Feature set, editor windows, event types, preferences |
| `Application/*.xml`, `*.ens`, `*.nmw` | These are zlib-compressed XML; decompressed and read the element structure | Scene and keyboard-profile data models |
| `Resources/Scenes/Sample Scenes/*.ens` | Element tree plus counts of event types across all 16 samples | How the timeline is used in practice |
| `Resources/Behaviours/*.nmb` | Element tree only | Behaviour definitions are deliberately obfuscated; not analysed further |

The application was not run. Everything below is what the shipped files contain, not
observed runtime behaviour.

## Findings

**1. The timeline is a list of typed events per character.** A scene holds an `Events`
list (plus separate lists for forces and secondary behaviours). Each event names a
character and has a start frame, end frame, priority, enabled flag and visibility.
The event kinds the Character menu can create are: active pose, behaviour, constraint,
force, sever, animation, motion transfer, simulation and transition. Across the 16
sample scenes the most used are forces (58), behaviours (53), animation (25) and
transitions (6).

*Takeaway for NeuroMotion:* clips on tracks with a start, duration, priority (track
order) and enabled flag are the right core abstraction. Later phases can add clip
*kinds* beyond keyframed motion.

**2. Blending is a property of the event, not a separate object.** Animation events
carry a strength, an entry blend period and a frame offset; transition events carry a
blend window. Priority decides which event wins.

*Takeaway:* NeuroMotion clips carry weight, blend-in and blend-out directly, and the
timeline draws the resulting envelope on the clip.

**3. Animation data is per-joint channels of keys.** Inside an animation event the data
is a set of channels named `character.Joints.<joint>.<property>`, either scalar
channels (frame, value) or quaternion channels (time + xyzw).

*Takeaway:* NeuroMotion's `Channel { joint, property, keys }` matches this shape.
Quaternion rotation channels are a likely later addition; Phase 1 uses Euler degrees.

**4. Characters reference a rig and keep their own copy of it.** A scene embeds each
character's skeleton (joints with transforms and DOF order), body parts, rig nodes and
calibration, and records the source path of the character file.

*Takeaway:* `SkeletonRef` stores both a source path and a cached joint hierarchy so a
project still previews when the rig file is missing.

**5. Three time ranges and two rates.** The engine block stores separate viewable,
playback and save ranges, a 60 fps output buffer and a 120 Hz simulation rate.

*Takeaway:* Phase 1 has one project range and one fps. Separate playback/export ranges
and a simulation rate are noted for later.

**6. Editing tools.** Menus expose undo/redo with an undo-history window (undo/redo
stacks and an adjustable undo limit), pose mirroring left↔right, load/save pose,
create key, multiple camera views, and dockable windows that can be locked.

*Takeaway:* undo is model-level with an entry limit (NeuroMotion: 500). A visible
history list, pose mirroring and saved poses are good Phase 2+ candidates.

**7. Shortcuts are a data table.** `KeyboardProfiles.xml` holds named profiles, each a
list of `command name → key + ctrl/shift/alt` entries, with an in-app editor.

*Takeaway:* same idea, independently implemented: one `BINDINGS` table keyed by
action. A user-editable file can be layered on top later.

See also `endorphin-physics.md` (simulation model) and `endorphin-formats.md` (file
import).

## Not investigated

- Behaviour definitions (protected by the vendor).
- Exact on-screen layout: the window arrangement is stored in the registry/runtime,
  not in the shipped files, and NeuroMotion's layout is designed separately anyway
  (see `layout.md`).
