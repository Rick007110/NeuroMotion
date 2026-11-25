# Project file format (`.nmproj`)

Pretty-printed JSON, chosen over RON because every language and diff tool reads it.

```json
{
  "format": "neuromotion.project",
  "version": 2,
  "project": {
    "settings": { "name": "Placeholder", "fps": 30.0, "range": { "start": 0, "end": 240 } },
    "characters": [
      {
        "id": 1, "name": "character01", "kind": "character",
        "skeleton": {
          "name": "Placeholder biped",
          "source": null,
          "joints": [
            { "name": "hips", "parent": null, "offset": [0.0, 0.95, 0.0], "rest_rotation": [0.0, 0.0, 0.0, 1.0] },
            "..."
          ]
        }
      }
    ],
    "tracks": [ { "id": 2, "name": "Base", "character": 1, "muted": false } ],
    "clips": [
      {
        "id": 3,
        "props": {
          "name": "Idle breathe", "track": 2, "start": 0, "duration": 240,
          "blend_in": 0, "blend_out": 0, "weight": 1.0, "enabled": true
        },
        "kind": "motion",
        "channels": [
          {
            "joint": "spine", "property": "rot_x",
            "keys": [ { "frame": 0, "value": 2.0, "interp": "smooth" } ]
          }
        ]
      }
    ],
    "physics": {
      "rate": 120.0,
      "gravity": [0.0, -9.81, 0.0],
      "ground": { "height": 0.0, "material": "tarmac" },
      "contact": "coulomb",
      "spring_cap": "bodies",
      "pose_swing_share": 1.0,
      "interactions": {
        "Default": { "restitution": 0.2, "friction": 100.0, "adhesion": 0.0, "softness": 0.0 }
      },
      "pairs": [ ["skin", "tarmac", "Default"] ]
    },
    "next_id": 5
  }
}
```

- `format` must match exactly; `version` newer than the build supports is rejected with
  a clear message, older versions are the hook for future migrations.
- Each track animates one character (`character`); per character, track order is blend
  order (first = bottom layer).
- `kind` is `"motion"` (keyed channels) or `{"event": {"event": "Force"}}` for physics
  events. Clips may carry a `source` map of settings: an event's physics settings, under
  the keys listed in [events.md](events.md), plus anything else an importer kept.
- `rest_rotation` is the joint's rest orientation relative to its parent (`[x, y, z, w]`);
  rotation channels are applied on top of it.
- Version 1 files (single top-level `skeleton`) are migrated automatically on load.
- `physics.contact` is the friction model: `"coulomb"` (μ = the interaction's
  friction / 100, endorphin's meaning, the default for new projects) or
  `"force_capped"` (a cap of that many newtons per contact, what NeuroMotion used
  before). A project keeps the one it was saved with, so it simulates as it did; a
  file with no `physics` settings gets the default.
- `physics.spring_cap` is what the muscle spring cap counts as the mass a muscle
  moves: `"bodies"` (the joint's two bodies, the default, what Balance and Stagger are
  tuned on) or `"limbs"` (the whole limb hanging from the joint, as endorphin's muscles
  behave; imported scenes get it). A file without it gets `"bodies"`.
- `physics.pose_swing_share` is the share of their preset's stiffness and damping
  active poses drive a joint's swing (bend) axes with; the twist axis, all a knee or
  elbow has, always gets the whole preset. `1.0` is the default and what NeuroMotion's
  own projects use; imported scenes get `0.0625`, as endorphin's own runs bend
  (physics-design.md, lesson 46). A file without it gets `1.0`, so a scene imported and
  saved before keeps simulating as it did; the version stays 2.
- A character's physics rig may list `virtual_muscles`: springs between two of its
  bodies (endorphin's chest-to-hand and pelvis-to-heel virtual muscles), each with
  `name`, `muscle` (its preset entry), `parent_body`, `body`, `point` (metres, in the
  body's frame) and whether it drives `position` and `orientation`. A muscle preset
  may list `linear` gains (position drives, per muscle name). Both are left out of
  files that have none and missing in older files, which load with none; the version
  stays 2. They aren't joints: the skeleton and exported motion don't show them.
- Key frames are local to the clip. Rotations are degrees, offsets are metres added to
  the joint's rest offset.
- `skeleton.source` points at an external rig file; `joints` is a cached copy so the
  project still opens without it. Parents always precede children.
- `interp` is the curve *leaving* that key: `step`, `linear` or `smooth`.
- `physics` holds the scene-wide physics settings and is optional: a file without it (older
  builds wrote none) opens with the defaults shown above. Those defaults were force caps
  until Coulomb friction became the default (physics-design.md, lesson 26), so such a
  file now simulates with Coulomb friction; a file that stores its settings keeps them.
  `rate` is physics ticks per second (each frame is split into `round(rate / fps)` steps,
  at least one); `gravity` is in m/s²; `ground` is `null` for a scene without a ground
  plane; `contact`, `spring_cap` and `pose_swing_share` are described above; each interaction has `restitution`, `friction`,
  `adhesion` (N) and `softness`; each pair is `[material, material, interaction]`, in
  either order, and unlisted pairs use the `Default` interaction. The editor's Physics
  group on the project page edits all of these (docs/layout.md). Adding the editing needed
  no format change: the fields were already stored, so the version stays 2.
