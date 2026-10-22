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
- Key frames are local to the clip. Rotations are degrees, offsets are metres added to
  the joint's rest offset.
- `skeleton.source` points at an external rig file; `joints` is a cached copy so the
  project still opens without it. Parents always precede children.
- `interp` is the curve *leaving* that key: `step`, `linear` or `smooth`.
