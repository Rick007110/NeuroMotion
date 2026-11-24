"""Writes the roadmap timeline as an HTML page, for a headless browser to screenshot.

Used by the Discord roadmap workflow: reads docs/roadmap-timeline.json (releases, each
"released", "current" or "planned") and writes the page to the given path.

    python3 tools/roadmap_timeline.py docs/roadmap-timeline.json timeline.html
"""

import html
import json
import sys

LOGO = (
    '<svg width="40" height="40" viewBox="96 132 320 248">'
    '<path d="M216 152 L306 292 L396 152 V360" fill="none" stroke="#2BB3A3" stroke-width="38"'
    ' stroke-linecap="round" stroke-linejoin="round"/>'
    '<path d="M116 360 V152 L216 360 V152" fill="none" stroke="#F2F4F7" stroke-width="38"'
    ' stroke-linecap="round" stroke-linejoin="round"/>'
    '<circle cx="216" cy="152" r="27" fill="#F28C4A"/></svg>'
)

STYLE = """
body{margin:0;width:1200px;background:#16181C;font-family:Inter,'DejaVu Sans',sans-serif;color:#E6E8EC}
.wrap{padding:30px 40px 34px}
.head{display:flex;align-items:center;gap:14px;margin-bottom:34px}
.title{font-size:28px;font-weight:700;letter-spacing:-.4px}
.sub{color:#8C929C;font-size:15px;margin-left:auto}
.track{display:flex;position:relative}
.ms{flex:1;text-align:center;position:relative}
.dot{width:26px;height:26px;border-radius:50%;margin:0 auto 14px;background:#2A2E36;
  border:3px solid #3A3F49;position:relative;z-index:2;box-sizing:border-box}
.released .dot{background:#2BB3A3;border-color:#2BB3A3}
.current .dot{background:#16181C;border-color:#F28C4A;box-shadow:0 0 0 7px rgba(242,140,74,.18)}
.v{font-weight:700;font-size:20px}
.released .v{color:#2BB3A3}.current .v{color:#F28C4A}
.n{color:#C3C8D0;font-size:14px;margin-top:4px;font-weight:600}
.tag{display:inline-block;margin-top:8px;height:20px;line-height:20px;font-size:11px;font-weight:700;
  letter-spacing:.6px;padding:0 8px;border-radius:20px;visibility:hidden}
.released .tag{visibility:visible;background:rgba(43,179,163,.15);color:#2BB3A3}
.current .tag{visibility:visible;background:rgba(242,140,74,.15);color:#F28C4A}
.d{color:#868C96;font-size:12.5px;margin-top:8px;line-height:1.35;padding:0 8px}
.line,.fill{position:absolute;top:12px;height:4px;z-index:1;border-radius:2px}
.line{background:#2A2E36}
.fill{background:linear-gradient(90deg,#2BB3A3,#F28C4A)}
"""

TAGS = {"released": "RELEASED", "current": "IN PROGRESS", "planned": "&nbsp;"}


def page(data):
    releases = data["releases"]
    n = len(releases)
    step = 100 / n
    start = step / 2
    # The coloured part of the line runs from the first release to the current one
    # (or the last released one, if none is current).
    reached = max(
        (i for i, r in enumerate(releases) if r["state"] in ("released", "current")),
        default=0,
    )
    stones = "".join(
        f'<div class="ms {html.escape(r["state"])}"><div class="dot"></div>'
        f'<div class="v">{html.escape(r["version"])}</div>'
        f'<div class="n">{html.escape(r["name"])}</div>'
        f'<div class="tag">{TAGS.get(r["state"], "&nbsp;")}</div>'
        f'<div class="d">{html.escape(r["detail"])}</div></div>'
        for r in releases
    )
    return (
        f"<!doctype html><html><head><meta charset='utf-8'><style>{STYLE}</style></head>"
        f"<body><div class='wrap'><div class='head'>{LOGO}"
        f"<div class='title'>{html.escape(data['title'])}</div>"
        f"<div class='sub'>{html.escape(data.get('subtitle', ''))}</div></div>"
        f"<div class='track'><div class='line' style='left:{start}%;right:{start}%'></div>"
        f"<div class='fill' style='left:{start}%;width:{reached * step}%'></div>"
        f"{stones}</div></div></body></html>"
    )


if __name__ == "__main__":
    with open(sys.argv[1], encoding="utf-8") as f:
        data = json.load(f)
    with open(sys.argv[2], "w", encoding="utf-8") as f:
        f.write(page(data))
