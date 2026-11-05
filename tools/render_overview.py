"""Render docs figures: docs/media/overview.{svg,png} and docs/media/architecture.{svg,png}.

Usage: python3 tools/render_overview.py   (PNG export needs Inkscape)
Latency numbers are read from docs/reference/data/performance-results.json.
Style: light band, white cards, one accent blue, Inter / Inter Display / mono.
"""

import json
import shutil
import subprocess
from pathlib import Path
from xml.sax.saxutils import escape

ROOT = Path(__file__).resolve().parents[1]
BAND, CARD, FILL = "#f2f6f5", "#ffffff", "#e2ebe9"
INK, INK2, ARROW, GREY = "#13201e", "#5f6f6c", "#84948f", "#b6c4c0"
ACCENT = "#0f766e"  # teal
ACCENT_SOFT = "#dcefec"
DISPLAY = "'Inter Display', 'Inter Tight', Inter, 'Helvetica Neue', Arial, sans-serif"
TEXT = "Inter, 'Helvetica Neue', Arial, sans-serif"
MONO = "'JetBrains Mono', 'DejaVu Sans Mono', monospace"
W, H, PAD, GAP = 1200, 440, 24, 24
CW = (W - 2 * PAD - 2 * GAP) / 3
LINK_ROW = 40  # extra card height for the homepage variant's "Learn more" links
EXTRA = 0


def text(x, y, s, size=14, fill=INK2, family=TEXT, weight=400, anchor="start", spacing=0):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-family="{family}" font-size="{size}" '
            f'font-weight="{weight}" fill="{fill}" text-anchor="{anchor}" '
            f'letter-spacing="{spacing}">{escape(s)}</text>')


def rect(x, y, w, h, r, fill):
    return f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{r}" fill="{fill}"/>'


def arrow(d):
    return f'<path d="{d}" fill="none" stroke="{ARROW}" stroke-width="1.5" marker-end="url(#ah)"/>'


def card(i, title, subtitle):
    x = PAD + i * (CW + GAP)
    return x, [
        rect(x, PAD, CW, H - 2 * PAD + EXTRA, 20, CARD),
        text(x + 28, PAD + 52, title, 22, INK, DISPLAY, 600, spacing=-0.3),
        text(x + 28, PAD + 76, subtitle, 14, INK2),
    ]


def modular():
    x, out = card(0, "Modular observations", "Each policy composes registered terms")
    cols = [("MimicLite", 192), ("SONIC", 252), ("yours", 312)]
    # MimicLite v1.1 and SONIC G1 rows match their shipped policy.yaml; "yours" is illustrative.
    used = [
        {"joint_pos", "gravity", "ref_joint_pos", "ref_root_pos", "ref_root_ori"},
        {"joint_pos", "gravity", "ref_joint_pos", "ref_joint_vel", "ref_root_ori"},
        {"joint_pos", "gravity", "ref_root_pos", "ref_body_pos"},
    ]
    terms = ["joint_pos", "gravity", "ref_joint_pos", "ref_joint_vel", "ref_root_pos", "ref_root_ori",
             "ref_body_pos"]
    for name, cx in cols:
        out.append(text(x + cx, 146, name, 12, INK2, TEXT, 500, "middle"))
    for r, term in enumerate(terms):
        y = 172 + r * 32
        out.append(text(x + 28, y + 14, term, 13, INK, MONO))
        for (_, cx), u in zip(cols, used):
            out.append(rect(x + cx - 8, y + 2, 16, 16, 5, ACCENT if term in u else FILL))
    return out


def backends():
    x, out = card(1, "Seamless sim-to-real", "Sim and real share one interface")
    left, right = x + 28, x + CW - 28
    out.append(rect(left, 124, right - left, 46, 14, ACCENT))
    out.append(text(x + CW / 2, 153, "policy loop", 17, "#ffffff", DISPLAY, 600, "middle", -0.2))
    out.append(arrow(f"M{x + CW / 2 - 24:.1f} 175 L{x + CW / 2 - 24:.1f} 206"))
    out.append(arrow(f"M{x + CW / 2 + 24:.1f} 210 L{x + CW / 2 + 24:.1f} 179"))
    out.append(rect(left, 212, right - left, 34, 10, FILL))
    out.append(text(left + 14, 234, "RobotBackend", 12.5, INK, MONO, 500))
    out.append(text(right - 14, 234, "read_state · submit", 12, INK2, TEXT, 400, "end"))
    tiles = [("MuJoCo", "sim", "mujoco"), ("G1", "real", "g1"), ("RP1", "real", "rp1"), ("Yours", "real", "...")]
    gap = 10
    tw = (right - left - gap * (len(tiles) - 1)) / len(tiles)
    for k, (name, kind, flag) in enumerate(tiles):
        tx = left + k * (tw + gap)
        c = tx + tw / 2
        out.append(f'<path d="M{c:.1f} 246 L{c:.1f} 272" stroke="{ARROW}" stroke-width="1.5"/>')
        if name == "Yours":
            out.append(f'<rect x="{tx:.1f}" y="272" width="{tw:.1f}" height="114" rx="14" fill="none" '
                       f'stroke="{GREY}" stroke-width="1.5" stroke-dasharray="5 4"/>')
        else:
            out.append(rect(tx, 272, tw, 114, 14, BAND))
        out.append(text(c, 312, name, 15, INK if name != "Yours" else INK2, DISPLAY, 600, "middle", -0.2))
        out.append(text(c, 334, kind, 12, INK2, TEXT, 400, "middle"))
        out.append(text(c, 366, flag, 11.5, INK, MONO, 400, "middle"))
    return out


def fast(results):
    x, out = card(2, "Fast native C++", "Full policy step, median")
    s = results["summary_ms"]
    py, bolt = s["Python"]["policy_ms"]["p50"], s["C++"]["policy_ms"]["p50"]
    out.append(text(x + 28, 178, f"{py / bolt:.2f}×", 64, INK, DISPLAY, 600, spacing=-1.5))
    out.append(text(x + 28, 206, "faster than the Python stack", 14, INK2))
    span = CW - 56 - 70
    for k, (name, value, color) in enumerate([("Python", py, GREY), ("Bolt Runtime", bolt, ACCENT)]):
        y = 250 + k * 56
        out.append(text(x + 28, y, name, 13, INK, TEXT, 500))
        w = span * value / py
        out.append(rect(x + 28, y + 10, w, 14, 7, color))
        out.append(text(x + 28 + w + 10, y + 22, f"{value:.2f} ms", 13, INK, TEXT, 500))
    out.append(text(x + 28, H - PAD - 24, f"{results['cpu']} · 1 CPU thread", 12, INK2))
    return out


def architecture(width=1000, height=470):
    """Docs architecture figure: fewer, larger labels and stronger contrast than the overview."""
    ink2, line = "#3f4d4a", "#5f6f6c"
    out = [rect(0, 0, width, height, 0, BAND)]

    def box(x, y, w, h, title, sub, fill, title_fill=INK, dashed=False):
        if dashed:
            out.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="14" fill="none" '
                       f'stroke="{line}" stroke-width="1.5" stroke-dasharray="6 4"/>')
        else:
            out.append(rect(x, y, w, h, 14, fill))
        out.append(text(x + w / 2, y + h / 2 - 4, title, 19, title_fill, DISPLAY, 600, "middle", -0.2))
        out.append(text(x + w / 2, y + h / 2 + 21, sub, 16, ink2, TEXT, 400, "middle"))

    def link(d, both=False):
        start = ' marker-start="url(#ahs)"' if both else ""
        out.append(f'<path d="{d}" fill="none" stroke="{line}" stroke-width="2"{start} marker-end="url(#ahd)"/>')

    # Reference input (tracking task) and the PolicyLoop card with its pipeline.
    box(24, 112, 150, 100, "Reference", "file · ZMQ", BAND, dashed=True)
    left, right = 200, width - 24
    out.append(rect(left, 24, right - left, 214, 20, CARD))
    out.append(text(left + 24, 64, "PolicyLoop", 22, INK, DISPLAY, 600, spacing=-0.3))
    stages = [("Observations", "plug-in terms", ACCENT_SOFT), ("ONNX Runtime", "CPU inference", BAND),
              ("Action", "scale · clip", BAND), ("Joint command", "PD targets", BAND)]
    gap = 18
    sw = (right - left - 48 - 3 * gap) / 4
    xs = [left + 24 + k * (sw + gap) for k in range(4)]
    for k, (title, sub, fill) in enumerate(stages):
        box(xs[k], 112, sw, 100, title, sub, fill)
        if k:
            link(f"M{xs[k - 1] + sw + 2:.1f} 162 L{xs[k] - 3:.1f} 162")
    link(f"M176 162 L{xs[0] - 3:.1f} 162")

    # Interface bar with the two calls crossing it.
    out.append(rect(left, 290, right - left, 46, 12, FILL))
    out.append(text(left + 20, 319, "RobotBackend", 16, INK, MONO, 500))
    out.append(text(right - 20, 319, "read_state · submit", 16, ink2, TEXT, 500, "end"))
    obs_c, cmd_c = xs[0] + sw / 2, xs[3] + sw / 2
    link(f"M{obs_c:.1f} 287 L{obs_c:.1f} 218")
    out.append(text(obs_c + 12, 258, "state", 15, INK, MONO))
    link(f"M{cmd_c:.1f} 216 L{cmd_c:.1f} 285")
    out.append(text(cmd_c + 12, 258, "command", 15, INK, MONO))

    # Equal backends.
    tiles = [("MuJoCo", "--backend mujoco"), ("Unitree G1", "--backend g1"),
             ("RoboParty RP1", "--backend rp1"), ("Your robot", "your SDK")]
    tgap = 16
    tw = (right - left - 3 * tgap) / 4
    for k, (title, flag) in enumerate(tiles):
        tx = left + k * (tw + tgap)
        out.append(f'<path d="M{tx + tw / 2:.1f} 336 L{tx + tw / 2:.1f} 362" stroke="{line}" stroke-width="2"/>')
        yours = title == "Your robot"
        if yours:
            out.append(f'<rect x="{tx:.1f}" y="362" width="{tw:.1f}" height="84" rx="14" fill="none" '
                       f'stroke="{line}" stroke-width="1.5" stroke-dasharray="6 4"/>')
        else:
            out.append(rect(tx, 362, tw, 84, 14, CARD))
        out.append(text(tx + tw / 2, 396, title, 19, ink2 if yours else INK, DISPLAY, 600, "middle", -0.2))
        out.append(text(tx + tw / 2, 425, flag, 15, ink2 if yours else ACCENT, MONO, 500, "middle"))
    out.append(f'<defs><marker id="ahd" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" '
               f'orient="auto"><path d="M0 0L10 5L0 10z" fill="{line}"/></marker>'
               f'<marker id="ahs" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" '
               f'orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" fill="{line}"/></marker></defs>')
    return out, width, height


def write(name, body, width, height, png=True):
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
           f'viewBox="0 0 {width} {height}">\n'
           f'<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
           f'markerHeight="7" orient="auto"><path d="M0 0L10 5L0 10z" fill="{ARROW}"/></marker></defs>\n'
           + "\n".join(body) + "\n</svg>\n")
    out = ROOT / f"docs/media/{name}.svg"
    out.write_text(svg)
    if png and shutil.which("inkscape"):
        png = out.with_suffix(".png")
        subprocess.run(["inkscape", str(out), "--export-type=png", f"--export-filename={png}",
                        f"--export-width={width * 2}"], check=True, capture_output=True)
        print(png)
    else:
        print(out if not png else f"{out} (install Inkscape to export PNG)")


def learn_more():
    return [text(PAD + i * (CW + GAP) + 28, H + EXTRA - PAD - 26, "Learn more →", 15, ACCENT, TEXT, 600)
            for i in range(3)]


def main():
    global EXTRA
    results = json.loads((ROOT / "docs/reference/data/performance-results.json").read_text())
    write("overview", [rect(0, 0, W, H, 0, BAND)] + modular() + backends() + fast(results), W, H)
    # Homepage variant: taller cards with a "Learn more" row; the page overlays links on each card.
    EXTRA = LINK_ROW
    body = [rect(0, 0, W, H + EXTRA, 0, BAND)] + modular() + backends() + fast(results) + learn_more()
    write("overview-home", body, W, H + EXTRA, png=False)
    EXTRA = 0
    write("architecture", *architecture())

if __name__ == "__main__":
    main()
