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
        rect(x, PAD, CW, H - 2 * PAD, 20, CARD),
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


def node(x, y, w, h, title, lines, fill=BAND, dashed=False):
    out = []
    if dashed:
        out.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="14" fill="none" '
                   f'stroke="{GREY}" stroke-width="1.5" stroke-dasharray="5 4"/>')
    else:
        out.append(rect(x, y, w, h, 14, fill))
    out.append(text(x + 18, y + 34, title, 16, INK, DISPLAY, 600, spacing=-0.2))
    for k, line in enumerate(lines):
        out.append(text(x + 18, y + 58 + k * 20, line, 12.5, INK2))
    return out


def architecture(width=1200, height=540):
    out = [rect(0, 0, width, height, 0, BAND)]
    left, right = 240, width - 40
    # Row labels.
    for y, label in [(80, "policy loop"), (326, "interface"), (446, "backends")]:
        out.append(text(40, y, label, 12, INK2, TEXT, 500))
    # Reference motion input (tracking task only).
    out += node(40, 110, 170, 110, "Reference", ["file or ZMQ teleop", "--task tracking"], dashed=True)
    # PolicyLoop card with its pipeline.
    out.append(rect(left, 40, right - left, 214, 20, CARD))
    out.append(text(left + 24, 78, "PolicyLoop", 20, INK, DISPLAY, 600, spacing=-0.3))
    out.append(text(left + 148, 78, "one clock · hold / init / policy modes", 13, INK2))
    stages = [("Observations", ["registered terms", "composed in YAML"]),
              ("ONNX Runtime", ["CPU inference", "typed · recurrent · KV"]),
              ("Action", ["scale · clip", "default pose"]),
              ("Joint command", ["PD targets", "gains · feedforward"])]
    gap, sw = 24, (right - left - 48 - 3 * 24) / 4
    xs = [left + 24 + k * (sw + gap) for k in range(4)]
    for k, (title, lines) in enumerate(stages):
        out += node(xs[k], 110, sw, 120, title, lines, ACCENT_SOFT if k == 0 else BAND)
        if k:
            out.append(arrow(f"M{xs[k - 1] + sw + 3:.1f} 170 L{xs[k] - 3:.1f} 170"))
    out.append(arrow(f"M212 165 L{xs[0] - 3:.1f} 165"))
    # Interface bar and the two calls crossing it.
    out.append(rect(left, 300, right - left, 40, 10, FILL))
    out.append(text(left + 18, 325, "RobotBackend", 13, INK, MONO, 500))
    out.append(text(right - 18, 325, "start · read_state · submit · stop", 12.5, INK2, TEXT, 400, "end"))
    obs_c, cmd_c = xs[0] + sw / 2, xs[3] + sw / 2
    out.append(arrow(f"M{obs_c:.1f} 297 L{obs_c:.1f} 236"))
    out.append(text(obs_c + 10, 276, "read_state", 12, INK, MONO))
    out.append(arrow(f"M{cmd_c:.1f} 234 L{cmd_c:.1f} 295"))
    out.append(text(cmd_c + 10, 276, "submit", 12, INK, MONO))
    # Equal backends.
    tiles = [("MuJoCo", ["sim · physics thread", "native viewer"], "mujoco"),
             ("Unitree G1", ["real · 500 Hz writer", "command watchdog"], "g1"),
             ("RoboParty RP1", ["real · writer", "command watchdog"], "rp1"),
             ("Replay", ["recorded states", "parity · timing"], "replay")]
    tw = (right - left - 3 * 16) / 4
    for k, (title, lines, flag) in enumerate(tiles):
        tx = left + k * (tw + 16)
        out.append(f'<path d="M{tx + tw / 2:.1f} 340 L{tx + tw / 2:.1f} 380" stroke="{ARROW}" stroke-width="1.5"/>')
        out.append(rect(tx, 380, tw, 130, 14, CARD))
        out.append(text(tx + 18, 414, title, 16, INK, DISPLAY, 600, spacing=-0.2))
        for j, line in enumerate(lines):
            out.append(text(tx + 18, 438 + j * 20, line, 12.5, INK2))
        out.append(text(tx + 18, 494, f"--backend {flag}", 12, ACCENT, MONO, 500))
    return out, width, height


def write(name, body, width, height):
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
           f'viewBox="0 0 {width} {height}">\n'
           f'<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
           f'markerHeight="7" orient="auto"><path d="M0 0L10 5L0 10z" fill="{ARROW}"/></marker></defs>\n'
           + "\n".join(body) + "\n</svg>\n")
    out = ROOT / f"docs/media/{name}.svg"
    out.write_text(svg)
    if shutil.which("inkscape"):
        png = out.with_suffix(".png")
        subprocess.run(["inkscape", str(out), "--export-type=png", f"--export-filename={png}",
                        f"--export-width={width * 2}"], check=True, capture_output=True)
        print(png)
    else:
        print(f"{out} (install Inkscape to export PNG)")


def main():
    results = json.loads((ROOT / "docs/reference/data/performance-results.json").read_text())
    write("overview", [rect(0, 0, W, H, 0, BAND)] + modular() + backends() + fast(results), W, H)
    write("architecture", *architecture())


if __name__ == "__main__":
    main()
