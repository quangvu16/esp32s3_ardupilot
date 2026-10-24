#!/usr/bin/env python3
"""Draw the Windows, iPhone and Mac ground station apps as SVG.

    python3 docs/gen_app_screens.py            # writes the three files below

    android_gcs/desktop/docs/screens.svg   Windows app: Fly, Plan and LiDAR tabs
    apple_gcs/docs/screens-ios.svg         iPhone app: Data, Plan, LiDAR and Setup tabs
    apple_gcs/docs/screens-mac.svg         Mac app: Data, Plan and LiDAR tabs

Drawings, not screenshots. The layout, labels and colours follow the apps' code
(desktop/GcsPanel.kt and Panels.kt; apple_gcs/Esp32RoverGcs/Views); the map, the scan
and the numbers are made up: a car in Auto with RTK Fixed, at waypoint 2 of 4, and
running Avoid on the LiDAR tabs. Same style as android_gcs/docs/gen_screens.py.
"""
import math
import os
import random

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

FG, MUTED, LINE = "#17201C", "#5A6861", "#CFD7D1"
ROUTE, WP_CUR, TRAIL, SCAN = "#FACC15", "#22C55E", "#F97316", "#F97316"
MONO = "DejaVu Sans Mono, Menlo, Consolas, monospace"


class Svg:
    def __init__(self):
        self.out = []
        self.uid = 0

    def e(self, s):
        self.out.append(s)

    def clip(self, x, y, w, h, rx=0):
        self.uid += 1
        cid = f"c{self.uid}"
        self.e(f'<clipPath id="{cid}"><rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}"/></clipPath>'
               f'<g clip-path="url(#{cid})">')

    def end(self):
        self.e("</g>")

    def text(self, x, y, t, size=11, fill=FG, weight=None, anchor="start", family=None, extra=""):
        t = t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        w = f' font-weight="{weight}"' if weight else ""
        f = f' font-family="{family}"' if family else ""
        a = f' text-anchor="{anchor}"' if anchor != "start" else ""
        self.e(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}"{w}{a}{f}{extra}>{t}</text>')

    def rect(self, x, y, w, h, fill, rx=0, stroke=None, sw=1, opacity=None):
        s = f' stroke="{stroke}" stroke-width="{sw}"' if stroke else ""
        o = f' opacity="{opacity}"' if opacity is not None else ""
        self.e(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" fill="{fill}"{s}{o}/>')

    def save(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write("\n".join(self.out) + "\n")
        print("wrote", os.path.relpath(path, ROOT))


def tw(t, size):
    return len(t) * size * 0.56


# ---------------------------------------------------------------------------
# shared pieces: map, HUD, LiDAR scan

MISSION = [(0.22, 0.82, "H"), (0.28, 0.3, 1), (0.66, 0.22, 2), (0.74, 0.66, 3), (0.5, 0.72, 4)]
CAR = (0.47, 0.26, 78)          # fraction x, y, heading on screen (deg)


def osm_map(s, x, y, w, h):
    """OpenStreetMap-style tiles (Windows app: MapPanel, street layer)."""
    s.rect(x, y, w, h, "#F2EFE9")
    s.e(f'<path d="M{x + w * 0.52} {y + h * 0.06} l{w * 0.42} {h * 0.05} l-{w * 0.05} {h * 0.33} l-{w * 0.4} -{h * 0.04} z" fill="#CDEBB0"/>')
    s.e(f'<ellipse cx="{x + w * 0.78}" cy="{y + h * 0.2}" rx="{w * 0.08}" ry="{h * 0.05}" fill="#AAD3DF"/>')
    roads = [(x - 10, y + h * 0.58, x + w + 10, y + h * 0.52, 9), (x + w * 0.3, y - 10, x + w * 0.36, y + h + 10, 7),
             (x + w * 0.8, y + h * 0.5, x + w * 0.85, y + h + 10, 6), (x - 10, y + h * 0.18, x + w * 0.5, y + h * 0.13, 5),
             (x + w * 0.36, y + h * 0.84, x + w + 10, y + h * 0.9, 5)]
    for x1, y1, x2, y2, sw in roads:
        s.e(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#CFC8BE" stroke-width="{sw + 2}" stroke-linecap="round"/>')
    for x1, y1, x2, y2, sw in roads:
        s.e(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#FFF" stroke-width="{sw}" stroke-linecap="round"/>')
    for bx, by, bw, bh in ((0.05, 0.26, 0.16, 0.12), (0.06, 0.64, 0.12, 0.1), (0.42, 0.6, 0.18, 0.1), (0.88, 0.62, 0.1, 0.13)):
        s.rect(x + w * bx, y + h * by, w * bw, h * bh, "#D9D0C9", stroke="#C4B6AB", sw=0.6)


def satellite_map(s, x, y, w, h, seed=3):
    """Apple Maps hybrid (iPhone and Mac apps): a park seen from above."""
    rnd = random.Random(seed)
    s.rect(x, y, w, h, "#3E5A33")
    for _ in range(int(w * h / 900)):
        cx, cy, r = x + rnd.random() * w, y + rnd.random() * h, 6 + rnd.random() * 16
        g = rnd.choice(("#2F4A28", "#4C6B3B", "#557A41", "#36522D", "#5E8447"))
        s.e(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{g}" opacity="0.85"/>')
    paths = [(x - 10, y + h * 0.55, x + w * 0.4, y + h * 0.42, x + w + 10, y + h * 0.5),
             (x + w * 0.62, y - 10, x + w * 0.7, y + h * 0.5, x + w * 0.58, y + h + 10),
             (x - 10, y + h * 0.15, x + w * 0.3, y + h * 0.22, x + w * 0.55, y + h * 0.12)]
    for x1, y1, cx, cy, x2, y2 in paths:
        s.e(f'<path d="M{x1:.1f} {y1:.1f} Q{cx:.1f} {cy:.1f} {x2:.1f} {y2:.1f}" fill="none" stroke="#8C8577" stroke-width="7" stroke-linecap="round" opacity="0.9"/>')
    for bx, by in ((0.12, 0.3), (0.83, 0.12), (0.38, 0.78)):
        s.e(f'<rect x="{x + w * bx:.1f}" y="{y + h * by:.1f}" width="22" height="16" fill="#B5654A" transform="rotate(15 {x + w * bx:.1f} {y + h * by:.1f})"/>')


def mission_overlay(s, x, y, w, h, cur=2, scan=False, trail=True, home_style="osm", car=CAR):
    pts = [(x + w * px, y + h * py) for (px, py, _) in MISSION]
    if trail:
        tr = [(pts[0][0] + 3, pts[0][1] - 3), (pts[1][0] - 3, pts[1][1] + 40), (pts[1][0] + 2, pts[1][1] + 4),
              (x + w * car[0] - 4, y + h * car[1] + 2)]
        s.e('<polyline points="' + " ".join(f"{a:.1f},{b:.1f}" for a, b in tr) +
            f'" fill="none" stroke="{TRAIL if home_style == "apple" else "#EC4899"}" stroke-width="2" opacity="0.9" stroke-linejoin="round"/>')
    s.e('<polyline points="' + " ".join(f"{a:.1f},{b:.1f}" for a, b in pts[1:]) +
        f'" fill="none" stroke="{ROUTE}" stroke-width="3" stroke-linejoin="round"/>')
    for (px, py), (_, _, label) in zip(pts[1:], MISSION[1:]):
        fill = WP_CUR if label == cur else ROUTE
        s.e(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="9" fill="{fill}"/>')
        s.text(px, py + 3.6, str(label), 9, "#000", 700, "middle")
    hx, hy = pts[0]
    if home_style == "apple":
        s.e(f'<circle cx="{hx:.1f}" cy="{hy:.1f}" r="10" fill="#34C759" stroke="#FFF" stroke-width="2"/>')
        s.e(f'<path d="M{hx - 5:.1f} {hy + 4:.1f} v-5 l5 -4 l5 4 v5 z" fill="#FFF"/>')
        s.text(hx, hy + 22, "Home", 9, "#FFF", 600, "middle", extra=' style="paint-order:stroke" stroke="#000" stroke-width="2" stroke-opacity="0.5"')
    else:
        s.rect(hx - 7.5, hy - 7.5, 15, 15, "#EF4444", stroke="#000", sw=1.1)
        s.text(hx, hy + 3.4, "H", 8.5, "#FFF", 700, "middle")
    vx, vy, hdg = x + w * car[0], y + h * car[1], car[2]
    if scan:
        rnd = random.Random(7)
        for k in range(12):
            s.e(f'<circle cx="{vx - 50 + k * 5:.1f}" cy="{vy - 34 + k * 1.3:.1f}" r="1.8" fill="{SCAN}"/>')
        for a in (2.2, 2.7, 3.2, 3.7, 4.2):
            s.e(f'<circle cx="{vx + 32 + 6 * math.cos(a):.1f}" cy="{vy + 24 + 6 * math.sin(a):.1f}" r="1.8" fill="{SCAN}"/>')
    sz = 12
    color = "#E5484D"
    s.e(f'<path transform="rotate({hdg} {vx:.1f} {vy:.1f})" d="M{vx:.1f} {vy - sz:.1f} L{vx + sz * 0.7:.1f} {vy + sz * 0.8:.1f} '
        f'L{vx:.1f} {vy + sz * 0.4:.1f} L{vx - sz * 0.7:.1f} {vy + sz * 0.8:.1f} Z" fill="{color}" stroke="#FFF" stroke-width="1.6" stroke-linejoin="round"/>')


def hud(s, x, y, w, h, style="android", roll=-6):
    """The artificial horizon: Windows draws the Android HudView; SwiftUI draws a plainer one."""
    cx, cy = x + w / 2, y + h / 2
    pitch = 3
    s.clip(x, y, w, h)
    if style == "swiftui":
        off = pitch * h / 60
        s.e(f'<g transform="rotate({-roll} {cx:.1f} {cy:.1f})">')
        s.rect(x - w, y - h, 3 * w, h * 1.5 + off, "#4D85D1")
        s.rect(x - w, cy + off, 3 * w, 2 * h, "#735438")
        s.e(f'<line x1="{x - w}" y1="{cy + off:.1f}" x2="{x + 2 * w}" y2="{cy + off:.1f}" stroke="#FFF" stroke-width="2"/>')
        s.e("</g>")
        s.e(f'<path d="M{cx - 40} {cy} h28 M{cx + 12} {cy} h28" stroke="#FACC15" stroke-width="3"/>')
        sh = ' style="paint-order:stroke" stroke="#000" stroke-width="2" stroke-opacity="0.35"'
        s.text(x + 10, y + 22, "342°", 15, "#FFF", 700, extra=sh)
        s.text(x + w - 10, y + 22, "Auto", 15, "#FFF", 700, "end", extra=sh)
        s.text(x + 10, y + h - 24, "1.6 m/s", 13, "#FFF", 700, extra=sh)
        s.text(x + 10, y + h - 10, "Throttle 34%", 9.5, "#FFF", extra=sh)
        s.text(x + w - 10, y + h - 24, "WP 2 · 12 m", 13, "#FFF", 700, "end", extra=sh)
        s.text(x + w - 10, y + h - 10, "hAcc 0.01 m · HDOP 0.6", 9.5, "#FFF", anchor="end", extra=sh)
        s.end()
        return
    ppd = h / 50
    off = pitch * ppd
    s.e(f'<g transform="rotate({-roll} {cx:.1f} {cy:.1f})">')
    s.rect(x - w, y - h, 3 * w, h * 1.5 + off, "#2E74C4")
    s.rect(x - w, cy + off, 3 * w, 2 * h, "#7A562C")
    s.e(f'<line x1="{x - w}" y1="{cy + off:.1f}" x2="{x + 2 * w}" y2="{cy + off:.1f}" stroke="#FFF" stroke-width="1.2"/>')
    for deg in (-20, -10, 10, 20):
        yy = cy + off - deg * ppd
        half = w * (0.12 if deg % 20 == 0 else 0.07)
        s.e(f'<line x1="{cx - half:.1f}" y1="{yy:.1f}" x2="{cx + half:.1f}" y2="{yy:.1f}" stroke="#FFF" stroke-width="1.1"/>')
        s.text(cx + half + 4, yy + 3.5, str(deg), 9, "#FFF")
    s.e("</g>")
    s.e(f'<polyline points="{cx - 45},{cy} {cx - 14},{cy} {cx - 7},{cy + 7} {cx},{cy} {cx + 7},{cy + 7} {cx + 14},{cy} {cx + 45},{cy}" '
        'fill="none" stroke="#FACC15" stroke-width="3" stroke-linejoin="round"/>')
    tape, hdg = 24, 342
    s.rect(x, y, w, tape, "#000", opacity=0.43)
    ppx = w / 90
    for k in range(hdg - 50, hdg + 51):
        if k % 5:
            continue
        xx = cx + (k - hdg) * ppx
        hd = k % 360
        s.e(f'<line x1="{xx:.1f}" y1="{y + tape}" x2="{xx:.1f}" y2="{y + tape - (7 if hd % 15 == 0 else 3)}" stroke="#FFF" stroke-width="1"/>')
        lbl = {0: "N", 45: "NE", 90: "E", 135: "SE", 180: "S", 225: "SW", 270: "W", 315: "NW"}.get(hd)
        if lbl is None and hd % 15 == 0:
            lbl = str(hd)
        if lbl:
            s.text(xx, y + 12, lbl, 9, "#FFF", anchor="middle")
    s.rect(cx - 24, y + tape, 48, 19, "#000", opacity=0.43)
    s.text(cx, y + tape + 14.5, f"{hdg:03d}°", 13, "#FFF", 700, "middle")
    s.text(x + 8, cy - 4, "1.6 m/s", 15, "#FFF", 700)
    s.text(x + 8, cy + 13, "thr 34%", 12, "#FFF")
    s.text(x + w - 8, cy - 4, "Auto", 15, "#FFF", 700, "end")
    s.text(x + w - 8, cy + 13, "ARMED", 15, "#F87171", 700, "end")
    s.rect(x, y + h - 30, w, 30, "#000", opacity=0.43)
    s.text(x + 8, y + h - 18, "RTK Fixed  24 sats  hdop 0.6  ±0.01 m", 10, "#FFF")
    s.text(x + 8, y + h - 5, "WP 2  12 m  xtrack 0.1 m", 10, "#FFF")
    s.text(x + w - 8, y + h - 18, "12.4 V  1.5 A", 10, "#FFF", anchor="end")
    s.end()


def scan_points():
    """A box ahead-right, a wall on the left, a post behind (deg left-positive, m)."""
    pts = [(a, 1.05 + 0.006 * abs(a + 15)) for a in range(-40, 5, 5)]
    pts += [(a, 2.1 / max(0.35, math.sin(math.radians(a)))) for a in range(40, 140, 5)]
    pts += [(a, 2.6) for a in (170, 175, 180, -175)]
    pts += [(-70, 3.4), (-75, 3.3), (-80, 3.35), (-120, 3.8), (-125, 3.7), (60, 3.9)]
    return pts


def scan_disc(s, cx, cy, r, style, rmax=4.0):
    """The LiDAR view: Windows/Android = light card; SwiftUI = black disc with green returns."""
    ppm = r / rmax

    def xy(deg, m):
        a = math.radians(deg)
        return cx - m * ppm * math.sin(a), cy - m * ppm * math.cos(a)

    if style == "swiftui":
        s.e(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="#111" opacity="0.88"/>')
        ring = "#777"
        dot, close_dot = "#34C759", "#34C759"
    else:
        s.e(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="#FFF" stroke="{LINE}"/>')
        ring = LINE
        dot, close_dot = FG, "#EF4444"
    for k in range(1, int(rmax) + 1):
        s.e(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{k * ppm:.1f}" fill="none" stroke="{ring}" stroke-width="0.8" opacity="0.7"/>')
        if style != "swiftui":
            s.text(cx + 2.5, cy - k * ppm + 9, f"{k} m", 8, MUTED)

    def wedge(a0, a1, rr, hot):
        p0, p1 = xy(a0, rr), xy(a1, rr)
        s.e(f'<path d="M{cx:.1f} {cy:.1f} L{p0[0]:.1f} {p0[1]:.1f} A{rr * ppm:.1f} {rr * ppm:.1f} 0 0 0 {p1[0]:.1f} {p1[1]:.1f} Z" '
            f'fill="{"#EF4444" if hot else ("#3B82F6" if style == "swiftui" else "#22C55E")}" fill-opacity="{0.3 if hot else 0.25}"/>')
    wedge(-20, 20, 1.5, True)
    wedge(20, 45, 1.5, False)
    wedge(-45, -20, 1.5, True)
    for a, m in scan_points():
        if m > rmax:
            continue
        px, py = xy(a, m)
        close = m < 0.6
        s.e(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="{2.6 if style == "swiftui" else 2}" fill="{close_dot if close else dot}"/>')
    if style == "swiftui":
        s.e(f'<path d="M{cx} {cy - 12} L{cx + 8} {cy + 10} L{cx - 8} {cy + 10} Z" fill="#FFF"/>')
    else:
        s.rect(cx - 7, cy - 11, 14, 22, "#1F5FD1", rx=2.5)
        s.e(f'<path d="M{cx} {cy - 16} l-4.5 4.5 h9 z" fill="#1F5FD1"/>')


# ---------------------------------------------------------------------------
# Windows (Swing, Windows look and feel): GcsPanel.kt

WIN_W, WIN_H = 840, 470


def win_button(s, x, y, label, h=24, pressed=False, primary=False):
    w = tw(label, 11) + 18
    s.rect(x, y, w, h, "#CCE4F7" if pressed else ("#E5F1FB" if primary else "#FDFDFD"), rx=3,
           stroke="#0078D4" if (pressed or primary) else "#C5C5C5", sw=1)
    s.text(x + w / 2, y + h / 2 + 4, label, 11, "#000", anchor="middle")
    return w + 5


def win_field(s, x, y, w, value, combo=False):
    s.rect(x, y, w, 24, "#FFF", rx=3, stroke="#C5C5C5")
    s.e(f'<line x1="{x + 1}" y1="{y + 23}" x2="{x + w - 1}" y2="{y + 23}" stroke="#868686"/>')
    s.text(x + 6, y + 16, value, 11, "#000")
    if combo:
        s.e(f'<path d="M{x + w - 16} {y + 10} l4 5 l4 -5" fill="none" stroke="#444" stroke-width="1.3"/>')
    return w + 5


def win_frame(s, x, y, tab):
    """Window, title bar, the connection bar and the tabs. Returns the content rectangle."""
    for k, op in ((10, 0.04), (6, 0.05), (3, 0.08)):
        s.rect(x - k / 2, y + k, WIN_W + k, WIN_H, "#000", rx=8 + k / 2, opacity=op)
    s.rect(x, y, WIN_W, WIN_H, "#F3F3F3", rx=8, stroke="#B9B9B9")
    s.clip(x, y, WIN_W, WIN_H, 8)
    # title bar
    s.rect(x, y, WIN_W, 32, "#EEF4F9")
    s.rect(x + 12, y + 9, 14, 14, "#1C63C4", rx=3)
    s.e(f'<path d="M{x + 19} {y + 11} l4 9 l-4 -2 l-4 2 z" fill="#FFF"/>')
    s.text(x + 34, y + 21, "ESP32 Rover GCS", 12, "#000")
    gx = x + WIN_W - 140
    s.e(f'<line x1="{gx + 18}" y1="{y + 16}" x2="{gx + 28}" y2="{y + 16}" stroke="#000"/>')
    s.rect(gx + 64, y + 11, 10, 10, "none", stroke="#000", sw=1)
    s.e(f'<path d="M{gx + 110} {y + 11} l10 10 M{gx + 120} {y + 11} l-10 10" stroke="#000"/>')
    # connection bar (topBar)
    cy = y + 40
    cx = x + 8
    cx += win_field(s, cx, cy, 58, "TCP", combo=True)
    cx += win_field(s, cx, cy, 110, "192.168.4.1")
    cx += win_field(s, cx, cy, 52, "5760")
    cx += win_button(s, cx, cy, "Connect")
    cx += win_button(s, cx, cy, "Disconnect") + 10
    for lb, col in (("TCP 192.168.4.1:5760", "#2F7D4F"), ("Auto · ARMED", "#C2410C"), ("RTK Fixed · 24 sats", "#2F7D4F"),
                    ("12.4 V 82%", "#333")):
        s.text(cx, cy + 16, lb, 11, col, 600 if col != "#333" else None)
        cx += tw(lb, 11) + 14
    # tabs (JTabbedPane)
    ty = y + 72
    s.e(f'<line x1="{x}" y1="{ty + 24}" x2="{x + WIN_W}" y2="{ty + 24}" stroke="#D0D0D0"/>')
    tx = x + 8
    for name in ("Fly", "Plan", "LiDAR", "Params", "Log"):
        w = tw(name, 11.5) + 22
        on = name == tab
        s.rect(tx, ty + (0 if on else 3), w, 24 if on else 21, "#FFF" if on else "#EAEAEA", rx=2,
               stroke="#D0D0D0")
        if on:
            s.rect(tx + 1, ty + 22, w - 2, 3, "#FFF")
        s.text(tx + w / 2, ty + 16, name, 11.5, "#000", 600 if on else None, "middle")
        tx += w + 1
    top = ty + 26
    s.rect(x, top, WIN_W, WIN_H - (top - y), "#FFF")
    return x, top, WIN_W, y + WIN_H - top


def win_fly(s, x, y):
    cx, cy, cw, ch = win_frame(s, x, y, "Fly")
    bx = cx + 6
    for lb in ("Disarm", "Manual", "Hold", "Auto", "RTL", "Guided", "Acro", "Steering", "Loiter", "SmartRTL"):
        bx += win_button(s, bx, cy + 6, lb, pressed=(lb == "Auto"))
    top = cy + 36
    side = 300
    hud(s, cx, top, side, 190)
    yy = top + 198
    s.rect(cx + 8, yy + 2, 13, 13, "#FFF", rx=2, stroke="#333")
    s.text(cx + 27, yy + 13, "Drive with arrow keys", 11, "#000")
    yy += 24
    s.text(cx + 8, yy + 13, "Power", 11, "#000")
    s.e(f'<line x1="{cx + 52}" y1="{yy + 9}" x2="{cx + 280}" y2="{yy + 9}" stroke="#B5B5B5" stroke-width="4" stroke-linecap="round"/>')
    s.e(f'<line x1="{cx + 52}" y1="{yy + 9}" x2="{cx + 120}" y2="{yy + 9}" stroke="#0078D4" stroke-width="4" stroke-linecap="round"/>')
    s.e(f'<circle cx="{cx + 120}" cy="{yy + 9}" r="7" fill="#0078D4" stroke="#FFF" stroke-width="2"/>')
    yy += 26
    bx = cx + 8
    for lb in ("Follow", "Fit", "Map / Satellite", "Clear trail"):
        bx += win_button(s, bx, yy, lb)
    yy += 34
    s.text(cx + 8, yy + 4, "Click the map in Guided to drive there.", 10, MUTED)
    s.text(cx + 8, yy + 18, "Right-click: drive here, set home.", 10, MUTED)
    mx = cx + side + 4
    s.e(f'<line x1="{mx - 2}" y1="{top}" x2="{mx - 2}" y2="{cy + ch}" stroke="#D0D0D0" stroke-width="4"/>')
    s.clip(mx, top, cw - side - 4, cy + ch - top)
    osm_map(s, mx, top, cw - side - 4, cy + ch - top)
    mission_overlay(s, mx, top, cw - side - 4, cy + ch - top, scan=True)
    s.rect(mx + cw - side - 4 - 180, cy + ch - 14, 180, 14, "#FFF", opacity=0.7)
    s.text(mx + cw - side - 8, cy + ch - 3, "© OpenStreetMap contributors", 8.5, "#222", anchor="end")
    s.end()
    s.end()


def win_plan(s, x, y):
    cx, cy, cw, ch = win_frame(s, x, y, "Plan")
    bx = cx + 6
    bx += win_button(s, bx, cy + 6, "Fence")
    for lb in ("Read", "Write", "Speed…", "Zig zag…", "Patrol…", "Clear", "Start Auto", "Load…", "Save…", "Fit"):
        bx += win_button(s, bx, cy + 6, lb, primary=(lb == "Write"))
    s.text(cx + 10, cy + 47, "Mission uploaded: 4 items · 128 m", 10.5, MUTED)
    top = cy + 54
    lw = 200
    mw = cw - lw - 4
    s.clip(cx, top, mw, cy + ch - top)
    osm_map(s, cx, top, mw, cy + ch - top)
    mission_overlay(s, cx, top, mw, cy + ch - top)
    s.end()
    s.e(f'<line x1="{cx + mw + 2}" y1="{top}" x2="{cx + mw + 2}" y2="{cy + ch}" stroke="#D0D0D0" stroke-width="4"/>')
    lx = cx + mw + 4
    rows = ["1. Waypoint  31 m", "2. Waypoint, wait 2 s  38 m", "3. Waypoint  36 m", "4. Waypoint  23 m"]
    for i, r in enumerate(rows):
        if i == 1:
            s.rect(lx, top + 4 + i * 20, lw, 20, "#CCE4F7")
        s.text(lx + 6, top + 18 + i * 20, r, 11, "#000")
    s.text(lx + 6, top + 18 + 5 * 20, "Double-click to edit.", 10, MUTED)
    s.text(lx + 6, top + 32 + 5 * 20, "Right-click the map:", 10, MUTED)
    s.text(lx + 6, top + 46 + 5 * 20, "zig-zag points A and B.", 10, MUTED)
    s.end()


def win_lidar(s, x, y):
    cx, cy, cw, ch = win_frame(s, x, y, "LiDAR")
    scan_disc(s, cx + 190, cy + ch / 2, ch / 2 - 18, "android")
    s.text(cx + 12, cy + ch - 10, "2316 scans · nearest 1.05 m at -15° · click to zoom", 10, MUTED)
    rx = cx + 390
    bx = rx
    for lb in ("Off", "Avoid", "Follow", "Warn"):
        bx += win_button(s, bx, cy + 12, lb, pressed=(lb == "Avoid"))
    win_button(s, bx + 6, cy + 12, "Stop", primary=True)
    s.text(rx, cy + 60, "Front+right blocked: turn left", 15, "#000", 700)
    s.text(rx, cy + 80, "thr +0.15  steer -0.80", 12, "#000")
    for i, line in enumerate(("laser_Avoidance: drives forward and steers away",
                              "from obstacles in the front, left and right sectors.",
                              "Holding an arrow key pauses it.")):
        s.text(rx, cy + 104 + i * 15, line, 10.5, MUTED)
    yy = cy + 160
    for lab, val in (("Vehicle", "RC car"), ("Avoid: speed (stick 0-1)", "0.15"), ("Avoid: steering (stick 0-1)", "0.8"),
                     ("Avoid: response dist (m)", "1"), ("Avoid: side sector to (°)", "45"), ("Follow: keep distance (m)", "1"),
                     ("Warn: distance (m)", "0.5")):
        s.text(rx, yy + 16, lab, 11, "#000")
        win_field(s, rx + 170, yy, 120, val)
        yy += 30
    s.end()


def windows_svg():
    s = Svg()
    gap, margin, title_h = 40, 40, 96
    n = 3
    tw_ = margin * 2 + n * WIN_W + (n - 1) * gap
    th = title_h + WIN_H + 70
    s.e(f'<svg xmlns="http://www.w3.org/2000/svg" width="{tw_}" height="{th}" viewBox="0 0 {tw_} {th}" '
        'font-family="Segoe UI, Roboto, Helvetica, Arial, sans-serif" font-size="12">')
    s.rect(0, 0, tw_, th, "#FFFFFF")
    s.text(margin, 46, "ESP32 Rover GCS — Windows app", 24, "#1f2328", 600)
    s.text(margin, 72, "Esp32RoverGcs.exe (Java inside) · MAVLink over the board's WiFi (TCP 192.168.4.1:5760) · drawn from "
           "the app's layout, with a car in Auto and RTK Fixed, and Avoid running on the LiDAR tab", 14, "#59636e")
    caps = [("Fly", "HUD, modes, arrow-key driving, map with the mission, trail and scan", win_fly),
            ("Plan", "Click to add waypoints, Write, Start Auto; zig-zag, patrol, fences", win_plan),
            ("LiDAR", "The scan and the Avoid / Follow / Warn behaviours with their settings", win_lidar)]
    for i, (name, cap, fn) in enumerate(caps):
        x = margin + i * (WIN_W + gap)
        fn(s, x, title_h)
        s.text(x + WIN_W / 2, title_h + WIN_H + 34, name, 17, "#1f2328", 600, "middle")
        s.text(x + WIN_W / 2, title_h + WIN_H + 54, cap, 12.5, "#59636e", anchor="middle")
    s.e("</svg>")
    s.save(os.path.join(ROOT, "android_gcs/desktop/docs/screens.svg"))


# ---------------------------------------------------------------------------
# Apple (SwiftUI): apple_gcs/Esp32RoverGcs/Views

A_BG, A_GROUPED, A_CARD = "#FFFFFF", "#F2F2F7", "#FFFFFF"
A_BLUE, A_GREEN, A_RED, A_ORANGE, A_GRAY = "#007AFF", "#34C759", "#FF3B30", "#FF9500", "#8E8E93"


def apill(s, x, y, label, color, size=9):
    w = tw(label, size) * 1.1 + 14
    s.rect(x, y, w, 18, color, rx=9, opacity=0.18)
    s.rect(x, y, w, 18, "none", rx=9, stroke=color, sw=0.8)
    s.text(x + 7, y + 12.5, label, size, color, 600)
    return w + 5


def status_pills(s, x, y, mode="Auto"):
    for lb, col in (("TCP 192.168.4.1:5760", A_GREEN), (mode, A_BLUE), ("ARMED", A_RED), ("RTK Fixed · 24 sats", A_GREEN),
                    ("12.4 V 82%", A_GRAY)):
        x += apill(s, x, y, lb, col)


def abutton(s, x, y, label, style="bordered", tint=None, size=10.5, h=24):
    w = tw(label, size) + 18
    if style == "prominent":
        s.rect(x, y, w, h, tint or A_BLUE, rx=7)
        s.text(x + w / 2, y + h / 2 + 4, label, size, "#FFF", 600, "middle")
    else:
        col = tint or A_GRAY
        s.rect(x, y, w, h, col, rx=7, opacity=0.15)
        s.text(x + w / 2, y + h / 2 + 4, label, size, col if tint else "#3A3A3C", None, "middle")
    return w + 5


def mode_row(s, x, y, maxx, cur="Auto"):
    x += abutton(s, x, y, "Disarm", "prominent", A_RED)
    for m in ("Manual", "Acro", "Hold", "Auto", "Guided", "RTL", "SmartRTL", "Steering", "Loiter"):
        if x + tw(m, 10.5) + 18 > maxx:
            break
        x += abutton(s, x, y, m, tint=A_BLUE if m == cur else None)


def map_buttons(s, x, y):
    s.rect(x, y, 34, 70, "#F5F5F7", rx=8, opacity=0.92)
    s.e(f'<path d="M{x + 11} {y + 24} l14 -7 l-6 14 l-1 -6 z" fill="{A_BLUE}"/>')
    s.rect(x + 10, y + 44, 14, 12, "none", rx=1, stroke=A_BLUE, sw=1.5)
    s.e(f'<line x1="{x + 15}" y1="{y + 44}" x2="{x + 15}" y2="{y + 56}" stroke="{A_BLUE}" stroke-width="1.2"/>')


def tap_menu(s, x, y, label="Tap: Drive here"):
    w = tw(label, 10) + 30
    s.rect(x, y, w, 24, "#F5F5F7", rx=7, opacity=0.92)
    s.text(x + 9, y + 16, label, 10, "#000")
    s.text(x + w - 11, y + 16, "⌃", 9, "#555", anchor="middle")


PH_W, PH_H = 300, 640
P_SX, P_SY = 10, 10
P_W, P_H = PH_W - 2 * P_SX, PH_H - 2 * P_SY


def phone_chrome(s, tab):
    """iOS status bar, the app's pill bar, and the tab bar (Data, Plan, LiDAR, Setup, More)."""
    s.rect(0, 0, P_W, P_H, A_BG)
    s.text(26, 22, "9:41", 12, "#000", 600)
    s.rect(P_W / 2 - 42, 9, 84, 22, "#000", rx=11)                   # Dynamic Island
    for i in range(4):
        s.rect(P_W - 62 + i * 4.5, 19 - i * 2, 3, 4 + i * 2, "#000", rx=0.6)
    s.rect(P_W - 36, 11, 20, 10, "none", rx=2.5, stroke="#000")
    s.rect(P_W - 34.5, 12.5, 14, 7, "#000", rx=1.5)
    top = 40
    s.clip(0, top, P_W, 22)
    status_pills(s, 6, top + 2)
    s.end()
    top += 24
    s.e(f'<line x1="0" y1="{top}" x2="{P_W}" y2="{top}" stroke="#E5E5EA"/>')
    bot = P_H - 56
    s.rect(0, bot, P_W, 56, "#F9F9F9")
    s.e(f'<line x1="0" y1="{bot}" x2="{P_W}" y2="{bot}" stroke="#D1D1D6"/>')
    names = ["Data", "Plan", "LiDAR", "Setup", "More"]
    tw_ = P_W / len(names)
    for i, n in enumerate(names):
        col = A_BLUE if n == tab else A_GRAY
        ix, iy = i * tw_ + tw_ / 2, bot + 17
        if n == "Data":
            s.e(f'<path d="M{ix - 8} {iy + 4} a8 8 0 1 1 16 0" fill="none" stroke="{col}" stroke-width="2"/><line x1="{ix}" y1="{iy + 4}" x2="{ix + 4}" y2="{iy - 2}" stroke="{col}" stroke-width="2"/>')
        elif n == "Plan":
            s.e(f'<path d="M{ix - 9} {iy - 5} l6 -2 l6 2 l6 -2 v11 l-6 2 l-6 -2 l-6 2 z" fill="none" stroke="{col}" stroke-width="1.6"/>')
        elif n == "LiDAR":
            s.e(f'<circle cx="{ix}" cy="{iy}" r="2.5" fill="{col}"/><path d="M{ix - 6} {iy - 6} a8.5 8.5 0 0 0 0 12 M{ix + 6} {iy - 6} a8.5 8.5 0 0 1 0 12" fill="none" stroke="{col}" stroke-width="1.7"/>')
        elif n == "Setup":
            s.e(f'<path d="M{ix - 7} {iy + 7} l10 -10 M{ix + 1} {iy - 7} a4 4 0 1 0 6 6" fill="none" stroke="{col}" stroke-width="2"/>')
        else:
            for k in (-7, 0, 7):
                s.e(f'<circle cx="{ix + k}" cy="{iy}" r="2" fill="{col}"/>')
        s.text(ix, bot + 38, n, 9.5, col, 500, "middle")
    s.rect(P_W / 2 - 60, P_H - 9, 120, 4, "#000", rx=2)
    return top, bot


def ios_data(s):
    top, bot = phone_chrome(s, "Data")
    hud(s, 0, top, P_W, 150, "swiftui", roll=-4)
    y = top + 150
    s.clip(0, y, P_W, 36)
    mode_row(s, 8, y + 6, P_W + 200)
    s.end()
    y += 36
    s.clip(0, y, P_W, bot - y)
    satellite_map(s, 0, y, P_W, bot - y)
    mission_overlay(s, 0, y, P_W, bot - y, home_style="apple")
    tap_menu(s, 8, y + 8)
    map_buttons(s, P_W - 42, y + 8)
    s.end()


def ios_plan(s):
    top, bot = phone_chrome(s, "Plan")
    y = top + 6
    s.rect(10, y, P_W - 20, 26, "#E9E9EB", rx=7)
    s.rect(12, y + 2, (P_W - 24) / 2, 22, "#FFF", rx=6)
    s.text(10 + (P_W - 20) * 0.25, y + 17, "Waypoints", 11, "#000", 600, "middle")
    s.text(10 + (P_W - 20) * 0.75, y + 17, "Fence", 11, "#000", None, "middle")
    y += 32
    mh = 250
    s.clip(0, y, P_W, mh)
    satellite_map(s, 0, y, P_W, mh, seed=5)
    mission_overlay(s, 0, y, P_W, mh, home_style="apple")
    map_buttons(s, P_W - 42, y + 8)
    s.end()
    y += mh + 6
    s.clip(0, y, P_W, 30)
    x = 8
    x += abutton(s, x, y, "Read")
    x += abutton(s, x, y, "Write", "prominent")
    x += abutton(s, x, y, "Start Auto", tint=A_GREEN)
    x += abutton(s, x, y, "Zig zag…")
    abutton(s, x, y, "Patrol…")
    s.end()
    y += 32
    s.text(10, y + 13, "Mission speed", 10.5, "#000")
    s.e(f'<line x1="98" y1="{y + 9}" x2="{P_W - 82}" y2="{y + 9}" stroke="#D1D1D6" stroke-width="4" stroke-linecap="round"/>')
    s.e(f'<line x1="98" y1="{y + 9}" x2="140" y2="{y + 9}" stroke="{A_BLUE}" stroke-width="4" stroke-linecap="round"/>')
    s.e(f'<circle cx="140" cy="{y + 9}" r="9" fill="#FFF" stroke="#D1D1D6"/>')
    s.text(P_W - 10, y + 13, "1.50 m/s", 10.5, "#000", anchor="end")
    y += 24
    for i, (lb, pos) in enumerate((("Waypoint", "10.774406, 106.692243"), ("Waypoint, wait 2 s", "10.774187, 106.692378"),
                                   ("Waypoint", "10.774096, 106.692108"), ("Waypoint", "10.774015, 106.691964"))):
        ry = y + i * 30
        if ry + 30 > bot:
            break
        s.text(16, ry + 19, str(i + 1), 11.5, "#000", 700)
        s.text(38, ry + 19, lb, 11.5, "#000")
        s.text(P_W - 10, ry + 19, pos, 8.5, A_GRAY, anchor="end")
        s.e(f'<line x1="38" y1="{ry + 30}" x2="{P_W}" y2="{ry + 30}" stroke="#E5E5EA"/>')


def ios_lidar(s):
    top, bot = phone_chrome(s, "LiDAR")
    y = top + 10
    r = (P_W - 40) / 2
    scan_disc(s, P_W / 2, y + r, r, "swiftui")
    y += 2 * r + 8
    s.text(P_W / 2, y + 8, "2316 scans · range 4 m · nearest 1.05 m at -15°", 9, A_GRAY, anchor="middle")
    y += 18
    s.rect(10, y, P_W - 20, 26, "#E9E9EB", rx=7)
    segw = (P_W - 24) / 4
    for i, n in enumerate(("Off", "Avoid", "Follow", "Warn")):
        if n == "Avoid":
            s.rect(12 + i * segw, y + 2, segw, 22, "#FFF", rx=6)
        s.text(12 + i * segw + segw / 2, y + 17, n, 11, "#000", 600 if n == "Avoid" else None, "middle")
    y += 34
    for line in ("laser_Avoidance: drives forward and steers away from",
                 "obstacles in the front, left and right sectors. Drives with",
                 "stick overrides (arm, then Acro, Manual or Steering)."):
        s.text(10, y + 9, line, 9.5, A_GRAY)
        y += 13
    y += 6
    x = 10 + abutton(s, 10, y, "Stop", "prominent", A_RED, h=28)
    s.rect(x + 8, y + 4, 38, 22, A_GREEN, rx=11)
    s.e(f'<circle cx="{x + 35}" cy="{y + 15}" r="9" fill="#FFF"/>')
    s.text(x + 52, y + 19, "Sound", 11, "#000")
    y += 38
    s.text(10, y + 10, "Front+right blocked: turn left   thr +0.15  steer -0.80", 10, "#000", 600)
    y += 22
    for lab, val, frac in (("Speed (stick)", "0.15", 0.18), ("Turn (stick)", "0.80", 0.75), ("Response dist (m)", "1.00", 0.26)):
        if y + 34 > bot:
            break
        s.text(10, y + 10, lab, 10.5, "#000")
        s.text(P_W - 10, y + 10, val, 10.5, A_GRAY, anchor="end")
        s.e(f'<line x1="10" y1="{y + 22}" x2="{P_W - 10}" y2="{y + 22}" stroke="#D1D1D6" stroke-width="4" stroke-linecap="round"/>')
        s.e(f'<line x1="10" y1="{y + 22}" x2="{10 + (P_W - 20) * frac:.1f}" y2="{y + 22}" stroke="{A_BLUE}" stroke-width="4" stroke-linecap="round"/>')
        s.e(f'<circle cx="{10 + (P_W - 20) * frac:.1f}" cy="{y + 22}" r="8" fill="#FFF" stroke="#D1D1D6"/>')
        y += 34


def ios_setup(s):
    top, bot = phone_chrome(s, "Setup")
    s.rect(0, top, P_W, bot - top, A_GROUPED)
    y = top + 8

    def header(t):
        nonlocal y
        s.text(26, y + 14, t, 10.5, A_GRAY)
        y += 20

    def card(rows_h):
        nonlocal y
        s.rect(12, y, P_W - 24, rows_h, A_CARD, rx=10)

    header("Connection")
    card(176)
    s.text(24, y + 22, "Protocol", 11.5, "#000")
    s.rect(P_W - 118, y + 9, 92, 24, "#E9E9EB", rx=7)
    s.rect(P_W - 116, y + 11, 44, 20, A_BLUE, rx=6)
    s.text(P_W - 94, y + 25, "TCP", 10.5, "#FFF", 600, "middle")
    s.text(P_W - 48, y + 25, "UDP", 10.5, "#000", None, "middle")
    for i, (lab, val) in enumerate((("Address", "192.168.4.1"), ("Port", "5760"))):
        ry = y + 40 + i * 34
        s.e(f'<line x1="24" y1="{ry}" x2="{P_W - 12}" y2="{ry}" stroke="#E5E5EA"/>')
        s.text(24, ry + 22, lab, 11.5, "#000")
        s.text(P_W - 26, ry + 22, val, 11.5, A_GRAY, anchor="end")
    ry = y + 112
    s.e(f'<line x1="24" y1="{ry}" x2="{P_W - 12}" y2="{ry}" stroke="#E5E5EA"/>')
    x = 24 + abutton(s, 24, ry + 9, "Connect", "prominent", h=26)
    abutton(s, x, ry + 9, "Disconnect", h=26)
    s.text(24, ry + 54, "TCP 192.168.4.1:5760 · 15 230 packets", 9.5, A_GRAY)
    y += 186
    header("Accelerometer")
    card(70)
    s.text(24, y + 22, "Mount the board in the car first. Then 6", 11, "#000")
    s.text(24, y + 36, "positions, or level only.", 11, "#000")
    x = 24 + abutton(s, 24, y + 44, "6 positions", size=10)
    x += abutton(s, x, y + 44, "Level only", size=10)
    y += 80
    header("RTK / NTRIP")
    card(150)
    for i, (lab, val) in enumerate((("Caster", "rtk2go.com"), ("Port", "2101"), ("Mountpoint", "HCM_BASE"))):
        ry = y + i * 34
        if i:
            s.e(f'<line x1="24" y1="{ry}" x2="{P_W - 12}" y2="{ry}" stroke="#E5E5EA"/>')
        s.text(24, ry + 22, lab, 11.5, "#000")
        s.text(P_W - 26, ry + 22, val, 11.5, A_GRAY, anchor="end")
    ry = y + 102
    s.e(f'<line x1="24" y1="{ry}" x2="{P_W - 12}" y2="{ry}" stroke="#E5E5EA"/>')
    s.text(24, ry + 20, "Over mobile data", 11.5, "#000")
    s.rect(P_W - 66, ry + 6, 40, 24, A_GREEN, rx=12)
    s.e(f'<circle cx="{P_W - 38}" cy="{ry + 18}" r="10" fill="#FFF"/>')
    s.text(24, ry + 40, "Connected: HCM_BASE · 1005×12 1074×340 1084×340", 8.5, A_GRAY)


def phone(s, x, y, fn):
    s.e(f'<g transform="translate({x} {y})">')
    for k, op in ((9, 0.04), (6, 0.05), (3, 0.07)):
        s.rect(-k / 2, k, PH_W + k, PH_H, "#000", rx=44 + k / 2, opacity=op)
    s.rect(0, 0, PH_W, PH_H, "#1B1F23", rx=44)
    s.e(f'<g transform="translate({P_SX} {P_SY})">')
    s.clip(0, 0, P_W, P_H, 35)
    fn(s)
    s.end()
    s.e("</g></g>")


def ios_svg():
    s = Svg()
    screens = [("Data", "HUD, Arm, modes, map, Drive stick", ios_data),
               ("Plan", "Waypoints, zig-zag, patrol, fences", ios_plan),
               ("LiDAR", "The LD14 scan and Avoid / Follow / Warn", ios_lidar),
               ("Setup", "Connection, calibration, NTRIP", ios_setup)]
    gap, margin, title_h = 44, 40, 96
    w = margin * 2 + len(screens) * PH_W + (len(screens) - 1) * gap
    h = title_h + PH_H + 64
    s.e(f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
        'font-family="-apple-system, SF Pro Text, Helvetica Neue, Helvetica, Arial, sans-serif" font-size="12">')
    s.rect(0, 0, w, h, "#FFFFFF")
    s.text(margin, 46, "ESP32 Rover GCS — iPhone app", 24, "#1f2328", 600)
    s.text(margin, 72, "SwiftUI, iOS 17+ · the same app runs on iPad and Mac · drawn from the app's layout, with a car in Auto "
           "and RTK Fixed, and Avoid on the LiDAR tab · Params and Log are under More", 14, "#59636e")
    for i, (name, cap, fn) in enumerate(screens):
        x = margin + i * (PH_W + gap)
        phone(s, x, title_h, fn)
        s.text(x + PH_W / 2, title_h + PH_H + 30, name, 17, "#1f2328", 600, "middle")
        s.text(x + PH_W / 2, title_h + PH_H + 50, cap, 12, "#59636e", anchor="middle")
    s.e("</svg>")
    s.save(os.path.join(ROOT, "apple_gcs/docs/screens-ios.svg"))


MAC_W, MAC_H = 640, 470


def mac_frame(s, x, y, tab):
    for k, op in ((14, 0.05), (8, 0.06), (3, 0.08)):
        s.rect(x - k / 2, y + k, MAC_W + k, MAC_H, "#000", rx=12 + k / 2, opacity=op)
    s.rect(x, y, MAC_W, MAC_H, "#FFF", rx=11, stroke="#C8C8C8")
    s.clip(x, y, MAC_W, MAC_H, 11)
    s.rect(x, y, MAC_W, 46, "#F6F6F6")
    for i, col in enumerate(("#FF5F57", "#FEBC2E", "#28C840")):
        s.e(f'<circle cx="{x + 20 + i * 20}" cy="{y + 22}" r="6.5" fill="{col}"/>')
    names = ["Data", "Plan", "Params", "Setup", "LiDAR", "Log"]
    seg_w = 56
    sx = x + MAC_W / 2 - seg_w * len(names) / 2
    s.rect(sx - 4, y + 8, seg_w * len(names) + 8, 30, "#EBEBEB", rx=15)
    for i, n in enumerate(names):
        if n == tab:
            s.rect(sx + i * seg_w + 2, y + 11, seg_w - 4, 24, "#D9D9D9", rx=12)
        s.text(sx + i * seg_w + seg_w / 2, y + 27.5, n, 11.5, "#000", 500 if n == tab else None, "middle")
    s.e(f'<line x1="{x}" y1="{y + 46}" x2="{x + MAC_W}" y2="{y + 46}" stroke="#E0E0E0"/>')
    py = y + 50
    s.clip(x, py, MAC_W, 22)
    status_pills(s, x + 8, py + 1)
    s.end()
    s.e(f'<line x1="{x}" y1="{py + 24}" x2="{x + MAC_W}" y2="{py + 24}" stroke="#E5E5EA"/>')
    return x, py + 25, MAC_W, y + MAC_H - (py + 25)


def mac_data(s, x, y):
    cx, cy, cw, ch = mac_frame(s, x, y, "Data")
    hud(s, cx, cy, cw, 118, "swiftui", roll=-2)
    my = cy + 118
    s.clip(cx, my, cw, 32)
    mode_row(s, cx + 8, my + 5, cx + cw + 400)
    s.end()
    my += 32
    s.clip(cx, my, cw, cy + ch - my)
    satellite_map(s, cx, my, cw, cy + ch - my, seed=11)
    mission_overlay(s, cx, my, cw, cy + ch - my, home_style="apple", car=(0.47, 0.3, 78))
    tap_menu(s, cx + 8, my + 8)
    map_buttons(s, cx + cw - 42, my + 8)
    s.text(cx + 10, cy + ch - 8, "Maps", 10, "#FFF", 600)
    s.end()
    s.end()


def mac_plan(s, x, y):
    cx, cy, cw, ch = mac_frame(s, x, y, "Plan")
    s.rect(cx + cw / 2 - 90, cy + 6, 180, 22, "#E9E9EB", rx=6)
    s.rect(cx + cw / 2 - 88, cy + 8, 88, 18, A_BLUE, rx=5)
    s.text(cx + cw / 2 - 44, cy + 21, "Waypoints", 10.5, "#FFF", 600, "middle")
    s.text(cx + cw / 2 + 44, cy + 21, "Fence", 10.5, "#000", None, "middle")
    my = cy + 34
    mh = 250
    s.clip(cx, my, cw, mh)
    satellite_map(s, cx, my, cw, mh, seed=13)
    mission_overlay(s, cx, my, cw, mh, home_style="apple", car=(0.6, 0.3, 140))
    map_buttons(s, cx + cw - 42, my + 8)
    s.end()
    y2 = my + mh + 6
    x2 = cx + 8
    for lb, st, tint in (("Read", "b", None), ("Write", "p", None), ("Start Auto", "b", A_GREEN), ("Zig zag…", "b", None),
                         ("Patrol…", "b", None), ("Load", "b", None), ("Save", "b", None), ("Undo", "b", None), ("Clear", "b", None)):
        x2 += abutton(s, x2, y2, lb, "prominent" if st == "p" else "bordered", tint, size=10)
    y2 += 30
    s.text(cx + 10, y2 + 12, "Mission speed", 10.5, "#000")
    s.e(f'<line x1="{cx + 100}" y1="{y2 + 8}" x2="{cx + cw - 100}" y2="{y2 + 8}" stroke="#D1D1D6" stroke-width="4" stroke-linecap="round"/>')
    s.e(f'<circle cx="{cx + 100}" cy="{y2 + 8}" r="8" fill="#FFF" stroke="#D1D1D6"/>')
    s.text(cx + cw - 10, y2 + 12, "WP_SPEED", 10.5, "#000", anchor="end")
    y2 += 22
    for i, (lb, pos) in enumerate((("Waypoint", "10.774406, 106.692243"), ("Waypoint", "10.774187, 106.692378"),
                                   ("Waypoint", "10.774096, 106.692108"), ("Waypoint", "10.774015, 106.691964"))):
        ry = y2 + i * 22
        if ry + 22 > cy + ch:
            break
        s.text(cx + 14, ry + 15, str(i + 1), 11, "#000", 700)
        s.text(cx + 36, ry + 15, lb, 11, "#000")
        s.text(cx + cw - 10, ry + 15, pos, 9, A_GRAY, anchor="end")
        s.e(f'<line x1="{cx + 36}" y1="{ry + 21}" x2="{cx + cw}" y2="{ry + 21}" stroke="#E5E5EA"/>')
    s.end()


def mac_lidar(s, x, y):
    cx, cy, cw, ch = mac_frame(s, x, y, "LiDAR")
    r = 140
    scan_disc(s, cx + cw / 2, cy + 12 + r, r, "swiftui")
    yy = cy + 12 + 2 * r + 16
    s.text(cx + cw / 2, yy, "2316 scans · range 4 m · nearest 1.05 m at -15°", 10, A_GRAY, anchor="middle")
    yy += 10
    s.rect(cx + cw / 2 - 150, yy, 300, 24, "#E9E9EB", rx=6)
    for i, n in enumerate(("Off", "Avoid", "Follow", "Warn")):
        if n == "Avoid":
            s.rect(cx + cw / 2 - 148 + i * 74, yy + 2, 74, 20, A_BLUE, rx=5)
        s.text(cx + cw / 2 - 148 + i * 74 + 37, yy + 16, n, 10.5, "#FFF" if n == "Avoid" else "#000", 600 if n == "Avoid" else None, "middle")
    yy += 32
    x2 = cx + 30 + abutton(s, cx + 30, yy, "Stop", "prominent", A_RED, h=26)
    s.text(x2 + 6, yy + 17, "Front+right blocked: turn left   thr +0.15  steer -0.80", 11, "#000", 600)
    s.end()


def mac_svg():
    s = Svg()
    wins = [("Data", "HUD, Arm and modes, satellite map; arrow keys / WASD drive", mac_data),
            ("Plan", "Click to add waypoints, Write, Start Auto; zig-zag, patrol, fences", mac_plan),
            ("LiDAR", "The scan and Avoid / Follow / Warn", mac_lidar)]
    gap, margin, title_h = 40, 40, 96
    w = margin * 2 + len(wins) * MAC_W + (len(wins) - 1) * gap
    h = title_h + MAC_H + 74
    s.e(f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
        'font-family="-apple-system, SF Pro Text, Helvetica Neue, Helvetica, Arial, sans-serif" font-size="12">')
    s.rect(0, 0, w, h, "#FFFFFF")
    s.text(margin, 46, "ESP32 Rover GCS — Mac app", 24, "#1f2328", 600)
    s.text(margin, 72, "The same SwiftUI app as the iPhone one, macOS 14+ · drawn from the app's layout (as tested against "
           "tools/fake_rover.py), with a car in Auto and RTK Fixed", 14, "#59636e")
    for i, (name, cap, fn) in enumerate(wins):
        x = margin + i * (MAC_W + gap)
        fn(s, x, title_h)
        s.text(x + MAC_W / 2, title_h + MAC_H + 36, name, 17, "#1f2328", 600, "middle")
        s.text(x + MAC_W / 2, title_h + MAC_H + 56, cap, 12.5, "#59636e", anchor="middle")
    s.e("</svg>")
    s.save(os.path.join(ROOT, "apple_gcs/docs/screens-mac.svg"))


if __name__ == "__main__":
    windows_svg()
    ios_svg()
    mac_svg()
