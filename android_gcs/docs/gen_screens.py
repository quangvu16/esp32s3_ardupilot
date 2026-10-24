#!/usr/bin/env python3
"""Generate android_gcs/docs/screens.svg: the app's Data, Plan, Params and Setup tabs.

Usage: python3 android_gcs/docs/gen_screens.py android_gcs/docs/screens.svg
A drawing, not a screenshot: layout, labels and colours follow MainActivity.kt,
HudView.kt, MapView.kt, RadarView.kt and res/values/colors.xml (light theme).
The map is made up; the vehicle state is a car in Auto with RTK Fixed.
"""
import math
import sys

# res/values/colors.xml
BG, SURFACE, SUNK = "#EEF1EE", "#FFFFFF", "#E3E8E4"
FG, MUTED, LINE = "#17201C", "#5A6861", "#CFD7D1"
ACCENT, ACCENT_SOFT, ON_ACCENT = "#1F5FD1", "#DFE8FA", "#FFFFFF"
OK, OK_SOFT, WARN, WARN_SOFT = "#2F7D4F", "#DCEFE2", "#A16207", "#FBF0D2"
# MapView / HudView paints
ROUTE, WP_CUR, HOME, TRAIL, TARGET = "#FACC15", "#22C55E", "#EF4444", "#EC4899", "#3B82F6"

PW, PH = 300, 640          # phone body
SX, SY = 10, 10            # screen inset
W, H = PW - 2 * SX, PH - 2 * SY
STATUS, PILLS, TABS = 22, 32, 40
MONO = "DejaVu Sans Mono, Menlo, Consolas, monospace"

out = []
uid = [0]


def e(s):
    out.append(s)


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text(x, y, t, size=10.5, fill=FG, weight=None, anchor="start", family=None, extra=""):
    w = f' font-weight="{weight}"' if weight else ""
    f = f' font-family="{family}"' if family else ""
    a = f' text-anchor="{anchor}"' if anchor != "start" else ""
    e(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}"{w}{a}{f}{extra}>{esc(t)}</text>')


def tw(t, size):
    """Rough text width for the UI font."""
    return len(t) * size * 0.56


def button(x, y, label, accent=False, h=28, shadow=True):
    w = tw(label, 10.5) + 20
    if shadow:
        e(f'<rect x="{x:.1f}" y="{y + 1.2:.1f}" width="{w:.1f}" height="{h}" rx="6" fill="#000" opacity="0.10"/>')
    e(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h}" rx="6" fill="{ACCENT if accent else SURFACE}"/>')
    text(x + w / 2, y + h / 2 + 3.8, label, fill=ON_ACCENT if accent else FG, anchor="middle")
    return w


def button_row(y, labels, accent=(), x0=6):
    x = x0
    for lb in labels:
        x += button(x, y, lb, lb in accent) + 4.5
        if x > W:
            break


def pill(x, y, label, tone):
    bg, fg = {"ok": (OK_SOFT, OK), "warn": (WARN_SOFT, WARN), "bad": ("#FBE7DC", "#C2410C")}.get(tone, (SUNK, MUTED))
    w = len(label) * 9 * 0.62 + 15
    e(f'<rect x="{x:.1f}" y="{y}" width="{w:.1f}" height="18" rx="9" fill="{bg}"/>')
    text(x + 7.5, y + 12.6, label, 9, fg, family=MONO)
    return w + 4.5


def chrome(tab):
    """Status bar, the pill bar and the tab bar; returns the content area's top and bottom."""
    e(f'<rect width="{W}" height="{STATUS + PILLS}" fill="{SURFACE}"/>')
    text(14, 15, "9:41", 10, FG, 600)
    for i in range(3):
        e(f'<rect x="{W - 40 + i * 5}" y="{11 - i * 2}" width="3" height="{4 + i * 2}" rx="0.6" fill="{FG}"/>')
    e(f'<rect x="{W - 22}" y="6" width="14" height="8" rx="2" fill="none" stroke="{FG}" stroke-width="1"/>'
      f'<rect x="{W - 20.5}" y="7.5" width="9" height="5" rx="1" fill="{FG}"/>')
    x = 6
    for lb, tone in (("● TCP 192.168.4.1:5760", "ok"), ("Auto · ARMED", "bad"),
                     ("RTK Fixed · 28 sats", "ok"), ("12.42 V · 86%", "")):
        x += pill(x, STATUS + 7, lb, tone)
    top = STATUS + PILLS
    bot = H - TABS
    e(f'<rect y="{bot}" width="{W}" height="{TABS}" fill="{SURFACE}"/>')
    names = ["Data", "Plan", "Params", "Setup", "Log"]
    tw_ = (W - 6 - 4 * 2.5) / 5
    for i, n in enumerate(names):
        x = 3 + i * (tw_ + 2.5)
        on = n == tab
        e(f'<rect x="{x:.1f}" y="{bot + 4}" width="{tw_:.1f}" height="32" rx="6" fill="{ACCENT_SOFT if on else SURFACE}"/>')
        text(x + tw_ / 2, bot + 24, n, 10.5, ACCENT if on else FG, 600 if on else None, "middle")
    return top, bot


# ---------------------------------------------------------------------------
# map

def map_area(x, y, w, h, wp_pts, cur=None, tools=False, vehicle=None, trail=True, target=None):
    uid[0] += 1
    cid = f"map{uid[0]}"
    e(f'<clipPath id="{cid}"><rect x="{x}" y="{y}" width="{w}" height="{h}"/></clipPath><g clip-path="url(#{cid})">')
    e(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="#F2EFE9"/>')
    # made-up streets, a park and a pond in the OpenStreetMap style
    e(f'<path d="M{x + w * 0.52} {y + h * 0.08} l{w * 0.42} {h * 0.05} l-{w * 0.05} {h * 0.33} l-{w * 0.4} -{h * 0.04} z" fill="#CDEBB0"/>')
    e(f'<ellipse cx="{x + w * 0.75}" cy="{y + h * 0.22}" rx="{w * 0.1}" ry="{h * 0.06}" fill="#AAD3DF"/>')
    e(f'<path d="M{x - 10} {y + h * 0.78} l{w + 20} -{h * 0.05} l0 {h * 0.3} l-{w + 20} 0 z" fill="#E0DFDF" opacity="0.55"/>')
    roads = [
        (x - 10, y + h * 0.58, x + w + 10, y + h * 0.52, 9),
        (x + w * 0.3, y - 10, x + w * 0.36, y + h + 10, 7),
        (x + w * 0.78, y + h * 0.5, x + w * 0.83, y + h + 10, 6),
        (x - 10, y + h * 0.2, x + w * 0.5, y + h * 0.15, 5),
        (x + w * 0.36, y + h * 0.82, x + w + 10, y + h * 0.88, 5),
    ]
    for (x1, y1, x2, y2, s) in roads:
        e(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#CFC8BE" stroke-width="{s + 2}" stroke-linecap="round"/>')
    for (x1, y1, x2, y2, s) in roads:
        e(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#FFFFFF" stroke-width="{s}" stroke-linecap="round"/>')
    for bx, by, bw, bh in ((0.05, 0.27, 0.16, 0.12), (0.06, 0.66, 0.14, 0.09), (0.44, 0.62, 0.2, 0.1), (0.88, 0.6, 0.1, 0.14)):
        e(f'<rect x="{x + w * bx:.1f}" y="{y + h * by:.1f}" width="{w * bw:.1f}" height="{h * bh:.1f}" fill="#D9D0C9" stroke="#C4B6AB" stroke-width="0.6"/>')

    home = wp_pts[0]
    pts = [(x + w * px, y + h * py) for (px, py, _) in wp_pts]
    if trail:
        tr = [(pts[0][0] + 3, pts[0][1] - 2), (pts[1][0] - 4, pts[1][1] + 30), (pts[1][0] - 2, pts[1][1] + 6),
              ((pts[1][0] + pts[2][0]) / 2 - 3, (pts[1][1] + pts[2][1]) / 2 + 4)]
        e('<polyline points="' + " ".join(f"{a:.1f},{b:.1f}" for a, b in tr) +
          f'" fill="none" stroke="{TRAIL}" stroke-width="1.6" opacity="0.85" stroke-linejoin="round"/>')
    e('<polyline points="' + " ".join(f"{a:.1f},{b:.1f}" for a, b in pts) +
      f'" fill="none" stroke="{ROUTE}" stroke-width="2.4" stroke-linejoin="round"/>')
    for (px, py), (_, _, label) in zip(pts[1:], wp_pts[1:]):
        fill = WP_CUR if label == cur else ROUTE
        e(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="8.5" fill="{fill}" stroke="#000" stroke-width="1.1"/>')
        text(px, py + 3.4, str(label), 8.5, "#000", 700, "middle")
    hx, hy = pts[0]
    e(f'<rect x="{hx - 7.5:.1f}" y="{hy - 7.5:.1f}" width="15" height="15" fill="{HOME}" stroke="#000" stroke-width="1.1"/>')
    text(hx, hy + 3.4, "H", 8.5, "#FFF", 700, "middle")
    if target:
        tx, ty = x + w * target[0], y + h * target[1]
        e(f'<g stroke="{TARGET}" stroke-width="2.2" fill="none"><circle cx="{tx}" cy="{ty}" r="9"/>'
          f'<line x1="{tx - 13}" y1="{ty}" x2="{tx + 13}" y2="{ty}"/><line x1="{tx}" y1="{ty - 13}" x2="{tx}" y2="{ty + 13}"/></g>')
    if vehicle:
        vx, vy, hdg = x + w * vehicle[0], y + h * vehicle[1], vehicle[2]
        s = 11
        e(f'<path transform="rotate({hdg} {vx:.1f} {vy:.1f})" d="M{vx:.1f} {vy - s:.1f} L{vx + s * 0.7:.1f} {vy + s * 0.8:.1f} '
          f'L{vx:.1f} {vy + s * 0.4:.1f} L{vx - s * 0.7:.1f} {vy + s * 0.8:.1f} Z" fill="{HOME}" stroke="#FFF" stroke-width="1.6" stroke-linejoin="round"/>')
    # scale bar and credit (MapView.drawScale / credit)
    e(f'<rect x="{x + 6}" y="{y + h - 28}" width="70" height="15" fill="#FFF" opacity="0.67"/>')
    e(f'<line x1="{x + 9}" y1="{y + h - 16}" x2="{x + 66}" y2="{y + h - 16}" stroke="#000" stroke-width="1.6"/>')
    text(x + 9, y + h - 19, "20 m", 7.5, "#000")
    cr = "© OpenStreetMap contributors"
    e(f'<rect x="{x + w - tw(cr, 7) - 8:.1f}" y="{y + h - 11}" width="{tw(cr, 7) + 8:.1f}" height="11" fill="#FFF" opacity="0.67"/>')
    text(x + w - 3, y + h - 3, cr, 7, "#141414", anchor="end")
    if tools:
        ty = y + 5
        for lb in ("Follow", "Fit", "Satellite", "Drive"):
            bw = tw(lb, 10.5) + 20
            button(x + w - bw - 5, ty, lb)
            ty += 33
    e("</g>")


# ---------------------------------------------------------------------------
# screens

MISSION = [(0.2, 0.84, "H"), (0.27, 0.3, 2), (0.66, 0.24, 3), (0.72, 0.7, 4)]


def hud(y):
    h = 174
    cx, cy = W / 2, y + h / 2
    roll, pitch = -7, 3
    ppd = h / 50
    uid[0] += 1
    cid = f"hud{uid[0]}"
    e(f'<clipPath id="{cid}"><rect y="{y}" width="{W}" height="{h}"/></clipPath><g clip-path="url(#{cid})">')
    off = pitch * ppd
    e(f'<g transform="rotate({-roll} {cx} {cy})">')
    e(f'<rect x="-300" y="{y - 300}" width="{W + 600}" height="{300 + h / 2 + off}" fill="#2E74C4"/>')
    e(f'<rect x="-300" y="{cy + off}" width="{W + 600}" height="400" fill="#7A562C"/>')
    e(f'<line x1="-300" y1="{cy + off}" x2="{W + 300}" y2="{cy + off}" stroke="#FFF" stroke-width="1.2"/>')
    for deg in (-20, -10, 10, 20):
        yy = cy + off - deg * ppd
        half = W * (0.12 if deg % 20 == 0 else 0.07)
        e(f'<line x1="{cx - half:.1f}" y1="{yy:.1f}" x2="{cx + half:.1f}" y2="{yy:.1f}" stroke="#FFF" stroke-width="1.1"/>')
        text(cx + half + 3, yy + 3, str(deg), 8, "#FFF")
    e("</g>")
    r = h * 0.42
    for a in (-45, -30, -20, -10, 0, 10, 20, 30, 45):
        ln = 7 if a % 30 == 0 else 4
        e(f'<line transform="rotate({a} {cx} {cy})" x1="{cx}" y1="{cy - r}" x2="{cx}" y2="{cy - r - ln}" stroke="#FFF" stroke-width="1.1"/>')
    e(f'<path transform="rotate({-roll} {cx} {cy})" d="M{cx} {cy - r + 1} l-4.5 8 h9 z" fill="#FACC15"/>')
    e(f'<polyline points="{cx - 38},{cy} {cx - 12},{cy} {cx - 6},{cy + 6} {cx},{cy} {cx + 6},{cy + 6} {cx + 12},{cy} {cx + 38},{cy}" '
      'fill="none" stroke="#FACC15" stroke-width="2.3" stroke-linejoin="round"/>')
    # heading tape
    tape, hdg = 20, 342
    e(f'<rect y="{y}" width="{W}" height="{tape}" fill="#000" opacity="0.43"/>')
    ppx = W / 90
    for k in range(hdg - 50, hdg + 51):
        if k % 5:
            continue
        xx = cx + (k - hdg) * ppx
        hd = k % 360
        e(f'<line x1="{xx:.1f}" y1="{y + tape}" x2="{xx:.1f}" y2="{y + tape - (6 if hd % 15 == 0 else 3)}" stroke="#FFF" stroke-width="1"/>')
        lbl = {0: "N", 45: "NE", 90: "E", 135: "SE", 180: "S", 225: "SW", 270: "W", 315: "NW"}.get(hd)
        if lbl is None and hd % 15 == 0:
            lbl = str(hd)
        if lbl:
            text(xx, y + 9.5, lbl, 8, "#FFF", anchor="middle")
    e(f'<rect x="{cx - 19}" y="{y + tape}" width="38" height="16" fill="#000" opacity="0.43"/>')
    text(cx, y + tape + 12.5, f"{hdg:03d}°", 11, "#FFF", 700, "middle")
    sh = ' style="paint-order:stroke" stroke="#000" stroke-width="2" stroke-opacity="0.45"'
    text(6, cy - 3, "1.6 m/s", 11.5, "#FFF", 700, extra=sh)
    text(6, cy + 11, "thr 34%", 9, "#FFF", extra=sh)
    text(W - 6, cy - 3, "Auto", 11.5, "#FFF", 700, "end", extra=sh)
    text(W - 6, cy + 12, "ARMED", 11.5, "#F87171", 700, "end", extra=sh)
    e(f'<rect y="{y + h - 26}" width="{W}" height="26" fill="#000" opacity="0.43"/>')
    text(6, y + h - 15, "GPS RTK Fixed  28 sats  hdop 0.52  ±0.01 m", 9, "#FFF")
    text(6, y + h - 4, "WP 2  12 m  xtrack 0.1 m", 9, "#FFF")
    text(W - 6, y + h - 15, "12.42 V  1.8 A", 9, "#FFF", anchor="end")
    e("</g>")
    return h


def data_screen():
    top, bot = chrome("Data")
    y = top
    y += hud(y)
    text(8, y + 13, "Mission: 2 WP", 9, MUTED)
    y += 19
    button_row(y + 4, ["Disarm", "Manual", "Hold", "Auto", "RTL", "Guided", "Acro"], accent=("Disarm",))
    y += 4 + 28 + 5
    map_area(0, y, W, bot - y, MISSION, cur=2, tools=True, vehicle=(0.44, 0.27, 74))


def plan_screen():
    top, bot = chrome("Plan")
    y = top
    button_row(y + 6, ["Read", "Write", "Tap adds: on", "Speed…", "Clear"], accent=("Write",))
    y += 6 + 28 + 4
    text(8, y + 9, "Mission uploaded: 5 items", 9, MUTED)
    y += 15
    list_h = 112
    map_area(0, y, W, bot - y - list_h, MISSION, cur=2, vehicle=(0.44, 0.27, 74))
    ly = bot - list_h
    e(f'<rect y="{ly}" width="{W}" height="{list_h}" fill="{BG}"/>')
    rows = ["1. Speed 3.0 m/s", "2. Waypoint  48 m", "3. Waypoint, wait 2 s  61 m", "4. Waypoint  55 m",
            "5. Return to launch"]
    for i, r in enumerate(rows):
        text(8, ly + 15 + i * 18, r, 10)
    text(8, ly + 15 + 5 * 18, "Total 164 m", 9, MUTED)


def params_screen():
    top, bot = chrome("Params")
    y = top
    button_row(y + 6, ["Refresh", "Write 2", "Load file", "Save file", "Undo edits"], accent=("Write 2",))
    y += 6 + 28 + 8
    text(10, y + 12, "WP_", 11)
    e(f'<line x1="8" y1="{y + 19}" x2="{W - 8}" y2="{y + 19}" stroke="{ACCENT}" stroke-width="1.5"/>')
    e(f'<line x1="{10 + tw("WP_", 11) + 1:.1f}" y1="{y + 2}" x2="{10 + tw("WP_", 11) + 1:.1f}" y2="{y + 15}" stroke="{ACCENT}" stroke-width="1"/>')
    y += 27
    text(9, y + 9, "Loaded: 2 values differ from the vehicle. Press Write.", 9, MUTED)
    y += 16
    rows = [("WP_ACCEL", "0", None), ("WP_JERK", "0", None), ("WP_OVERSHOOT", "2", None),
            ("WP_PIVOT_ANGLE", "60", None), ("WP_PIVOT_DELAY", "0", None), ("WP_PIVOT_RATE", "90", None),
            ("WP_RADIUS", "2", "1"), ("WP_SPEED", "2", "2.5")]
    rh = 27
    for i, (n, v, st) in enumerate(rows):
        ry = y + i * rh
        if ry + rh > bot:
            break
        text(9, ry + 17, n, 10.5, FG, family=MONO)
        val = f"{v} → {st}" if st else v
        text(W - 9, ry + 17, val, 10.5, WARN if st else FG, 700, "end", MONO)
        e(f'<line x1="0" y1="{ry + rh}" x2="{W}" y2="{ry + rh}" stroke="{LINE}" stroke-width="0.6"/>')


def setup_screen():
    top, bot = chrome("Setup")
    uid[0] += 1
    cid = f"setup{uid[0]}"
    e(f'<clipPath id="{cid}"><rect y="{top}" width="{W}" height="{bot - top}"/></clipPath><g clip-path="url(#{cid})">')
    x0 = 11
    y = top + 4

    def section(t):
        nonlocal y
        y += 14
        text(x0, y + 9, t, 9, MUTED, 700, extra=' letter-spacing="0.7"')
        y += 15

    section("RADIO")
    text(x0, y + 9, "Ch 1 steering, ch 3 throttle, ch 8 mode switch.", 9, MUTED)
    y += 16
    pwm = [1502, 1500, 1500, 1500, 1100, 1100, 1100, 1900]
    for i, p in enumerate(pwm):
        bw = W - 2 * x0 - 112
        e(f'<rect x="{x0}" y="{y + 4}" width="{bw}" height="4" rx="2" fill="{ACCENT_SOFT}"/>')
        e(f'<rect x="{x0}" y="{y + 4}" width="{bw * (p - 900) / 1200:.1f}" height="4" rx="2" fill="{ACCENT}"/>')
        text(x0 + bw + 6, y + 9.5, f"{i + 1}: {p}", 9, FG, family=MONO)
        y += 14
    y += 4
    button(x0, y, "Calibrate radio")
    y += 34

    section("LIDAR PROXIMITY")
    rh = 170
    cx, cy = W / 2, y + rh / 2
    r = rh / 2 - 11
    for k in range(1, 5):
        e(f'<circle cx="{cx}" cy="{cy}" r="{r * k / 4:.1f}" fill="none" stroke="{LINE}" stroke-width="0.8"/>')
        text(cx + 2.5, cy - r * k / 4 + 8.5, f"{2 * k} m", 7.5, MUTED)
    dists = [0, 3.4, 0, 5.2, 0, 3.8, 6.4, 1.7]
    for i, dd in enumerate(dists):
        if dd <= 0:
            continue
        rr = r * min(max(dd / 8, 0.05), 1)
        t = min(max(dd / 2, 0), 1)
        col = f"rgb({int(239 - 205 * t)},{int(68 + 129 * t)},{int(68 + 26 * t)})"
        a0 = math.radians(i * 45 - 90 - 22.5 + 2)
        a1 = math.radians(i * 45 - 90 - 22.5 + 43)
        e(f'<path d="M{cx + rr * math.cos(a0):.1f} {cy + rr * math.sin(a0):.1f} A{rr:.1f} {rr:.1f} 0 0 1 '
          f'{cx + rr * math.cos(a1):.1f} {cy + rr * math.sin(a1):.1f}" fill="none" stroke="{col}" stroke-width="5" opacity="0.8"/>')
        a = math.radians(i * 45 - 90)
        tr = rr + 10
        text(cx + tr * math.cos(a), cy + tr * math.sin(a) + 3, f"{dd:.1f}", 8.5, FG, 700, "middle")
    e(f'<rect x="{cx - 5.5}" y="{cy - 9}" width="11" height="18" fill="{ACCENT}"/>')
    text(cx, cy - 11, "▲", 8, FG, anchor="middle")
    y += rh + 2
    text(x0, y + 9, "Needs PRX1_TYPE=19 (LD14). Put a box in front: it", 8.5, MUTED)
    text(x0, y + 20, "should show at the top; else set PRX1_YAW_CORR.", 8.5, MUTED)
    y += 26

    section("RTK / NTRIP")
    for lab, val in (("Host", "rtk2go.com"), ("Port", "2101"), ("Mount", "HCM_BASE")):
        text(x0, y + 13, lab, 9.5, MUTED)
        text(x0 + 74, y + 13, val, 10.5)
        e(f'<line x1="{x0 + 72}" y1="{y + 20}" x2="{W - x0}" y2="{y + 20}" stroke="{MUTED}" stroke-width="0.8"/>')
        y += 26
    e("</g>")


SCREENS = [
    ("Data", "HUD, modes and arm, map with mission and trail", data_screen),
    ("Plan", "Tap to add, drag to move, Write to the car", plan_screen),
    ("Params", "Search, edit, load a .parm: only changes staged", params_screen),
    ("Setup", "Radio, LiDAR proximity, RTK over NTRIP, calibration", setup_screen),
]


def main(path):
    gap, margin, title_h = 44, 40, 96
    total_w = margin * 2 + len(SCREENS) * PW + (len(SCREENS) - 1) * gap
    total_h = title_h + PH + 64
    e(f'<svg xmlns="http://www.w3.org/2000/svg" width="{total_w}" height="{total_h}" viewBox="0 0 {total_w} {total_h}" '
      'font-family="Roboto, Segoe UI, Helvetica, Arial, sans-serif" font-size="14">')
    e(f'<rect width="{total_w}" height="{total_h}" fill="#ffffff"/>')
    text(margin, 46, "ESP32 Rover GCS — Android app", 24, "#1f2328", 600)
    text(margin, 72, "MAVLink over the board's WiFi (TCP 192.168.4.1:5760) · drawn from the app's layout, "
         "shown with a car in Auto and RTK Fixed", 14, "#59636e")
    for i, (name, caption, fn) in enumerate(SCREENS):
        px = margin + i * (PW + gap)
        py = title_h
        e(f'<g transform="translate({px} {py})">')
        for k, op in ((9, 0.04), (6, 0.05), (3, 0.07)):      # soft shadow without filters
            e(f'<rect x="{-k / 2}" y="{k}" width="{PW + k}" height="{PH}" rx="{38 + k / 2}" fill="#000" opacity="{op}"/>')
        e(f'<rect width="{PW}" height="{PH}" rx="38" fill="#1B1F23"/>')
        e(f'<g transform="translate({SX} {SY})">')
        uid[0] += 1
        cid = f"scr{uid[0]}"
        e(f'<clipPath id="{cid}"><rect width="{W}" height="{H}" rx="29"/></clipPath><g clip-path="url(#{cid})">')
        e(f'<rect width="{W}" height="{H}" fill="{BG}"/>')
        fn()
        e("</g></g>")
        e(f'<rect x="{PW / 2 - 40}" y="{PH - 7}" width="80" height="3" rx="1.5" fill="#666"/>')
        e("</g>")
        text(px + PW / 2, py + PH + 30, name, 17, "#1f2328", 600, "middle")
        text(px + PW / 2, py + PH + 50, caption, 12.5, "#59636e", anchor="middle")
    e("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "screens.svg")
