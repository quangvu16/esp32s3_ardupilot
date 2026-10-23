#!/usr/bin/env python3
"""Generate docs/wiring.svg for the esp32s3devkit RC rover build.

Usage: python3 docs/gen_wiring.py docs/wiring.svg
Pin assignments must match libraries/AP_HAL_ESP32/boards/esp32s3devkit.h.
"""
import sys

W, H = 1300, 1060
out = []
def add(s): out.append(s)

# ESP32-S3-DevKitC-1 headers, antenna end first
J1 = ["3V3", "3V3", "RST", "4", "5", "6", "7", "15", "16", "17", "18", "8", "3", "46",
      "9", "10", "11", "12", "13", "14", "5V", "GND"]
J3 = ["GND", "43 TX", "44 RX", "1", "2", "42", "41", "40", "39", "38", "37", "36", "35",
      "0", "45", "48", "47", "21", "20", "19", "GND", "GND"]

BX0, BX1 = 580, 860          # board body
PY0, PSTEP = 180, 26         # first pin y, pitch
LX, RX = BX0 + 16, BX1 - 16  # pin dot x
def py(i): return PY0 + i * PSTEP

C = {  # colours
    "bg": "#ffffff", "ink": "#1f2328", "muted": "#59636e", "box": "#f6f8fa", "edge": "#8c959f",
    "board": "#1b4d3e", "boardtxt": "#e6f4ea", "pwm": "#d4700a", "rc": "#8250df",
    "i2c": "#0f8a7e", "uart": "#0969da", "pwr": "#cf222e", "gnd": "#1f2328", "dead": "#9aa1a9",
}

used_left = {"15": ("SCL", "i2c"), "16": ("SDA", "i2c"), "17": ("GPS RX", "uart"),
             "18": ("GPS TX", "uart"), "9": ("SERVO3", "pwm"), "11": ("SERVO1", "pwm"),
             "14": ("RCIN", "rc"), "5V": ("", "pwr"), "GND": ("", "gnd")}
used_right = {"41": ("LiDAR RX", "uart"), "43 TX": ("console", "muted"),
              "44 RX": ("console", "muted")}
dead_right = {"35", "36", "37"}

add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
    f'font-family="Segoe UI, Helvetica, Arial, sans-serif" font-size="14">')
add(f'<rect width="{W}" height="{H}" fill="{C["bg"]}"/>')
add(f'<text x="40" y="44" font-size="24" font-weight="600" fill="{C["ink"]}">'
    f'ESP32-S3 RC rover — wiring</text>')
add(f'<text x="40" y="70" fill="{C["muted"]}">ArduRover 4.6 · esp32s3devkit firmware · '
    f'board drawn as ESP32-S3-DevKitC-1 (other S3 boards: match by GPIO number)</text>')

# legend
lg = [("pwm", "PWM out"), ("rc", "RC in"), ("i2c", "I2C"), ("uart", "UART"), ("pwr", "+5 V"),
      ("gnd", "GND")]
x = 820
for k, t in lg:
    add(f'<line x1="{x}" y1="40" x2="{x+22}" y2="40" stroke="{C[k]}" stroke-width="4"/>')
    add(f'<text x="{x+28}" y="45" fill="{C["ink"]}">{t}</text>')
    x += 28 + 8 * len(t) + 18

# board
add(f'<rect x="{BX0}" y="{PY0-62}" width="{BX1-BX0}" height="{py(21)-PY0+110}" rx="10" '
    f'fill="{C["board"]}"/>')
add(f'<rect x="{BX0+40}" y="{PY0-56}" width="{BX1-BX0-80}" height="34" rx="4" fill="#2f6b58"/>')
add(f'<text x="{(BX0+BX1)//2}" y="{PY0-34}" text-anchor="middle" fill="{C["boardtxt"]}" '
    f'font-size="12">antenna</text>')
add(f'<text x="{(BX0+BX1)//2}" y="{py(10)}" text-anchor="middle" fill="{C["boardtxt"]}" '
    f'font-size="16" font-weight="600">ESP32-S3</text>')
add(f'<text x="{(BX0+BX1)//2}" y="{py(10)+20}" text-anchor="middle" fill="{C["boardtxt"]}" '
    f'font-size="12">DevKitC-1 N8R8</text>')
add(f'<text x="{(BX0+BX1)//2}" y="{py(10)+38}" text-anchor="middle" fill="{C["boardtxt"]}" '
    f'font-size="12">(top view)</text>')
ub = py(21) + 30
for ux in (BX0 + 46, BX1 - 86):
    add(f'<rect x="{ux}" y="{ub}" width="40" height="24" rx="3" fill="#c9d1d9"/>')
add(f'<text x="{(BX0+BX1)//2}" y="{ub+44}" text-anchor="middle" fill="{C["ink"]}" font-size="13">'
    f'USB connectors: flash via the one marked <tspan font-weight="600">UART</tspan></text>')

def pin(x, y, label, side, state, color_key):
    if state == "used":
        fill = C[color_key] if color_key != "muted" else C["boardtxt"]
    else:
        fill = C["dead"] if state == "dead" else "#6e9a8c"
    add(f'<circle cx="{x}" cy="{y}" r="6" fill="{fill}" stroke="#0b2e24" stroke-width="1"/>')
    tx, anchor = (x + 12, "start") if side == "L" else (x - 12, "end")
    txt = label if label in ("3V3", "RST", "5V", "GND") or " " in label else "GPIO" + label
    col = C["boardtxt"] if state != "dead" else C["dead"]
    weight = "600" if state == "used" else "400"
    deco = ' text-decoration="line-through"' if state == "dead" else ""
    add(f'<text x="{tx}" y="{y+4}" text-anchor="{anchor}" fill="{col}" font-size="12" '
        f'font-family="Consolas, Menlo, monospace" font-weight="{weight}"{deco}>{txt}</text>')

for i, lab in enumerate(J1):
    if lab in used_left or (lab == "3V3" and i == 0):
        ck = used_left.get(lab, ("", "pwr"))[1] if lab != "3V3" else "pwr"
        pin(LX, py(i), lab, "L", "used", ck)
    else:
        pin(LX, py(i), lab, "L", "free", "")
for i, lab in enumerate(J3):
    if lab in dead_right:
        pin(RX, py(i), lab, "R", "dead", "")
    elif lab in used_right:
        pin(RX, py(i), lab, "R", "used", used_right[lab][1])
    else:
        pin(RX, py(i), lab, "R", "free", "")

def wire(points, key, width=3):
    d = " ".join(f"{x},{y}" for x, y in points)
    add(f'<polyline points="{d}" fill="none" stroke="{C[key]}" stroke-width="{width}" '
        f'stroke-linejoin="round" stroke-linecap="round"/>')

def comp(x, y, w, h, title, lines, sub=None):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{C["box"]}" '
        f'stroke="{C["edge"]}" stroke-width="1.5"/>')
    add(f'<text x="{x+14}" y="{y+24}" font-size="16" font-weight="600" fill="{C["ink"]}">{title}</text>')
    if sub:
        add(f'<text x="{x+14}" y="{y+42}" font-size="12" fill="{C["muted"]}">{sub}</text>')
    for (ly, txt, key) in lines:
        col = C[key] if key else C["ink"]
        add(f'<text x="{x+w-14}" y="{ly+4}" text-anchor="end" font-size="13" '
            f'font-family="Consolas, Menlo, monospace" fill="{col}">{txt}</text>')

CX0, CX1 = 60, 360          # left components
# MPU9250: straight to GPIO15/16
comp(CX0, 236, CX1-CX0, 168, "MPU9250 IMU", [
    (py(7), "SCL  ◀", "i2c"), (py(8), "SDA  ◀▶", "i2c")], "I2C address 0x68")
add(f'<text x="{CX0+14}" y="{290+4}" font-size="13" fill="{C["ink"]}">VCC ← 3V3 (top-left pin)</text>')
add(f'<text x="{CX0+14}" y="{312+4}" font-size="13" fill="{C["ink"]}">GND ← GND · AD0 ← GND</text>')
add(f'<text x="{CX0+14}" y="{336}" font-size="12" fill="{C["muted"]}">BMP280 (optional): same 4 wires, addr 0x77</text>')
wire([(CX1, py(7)), (LX, py(7))], "i2c")
wire([(CX1, py(8)), (LX, py(8))], "i2c")
wire([(CX0+150, 236), (CX0+150, 214), (LX-60, 214), (LX-60, py(0)), (LX, py(0))], "pwr", 2)

# ZED-F9P: GPIO17 <- F9P TX, GPIO18 -> F9P RX
fy = 420
comp(CX0, fy, CX1-CX0, 104, "ZED-F9P RTK GPS", [
    (fy+52, "UART1 TX  ▶", "uart"), (fy+78, "UART1 RX  ◀", "uart")], "D_SEL open (UART mode)")
add(f'<text x="{CX0+14}" y="{fy+96}" font-size="12" fill="{C["muted"]}">VCC/GND from 5 V rail</text>')
wire([(LX, py(9)), (480, py(9)), (480, fy+52), (CX1, fy+52)], "uart")
wire([(LX, py(10)), (510, py(10)), (510, fy+78), (CX1, fy+78)], "uart")

# ESC: GPIO9
ey = 536
comp(CX0, ey, CX1-CX0, 74, "ESC (throttle)", [(ey+48, "signal  ◀", "pwm")], "BEC 5 V out → servo, receiver")
wire([(LX, py(14)), (480, py(14)), (480, ey+48), (CX1, ey+48)], "pwm")
# Servo: GPIO11
sy = 620
comp(CX0, sy, CX1-CX0, 74, "Steering servo", [(sy+48, "signal  ◀", "pwm")], "+ / − from ESC BEC")
wire([(LX, py(16)), (480, py(16)), (480, sy+48), (CX1, sy+48)], "pwm")
# Receiver: GPIO14
ry = 704
comp(CX0, ry, CX1-CX0, 74, "RC receiver", [(ry+48, "PPM/SBUS  ▶", "rc")], "+ / − from ESC BEC · signal ≤ 3.3 V")
wire([(LX, py(19)), (480, py(19)), (480, ry+48), (CX1, ry+48)], "rc")

# 5V / GND pins of the board -> power section
wire([(LX, py(20)), (530, py(20)), (530, 900)], "pwr")
wire([(LX, py(21)), (550, py(21)), (550, 900)], "gnd")

# right: LD14 LiDAR on GPIO41
DX0, DX1 = 960, 1260
comp(DX0, 266, DX1-DX0, 140, "LD14 / LD14P LiDAR", [], "115200 baud · free-running")
add(f'<text x="{DX0+14}" y="{py(6)+4}" font-size="13" font-family="Consolas, Menlo, monospace" '
    f'fill="{C["uart"]}">▶  TX</text>')
add(f'<text x="{DX0+14}" y="{py(6)+30}" font-size="13" fill="{C["muted"]}">RX: not connected</text>')
add(f'<text x="{DX0+14}" y="{py(6)+52}" font-size="13" fill="{C["ink"]}">VCC 5 V (~200 mA), GND</text>')
wire([(DX0, py(6)), (RX, py(6))], "uart")

# right: notes
ny = 470
add(f'<rect x="{DX0}" y="{ny}" width="{DX1-DX0}" height="190" rx="8" fill="#fff8c5" stroke="#d4a72c"/>')
notes = ["Do not use:",
         "• GPIO35-37: octal PSRAM on N8R8",
         "• GPIO43/44: console (UART port)",
         "• GPIO0, 3, 45, 46: strapping pins",
         "• GPIO19/20: native USB",
         "Analog pins 1-4 (BATT_VOLT_PIN):",
         "  GPIO5, 4, 2, 1 — max 3.1 V"]
for k, t in enumerate(notes):
    add(f'<text x="{DX0+14}" y="{ny+26+k*23}" font-size="13" '
        f'font-weight="{"600" if t.endswith(":") else "400"}" fill="{C["ink"]}">{t}</text>')

# power section
add(f'<text x="40" y="{850}" font-size="17" font-weight="600" fill="{C["ink"]}">Power</text>')
def pbox(x, y, w, t, s):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="58" rx="8" fill="{C["box"]}" stroke="{C["edge"]}" stroke-width="1.5"/>')
    add(f'<text x="{x+w/2}" y="{y+25}" text-anchor="middle" font-size="14" font-weight="600" fill="{C["ink"]}">{t}</text>')
    add(f'<text x="{x+w/2}" y="{y+44}" text-anchor="middle" font-size="12" fill="{C["muted"]}">{s}</text>')
def arrow(x1, y1, x2, y2, key="pwr"):
    add(f'<line x1="{x1}" y1="{y1}" x2="{x2-8}" y2="{y2}" stroke="{C[key]}" stroke-width="3"/>')
    add(f'<polygon points="{x2},{y2} {x2-10},{y2-6} {x2-10},{y2+6}" fill="{C[key]}"/>')
yy = 880
pbox(60, yy, 150, "Battery", "2S/3S LiPo")
arrow(210, yy+29, 240, yy+29)
pbox(240, yy, 220, "ESC", "BEC 5 V → servo + receiver")
add(f'<polyline points="135,{yy+58} 135,{yy+110} 585,{yy+110} 585,{yy+86}" fill="none" stroke="{C["pwr"]}" stroke-width="3"/>')
add(f'<polygon points="585,{yy+78} 579,{yy+90} 591,{yy+90}" fill="{C["pwr"]}"/>')
pbox(470, yy+20, 230, "5 V regulator (≥ 1 A)", "out → board 5V + GND pins")
add(f'<text x="524" y="{yy+14}" text-anchor="end" font-size="12" fill="{C["pwr"]}">5V</text>')
add(f'<text x="556" y="{yy+14}" font-size="12" fill="{C["gnd"]}">GND</text>')
arrow(700, yy+49, 760, yy+49)
pbox(760, yy+20, 250, "LiDAR VCC, F9P VCC", "same 5 V rail (or USB on the bench)")
add(f'<text x="60" y="{H-18}" font-size="14" font-weight="600" fill="{C["ink"]}">'
    f'All grounds joined: battery, ESC, servo, receiver, ESP32, IMU, LiDAR, GPS. '
    f'Never power servos from the ESP32.</text>')
add('</svg>')
open(sys.argv[1], 'w').write("\n".join(out) + "\n")
