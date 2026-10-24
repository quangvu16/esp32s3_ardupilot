#!/usr/bin/env python3
"""
Picture of a sitl_indoor.py run: the room, the true path and the EKF's path,
and the position error over time.

    python3 Tools/lidar_slam_test/plot_run.py /tmp/slam_sitl_xxxx/logs/00000001.BIN docs/slam_sitl.svg
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
from pymavlink import DFReader                      # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from sitl_indoor import ne                          # noqa: E402

# the room of SIM_ROOM=1 (libraries/SITL/SITL.cpp), north/east of the start
BOXES = [(-2.5, -1.8, 7.5, 6.2), (2.0, 1.8, 2.8, 2.6), (6.1, -1.4, 7.1, -0.6), (-2.1, 5.0, -1.3, 5.8)]
WALL = (4.0, 6.2, 4.0, 5.0)


def main(logfile, out):
    log = DFReader.DFReader_binary(logfile)
    sim, pos, slam = [], [], []
    origin, auto = None, None
    while True:
        m = log.recv_match(type=["SIM", "POS", "SLAM", "ORGN", "MODE"])
        if m is None:
            break
        t = m.TimeUS * 1e-6
        k = m.get_type()
        if k == "SIM":
            sim.append((t, m.Lat, m.Lng))
        elif k == "POS":
            pos.append((t, m.Lat, m.Lng))
        elif k == "SLAM":
            slam.append(t)
        elif k == "ORGN" and m.Type == 0 and origin is None:
            origin = (m.Lat, m.Lng)
        elif k == "MODE" and m.Mode == 10 and auto is None:
            auto = t

    def truth(t):
        lo, hi = 0, len(sim) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if sim[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        a, b = sim[max(lo - 1, 0)], sim[lo]
        f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0
        return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])

    start = truth(slam[0])
    tp, ep, et, ee = [], [], [], []
    for t, lat, lon in pos:
        if t < auto or t > sim[-1][0]:
            continue
        tn, te = ne(start[0], start[1], *truth(t))
        en, eE = ne(origin[0], origin[1], lat, lon)
        tp.append((te, tn))
        ep.append((eE, en))
        et.append(t - auto)
        ee.append(math.hypot(en - tn, eE - te) * 100)
    rms = math.sqrt(sum(e * e for e in ee) / len(ee))
    # every 5th sample is plenty for the picture (POS is logged at ~25 Hz)
    tp, ep, et, ee_plot = tp[::5], ep[::5], et[::5], ee[::5]

    plt.rcParams.update({"font.size": 10, "svg.fonttype": "none", "path.simplify": True})
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.6), gridspec_kw={"width_ratios": [1, 1.25]})
    for x0, y0, x1, y1 in BOXES:
        a1.fill([y0, y1, y1, y0], [x0, x0, x1, x1], fill=False, edgecolor="#444", lw=1.5)
    a1.plot([WALL[1], WALL[3]], [WALL[0], WALL[2]], color="#444", lw=1.5)
    a1.plot([p[0] for p in tp], [p[1] for p in tp], color="#2a9d55", lw=3, label="true path")
    a1.plot([p[0] for p in ep], [p[1] for p in ep], color="#d1495b", lw=1, label="EKF (IMU + LiDAR SLAM)")
    a1.plot(0, 0, "ko", ms=5)
    a1.annotate("start", (0, 0), (0.3, -0.6))
    a1.set_aspect("equal")
    a1.set_xlabel("east (m)")
    a1.set_ylabel("north (m)")
    a1.set_title("10 m x 8 m room, no GPS, no compass")
    a1.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=2, fontsize=8, frameon=False)
    a2.plot(et, ee_plot, color="#d1495b", lw=1)
    a2.axhline(rms, color="#555", ls="--", lw=1)
    a2.text(et[-1], rms + 0.6, "RMS %.0f cm" % rms, ha="right", color="#555")
    a2.set_ylim(0, max(30, max(ee) * 1.1))
    a2.set_xlabel("time in Auto (s)")
    a2.set_ylabel("position error (cm)")
    a2.set_title("EKF position against the truth")
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out)
    print("RMS %.1f cm, max %.1f cm -> %s" % (rms, max(ee), out))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
