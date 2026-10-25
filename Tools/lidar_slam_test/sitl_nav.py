#!/usr/bin/env python3
"""
SITL test of the apps' indoor navigation (android_gcs/core .../nav): starts SITL
with the room (SIM_ROOM=1), no GPS and no compass, runs the Kotlin harness
NavSitlTest (map the room, save, reboot, load, go to B, patrol), then compares
the app's map pose with the simulator's truth from the log.

    ./waf configure --board sitl && ./waf rover
    python3 Tools/lidar_slam_test/sitl_nav.py --classpath <core.jar>:<test.jar>:<kotlin-stdlib.jar>

Exit code 0 when the harness passes and the pose error is within the limits.
"""
import argparse
import csv
import math
import os
import subprocess
import sys
import tempfile
import time

from pymavlink import DFReader

sys.path.insert(0, os.path.dirname(__file__))
from sitl_indoor import HOME, ROOT, ne        # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--classpath", required=True)
    ap.add_argument("--main", default="org.ardupilot.esp32gcs.nav.NavSitlTestKt",
                    help="harness class (org.ardupilot.esp32gcs.desktop.IndoorShotsKt also saves screenshots)")
    ap.add_argument("--binary", default=os.path.join(ROOT, "build", "sitl", "bin", "ardurover"))
    args = ap.parse_args()

    work = tempfile.mkdtemp(prefix="nav_sitl_")
    defaults = ",".join([os.path.join(ROOT, "Tools", "autotest", "default_params", "rover.parm"),
                         os.path.join(ROOT, "Tools", "lidar_slam_test", "sitl_indoor.parm")])
    out = open(os.path.join(work, "sitl.txt"), "w")
    sitl = subprocess.Popen([args.binary, "-S", "--model", "rover", "--speedup", "1",
                             "--home", "%f,%f,%f,0" % HOME, "--defaults", defaults,
                             "--serial5=sim:ld06", "-I0"],
                            cwd=work, env=dict(os.environ, SIM_ROOM="1"), stdout=out, stderr=subprocess.STDOUT)
    try:
        time.sleep(2)
        r = subprocess.run(["java", "-cp", args.classpath, args.main, work],
                           timeout=900)
        harness_ok = r.returncode == 0
    finally:
        sitl.terminate()
        sitl.wait()
        out.close()

    # The map's frame is the first boot's SLAM frame: (0, 0) where the car was when
    # the SLAM started (SITL's rover rolls a little at start-up, so not quite home).
    logs = sorted(f for f in os.listdir(os.path.join(work, "logs")) if f.endswith(".BIN"))
    first = DFReader.DFReader_binary(os.path.join(work, "logs", logs[0]))
    origin, last_sim = None, None
    while origin is None:
        m = first.recv_match(type=["SIM", "SLAM"])
        if m is None:
            break
        if m.get_type() == "SIM":
            last_sim = (m.Lat, m.Lng)
        elif last_sim is not None:
            origin = last_sim
    origin = origin or HOME[:2]
    # the app's pose against the truth, by the autopilot's clock, in the second boot's log
    log = DFReader.DFReader_binary(os.path.join(work, "logs", logs[-1]))
    sim = []
    while True:
        m = log.recv_match(type=["SIM"])
        if m is None:
            break
        sim.append((m.TimeUS / 1000.0, m.Lat, m.Lng))
    rows = list(csv.DictReader(open(os.path.join(work, "poses.csv"))))
    errs = []
    j = 0
    for row in rows:
        t = float(row["boot_ms"])
        while j + 1 < len(sim) and sim[j + 1][0] < t:
            j += 1
        if j + 1 >= len(sim) or sim[j][0] > t:
            continue
        tn, te = ne(origin[0], origin[1], sim[j][1], sim[j][2])
        errs.append(math.hypot(float(row["x"]) - tn, float(row["y"]) - te))
    if not errs:
        print("no poses to compare")
        return 1
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    print("\nwork dir %s" % work)
    print("app pose in the saved map against the truth: RMS %.3f m, max %.3f m (%d samples)" % (rms, max(errs), len(errs)))
    ok = harness_ok and rms < 0.2 and max(errs) < 0.5
    print("%s (harness %s; limits RMS < 0.2 m, max < 0.5 m)" % ("PASS" if ok else "FAIL", "ok" if harness_ok else "failed"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
