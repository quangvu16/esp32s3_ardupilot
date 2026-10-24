#!/usr/bin/env python3
"""
SITL indoor test of the LiDAR SLAM: a rover with no GPS and no compass drives
laps of a simulated room in Auto, steering only by the EKF, whose position and
heading come from the LD06 SLAM (PRX1_SLAM). The EKF's position is then
compared with the simulator's true position from the log.

    ./waf configure --board sitl && ./waf rover
    python3 Tools/lidar_slam_test/sitl_indoor.py --laps 3

Exit code 0 when the errors are within the limits printed at the end.
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile
import time

from pymavlink import mavutil, DFReader

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HOME = (-35.363261, 149.165230, 584.0)

# the lap, in metres north and east of the start: around the box in the middle
LAP = [(5.0, 0.0), (5.0, 4.4), (0.0, 4.4), (0.0, 0.0)]


def offset(lat, lon, north, east):
    """lat/lon moved by north/east metres (small distances)"""
    r = 6378137.0
    return (lat + math.degrees(north / r),
            lon + math.degrees(east / (r * math.cos(math.radians(lat)))))


def ne(lat0, lon0, lat, lon):
    r = 6378137.0
    return (math.radians(lat - lat0) * r,
            math.radians(lon - lon0) * r * math.cos(math.radians(lat0)))


def wait_text(m, text, timeout):
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if msg is not None:
            print("  <", msg.text)
            if text in msg.text:
                return True
    return False


def upload_mission(m, items):
    m.mav.mission_clear_all_send(m.target_system, m.target_component)
    m.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    m.mav.mission_count_send(m.target_system, m.target_component, len(items))
    sent = set()
    end = time.time() + 30
    while time.time() < end:
        msg = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                           blocking=True, timeout=5)
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            if msg.type != 0:
                raise RuntimeError("mission refused: %d" % msg.type)
            return
        seq = msg.seq
        cmd, p1, p2, lat, lon = items[seq]
        m.mav.mission_item_int_send(m.target_system, m.target_component, seq,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    cmd, 0, 1, p1, p2, 0, 0,
                                    int(lat * 1e7), int(lon * 1e7), 0)
        sent.add(seq)
    raise RuntimeError("mission upload timed out")


def analyse(logfile):
    """EKF position (POS) against the simulator's truth (SIM), from the log.

    The SLAM map, and so the EKF's position, starts at (0, 0) where the vehicle
    was at the first scan, and the EKF origin is that point. The truth is taken
    relative to the same point. Only the time in Auto is scored."""
    log = DFReader.DFReader_binary(logfile)
    sim, pos, slam = [], [], []
    origin = None
    auto = []                     # (start, end) of Auto
    while True:
        msg = log.recv_match(type=["SIM", "POS", "SLAM", "ORGN", "MODE", "ATT"])
        if msg is None:
            break
        t = msg.TimeUS * 1e-6
        typ = msg.get_type()
        if typ == "SIM":
            sim.append((t, msg.Lat, msg.Lng, msg.Yaw))
        elif typ == "POS":
            pos.append((t, msg.Lat, msg.Lng))
        elif typ == "SLAM":
            slam.append((t, msg.M, msg.Q, msg.Drop))
        elif typ == "ORGN" and msg.Type == 0 and origin is None:
            origin = (msg.Lat, msg.Lng)
        elif typ == "MODE":
            if msg.Mode == 10:
                auto.append([t, 1e9])
            elif auto and auto[-1][1] == 1e9:
                auto[-1][1] = t
    if not sim or not pos or not slam or origin is None or not auto:
        raise RuntimeError("missing SIM/POS/SLAM/ORGN/MODE messages in %s" % logfile)

    def truth_at(t):
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

    start = truth_at(slam[0][0])
    errs = []
    for t, lat, lon in pos:
        if not any(a <= t <= b for a, b in auto) or t > sim[-1][0]:
            continue
        tn, te = ne(start[0], start[1], *truth_at(t))
        en, ee = ne(origin[0], origin[1], lat, lon)
        errs.append(math.hypot(en - tn, ee - te))
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    matched = sum(1 for s in slam if s[1]) / len(slam)
    return rms, max(errs), len(errs), matched, len(slam), slam[-1][3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--laps", type=int, default=3)
    ap.add_argument("--speedup", type=int, default=3)
    ap.add_argument("--binary", default=os.path.join(ROOT, "build", "sitl", "bin", "ardurover"))
    args = ap.parse_args()

    work = tempfile.mkdtemp(prefix="slam_sitl_")
    defaults = ",".join([os.path.join(ROOT, "Tools", "autotest", "default_params", "rover.parm"),
                         os.path.join(ROOT, "Tools", "lidar_slam_test", "sitl_indoor.parm")])
    env = dict(os.environ, SIM_ROOM="1")
    out = open(os.path.join(work, "sitl.txt"), "w")
    sitl = subprocess.Popen([args.binary, "-S", "--model", "rover", "--speedup", str(args.speedup),
                             "--home", "%f,%f,%f,0" % HOME, "--defaults", defaults,
                             "--serial5=sim:ld06", "-I0"],
                            cwd=work, env=env, stdout=out, stderr=subprocess.STDOUT)
    try:
        time.sleep(2)
        m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
        m.wait_heartbeat(timeout=30)
        print("connected; waiting for the SLAM and the EKF")
        if not wait_text(m, "SLAM: tracking", 60):
            raise RuntimeError("SLAM never started tracking")
        # the EKF needs a little while on the external navigation before it has a position
        end = time.time() + 60
        while time.time() < end:
            msg = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
            if msg is not None and msg.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS:
                break
        else:
            raise RuntimeError("EKF has no position")
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 0, 0)
        origin = m.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=True, timeout=10)
        if origin is None:
            raise RuntimeError("no EKF origin")
        lat0, lon0 = origin.latitude * 1e-7, origin.longitude * 1e-7
        print("EKF origin %.7f %.7f" % (lat0, lon0))

        # mission: home, the lap, then DO_JUMP back for the other laps
        items = [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, lat0, lon0)]
        for n, e in LAP:
            lat, lon = offset(lat0, lon0, n, e)
            items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, lat, lon))
        if args.laps > 1:
            items.append((mavutil.mavlink.MAV_CMD_DO_JUMP, 1, args.laps - 1, 0, 0))
        upload_mission(m, items)
        print("mission: %d laps of %s" % (args.laps, LAP))

        m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 10)
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
        lap_s = 2 * (5.0 + 4.4) / 0.6 + 8
        if not wait_text(m, "Mission Complete", args.laps * lap_s * 2.5 / args.speedup + 60):
            print("mission did not complete in time; analysing what there is")
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
        time.sleep(2)
    finally:
        sitl.terminate()
        sitl.wait()
        out.close()

    logs = sorted(f for f in os.listdir(os.path.join(work, "logs")) if f.endswith(".BIN"))
    logfile = os.path.join(work, "logs", logs[-1])
    rms, mx, n, matched, scans, dropped = analyse(logfile)
    print("\nlog %s" % logfile)
    print("%d SLAM scans, %.1f%% matched, %d skipped (matcher busy)" % (scans, matched * 100, dropped))
    print("EKF position against the truth: RMS %.3f m, max %.3f m (%d samples)" % (rms, mx, n))
    ok = rms < 0.15 and mx < 0.35 and matched > 0.9
    print("%s (limits: RMS < 0.15 m, max < 0.35 m, > 90%% of scans matched)" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
