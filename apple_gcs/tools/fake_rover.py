#!/usr/bin/env python3
"""A pretend ArduRover for trying the ground station apps without the car.

Python 3 standard library only. Listens on TCP 127.0.0.1:5760 like the board and
speaks enough MAVLink 2 for every tab: heartbeat, attitude, position, GPS (RTK
fixed), HUD, battery, home, RC channels, an LD14-style LiDAR scan (OBSTACLE_DISTANCE)
with a few posts around the car, parameters (list / set), arm / disarm, modes,
MANUAL_CONTROL driving, Guided targets, and mission and fence upload / download
with Auto following the waypoints.

    python3 fake_rover.py            then connect the app to TCP 127.0.0.1:5760
    python3 fake_rover.py --port 5761 --lat 10.7743 --lon 106.6920

It is a toy model (a car with simple kinematics), not SITL.
"""
import argparse, math, socket, struct, time

CRC_EXTRA = {0: 50, 1: 124, 20: 214, 21: 159, 22: 220, 23: 168, 24: 24, 30: 39, 33: 104, 62: 183,
             40: 230, 41: 28, 42: 28, 43: 132, 44: 221, 45: 232, 47: 153, 51: 196, 65: 118, 66: 148,
             69: 243, 73: 38, 74: 20, 75: 158, 76: 152, 77: 143, 86: 5, 233: 35, 242: 104, 253: 83,
             330: 23}

def x25(data, extra):
    crc = 0xFFFF
    for b in bytes(data) + bytes([extra]):
        tmp = (b ^ crc) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

class Link:
    def __init__(self, conn):
        self.c = conn; self.seq = 0; self.buf = b""

    def send(self, msgid, payload):
        n = len(payload)
        while n > 1 and payload[n - 1] == 0:
            n -= 1
        payload = payload[:n]
        hdr = struct.pack("<BBBBBBBHB", 0xFD, n, 0, 0, self.seq, 1, 1, msgid & 0xFFFF, msgid >> 16)
        self.seq = (self.seq + 1) & 0xFF
        crc = x25(hdr[1:] + payload, CRC_EXTRA[msgid])
        try:
            self.c.sendall(hdr + payload + struct.pack("<H", crc))
        except OSError:
            pass

    def recv(self):
        """Yields (msgid, payload) of complete frames received so far."""
        try:
            d = self.c.recv(4096)
            if d == b"":
                raise ConnectionError("closed")
        except BlockingIOError:
            d = b""
        self.buf += d
        out = []
        while True:
            i = min([p for p in (self.buf.find(b"\xfd"), self.buf.find(b"\xfe")) if p >= 0], default=-1)
            if i < 0:
                self.buf = b""; break
            self.buf = self.buf[i:]
            if len(self.buf) < 3:
                break
            v2 = self.buf[0] == 0xFD
            hl = 10 if v2 else 6
            plen = self.buf[1]
            total = hl + plen + 2 + (13 if v2 and self.buf[2] & 1 else 0)
            if len(self.buf) < total:
                break
            frame = self.buf[:total]
            msgid = (frame[7] | frame[8] << 8 | frame[9] << 16) if v2 else frame[5]
            self.buf = self.buf[total:]
            if msgid not in CRC_EXTRA:
                continue
            if x25(frame[1:hl + plen], CRC_EXTRA[msgid]) != struct.unpack("<H", frame[hl + plen:hl + plen + 2])[0]:
                continue
            out.append((msgid, frame[hl:hl + plen] + bytes(64)))   # zero-padded
        return out

PARAMS = {
    "FRAME_CLASS": 1, "SYSID_MYGCS": 255, "CRUISE_SPEED": 2, "CRUISE_THROTTLE": 40, "WP_SPEED": 2,
    "WP_RADIUS": 2, "TURN_RADIUS": 0.9, "FS_THR_ENABLE": 1, "FS_GCS_ENABLE": 2, "FS_ACTION": 2,
    "FS_TIMEOUT": 1.5, "PRX1_TYPE": 19, "PRX1_YAW_CORR": 0, "AVOID_ENABLE": 0, "AVOID_MARGIN": 2,
    "OA_TYPE": 0, "FENCE_ENABLE": 0, "FENCE_TYPE": 6, "FENCE_ACTION": 1, "SERVO1_TRIM": 1500,
    "SERVO3_TRIM": 1500, "GPS1_TYPE": 2, "RC_OVERRIDE_TIME": 3,
}
# Rover modes that take stick overrides
STICK_MODES = (0, 1, 3)

class Rover:
    def __init__(self, lat, lon):
        self.home = (lat, lon)
        self.lat, self.lon = lat, lon
        self.yaw = 0.0          # rad, 0 = north
        self.speed = 0.0
        self.armed = False
        self.mode = 0
        self.steer = self.thr = 0.0
        self.stick_time = 0.0
        self.guided = None
        self.mission = [(lat, lon)]          # item 0 = home
        self.mission_raw = [None]
        self.fence_raw = []
        self.seq = 1
        # posts around home, metres north/east: what the LiDAR sees
        self.posts = [(4, 1), (3, -3), (-2, 3), (6, -1)]

    def xy(self, lat, lon):
        return ((lat - self.home[0]) * 111320.0, (lon - self.home[1]) * 111320.0 * math.cos(math.radians(self.home[0])))

    def step(self, dt, now):
        target_speed, target_yawrate = 0.0, 0.0
        if self.armed:
            if self.mode in STICK_MODES and now - self.stick_time < PARAMS["RC_OVERRIDE_TIME"]:
                target_speed = self.thr * 3.0
                target_yawrate = self.steer * 1.5 * (1 if target_speed >= 0 else -1)
            elif self.mode == 15 and self.guided:
                target_speed, target_yawrate = self.drive_to(self.guided)
            elif self.mode == 10 and self.seq < len(self.mission):
                target_speed, target_yawrate = self.drive_to(self.mission[self.seq])
                if self.dist(self.mission[self.seq]) < PARAMS["WP_RADIUS"]:
                    self.seq += 1
                    if self.seq >= len(self.mission):
                        self.mode = 4
            elif self.mode == 11:
                target_speed, target_yawrate = self.drive_to(self.home)
                if self.dist(self.home) < 1:
                    self.mode = 4
        self.speed += max(-2 * dt, min(2 * dt, target_speed - self.speed))
        self.yaw = (self.yaw + target_yawrate * dt * min(1.0, abs(self.speed) / 0.5 + 0.2)) % (2 * math.pi)
        n, e = self.xy(self.lat, self.lon)
        n += math.cos(self.yaw) * self.speed * dt
        e += math.sin(self.yaw) * self.speed * dt
        self.lat = self.home[0] + n / 111320.0
        self.lon = self.home[1] + e / (111320.0 * math.cos(math.radians(self.home[0])))

    def dist(self, p):
        n1, e1 = self.xy(self.lat, self.lon); n2, e2 = self.xy(*p)
        return math.hypot(n2 - n1, e2 - e1)

    def drive_to(self, p):
        n1, e1 = self.xy(self.lat, self.lon); n2, e2 = self.xy(*p)
        bearing = math.atan2(e2 - e1, n2 - n1)
        err = (bearing - self.yaw + math.pi) % (2 * math.pi) - math.pi
        d = math.hypot(n2 - n1, e2 - e1)
        return min(PARAMS["WP_SPEED"], d), max(-1.5, min(1.5, 2 * err))

    def scan_cm(self):
        """72 bins of 5° clockwise from the nose: nearest post in cm, 801 = clear."""
        n0, e0 = self.xy(self.lat, self.lon)
        out = [801] * 72
        for pn, pe in self.posts:
            dn, de = pn - n0, pe - e0
            d = math.hypot(dn, de) - 0.15
            if d > 8 or d < 0.05:
                continue
            rel = (math.degrees(math.atan2(de, dn) - self.yaw)) % 360
            span = max(1, int(math.degrees(math.atan2(0.15, max(d, 0.1))) / 5) + 1)
            b = int(rel / 5)
            for k in range(-span + 1, span):
                out[(b + k) % 72] = min(out[(b + k) % 72], int(d * 100))
        return out


def serve(port, lat, lon):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    print(f"Fake rover on TCP 127.0.0.1:{port}, home {lat:.6f}, {lon:.6f}. Ctrl+C to stop.")
    rover = Rover(lat, lon)
    while True:
        conn, addr = srv.accept()
        print("ground station connected:", addr)
        conn.setblocking(False)
        try:
            session(Link(conn), rover)
        except (ConnectionError, OSError) as e:
            print("ground station left:", e)
        conn.close()


def text(link, msg, sev=6):
    link.send(253, struct.pack("<B50s", sev, msg.encode()[:50]))


def session(link, r):
    t0 = time.monotonic()
    last = {"hb": 0, "fast": 0, "slow": 0, "scan": 0}
    upload = None        # (mission_type, count, items)
    dl_type = 0
    text(link, "Fake rover ready (not a real vehicle)")
    while True:
        now = time.monotonic()
        boot_ms = int((now - t0) * 1000) & 0xFFFFFFFF
        for msgid, p in link.recv():
            if msgid == 21:                                   # PARAM_REQUEST_LIST
                for i, (k, v) in enumerate(PARAMS.items()):
                    link.send(22, struct.pack("<fHH16sB", v, len(PARAMS), i, k.encode(), 9))
            elif msgid == 20:                                 # PARAM_REQUEST_READ
                idx = struct.unpack_from("<h", p, 0)[0]
                if 0 <= idx < len(PARAMS):
                    k = list(PARAMS)[idx]
                    link.send(22, struct.pack("<fHH16sB", PARAMS[k], len(PARAMS), idx, k.encode(), 9))
            elif msgid == 23:                                 # PARAM_SET
                v = struct.unpack_from("<f", p, 0)[0]
                k = p[6:22].split(b"\0")[0].decode()
                if k in PARAMS:
                    PARAMS[k] = v
                    link.send(22, struct.pack("<fHH16sB", v, len(PARAMS), list(PARAMS).index(k), k.encode(), 9))
            elif msgid == 76:                                 # COMMAND_LONG
                prm = struct.unpack_from("<7f", p, 0); cmd = struct.unpack_from("<H", p, 28)[0]
                res = 0
                if cmd == 400:
                    r.armed = prm[0] > 0.5
                    text(link, "Armed" if r.armed else "Disarmed", 5)
                elif cmd == 176:
                    r.mode = int(prm[1])
                    if r.mode == 10:
                        r.seq = 1
                elif cmd == 300:
                    r.mode = 10; r.seq = 1
                elif cmd == 246:
                    text(link, "Reboot requested (ignored by the fake rover)", 4)
                elif cmd == 179:
                    r.home = (r.lat, r.lon) if prm[0] > 0.5 else r.home
                link.send(77, struct.pack("<HB", cmd, res))
            elif msgid == 75:                                 # COMMAND_INT (set home)
                cmd = struct.unpack_from("<H", p, 28)[0]
                x, y = struct.unpack_from("<ii", p, 16)
                if cmd == 179:
                    r.home = (x / 1e7, y / 1e7)
                link.send(77, struct.pack("<HB", cmd, 0))
            elif msgid == 69:                                 # MANUAL_CONTROL
                x, y, z, rr = struct.unpack_from("<hhhh", p, 0)
                r.steer, r.thr = y / 1000.0, z / 1000.0
                r.stick_time = now
            elif msgid == 86:                                 # SET_POSITION_TARGET_GLOBAL_INT
                la, lo = struct.unpack_from("<ii", p, 4)
                r.guided = (la / 1e7, lo / 1e7)
            elif msgid == 41:                                 # MISSION_SET_CURRENT
                r.seq = struct.unpack_from("<H", p, 0)[0]
            elif msgid == 44:                                 # MISSION_COUNT: an upload starts
                count = struct.unpack_from("<H", p, 0)[0]; mtype = p[4]
                upload = (mtype, count, [])
                if count == 0:
                    if mtype == 1: r.fence_raw = []
                    link.send(47, struct.pack("<BBBB", 255, 190, 0, mtype)); upload = None
                else:
                    link.send(51, struct.pack("<HBBB", 0, 255, 190, mtype))
            elif msgid == 73 and upload:                      # MISSION_ITEM_INT
                mtype, count, items = upload
                seq = struct.unpack_from("<H", p, 28)[0]
                if seq == len(items):
                    items.append(bytes(p[:38]))
                    if len(items) < count:
                        link.send(51, struct.pack("<HBBB", len(items), 255, 190, mtype))
                    else:
                        if mtype == 0:
                            r.mission_raw = items
                            r.mission = []
                            for it in items:
                                cmd = struct.unpack_from("<H", it, 30)[0]
                                x, y = struct.unpack_from("<ii", it, 16)
                                r.mission.append((x / 1e7, y / 1e7) if cmd == 16 else r.mission[-1] if r.mission else r.home)
                            text(link, f"Mission: {count - 1} items received")
                        else:
                            r.fence_raw = items
                            text(link, f"Fence: {count} items received")
                        link.send(47, struct.pack("<BBBB", 255, 190, 0, mtype))
                        upload = None
            elif msgid == 43:                                 # MISSION_REQUEST_LIST: a download
                dl_type = p[2]
                raw = r.fence_raw if dl_type == 1 else [it for it in r.mission_raw if it]
                if dl_type == 0 and not raw:
                    raw = []
                link.send(44, struct.pack("<HBBB", len(raw) if (dl_type == 1 or raw) else 1, 255, 190, dl_type))
            elif msgid in (51, 40):                           # MISSION_REQUEST(_INT)
                seq = struct.unpack_from("<H", p, 0)[0]
                raw = r.fence_raw if dl_type == 1 else [it for it in r.mission_raw if it]
                if seq < len(raw):
                    it = bytearray(raw[seq]); it[37] = dl_type
                    link.send(73, bytes(it))
                elif dl_type == 0 and seq == 0:
                    link.send(73, struct.pack("<4fiifHHBBBBB", 0, 0, 0, 0, int(r.home[0] * 1e7), int(r.home[1] * 1e7), 0, 0, 16, 255, 190, 0, 1, 1) + b"\0")

        dt = 0.1
        if now - last["fast"] >= dt:
            r.step(now - last["fast"] if last["fast"] else dt, now)
            last["fast"] = now
            yawrate = 0.0
            link.send(30, struct.pack("<I6f", boot_ms, 0.0, 0.0, (r.yaw + math.pi) % (2 * math.pi) - math.pi, 0, 0, yawrate))
            vn, ve = r.speed * math.cos(r.yaw), r.speed * math.sin(r.yaw)
            link.send(33, struct.pack("<IiiiihhhH", boot_ms, int(r.lat * 1e7), int(r.lon * 1e7), 10000, 0,
                                      int(vn * 100), int(ve * 100), 0, int(math.degrees(r.yaw) * 100) % 36000))
            link.send(74, struct.pack("<ffffhH", abs(r.speed), abs(r.speed), 0, 0, int(math.degrees(r.yaw)) % 360,
                                      int(abs(r.thr) * 100) if r.mode in STICK_MODES else int(min(100, abs(r.speed) * 30))))
            wp = r.mission[r.seq] if (r.mode == 10 and r.seq < len(r.mission)) else None
            link.send(42, struct.pack("<H", r.seq))
            if wp:
                link.send(62, struct.pack("<fffffhhH", 0, 0, 0, 0, 0, 0, 0, int(r.dist(wp))))
        if now - last["scan"] >= 0.2:
            last["scan"] = now
            link.send(330, struct.pack("<Q72HHHBBffB", int(now * 1e6), *r.scan_cm(), 5, 800, 0, 5, 5.0, 0.0, 12))
        if now - last["hb"] >= 1.0:
            last["hb"] = now
            base = 1 | (128 if r.armed else 0)
            link.send(0, struct.pack("<IBBBBB", r.mode, 10, 3, base, 4, 3))
            link.send(1, struct.pack("<IIIHHhHHHHHHb", 0x2000000F, 0x2000000F, 0x2000000F, 300, 12400, 150,
                                     0, 0, 0, 0, 0, 0, 82))
            link.send(24, struct.pack("<QiiiHHHHBBiIIII", int(now * 1e6), int(r.lat * 1e7), int(r.lon * 1e7), 10000,
                                      60, 90, int(abs(r.speed) * 100), 0, 6, 22, 10000, 14, 20, 0, 0))
            link.send(242, struct.pack("<iii3f4f3fQ", int(r.home[0] * 1e7), int(r.home[1] * 1e7), 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0))
            rc = [1500] * 18
            rc[2] = int(1500 + r.thr * 500); rc[0] = int(1500 + r.steer * 500); rc[7] = 1100
            link.send(65, struct.pack("<I18HBB", boot_ms, *rc, 8, 255))
        time.sleep(0.02)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=5760)
    ap.add_argument("--lat", type=float, default=10.7743)      # Tao Dan Park, Ho Chi Minh City
    ap.add_argument("--lon", type=float, default=106.6920)
    a = ap.parse_args()
    try:
        serve(a.port, a.lat, a.lon)
    except KeyboardInterrupt:
        print("\nstopped")
