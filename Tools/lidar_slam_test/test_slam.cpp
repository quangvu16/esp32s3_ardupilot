/*
  Offline test of libraries/AP_Proximity/LidarSlam: a car drives laps in a
  simulated room while an LD14-like scanner (about 2300 points/s, 6 rotations/s,
  8 m) sees the walls and some boxes. The points are taken while the car moves,
  with range noise and dropouts; the gyro has a bias. The SLAM pose is compared
  with the truth.

    g++ -O2 -std=c++17 -I../../libraries/AP_Proximity test_slam.cpp \
        ../../libraries/AP_Proximity/LidarSlam.cpp -o test_slam && ./test_slam

  Exit code 0 when the errors are within the limits printed at the end.
*/
#include "LidarSlam.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

struct Seg { double x1, y1, x2, y2; };

static std::vector<Seg> room()
{
    std::vector<Seg> w;
    auto box = [&](double x0, double y0, double x1, double y1) {
        w.push_back({x0, y0, x1, y0}); w.push_back({x1, y0, x1, y1});
        w.push_back({x1, y1, x0, y1}); w.push_back({x0, y1, x0, y0});
    };
    box(-2.0, -3.0, 8.0, 5.0);          // the room: 10 m x 8 m (x north, y east)
    box(2.5, 0.6, 3.3, 1.4);            // a box in the middle of the loop
    box(6.6, -2.6, 7.6, -1.8);          // a cupboard in a corner
    box(-1.6, 3.8, -0.8, 4.6);          // a chair
    w.push_back({4.5, 5.0, 4.5, 3.8});  // a short wall sticking out
    return w;
}

// distance along a ray to the nearest wall, or -1
static double cast(const std::vector<Seg> &walls, double ox, double oy, double a)
{
    const double dx = cos(a), dy = sin(a);
    double best = -1;
    for (const auto &s : walls) {
        const double ex = s.x2 - s.x1, ey = s.y2 - s.y1;
        const double den = dx * ey - dy * ex;
        if (fabs(den) < 1e-12) {
            continue;
        }
        const double t = ((s.x1 - ox) * ey - (s.y1 - oy) * ex) / den;
        const double u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / den;
        if (t > 0 && u >= 0 && u <= 1 && (best < 0 || t < best)) {
            best = t;
        }
    }
    return best;
}

// the path: laps of a rounded rectangle around the middle box, 0.5 m/s
struct State { double x, y, yaw, v, w; };

static State path(double t)
{
    const double v = 0.5;
    const double L1 = 5.0, L2 = 3.0, r = 0.8;          // straight lengths, corner radius
    const double arc = M_PI / 2 * r;
    const double lap = 2 * (L1 + L2) + 4 * arc;
    double s = fmod(t * v, lap);
    double x = 0.5, y = -1.2, yaw = 0;                // start, heading north
    const double seg[8] = {L1, arc, L2, arc, L1, arc, L2, arc};
    double w = 0;
    for (int k = 0; k < 8; k++) {
        const double len = seg[k];
        const double d = s < len ? s : len;
        if (k % 2 == 0) {
            x += d * cos(yaw); y += d * sin(yaw);
            w = 0;
        } else {
            // a right turn (clockwise) of radius r
            const double cx = x - r * sin(yaw), cy = y + r * cos(yaw);
            const double a = d / r;
            const double y1 = yaw + a;
            x = cx + r * sin(y1); y = cy - r * cos(y1);
            yaw = y1;
            w = v / r;
        }
        s -= d;
        if (s <= 0) {
            break;
        }
    }
    return {x, y, yaw, v, w};
}

int main(int argc, char **argv)
{
    const double duration = argc > 1 ? atof(argv[1]) : 180.0;     // s
    const auto walls = room();
    std::mt19937 rng(42);
    std::normal_distribution<double> N(0, 1);
    std::uniform_real_distribution<double> U(0, 1);

    LidarSlam slam;
    const float cell = getenv("CELL") ? float(atof(getenv("CELL"))) : 0.10f;
    const float side = getenv("SIDE") ? float(atof(getenv("SIDE"))) : 20.0f;
    if (!slam.init(cell, side)) {
        printf("init failed\n");
        return 2;
    }
    printf("map memory %u bytes\n", unsigned(slam.memory_bytes()));

    const double rot_hz = 6.0, pts_per_s = 2300.0;
    const double gyro_bias = 0.3 * M_PI / 180;               // 0.3 deg/s
    std::vector<float> px, py;
    double t = 0, scanner_angle = 0;
    double gyro_yaw = 0, gyro_yaw_last = 0;
    // forward speed estimate for the deskew: from the SLAM's own last two poses, as the
    // firmware takes it from the EKF (which the SLAM feeds)
    double speed_est = 0, last_px = 0, last_py = 0;
    State s0 = path(0);           // replaced by the pose at the first scan: the map starts there
    double err_sum2 = 0, err_max = 0, yaw_max = 0;
    double along_sum = 0, cross_sum = 0, along_straight = 0;
    int straight_n = 0;
    int scans = 0, matched = 0;
    std::vector<double> pt_t;                                // time of each point in the rotation
    std::vector<double> pt_a, pt_r;

    const double dt = 1.0 / pts_per_s;
    while (t < duration) {
        const State st = path(t);
        gyro_yaw += (st.w + gyro_bias + 0.002 * N(rng)) * dt;
        // one LiDAR point (the scanner turns clockwise, like the LD14)
        scanner_angle += 2 * M_PI * rot_hz * dt;
        const double a_body = fmod(scanner_angle, 2 * M_PI);
        double r = cast(walls, st.x, st.y, st.yaw + a_body);
        if (r > 0.05 && r < 8.0 && U(rng) > 0.05) {
            r += N(rng) * (0.012 + 0.01 * r);
            pt_t.push_back(t); pt_a.push_back(a_body); pt_r.push_back(r);
        }
        t += dt;
        if (scanner_angle >= 2 * M_PI) {
            scanner_angle -= 2 * M_PI;
            // end of a rotation: deskew the heading with the gyro (as the firmware does),
            // points in the body frame at the end time
            const double rate = (gyro_yaw - gyro_yaw_last) * rot_hz;   // rad/s over the rotation
            px.clear(); py.clear();
            for (size_t k = 0; k < pt_t.size(); k++) {
                const double ago = t - pt_t[k];
                const double back = ago * rate;                        // how much less turned then
                const double a = pt_a[k] - back;
                // and how far behind the scanner was (forward speed only: a car doesn't slide)
                px.push_back(float(pt_r[k] * cos(a) - speed_est * ago));
                py.push_back(float(pt_r[k] * sin(a)));
            }
            pt_t.clear(); pt_a.clear(); pt_r.clear();
            const float dyaw = float(gyro_yaw - gyro_yaw_last);
            gyro_yaw_last = gyro_yaw;
            const auto res = slam.process(px.data(), py.data(), uint16_t(px.size()), dyaw);
            if (scans == 0) {
                s0 = path(t);
            }
            scans++;
            matched += res.matched ? 1 : 0;
            {
                const auto &q = slam.pose();
                const double d = hypot(q.x - last_px, q.y - last_py) * rot_hz;
                speed_est = res.matched ? 0.5 * speed_est + 0.5 * d : speed_est;
                last_px = q.x; last_py = q.y;
            }
            // truth in the map frame: relative to the start pose
            const State now = path(t);
            const double c = cos(-s0.yaw), s = sin(-s0.yaw);
            const double tx = c * (now.x - s0.x) - s * (now.y - s0.y);
            const double ty = s * (now.x - s0.x) + c * (now.y - s0.y);
            const auto &p = slam.pose();
            const double e = hypot(p.x - tx, p.y - ty);
            double ye = fabs(remainder(p.yaw - (now.yaw - s0.yaw), 2 * M_PI));
            err_sum2 += e * e;
            {
                const double hy = now.yaw - s0.yaw;
                along_sum += (p.x - tx) * cos(hy) + (p.y - ty) * sin(hy);
                cross_sum += -(p.x - tx) * sin(hy) + (p.y - ty) * cos(hy);
                if (now.w == 0) { straight_n++; along_straight += (p.x - tx) * cos(hy) + (p.y - ty) * sin(hy); }
            }
            if (e > err_max) { err_max = e; }
            if (ye > yaw_max) { yaw_max = ye; }
            if (scans % 120 == 0) {
                printf("t=%6.1fs  truth %6.2f %6.2f  slam %6.2f %6.2f  err %.3f m  yaw err %.2f deg  q %.2f  pts %u\n",
                       t, tx, ty, p.x, p.y, e, ye * 180 / M_PI, res.quality, unsigned(res.used));
            }
        }
    }
    const double rms = sqrt(err_sum2 / scans);
    const double laps = duration * 0.5 / (2 * (5.0 + 3.0) + 4 * M_PI / 2 * 0.8);
    const double drift_gyro = gyro_bias * duration * 180 / M_PI;
    printf("\n%d scans (%.1f laps, %.0f s), %d matched (%.1f%%)\n", scans, laps, duration, matched, 100.0 * matched / scans);
    printf("position error: RMS %.3f m, max %.3f m;  heading error max %.2f deg\n", rms, err_max, yaw_max * 180 / M_PI);
    printf("mean error along the travel %.3f m (on straights %.3f m), sideways %.3f m\n", along_sum / scans, along_straight / (straight_n ? straight_n : 1), cross_sum / scans);
    printf("(the gyro alone would have drifted %.0f deg in heading)\n", drift_gyro);
    const bool ok = rms < 0.10 && err_max < 0.20 && yaw_max * 180 / M_PI < 3.0;
    printf("%s (limits: RMS < 0.10 m, max < 0.20 m, heading < 3 deg)\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
