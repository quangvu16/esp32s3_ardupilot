/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "LidarSlam.h"

#include <math.h>
#include <new>

#ifndef NEW_NOTHROW
#define NEW_NOTHROW new (std::nothrow)
#endif

// log-odds steps: a hit adds OCC, a pass-through subtracts FREE
static constexpr int16_t OCC_STEP = 24;
static constexpr int16_t FREE_STEP = 6;
static constexpr float LOGODDS_SCALE = 0.05f;   // p = 1 / (1 + exp(-l * scale))

static float wrap_pi(float a)
{
    while (a > float(M_PI)) {
        a -= 2.0f * float(M_PI);
    }
    while (a < -float(M_PI)) {
        a += 2.0f * float(M_PI);
    }
    return a;
}

LidarSlam::~LidarSlam()
{
    for (auto &lv : _level) {
        delete[] lv.cells;
        lv.cells = nullptr;
    }
}

bool LidarSlam::init(float cell_m, float size_m)
{
    if (ready()) {
        return true;
    }
    if (cell_m < 0.02f || size_m < 2.0f) {
        return false;
    }
    _size = size_m;
    for (uint8_t i = 0; i < LEVELS; i++) {
        Level &lv = _level[i];
        lv.res = cell_m * float(1U << i);
        const uint32_t w = uint32_t(size_m / lv.res + 0.5f);
        if (w < 8 || w > 4000) {
            return false;
        }
        lv.w = uint16_t(w);
        lv.cells = NEW_NOTHROW int8_t[w * w];
        if (lv.cells == nullptr) {
            for (auto &l2 : _level) {
                delete[] l2.cells;
                l2.cells = nullptr;
            }
            return false;
        }
    }
    reset();
    return true;
}

uint32_t LidarSlam::memory_bytes() const
{
    uint32_t b = 0;
    for (const auto &lv : _level) {
        b += uint32_t(lv.w) * lv.w;
    }
    return b;
}

void LidarSlam::reset(float yaw0)
{
    for (auto &lv : _level) {
        if (lv.cells != nullptr) {
            for (uint32_t k = 0; k < uint32_t(lv.w) * lv.w; k++) {
                lv.cells[k] = 0;
            }
        }
    }
    _scans = 0;
    _yaw0 = yaw0;
    _pose = {0, 0, yaw0};
    _last_mapped = _pose;
    _velocity = {0, 0, 0};
}

float LidarSlam::prob(int8_t l) const
{
    // a 256-entry table is faster on a microcontroller than expf per lookup
    static float table[256];
    static bool filled = false;
    if (!filled) {
        for (int16_t k = -128; k < 128; k++) {
            table[k + 128] = 1.0f / (1.0f + expf(-float(k) * LOGODDS_SCALE));
        }
        filled = true;
    }
    return table[int16_t(l) + 128];
}

float LidarSlam::sample(const Level &lv, float wx, float wy, float &dx, float &dy) const
{
    // continuous cell coordinates, cell centres at k + 0.5
    const float u = (wx + _size * 0.5f) / lv.res - 0.5f;
    const float v = (wy + _size * 0.5f) / lv.res - 0.5f;
    const int32_t i0 = int32_t(floorf(u));
    const int32_t j0 = int32_t(floorf(v));
    if (i0 < 0 || j0 < 0 || i0 + 1 >= lv.w || j0 + 1 >= lv.w) {
        dx = dy = 0.0f;
        return 0.0f;
    }
    const float fu = u - float(i0);
    const float fv = v - float(j0);
    const int8_t *c = lv.cells;
    const uint32_t w = lv.w;
    const float m00 = prob(c[uint32_t(i0) * w + uint32_t(j0)]);
    const float m10 = prob(c[uint32_t(i0 + 1) * w + uint32_t(j0)]);
    const float m01 = prob(c[uint32_t(i0) * w + uint32_t(j0 + 1)]);
    const float m11 = prob(c[uint32_t(i0 + 1) * w + uint32_t(j0 + 1)]);
    const float m = (1 - fu) * (1 - fv) * m00 + fu * (1 - fv) * m10 + (1 - fu) * fv * m01 + fu * fv * m11;
    dx = ((1 - fv) * (m10 - m00) + fv * (m11 - m01)) / lv.res;
    dy = ((1 - fu) * (m01 - m00) + fu * (m11 - m10)) / lv.res;
    return m;
}

void LidarSlam::match(const Level &lv, uint16_t n, Pose &p, const Pose &prior, uint8_t iterations) const
{
    for (uint8_t it = 0; it < iterations; it++) {
        const float c = cosf(p.yaw), s = sinf(p.yaw);
        float h00 = 0, h01 = 0, h02 = 0, h11 = 0, h12 = 0, h22 = 0;
        float g0 = 0, g1 = 0, g2 = 0;
        for (uint16_t k = 0; k < n; k++) {
            const float bx = _bx[k], by = _by[k];
            const float wx = p.x + c * bx - s * by;
            const float wy = p.y + s * bx + c * by;
            float dmx, dmy;
            const float m = sample(lv, wx, wy, dmx, dmy);
            const float r = 1.0f - m;
            const float j2 = dmx * (-s * bx - c * by) + dmy * (c * bx - s * by);
            h00 += dmx * dmx; h01 += dmx * dmy; h02 += dmx * j2;
            h11 += dmy * dmy; h12 += dmy * j2; h22 += j2 * j2;
            g0 += dmx * r; g1 += dmy * r; g2 += j2 * r;
        }
        // A weak pull towards the prediction keeps the solution defined where the
        // scan does not constrain it (a long corridor, a single wall).
        const float wt = 1e-3f * (h00 + h11) + 1.0f;
        const float wy = 2e-3f * h22 + 1.0f;
        h00 += wt; h11 += wt; h22 += wy;
        g0 += wt * (prior.x - p.x);
        g1 += wt * (prior.y - p.y);
        g2 += wy * wrap_pi(prior.yaw - p.yaw);
        // solve the symmetric 3x3 system H d = g (Cramer)
        const float a = h11 * h22 - h12 * h12;
        const float b = h02 * h12 - h01 * h22;
        const float cc = h01 * h12 - h02 * h11;
        const float det = h00 * a + h01 * b + h02 * cc;
        if (fabsf(det) < 1e-9f) {
            return;
        }
        const float e = h00 * h22 - h02 * h02;
        const float f = h01 * h02 - h00 * h12;
        const float i = h00 * h11 - h01 * h01;
        float d0 = (a * g0 + b * g1 + cc * g2) / det;
        float d1 = (b * g0 + e * g1 + f * g2) / det;
        float d2 = (cc * g0 + f * g1 + i * g2) / det;
        // no wild jumps from a poor linearisation
        const float step = sqrtf(d0 * d0 + d1 * d1);
        const float max_step = 2.0f * lv.res;
        if (step > max_step) {
            d0 *= max_step / step;
            d1 *= max_step / step;
        }
        if (d2 > 0.1f) { d2 = 0.1f; }
        if (d2 < -0.1f) { d2 = -0.1f; }
        p.x += d0;
        p.y += d1;
        p.yaw = wrap_pi(p.yaw + d2);
        if (fabsf(d0) + fabsf(d1) < 0.002f && fabsf(d2) < 0.001f) {
            return;
        }
    }
}

float LidarSlam::score(const Level &lv, uint16_t n, const Pose &p, uint16_t &inside) const
{
    const float c = cosf(p.yaw), s = sinf(p.yaw);
    uint16_t hits = 0;
    inside = 0;
    for (uint16_t k = 0; k < n; k++) {
        const float wx = p.x + c * _bx[k] - s * _by[k];
        const float wy = p.y + s * _bx[k] + c * _by[k];
        const int32_t i = int32_t(floorf((wx + _size * 0.5f) / lv.res));
        const int32_t j = int32_t(floorf((wy + _size * 0.5f) / lv.res));
        if (i < 1 || j < 1 || i + 1 >= lv.w || j + 1 >= lv.w) {
            continue;
        }
        // the cell or a neighbour is a known obstacle: the point agrees with the map
        int8_t best = -128;
        bool known = false;
        for (int32_t di = -1; di <= 1; di++) {
            for (int32_t dj = -1; dj <= 1; dj++) {
                const int8_t l = lv.cells[uint32_t(i + di) * lv.w + uint32_t(j + dj)];
                known |= (l != 0);
                if (l > best) {
                    best = l;
                }
            }
        }
        if (!known) {
            continue;
        }
        inside++;
        if (best > 20) {
            hits++;
        }
    }
    return inside > 0 ? float(hits) / float(inside) : 0.0f;
}

void LidarSlam::trace(Level &lv, float x0, float y0, float x1, float y1, bool hit)
{
    const float half = _size * 0.5f;
    int32_t i0 = int32_t(floorf((x0 + half) / lv.res));
    int32_t j0 = int32_t(floorf((y0 + half) / lv.res));
    const int32_t i1 = int32_t(floorf((x1 + half) / lv.res));
    const int32_t j1 = int32_t(floorf((y1 + half) / lv.res));
    const int32_t w = lv.w;
    if (i1 < 0 || j1 < 0 || i1 >= w || j1 >= w || i0 < 0 || j0 < 0 || i0 >= w || j0 >= w) {
        return;
    }
    // Bresenham: free cells up to the end point
    const int32_t di = (i1 > i0) ? (i1 - i0) : (i0 - i1);
    const int32_t dj = (j1 > j0) ? (j1 - j0) : (j0 - j1);
    const int32_t si = (i0 < i1) ? 1 : -1;
    const int32_t sj = (j0 < j1) ? 1 : -1;
    int32_t err = di - dj;
    while (i0 != i1 || j0 != j1) {
        // Leave the cells next to the end point alone: rays that graze a wall at a
        // shallow angle would otherwise wear the wall away.
        const int32_t near_i = (i1 > i0) ? (i1 - i0) : (i0 - i1);
        const int32_t near_j = (j1 > j0) ? (j1 - j0) : (j0 - j1);
        if (near_i > 1 || near_j > 1) {
            int8_t &cell = lv.cells[uint32_t(i0) * uint32_t(w) + uint32_t(j0)];
            const int16_t v = int16_t(cell) - FREE_STEP;
            cell = int8_t(v < -127 ? -127 : v);
        }
        const int32_t e2 = 2 * err;
        if (e2 > -dj) {
            err -= dj;
            i0 += si;
        }
        if (e2 < di) {
            err += di;
            j0 += sj;
        }
    }
    if (hit) {
        int8_t &cell = lv.cells[uint32_t(i1) * uint32_t(w) + uint32_t(j1)];
        const int16_t v = int16_t(cell) + OCC_STEP;
        cell = int8_t(v > 127 ? 127 : v);
    }
}

void LidarSlam::update_map(uint16_t n, const Pose &p, float sx, float sy)
{
    const float c = cosf(p.yaw), s = sinf(p.yaw);
    const float ox = p.x + c * sx - s * sy;     // the scanner in the map
    const float oy = p.y + s * sx + c * sy;
    for (auto &lv : _level) {
        for (uint16_t k = 0; k < n; k++) {
            const float wx = p.x + c * _bx[k] - s * _by[k];
            const float wy = p.y + s * _bx[k] + c * _by[k];
            trace(lv, ox, oy, wx, wy, true);
        }
    }
    _last_mapped = p;
}

LidarSlam::Result LidarSlam::process(const float *px, const float *py, uint16_t n, float dyaw,
                                     float sensor_x, float sensor_y)
{
    Result res {false, false, 0.0f, 0};
    if (!ready() || n == 0) {
        return res;
    }
    // keep at most MAX_POINTS, spread over the rotation; points to the vehicle frame
    const uint16_t stride = (n + MAX_POINTS - 1) / MAX_POINTS;
    uint16_t m = 0;
    for (uint16_t k = 0; k < n && m < MAX_POINTS; k += stride) {
        _bx[m] = px[k] + sensor_x;
        _by[m] = py[k] + sensor_y;
        m++;
    }
    res.used = m;

    if (_scans == 0) {
        _pose = {0, 0, _yaw0};
        update_map(m, _pose, sensor_x, sensor_y);
        _scans = 1;
        res.matched = true;
        res.mapped = true;
        res.quality = 1.0f;
        return res;
    }
    _scans++;

    // prediction: the gyro's turn, and the last scan's motion
    Pose prior {_pose.x + _velocity.x, _pose.y + _velocity.y, wrap_pi(_pose.yaw + dyaw)};
    Pose p = prior;
    match(_level[2], m, p, prior, 5);
    match(_level[1], m, p, prior, 5);
    match(_level[0], m, p, prior, 8);
    uint16_t inside = 0;
    res.quality = score(_level[0], m, p, inside);

    if (res.quality >= min_quality && inside >= 20) {
        res.matched = true;
        // damped motion for the next prediction; a scan is ~0.17 s apart
        _velocity.x = 0.7f * (p.x - _pose.x);
        _velocity.y = 0.7f * (p.y - _pose.y);
        _pose = p;
        const float moved = sqrtf((p.x - _last_mapped.x) * (p.x - _last_mapped.x) + (p.y - _last_mapped.y) * (p.y - _last_mapped.y));
        if (moved > map_update_dist || fabsf(wrap_pi(p.yaw - _last_mapped.yaw)) > map_update_yaw) {
            update_map(m, p, sensor_x, sensor_y);
            res.mapped = true;
        }
    } else {
        // not trusted: carry on with the prediction, and don't spoil the map
        _pose = prior;
        _velocity.x *= 0.5f;
        _velocity.y *= 0.5f;
    }
    return res;
}
