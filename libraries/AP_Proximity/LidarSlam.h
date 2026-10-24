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

/*
  A small 2D LiDAR SLAM for a ground vehicle without GPS: an occupancy grid
  map at three resolutions, and Gauss-Newton matching of each rotation of the
  scanner against that map (the method of Hector SLAM: Kohlbrecher et al.,
  "A Flexible and Scalable SLAM System with Full 3D Motion Estimation", 2011).

  The pose it returns is the vehicle's position and heading in the map frame,
  which starts where the vehicle was at the first scan: x north (the first
  scan's forward direction, or the heading given to reset()), y east, yaw
  clockwise. AP_Proximity_LD06 feeds it and hands the pose to the EKF as
  external navigation.

  No ArduPilot dependencies: the same file builds in Tools/lidar_slam_test.
  Memory: (size/cell)^2 * 1.33 bytes, e.g. 20 m at 0.1 m = 53 KB.
*/
#pragma once

#include <stdint.h>

class LidarSlam {
public:
    struct Pose {
        float x, y;     // m, map frame (north, east)
        float yaw;      // rad, clockwise from north
    };

    struct Result {
        bool matched;   // scan matched against the map (false: pose is the prediction)
        bool mapped;    // the scan was added to the map
        float quality;  // 0..1: share of matched points that land on mapped obstacles
        uint16_t used;  // points used
    };

    static constexpr uint16_t MAX_POINTS = 360;     // one per degree is plenty
    static constexpr uint8_t LEVELS = 3;            // cell, 2 x cell, 4 x cell

    ~LidarSlam();

    // allocate the map: cell size and side length in metres. False if out of memory.
    bool init(float cell_m, float size_m);
    bool ready() const { return _level[0].cells != nullptr; }
    uint32_t memory_bytes() const;

    // forget the map; the next scan starts a new one at (0, 0, yaw0)
    void reset(float yaw0 = 0.0f);

    // One rotation of the scanner, deskewed: points in the body frame (x forward,
    // y right, metres) as seen from the scanner at the end of the rotation.
    // dyaw is the gyro's heading change since the previous scan (rad, clockwise).
    // sensor_x/y: the scanner's position on the vehicle (body frame, m).
    Result process(const float *px, const float *py, uint16_t n, float dyaw,
                   float sensor_x = 0.0f, float sensor_y = 0.0f);

    const Pose &pose() const { return _pose; }
    bool started() const { return _scans > 0; }
    uint32_t scans() const { return _scans; }

    // tuning
    float min_quality = 0.35f;      // below: the match is not trusted
    float map_update_dist = 0.30f;  // m moved before the map is updated again (Hector: 0.4)
    float map_update_yaw = 0.12f;   // rad turned before the map is updated again

private:
    struct Level {
        int8_t *cells;              // log-odds, -127 (free) .. 127 (occupied), 0 unknown
        uint16_t w;                 // cells per side
        float res;                  // m per cell
    };
    Level _level[LEVELS] {};
    float _size;                    // m per side; the map is centred on the start
    Pose _pose {};
    Pose _last_mapped {};
    Pose _velocity {};              // pose change of the last scan, for the prediction
    uint32_t _scans;
    float _yaw0;

    // work buffers: the points kept for this scan (body frame)
    float _bx[MAX_POINTS];
    float _by[MAX_POINTS];

    float prob(int8_t l) const;
    // map value and gradient (per metre) at a map-frame point, bilinear
    float sample(const Level &lv, float wx, float wy, float &dx, float &dy) const;
    void match(const Level &lv, uint16_t n, Pose &p, const Pose &prior, uint8_t iterations) const;
    float score(const Level &lv, uint16_t n, const Pose &p, uint16_t &inside) const;
    void update_map(uint16_t n, const Pose &p, float sx, float sy);
    void trace(Level &lv, float x0, float y0, float x1, float y1, bool hit);
};
