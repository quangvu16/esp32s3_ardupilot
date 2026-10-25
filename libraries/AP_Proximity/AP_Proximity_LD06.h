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
 * ArduPilot device driver for Inno-Maker LD06 LiDAR
 * and the LDROBOT LD14/LD14P, which use the same packet format
 *
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM LD06 DATASHEET:
 *
 * http://wiki.inno-maker.com/display/HOMEPAGE/LD06?preview=/6949506/6949511/LDROBOT_LD06_Development%20manual_v1.0_en.pdf
 *
 * Author: Adithya Patil, Georgia Institute of Technology
 * Based on the SLAMTEC RPLiDAR code written by Steven Josefs, IAV GmbH
 *
 */

#pragma once
#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LD06_ENABLED

#include "AP_Proximity_Backend_Serial.h"
#include <Filter/ModeFilter.h>
#include <GCS_MAVLink/GCS_config.h>
#if AP_PROXIMITY_LD06_SLAM_ENABLED
#include <AP_HAL/Semaphores.h>
#include "LidarSlam.h"
#define LD06_SLAM_MAX_POINTS 360              // points kept per rotation: one per degree
#endif

// Send each full rotation to the GCS as OBSTACLE_DISTANCE (72 bins of 5 degrees),
// so a ground station can show the scan and steer by it
#ifndef AP_PROXIMITY_LD06_SEND_SCAN
#define AP_PROXIMITY_LD06_SEND_SCAN HAL_GCS_ENABLED
#endif
#define LD06_SCAN_BINS 72
#define LD06_SCAN_FULL_BINS 360                // PRX_SCAN_FULL: one bin per degree, sent as 5 x 72
#define LD06_SCAN_MIN_INTERVAL_MS 200         // at most 5 scans a second

#define MESSAGE_LENGTH_LD06         47

// Minimum and maximum distance that the sensor can read in meters
#define MAX_READ_DISTANCE_LD06          12.0f
#define MIN_READ_DISTANCE_LD06           0.02f
#define MAX_READ_DISTANCE_LD14           8.0f

class AP_Proximity_LD06 : public AP_Proximity_Backend_Serial
{
public:

    // LD06/LD19 are time-of-flight sensors and report the measured angle
    // directly; the LD14/LD14P are triangulation sensors whose angles need
    // the ranging-centre correction from LDROBOT's SDK
    enum class Model : uint8_t {
        LD06,
        LD14,
    };

    AP_Proximity_LD06(AP_Proximity &_frontend,
                      AP_Proximity::Proximity_State &_state,
                      AP_Proximity_Params& _params,
                      uint8_t serial_instance,
                      Model model = Model::LD06) :
        AP_Proximity_Backend_Serial(_frontend, _state, _params, serial_instance),
        _model(model) {}

    // Update the state of the sensor
    void update(void) override;

    // Get the max and min distances for the sensor being used
    float distance_max() const override { return _model == Model::LD14 ? MAX_READ_DISTANCE_LD14 : MAX_READ_DISTANCE_LD06; }
    float distance_min() const override { return MIN_READ_DISTANCE_LD06; }

private:

    // Get and parse the sensor data
    void parse_response_data();
    void get_readings();

    // LD14 only: move a raw angle from the ranging centre to the centre of the sensor
    float correct_angle_ld14(float raw_angle_deg, uint16_t distance_mm);

#if AP_PROXIMITY_LD06_SEND_SCAN
    // a packet whose raw start angle is below the previous one begins a new rotation
    void scan_packet(float raw_start_angle_deg);
    // collect a valid reading into the current rotation's scan
    void scan_point(float angle_deg, float distance_m);
    void send_scan();
    uint16_t _scan_cm[LD06_SCAN_FULL_BINS];   ///< closest return per degree this rotation, 0 = none
    uint8_t _scan_seen[LD06_SCAN_FULL_BINS / 8]; ///< degrees measured this rotation, with or without a return
    void scan_seen(float angle_deg);
    float _scan_last_start_deg;               ///< raw start angle of the previous packet
    uint32_t _scan_last_sent_ms;
#endif

#if AP_PROXIMITY_LD06_SLAM_ENABLED
    // LiDAR SLAM (PRX_SLAM): the scan is matched against a map in a worker
    // thread, and the pose goes to the EKF as external navigation
    void slam_init();
    void slam_packet(float raw_start_angle_deg, uint16_t speed_dps);   // each packet, before its points
    void slam_point(float raw_angle_deg, float angle_deg, float distance_m);
    void slam_rotation_end();                 // hand the finished rotation to the worker
    void slam_update();                       // main thread: gyro, results to the EKF, origin
    void slam_thread();
    struct Slam {
        LidarSlam core;
        // the rotation being collected (main thread only)
        float raw_deg[LD06_SLAM_MAX_POINTS];  // sensor's own angle: when the point was taken
        float ang_deg[LD06_SLAM_MAX_POINTS];  // vehicle-frame angle
        float dist_m[LD06_SLAM_MAX_POINTS];
        uint16_t n;
        float last_start_deg;
        int16_t last_kept_deg;                // whole degree of the last point kept
        uint32_t last_wrap_ms;                // when the previous rotation ended
        uint8_t warmup;                       // steady rotations seen before the map starts
        float speed_dps;
        float rot_dyaw;                       // gyro heading change during this rotation (rad)
        float dyaw_acc;                       // and since the last scan given to the worker
        uint32_t last_gyro_us;
        // the job for the worker: owned by the worker while busy
        float px[LD06_SLAM_MAX_POINTS];
        float py[LD06_SLAM_MAX_POINTS];
        uint16_t job_n;
        float job_dyaw;
        uint32_t job_ms;
        bool job_reset;
        float job_yaw0;
        // results, under sem
        HAL_Semaphore sem;
        HAL_BinarySemaphore wake;
        bool busy;
        bool have_result;
        LidarSlam::Result result;
        LidarSlam::Pose pose;
        uint32_t result_ms;
        // main thread
        bool started;                         // the first scan has gone to the worker
        Vector2f offset_ne;                   // EKF position at the start: the map's (0,0)
        uint32_t reset_ms;
        uint16_t lost_count;
        bool tracking;
        uint32_t last_origin_try_ms;
        LidarSlam::Pose prev_pose;            // for the forward speed used in the deskew
        uint32_t prev_ms;
        float v_fwd;                          // m/s, from the SLAM's own poses
        uint32_t dropped;
    };
    Slam *_slam;
    bool _slam_init_done;
#endif

    const Model _model;
    float _ld14_shift_deg;                    ///< last ranging-centre correction, reused for zero-distance points

    // Store and keep track of the bytes being read from the sensor
    uint8_t _response[MESSAGE_LENGTH_LD06];
    uint16_t _byte_count;

    // Store for error-tracking purposes
    uint32_t  _last_distance_received_ms;

    // distance filter applies to raw measurements
    ModeFilterUInt16_Size3 _dist_filt_mm {1};

    // face related variables
    AP_Proximity_Boundary_3D::Face _last_face;///< last face requested
    float _last_angle_deg;                    ///< yaw angle (in degrees) of _last_distance_m
    float _last_distance_m;                   ///< shortest distance for _last_face
    bool _last_distance_valid;                ///< true if _last_distance_m is valid

    // angle and distance for the latest 2 degree sector
    uint16_t _angle_2deg;
    float _dist_2deg_m;
};
#endif // AP_PROXIMITY_LD06_ENABLED
