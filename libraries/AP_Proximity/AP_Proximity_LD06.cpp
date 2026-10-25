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
 *
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM InnoMaker DATASHEET:
 *
 * http://wiki.inno-maker.com/display/HOMEPAGE/LD06?preview=/6949506/6949511/LDROBOT_LD06_Development%20manual_v1.0_en.pdf
 *
 * Author: Adithya Patil, Georgia Institute of Technology
 * Based on the SLAMTEC RPLiDAR code written by Steven Josefs, IAV GmbH and CYGBOT D1 LiDAR code
 *
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LD06_ENABLED
#include "AP_Proximity_LD06.h"
#include <GCS_MAVLink/GCS.h>
#if AP_PROXIMITY_LD06_SLAM_ENABLED
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/Location.h>

extern const AP_HAL::HAL& hal;

#define LD06_SLAM_MIN_RANGE_M   0.15f         // closer points are the vehicle itself
#define LD06_SLAM_LOST_SCANS    30            // ~5 s of failed matches: report lost
#endif

#define LD_START_CHAR       0x54
#define PROXIMITY_LD06_TIMEOUT_MS 50

// Indices in data array where each value starts being recorded
// See comment below about data payload for more info about formatting
#define START_DATA_LENGTH           1
#define START_BEGIN_ANGLE           4
#define START_PAYLOAD               6
#define START_END_ANGLE             42
#define START_CHECK_SUM             46
#define MEASUREMENT_PAYLOAD_LENGTH  3
#define PAYLOAD_COUNT               12

// confidence for each measurement must be this value or higher
#define CONFIDENCE_THRESHOLD 20

// LD14/LD14P ranging-centre offsets in mm, from LDROBOT's SDK (sl_transform.cpp)
#define LD14_OFFSET_X       5.9f
#define LD14_OFFSET_Y       -18.975571f
#define LD14_SLOPE          0.11923f

 /* ------------------------------------------
    Data Packet Structure:
    Start Character : 1 Byte
    Data Length : 1 Byte
    Radar Speed : 2 Bytes
    Start Angle : 2 Bytes
    Data Measurements : 36 Bytes
        Contains 12 measurements of 3 Bytes each
        Each measurement has 2 Bytes for distance to closest object
        Each measurement has the 3rd Byte as measurement Confidence
    End Angle : 2 Bytes
    Timestamp : 2 Bytes
    Checksum : 1 Byte
    ------------------------------------------ */
// ----> 47 data bytes in total for one packet

// Update the sensor readings
void AP_Proximity_LD06::update(void)
{
    // Escape if no connection detected/supported while running
    if (_uart == nullptr) {
        return;
    }

#if AP_PROXIMITY_LD06_SLAM_ENABLED
    if (!_slam_init_done) {
        _slam_init_done = true;
        if (params.slam > 0) {
            slam_init();
        }
    }
    if (_slam != nullptr) {
        // integrate the gyro before the new packets: a rotation that ends in
        // them gets the heading change up to now
        const uint32_t now_us = AP_HAL::micros();
        if (_slam->last_gyro_us != 0) {
            const float dt = MIN((now_us - _slam->last_gyro_us) * 1.0e-6f, 0.1f);
            const float rate = AP::ahrs().get_gyro().z;      // rad/s, clockwise, bias-corrected
            _slam->rot_dyaw += rate * dt;
            _slam->dyaw_acc += rate * dt;
        }
        _slam->last_gyro_us = now_us;
    }
#endif

    // Begin getting sensor readings
    // Calls method that repeatedly reads through UART channel
    get_readings();

#if AP_PROXIMITY_LD06_SLAM_ENABLED
    if (_slam != nullptr) {
        slam_update();
    }
#endif

    // Check if the data is being received correctly and sets Proximity Status
    if (_last_distance_received_ms == 0 || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_LD06_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// Called repeatedly to get the readings at the current instant
void AP_Proximity_LD06::get_readings()
{
    if (_uart == nullptr) {
        return;
    }

    // Store the number of bytes available on the UART input
    uint32_t nbytes = MIN((uint16_t) 4000,  _uart->available());

    // Loops through all bytes that were received
    while (nbytes-- > 0) {

        uint8_t c;
        if (!_uart->read(c)) {
            break;
        }

        // Stores the byte in an array if the byte is a start byte or we have already read a start byte
        if (c == LD_START_CHAR || _byte_count) {
            // Stores the next byte in an array
            _response[_byte_count] = c;
            if (_byte_count < START_DATA_LENGTH) {
                _byte_count++;
                continue;
            }

            // total_packet_length = sizeof(header) + datalength + sizeof(footer):
            const uint32_t total_packet_length = 6 + 3*(_response[START_DATA_LENGTH] & 0x1F) + 5;
            if ((_response[START_DATA_LENGTH] & 0x1F) != PAYLOAD_COUNT ||
                total_packet_length > ARRAY_SIZE(_response)) {
                // invalid packet received; throw away all data and
                // start again.
                _byte_count = 0;
                _uart->discard_input();
                break;
            }

            _byte_count++;

            if (_byte_count == total_packet_length) {

                const uint32_t current_ms = AP_HAL::millis();

                _last_distance_received_ms =  current_ms;

                // Updates the temporary boundary and passes off the completed data
                parse_response_data();

                // Resets the bytes read and whether or not we are reading data to accept a new payload
                _byte_count = 0;
            }
        }
    }
}

// Parses the data packet received from the LiDAR
void AP_Proximity_LD06::parse_response_data()
{

    // Data interpretation based on:
    // http://wiki.inno-maker.com/display/HOMEPAGE/LD06?preview=/6949506/6949511/LDROBOT_LD06_Development%20manual_v1.0_en.pdf

    // Second byte in array stores length of data - not used but stored for debugging
    // const uint8_t data_length = _response[START_DATA_LENGTH];

    // Verify the checksum that is stored in the last element of the response array
    // Return if checksum is incorrect - i.e. bad data, bad readings, etc.
    const uint8_t check_sum = _response[START_CHECK_SUM];
    if (check_sum != crc8_generic(&_response[0], sizeof(_response) / sizeof(_response[0]) - 1, 0x4D)) {
        return;
    }

    // Respective bits store the radar speed, start/end angles
    // Use bitwise operations to correctly obtain correct angles
    // Divide angles by 100 as per manual
    const float start_angle = float(UINT16_VALUE(_response[START_BEGIN_ANGLE + 1], _response[START_BEGIN_ANGLE])) * 0.01;
    const float end_angle = float(UINT16_VALUE(_response[START_END_ANGLE + 1], _response[START_END_ANGLE])) * 0.01;

    float angle_step;
    if (start_angle < end_angle) {
        angle_step = (end_angle - start_angle) / (PAYLOAD_COUNT - 1);
    } else {
        angle_step = (end_angle + 360 - start_angle) / (PAYLOAD_COUNT - 1);
    }
#if AP_PROXIMITY_LD06_SEND_SCAN
    scan_packet(start_angle);
#endif
#if AP_PROXIMITY_LD06_SLAM_ENABLED
    if (_slam != nullptr) {
        slam_packet(start_angle, UINT16_VALUE(_response[3], _response[2]));
    }
#endif
    // Each recording point is three bytes long, goes through all of that and updates database
    for (uint16_t i = START_PAYLOAD; i < START_PAYLOAD + MEASUREMENT_PAYLOAD_LENGTH * PAYLOAD_COUNT; i += MEASUREMENT_PAYLOAD_LENGTH) {

        // Gets the distance recorded and converts to meters
        const uint16_t distance_mm = UINT16_VALUE(_response[i + 1], _response[i]);
        float raw_angle_deg = start_angle + angle_step * (i / MEASUREMENT_PAYLOAD_LENGTH);
        if (_model == Model::LD14) {
            raw_angle_deg = correct_angle_ld14(raw_angle_deg, distance_mm);
        }
        const float angle_deg = correct_angle_for_orientation(raw_angle_deg);
        const float distance_m = _dist_filt_mm.apply(distance_mm) * 0.001;
        const float confidence = _response[i + 2];
#if AP_PROXIMITY_LD06_SEND_SCAN
        scan_seen(angle_deg);
#endif

        // ignore distance that are out-of-range or have low confidence.
        // The LD14's intensity is not a confidence, so it is not filtered on
        if (distance_m < distance_min() || distance_m > distance_max() ||
            (_model == Model::LD06 && confidence < CONFIDENCE_THRESHOLD)) {
            continue;
        }

        // ignore distances that are out-of-range (based on user parameters) or within ignore areas
        if (ignore_reading(angle_deg, distance_m)) {
            continue;
        }

#if AP_PROXIMITY_LD06_SEND_SCAN
        scan_point(angle_deg, distance_m);
#endif
#if AP_PROXIMITY_LD06_SLAM_ENABLED
        if (_slam != nullptr) {
            slam_point(start_angle + angle_step * (i / MEASUREMENT_PAYLOAD_LENGTH), angle_deg, distance_m);
        }
#endif

        // use the shortest distance within 2 degree sectors
        const uint16_t a2d = (int)(angle_deg / 2.0) * 2;
        if (_angle_2deg == a2d) {
            if (distance_m < _dist_2deg_m) {
                _dist_2deg_m = distance_m;
            }
        } else {
            // new 2 degree sector, process the old one

            // check if new sector is also a new face
            const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face((float)_angle_2deg);
            if (face != _last_face) {
                // distance is for a new face, the previous one can be updated now
                if (_last_distance_valid) {
                    frontend.boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m, state.instance);
                } else {
                    // reset distance from last face
                    frontend.boundary.reset_face(face, state.instance);
                }

                // initialize the new face
                _last_face = face;
                _last_distance_valid = false;
            }

            // update face's shortest distance
            if (!_last_distance_valid || (_dist_2deg_m < _last_distance_m)) {
                _last_distance_m = _dist_2deg_m;
                _last_distance_valid = true;
                _last_angle_deg = (float)_angle_2deg;
            }

            // update OA database with the 2 degree sectors distance
            database_push(_angle_2deg, _dist_2deg_m);

            // advance to the next 2 degree sector
            _angle_2deg = a2d;
            _dist_2deg_m = distance_m;
        }
    }
}

#if AP_PROXIMITY_LD06_SEND_SCAN
// The sensor's own packet start angles rise through each rotation and fall
// back on the wrap through 0. They are used, rather than the corrected angles
// of the readings, because the LD14 correction moves very close returns (the
// car's own chassis) by up to ~30 degrees: near 0 those would look like extra
// wraps and throw away part of the scan. Mounting (PRX_ORIENT, PRX_YAW_CORR)
// does not change the raw angles either.
void AP_Proximity_LD06::scan_packet(float raw_start_angle_deg)
{
    if (raw_start_angle_deg < _scan_last_start_deg) {
        send_scan();
        memset(_scan_cm, 0, sizeof(_scan_cm));
        memset(_scan_seen, 0, sizeof(_scan_seen));
    }
    _scan_last_start_deg = raw_start_angle_deg;
}

// a measurement was made at angle_deg, whether or not it found anything: a
// degree with no measurement at all this rotation (the packet that ends a
// rotation carries a few degrees into the next) is sent as unknown, not clear
void AP_Proximity_LD06::scan_seen(float angle_deg)
{
    const uint16_t bin = MIN(uint16_t(wrap_360(angle_deg)), uint16_t(LD06_SCAN_FULL_BINS - 1));
    _scan_seen[bin / 8] |= 1U << (bin % 8);
}

// angle_deg is in the vehicle frame (after PRX_ORIENT and PRX_YAW_CORR)
void AP_Proximity_LD06::scan_point(float angle_deg, float distance_m)
{
    const uint16_t bin = MIN(uint16_t(wrap_360(angle_deg)), uint16_t(LD06_SCAN_FULL_BINS - 1));
    const uint16_t cm = MAX(uint16_t(distance_m * 100.0f), uint16_t(1));
    if (_scan_cm[bin] == 0 || cm < _scan_cm[bin]) {
        _scan_cm[bin] = cm;
    }
}

// OBSTACLE_DISTANCE in the vehicle frame: bin 0 is straight ahead, angles
// clockwise. Normally 72 bins of 5 degrees in one message. With PRX_SCAN_FULL
// 1, 360 bins of 1 degree in five messages of 72 (angle_offset 0, 72, 144,
// 216, 288) with the same time_usec, for maps and localisation in the apps.
// Bins measured without a return are sent as max_distance + 1 (clear), bins not
// measured this rotation as UINT16_MAX (unknown). time_usec is the
// end of the rotation, on the autopilot's clock (as LOCAL_POSITION_NED's
// time_boot_ms), so a ground station can match the scan to the pose.
void AP_Proximity_LD06::send_scan()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _scan_last_sent_ms < LD06_SCAN_MIN_INTERVAL_MS) {
        return;
    }
    _scan_last_sent_ms = now_ms;
    mavlink_obstacle_distance_t pkt {};
    pkt.time_usec = AP_HAL::micros64();
    pkt.sensor_type = MAV_DISTANCE_SENSOR_LASER;
    pkt.min_distance = uint16_t(distance_min() * 100.0f);
    pkt.max_distance = uint16_t(distance_max() * 100.0f);
    pkt.frame = MAV_FRAME_BODY_FRD;
    const uint16_t clear = pkt.max_distance + 1;
    auto seen = [this](uint16_t bin) { return (_scan_seen[bin / 8] & (1U << (bin % 8))) != 0; };
    if (params.scan_full > 0) {
        pkt.increment = 1;
        pkt.increment_f = 1.0f;
        for (uint16_t part = 0; part < LD06_SCAN_FULL_BINS / LD06_SCAN_BINS; part++) {
            pkt.angle_offset = part * LD06_SCAN_BINS;
            for (uint8_t i = 0; i < LD06_SCAN_BINS; i++) {
                const uint16_t bin = part * LD06_SCAN_BINS + i;
                const uint16_t cm = _scan_cm[bin];
                pkt.distances[i] = cm != 0 ? cm : seen(bin) ? clear : UINT16_MAX;
            }
            gcs().send_to_active_channels(MAVLINK_MSG_ID_OBSTACLE_DISTANCE, (const char *)&pkt);
        }
        return;
    }
    const uint8_t per_bin = LD06_SCAN_FULL_BINS / LD06_SCAN_BINS;
    pkt.increment = 360 / LD06_SCAN_BINS;
    for (uint8_t i = 0; i < LD06_SCAN_BINS; i++) {
        uint16_t best = 0;
        bool any_seen = false;
        for (uint8_t k = 0; k < per_bin; k++) {
            const uint16_t cm = _scan_cm[i * per_bin + k];
            if (cm != 0 && (best == 0 || cm < best)) {
                best = cm;
            }
            any_seen |= seen(i * per_bin + k);
        }
        pkt.distances[i] = best != 0 ? best : any_seen ? clear : UINT16_MAX;
    }
    gcs().send_to_active_channels(MAVLINK_MSG_ID_OBSTACLE_DISTANCE, (const char *)&pkt);
}
#endif // AP_PROXIMITY_LD06_SEND_SCAN

#if AP_PROXIMITY_LD06_SLAM_ENABLED
/*
  LiDAR SLAM. The main thread collects each rotation's points, straightens out
  the motion during the rotation (deskew) and hands them to a worker thread,
  which matches them against the map (LidarSlam, ~15-30 ms on an ESP32-S3).
  The pose comes back to the main thread, which gives it to the EKF.
*/
void AP_Proximity_LD06::slam_init()
{
    _slam = NEW_NOTHROW Slam;
    if (_slam == nullptr || !_slam->core.init(params.slam_cell, params.slam_size)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SLAM: out of memory (%.0f m map, %.2f m cells)",
                      double(params.slam_size.get()), double(params.slam_cell.get()));
        delete _slam;
        _slam = nullptr;
        return;
    }
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Proximity_LD06::slam_thread, void),
                                      "slam", 4096, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SLAM: no thread");
        delete _slam;
        _slam = nullptr;
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SLAM: map %.0f m, %.2f m cells, %u KB",
                  double(params.slam_size.get()), double(params.slam_cell.get()),
                  unsigned(_slam->core.memory_bytes() / 1024));
}

void AP_Proximity_LD06::slam_packet(float raw_start_angle_deg, uint16_t speed_dps)
{
    Slam &s = *_slam;
    if (speed_dps > 360) {
        s.speed_dps = speed_dps;
    }
    if (raw_start_angle_deg < s.last_start_deg) {
        // a rotation has ended. Only one that took about as long as a turn of
        // the sensor is used: a backlog of packets read at once (at start-up,
        // or after a pause) has the wrong timing for the deskew
        const uint32_t now_ms = AP_HAL::millis();
        const float period_ms = 360000.0f / (s.speed_dps > 360 ? s.speed_dps : 2160.0f);
        const uint32_t took_ms = now_ms - s.last_wrap_ms;
        s.last_wrap_ms = now_ms;
        if (took_ms > 0.6f * period_ms && took_ms < 3.0f * period_ms) {
            slam_rotation_end();
        } else {
            s.n = 0;
            s.rot_dyaw = 0;
        }
    }
    s.last_start_deg = raw_start_angle_deg;
}

void AP_Proximity_LD06::slam_point(float raw_angle_deg, float angle_deg, float distance_m)
{
    if (distance_m < LD06_SLAM_MIN_RANGE_M || _slam->n >= LD06_SLAM_MAX_POINTS) {
        return;
    }
    // thin out to one point per degree, spread over the whole rotation
    // (the LD06 measures about two per degree, the LD14 about one)
    const int16_t whole_deg = int16_t(raw_angle_deg);
    if (_slam->n > 0 && whole_deg == _slam->last_kept_deg) {
        return;
    }
    _slam->last_kept_deg = whole_deg;
    const uint16_t k = _slam->n++;
    _slam->raw_deg[k] = raw_angle_deg;
    _slam->ang_deg[k] = angle_deg;
    _slam->dist_m[k] = distance_m;
}

void AP_Proximity_LD06::slam_rotation_end()
{
    Slam &s = *_slam;
    const uint16_t n = s.n;
    const float rot_dyaw = s.rot_dyaw;
    s.n = 0;
    s.rot_dyaw = 0;
    if (n < 30) {
        return;                                // nothing to see
    }
    if (s.warmup < 10) {
        s.warmup++;                            // let the sensor settle for a couple of seconds first
        return;
    }
    {
        WITH_SEMAPHORE(s.sem);
        if (s.busy) {
            s.dropped++;                       // the worker is behind: skip this one, keep the heading change
            return;
        }
    }

    AP_AHRS &ahrs = AP::ahrs();
    // the scan is deskewed to the time of its last point: an earlier point
    // was taken when the vehicle was turned less and was further back
    const float speed_dps = s.speed_dps > 360 ? s.speed_dps : 2160.0f;
    const float period_s = 360.0f / speed_dps;
    const float rate = rot_dyaw / period_s;                  // rad/s
    const float v_fwd = s.v_fwd;
    const float raw_end = s.raw_deg[n - 1];
    for (uint16_t k = 0; k < n; k++) {
        const float ago = wrap_360(raw_end - s.raw_deg[k]) / speed_dps;
        const float a = radians(s.ang_deg[k]) - rate * ago;
        s.px[k] = s.dist_m[k] * cosf(a) - v_fwd * ago;
        s.py[k] = s.dist_m[k] * sinf(a);
    }
    s.job_n = n;
    s.job_dyaw = s.dyaw_acc;
    s.dyaw_acc = 0;
    s.job_ms = AP_HAL::millis();
    s.job_reset = !s.started;
    if (!s.started) {
        // the map starts here, lined up with the heading the AHRS has now
        s.started = true;
        s.job_yaw0 = ahrs.get_yaw();
        s.reset_ms = s.job_ms;
        Vector2f ne;
        if (ahrs.get_relative_position_NE_origin(ne)) {
            s.offset_ne = ne;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SLAM: started, heading %.0f", double(degrees(wrap_2PI(s.job_yaw0))));
    }
    {
        WITH_SEMAPHORE(s.sem);
        s.busy = true;
    }
    s.wake.signal();
}

void AP_Proximity_LD06::slam_thread()
{
    Slam &s = *_slam;
    while (true) {
        s.wake.wait_blocking();
        if (s.job_reset) {
            s.core.reset(s.job_yaw0);
        }
        const LidarSlam::Result r = s.core.process(s.px, s.py, s.job_n, s.job_dyaw,
                                                   params.slam_pos_x, params.slam_pos_y);
        WITH_SEMAPHORE(s.sem);
        s.result = r;
        s.pose = s.core.pose();
        s.result_ms = s.job_ms;
        s.have_result = true;
        s.busy = false;
    }
}

void AP_Proximity_LD06::slam_update()
{
    Slam &s = *_slam;
    AP_AHRS &ahrs = AP::ahrs();
    const uint32_t now_ms = AP_HAL::millis();

    // without GPS the EKF has no origin, and Auto, Guided and RTL need one
    if (s.started && now_ms - s.last_origin_try_ms > 1000) {
        s.last_origin_try_ms = now_ms;
        Location origin;
        if (!ahrs.get_origin(origin)
#if AP_GPS_ENABLED
            && AP::gps().status() < AP_GPS::GPS_OK_FIX_3D
#endif
            ) {
            origin.zero();
            origin.lat = int32_t(params.slam_lat * 1.0e7f);
            origin.lng = int32_t(params.slam_lon * 1.0e7f);
            if (ahrs.set_origin(origin)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SLAM: EKF origin %.6f %.6f",
                              double(params.slam_lat.get()), double(params.slam_lon.get()));
            }
        }
    }

    LidarSlam::Result r;
    LidarSlam::Pose p;
    uint32_t t_ms;
    {
        WITH_SEMAPHORE(s.sem);
        if (!s.have_result) {
            return;
        }
        s.have_result = false;
        r = s.result;
        p = s.pose;
        t_ms = s.result_ms;
    }

    if (r.matched) {
        // forward speed for the next deskew, from the last two poses
        if (s.prev_ms != 0 && t_ms - s.prev_ms > 20 && t_ms - s.prev_ms < 1000) {
            const float d = (p.x - s.prev_pose.x) * cosf(p.yaw) + (p.y - s.prev_pose.y) * sinf(p.yaw);
            const float v = constrain_float(d / ((t_ms - s.prev_ms) * 0.001f), -5.0f, 5.0f);
            s.v_fwd = 0.5f * s.v_fwd + 0.5f * v;
        }
        s.prev_pose = p;
        s.prev_ms = t_ms;
        s.lost_count = 0;
        if (!s.tracking) {
            s.tracking = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SLAM: tracking");
        }
        // uncertainty grows as fewer points land on the map
        const float q = constrain_float(r.quality, 0.0f, 1.0f);
        const float pos_err = 0.05f + (1.0f - q) * 0.5f;
        const float ang_err = 0.03f + (1.0f - q) * 0.2f;
        Quaternion att;
        att.from_euler(ahrs.get_roll(), ahrs.get_pitch(), p.yaw);
        const Vector3f pos {p.x + s.offset_ne.x, p.y + s.offset_ne.y, 0.0f};
        ahrs.writeExtNavData(pos, att, pos_err, ang_err, t_ms, 0, s.reset_ms);
    } else if (s.lost_count < UINT16_MAX) {
        s.lost_count++;
        if (s.tracking && s.lost_count == LD06_SLAM_LOST_SCANS) {
            s.tracking = false;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SLAM: lost (scan does not match the map)");
        }
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: SLAM
    // @Description: LiDAR SLAM pose, given to the EKF as external navigation
    // @Field: TimeUS: Time since system startup
    // @Field: X: position north of the start (map frame)
    // @Field: Y: position east of the start (map frame)
    // @Field: Yaw: heading
    // @Field: Q: match quality, 0..1
    // @Field: N: points used
    // @Field: M: 1 if the scan matched the map
    // @Field: Map: 1 if the scan was added to the map
    // @Field: Drop: scans skipped because the matcher was busy
    // @Field: Age: time from the end of the scan to now (the pose is for the end of the scan)
    AP::logger().WriteStreaming("SLAM", "TimeUS,X,Y,Yaw,Q,N,M,Map,Drop,Age", "smmd-----s", "F000-----C",
                                "QffffHBBIH",
                                AP_HAL::micros64(), p.x, p.y, degrees(wrap_2PI(p.yaw)), r.quality,
                                r.used, uint8_t(r.matched), uint8_t(r.mapped), s.dropped,
                                uint16_t(MIN(now_ms - t_ms, 65535U)));
#endif
}
#endif // AP_PROXIMITY_LD06_SLAM_ENABLED

// The LD14/LD14P is a triangulation sensor: the reported angle is that of the
// ranging centre, offset from the centre of rotation. Move it to the centre
// (LDROBOT SDK, sl_transform.cpp). The SDK spells this (360 - raw) + shift
// because it flips to counter-clockwise at the same time; ArduPilot's angles
// are clockwise like the sensor's own, so the correction is raw - shift.
float AP_Proximity_LD06::correct_angle_ld14(float raw_angle_deg, uint16_t distance_mm)
{
    if (distance_mm > 0) {
        const float x = distance_mm + LD14_OFFSET_X;
        const float y = distance_mm * LD14_SLOPE + LD14_OFFSET_Y;
        _ld14_shift_deg = degrees(atanf(y / x));
    }
    // a zero-distance point reuses the previous point's correction
    return wrap_360(raw_angle_deg - _ld14_shift_deg);
}
#endif // AP_PROXIMITY_LD06_ENABLED
