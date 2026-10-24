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

    // Begin getting sensor readings
    // Calls method that repeatedly reads through UART channel
    get_readings();

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
    }
    _scan_last_start_deg = raw_start_angle_deg;
}

// angle_deg is in the vehicle frame (after PRX_ORIENT and PRX_YAW_CORR)
void AP_Proximity_LD06::scan_point(float angle_deg, float distance_m)
{
    const uint8_t bin = MIN(uint8_t(wrap_360(angle_deg) / (360.0f / LD06_SCAN_BINS)), LD06_SCAN_BINS - 1);
    const uint16_t cm = MAX(uint16_t(distance_m * 100.0f), uint16_t(1));
    if (_scan_cm[bin] == 0 || cm < _scan_cm[bin]) {
        _scan_cm[bin] = cm;
    }
}

// OBSTACLE_DISTANCE in the vehicle frame: bin 0 is straight ahead, 5 degrees
// per bin clockwise. Bins without a return are sent as max_distance + 1 (clear).
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
    pkt.increment = 360 / LD06_SCAN_BINS;
    pkt.min_distance = uint16_t(distance_min() * 100.0f);
    pkt.max_distance = uint16_t(distance_max() * 100.0f);
    pkt.frame = MAV_FRAME_BODY_FRD;
    for (uint8_t i = 0; i < LD06_SCAN_BINS; i++) {
        pkt.distances[i] = _scan_cm[i] != 0 ? _scan_cm[i] : pkt.max_distance + 1;
    }
    gcs().send_to_active_channels(MAVLINK_MSG_ID_OBSTACLE_DISTANCE, (const char *)&pkt);
}
#endif // AP_PROXIMITY_LD06_SEND_SCAN

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
