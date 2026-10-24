#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_Proximity_config.h"

#define PROXIMITY_MAX_IGNORE                4   // up to six areas can be ignored

class AP_Proximity_Params {

public:

    static const struct AP_Param::GroupInfo var_info[];

    AP_Proximity_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Proximity_Params);

    AP_Int8 type;                                       // type of sensor
    AP_Int8 orientation;                                // orientation (e.g. right-side-up or upside-down)
    AP_Int16 yaw_correction;                            // yaw correction in degrees
    AP_Int16 ignore_angle_deg[PROXIMITY_MAX_IGNORE];    // angle (in degrees) of area that should be ignored by sensor (i.e. leg shows up)
    AP_Int8 ignore_width_deg[PROXIMITY_MAX_IGNORE];     // width of beam (in degrees) that should be ignored
    AP_Float max_m;                                     // maximum range in meters
    AP_Float min_m;                                     // minimum range in meters
    AP_Int8  address;                                   // proximity address (for AP_Periph CAN)
#if AP_PROXIMITY_LD06_SLAM_ENABLED
    AP_Int8  slam;                                      // LD06/LD14: 1 = LiDAR SLAM to the EKF as external navigation
    AP_Float slam_cell;                                 // map cell size (m)
    AP_Float slam_size;                                 // map side length (m)
    AP_Float slam_lat;                                  // EKF origin when there is no GPS (deg)
    AP_Float slam_lon;
    AP_Float slam_pos_x;                                // scanner position on the vehicle, forward (m)
    AP_Float slam_pos_y;                                // and right (m)
#endif
};
