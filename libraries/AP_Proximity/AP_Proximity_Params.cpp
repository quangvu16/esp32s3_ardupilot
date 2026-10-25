#include "AP_Proximity_Params.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Proximity_Params::var_info[] = {

    // 0 should not be used

    // @Param: _TYPE
    // @DisplayName: Proximity type
    // @Description: What type of proximity sensor is connected
    // @SortValues: AlphabeticalZeroAtTop
    // @Values: 0:None,7:LightwareSF40c,2:MAVLink,3:TeraRangerTower,4:RangeFinder,5:RPLidarA2,6:TeraRangerTowerEvo,8:LightwareSF45B,10:SITL,12:AirSimSITL,13:CygbotD1, 14:DroneCAN, 15:Scripting, 16:LD06, 17:MR72_CAN, 18:HexsoonRadar, 19:LD14
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE",   1, AP_Proximity_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _ORIENT
    // @DisplayName: Proximity sensor orientation
    // @Description: Proximity sensor orientation
    // @Values: 0:Default,1:Upside Down
    // @User: Standard
    AP_GROUPINFO("_ORIENT", 2, AP_Proximity_Params, orientation, 0),

    // @Param: _YAW_CORR
    // @DisplayName: Proximity sensor yaw correction
    // @Description: Proximity sensor yaw correction
    // @Units: deg
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("_YAW_CORR", 3, AP_Proximity_Params, yaw_correction, 0),

    // @Param: _IGN_ANG1
    // @DisplayName: Proximity sensor ignore angle 1
    // @Description: Proximity sensor ignore angle 1
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG1", 4, AP_Proximity_Params, ignore_angle_deg[0], 0),

    // @Param: _IGN_WID1
    // @DisplayName: Proximity sensor ignore width 1
    // @Description: Proximity sensor ignore width 1
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID1", 5, AP_Proximity_Params, ignore_width_deg[0], 0),

    // @Param: _IGN_ANG2
    // @DisplayName: Proximity sensor ignore angle 2
    // @Description: Proximity sensor ignore angle 2
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG2", 6, AP_Proximity_Params, ignore_angle_deg[1], 0),

    // @Param: _IGN_WID2
    // @DisplayName: Proximity sensor ignore width 2
    // @Description: Proximity sensor ignore width 2
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID2", 7, AP_Proximity_Params, ignore_width_deg[1], 0),

    // @Param: _IGN_ANG3
    // @DisplayName: Proximity sensor ignore angle 3
    // @Description: Proximity sensor ignore angle 3
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG3", 8, AP_Proximity_Params, ignore_angle_deg[2], 0),

    // @Param: _IGN_WID3
    // @DisplayName: Proximity sensor ignore width 3
    // @Description: Proximity sensor ignore width 3
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID3", 9, AP_Proximity_Params, ignore_width_deg[2], 0),

    // @Param: _IGN_ANG4
    // @DisplayName: Proximity sensor ignore angle 4
    // @Description: Proximity sensor ignore angle 4
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG4", 10, AP_Proximity_Params, ignore_angle_deg[3], 0),

    // @Param: _IGN_WID4
    // @DisplayName: Proximity sensor ignore width 4
    // @Description: Proximity sensor ignore width 4
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID4", 11, AP_Proximity_Params, ignore_width_deg[3], 0),

    // @Param: _MIN
    // @DisplayName: Proximity minimum range
    // @Description: Minimum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.
    // @Units: m
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("_MIN", 16, AP_Proximity_Params, min_m, 0.0f),

    // @Param: _MAX
    // @DisplayName: Proximity maximum range
    // @Description: Maximum expected range for Proximity Sensor. Setting this to 0 will set value to manufacturer reported range.
    // @Units: m
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("_MAX", 17, AP_Proximity_Params, max_m, 0.0f),

    // @Param: _ADDR
    // @DisplayName: Bus address of sensor
    // @Description: The bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ADDR", 25, AP_Proximity_Params, address, 0),

#if AP_PROXIMITY_LD06_SLAM_ENABLED
    // @Param: _SLAM
    // @DisplayName: LiDAR SLAM
    // @Description: LD06/LD14 only (PRX_TYPE 16 or 19). Builds a 2D map of the surroundings from the scan, matches every rotation against it, and gives the vehicle's position and heading in that map to the EKF as external navigation, for driving without GPS (indoors). Set EK3_SRC1_POSXY, EK3_SRC1_POSZ and EK3_SRC1_YAW to 6 (ExternalNav) to use it. The map is kept in RAM and starts again at each boot, where the vehicle stands.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_SLAM", 26, AP_Proximity_Params, slam, 0),

    // @Param: _SLAM_CELL
    // @DisplayName: LiDAR SLAM map cell size
    // @Description: Size of one map cell. Smaller cells need more memory: (SLAM_SIZE / SLAM_CELL)^2 x 1.33 bytes.
    // @Units: m
    // @Range: 0.05 0.3
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_SLAM_CELL", 27, AP_Proximity_Params, slam_cell, 0.1f),

    // @Param: _SLAM_SIZE
    // @DisplayName: LiDAR SLAM map size
    // @Description: Side length of the square map, centred where the vehicle starts. Keep the vehicle well inside it. 20 m with 0.1 m cells uses 53 KB of RAM.
    // @Units: m
    // @Range: 5 50
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_SLAM_SIZE", 28, AP_Proximity_Params, slam_size, 20.0f),

    // @Param: _SLAM_LAT
    // @DisplayName: LiDAR SLAM origin latitude
    // @Description: Without GPS the EKF has no origin, which Auto, Guided and RTL need. SLAM sets it here (with SLAM_LON) when no GPS has given one, so maps and missions show where the building really is.
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("_SLAM_LAT", 29, AP_Proximity_Params, slam_lat, 0.0f),

    // @Param: _SLAM_LON
    // @DisplayName: LiDAR SLAM origin longitude
    // @Description: See SLAM_LAT.
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("_SLAM_LON", 30, AP_Proximity_Params, slam_lon, 0.0f),

    // @Param: _SLAM_X
    // @DisplayName: LiDAR position, forward
    // @Description: The LiDAR's distance in front of the vehicle's centre (the IMU), negative if behind.
    // @Units: m
    // @Range: -1 1
    // @User: Advanced
    AP_GROUPINFO("_SLAM_X", 31, AP_Proximity_Params, slam_pos_x, 0.0f),

    // @Param: _SLAM_Y
    // @DisplayName: LiDAR position, right
    // @Description: The LiDAR's distance to the right of the vehicle's centre (the IMU), negative if left.
    // @Units: m
    // @Range: -1 1
    // @User: Advanced
    AP_GROUPINFO("_SLAM_Y", 32, AP_Proximity_Params, slam_pos_y, 0.0f),
#endif

#if AP_PROXIMITY_LD06_ENABLED
    // @Param: _SCAN_FULL
    // @DisplayName: Full-resolution scan to the GCS
    // @Description: LD06/LD14 only. Each rotation is sent to the ground station as OBSTACLE_DISTANCE: 0 = 72 bins of 5 degrees in one message, 1 = 360 bins of 1 degree in five messages (angle_offset 0, 72, 144, 216, 288), for maps and indoor navigation in the ground station. At most 5 rotations a second either way; 1 uses about 4.5 KB/s of the link.
    // @Values: 0:72 bins of 5 deg,1:360 bins of 1 deg
    // @User: Advanced
    AP_GROUPINFO("_SCAN_FULL", 33, AP_Proximity_Params, scan_full, 0),
#endif

    AP_GROUPEND
};

AP_Proximity_Params::AP_Proximity_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
