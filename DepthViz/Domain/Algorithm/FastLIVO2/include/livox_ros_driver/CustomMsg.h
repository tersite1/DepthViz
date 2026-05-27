#ifndef LIVOX_ROS_DRIVER_CUSTOMMSG_H
#define LIVOX_ROS_DRIVER_CUSTOMMSG_H

#include <vector>
#include <memory>
#include <cstdint>
#include <sensor_msgs/Imu.h> // For std_msgs::Header via Imu or direct

namespace livox_ros_driver {

struct CustomPoint {
    double x;
    double y;
    double z;
    uint8_t reflectivity;
    uint8_t tag;
    uint8_t line;
    uint32_t offset_time;
};

struct CustomMsg {
    using Ptr = std::shared_ptr<CustomMsg>;
    using ConstPtr = std::shared_ptr<const CustomMsg>;
    
    std_msgs::Header header;
    uint64_t timebase;
    uint32_t point_num;
    uint8_t lidar_id;
    std::vector<CustomPoint> points;
};

}

#endif
