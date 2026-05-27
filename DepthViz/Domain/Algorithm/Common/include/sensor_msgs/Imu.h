#ifndef SENSOR_MSGS_IMU_H
#define SENSOR_MSGS_IMU_H

#include <memory>
#include <vector>
#include <string>
#include "std_msgs/Header.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

namespace sensor_msgs {

struct Imu {
    using Ptr = std::shared_ptr<Imu>;
    using ConstPtr = std::shared_ptr<const Imu>;
    
    std_msgs::Header header;
    
    geometry_msgs::Quaternion orientation;
    geometry_msgs::Vector3 angular_velocity;
    geometry_msgs::Vector3 linear_acceleration;
};

}

#endif
