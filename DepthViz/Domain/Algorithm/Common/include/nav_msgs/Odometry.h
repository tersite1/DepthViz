#ifndef NAV_MSGS_ODOMETRY_H
#define NAV_MSGS_ODOMETRY_H

#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"

namespace nav_msgs {
struct Odometry {
    std_msgs::Header header;
    std::string child_frame_id;
    geometry_msgs::PoseWithCovariance pose;
    geometry_msgs::TwistWithCovariance twist;
};
}

#endif
