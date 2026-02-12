#ifndef GEOMETRY_MSGS_POSESTAMPED_H
#define GEOMETRY_MSGS_POSESTAMPED_H

#include "std_msgs/Header.h"
#include "Pose.h"

namespace geometry_msgs {
struct PoseStamped {
    std_msgs::Header header;
    Pose pose;
};
}

#endif
