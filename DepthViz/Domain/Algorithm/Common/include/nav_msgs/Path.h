#ifndef NAV_MSGS_PATH_H
#define NAV_MSGS_PATH_H

#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>

namespace nav_msgs {
struct Path {
    std_msgs::Header header;
    std::vector<geometry_msgs::PoseStamped> poses;
};
}

#endif
