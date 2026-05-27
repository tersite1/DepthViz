#ifndef STD_MSGS_HEADER_H
#define STD_MSGS_HEADER_H

#include <string>
#include "../ros/ros.h" // For ros::Time

namespace std_msgs {
struct Header {
    ros::Time stamp;
    std::string frame_id;
};
}

#endif
