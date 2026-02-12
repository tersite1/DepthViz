#ifndef GEOMETRY_MSGS_TRANSFORMSTAMPED_H
#define GEOMETRY_MSGS_TRANSFORMSTAMPED_H

#include "std_msgs/Header.h"
#include "Transform.h"

namespace geometry_msgs {
struct TransformStamped {
    std_msgs::Header header;
    std::string child_frame_id;
    Transform transform;
};
}

#endif
