#ifndef GEOMETRY_MSGS_POSE_H
#define GEOMETRY_MSGS_POSE_H

#include "Point.h"
#include "Quaternion.h"

namespace geometry_msgs {
struct Pose {
    Point position;
    Quaternion orientation;
};
}

#endif
