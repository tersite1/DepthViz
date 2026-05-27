#ifndef GEOMETRY_MSGS_TWIST_H
#define GEOMETRY_MSGS_TWIST_H

#include "Vector3.h"

namespace geometry_msgs {
struct Twist {
    Vector3 linear;
    Vector3 angular;
};
}

#endif
