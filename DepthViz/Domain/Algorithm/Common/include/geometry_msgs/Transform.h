#ifndef GEOMETRY_MSGS_TRANSFORM_H
#define GEOMETRY_MSGS_TRANSFORM_H

#include "Vector3.h"
#include "Quaternion.h"

namespace geometry_msgs {
struct Transform {
    Vector3 translation;
    Quaternion rotation;
};
}

#endif
