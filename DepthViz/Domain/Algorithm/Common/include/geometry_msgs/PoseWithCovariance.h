#ifndef GEOMETRY_MSGS_POSEWITHCOVARIANCE_H
#define GEOMETRY_MSGS_POSEWITHCOVARIANCE_H

#include "Pose.h"

namespace geometry_msgs {
struct PoseWithCovariance {
    Pose pose;
    double covariance[36];
};
}

#endif
