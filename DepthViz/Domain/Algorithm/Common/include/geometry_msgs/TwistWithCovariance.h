#ifndef GEOMETRY_MSGS_TWISTWITHCOVARIANCE_H
#define GEOMETRY_MSGS_TWISTWITHCOVARIANCE_H

#include "Twist.h"

namespace geometry_msgs {
struct TwistWithCovariance {
    Twist twist;
    double covariance[36];
};
}

#endif
