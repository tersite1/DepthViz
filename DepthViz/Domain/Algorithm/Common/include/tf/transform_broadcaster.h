#ifndef TF_TRANSFORM_BROADCASTER_H
#define TF_TRANSFORM_BROADCASTER_H

#include <geometry_msgs/TransformStamped.h>

namespace tf {
class TransformBroadcaster {
public:
    void sendTransform(const geometry_msgs::TransformStamped& transform) {}
};
}

namespace tf2_ros {
using TransformBroadcaster = tf::TransformBroadcaster;
}

#endif
