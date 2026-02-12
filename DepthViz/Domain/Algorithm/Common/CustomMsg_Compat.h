//
//  CustomMsg_Compat.h
//  DepthViz
//
//  iOS-compatible replacement for livox_ros_driver/CustomMsg.h
//

#ifndef CustomMsg_Compat_h
#define CustomMsg_Compat_h

// We already define custom_messages in iOS_ROS_Compat.h
// Just include that instead
#include "iOS_ROS_Compat.h"

// For backward compatibility, define namespace with legacy name
namespace livox_ros_driver {
    using CustomMsg = custom_messages::CustomMsg;
    using CustomMsgConstPtr = custom_messages::CustomMsgConstPtr;
}

#endif /* CustomMsg_Compat_h */
