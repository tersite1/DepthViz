#ifndef SENSOR_MSGS_POINTCLOUD2_H
#define SENSOR_MSGS_POINTCLOUD2_H

#include "std_msgs/Header.h"
#include <vector>

namespace sensor_msgs {

struct PointCloud2 {
    using Ptr = std::shared_ptr<PointCloud2>;
    using ConstPtr = std::shared_ptr<const PointCloud2>;
    
    std_msgs::Header header;
    std::vector<unsigned char> data;
    // Add fields if needed
};

}

#endif
