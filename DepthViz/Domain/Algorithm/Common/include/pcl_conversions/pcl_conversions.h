#ifndef PCL_CONVERSIONS_H
#define PCL_CONVERSIONS_H

#include <pcl/PCLHeader.h>
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>

namespace pcl_conversions {

inline void toPCL(const std_msgs::Header& header, pcl::PCLHeader& pcl_header) {
    pcl_header.stamp = static_cast<uint64_t>(header.stamp.toSec() * 1e6);
    pcl_header.frame_id = header.frame_id;
}

inline void fromPCL(const pcl::PCLHeader& pcl_header, std_msgs::Header& header) {
    header.stamp = ros::Time::fromSec(static_cast<double>(pcl_header.stamp) / 1e6);
    header.frame_id = pcl_header.frame_id;
}

template<typename T>
inline void toROSMsg(const pcl::PointCloud<T>& pcl_cloud, sensor_msgs::PointCloud2& msg) {
    // Stub
}

template<typename T>
inline void fromROSMsg(const sensor_msgs::PointCloud2& msg, pcl::PointCloud<T>& pcl_cloud) {
    // Stub
}

}

namespace pcl {
    // Some codes use pcl::toROSMsg directly if they include pcl_ros/point_cloud.h
    template<typename T>
    inline void toROSMsg(const pcl::PointCloud<T>& pcl_cloud, sensor_msgs::PointCloud2& msg) {
        pcl_conversions::toROSMsg(pcl_cloud, msg);
    }
    
    template<typename T>
    inline void fromROSMsg(const sensor_msgs::PointCloud2& msg, pcl::PointCloud<T>& pcl_cloud) {
        pcl_conversions::fromROSMsg(msg, pcl_cloud);
    }
}

#endif
