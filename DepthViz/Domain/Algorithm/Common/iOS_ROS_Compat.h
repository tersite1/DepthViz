//
//  iOS_ROS_Compat.h
//  DepthViz
//
//  Created by DepthViz iOS Compatibility Layer
//  Provides iOS-compatible replacements for ROS dependencies
//

#ifndef iOS_ROS_Compat_h
#define iOS_ROS_Compat_h

#include <memory>
#include <vector>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

// ============================================================================
// 1. ROS Time/Header Mock
// ============================================================================

namespace ros {
    struct Time {
        uint32_t sec = 0;
        uint32_t nsec = 0;
        
        Time() : sec(0), nsec(0) {}
        Time(uint32_t s, uint32_t n) : sec(s), nsec(n) {}
        
        static Time now() {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = now.time_since_epoch();
            auto secs = std::chrono::duration_cast<std::chrono::seconds>(duration);
            auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(duration) - 
                        std::chrono::duration_cast<std::chrono::nanoseconds>(secs);
            return Time(secs.count(), nsecs.count());
        }
        
        double toSec() const {
            return sec + nsec * 1e-9;
        }
    };

    struct Header {
        uint32_t seq = 0;
        ros::Time stamp;
        std::string frame_id;
        
        uint32_t stamp_sec = 0;  // For backward compatibility
        uint32_t stamp_nsec = 0;
    };
}

// ============================================================================
// 2. ROS Sensor Messages Mock (Imu, PointCloud2)
// ============================================================================

namespace sensor_msgs {
    
    struct Imu {
        ros::Header header;
        
        // Orientation (using Eigen)
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        double orientation_covariance[9] = {0};
        
        // Angular velocity
        Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
        double angular_velocity_covariance[9] = {0};
        
        // Linear acceleration
        Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
        double linear_acceleration_covariance[9] = {0};
        
        using Ptr = std::shared_ptr<Imu>;
        using ConstPtr = std::shared_ptr<const Imu>;
    };
    
    struct PointCloud2 {
        ros::Header header;
        
        uint32_t height = 0;
        uint32_t width = 0;
        std::vector<uint8_t> data;
        uint32_t point_step = 0;
        uint32_t row_step = 0;
        bool is_dense = true;
        
        using Ptr = std::shared_ptr<PointCloud2>;
        using ConstPtr = std::shared_ptr<const PointCloud2>;
    };
}

// ============================================================================
// 3. ROS NodeHandle Mock
// ============================================================================

namespace ros {
    class NodeHandle {
    public:
        NodeHandle() = default;
        NodeHandle(const std::string& ns) {}
        
        template<typename T>
        bool getParam(const std::string& key, T& value) const {
            // iOS app: return default values instead
            return false;
        }
        
        template<typename T>
        void setParam(const std::string& key, const T& value) {
            // iOS app: no-op
        }
    };
}

// ============================================================================
// 4. Custom Messages Mock (for Livox LiDAR)
// ============================================================================

namespace custom_messages {
    
    struct Point {
        float x = 0.f;
        float y = 0.f;
        float z = 0.f;
        float intensity = 0.f;
        uint8_t tag = 0;
    };
    
    struct CustomMsg {
        ros::Header header;
        uint32_t timebase = 0;
        uint32_t point_num = 0;
        std::vector<Point> points;
        
        using Ptr = std::shared_ptr<CustomMsg>;
        using ConstPtr = std::shared_ptr<const CustomMsg>;
    };
    
    using ImuConstPtr = std::shared_ptr<const sensor_msgs::Imu>;
    using CustomMsgConstPtr = std::shared_ptr<const CustomMsg>;
}

// ============================================================================
// 5. PCL Conversions Mock
// ============================================================================

namespace pcl_conversions {
    
    template<typename PointT>
    inline void fromPCLPointCloud2(const pcl::PCLPointCloud2& pcl_pc2,
                                  pcl::PointCloud<PointT>& pc) {
        // Simplified: just copy if types are compatible
        // Full implementation would handle field conversions
    }
    
    template<typename PointT>
    inline void toPCLPointCloud2(const pcl::PointCloud<PointT>& pc,
                                pcl::PCLPointCloud2& pcl_pc2) {
        // Simplified conversion
    }
    
    inline void fromROSMsg(const sensor_msgs::PointCloud2& ros_pc2,
                          pcl::PCLPointCloud2& pcl_pc2) {
        // Simplified: map ROS message to PCL format
    }
}

#endif /* iOS_ROS_Compat_h */
