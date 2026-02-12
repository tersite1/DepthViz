//
//  DLIOEngine.hpp
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  Abstract: Core C++ Engine class for DLIO, separated from ROS.
//

#ifndef DLIOEngine_hpp
#define DLIOEngine_hpp

#define PCL_NO_PRECOMPILE

#ifdef USE_LEGACY_DLIO

#include <atomic>
#include <ctime>
#include <mutex>
#include <thread>
#include <vector>
#include <deque>
#include <future>
#include <boost/circular_buffer.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
// ... existing imports ...
#include "nano_gicp/nano_gicp.h"

// Define PointType
namespace dlio {
// ... rest of file ...
  enum class SensorType { OUSTER, VELODYNE, HESAI, LIVOX, UNKNOWN };

  struct Point {
    Point(): data{0.f, 0.f, 0.f, 1.f} {}

    PCL_ADD_POINT4D;
    float intensity; // intensity
    union {
    std::uint32_t t;   // (Ouster) time since beginning of scan in nanoseconds
    float time;        // (Velodyne) time since beginning of scan in seconds
    double timestamp;  // (Hesai) absolute timestamp in seconds
                       // (Livox) absolute timestamp in (seconds * 10e9)
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  }; // EIGEN_ALIGN16; - Removed EIGEN_ALIGN16 as it's deprecated/problematic with C++17 aligned allocation
}

POINT_CLOUD_REGISTER_POINT_STRUCT(dlio::Point,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (std::uint32_t, t, t)
                                 (float, time, time)
                                 (double, timestamp, timestamp))

typedef dlio::Point PointType;

struct AppPoint3D {
    float x, y, z;
    float intensity;
    double timestamp;
};

class DLIOEngine {
public:
    DLIOEngine();
    ~DLIOEngine();

    void init();
    void start();
    void stop();

    void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr);
    void pushPointCloud(double timestamp, const std::vector<AppPoint3D>& points);

    Eigen::Matrix4d getPose();
    std::vector<AppPoint3D> getDisplayCloud();

private:
    // Internal Structures
    struct ImuMeas {
        double stamp;
        double dt;
        Eigen::Vector3f ang_vel;
        Eigen::Vector3f lin_accel;
    };
    
    struct ImuBias {
        Eigen::Vector3f gyro;
        Eigen::Vector3f accel;
    };

    struct Frames {
        Eigen::Vector3f b;
        Eigen::Vector3f w;
    };

    struct Velocity {
        Frames lin;
        Frames ang;
    };

    struct State {
        Eigen::Vector3f p; // position in world frame
        Eigen::Quaternionf q; // orientation in world frame
        Velocity v;
        ImuBias b; // imu biases in body frame
    };
    
    struct Pose {
        Eigen::Vector3f p;
        Eigen::Quaternionf q;
    };
    
    struct Geo {
        bool first_opt_done;
        std::mutex mtx;
        double dp;
        double dq_deg;
        Eigen::Vector3f prev_p;
        Eigen::Quaternionf prev_q;
        Eigen::Vector3f prev_vel;
    };

    struct Extrinsics {
        struct SE3 {
            Eigen::Vector3f t;
            Eigen::Matrix3f R;
        };
        SE3 baselink2imu;
        SE3 baselink2lidar;
        Eigen::Matrix4f baselink2imu_T;
        Eigen::Matrix4f baselink2lidar_T;
    };
    
    struct Metrics {
        std::vector<float> spaciousness;
        std::vector<float> density;
    };

    // Internal Methods
    void getParams();
    void preprocessPoints();
    void deskewPointcloud();
    void initializeInputTarget();
    void setInputSource();
    void initializeDLIO();
    void getNextPose();
    
    bool imuMeasFromTimeRange(double start_time, double end_time,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it);
                            
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                 const std::vector<double>& sorted_timestamps);
                 
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                         const std::vector<double>& sorted_timestamps,
                         boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                         boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);
                         
    void propagateGICP();
    void propagateState();
    void updateState();
    void setAdaptiveParams();
    void updateKeyframes();
    void buildKeyframesAndSubmap(State vehicle_state);
    void computeMetrics();
    void debug();

    // Member Variables
    std::atomic<bool> _isRunning;
    
    // Core DLIO State
    State state;
    Pose lidarPose;
    Pose imuPose;
    Geo geo;
    Extrinsics extrinsics;
    Metrics metrics;
    
    Eigen::Matrix4f T, T_prior, T_corr;
    Eigen::Quaternionf q_final;
    Eigen::Vector3f origin;
    
    // Timestamps
    double scan_stamp;
    double prev_scan_stamp;
    double scan_dt;
    double first_scan_stamp;
    double elapsed_time;
    
    // IMU
    double imu_stamp;
    double first_imu_stamp;
    double prev_imu_stamp;
    
    ImuMeas imu_meas;
    boost::circular_buffer<ImuMeas> imu_buffer;
    std::mutex mtx_imu;
    std::condition_variable cv_imu_stamp;
    
    // Point Clouds
    pcl::PointCloud<PointType>::Ptr original_scan;
    pcl::PointCloud<PointType>::Ptr deskewed_scan;
    pcl::PointCloud<PointType>::Ptr current_scan;
    pcl::PointCloud<PointType>::Ptr submap_cloud;
    
    // Keyframes
    std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                          pcl::PointCloud<PointType>::ConstPtr>> keyframes;
    std::vector<double> keyframe_timestamps;
    std::vector<std::shared_ptr<const nano_gicp::CovarianceList>> keyframe_normals;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> keyframe_transformations;
    std::mutex keyframes_mutex;
    
    // Submap
    std::shared_ptr<const nano_gicp::CovarianceList> submap_normals;
    std::shared_ptr<const nanoflann::KdTreeFLANN<PointType>> submap_kdtree;
    std::vector<int> submap_kf_idx_curr;
    std::vector<int> submap_kf_idx_prev;
    bool new_submap_is_ready;
    std::future<void> submap_future;
    std::condition_variable submap_build_cv;
    bool main_loop_running;
    std::mutex main_loop_running_mutex;
    bool submap_hasChanged;
    
    // Algorithms
    nano_gicp::NanoGICP<PointType, PointType> gicp;
    nano_gicp::NanoGICP<PointType, PointType> gicp_temp;
    pcl::CropBox<PointType> crop;
    pcl::VoxelGrid<PointType> voxel;
    // pcl::ConvexHull<PointType> convex_hull;
    // pcl::ConcaveHull<PointType> concave_hull;
    
    // Params
    std::string version_;
    int num_threads_;
    bool verbose;
    bool deskew_;
    double gravity_;
    bool time_offset_;
    bool adaptive_params_;
    double obs_submap_thresh_;
    double obs_keyframe_thresh_;
    double obs_keyframe_lag_;
    double keyframe_thresh_dist_;
    double keyframe_thresh_rot_;
    int submap_knn_;
    int submap_kcv_;
    int submap_kcc_;
    double submap_concave_alpha_;
    bool densemap_filtered_;
    bool wait_until_move_;
    double crop_size_;
    bool vf_use_;
    double vf_res_;
    bool imu_calibrate_;
    bool calibrate_gyro_;
    bool calibrate_accel_;
    bool gravity_align_;
    double imu_calib_time_;
    int imu_buffer_size_;
    Eigen::Matrix3f imu_accel_sm_;
    int gicp_min_num_points_;
    int gicp_k_correspondences_;
    double gicp_max_corr_dist_;
    int gicp_max_iter_;
    double gicp_transformation_ep_;
    double gicp_rotation_ep_;
    double gicp_init_lambda_factor_;
    double geo_Kp_;
    double geo_Kv_;
    double geo_Kq_;
    double geo_Kab_;
    double geo_Kgb_;
    double geo_abias_max_;
    double geo_gbias_max_;
    
    // Flags
    std::atomic<bool> dlio_initialized;
    std::atomic<bool> first_valid_scan;
    std::atomic<bool> first_imu_received;
    std::atomic<bool> imu_calibrated;
    std::atomic<bool> gicp_hasConverged;
    std::atomic<bool> deskew_status;
    std::atomic<int> deskew_size;
    
    dlio::SensorType sensor;
    double length_traversed;
    
    // Output Buffer
    std::vector<AppPoint3D> _displayCloud;
    std::mutex _mtxCloud;
    Eigen::Matrix4d _currentPose;
    std::mutex _mtxPose;
};

#endif

#endif /* DLIOEngine_hpp */
