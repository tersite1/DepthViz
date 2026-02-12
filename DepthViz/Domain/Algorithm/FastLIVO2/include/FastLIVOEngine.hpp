//
//  FastLIVOEngine.hpp
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

#ifndef FastLIVOEngine_hpp
#define FastLIVOEngine_hpp

#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <condition_variable>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv.hpp>

// Fast-LIVO2 Includes
#include "msgs.h" // For custom_messages definitions
#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
// #include "LIVMapper.h" // Removed to avoid ROS dependencies
#include "voxel_map.h" // Added explicit include if needed

struct AppPoint3D {
    float x, y, z;
    float intensity;
    double timestamp;
};

class FastLIVOEngine {
public:
    FastLIVOEngine();
    ~FastLIVOEngine();

    void init();
    void start();
    void stop();

    void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr);
    void pushPointCloud(double timestamp, const std::vector<AppPoint3D>& points);
    void pushImage(double timestamp, const cv::Mat& image);

    Eigen::Matrix4d getPose();
    std::vector<AppPoint3D> getDisplayCloud();

private:
    void run();
    bool sync_packages(LidarMeasureGroup &meas);
    void processImu();
    void stateEstimationAndMapping();
    void handleVIO();
    void handleLIO();
    void gravityAlignment();
    void handleFirstFrame();
    
    // Internal State
    StatesGroup _state;
    StatesGroup state_propagat;
    LidarMeasureGroup LidarMeasures;
    
    std::shared_ptr<Preprocess> p_pre;
    std::shared_ptr<ImuProcess> p_imu;
    std::shared_ptr<VoxelMapManager> voxelmap_manager;
    std::shared_ptr<VIOManager> vio_manager;
    
    std::atomic<bool> _isRunning;
    std::unique_ptr<std::thread> _processThread;
    
    // Data Buffers
    std::deque<custom_messages::ImuConstPtr> imu_buffer;
    
    std::deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
    std::deque<double> lid_header_time_buffer;
    std::deque<cv::Mat> img_buffer;
    std::deque<double> img_time_buffer;
    
    std::mutex mtx_buffer;
    std::condition_variable sig_buffer;
    
    // Flags and Params
    bool lidar_pushed;
    double last_timestamp_lidar;
    double last_timestamp_imu;
    double last_timestamp_img;
    
    bool imu_en;
    bool img_en;
    bool lidar_en;
    
    SLAM_MODE slam_mode_;
    
    // Output
    Eigen::Matrix4d _currentPose;
    std::vector<AppPoint3D> _displayCloud;
    std::mutex _mtxOutput;
};

#endif /* FastLIVOEngine_hpp */
