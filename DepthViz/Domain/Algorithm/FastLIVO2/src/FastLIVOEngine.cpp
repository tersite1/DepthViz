//
//  FastLIVOEngine.cpp
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  iOS-compatible LIVO2 SLAM Engine
//

#include "../include/FastLIVOEngine.hpp"
#include "../../../Common/iOS_ROS_Compat.h"
#include <iostream>
#include <iomanip>

// Define macros if they are missing
#ifndef ROOT_DIR
#define ROOT_DIR "./"
#endif

#ifndef DEBUG_FILE_DIR
#define DEBUG_FILE_DIR(name) (std::string(ROOT_DIR) + "Log/" + name)
#endif

FastLIVOEngine::FastLIVOEngine() {
    _currentPose = Eigen::Matrix4d::Identity();
    _isRunning = false;
    lidar_pushed = false;
    last_timestamp_lidar = -1.0;
    last_timestamp_imu = -1.0;
    last_timestamp_img = -1.0;
    
    imu_en = true;
    img_en = true;
    lidar_en = true;
    
    slam_mode_ = SLAM_MODE::LIO; // Default to LIO mode
    
    // Initialize components
    p_pre = std::make_shared<Preprocess>();
    p_imu = std::make_shared<ImuProcess>();
    
    // Initialize managers if available
    // voxelmap_manager and vio_manager are optional for iOS
}

FastLIVOEngine::~FastLIVOEngine() {
    stop();
}

void FastLIVOEngine::init() {
    // Initialization logic from LIVMapper::initializeComponents
    if (p_pre) {
        // p_pre->lidarType = LidarType::DEFAULT; // Set based on config
    }
    if (p_imu) {
        p_imu->set_gyr_cov(Eigen::Vector3d(0.01, 0.01, 0.01));
        p_imu->set_acc_cov(Eigen::Vector3d(0.1, 0.1, 0.1));
        p_imu->set_gyr_bias_cov(Eigen::Vector3d(0.0001, 0.0001, 0.0001));
        p_imu->set_acc_bias_cov(Eigen::Vector3d(0.0001, 0.0001, 0.0001));
    }
}

void FastLIVOEngine::start() {
    if (_isRunning) return;
    _isRunning = true;
    _processThread = std::make_unique<std::thread>(&FastLIVOEngine::run, this);
}

void FastLIVOEngine::stop() {
    _isRunning = false;
    if (_processThread && _processThread->joinable()) {
        _processThread->join();
    }
}

void FastLIVOEngine::pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu());
    msg->header.stamp.sec = (uint32_t)timestamp;
    msg->header.stamp_nsec = (uint32_t)((timestamp - (uint32_t)timestamp) * 1e9);
    msg->header.stamp_sec = (uint32_t)timestamp;
    msg->linear_acceleration = acc;
    msg->angular_velocity = gyr;
    
    std::lock_guard<std::mutex> lock(mtx_buffer);
    imu_buffer.push_back(msg);
    sig_buffer.notify_all();
}

void FastLIVOEngine::pushPointCloud(double timestamp, const std::vector<AppPoint3D>& points) {
    // Convert to PointCloudXYZI
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    ptr->reserve(points.size());
    for(const auto& p : points) {
        PointType pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = p.intensity;
        ptr->push_back(pt);
    }
    
    std::lock_guard<std::mutex> lock(mtx_buffer);
    lid_raw_data_buffer.push_back(ptr);
    lid_header_time_buffer.push_back(timestamp);
    sig_buffer.notify_all();
}

void FastLIVOEngine::pushImage(double timestamp, const cv::Mat& image) {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    img_buffer.push_back(image.clone());
    img_time_buffer.push_back(timestamp);
    sig_buffer.notify_all();
}

void FastLIVOEngine::run() {
    while (_isRunning) {
        if (!sync_packages(LidarMeasures)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        
        handleFirstFrame();
        processImu();
        stateEstimationAndMapping();
    }
}

bool FastLIVOEngine::sync_packages(LidarMeasureGroup &meas) {
    // Simplified sync logic
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (lid_raw_data_buffer.empty()) return false;
    
    meas.lidar = lid_raw_data_buffer.front();
    meas.lidar_frame_beg_time = lid_header_time_buffer.front();
    meas.lidar_frame_end_time = meas.lidar_frame_beg_time + 0.1; // Assume 100ms
    
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    
    return true;
}

void FastLIVOEngine::handleFirstFrame() {
    // Stub: Initialization on first frame
}

void FastLIVOEngine::processImu() {
    // Call p_imu->Process if available
    if (p_imu) {
        // p_imu->Process(LidarMeasures, _state, state_propagat);
    }
}

void FastLIVOEngine::stateEstimationAndMapping() {
    // Update pose and point cloud output
    std::lock_guard<std::mutex> lock(_mtxOutput);
    
    // Stub: Keep identity for now
    // In full implementation:
    // - Run voxelmap_manager for mapping
    // - Update _currentPose from state
    // - Extract display cloud from voxel map
}

void FastLIVOEngine::handleVIO() {
    // Handle Visual-Inertial Odometry
}

void FastLIVOEngine::handleLIO() {
    // Handle LiDAR-Inertial Odometry
}

void FastLIVOEngine::gravityAlignment() {
    // Align gravity vector with Z-axis
}

Eigen::Matrix4d FastLIVOEngine::getPose() {
    std::lock_guard<std::mutex> lock(_mtxOutput);
    return _currentPose;
}

std::vector<AppPoint3D> FastLIVOEngine::getDisplayCloud() {
    std::lock_guard<std::mutex> lock(_mtxOutput);
    return _displayCloud;
}
