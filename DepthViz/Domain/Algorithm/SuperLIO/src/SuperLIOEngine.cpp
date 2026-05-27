//
//  SuperLIOEngine.cpp
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

#include "../include/SuperLIOEngine.hpp"
#include <iostream>

#ifdef USE_LEGACY_SUPER_LIO

using namespace LI2Sup;
using namespace BASIC;

SuperLIOEngine::SuperLIOEngine() {
// ... rest of file ...
    _core = std::make_shared<SuperLIOCore>();
    _lidarPushed = false;
    _lastTimestampImu = -1.0;
    _lastTimestampLidar = -1.0;
    _isRunning = false;
    _currentPose = Eigen::Matrix4d::Identity();
}

SuperLIOEngine::~SuperLIOEngine() {
    stop();
}

void SuperLIOEngine::init() {
    _core->setDataWrapper(std::shared_ptr<IDataWrapper>(this, [](IDataWrapper*){})); 
    
    _core->init();
}

void SuperLIOEngine::start() {
    if (_isRunning) return;
    _isRunning = true;
    _processThread = std::make_unique<std::thread>(&SuperLIOEngine::run, this);
}

void SuperLIOEngine::stop() {
    _isRunning = false;
    if (_processThread && _processThread->joinable()) {
        _processThread->join();
    }
}

void SuperLIOEngine::run() {
    while (_isRunning) {
        _core->process();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SuperLIOEngine::pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
    IMUData data;
    data.secs = timestamp;
    data.acc = V3(acc.x(), acc.y(), acc.z());
    data.gyr = V3(gyr.x(), gyr.y(), gyr.z());

    std::lock_guard<std::mutex> lock(_mtxBuffers);
    if (data.secs < _lastTimestampImu) {
        _imuBuffer.clear();
        _lastTimestampImu = data.secs;
    }
    _imuBuffer.push_back(data);
    _lastTimestampImu = data.secs;
}

void SuperLIOEngine::pushPointCloud(double timestamp, const std::vector<AppPoint3D>& points) {
    LidarData data;
    data.start_time = timestamp;
    // Estimate end time or use same? Usually end_time = start + duration.
    // Assuming instant scan for now or calculate from points if they have relative time.
    // AppPoint3D has timestamp (absolute?).
    // If AppPoint3D timestamp is relative to start, use it.
    
    double max_offset = 0;
    
    data.pc.reset(new pcl::PointCloud<PointXTZIT>());
    data.pc->reserve(points.size());
    
    for (const auto& p : points) {
        PointXTZIT pt;
        pt.x = p.x; pt.y = p.y; pt.z = p.z;
        pt.intensity = p.intensity;
        // Check if timestamp is relative or absolute
        // If absolute, offset = p.timestamp - timestamp
        // If p.timestamp is close to timestamp (e.g. 1.6e9), it's absolute.
        // Assuming absolute for now.
        double offset = p.timestamp - timestamp;
        if (offset < 0) offset = 0; // Should not happen if sorted
        pt.offset_time = offset;
        if (offset > max_offset) max_offset = offset;
        
        data.pc->push_back(pt);
    }
    data.end_time = timestamp + max_offset;

    std::lock_guard<std::mutex> lock(_mtxBuffers);
    _lidarBuffer.push_back(data);
}

bool SuperLIOEngine::sync_measure(MeasureGroup& meas) {
    std::lock_guard<std::mutex> lock(_mtxBuffers);
    if (_lidarBuffer.empty() || _imuBuffer.empty()) {
        return false;
    }

    if (!_lidarPushed) {
        meas.lidar = _lidarBuffer.front();
        _lidarPushed = true;
    }

    if (_lastTimestampLidar > meas.lidar.end_time) {
        _lidarBuffer.pop_front();
        _lidarPushed = false;
        return false;
    }

    if (_lastTimestampImu < meas.lidar.end_time) {
        return false;
    }

    double imu_time = _imuBuffer.front().secs;
    meas.imu.clear();
    while ((!_imuBuffer.empty()) && (imu_time < meas.lidar.end_time)) {
        imu_time = _imuBuffer.front().secs;
        if (imu_time > meas.lidar.end_time) break;
        meas.imu.push_back(_imuBuffer.front());
        _imuBuffer.pop_front();
    }

    _lastTimestampLidar = meas.lidar.end_time;
    _lidarBuffer.pop_front();
    _lidarPushed = false;
    return true;
}

void SuperLIOEngine::setESKF(ESKF::Ptr& eskf) {
    _eskf = eskf;
}

void SuperLIOEngine::pub_odom(const NavState& state) {
    std::lock_guard<std::mutex> lock(_mtxOutput);
    
    _currentPose.setIdentity();
    _currentPose.block<3,3>(0,0) = state.R.R_.cast<double>();
    _currentPose.block<3,1>(0,3) = state.p.cast<double>();
}

void SuperLIOEngine::pub_cloud_world(const BASIC::CloudPtr& pc, double time) {
    std::lock_guard<std::mutex> lock(_mtxOutput);
    
    _displayCloud.clear();
    _displayCloud.reserve(pc->size());
    for (const auto& p : pc->points) {
        AppPoint3D ap;
        ap.x = p.x;
        ap.y = p.y;
        ap.z = p.z;
        ap.intensity = p.intensity;
        ap.timestamp = time; // Use frame time?
        _displayCloud.push_back(ap);
    }
}

void SuperLIOEngine::pub_cloud2planner(const BASIC::CloudPtr& pc, double time) {
    // Optional
}

Eigen::Matrix4d SuperLIOEngine::getPose() {
    std::lock_guard<std::mutex> lock(_mtxOutput);
    return _currentPose;
}

std::vector<AppPoint3D> SuperLIOEngine::getDisplayCloud() {
    std::lock_guard<std::mutex> lock(_mtxOutput);
    return _displayCloud;
}

#endif
