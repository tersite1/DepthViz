//
//  SuperLIOEngine.hpp
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//
//

#ifndef SuperLIOEngine_hpp
#define SuperLIOEngine_hpp

#ifdef USE_LEGACY_SUPER_LIO

#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "../include/IDataWrapper.h"
#include "../include/SuperLIOCore.h"

struct AppPoint3D {
    float x, y, z;
    float intensity;
    double timestamp;
};

class SuperLIOEngine : public LI2Sup::IDataWrapper {
public:
    SuperLIOEngine();
    ~SuperLIOEngine();

    void init();
    void start();
    void stop();

    void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr);
    void pushPointCloud(double timestamp, const std::vector<AppPoint3D>& points);

    Eigen::Matrix4d getPose();
    std::vector<AppPoint3D> getDisplayCloud();

    // IDataWrapper overrides
    bool sync_measure(LI2Sup::MeasureGroup& meas) override;
    void setESKF(LI2Sup::ESKF::Ptr& eskf) override;
    void pub_odom(const LI2Sup::NavState& state) override;
    void pub_cloud_world(const BASIC::CloudPtr& pc, double time) override;
    void pub_cloud2planner(const BASIC::CloudPtr& pc, double time) override;

private:
    void run();

    std::shared_ptr<LI2Sup::SuperLIOCore> _core;
    LI2Sup::ESKF::Ptr _eskf;

    std::atomic<bool> _isRunning;
    std::unique_ptr<std::thread> _processThread;

    std::deque<LI2Sup::IMUData> _imuBuffer;
    std::deque<LI2Sup::LidarData> _lidarBuffer;
    std::mutex _mtxBuffers;

    bool _lidarPushed;
    double _lastTimestampImu;
    double _lastTimestampLidar;

    // Output State
    Eigen::Matrix4d _currentPose;
    std::vector<AppPoint3D> _displayCloud;
    std::mutex _mtxOutput;
};

#endif

#endif /* SuperLIOEngine_hpp */
