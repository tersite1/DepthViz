//
//  FastLIOEngine.hpp
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  Abstract: Core C++ Engine class for Fast-LIO2, separated from ROS.
//

#ifndef FastLIOEngine_hpp
#define FastLIOEngine_hpp

#define PCL_NO_PRECOMPILE
#define USE_LEGACY_FAST_LIO2 // Enable legacy headers

#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <memory>
#include <atomic>

// Headers should be found via Header Search Paths in Xcode
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h> // Added missing include

#include "ikd-Tree/ikd_Tree.h"
#include "common_lib.h"
#include "use-ikfom.hpp"
#include "preprocess.h"
#include "IMU_Processing.hpp"

struct AppPoint3D {
    float x, y, z;
    float intensity;
    double timestamp;
};

class FastLIOEngine {
public:
    FastLIOEngine();
    ~FastLIOEngine();

    void init();
    void start();
    void stop();

    void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr);
    void pushPointCloud(double timestamp, const std::vector<AppPoint3D>& points);

    Eigen::Matrix4d getPose();
    std::vector<AppPoint3D> getDisplayCloud();
    
    // Public for IKFOM callback access
    void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

private:
    void run();
    bool syncPackages(MeasureGroup &meas);
    void mapIncremental();
    void pointBodyToWorld(PointType const * const pi, PointType * const po);

    std::atomic<bool> _isRunning;
    std::unique_ptr<std::thread> _processThread;
    
    std::deque<custom_messages::ImuPtr> _imuBuffer;
    std::deque<PointCloudXYZI::Ptr> _lidarBuffer;
    std::deque<double> _timeBuffer;
    
    std::mutex _mtxBuffer;
    
    std::shared_ptr<Preprocess> _preProcessor;
    std::shared_ptr<ImuProcess> _imuProcessor;
    
    esekfom::esekf<state_ikfom, 12, input_ikfom> _kf;
    state_ikfom _statePoint;
    
    KD_TREE<PointType> _ikdTree;
    
    Eigen::Matrix4d _currentPose;
    std::mutex _mtxPose;
    
    std::vector<AppPoint3D> _displayCloud;
    std::mutex _mtxCloud;
    
    bool _firstLidar;
    double _firstLidarTime;
    MeasureGroup _measures;
    
    const int kMaxIterations = 4;
    // const int NUM_MATCH_POINTS = 5; // Defined as macro in common_lib.h
    
    std::vector<PointVector> _nearestPoints;
    std::vector<std::vector<int>> _pointSearchIndSurf;
    
    PointCloudXYZI::Ptr _featsFromMap;
    PointCloudXYZI::Ptr _featsUndistort;
    PointCloudXYZI::Ptr _featsDownBody;
    PointCloudXYZI::Ptr _featsDownWorld;
    PointCloudXYZI::Ptr _normVec;
    PointCloudXYZI::Ptr _laserCloudOri;
    PointCloudXYZI::Ptr _corrNormVect;
    
    pcl::VoxelGrid<PointType> _downSizeFilterSurf;
};

#endif /* FastLIOEngine_hpp */
