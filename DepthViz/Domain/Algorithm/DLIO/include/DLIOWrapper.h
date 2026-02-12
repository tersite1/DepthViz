//
//  DLIOWrapper.h
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  Abstract: Objective-C++ Wrapper for DLIO C++ Core implementing ISLAMSystem.
//

#ifndef DLIOWrapper_h
#define DLIOWrapper_h

#include "../../Interfaces/ISLAMSystem.h"
#include <memory>

namespace dlio {
    class Odom;
}

class DLIOEngine;

class DLIOWrapper : public ISLAMSystem {
public:
    DLIOWrapper();
    ~DLIOWrapper() override;
    
    void init() override;
    void start() override;
    void stop() override;
    
    void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) override;
    void pushPointCloud(double timestamp, const std::vector<Point3D>& points) override;
    
    Eigen::Matrix4d getCurrentPose() override;
    std::vector<Point3D> getDisplayCloud() override;

private:
    std::shared_ptr<dlio::Odom> _odom;
    bool _isInitialized;
    std::shared_ptr<DLIOEngine> _engine;
};

#endif /* DLIOWrapper_h */
