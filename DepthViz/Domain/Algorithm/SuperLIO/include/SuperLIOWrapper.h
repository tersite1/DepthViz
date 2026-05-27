//
//  SuperLIOWrapper.h
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

#ifndef SuperLIOWrapper_h
#define SuperLIOWrapper_h

#include "../../Interfaces/ISLAMSystem.h"
#include <memory>

class SuperLIOEngine;

class SuperLIOWrapper : public ISLAMSystem {
public:
    SuperLIOWrapper();
    ~SuperLIOWrapper() override;
    
    void init() override;
    void start() override;
    void stop() override;
    
    void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) override;
    void pushPointCloud(double timestamp, const std::vector<Point3D>& points) override;
    
    Eigen::Matrix4d getCurrentPose() override;
    std::vector<Point3D> getDisplayCloud() override;

private:
    bool _isInitialized;
    std::shared_ptr<SuperLIOEngine> _engine;
};

#endif /* SuperLIOWrapper_h */
