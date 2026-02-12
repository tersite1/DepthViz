//
//  FastLIO2Wrapper.h
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

#ifndef FastLIO2Wrapper_h
#define FastLIO2Wrapper_h

#include "../../Interfaces/ISLAMSystem.h"
#include <memory>

class FastLIOEngine;

class FastLIO2Wrapper : public ISLAMSystem {
public:
    FastLIO2Wrapper();
    ~FastLIO2Wrapper() override;
    
    void init() override;
    void start() override;
    void stop() override;
    
    void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) override;
    void pushPointCloud(double timestamp, const std::vector<Point3D>& points) override;
    
    Eigen::Matrix4d getCurrentPose() override;
    std::vector<Point3D> getDisplayCloud() override;

private:
    std::shared_ptr<FastLIOEngine> _engine;
};

#endif /* FastLIO2Wrapper_h */
