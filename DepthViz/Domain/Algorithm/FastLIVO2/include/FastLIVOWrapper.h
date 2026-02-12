//
//  FastLIVOWrapper.h
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  Abstract: Objective-C++ Wrapper for Fast-LIVO2 C++ Core implementing ISLAMSystem.
//

#ifndef FastLIVOWrapper_h
#define FastLIVOWrapper_h

#include "../../Interfaces/ISLAMSystem.h"
#include <memory>

// Forward declaration
class FastLIVOEngine;

class FastLIVOWrapper : public ISLAMSystem {
public:
    FastLIVOWrapper();
    ~FastLIVOWrapper() override;
    
    void init() override;
    void start() override;
    void stop() override;
    
    void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) override;
    void pushPointCloud(double timestamp, const std::vector<Point3D>& points) override;
    void pushImage(double timestamp, const void* imageData, int width, int height) override;
    
    Eigen::Matrix4d getCurrentPose() override;
    std::vector<Point3D> getDisplayCloud() override;

private:
    std::shared_ptr<FastLIVOEngine> _engine;
    bool _isInitialized;
};

#endif /* FastLIVOWrapper_h */
