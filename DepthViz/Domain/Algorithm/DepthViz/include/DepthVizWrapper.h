#ifndef DepthVizWrapper_h
#define DepthVizWrapper_h

#include "../../Interfaces/ISLAMSystem.h"
#include <memory>
#include <vector>

class DepthVizEngine;

class DepthVizWrapper : public ISLAMSystem {
public:
    DepthVizWrapper();
    ~DepthVizWrapper() override;
    
    void init() override;
    void start() override;
    void stop() override;
    
    void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) override;
    void pushPointCloud(double timestamp, const std::vector<Point3D>& points) override;
    void pushImage(double timestamp, const void* imageData, int width, int height) override;
    
    Eigen::Matrix4d getCurrentPose() override;
    std::vector<Point3D> getDisplayCloud() override;
    
    // Returns full optimized map for export
    std::vector<Point3D> getFullMap();

    // ARKit pose prior
    void pushARKitPose(double timestamp, const Eigen::Matrix4d& pose);

    // Specialized method for iOS raw buffers (with RGB from camera)
    void pushPointCloudRaw(double timestamp, const float* xyz, const uint8_t* conf, const uint8_t* rgb, int count);

private:
    std::shared_ptr<DepthVizEngine> _engine;
};

#endif /* DepthVizWrapper_h */
