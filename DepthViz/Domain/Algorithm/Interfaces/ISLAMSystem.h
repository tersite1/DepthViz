#ifndef ISLAMSystem_h
#define ISLAMSystem_h

#include <vector>
#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Dense>

// Common Point Structure
struct Point3D {
    float x, y, z;
    float intensity;
    uint8_t r, g, b;  // RGB color from camera
    double timestamp;
};

// Abstract Interface
class ISLAMSystem {
public:
    virtual ~ISLAMSystem() {}

    // Lifecycle
    virtual void init() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

    // Unified Input
    virtual void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) = 0;

    virtual void pushPointCloud(double timestamp, const std::vector<Point3D>& points) = 0;

    // Optional: For Visual SLAM (Fast-LIVO2)
    virtual void pushImage(double timestamp, const void* imageData, int width, int height) {}

    // Unified Output
    virtual Eigen::Matrix4d getCurrentPose() = 0;

    virtual std::vector<Point3D> getDisplayCloud() = 0;

    // Optional: Get full map for export (default empty)
    virtual std::vector<Point3D> getFullMap() { return {}; }
};

#endif /* ISLAMSystem_h */
