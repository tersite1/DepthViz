#ifndef VIKIT_ABSTRACT_CAMERA_H
#define VIKIT_ABSTRACT_CAMERA_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace vk {

class AbstractCamera {
public:
    virtual ~AbstractCamera() {}
    
    // Project 3D point in camera frame to pixel coordinates
    virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const = 0;
    
    // Back-project pixel coordinates to unit vector in camera frame
    virtual Eigen::Vector3d cam2world(const double x, const double y) const = 0;
    
    // Check if point is in frame with border
    virtual bool isInFrame(const Eigen::Vector2i& obs, int boundary) const = 0;
    
    virtual double errorMultiplier2() const = 0; // Guessing needed by robust cost?
};

} // namespace vk

#endif
