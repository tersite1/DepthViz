#ifndef VIKIT_PINHOLE_CAMERA_H
#define VIKIT_PINHOLE_CAMERA_H

#include "abstract_camera.h"

namespace vk {

class PinholeCamera : public AbstractCamera {
public:
    PinholeCamera(double fx, double fy, double cx, double cy, int width, int height)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), width_(width), height_(height) {}
        
    virtual ~PinholeCamera() {}
    
    Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const override {
        return Eigen::Vector2d(fx_ * xyz_c[0] / xyz_c[2] + cx_, fy_ * xyz_c[1] / xyz_c[2] + cy_);
    }
    
    Eigen::Vector3d cam2world(const double x, const double y) const override {
        double mx = (x - cx_) / fx_;
        double my = (y - cy_) / fy_;
        Eigen::Vector3d vec(mx, my, 1.0);
        return vec.normalized();
    }
    
    bool isInFrame(const Eigen::Vector2i& obs, int boundary) const override {
        return obs[0] >= boundary && obs[0] < width_ - boundary &&
               obs[1] >= boundary && obs[1] < height_ - boundary;
    }
    
    double errorMultiplier2() const override { return 1.0; } // Stub

private:
    double fx_, fy_, cx_, cy_;
    int width_, height_;
};

} // namespace vk

#endif
