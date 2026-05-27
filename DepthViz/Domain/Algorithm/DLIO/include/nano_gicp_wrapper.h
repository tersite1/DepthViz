//
//  nano_gicp_wrapper.h
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  Abstract: Lightweight GICP wrapper to replace GTSAM dependency in DLIO.
//

#ifndef NANO_GICP_WRAPPER_H
#define NANO_GICP_WRAPPER_H

#include <vector>

#if __has_include("Eigen/Core")
#include "Eigen/Core"
#include "Eigen/Geometry"
#else
namespace Eigen {
    struct Vector3f { float x, y, z; };
    struct Matrix4f { float data[16]; };
}
#endif

class NanoGICPWrapper {
public:
    NanoGICPWrapper();
    ~NanoGICPWrapper();

    void setSourceCloud(const std::vector<Eigen::Vector3f>& cloud);
    void setTargetCloud(const std::vector<Eigen::Vector3f>& cloud);
    
    Eigen::Matrix4f align(const Eigen::Matrix4f& initial_guess);
    
    void setMaxCorrespondenceDistance(float distance);
    void setRANSACIterations(int n);
    
private:
    struct Impl;
    Impl* pImpl;
};

#endif /* NANO_GICP_WRAPPER_H */
