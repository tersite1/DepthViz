//
//  FastLIVOWrapper.mm
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

#import "FastLIVOWrapper.h"

FastLIVOWrapper::FastLIVOWrapper() : _isInitialized(false) {
}

FastLIVOWrapper::~FastLIVOWrapper() {
}

void FastLIVOWrapper::init() {
    _isInitialized = true;
}

void FastLIVOWrapper::start() {
}

void FastLIVOWrapper::stop() {
}

void FastLIVOWrapper::pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
}

void FastLIVOWrapper::pushPointCloud(double timestamp, const std::vector<Point3D>& points) {
}

void FastLIVOWrapper::pushImage(double timestamp, const void* imageData, int width, int height) {
}

Eigen::Matrix4d FastLIVOWrapper::getCurrentPose() {
    return Eigen::Matrix4d::Identity();
}

std::vector<Point3D> FastLIVOWrapper::getDisplayCloud() {
    return std::vector<Point3D>();
}

std::vector<Point3D> FastLIVOWrapper::getFullMap() const {
    return std::vector<Point3D>();
}
