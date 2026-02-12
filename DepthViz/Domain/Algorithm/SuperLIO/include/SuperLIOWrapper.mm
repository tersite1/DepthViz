//
//  SuperLIOWrapper.mm
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

#import "SuperLIOWrapper.h"

SuperLIOWrapper::SuperLIOWrapper() : _isInitialized(false) {
}

SuperLIOWrapper::~SuperLIOWrapper() {
}

void SuperLIOWrapper::init() {
    _isInitialized = true;
}

void SuperLIOWrapper::start() {
}

void SuperLIOWrapper::stop() {
}

void SuperLIOWrapper::pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
}

void SuperLIOWrapper::pushPointCloud(double timestamp, const std::vector<Point3D>& points) {
}

Eigen::Matrix4d SuperLIOWrapper::getCurrentPose() {
    return Eigen::Matrix4d::Identity();
}

std::vector<Point3D> SuperLIOWrapper::getDisplayCloud() {
    return std::vector<Point3D>();
}

std::vector<Point3D> SuperLIOWrapper::getFullMap() const {
    return std::vector<Point3D>();
}
