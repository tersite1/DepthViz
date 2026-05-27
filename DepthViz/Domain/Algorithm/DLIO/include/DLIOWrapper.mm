//
//  DLIOWrapper.mm
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

#import "DLIOWrapper.h"

DLIOWrapper::DLIOWrapper() : _isInitialized(false) {
}

DLIOWrapper::~DLIOWrapper() {
}

void DLIOWrapper::init() {
    _isInitialized = true;
}

void DLIOWrapper::start() {
}

void DLIOWrapper::stop() {
}

void DLIOWrapper::pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
}

void DLIOWrapper::pushPointCloud(double timestamp, const std::vector<Point3D>& points) {
}

Eigen::Matrix4d DLIOWrapper::getCurrentPose() {
    return Eigen::Matrix4d::Identity();
}

std::vector<Point3D> DLIOWrapper::getDisplayCloud() {
    return std::vector<Point3D>();
}

std::vector<Point3D> DLIOWrapper::getFullMap() const {
    return std::vector<Point3D>();
}
