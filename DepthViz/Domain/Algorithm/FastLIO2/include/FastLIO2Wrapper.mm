//
//  FastLIO2Wrapper.mm
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

#import "FastLIO2Wrapper.h"
#include <iostream>

FastLIO2Wrapper::FastLIO2Wrapper() {
}

FastLIO2Wrapper::~FastLIO2Wrapper() {
}

void FastLIO2Wrapper::init() {
}

void FastLIO2Wrapper::start() {
}

void FastLIO2Wrapper::stop() {
}

void FastLIO2Wrapper::pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
}

void FastLIO2Wrapper::pushPointCloud(double timestamp, const std::vector<Point3D>& points) {
}

Eigen::Matrix4d FastLIO2Wrapper::getCurrentPose() {
    return Eigen::Matrix4d::Identity();
}

std::vector<Point3D> FastLIO2Wrapper::getDisplayCloud() {
    return std::vector<Point3D>();
}

std::vector<Point3D> FastLIO2Wrapper::getFullMap() const {
    return std::vector<Point3D>();
}
