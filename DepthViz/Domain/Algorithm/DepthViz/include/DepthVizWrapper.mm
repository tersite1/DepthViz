#include "DepthVizWrapper.h"
#include "DepthVizEngine.hpp"

#include <cstdio>

DepthVizWrapper::DepthVizWrapper() {
    _engine = std::make_shared<DepthVizEngine>();
}

DepthVizWrapper::~DepthVizWrapper() {
    stop();
}

void DepthVizWrapper::init() {
    if (_engine) {
        _engine->init();
        printf("[DepthVizWrapper] Initialized\n");
    }
}

void DepthVizWrapper::start() {
    if (_engine) {
        _engine->start();
        printf("[DepthVizWrapper] Started\n");
    }
}

void DepthVizWrapper::stop() {
    if (_engine) {
        _engine->stop();
        printf("[DepthVizWrapper] Stopped\n");
    }
}

void DepthVizWrapper::pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
    if (_engine) {
        _engine->pushIMU(timestamp, acc, gyr);
    }
}

void DepthVizWrapper::pushPointCloud(double timestamp, const std::vector<Point3D>& points) {
    if (!_engine || points.empty()) return;

    // Convert Point3D vector to raw float*/uint8_t* buffers
    int count = static_cast<int>(points.size());
    std::vector<float> xyz(count * 3);
    std::vector<uint8_t> conf(count);
    std::vector<uint8_t> rgb(count * 3);

    for (int i = 0; i < count; i++) {
        xyz[i * 3] = points[i].x;
        xyz[i * 3 + 1] = points[i].y;
        xyz[i * 3 + 2] = points[i].z;
        conf[i] = static_cast<uint8_t>(points[i].intensity * 2.0f); // Map 0-1 intensity to 0-2 confidence
        rgb[i * 3] = points[i].r;
        rgb[i * 3 + 1] = points[i].g;
        rgb[i * 3 + 2] = points[i].b;
    }

    _engine->pushPointCloud(timestamp, xyz.data(), conf.data(), rgb.data(), count);
}

void DepthVizWrapper::pushImage(double timestamp, const void* imageData, int width, int height) {
    if (_engine) {
        _engine->pushImage(timestamp, imageData, width, height);
    }
}

Eigen::Matrix4d DepthVizWrapper::getCurrentPose() {
    if (_engine) {
        return _engine->getPose();
    }
    return Eigen::Matrix4d::Identity();
}

std::vector<Point3D> DepthVizWrapper::getDisplayCloud() {
    if (!_engine) return {};

    auto dvPoints = _engine->getDisplayCloud();
    std::vector<Point3D> result;
    result.reserve(dvPoints.size());

    for (const auto& dp : dvPoints) {
        Point3D p;
        p.x = dp.x;
        p.y = dp.y;
        p.z = dp.z;
        p.intensity = dp.intensity;
        p.r = dp.r;
        p.g = dp.g;
        p.b = dp.b;
        p.timestamp = dp.timestamp;
        result.push_back(p);
    }

    return result;
}

std::vector<Point3D> DepthVizWrapper::getFullMap() {
    if (!_engine) return {};

    auto dvPoints = _engine->getFullMap();
    std::vector<Point3D> result;
    result.reserve(dvPoints.size());

    for (const auto& dp : dvPoints) {
        Point3D p;
        p.x = dp.x;
        p.y = dp.y;
        p.z = dp.z;
        p.intensity = dp.intensity;
        p.r = dp.r;
        p.g = dp.g;
        p.b = dp.b;
        p.timestamp = dp.timestamp;
        result.push_back(p);
    }

    return result;
}

void DepthVizWrapper::pushARKitPose(double timestamp, const Eigen::Matrix4d& pose) {
    if (_engine) {
        _engine->pushARKitPose(timestamp, pose);
    }
}

void DepthVizWrapper::pushPointCloudRaw(double timestamp, const float* xyz, const uint8_t* conf, const uint8_t* rgb, int count) {
    if (_engine) {
        _engine->pushPointCloud(timestamp, xyz, conf, rgb, count);
    }
}
