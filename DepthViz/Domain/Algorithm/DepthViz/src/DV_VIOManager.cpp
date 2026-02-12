#include "../include/DV_VIOManager.h"

DV_VIOManager::DV_VIOManager() = default;
DV_VIOManager::~DV_VIOManager() = default;

void DV_VIOManager::init() {
    std::lock_guard<std::mutex> lock(mtx_);
    current_pose_ = Eigen::Matrix4d::Identity();
    lio_correction_ = Eigen::Matrix4d::Identity();
    last_timestamp_ = 0.0;
    has_pose_.store(false);
    has_lio_correction_.store(false);
}

void DV_VIOManager::pushARKitPose(double timestamp, const Eigen::Matrix4d& pose) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_pose_ = pose;
    last_timestamp_ = timestamp;
    has_pose_.store(true);
}

Eigen::Matrix4d DV_VIOManager::getPose() const {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!has_pose_.load()) {
        return Eigen::Matrix4d::Identity();
    }

    // If LIO has provided a correction, apply it to the ARKit pose
    if (has_lio_correction_.load()) {
        return lio_correction_ * current_pose_;
    }

    return current_pose_;
}

Eigen::Matrix3d DV_VIOManager::getRotation() const {
    return getPose().block<3, 3>(0, 0);
}

Eigen::Vector3d DV_VIOManager::getPosition() const {
    return getPose().block<3, 1>(0, 3);
}

void DV_VIOManager::updatePoseFromLIO(const Eigen::Matrix4d& lio_pose) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!has_pose_.load()) return;

    // Compute correction: T_lio = T_correction * T_arkit
    // => T_correction = T_lio * T_arkit^{-1}
    Eigen::Matrix4d arkit_inv = Eigen::Matrix4d::Identity();
    arkit_inv.block<3, 3>(0, 0) = current_pose_.block<3, 3>(0, 0).transpose();
    arkit_inv.block<3, 1>(0, 3) = -current_pose_.block<3, 3>(0, 0).transpose() * current_pose_.block<3, 1>(0, 3);

    lio_correction_ = lio_pose * arkit_inv;
    has_lio_correction_.store(true);
}
