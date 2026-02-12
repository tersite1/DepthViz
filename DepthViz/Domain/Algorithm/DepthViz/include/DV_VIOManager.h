#ifndef DEPTHVIZ_VIO_MANAGER_H
#define DEPTHVIZ_VIO_MANAGER_H

// DV-SLAM VIO Manager
// Thin wrapper around ARKit pose — no custom feature tracking.
// ARKit provides the VIO prior; LIO refines it.
// Self-contained: only requires DV_Types.h (no OpenCV/FastLIVO2).

#include <mutex>
#include <atomic>

#include "DV_Types.h"

class DV_VIOManager {
public:
    DV_VIOManager();
    ~DV_VIOManager();

    void init();

    // Store latest ARKit VIO pose
    void pushARKitPose(double timestamp, const Eigen::Matrix4d& pose);

    // Get current pose (ARKit or LIO-refined)
    Eigen::Matrix4d getPose() const;

    // Get rotation/position separately
    Eigen::Matrix3d getRotation() const;
    Eigen::Vector3d getPosition() const;

    // Feedback from LIO backend
    void updatePoseFromLIO(const Eigen::Matrix4d& lio_pose);

    // Whether we have received at least one ARKit pose
    bool hasValidPose() const { return has_pose_.load(); }

private:
    mutable std::mutex mtx_;
    Eigen::Matrix4d current_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d lio_correction_ = Eigen::Matrix4d::Identity();
    double last_timestamp_ = 0.0;
    std::atomic<bool> has_pose_{false};
    std::atomic<bool> has_lio_correction_{false};
};

#endif // DEPTHVIZ_VIO_MANAGER_H
