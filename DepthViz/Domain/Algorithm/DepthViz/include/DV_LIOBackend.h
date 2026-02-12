#ifndef DV_LIO_BACKEND_H
#define DV_LIO_BACKEND_H

// DV-SLAM LIO Backend
// Point-to-plane ICP with confidence weighting and Truncated Least Squares.
// Self-contained: uses DV_VoxelHashMap + DV_ESKF instead of PCL/SuperLIO.

#include <vector>
#include <memory>
#include <mutex>

#include "DV_Types.h"
#include "DV_VoxelHashMap.h"
#include "DV_ESKF.h"

class DV_LIOBackend {
public:
    DV_LIOBackend();
    ~DV_LIOBackend();

    void init();

    // Process IMU measurements for ESKF prediction step.
    // Call this with all IMU data between the previous and current LiDAR frame.
    void processIMU(const std::vector<DV::IMUData>& imu_data);

    // Process a frame: takes prior pose + bundled points, returns refined pose.
    // First frame: inserts points into map, returns prior.
    // Subsequent frames: point-to-plane ICP with robust weighting.
    Eigen::Matrix4d process(const Eigen::Matrix4d& prior_pose,
                            const std::vector<DV::DVPoint3D>& points);

    // Get map points (voxel centroids) for export/visualization
    std::vector<DV::DVPoint3D> getMapPoints();

    Eigen::Matrix4d getCurrentPose() const;

private:
    // Point-to-plane residual computation
    struct PlaneResidual {
        Eigen::Vector3f normal;
        Eigen::Vector3f centroid;
        float residual;
        bool valid;
    };

    PlaneResidual computePointToPlaneResidual(
        const Eigen::Vector3f& query_world, float confidence);

    // Insert points into map (world frame)
    void insertPoints(const Eigen::Matrix4d& pose,
                      const std::vector<DV::DVPoint3D>& points);

    // Core components
    std::unique_ptr<DV::DV_VoxelHashMap> map_;
    std::unique_ptr<DV::DV_ESKF> eskf_;

    // State
    Eigen::Matrix4d current_pose_ = Eigen::Matrix4d::Identity();
    bool first_frame_ = true;
    double last_imu_timestamp_ = -1.0;
    mutable std::mutex mtx_;

    // Config
    DV::ESKFOptions eskf_opts_;
    int min_observations_ = 10; // Min valid residuals to run update

public:
    // Ablation flags (forwarded from engine)
    bool use_confidence_weight_ = true;
    bool use_tls_ = true;
};

#endif // DV_LIO_BACKEND_H
