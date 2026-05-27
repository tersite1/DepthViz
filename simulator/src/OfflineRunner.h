#pragma once
// Offline DV-SLAM Runner
// Feeds ScanNet++ data through DV-SLAM pipeline and collects results.

#include "ScanNetLoader.h"
#include "DepthVizEngine.hpp"
#include <memory>

struct RunConfig {
    // Experiment preset
    std::string preset_name = "full";

    // Ablation flags
    bool enable_imu = true;
    bool enable_lio = true;
    bool enable_confidence_weight = true;
    bool enable_tls = true;

    // Point subsampling
    int depth_stride = 4;  // 1=all, 4=default, 12=~100pts

    // Output paths
    std::string output_dir = "./output";
};

struct RunResult {
    // Trajectory
    std::vector<std::pair<double, Eigen::Matrix4d>> estimated_trajectory;
    std::vector<std::pair<double, Eigen::Matrix4d>> gt_trajectory;

    // Metrics
    double ate_rmse = 0;
    double ate_mean = 0;
    double ate_max = 0;
    double rpe_trans = 0;
    double rpe_rot_deg = 0;
    int num_frames = 0;

    // Timing
    double total_time_sec = 0;
    double avg_frame_ms = 0;
};

class OfflineRunner {
public:
    OfflineRunner();

    // Run DV-SLAM on a loaded ScanNet++ scene
    RunResult run(const ScanNetScene& scene, const RunConfig& config);

    // Write outputs
    bool writeTUMTrajectory(const std::string& path,
                            const std::vector<std::pair<double, Eigen::Matrix4d>>& traj);
    bool writePLY(const std::string& path);
    bool writeReport(const std::string& path, const RunResult& result);

private:
    std::shared_ptr<DepthVizEngine> engine_;

    // Unproject depth to 3D point cloud
    void unprojectDepth(const DepthFrame& depth,
                        float fx, float fy, float cx, float cy,
                        int stride,
                        std::vector<float>& xyz_out,
                        std::vector<uint8_t>& conf_out,
                        int& count_out);

    // Compute ATE
    RunResult computeMetrics(
        const std::vector<std::pair<double, Eigen::Matrix4d>>& est,
        const std::vector<std::pair<double, Eigen::Matrix4d>>& gt);
};
