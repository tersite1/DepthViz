#ifndef DV_OFFLINE_EVALUATOR_H
#define DV_OFFLINE_EVALUATOR_H

// DV-SLAM Offline Evaluator
// Loads preprocessed ARKitScenes sequences and evaluates DV-SLAM.
// Supports ablation studies, PLY export, and profiling output for RA-L submission.

#include "DV_Types.h"
#include "DepthVizEngine.hpp"
#include <vector>
#include <string>
#include <memory>

class DepthVizEngine;

namespace DV {

struct EvalFrame {
    double timestamp;
    std::vector<float> xyz;      // Nx3 interleaved
    std::vector<uint8_t> conf;   // N confidence values
    int point_count;
    M4d gt_pose;                 // Ground-truth pose (if available)
    M4d arkit_pose;              // ARKit pose
};

struct TrajectoryError {
    double ate_rmse = 0;     // Absolute Trajectory Error (RMSE, meters)
    double ate_mean = 0;     // ATE mean
    double ate_max = 0;      // ATE max
    double rpe_rmse = 0;     // Relative Pose Error (RMSE, meters)
    double rpe_rot_rmse = 0; // RPE rotation (degrees)
    int num_frames = 0;
};

class DV_OfflineEvaluator {
public:
    DV_OfflineEvaluator();
    ~DV_OfflineEvaluator();

    // Load a preprocessed ARKitScenes sequence directory
    // Expected layout (output of preprocess_arkitscenes.py):
    //   <path>/depth/<timestamp>.bin       — float32 binary (depth in meters)
    //   <path>/confidence/<timestamp>.bin  — uint8 binary
    //   <path>/poses.txt                   — timestamp tx ty tz qx qy qz qw (TUM)
    //   <path>/intrinsics.txt              — fx fy cx cy width height
    bool loadSequence(const std::string& sequence_path);

    // Feed a manually constructed sequence
    void setFrames(const std::vector<EvalFrame>& frames);

    // Set ablation config before evaluation
    void setAblation(const DepthVizEngine::AblationConfig& config) { ablation_ = config; }

    // Run evaluation
    TrajectoryError evaluate();

    // Get estimated trajectory
    std::vector<std::pair<double, M4d>> getEstimatedTrajectory() const;

    // Get ground-truth trajectory
    std::vector<std::pair<double, M4d>> getGroundTruthTrajectory() const;

    // Get profiling stats from last evaluation
    DepthVizEngine::ProfilingStats getProfilingStats() const;

    // ====================================================================
    // Output writers
    // ====================================================================

    bool writeTUMTrajectory(const std::string& filepath,
                            const std::vector<std::pair<double, M4d>>& trajectory) const;

    bool writeErrorReport(const std::string& filepath,
                          const TrajectoryError& error,
                          const DepthVizEngine::ProfilingStats* profiling = nullptr) const;

    // Write reconstructed point cloud as PLY (from getFullMap)
    bool writePLY(const std::string& filepath) const;

    // Write reconstructed point cloud with trajectory as colored PLY
    bool writePLYWithTrajectory(const std::string& filepath) const;

    // ====================================================================
    // Static metric computation
    // ====================================================================

    static TrajectoryError computeATE(
        const std::vector<std::pair<double, M4d>>& estimated,
        const std::vector<std::pair<double, M4d>>& ground_truth);

    static double computeRPE(
        const std::vector<std::pair<double, M4d>>& estimated,
        const std::vector<std::pair<double, M4d>>& ground_truth);

private:
    std::shared_ptr<DepthVizEngine> engine_;
    std::vector<EvalFrame> frames_;
    std::vector<std::pair<double, M4d>> estimated_trajectory_;
    std::vector<std::pair<double, M4d>> gt_trajectory_;
    DepthVizEngine::AblationConfig ablation_;
    DepthVizEngine::ProfilingStats last_profiling_;
};

} // namespace DV

#endif // DV_OFFLINE_EVALUATOR_H
