#include "../include/DV_OfflineEvaluator.h"
#include "../include/DepthVizEngine.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <dirent.h>

namespace DV {

DV_OfflineEvaluator::DV_OfflineEvaluator() {
    engine_ = std::make_shared<DepthVizEngine>();
}

DV_OfflineEvaluator::~DV_OfflineEvaluator() = default;

bool DV_OfflineEvaluator::loadSequence(const std::string& sequence_path) {
    // Load poses (TUM format: timestamp tx ty tz qx qy qz qw)
    std::string poses_path = sequence_path + "/poses.txt";
    std::ifstream poses_file(poses_path);
    if (!poses_file.is_open()) {
        printf("[Evaluator] ERROR: Cannot open %s\n", poses_path.c_str());
        return false;
    }

    std::vector<std::pair<double, M4d>> gt_poses;
    std::string line;
    while (std::getline(poses_file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        double ts, tx, ty, tz, qx, qy, qz, qw;
        if (!(iss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) continue;

        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();
        M4d pose = M4d::Identity();
        pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        pose(0, 3) = tx;
        pose(1, 3) = ty;
        pose(2, 3) = tz;

        gt_poses.emplace_back(ts, pose);
    }

    if (gt_poses.empty()) {
        printf("[Evaluator] ERROR: No poses found\n");
        return false;
    }
    printf("[Evaluator] Loaded %zu poses\n", gt_poses.size());

    // Load intrinsics
    std::string intrinsics_path = sequence_path + "/intrinsics.txt";
    std::ifstream intrinsics_file(intrinsics_path);
    float fx = 500, fy = 500, cx = 128, cy = 96;
    int img_width = 256, img_height = 192;
    if (intrinsics_file.is_open()) {
        intrinsics_file >> fx >> fy >> cx >> cy >> img_width >> img_height;
        printf("[Evaluator] Intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f %dx%d\n",
               fx, fy, cx, cy, img_width, img_height);
    }

    // Enumerate available depth files to find matching timestamps
    std::string depth_dir = sequence_path + "/depth";
    std::string conf_dir = sequence_path + "/confidence";

    // Build timestamp → file mapping
    std::vector<std::string> depth_files;
    DIR* dir = opendir(depth_dir.c_str());
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;
            if (name.size() > 4 && name.substr(name.size() - 4) == ".bin") {
                depth_files.push_back(name);
            }
        }
        closedir(dir);
    }
    std::sort(depth_files.begin(), depth_files.end());
    printf("[Evaluator] Found %zu depth files\n", depth_files.size());

    // Build frame set: match each pose with nearest depth file
    frames_.clear();
    frames_.reserve(gt_poses.size());

    for (const auto& [ts, pose] : gt_poses) {
        EvalFrame frame;
        frame.timestamp = ts;
        frame.gt_pose = pose;
        frame.arkit_pose = pose; // Use GT as ARKit proxy for evaluation
        frame.point_count = 0;

        // Try to load depth + confidence for this timestamp
        std::ostringstream depth_path_ss, conf_path_ss;
        depth_path_ss << depth_dir << "/" << std::fixed << std::setprecision(6) << ts << ".bin";
        conf_path_ss << conf_dir << "/" << std::fixed << std::setprecision(6) << ts << ".bin";

        std::ifstream depth_file(depth_path_ss.str(), std::ios::binary);
        std::ifstream conf_file(conf_path_ss.str(), std::ios::binary);

        if (depth_file.is_open()) {
            // Read binary depth (float32 array, row-major)
            depth_file.seekg(0, std::ios::end);
            size_t depth_size = depth_file.tellg();
            depth_file.seekg(0);

            int num_pixels = static_cast<int>(depth_size / sizeof(float));
            std::vector<float> depth_data(num_pixels);
            depth_file.read(reinterpret_cast<char*>(depth_data.data()), depth_size);

            // Read confidence
            std::vector<uint8_t> conf_data(num_pixels, 2); // Default high confidence
            if (conf_file.is_open()) {
                conf_file.read(reinterpret_cast<char*>(conf_data.data()), num_pixels);
            }

            // Subsample and unproject to 3D
            int step = 4;
            frame.xyz.reserve((img_width / step) * (img_height / step) * 3);
            frame.conf.reserve((img_width / step) * (img_height / step));

            int pt_count = 0;
            for (int row = 0; row < img_height; row += step) {
                for (int col = 0; col < img_width; col += step) {
                    int idx = row * img_width + col;
                    if (idx >= num_pixels) continue;

                    float depth = depth_data[idx];
                    if (depth <= 0.0f || depth > 10.0f || std::isnan(depth)) continue;

                    float x = (col - cx) * depth / fx;
                    float y = (row - cy) * depth / fy;
                    float z = depth;

                    frame.xyz.push_back(x);
                    frame.xyz.push_back(y);
                    frame.xyz.push_back(z);
                    frame.conf.push_back(conf_data[idx]);
                    pt_count++;
                }
            }
            frame.point_count = pt_count;
        }

        frames_.push_back(std::move(frame));
    }

    int with_depth = 0;
    for (const auto& f : frames_) {
        if (f.point_count > 0) with_depth++;
    }
    printf("[Evaluator] Built %zu frames (%d with depth data)\n", frames_.size(), with_depth);

    return !frames_.empty();
}

void DV_OfflineEvaluator::setFrames(const std::vector<EvalFrame>& frames) {
    frames_ = frames;
}

TrajectoryError DV_OfflineEvaluator::evaluate() {
    if (frames_.empty()) {
        return {};
    }

    // Reset engine with ablation config
    engine_ = std::make_shared<DepthVizEngine>();
    engine_->setAblationConfig(ablation_);
    engine_->init();
    engine_->start();

    estimated_trajectory_.clear();
    gt_trajectory_.clear();

    printf("[Evaluator] Running DV-SLAM on %zu frames...\n", frames_.size());

    for (size_t i = 0; i < frames_.size(); i++) {
        const auto& frame = frames_[i];

        // Push ARKit pose
        engine_->pushARKitPose(frame.timestamp, frame.arkit_pose);

        // Push point cloud if available
        if (frame.point_count > 0) {
            engine_->pushPointCloud(
                frame.timestamp,
                frame.xyz.data(),
                frame.conf.data(),
                frame.point_count);
        }

        // Wait for processing (offline: allow engine thread to consume)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Record estimated pose
        M4d estimated = engine_->getPose();
        estimated_trajectory_.emplace_back(frame.timestamp, estimated);
        gt_trajectory_.emplace_back(frame.timestamp, frame.gt_pose);

        // Progress
        if ((i + 1) % 100 == 0 || i + 1 == frames_.size()) {
            printf("[Evaluator] Processed %zu / %zu frames\r", i + 1, frames_.size());
            fflush(stdout);
        }
    }
    printf("\n");

    // Capture profiling stats before stopping
    last_profiling_ = engine_->getProfilingStats();

    engine_->stop();

    // Compute error metrics
    return computeATE(estimated_trajectory_, gt_trajectory_);
}

std::vector<std::pair<double, M4d>> DV_OfflineEvaluator::getEstimatedTrajectory() const {
    return estimated_trajectory_;
}

std::vector<std::pair<double, M4d>> DV_OfflineEvaluator::getGroundTruthTrajectory() const {
    return gt_trajectory_;
}

DepthVizEngine::ProfilingStats DV_OfflineEvaluator::getProfilingStats() const {
    return last_profiling_;
}

TrajectoryError DV_OfflineEvaluator::computeATE(
    const std::vector<std::pair<double, M4d>>& estimated,
    const std::vector<std::pair<double, M4d>>& ground_truth)
{
    TrajectoryError result = {};
    int n = std::min(static_cast<int>(estimated.size()), static_cast<int>(ground_truth.size()));
    if (n == 0) return result;

    result.num_frames = n;

    // Compute ATE (translation error)
    std::vector<double> errors;
    errors.reserve(n);

    for (int i = 0; i < n; i++) {
        V3d est_t = estimated[i].second.block<3, 1>(0, 3);
        V3d gt_t = ground_truth[i].second.block<3, 1>(0, 3);
        double err = (est_t - gt_t).norm();
        errors.push_back(err);
    }

    // RMSE
    double sum_sq = 0;
    for (double e : errors) sum_sq += e * e;
    result.ate_rmse = std::sqrt(sum_sq / n);

    // Mean
    result.ate_mean = std::accumulate(errors.begin(), errors.end(), 0.0) / n;

    // Max
    result.ate_max = *std::max_element(errors.begin(), errors.end());

    // RPE (relative pose error between consecutive frames)
    if (n >= 2) {
        std::vector<double> rpe_trans, rpe_rot;
        for (int i = 1; i < n; i++) {
            M4d est_rel = estimated[i - 1].second.inverse() * estimated[i].second;
            M4d gt_rel = ground_truth[i - 1].second.inverse() * ground_truth[i].second;
            M4d delta = gt_rel.inverse() * est_rel;

            double t_err = delta.block<3, 1>(0, 3).norm();
            rpe_trans.push_back(t_err);

            M3d dR = delta.block<3, 3>(0, 0);
            double cos_angle = (dR.trace() - 1.0) * 0.5;
            cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
            double angle_rad = std::acos(cos_angle);
            rpe_rot.push_back(angle_rad * 180.0 / M_PI);
        }

        double rpe_sum_sq = 0;
        for (double e : rpe_trans) rpe_sum_sq += e * e;
        result.rpe_rmse = std::sqrt(rpe_sum_sq / rpe_trans.size());

        double rpe_rot_sum_sq = 0;
        for (double e : rpe_rot) rpe_rot_sum_sq += e * e;
        result.rpe_rot_rmse = std::sqrt(rpe_rot_sum_sq / rpe_rot.size());
    }

    return result;
}

double DV_OfflineEvaluator::computeRPE(
    const std::vector<std::pair<double, M4d>>& estimated,
    const std::vector<std::pair<double, M4d>>& ground_truth)
{
    auto error = computeATE(estimated, ground_truth);
    return error.rpe_rmse;
}

bool DV_OfflineEvaluator::writeTUMTrajectory(
    const std::string& filepath,
    const std::vector<std::pair<double, M4d>>& trajectory) const
{
    std::ofstream file(filepath);
    if (!file.is_open()) return false;

    file << "# timestamp tx ty tz qx qy qz qw\n";

    for (const auto& [ts, pose] : trajectory) {
        V3d t = pose.block<3, 1>(0, 3);
        Eigen::Quaterniond q(pose.block<3, 3>(0, 0));

        file << std::fixed << std::setprecision(6) << ts << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }

    return true;
}

bool DV_OfflineEvaluator::writeErrorReport(
    const std::string& filepath,
    const TrajectoryError& error,
    const DepthVizEngine::ProfilingStats* profiling) const
{
    std::ofstream file(filepath);
    if (!file.is_open()) return false;

    file << "DV-SLAM Evaluation Report\n";
    file << "========================\n\n";
    file << "Frames: " << error.num_frames << "\n\n";

    file << "Absolute Trajectory Error (ATE):\n";
    file << std::fixed << std::setprecision(4);
    file << "  RMSE:  " << error.ate_rmse << " m\n";
    file << "  Mean:  " << error.ate_mean << " m\n";
    file << "  Max:   " << error.ate_max << " m\n\n";

    file << "Relative Pose Error (RPE):\n";
    file << "  Trans RMSE: " << error.rpe_rmse << " m\n";
    file << "  Rot RMSE:   " << error.rpe_rot_rmse << " deg\n\n";

    if (profiling) {
        file << "Runtime Performance:\n";
        file << std::setprecision(2);
        file << "  Total frames:       " << profiling->total_frames << "\n";
        file << "  Keyframes:          " << profiling->keyframes << "\n";
        file << "  Avg B&D latency:    " << profiling->avgBundleDiscardMs() << " ms\n";
        file << "  Avg LIO latency:    " << profiling->avgLioMs() << " ms\n";
        file << "  Avg pipeline:       " << profiling->avgPipelineMs() << " ms\n";
        file << "  Throughput:         " << profiling->fps() << " FPS\n";
        file << "  Point reduction:    " << std::setprecision(1)
             << profiling->avgReductionRatio() * 100.0 << "%\n";
        file << "  Input points:       " << profiling->total_input_points << "\n";
        file << "  Output points:      " << profiling->total_output_points << "\n";
    }

    return true;
}

bool DV_OfflineEvaluator::writePLY(const std::string& filepath) const {
    if (!engine_) return false;

    // Get map points from the engine (this works even after stop() since the map persists)
    auto map_points = engine_->getFullMap();
    if (map_points.empty()) {
        printf("[Evaluator] WARNING: No map points to export\n");
        return false;
    }

    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) return false;

    // PLY header (binary little-endian)
    file << "ply\n";
    file << "format binary_little_endian 1.0\n";
    file << "element vertex " << map_points.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "property float confidence\n";
    file << "end_header\n";

    // Write binary vertex data
    for (const auto& pt : map_points) {
        file.write(reinterpret_cast<const char*>(&pt.x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&pt.y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&pt.z), sizeof(float));

        // Color by confidence (blue=low, green=medium, red=high)
        uint8_t r = 0, g = 0, b = 0;
        if (pt.confidence >= 1.5f) { r = 0; g = 200; b = 0; }       // High: green
        else if (pt.confidence >= 0.5f) { r = 200; g = 200; b = 0; } // Med: yellow
        else { r = 200; g = 0; b = 0; }                              // Low: red
        file.write(reinterpret_cast<const char*>(&r), 1);
        file.write(reinterpret_cast<const char*>(&g), 1);
        file.write(reinterpret_cast<const char*>(&b), 1);

        file.write(reinterpret_cast<const char*>(&pt.confidence), sizeof(float));
    }

    printf("[Evaluator] Exported %zu points to %s\n", map_points.size(), filepath.c_str());
    return true;
}

bool DV_OfflineEvaluator::writePLYWithTrajectory(const std::string& filepath) const {
    if (!engine_) return false;

    auto map_points = engine_->getFullMap();
    size_t n_traj = estimated_trajectory_.size();
    size_t total_vertices = map_points.size() + n_traj;

    if (total_vertices == 0) return false;

    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) return false;

    file << "ply\n";
    file << "format binary_little_endian 1.0\n";
    file << "element vertex " << total_vertices << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "end_header\n";

    // Write map points (grey)
    for (const auto& pt : map_points) {
        file.write(reinterpret_cast<const char*>(&pt.x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&pt.y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&pt.z), sizeof(float));
        uint8_t r = 180, g = 180, b = 180;
        file.write(reinterpret_cast<const char*>(&r), 1);
        file.write(reinterpret_cast<const char*>(&g), 1);
        file.write(reinterpret_cast<const char*>(&b), 1);
    }

    // Write trajectory points (bright red, larger visual weight)
    for (const auto& [ts, pose] : estimated_trajectory_) {
        float x = static_cast<float>(pose(0, 3));
        float y = static_cast<float>(pose(1, 3));
        float z = static_cast<float>(pose(2, 3));
        file.write(reinterpret_cast<const char*>(&x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&z), sizeof(float));
        uint8_t r = 255, g = 0, b = 0;
        file.write(reinterpret_cast<const char*>(&r), 1);
        file.write(reinterpret_cast<const char*>(&g), 1);
        file.write(reinterpret_cast<const char*>(&b), 1);
    }

    printf("[Evaluator] Exported %zu vertices (%zu map + %zu trajectory) to %s\n",
           total_vertices, map_points.size(), n_traj, filepath.c_str());
    return true;
}

} // namespace DV
