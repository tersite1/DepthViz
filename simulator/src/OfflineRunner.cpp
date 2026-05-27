#include "OfflineRunner.h"
#include <fstream>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <filesystem>
#include <map>
#include <iomanip>

namespace fs = std::filesystem;

OfflineRunner::OfflineRunner() {}

RunResult OfflineRunner::run(const ScanNetScene& scene, const RunConfig& config) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "Running DV-SLAM: " << config.preset_name << std::endl;
    std::cout << "  IMU: " << (config.enable_imu ? "ON" : "OFF") << std::endl;
    std::cout << "  LIO: " << (config.enable_lio ? "ON" : "OFF") << std::endl;
    std::cout << "  Confidence: " << (config.enable_confidence_weight ? "ON" : "OFF") << std::endl;
    std::cout << "  TLS: " << (config.enable_tls ? "ON" : "OFF") << std::endl;
    std::cout << "  Stride: " << config.depth_stride << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Create output directory
    fs::create_directories(config.output_dir);

    // Init engine
    engine_ = std::make_shared<DepthVizEngine>();
    DepthVizEngine::AblationConfig ablation;
    ablation.enable_imu = config.enable_imu;
    ablation.enable_lio = config.enable_lio;
    ablation.enable_confidence_weight = config.enable_confidence_weight;
    ablation.enable_tls = config.enable_tls;
    engine_->setAblationConfig(ablation);
    engine_->init();
    engine_->start();

    RunResult result;
    auto global_start = std::chrono::high_resolution_clock::now();

    // Build time-sorted event queue: IMU samples + depth frames
    // We process IMU between depth frames for correct timing
    size_t imu_idx = 0;
    size_t frame_idx = 0;

    // Match depth frames with poses
    std::map<std::string, size_t> pose_map;
    for (size_t i = 0; i < scene.poses.size(); i++) {
        pose_map[scene.poses[i].frame_name] = i;
    }

    int processed = 0;
    int total_frames = static_cast<int>(scene.depth_frames.size());

    for (const auto& depth : scene.depth_frames) {
        // 1. Feed all IMU samples up to this depth frame's timestamp
        while (imu_idx < scene.imu_samples.size() &&
               scene.imu_samples[imu_idx].timestamp <= depth.timestamp) {
            const auto& imu = scene.imu_samples[imu_idx];
            engine_->pushIMU(imu.timestamp, imu.acc, imu.gyr);
            imu_idx++;
        }

        // 2. Find matching pose for this frame
        auto pit = pose_map.find(depth.frame_name);
        Eigen::Matrix4d gt_pose = Eigen::Matrix4d::Identity();
        if (pit != pose_map.end()) {
            gt_pose = scene.poses[pit->second].aligned_pose;
        }

        // 3. Unproject depth to 3D point cloud
        std::vector<float> xyz;
        std::vector<uint8_t> conf;
        int pt_count = 0;
        unprojectDepth(depth, scene.fx_depth, scene.fy_depth,
                       scene.cx_depth, scene.cy_depth,
                       config.depth_stride, xyz, conf, pt_count);

        // 4. Push to engine
        if (pt_count > 0) {
            engine_->pushPointCloud(depth.timestamp,
                                    xyz.data(), conf.data(), nullptr, pt_count);
        }

        // 5. Wait for engine to process (offline: give thread time)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // 6. Record trajectory
        Eigen::Matrix4d est_pose = engine_->getPose();
        result.estimated_trajectory.emplace_back(depth.timestamp, est_pose);
        result.gt_trajectory.emplace_back(depth.timestamp, gt_pose);

        processed++;
        if (processed % 50 == 0 || processed == total_frames) {
            printf("[Runner] %d / %d frames (pts=%d)\r", processed, total_frames, pt_count);
            fflush(stdout);
        }
    }
    printf("\n");

    auto global_end = std::chrono::high_resolution_clock::now();
    result.total_time_sec = std::chrono::duration<double>(global_end - global_start).count();
    result.num_frames = processed;
    result.avg_frame_ms = (result.total_time_sec / std::max(1, processed)) * 1000.0;

    // Stop engine
    engine_->stop();

    // Compute metrics
    auto metrics = computeMetrics(result.estimated_trajectory, result.gt_trajectory);
    result.ate_rmse = metrics.ate_rmse;
    result.ate_mean = metrics.ate_mean;
    result.ate_max = metrics.ate_max;
    result.rpe_trans = metrics.rpe_trans;
    result.rpe_rot_deg = metrics.rpe_rot_deg;

    std::cout << "\n=== Results ===" << std::endl;
    std::cout << "  ATE RMSE: " << result.ate_rmse * 100.0 << " cm" << std::endl;
    std::cout << "  ATE Mean: " << result.ate_mean * 100.0 << " cm" << std::endl;
    std::cout << "  ATE Max:  " << result.ate_max * 100.0 << " cm" << std::endl;
    std::cout << "  RPE Trans: " << result.rpe_trans * 100.0 << " cm" << std::endl;
    std::cout << "  RPE Rot:   " << result.rpe_rot_deg << " deg" << std::endl;
    std::cout << "  Frames: " << result.num_frames << std::endl;
    std::cout << "  Total time: " << result.total_time_sec << " s" << std::endl;
    std::cout << "  Avg frame: " << result.avg_frame_ms << " ms" << std::endl;

    return result;
}

void OfflineRunner::unprojectDepth(
    const DepthFrame& depth,
    float fx, float fy, float cx, float cy,
    int stride,
    std::vector<float>& xyz_out,
    std::vector<uint8_t>& conf_out,
    int& count_out)
{
    int w = depth.width;
    int h = depth.height;
    int max_pts = (w / stride) * (h / stride);
    xyz_out.reserve(max_pts * 3);
    conf_out.reserve(max_pts);
    count_out = 0;

    for (int row = 0; row < h; row += stride) {
        for (int col = 0; col < w; col += stride) {
            int idx = row * w + col;
            if (idx >= static_cast<int>(depth.depth_mm.size())) continue;

            uint16_t d_mm = depth.depth_mm[idx];
            if (d_mm == 0 || d_mm > 10000) continue; // 0=invalid, >10m=too far

            float d = static_cast<float>(d_mm) / 1000.0f; // mm → m

            float x = (col - cx) * d / fx;
            float y = -((row - cy) * d / fy);  // flip Y: image Y-down → ARKit Y-up
            float z = -d;                        // flip Z: depth forward → ARKit Z-backward

            xyz_out.push_back(x);
            xyz_out.push_back(y);
            xyz_out.push_back(z);
            conf_out.push_back(2); // No confidence map → default high (2)
            count_out++;
        }
    }
}

RunResult OfflineRunner::computeMetrics(
    const std::vector<std::pair<double, Eigen::Matrix4d>>& est,
    const std::vector<std::pair<double, Eigen::Matrix4d>>& gt)
{
    RunResult result;
    int n = std::min(static_cast<int>(est.size()), static_cast<int>(gt.size()));
    if (n == 0) return result;
    result.num_frames = n;

    // ATE
    std::vector<double> errors;
    for (int i = 0; i < n; i++) {
        Eigen::Vector3d e_t = est[i].second.block<3,1>(0,3);
        Eigen::Vector3d g_t = gt[i].second.block<3,1>(0,3);
        errors.push_back((e_t - g_t).norm());
    }
    double sum_sq = 0;
    for (double e : errors) sum_sq += e * e;
    result.ate_rmse = std::sqrt(sum_sq / n);
    result.ate_mean = std::accumulate(errors.begin(), errors.end(), 0.0) / n;
    result.ate_max = *std::max_element(errors.begin(), errors.end());

    // RPE
    if (n >= 2) {
        std::vector<double> rpe_t, rpe_r;
        for (int i = 1; i < n; i++) {
            Eigen::Matrix4d e_rel = est[i-1].second.inverse() * est[i].second;
            Eigen::Matrix4d g_rel = gt[i-1].second.inverse() * gt[i].second;
            Eigen::Matrix4d delta = g_rel.inverse() * e_rel;
            rpe_t.push_back(delta.block<3,1>(0,3).norm());

            Eigen::Matrix3d dR = delta.block<3,3>(0,0);
            double cos_a = std::max(-1.0, std::min(1.0, (dR.trace()-1.0)*0.5));
            rpe_r.push_back(std::acos(cos_a) * 180.0 / M_PI);
        }
        double rt_sq = 0;
        for (double e : rpe_t) rt_sq += e * e;
        result.rpe_trans = std::sqrt(rt_sq / rpe_t.size());
        double rr_sq = 0;
        for (double e : rpe_r) rr_sq += e * e;
        result.rpe_rot_deg = std::sqrt(rr_sq / rpe_r.size());
    }

    return result;
}

bool OfflineRunner::writeTUMTrajectory(
    const std::string& path,
    const std::vector<std::pair<double, Eigen::Matrix4d>>& traj)
{
    std::ofstream f(path);
    if (!f.is_open()) return false;
    f << "# timestamp tx ty tz qx qy qz qw\n";
    for (auto& [ts, pose] : traj) {
        Eigen::Vector3d t = pose.block<3,1>(0,3);
        Eigen::Quaterniond q(pose.block<3,3>(0,0));
        f << std::fixed << std::setprecision(6)
          << ts << " " << t.x() << " " << t.y() << " " << t.z()
          << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    return true;
}

bool OfflineRunner::writePLY(const std::string& path) {
    if (!engine_) return false;
    auto pts = engine_->getFullMap();
    if (pts.empty()) return false;

    std::ofstream f(path, std::ios::binary);
    f << "ply\nformat binary_little_endian 1.0\n";
    f << "element vertex " << pts.size() << "\n";
    f << "property float x\nproperty float y\nproperty float z\n";
    f << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    f << "end_header\n";

    for (auto& pt : pts) {
        f.write(reinterpret_cast<const char*>(&pt.x), 4);
        f.write(reinterpret_cast<const char*>(&pt.y), 4);
        f.write(reinterpret_cast<const char*>(&pt.z), 4);
        f.write(reinterpret_cast<const char*>(&pt.r), 1);
        f.write(reinterpret_cast<const char*>(&pt.g), 1);
        f.write(reinterpret_cast<const char*>(&pt.b), 1);
    }
    std::cout << "[Runner] PLY: " << pts.size() << " pts → " << path << std::endl;
    return true;
}

bool OfflineRunner::writeReport(const std::string& path, const RunResult& result) {
    std::ofstream f(path);
    if (!f.is_open()) return false;
    f << "DV-SLAM Evaluation Report\n";
    f << "=========================\n\n";
    f << std::fixed << std::setprecision(4);
    f << "ATE RMSE:  " << result.ate_rmse << " m (" << result.ate_rmse*100 << " cm)\n";
    f << "ATE Mean:  " << result.ate_mean << " m\n";
    f << "ATE Max:   " << result.ate_max << " m\n";
    f << "RPE Trans: " << result.rpe_trans << " m\n";
    f << "RPE Rot:   " << result.rpe_rot_deg << " deg\n";
    f << "Frames:    " << result.num_frames << "\n";
    f << "Total:     " << result.total_time_sec << " s\n";
    f << "Avg/frame: " << result.avg_frame_ms << " ms\n";
    return true;
}
