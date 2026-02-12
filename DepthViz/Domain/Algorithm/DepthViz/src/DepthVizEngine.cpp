#include "../include/DepthVizEngine.hpp"
#include "../include/DV_LIOBackend.h"
#include "../include/DV_VIOManager.h"

#include <unordered_map>
#include <cmath>
#include <cstring>
#include <chrono>

DepthVizEngine::DepthVizEngine() = default;

DepthVizEngine::~DepthVizEngine() {
    stop();
}

void DepthVizEngine::init() {
    lio_ = std::make_shared<DV_LIOBackend>();
    lio_->init();

    vio_ = std::make_shared<DV_VIOManager>();
    vio_->init();

    last_optimized_pose_ = Eigen::Matrix4d::Identity();
    last_keyframe_pose_ = Eigen::Matrix4d::Identity();
    first_frame_.store(true);

    // Reserve space for full map with RGB
    full_map_.clear();
    full_map_.reserve(MAX_FULL_MAP_POINTS / 4); // Start with 500K, will grow as needed
}

void DepthVizEngine::start() {
    if (is_running_.load()) return;

    // Forward ablation config to LIO backend
    if (lio_) {
        lio_->use_confidence_weight_ = ablation_.enable_confidence_weight;
        lio_->use_tls_ = ablation_.enable_tls;
    }

    is_running_.store(true);
    thread_ = std::make_unique<std::thread>(&DepthVizEngine::run, this);
}

void DepthVizEngine::stop() {
    is_running_.store(false);
    if (thread_ && thread_->joinable()) {
        thread_->join();
    }
    thread_.reset();
}

// ============================================================================
// Input Interfaces
// ============================================================================

void DepthVizEngine::pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
    std::lock_guard<std::mutex> lock(mtx_data_);
    DV::IMUData imu;
    imu.timestamp = timestamp;
    imu.acc = acc;
    imu.gyr = gyr;
    imu_buf_.push_back(imu);

    // Keep buffer bounded
    while (imu_buf_.size() > 2000) {
        imu_buf_.pop_front();
    }
}

void DepthVizEngine::pushPointCloud(double timestamp, const float* xyz, const uint8_t* conf, const uint8_t* rgb, int count) {
    if (!xyz || !conf || count <= 0) return;
    std::lock_guard<std::mutex> lock(mtx_data_);
    cloud_ring_buf_.push(timestamp, xyz, conf, rgb, count);
}

void DepthVizEngine::pushImage(double timestamp, const void* imageData, int width, int height) {
    // Not used in DV-SLAM (ARKit handles visual odometry)
    (void)timestamp; (void)imageData; (void)width; (void)height;
}

void DepthVizEngine::pushARKitPose(double timestamp, const Eigen::Matrix4d& pose) {
    std::lock_guard<std::mutex> lock(mtx_state_);
    arkit_pose_ = pose;
    has_arkit_pose_.store(true);

    if (vio_) {
        vio_->pushARKitPose(timestamp, pose);
    }
}

// ============================================================================
// Output Interfaces
// ============================================================================

Eigen::Matrix4d DepthVizEngine::getPose() {
    std::lock_guard<std::mutex> lock(mtx_state_);
    return last_optimized_pose_;
}

std::vector<DV::DVPoint3D> DepthVizEngine::getDisplayCloud() {
    std::lock_guard<std::mutex> lock(mtx_state_);
    return display_cloud_;
}

std::vector<DV::DVPoint3D> DepthVizEngine::getFullMap() {
    // Return accumulated full map with RGB colors preserved
    std::lock_guard<std::mutex> lock(mtx_state_);
    return full_map_;
}

// ============================================================================
// Bundle & Discard — core DV-SLAM preprocessing
// ============================================================================

std::vector<DV::DVPoint3D> DepthVizEngine::bundleAndDiscard(
    const float* xyz, const uint8_t* conf, const uint8_t* rgb, int count)
{
    if (count <= 0) return {};

    const float voxel_size = bundle_config_.voxel_size;
    const int min_density = bundle_config_.min_density;
    const float min_avg_conf = bundle_config_.min_avg_confidence;

    // Voxel accumulation structure with RGB
    struct VoxelAccum {
        float sum_x = 0.f, sum_y = 0.f, sum_z = 0.f;
        float sum_confidence = 0.f;
        float sum_r = 0.f, sum_g = 0.f, sum_b = 0.f;
        int point_count = 0;
    };

    std::unordered_map<int64_t, VoxelAccum> voxels;
    voxels.reserve(count / 5); // Expect ~5 points per voxel on average

    for (int i = 0; i < count; i++) {
        uint8_t c = conf[i];

        // Step 1: Hard gate — skip confidence=0 immediately
        if (c == 0) continue;

        float px = xyz[i * 3];
        float py = xyz[i * 3 + 1];
        float pz = xyz[i * 3 + 2];

        // Skip invalid points (NaN or very far)
        if (std::isnan(px) || std::isnan(py) || std::isnan(pz)) continue;
        if (px * px + py * py + pz * pz > 100.f) continue; // >10m

        // Step 2: Hash into voxels
        int64_t vidx = DV::DVPoint3D::computeVoxelIdx(px, py, pz, voxel_size);

        auto& v = voxels[vidx];
        v.sum_x += px;
        v.sum_y += py;
        v.sum_z += pz;
        v.sum_confidence += static_cast<float>(c);
        if (rgb) {
            v.sum_r += static_cast<float>(rgb[i * 3]);
            v.sum_g += static_cast<float>(rgb[i * 3 + 1]);
            v.sum_b += static_cast<float>(rgb[i * 3 + 2]);
        } else {
            v.sum_r += 128.f;
            v.sum_g += 128.f;
            v.sum_b += 128.f;
        }
        v.point_count++;
    }

    // Step 3 & 4: Discard sparse/low-confidence voxels, emit centroids with averaged color
    std::vector<DV::DVPoint3D> result;
    result.reserve(voxels.size());

    for (const auto& pair : voxels) {
        const VoxelAccum& v = pair.second;

        // Density gate
        if (v.point_count < min_density) continue;

        // Average confidence gate
        float avg_conf = v.sum_confidence / static_cast<float>(v.point_count);
        if (avg_conf < min_avg_conf) continue;

        // Emit centroid with averaged color
        DV::DVPoint3D pt;
        float inv_count = 1.0f / static_cast<float>(v.point_count);
        pt.x = v.sum_x * inv_count;
        pt.y = v.sum_y * inv_count;
        pt.z = v.sum_z * inv_count;
        pt.intensity = avg_conf;
        pt.confidence = avg_conf;
        pt.r = static_cast<uint8_t>(std::min(255.f, v.sum_r * inv_count));
        pt.g = static_cast<uint8_t>(std::min(255.f, v.sum_g * inv_count));
        pt.b = static_cast<uint8_t>(std::min(255.f, v.sum_b * inv_count));
        pt.voxel_idx = pair.first;
        result.push_back(pt);
    }

    return result;
}

// ============================================================================
// Keyframe Check
// ============================================================================

bool DepthVizEngine::isKeyframe(const Eigen::Matrix4d& current_pose) {
    // Translation check
    Eigen::Vector3d dt = current_pose.block<3, 1>(0, 3) - last_keyframe_pose_.block<3, 1>(0, 3);
    if (dt.norm() >= keyframe_config_.translation_threshold) {
        return true;
    }

    // Rotation check
    Eigen::Matrix3d dR = last_keyframe_pose_.block<3, 3>(0, 0).transpose() * current_pose.block<3, 3>(0, 0);
    double cos_angle = (dR.trace() - 1.0) * 0.5;
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    double angle = std::acos(cos_angle);
    if (angle >= keyframe_config_.rotation_threshold_rad()) {
        return true;
    }

    return false;
}

// ============================================================================
// Main Processing Loop
// ============================================================================

void DepthVizEngine::run() {
    // Local buffers for safe copy-out from ring buffer
    std::vector<float> local_xyz(MAX_POINTS_PER_FRAME * 3);
    std::vector<uint8_t> local_conf(MAX_POINTS_PER_FRAME);
    std::vector<uint8_t> local_rgb(MAX_POINTS_PER_FRAME * 3);

    while (is_running_.load()) {
        double timestamp = 0.0;
        int n_points = 0;

        // Pop from ring buffer — copies data into local buffers under the lock
        bool has_data = false;
        {
            std::lock_guard<std::mutex> lock(mtx_data_);
            has_data = cloud_ring_buf_.pop(timestamp, local_xyz.data(), local_conf.data(), local_rgb.data(), n_points);
        }

        if (!has_data) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto frame_start = std::chrono::high_resolution_clock::now();

        // Bundle & Discard (or pass-through if ablation disabled)
        std::vector<DV::DVPoint3D> bundled;
        {
            auto bd_start = std::chrono::high_resolution_clock::now();

            if (ablation_.enable_bundle_discard) {
                bundled = bundleAndDiscard(local_xyz.data(), local_conf.data(), local_rgb.data(), n_points);
            } else {
                // Pass-through: convert raw points without filtering, keeping RGB
                bundled.reserve(n_points);
                for (int i = 0; i < n_points; i++) {
                    if (local_conf[i] == 0) continue; // Still skip invalid
                    DV::DVPoint3D pt;
                    pt.x = local_xyz[i * 3];
                    pt.y = local_xyz[i * 3 + 1];
                    pt.z = local_xyz[i * 3 + 2];
                    pt.confidence = static_cast<float>(local_conf[i]);
                    pt.intensity = pt.confidence;
                    pt.r = local_rgb[i * 3];
                    pt.g = local_rgb[i * 3 + 1];
                    pt.b = local_rgb[i * 3 + 2];
                    bundled.push_back(pt);
                }
            }

            auto bd_end = std::chrono::high_resolution_clock::now();
            double bd_ms = std::chrono::duration<double, std::milli>(bd_end - bd_start).count();

            std::lock_guard<std::mutex> lock(mtx_state_);
            profiling_.total_bundle_discard_ms += bd_ms;
            profiling_.total_input_points += n_points;
            profiling_.total_output_points += static_cast<int>(bundled.size());
        }

        if (bundled.empty()) continue;

        // Stamp frame timestamp on all bundled points
        for (auto& pt : bundled) {
            pt.timestamp = timestamp;
        }

        // Get pose prior (ARKit or last optimized)
        Eigen::Matrix4d prior_pose;
        {
            std::lock_guard<std::mutex> lock(mtx_state_);
            if (has_arkit_pose_.load()) {
                prior_pose = arkit_pose_;
            } else {
                prior_pose = last_optimized_pose_;
            }
        }

        // First frame: initialize and continue
        if (first_frame_.load()) {
            if (lio_ && ablation_.enable_lio) {
                lio_->process(prior_pose, bundled);
            }
            {
                std::lock_guard<std::mutex> lock(mtx_state_);
                last_optimized_pose_ = prior_pose;
                last_keyframe_pose_ = prior_pose;
                display_cloud_ = bundled;
                profiling_.total_frames++;

                // Accumulate first frame into full_map_ with RGB
                Eigen::Matrix3d R = prior_pose.block<3, 3>(0, 0);
                Eigen::Vector3d t = prior_pose.block<3, 1>(0, 3);
                for (const auto& pt : bundled) {
                    if (full_map_.size() >= MAX_FULL_MAP_POINTS) break;
                    Eigen::Vector3d p_camera(pt.x, pt.y, pt.z);
                    Eigen::Vector3d p_world = R * p_camera + t;
                    DV::DVPoint3D world_pt;
                    world_pt.x = static_cast<float>(p_world.x());
                    world_pt.y = static_cast<float>(p_world.y());
                    world_pt.z = static_cast<float>(p_world.z());
                    world_pt.intensity = pt.intensity;
                    world_pt.confidence = pt.confidence;
                    world_pt.r = pt.r;
                    world_pt.g = pt.g;
                    world_pt.b = pt.b;
                    world_pt.timestamp = pt.timestamp;
                    full_map_.push_back(world_pt);
                }
            }
            first_frame_.store(false);
            continue;
        }

        // Drain IMU buffer and run ESKF prediction
        if (ablation_.enable_imu) {
            std::vector<DV::IMUData> imu_batch;
            {
                std::lock_guard<std::mutex> lock(mtx_data_);
                while (!imu_buf_.empty() && imu_buf_.front().timestamp <= timestamp) {
                    imu_batch.push_back(imu_buf_.front());
                    imu_buf_.pop_front();
                }
            }
            if (!imu_batch.empty() && lio_) {
                lio_->processIMU(imu_batch);
            }
        }

        // Keyframe check
        if (!isKeyframe(prior_pose)) {
            std::lock_guard<std::mutex> lock(mtx_state_);
            last_optimized_pose_ = prior_pose;
            display_cloud_ = bundled;
            profiling_.total_frames++;

            auto frame_end = std::chrono::high_resolution_clock::now();
            profiling_.total_pipeline_ms += std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
            continue;
        }

        // LIO optimization (or skip if ablation disabled → ARKit-only)
        Eigen::Matrix4d refined_pose = prior_pose;
        if (ablation_.enable_lio && lio_) {
            auto lio_start = std::chrono::high_resolution_clock::now();
            refined_pose = lio_->process(prior_pose, bundled);
            auto lio_end = std::chrono::high_resolution_clock::now();

            std::lock_guard<std::mutex> lock(mtx_state_);
            profiling_.total_lio_ms += std::chrono::duration<double, std::milli>(lio_end - lio_start).count();
            profiling_.keyframes++;
        }

        // Update state and accumulate full map with RGB
        {
            std::lock_guard<std::mutex> lock(mtx_state_);
            last_optimized_pose_ = refined_pose;
            last_keyframe_pose_ = refined_pose;
            display_cloud_ = bundled;
            profiling_.total_frames++;

            // Accumulate bundled points into full_map_ (transformed to world frame)
            // Only accumulate on keyframes to avoid excessive memory usage
            if (full_map_.size() < MAX_FULL_MAP_POINTS) {
                Eigen::Matrix3d R = refined_pose.block<3, 3>(0, 0);
                Eigen::Vector3d t = refined_pose.block<3, 1>(0, 3);

                for (const auto& pt : bundled) {
                    if (full_map_.size() >= MAX_FULL_MAP_POINTS) break;

                    // Transform camera-frame point to world frame (T_camera_imu ≈ I)
                    Eigen::Vector3d p_camera(pt.x, pt.y, pt.z);
                    Eigen::Vector3d p_world = R * p_camera + t;

                    DV::DVPoint3D world_pt;
                    world_pt.x = static_cast<float>(p_world.x());
                    world_pt.y = static_cast<float>(p_world.y());
                    world_pt.z = static_cast<float>(p_world.z());
                    world_pt.intensity = pt.intensity;
                    world_pt.confidence = pt.confidence;
                    world_pt.r = pt.r;
                    world_pt.g = pt.g;
                    world_pt.b = pt.b;
                    world_pt.timestamp = pt.timestamp;
                    world_pt.voxel_idx = pt.voxel_idx;
                    full_map_.push_back(world_pt);
                }
            }

            auto frame_end = std::chrono::high_resolution_clock::now();
            profiling_.total_pipeline_ms += std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
        }

        // Feedback to VIO
        if (vio_) {
            vio_->updatePoseFromLIO(refined_pose);
        }
    }
}
