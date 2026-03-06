#include "../include/DepthVizEngine.hpp"
#include "../include/DV_LIOBackend.h"
#include "../include/DV_VIOManager.h"

#include <unordered_map>
#include <cmath>
#include <cstring>
#include <chrono>
#include <cstdio>

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

    // Camera-IMU extrinsic rotation for portrait orientation.
    // SLAMService point cloud uses camera-flipped frame: flipYZ(intrinsics)
    //   cam_x = landscape_right (= portrait up)
    //   cam_y = landscape_up    (= portrait left)
    //   cam_z = backward        (= toward user)
    // IMU device frame (portrait):
    //   dev_x = right, dev_y = up, dev_z = toward user
    // Relationship: cam = Rz(-90°) * dev → cam_x=dev_y, cam_y=-dev_x, cam_z=dev_z
    // Rotate IMU to match point cloud frame so ESKF is self-consistent.
    imu.acc = Eigen::Vector3d(acc.y(), -acc.x(), acc.z());
    imu.gyr = Eigen::Vector3d(gyr.y(), -gyr.x(), gyr.z());

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
    // Legacy interface — use pushImageAndDepth for visual tracking
    (void)timestamp; (void)imageData; (void)width; (void)height;
}

void DepthVizEngine::pushImageAndDepth(double timestamp,
                                        const uint8_t* gray, int gray_w, int gray_h,
                                        const float* depth, int depth_w, int depth_h,
                                        float fx, float fy, float cx, float cy,
                                        int full_w, int full_h) {
    std::lock_guard<std::mutex> lock(mtx_data_);

    // Set intrinsics once
    if (!vio_intrinsics_set_ && vio_) {
        vio_->setIntrinsics(fx, fy, cx, cy, full_w, full_h, depth_w, depth_h);
        vio_intrinsics_set_ = true;
    }

    // Store latest image frame
    latest_image_.timestamp = timestamp;
    latest_image_.gray_w = gray_w;
    latest_image_.gray_h = gray_h;
    latest_image_.depth_w = depth_w;
    latest_image_.depth_h = depth_h;
    latest_image_.gray.assign(gray, gray + gray_w * gray_h);
    latest_image_.depth.assign(depth, depth + depth_w * depth_h);
    latest_image_.valid = true;
}

void DepthVizEngine::pushARKitPose(double timestamp, const Eigen::Matrix4d& pose) {
    // Paper branch: ARKit VIO pose not used for ESKF.
    // Keep function signature for compatibility but no-op.
    (void)timestamp; (void)pose;
}

// ============================================================================
// Output Interfaces
// ============================================================================

Eigen::Matrix4d DepthVizEngine::getPose() {
    // Use cached pose (updated by run() loop) — never block on LIO mutex.
    // lio_->getCurrentPose() blocks on mtx_ which is held during ICP (500ms+).
    // Renderer calls getPose() at 60Hz and must not be blocked.
    Eigen::Matrix4d pose;
    {
        std::lock_guard<std::mutex> lock(mtx_state_);
        pose = last_optimized_pose_;
    }

    // Camera-IMU extrinsic correction for renderer.
    // ESKF body frame = camera-flipped frame (SLAMService applies flipYZ only).
    // Renderer applies: localToWorld = slamPose * rotateToARCamera
    //   where rotateToARCamera = flipYZ * Rz(90°) for portrait.
    // slamPose = T_eskf * Rz(90°) so that the composition correctly maps
    // camera intrinsics space → world space.
    Eigen::Matrix3d Rz90;
    Rz90 << 0, -1, 0,
            1,  0, 0,
            0,  0, 1;
    pose.block<3,3>(0,0) = pose.block<3,3>(0,0) * Rz90;
    return pose;
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

    // Diagnostic (first 10 non-keyframes)
    static int non_kf_log_count = 0;
    if (non_kf_log_count < 10) {
        printf("[ENG] not-KF: dt=%.4fm (th=%.2f) rot=%.2f° (th=%.1f°) pos=(%.3f,%.3f,%.3f)\n",
               dt.norm(), keyframe_config_.translation_threshold,
               angle * 180.0 / M_PI, keyframe_config_.rotation_threshold_deg,
               current_pose(0,3), current_pose(1,3), current_pose(2,3));
        non_kf_log_count++;
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

        // Pop from ring buffer — skip to latest frame to prevent queue buildup
        // When ICP fails fast (2ms), the loop outruns sensor input (30Hz),
        // starving VIO of images. Always process only the newest frame.
        bool has_data = false;
        {
            std::lock_guard<std::mutex> lock(mtx_data_);
            // Drain to latest: pop all, keep last
            bool got_any = false;
            while (cloud_ring_buf_.pop(timestamp, local_xyz.data(), local_conf.data(), local_rgb.data(), n_points)) {
                got_any = true;
            }
            has_data = got_any;
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

        if (bundled.empty()) {
            printf("[ENG] frame: %d raw pts → 0 after B&D → SKIP (vs=%.0fmm, minD=%d, minC=%.1f)\n",
                   n_points,
                   bundle_config_.voxel_size * 1000.f,
                   bundle_config_.min_density,
                   bundle_config_.min_avg_confidence);
            continue;
        }

        // B&D 결과 로그 (처음 5프레임만)
        {
            std::lock_guard<std::mutex> lock(mtx_state_);
            if (profiling_.total_frames < 5) {
                printf("[ENG] frame: %d raw pts → %d after B&D (vs=%.0fmm)\n",
                       n_points, (int)bundled.size(), bundle_config_.voxel_size * 1000.f);
            }
        }

        // Stamp frame timestamp on all bundled points
        for (auto& pt : bundled) {
            pt.timestamp = timestamp;
        }

        // Drain IMU buffer and handle initialization
        {
            std::vector<DV::IMUData> imu_batch;
            {
                std::lock_guard<std::mutex> lock(mtx_data_);
                while (!imu_buf_.empty() && imu_buf_.front().timestamp <= timestamp) {
                    imu_batch.push_back(imu_buf_.front());
                    imu_buf_.pop_front();
                }
            }

            // Paper branch: IMU static init phase
            if (init_phase_ == InitPhase::COLLECTING_IMU && lio_) {
                size_t init_end_idx = 0;
                for (size_t i = 0; i < imu_batch.size(); i++) {
                    if (lio_->initFromIMU(imu_batch[i])) {
                        init_phase_ = InitPhase::READY;
                        init_end_idx = i + 1;
                        printf("[Engine] IMU static init complete → READY\n");
                        break;
                    }
                }
                if (init_phase_ != InitPhase::READY) {
                    continue;  // Still collecting IMU samples, skip LIO
                }
                // Remove init samples — only keep post-init IMU for first predict
                imu_batch.erase(imu_batch.begin(), imu_batch.begin() + init_end_idx);
            }

            // Run ESKF prediction with remaining IMU data
            if (ablation_.enable_imu && !imu_batch.empty() && lio_) {
                lio_->processIMU(imu_batch);
            }
        }

        // Visual tracking: ESKF update with reprojection residuals (every frame)
        {
            ImageFrame img_copy;
            {
                std::lock_guard<std::mutex> lock(mtx_data_);
                if (latest_image_.valid && std::abs(latest_image_.timestamp - timestamp) < 0.05) {
                    img_copy = latest_image_;
                    latest_image_.valid = false; // Consume
                }
            }

            if (img_copy.valid && vio_ && lio_) {
                // Get ESKF predicted pose for landmark initialization
                Eigen::Matrix4d eskf_pred = lio_->getCurrentPose();

                int n_vis = vio_->processFrame(
                    img_copy.gray.data(), img_copy.gray_w, img_copy.gray_h,
                    img_copy.depth.data(), img_copy.depth_w, img_copy.depth_h,
                    eskf_pred);

                // Run ESKF visual update if we have enough tracked features
                if (n_vis >= 5) {
                    lio_->processVisual(vio_->getLandmarks(), vio_->getWorkingIntrinsics());
                }
            }
        }

        // Get pose prior (ESKF predicted state, no ARKit)
        Eigen::Matrix4d prior_pose;
        {
            std::lock_guard<std::mutex> lock(mtx_state_);
            prior_pose = last_optimized_pose_;
        }

        // First frame: initialize and continue
        if (first_frame_.load()) {
            if (lio_ && ablation_.enable_lio) {
                lio_->process(bundled);
            }
            {
                std::lock_guard<std::mutex> lock(mtx_state_);
                Eigen::Matrix4d init_pose = (lio_) ? lio_->getCurrentPose() : Eigen::Matrix4d::Identity();
                last_optimized_pose_ = init_pose;
                last_keyframe_pose_ = init_pose;
                profiling_.total_frames++;

                // Transform to world frame for both display and full map
                Eigen::Matrix3d R = init_pose.block<3, 3>(0, 0);
                Eigen::Vector3d t = init_pose.block<3, 1>(0, 3);

                display_cloud_.clear();
                display_cloud_.reserve(bundled.size());
                for (const auto& pt : bundled) {
                    Eigen::Vector3d p_camera(pt.x, pt.y, pt.z);
                    Eigen::Vector3d p_world = R * p_camera + t;
                    DV::DVPoint3D world_pt = pt;
                    world_pt.x = static_cast<float>(p_world.x());
                    world_pt.y = static_cast<float>(p_world.y());
                    world_pt.z = static_cast<float>(p_world.z());
                    display_cloud_.push_back(world_pt);
                    if (full_map_.size() < MAX_FULL_MAP_POINTS) {
                        full_map_.push_back(world_pt);
                    }
                }
            }
            first_frame_.store(false);
            continue;
        }

        // Keyframe check (use ESKF predicted pose)
        Eigen::Matrix4d eskf_pose = (lio_) ? lio_->getCurrentPose() : prior_pose;
        if (!isKeyframe(eskf_pose)) {
            // Non-keyframe: update pose and display cloud (world-frame transform)
            Eigen::Matrix3d R_disp = eskf_pose.block<3, 3>(0, 0);
            Eigen::Vector3d t_disp = eskf_pose.block<3, 1>(0, 3);
            std::vector<DV::DVPoint3D> world_display;
            world_display.reserve(bundled.size());
            for (const auto& pt : bundled) {
                Eigen::Vector3d pw = R_disp * Eigen::Vector3d(pt.x, pt.y, pt.z) + t_disp;
                DV::DVPoint3D wp = pt;
                wp.x = static_cast<float>(pw.x());
                wp.y = static_cast<float>(pw.y());
                wp.z = static_cast<float>(pw.z());
                world_display.push_back(wp);
            }

            {
                std::lock_guard<std::mutex> lock(mtx_state_);
                last_optimized_pose_ = eskf_pose;
                display_cloud_ = std::move(world_display);
                profiling_.total_frames++;
            }

            auto frame_end = std::chrono::high_resolution_clock::now();
            profiling_.total_pipeline_ms += std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
            continue;
        }

        // Log keyframe info
        {
            Eigen::Vector3d dt_kf = prior_pose.block<3, 1>(0, 3) - last_keyframe_pose_.block<3, 1>(0, 3);
            Eigen::Matrix3d dR_kf = last_keyframe_pose_.block<3, 3>(0, 0).transpose() * prior_pose.block<3, 3>(0, 0);
            double cos_a = std::max(-1.0, std::min(1.0, (dR_kf.trace() - 1.0) * 0.5));
            double angle_deg = std::acos(cos_a) * 180.0 / M_PI;
            printf("[ENG] KEYFRAME #%d raw=%d bd=%zu move=%.3fm rot=%.1f°\n",
                   profiling_.keyframes + 1, n_points, bundled.size(),
                   dt_kf.norm(), angle_deg);
        }

        // LIO optimization (or skip if ablation disabled)
        Eigen::Matrix4d refined_pose = eskf_pose;
        bool lio_success = false;
        if (ablation_.enable_lio && lio_) {
            auto lio_start = std::chrono::high_resolution_clock::now();
            refined_pose = lio_->process(bundled);
            auto lio_end = std::chrono::high_resolution_clock::now();
            double lio_ms = std::chrono::duration<double, std::milli>(lio_end - lio_start).count();

            // Check if LIO actually produced an observation update (obs > 0)
            // process() returns eskf pose regardless, but we can detect failure
            // by checking if the pose changed from pre-ICP state
            lio_success = (lio_ms > 5.0); // obs=0 completes in <3ms, real ICP takes >5ms

            if (lio_success) {
                consecutive_icp_failures_ = 0;
            } else {
                consecutive_icp_failures_++;
                if (consecutive_icp_failures_ == kMaxICPFailures) {
                    printf("[ENG] *** ICP FAILED %d consecutive times — RESETTING MAP ***\n", kMaxICPFailures);
                    // Clear voxel map but keep ESKF state (pose, velocity, biases)
                    lio_->resetMap();
                    // Insert current frame as new map seed
                    refined_pose = lio_->process(bundled);
                    consecutive_icp_failures_ = 0;
                    printf("[ENG] Map reset complete. Resuming from ESKF pose.\n");
                }
            }

            {
                std::lock_guard<std::mutex> lock(mtx_state_);
                profiling_.total_lio_ms += lio_ms;
                profiling_.keyframes++;
                printf("[ENG] LIO %.1fms %s | total KF=%d avgLIO=%.1fms\n",
                       lio_ms, lio_success ? "OK" : "FAIL",
                       profiling_.keyframes,
                       profiling_.keyframes > 0 ? profiling_.total_lio_ms / profiling_.keyframes : 0.0);
            }
        }

        // Update state and accumulate full map with RGB (world-frame transform)
        {
            std::lock_guard<std::mutex> lock(mtx_state_);
            last_optimized_pose_ = refined_pose;
            last_keyframe_pose_ = refined_pose;

            // Transform display cloud to world frame
            Eigen::Matrix3d R_w = refined_pose.block<3, 3>(0, 0);
            Eigen::Vector3d t_w = refined_pose.block<3, 1>(0, 3);
            display_cloud_.clear();
            display_cloud_.reserve(bundled.size());
            for (const auto& pt : bundled) {
                Eigen::Vector3d pw = R_w * Eigen::Vector3d(pt.x, pt.y, pt.z) + t_w;
                DV::DVPoint3D wp = pt;
                wp.x = static_cast<float>(pw.x());
                wp.y = static_cast<float>(pw.y());
                wp.z = static_cast<float>(pw.z());
                display_cloud_.push_back(wp);
            }

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
            double total_ms = std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
            profiling_.total_pipeline_ms += total_ms;

            // Summary every 10 keyframes
            if (profiling_.keyframes % 10 == 0) {
                printf("[ENG] === KF#%d summary: frames=%d map=%zu avgLIO=%.1fms avgPipe=%.1fms B&D_ratio=%.1f%% ===\n",
                       profiling_.keyframes,
                       profiling_.total_frames,
                       full_map_.size(),
                       profiling_.keyframes > 0 ? profiling_.total_lio_ms / profiling_.keyframes : 0.0,
                       profiling_.total_frames > 0 ? profiling_.total_pipeline_ms / profiling_.total_frames : 0.0,
                       profiling_.total_input_points > 0 ? 100.0 * profiling_.total_output_points / profiling_.total_input_points : 0.0);
            }
        }

        // VIO landmarks use world-frame coordinates initialized at detection time.
        // No feedback needed — ESKF state is shared between visual and LiDAR updates.
    }
}
