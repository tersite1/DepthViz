#include "../include/DV_LIOBackend.h"
#include "../include/DV_RobustKernels.h"

#include <cmath>
#include <cstdio>

DV_LIOBackend::DV_LIOBackend() = default;
DV_LIOBackend::~DV_LIOBackend() = default;

void DV_LIOBackend::init() {
    map_ = std::make_unique<DV::DV_VoxelHashMap>(0.10f, 500000);
    eskf_ = std::make_unique<DV::DV_ESKF>(eskf_opts_);
    // Paper branch: ESKF initialized via initFromIMU(), not here

    current_pose_ = Eigen::Matrix4d::Identity();
    first_frame_ = true;
}

void DV_LIOBackend::resetMap() {
    std::lock_guard<std::mutex> lock(mtx_);
    // Clear voxel map but preserve ESKF state (pose, velocity, biases)
    map_ = std::make_unique<DV::DV_VoxelHashMap>(0.10f, 500000);
    first_frame_ = true;  // Next process() call will insert points as new map
    map_seed_count_ = 0;  // Re-seed map from multiple viewpoints
    current_pose_ = eskf_ ? eskf_->getPoseMatrix() : current_pose_;
    printf("[LIO] Map reset — ESKF state preserved, pose=(%.3f,%.3f,%.3f)\n",
           current_pose_(0,3), current_pose_(1,3), current_pose_(2,3));
}

void DV_LIOBackend::processIMU(const std::vector<DV::IMUData>& imu_data) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!eskf_) return;
    // Paper branch: allow IMU processing before first_frame_ for initFromIMU collection.
    // ESKF::predict() internally checks initialized_ and returns early if not ready.

    static int imu_log_counter = 0;
    for (const auto& imu : imu_data) {
        double dt = 0.0;
        if (last_imu_timestamp_ > 0.0) {
            dt = imu.timestamp - last_imu_timestamp_;
        }
        last_imu_timestamp_ = imu.timestamp;

        if (dt > 0.0 && dt < 0.5) {
            eskf_->predict(imu, dt);
        }
    }

    // Log IMU-predicted state every 50 batches (~1sec at 50Hz)
    if (++imu_log_counter % 50 == 0 && eskf_->getState().v.norm() > 0.01) {
        auto& s = eskf_->getState();
        printf("[TRAJ-IMU] pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f) |v|=%.3f ba=(%.4f,%.4f,%.4f)\n",
               s.p.x(), s.p.y(), s.p.z(),
               s.v.x(), s.v.y(), s.v.z(), s.v.norm(),
               s.ba.x(), s.ba.y(), s.ba.z());
    }
}

bool DV_LIOBackend::initFromIMU(const DV::IMUData& imu) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!eskf_) return false;
    bool done = eskf_->initFromIMU(imu);
    if (done) {
        // Seed last_imu_timestamp_ so processIMU computes correct dt
        // from the very first post-init sample (no wasted first-sample skip)
        last_imu_timestamp_ = imu.timestamp;
    }
    return done;
}

bool DV_LIOBackend::isInitialized() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return eskf_ && !first_frame_;
}

Eigen::Matrix4d DV_LIOBackend::process(
    const std::vector<DV::DVPoint3D>& points)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (points.empty() || !eskf_) return current_pose_;

    // First frame: insert points at ESKF's current pose (from static init)
    if (first_frame_) {
        current_pose_ = eskf_->getPoseMatrix();
        insertPoints(current_pose_, points);
        first_frame_ = false;
        map_seed_count_ = 1;
        printf("[LIO] First frame: %zu pts inserted at ESKF pose (seed 1/%d)\n",
               points.size(), kMapSeedFrames);
        return current_pose_;
    }

    // FIX #1: Map seeding — first N keyframes insert at IMU-only pose, no ICP.
    // Single-viewpoint maps have rank-deficient geometry for point-to-plane ICP.
    // Build map from multiple viewpoints before attempting ICP refinement.
    if (map_seed_count_ < kMapSeedFrames) {
        current_pose_ = eskf_->getPoseMatrix();
        insertPoints(current_pose_, points);
        map_seed_count_++;
        printf("[LIO] Map seeding: %zu pts inserted at IMU pose (seed %d/%d)\n",
               points.size(), map_seed_count_, kMapSeedFrames);
        return current_pose_;
    }

    // Prior = ESKF IMU-predicted state (no ARKit anchoring)
    Eigen::Matrix4d pre_icp_pose = eskf_->getPoseMatrix();
    Eigen::Vector3d pre_icp_pos = pre_icp_pose.block<3,1>(0,3);
    Eigen::Vector3d pre_icp_vel = eskf_->getState().v;
    printf("[TRAJ] pre-ICP  pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f) |v|=%.3f m/s\n",
           pre_icp_pos.x(), pre_icp_pos.y(), pre_icp_pos.z(),
           pre_icp_vel.x(), pre_icp_vel.y(), pre_icp_vel.z(),
           pre_icp_vel.norm());

    // Capture points and map pointer for the observation lambda
    const auto& pts = points;
    const auto* map_ptr = map_.get();
    bool use_conf_w = use_confidence_weight_;
    bool use_tls = use_tls_;

    // Build observation function for ESKF
    auto obs_func = [&pts, map_ptr, use_conf_w, use_tls](
        const DV::SysState& state,
        Eigen::MatrixXd& H,
        Eigen::VectorXd& residual,
        Eigen::MatrixXd& R_obs) -> bool
    {
        Eigen::Matrix3d R = state.R;
        Eigen::Vector3d t = state.p;

        struct Observation {
            Eigen::Vector3d normal_d;
            Eigen::Vector3d p_camera_d;
            double residual_val;
            double weight;
        };
        std::vector<Observation> observations;
        observations.reserve(pts.size());

        for (const auto& pt : pts) {
            Eigen::Vector3d p_camera(pt.x, pt.y, pt.z);
            Eigen::Vector3d p_world = R * p_camera + t;
            Eigen::Vector3f p_world_f = p_world.cast<float>();

            DV::KNNResult knn;
            if (!map_ptr->getTopK(p_world_f, knn, 5)) continue;

            Eigen::Vector3f normal_f, centroid_f;
            if (!DV::DV_VoxelHashMap::fitPlane(knn, normal_f, centroid_f)) continue;

            // Max correspondence distance: reject matches > 0.3m
            Eigen::Vector3f diff = p_world_f - centroid_f;
            if (diff.squaredNorm() > 0.09f) continue; // 0.3m² = 0.09

            float r = normal_f.dot(diff);

            float conf_w = use_conf_w ? DepthViz::getConfidenceWeight(pt.confidence) : 1.0f;
            if (conf_w <= 0.f) continue;

            float tls_w = use_tls ? DepthViz::computeTLSWeight(r, 0.10f) : 1.0f;
            if (tls_w <= 0.f) continue;

            Observation obs;
            obs.normal_d = normal_f.cast<double>();
            obs.p_camera_d = p_camera;
            obs.residual_val = static_cast<double>(r);
            obs.weight = static_cast<double>(conf_w * tls_w);
            observations.push_back(obs);
        }

        int n_total = static_cast<int>(observations.size());
        if (n_total < 10) {
            printf("[LIO] obs=%d (< 10 min) from %zu pts → SKIP (voxel=%.0fmm, 5x5x5 search)\n",
                   n_total, pts.size(), map_ptr->voxelSize() * 1000.f);
            return false;
        }

        // Cap observations at 100 to keep LDLT decomposition fast.
        // S = H*P*H^T+R is NxN → O(N³): 500³=125M vs 100³=1M (125x faster).
        // Stride-sample to maintain spatial distribution.
        constexpr int kMaxObs = 100;
        int stride = 1;
        if (n_total > kMaxObs) {
            stride = (n_total + kMaxObs - 1) / kMaxObs;
        }
        int n = 0;
        for (int j = 0; j < n_total; j += stride) n++;

        H = Eigen::MatrixXd::Zero(n, 18);
        residual = Eigen::VectorXd::Zero(n);
        R_obs = Eigen::MatrixXd::Zero(n, n);

        int idx = 0;
        for (int j = 0; j < n_total; j += stride) {
            const auto& obs = observations[j];

            Eigen::Matrix3d p_camera_hat = DV::SO3::hat(obs.p_camera_d);
            H.block<1, 3>(idx, 0) = -obs.normal_d.transpose() * R * p_camera_hat;
            H.block<1, 3>(idx, 3) = obs.normal_d.transpose();
            residual(idx) = -obs.residual_val;

            double sigma2 = 0.01 * 0.01;
            R_obs(idx, idx) = sigma2 / std::max(obs.weight, 0.01);
            idx++;
        }

        return true;
    };

    // Run iterated Kalman update
    bool success = eskf_->updateObserve(obs_func);

    if (success) {
        current_pose_ = eskf_->getPoseMatrix();
        Eigen::Vector3d post_pos = current_pose_.block<3,1>(0,3);
        Eigen::Vector3d post_vel = eskf_->getState().v;
        Eigen::Vector3d correction = post_pos - pre_icp_pos;
        // Covariance-based divergence check (replaces ARKit comparison)
        double pos_trace = eskf_->getCovariance().block<3,3>(3,3).trace();
        int map_size = map_ ? static_cast<int>(map_->size()) : 0;
        printf("[TRAJ] post-ICP pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f) |v|=%.3f\n",
               post_pos.x(), post_pos.y(), post_pos.z(),
               post_vel.x(), post_vel.y(), post_vel.z(), post_vel.norm());
        printf("[TRAJ] ICP correction=(%.4f,%.4f,%.4f) |corr|=%.4fm cov_tr=%.4f map=%d\n",
               correction.x(), correction.y(), correction.z(),
               correction.norm(), pos_trace, map_size);
        if (pos_trace > 1.0) {
            printf("[LIO] WARNING: high uncertainty trace=%.3f\n", pos_trace);
        }
        // FIX #2: Only insert points on ICP success — prevents map corruption
        // from misaligned points when ICP produces wrong pose.
        insertPoints(current_pose_, points);
    } else {
        // ICP failed: dampen velocity to prevent runaway drift.
        // Without LiDAR correction, IMU-only velocity is unreliable.
        DV::SysState s = eskf_->getState();
        s.v *= 0.5; // Halve velocity on each ICP failure
        eskf_->setState(s);
        current_pose_ = eskf_->getPoseMatrix();
        printf("[TRAJ] ICP FAIL — vel damped: (%.3f,%.3f,%.3f) → |v|=%.3f\n",
               s.v.x(), s.v.y(), s.v.z(), s.v.norm());
        // Do NOT insert points — pose is unreliable, would corrupt map
    }

    return current_pose_;
}

DV_LIOBackend::PlaneResidual DV_LIOBackend::computePointToPlaneResidual(
    const Eigen::Vector3f& query_world, float confidence)
{
    PlaneResidual result;
    result.valid = false;

    DV::KNNResult knn;
    if (!map_->getTopK(query_world, knn, 5)) return result;

    if (!DV::DV_VoxelHashMap::fitPlane(knn, result.normal, result.centroid)) return result;

    Eigen::Vector3f diff = query_world - result.centroid;
    result.residual = result.normal.dot(diff);
    result.valid = true;
    return result;
}

void DV_LIOBackend::insertPoints(
    const Eigen::Matrix4d& pose,
    const std::vector<DV::DVPoint3D>& points)
{
    Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
    Eigen::Vector3d t = pose.block<3, 1>(0, 3);

    std::vector<Eigen::Vector3f> world_points;
    world_points.reserve(points.size());

    for (const auto& pt : points) {
        Eigen::Vector3d p_camera(pt.x, pt.y, pt.z); // Camera frame (T_camera_imu ≈ I)
        Eigen::Vector3d p_world = R * p_camera + t;
        world_points.push_back(p_world.cast<float>());
    }

    map_->insert(world_points);
}

std::vector<DV::DVPoint3D> DV_LIOBackend::getMapPoints() {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!map_) return {};

    auto centroids = map_->getAllPoints();
    std::vector<DV::DVPoint3D> result;
    result.reserve(centroids.size());

    for (const auto& c : centroids) {
        DV::DVPoint3D pt;
        pt.x = c.x();
        pt.y = c.y();
        pt.z = c.z();
        pt.intensity = 1.0f;
        pt.confidence = 2.0f;
        result.push_back(pt);
    }

    return result;
}

Eigen::Matrix4d DV_LIOBackend::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(mtx_);
    // Return ESKF's live predicted state, not stale current_pose_
    // (current_pose_ is only updated during process()/ICP, missing IMU predict updates)
    if (eskf_) return eskf_->getPoseMatrix();
    return current_pose_;
}

// ============================================================================
// Visual Observation ESKF Update (reprojection residuals)
// ============================================================================

bool DV_LIOBackend::processVisual(
    const std::vector<DV_VIOManager::VisualLandmark>& landmarks,
    const DV_VIOManager::CameraIntrinsics& K)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (!eskf_) return false;

    // Filter valid landmarks with sufficient track length
    std::vector<int> valid_idx;
    for (int i = 0; i < static_cast<int>(landmarks.size()); i++) {
        if (landmarks[i].valid && landmarks[i].track_length >= 2) {
            valid_idx.push_back(i);
        }
    }
    if (static_cast<int>(valid_idx.size()) < 5) return false;

    auto obs_func = [&](const DV::SysState& state,
                        Eigen::MatrixXd& H,
                        Eigen::VectorXd& residual,
                        Eigen::MatrixXd& R_obs) -> bool
    {
        Eigen::Matrix3d R = state.R;
        Eigen::Vector3d p = state.p;
        Eigen::Matrix3d Rt = R.transpose();

        // Count features actually in front of camera
        struct VisObs {
            int idx;
            Eigen::Vector3d p_cam;
            double u_pred, v_pred;
        };
        std::vector<VisObs> obs_list;
        obs_list.reserve(valid_idx.size());

        for (int vi : valid_idx) {
            const auto& lm = landmarks[vi];
            Eigen::Vector3d p_cam = Rt * (lm.p_world - p);

            // Must be in front of camera (ARKit: Z < 0 for visible)
            if (p_cam.z() > -0.05) continue;

            double X = p_cam.x();
            double Y = p_cam.y();
            double Z = p_cam.z();

            // Project to pixel: u = -fx*X/Z + cx, v = fy*Y/Z + cy
            double u_pred = -static_cast<double>(K.fx) * X / Z + static_cast<double>(K.cx);
            double v_pred =  static_cast<double>(K.fy) * Y / Z + static_cast<double>(K.cy);

            // Reject if reprojection is way off (> 50 pixels)
            double err_u = lm.uv.x() - u_pred;
            double err_v = lm.uv.y() - v_pred;
            if (std::abs(err_u) > 50.0 || std::abs(err_v) > 50.0) continue;

            obs_list.push_back({vi, p_cam, u_pred, v_pred});
        }

        int n = static_cast<int>(obs_list.size());
        if (n < 5) {
            printf("[VIS] obs=%d (< 5 min) → SKIP\n", n);
            return false;
        }

        H = Eigen::MatrixXd::Zero(2 * n, 18);
        residual = Eigen::VectorXd::Zero(2 * n);
        R_obs = Eigen::MatrixXd::Zero(2 * n, 2 * n);

        for (int i = 0; i < n; i++) {
            const auto& ob = obs_list[i];
            const auto& lm = landmarks[ob.idx];
            double X = ob.p_cam.x();
            double Y = ob.p_cam.y();
            double Z = ob.p_cam.z();

            // Residual
            residual(2 * i)     = lm.uv.x() - ob.u_pred;
            residual(2 * i + 1) = lm.uv.y() - ob.v_pred;

            // Jacobian of projection w.r.t. camera-frame point p_c
            // u = -fx*X/Z + cx → du/dX = -fx/Z, du/dZ = fx*X/Z²
            // v = fy*Y/Z + cy  → dv/dY = fy/Z,  dv/dZ = -fy*Y/Z²
            double fx_d = static_cast<double>(K.fx);
            double fy_d = static_cast<double>(K.fy);
            Eigen::Matrix<double, 2, 3> J_proj;
            J_proj << -fx_d / Z,      0.0,       fx_d * X / (Z * Z),
                       0.0,           fy_d / Z,  -fy_d * Y / (Z * Z);

            // dp_c / d(delta_theta) = [p_c]×  (skew symmetric)
            Eigen::Matrix3d p_cam_hat = DV::SO3::hat(ob.p_cam);
            // dp_c / d(delta_p) = -R^T
            Eigen::Matrix3d neg_Rt = -Rt;

            // H_i (2×18) = J_proj * [p_cam_hat | -R^T | 0 | 0 | 0 | 0]
            H.block<2, 3>(2 * i, 0) = J_proj * p_cam_hat;  // rotation
            H.block<2, 3>(2 * i, 3) = J_proj * neg_Rt;     // position
            // columns 6-17 (v, bg, ba, g) are zero

            // Observation noise: 2 pixel std dev
            double sigma_px = 2.0;
            R_obs(2 * i, 2 * i)         = sigma_px * sigma_px;
            R_obs(2 * i + 1, 2 * i + 1) = sigma_px * sigma_px;
        }

        return true;
    };

    bool ok = eskf_->updateObserve(obs_func);
    if (ok) {
        current_pose_ = eskf_->getPoseMatrix();
    }
    return ok;
}
