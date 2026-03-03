#include "../include/DV_LIOBackend.h"
#include "../include/DV_RobustKernels.h"

#include <cmath>
#include <cstdio>

DV_LIOBackend::DV_LIOBackend() = default;
DV_LIOBackend::~DV_LIOBackend() = default;

void DV_LIOBackend::init() {
    map_ = std::make_unique<DV::DV_VoxelHashMap>(0.05f, 500000);
    eskf_ = std::make_unique<DV::DV_ESKF>(eskf_opts_);

    DV::SysState initial_state;
    eskf_->init(initial_state);

    current_pose_ = Eigen::Matrix4d::Identity();
    first_frame_ = true;
}

void DV_LIOBackend::processIMU(const std::vector<DV::IMUData>& imu_data) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!eskf_ || first_frame_) return;

    for (const auto& imu : imu_data) {
        double dt = 0.0;
        if (last_imu_timestamp_ > 0.0) {
            dt = imu.timestamp - last_imu_timestamp_;
        }
        last_imu_timestamp_ = imu.timestamp;

        if (dt > 0.0 && dt < 0.5) { // Sanity: skip if dt > 500ms (stale data)
            eskf_->predict(imu, dt);
        }
    }
}

Eigen::Matrix4d DV_LIOBackend::process(
    const Eigen::Matrix4d& prior_pose,
    const std::vector<DV::DVPoint3D>& points)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (points.empty()) return prior_pose;

    // First frame: just insert points and return prior
    if (first_frame_) {
        insertPoints(prior_pose, points);
        current_pose_ = prior_pose;

        // Initialize ESKF state from prior
        DV::SysState state;
        state.R = prior_pose.block<3, 3>(0, 0);
        state.p = prior_pose.block<3, 1>(0, 3);
        eskf_->init(state);

        first_frame_ = false;
        return prior_pose;
    }

    // Use IMU-predicted state if available; fall back to ARKit prior if no IMU data
    DV::SysState current_eskf_state = eskf_->getState();
    Eigen::Vector3d eskf_pos = current_eskf_state.p;
    Eigen::Vector3d prior_pos = prior_pose.block<3, 1>(0, 3);
    Eigen::Matrix3d prior_R = prior_pose.block<3, 3>(0, 0);

    // Check both position AND rotation divergence
    double pos_diff = (eskf_pos - prior_pos).norm();
    Eigen::Matrix3d dR = current_eskf_state.R.transpose() * prior_R;
    double cos_angle = std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) * 0.5));
    double rot_diff_deg = std::acos(cos_angle) * 180.0 / M_PI;

    // Always anchor ESKF to ARKit pose before ICP.
    // This prevents drift accumulation: each frame's ICP starts fresh from ARKit,
    // so corrections are always small and independent (not compounding).
    // Velocity estimated from ARKit pose delta; biases preserved for calibration.
    {
        // Estimate velocity from ARKit pose difference
        Eigen::Vector3d est_v = Eigen::Vector3d::Zero();
        double dt_pose = current_eskf_state.timestamp > 0.0 ?
            (prior_pose(3, 3) > 0.0 ? 0.0 : 0.0) : 0.0; // timestamp not in pose
        if (last_arkit_timestamp_ > 0.0) {
            double dt_ark = current_eskf_state.timestamp - last_arkit_timestamp_;
            if (dt_ark > 0.001 && dt_ark < 1.0) {
                est_v = (prior_pos - last_arkit_pos_) / dt_ark;
                // Sanity: clamp to 5 m/s (walking speed)
                double v_norm = est_v.norm();
                if (v_norm > 5.0) {
                    est_v = est_v * (5.0 / v_norm);
                }
            }
        }
        last_arkit_pos_ = prior_pos;
        last_arkit_timestamp_ = current_eskf_state.timestamp;

        DV::SysState anchored;
        anchored.R = prior_R;
        anchored.p = prior_pos;
        anchored.v = est_v;                   // Velocity from ARKit, not IMU
        anchored.bg = current_eskf_state.bg;  // Keep bias calibration
        anchored.ba = current_eskf_state.ba;
        anchored.g = current_eskf_state.g;
        anchored.timestamp = current_eskf_state.timestamp;
        eskf_->setState(anchored);

        if (pos_diff > 0.1 || rot_diff_deg > 10.0) {
            printf("[LIO] ANCHOR drift=%.3fm rot=%.1f° vel=%.2fm/s → reset to ARKit\n",
                   pos_diff, rot_diff_deg, current_eskf_state.v.norm());
        }
    }

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

            Eigen::Vector3f diff = p_world_f - centroid_f;
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

        int n = static_cast<int>(observations.size());
        if (n < 10) {
            printf("[LIO] obs=%d (< 10 min) from %zu pts → SKIP\n", n, pts.size());
            return false;
        }

        H = Eigen::MatrixXd::Zero(n, 18);
        residual = Eigen::VectorXd::Zero(n);
        R_obs = Eigen::MatrixXd::Zero(n, n);

        for (int i = 0; i < n; i++) {
            const auto& obs = observations[i];

            Eigen::Matrix3d p_camera_hat = DV::SO3::hat(obs.p_camera_d);
            H.block<1, 3>(i, 0) = -obs.normal_d.transpose() * R * p_camera_hat;
            H.block<1, 3>(i, 3) = obs.normal_d.transpose();
            residual(i) = -obs.residual_val;

            double sigma2 = 0.01 * 0.01;
            R_obs(i, i) = sigma2 / std::max(obs.weight, 0.01);
        }

        return true;
    };

    // Run iterated Kalman update
    bool success = eskf_->updateObserve(obs_func);

    // Compute pose delta (SLAM refinement amount)
    Eigen::Vector3d prior_t = prior_pose.block<3, 1>(0, 3);

    if (success) {
        current_pose_ = eskf_->getPoseMatrix();
        Eigen::Vector3d refined_t = current_pose_.block<3, 1>(0, 3);
        double refine_dist = (refined_t - prior_t).norm();
        int map_size = map_ ? static_cast<int>(map_->size()) : 0;
        printf("[LIO] OK pts=%zu refine=%.4fm map=%d\n",
               points.size(), refine_dist, map_size);
    } else {
        current_pose_ = prior_pose;
        printf("[LIO] FAIL pts=%zu → fallback to ARKit prior\n", points.size());
    }

    // Insert with refined pose — safe because ESKF is anchored to ARKit each frame,
    // so corrections are small and independent (no drift accumulation).
    insertPoints(current_pose_, points);

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
    return current_pose_;
}
