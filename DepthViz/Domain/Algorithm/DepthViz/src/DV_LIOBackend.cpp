#include "../include/DV_LIOBackend.h"
#include "../include/DV_RobustKernels.h"

#include <cmath>

DV_LIOBackend::DV_LIOBackend() = default;
DV_LIOBackend::~DV_LIOBackend() = default;

void DV_LIOBackend::init() {
    map_ = std::make_unique<DV::DV_VoxelHashMap>(0.1f, 500000);
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

    // If the ESKF position hasn't moved from the last frame (no IMU predictions),
    // use the ARKit prior as the starting point instead
    double pos_diff = (eskf_pos - prior_pos).norm();
    if (pos_diff > 1.0) {
        // Large disagreement: ESKF has diverged or no IMU data — reset to ARKit prior
        DV::SysState reset_state = current_eskf_state;
        reset_state.R = prior_pose.block<3, 3>(0, 0);
        reset_state.p = prior_pos;
        // Preserve velocity, biases, and gravity from ESKF
        eskf_->setState(reset_state);
    }
    // Otherwise: trust the IMU-propagated ESKF state (preserves R, p, v from IMU integration)

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

        // Accumulate valid observations
        struct Observation {
            Eigen::Vector3d normal_d;
            Eigen::Vector3d p_camera_d; // Camera-frame point (needed for rotation Jacobian)
            double residual_val;
            double weight;
        };
        std::vector<Observation> observations;
        observations.reserve(pts.size());

        for (const auto& pt : pts) {
            // Transform point to world frame
            // NOTE: Points are in the camera frame (from ARKit sceneDepth unprojection).
            // We assume T_camera_imu ≈ I (camera-IMU co-location approximation).
            // On iPhone, the physical offset is ~5-10mm, introducing a bounded lever-arm
            // error during rotation. The divergence guard (1m) masks this. For cm-level
            // accuracy claims in publications, this approximation must be stated explicitly.
            Eigen::Vector3d p_camera(pt.x, pt.y, pt.z);
            Eigen::Vector3d p_world = R * p_camera + t;
            Eigen::Vector3f p_world_f = p_world.cast<float>();

            // Query nearest neighbors
            DV::KNNResult knn;
            if (!map_ptr->getTopK(p_world_f, knn, 5)) continue;

            // Fit plane
            Eigen::Vector3f normal_f, centroid_f;
            if (!DV::DV_VoxelHashMap::fitPlane(knn, normal_f, centroid_f)) continue;

            // Point-to-plane residual: r = n^T (p_world - centroid)
            Eigen::Vector3f diff = p_world_f - centroid_f;
            float r = normal_f.dot(diff);

            // Confidence weight (ablation: can disable)
            float conf_w = use_conf_w ? DepthViz::getConfidenceWeight(pt.confidence) : 1.0f;
            if (conf_w <= 0.f) continue;

            // TLS weight (ablation: can disable)
            float tls_w = use_tls ? DepthViz::computeTLSWeight(r, 0.10f) : 1.0f;
            if (tls_w <= 0.f) continue;

            float total_weight = conf_w * tls_w;

            Observation obs;
            obs.normal_d = normal_f.cast<double>();
            obs.p_camera_d = p_camera;
            obs.residual_val = static_cast<double>(r); // Unweighted raw residual
            obs.weight = static_cast<double>(total_weight);
            observations.push_back(obs);
        }

        int n = static_cast<int>(observations.size());
        if (n < 10) return false; // Not enough valid observations

        // Build H (Nx18) and residual (Nx1)
        H = Eigen::MatrixXd::Zero(n, 18);
        residual = Eigen::VectorXd::Zero(n);
        R_obs = Eigen::MatrixXd::Zero(n, n);

        for (int i = 0; i < n; i++) {
            const auto& obs = observations[i];

            // Jacobian of point-to-plane residual w.r.t. error state:
            // r = n^T (R * p_camera + t - q)
            // dr/dtheta = -n^T * R * [p_camera]_x  (rotation perturbation)
            // dr/dp     = n^T                        (translation perturbation)
            // Other states: zero (not directly observable from LiDAR)
            // NOTE: p_camera used as p_body under T_camera_imu ≈ I assumption.

            // Rotation Jacobian: H_theta = -n^T * R * hat(p_camera)
            Eigen::Matrix3d p_camera_hat = DV::SO3::hat(obs.p_camera_d);
            H.block<1, 3>(i, 0) = -obs.normal_d.transpose() * R * p_camera_hat;

            // Translation Jacobian: H_p = n^T
            H.block<1, 3>(i, 3) = obs.normal_d.transpose();

            residual(i) = -obs.residual_val; // Negative because update = K * (-r)

            // Observation noise: sigma² / weight (higher weight = lower noise = more trusted)
            double sigma2 = 0.01 * 0.01; // Base observation noise variance (1cm)
            R_obs(i, i) = sigma2 / std::max(obs.weight, 0.01);
        }

        return true;
    };

    // Run iterated Kalman update
    bool success = eskf_->updateObserve(obs_func);

    if (success) {
        current_pose_ = eskf_->getPoseMatrix();
    } else {
        // Fall back to prior
        current_pose_ = prior_pose;
    }

    // Insert surviving points into map
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
