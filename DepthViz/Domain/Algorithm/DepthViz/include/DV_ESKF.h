#ifndef DV_ESKF_H
#define DV_ESKF_H

// DV-SLAM Error-State Kalman Filter
// Replaces LI2Sup::ESKF (which depends on alias.h -> PCL, Manifold.h, params.h).
// 18D state: [R(SO3), p(3), v(3), bg(3), ba(3), g(3)]
// Uses double for covariance (numerical stability), float for points.

#include "DV_Types.h"
#include <functional>

namespace DV {

class DV_ESKF {
public:
    // Observation function signature:
    // Given current state, compute measurement residual and Jacobian.
    // Returns: bool (true if observation is valid)
    // Outputs: residual (Nx1), H_x (Nx18 Jacobian)
    using ObsFunc = std::function<bool(
        const SysState& state,
        Eigen::MatrixXd& H,        // Jacobian (Nx18)
        Eigen::VectorXd& residual,  // Residual (Nx1)
        Eigen::MatrixXd& R_obs      // Observation noise (NxN)
    )>;

    DV_ESKF();
    explicit DV_ESKF(const ESKFOptions& options);
    ~DV_ESKF() = default;

    // Initialize state
    void init(const SysState& state);

    // IMU prediction step (midpoint integration)
    void predict(const IMUData& imu, double dt);

    // Iterated Kalman update with observation function
    // Uses Sherman-Morrison-Woodbury identity for efficiency
    bool updateObserve(const ObsFunc& obs_func);

    // Accessors
    const SysState& getState() const { return state_; }
    M4d getPoseMatrix() const { return state_.poseMatrix(); }
    M3d getRotation() const { return state_.R; }
    V3d getPosition() const { return state_.p; }
    V3d getVelocity() const { return state_.v; }
    V3d getGyroBias() const { return state_.bg; }
    V3d getAccelBias() const { return state_.ba; }
    V3d getGravity() const { return state_.g; }
    const M18d& getCovariance() const { return P_; }

    // Set state directly (e.g. from ARKit prior)
    void setState(const SysState& state) { state_ = state; }
    void setPose(const M3d& R, const V3d& p) { state_.R = R; state_.p = p; }

private:
    // Apply error-state correction to nominal state
    void applyCorrection(const V18d& delta_x);

    // Build IMU noise covariance matrix Q
    M18d buildProcessNoise(double dt) const;

    SysState state_;
    M18d P_;         // 18x18 error-state covariance
    ESKFOptions opts_;
    bool initialized_ = false;
};

} // namespace DV

#endif // DV_ESKF_H
