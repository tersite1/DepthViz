#include "../include/DV_ESKF.h"
#include <cmath>

namespace DV {

DV_ESKF::DV_ESKF() : DV_ESKF(ESKFOptions{}) {}

DV_ESKF::DV_ESKF(const ESKFOptions& options) : opts_(options) {
    P_ = M18d::Identity() * 1e-4;
    // Large initial uncertainty for gravity direction
    P_.block<3, 3>(15, 15) = M3d::Identity() * 1.0;
}

void DV_ESKF::init(const SysState& state) {
    state_ = state;
    initialized_ = true;
}

void DV_ESKF::predict(const IMUData& imu, double dt) {
    if (!initialized_ || dt <= 0.0) return;

    const V3d& acc = imu.acc;
    const V3d& gyr = imu.gyr;

    // Remove biases
    V3d gyr_corrected = gyr - state_.bg;
    V3d acc_corrected = acc - state_.ba;

    // Midpoint integration for rotation
    M3d R_prev = state_.R;
    V3d omega_dt = gyr_corrected * dt;
    SO3 dR = SO3::Exp(omega_dt);
    state_.R = R_prev * dR.R;

    // Midpoint rotation for acceleration
    M3d R_mid = R_prev * SO3::Exp(omega_dt * 0.5).R;
    V3d acc_world = R_mid * acc_corrected + state_.g;

    // Position and velocity update (midpoint)
    state_.p += state_.v * dt + 0.5 * acc_world * dt * dt;
    state_.v += acc_world * dt;

    // Error-state transition matrix F (18x18)
    M18d F = M18d::Identity();

    // dR/dtheta: -R [acc_corrected]_x dt
    F.block<3, 3>(0, 0) = dR.R.transpose(); // rotation error propagation
    F.block<3, 3>(0, 9) = -M3d::Identity() * dt; // d(theta)/d(bg)

    // dp/dtheta: -R [acc_corrected]_x * dt^2 / 2
    M3d acc_skew = SO3::hat(acc_corrected);
    F.block<3, 3>(3, 0) = -R_prev * acc_skew * dt * dt * 0.5;
    F.block<3, 3>(3, 6) = M3d::Identity() * dt; // dp/dv
    F.block<3, 3>(3, 12) = -R_prev * dt * dt * 0.5; // dp/d(ba)
    F.block<3, 3>(3, 15) = M3d::Identity() * dt * dt * 0.5; // dp/dg

    // dv/dtheta: -R [acc_corrected]_x * dt
    F.block<3, 3>(6, 0) = -R_prev * acc_skew * dt;
    F.block<3, 3>(6, 12) = -R_prev * dt; // dv/d(ba)
    F.block<3, 3>(6, 15) = M3d::Identity() * dt; // dv/dg

    // Covariance propagation: P = F * P * F^T + Q
    M18d Q = buildProcessNoise(dt);
    P_ = F * P_ * F.transpose() + Q;

    // Ensure symmetry
    P_ = 0.5 * (P_ + P_.transpose());

    state_.timestamp = imu.timestamp;
}

bool DV_ESKF::updateObserve(const ObsFunc& obs_func) {
    if (!initialized_) return false;

    // Save state before iteration so we can restore on failure
    SysState state_backup = state_;
    M18d P_backup = P_;

    SysState state_iter = state_;
    M18d P_iter = P_;

    for (int iter = 0; iter < opts_.num_iterations; iter++) {
        Eigen::MatrixXd H;
        Eigen::VectorXd residual;
        Eigen::MatrixXd R_obs;

        if (!obs_func(state_iter, H, residual, R_obs)) {
            // Restore state on failure — partial corrections must not persist
            state_ = state_backup;
            P_ = P_backup;
            return false;
        }

        int n = static_cast<int>(residual.rows());
        if (n == 0) {
            state_ = state_backup;
            P_ = P_backup;
            return false;
        }

        // K = P * H^T * (H * P * H^T + R)^{-1}
        // Use LDLT decomposition for numerical stability instead of direct inverse
        Eigen::MatrixXd PHt = P_iter * H.transpose();       // 18xN
        Eigen::MatrixXd S = H * PHt + R_obs;                // NxN
        Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
        if (S_ldlt.info() != Eigen::Success) {
            state_ = state_backup;
            P_ = P_backup;
            return false;
        }
        Eigen::MatrixXd K = PHt * S_ldlt.solve(Eigen::MatrixXd::Identity(n, n)); // 18xN

        // Error-state update
        V18d dx = K * residual;

        // Apply correction to nominal state
        applyCorrection(dx);
        state_iter = state_;

        // Covariance update (Joseph form for numerical stability)
        M18d I_KH = M18d::Identity() - K * H;
        P_iter = I_KH * P_iter * I_KH.transpose() + K * R_obs * K.transpose();
        P_iter = 0.5 * (P_iter + P_iter.transpose());

        // Check convergence
        if (dx.norm() < opts_.quit_eps) {
            break;
        }
    }

    P_ = P_iter;
    return true;
}

void DV_ESKF::applyCorrection(const V18d& delta_x) {
    // Error-state ordering: [dtheta(3), dp(3), dv(3), dbg(3), dba(3), dg(3)]
    V3d dtheta = delta_x.segment<3>(0);
    V3d dp = delta_x.segment<3>(3);
    V3d dv = delta_x.segment<3>(6);
    V3d dbg = delta_x.segment<3>(9);
    V3d dba = delta_x.segment<3>(12);
    V3d dg = delta_x.segment<3>(15);

    // Apply rotation correction on the left: R <- R * Exp(dtheta)
    state_.R = state_.R * SO3::Exp(dtheta).R;
    state_.p += dp;
    state_.v += dv;
    state_.bg += dbg;
    state_.ba += dba;
    state_.g += dg;
}

M18d DV_ESKF::buildProcessNoise(double dt) const {
    M18d Q = M18d::Zero();

    double dt2 = dt * dt;
    double gyro_var = opts_.gyro_noise * opts_.gyro_noise * dt;
    double accel_var = opts_.accel_noise * opts_.accel_noise * dt;
    double gyro_bias_var = opts_.gyro_bias_noise * opts_.gyro_bias_noise * dt;
    double accel_bias_var = opts_.accel_bias_noise * opts_.accel_bias_noise * dt;

    // Rotation noise (from gyro)
    Q.block<3, 3>(0, 0) = M3d::Identity() * gyro_var;
    // Position noise (from accel integration)
    Q.block<3, 3>(3, 3) = M3d::Identity() * accel_var * dt2 * 0.25;
    // Velocity noise (from accel)
    Q.block<3, 3>(6, 6) = M3d::Identity() * accel_var;
    // Gyro bias random walk
    Q.block<3, 3>(9, 9) = M3d::Identity() * gyro_bias_var;
    // Accel bias random walk
    Q.block<3, 3>(12, 12) = M3d::Identity() * accel_bias_var;
    // Gravity (should not change, very small noise)
    Q.block<3, 3>(15, 15) = M3d::Identity() * 1e-10;

    return Q;
}

} // namespace DV
