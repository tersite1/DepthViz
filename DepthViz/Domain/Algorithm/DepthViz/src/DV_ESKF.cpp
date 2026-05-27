#include "../include/DV_ESKF.h"
#include <cmath>
#include <cstdio>

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

bool DV_ESKF::initFromIMU(const IMUData& imu) {
    if (initialized_) return true;

    init_samples_.push_back(imu);
    if (init_samples_.size() < static_cast<size_t>(kInitSampleCount)) {
        printf("[ESKF] collecting IMU samples: %zu/%d\n",
               init_samples_.size(), kInitSampleCount);
        return false;
    }

    // Compute mean acceleration (should be ~gravity if stationary)
    V3d mean_acc = V3d::Zero();
    for (const auto& s : init_samples_) {
        mean_acc += s.acc;
    }
    mean_acc /= static_cast<double>(init_samples_.size());

    double g_norm = mean_acc.norm();
    if (g_norm < 8.0 || g_norm > 12.0) {
        printf("[ESKF] WARNING: |g|=%.2f, expected ~9.81. Device not stationary?\n", g_norm);
        init_samples_.clear();
        return false;
    }

    // Find rotation aligning measured gravity to world gravity (0, -9.81, 0)
    // acc measures reaction force, so gravity_dir = -mean_acc.normalized()
    V3d g_measured = -mean_acc.normalized();
    V3d g_world = V3d(0, -1, 0);  // Y-down = gravity direction

    // Rotation from g_measured to g_world via axis-angle
    V3d axis = g_measured.cross(g_world);
    double sin_angle = axis.norm();
    double cos_angle = g_measured.dot(g_world);

    M3d R0;
    if (sin_angle < 1e-6) {
        R0 = (cos_angle > 0) ? M3d(M3d::Identity()) : M3d(-M3d::Identity());
    } else {
        axis.normalize();
        double angle = std::atan2(sin_angle, cos_angle);
        R0 = SO3::Exp(axis * angle).R;
    }

    SysState state;
    state.R = R0;
    state.p = V3d::Zero();
    state.v = V3d::Zero();
    state.g = V3d(0, -9.81, 0);
    state.timestamp = imu.timestamp;
    init(state);

    printf("[ESKF] Static init complete: |g|=%.3f, R0 angle=%.1f deg\n",
           g_norm, std::acos(std::max(-1.0, std::min(1.0, cos_angle))) * 180.0 / M_PI);

    init_samples_.clear();
    return true;
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

    // FIX #5: Velocity magnitude clamp — indoor handheld scanning cannot exceed 0.5 m/s.
    // 2.0 m/s was too permissive: IMU drift accumulates ~0.5 m/s² phantom accel
    // from gravity misalignment, reaching 2 m/s in 4 seconds before any correction.
    constexpr double kMaxVelocity = 0.5; // m/s
    double v_norm = state_.v.norm();
    if (v_norm > kMaxVelocity) {
        state_.v *= kMaxVelocity / v_norm;
    }

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

    // FIX #3 (W-1): Keep P_pred for all iterations.
    // Previously P_iter shrank every iteration, making later iterations
    // trust observations less than they should (gain K → 0).
    // Correct IEKF: always compute K from the prediction covariance P_pred.
    M18d P_pred = P_;  // Frozen prediction covariance

    SysState state_iter = state_;
    double last_dx_norm = 1e10;

    for (int iter = 0; iter < opts_.num_iterations; iter++) {
        Eigen::MatrixXd H;
        Eigen::VectorXd residual;
        Eigen::MatrixXd R_obs;

        if (!obs_func(state_iter, H, residual, R_obs)) {
            // Restore state on failure — partial corrections must not persist
            state_ = state_backup;
            P_ = P_backup;
            printf("[ESKF] obs_func FAIL at iter=%d → state restored\n", iter);
            return false;
        }

        int n = static_cast<int>(residual.rows());
        if (n == 0) {
            state_ = state_backup;
            P_ = P_backup;
            return false;
        }

        // K = P_pred * H^T * (H * P_pred * H^T + R)^{-1}
        // Use P_pred (not P_iter) — this is the key W-1 fix.
        Eigen::MatrixXd PHt = P_pred * H.transpose();       // 18xN
        Eigen::MatrixXd S = H * PHt + R_obs;                // NxN
        Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
        if (S_ldlt.info() != Eigen::Success) {
            state_ = state_backup;
            P_ = P_backup;
            printf("[ESKF] S matrix LDLT FAIL (singular) obs=%d → state restored\n", n);
            return false;
        }
        Eigen::MatrixXd K = PHt * S_ldlt.solve(Eigen::MatrixXd::Identity(n, n)); // 18xN

        // Error-state update
        V18d dx = K * residual;
        last_dx_norm = dx.norm();

        // Apply correction to nominal state
        applyCorrection(dx);
        state_iter = state_;

        // Check convergence
        if (last_dx_norm < opts_.quit_eps) {
            printf("[ESKF] converged iter=%d/%d dx=%.6f obs=%d\n",
                   iter + 1, opts_.num_iterations, last_dx_norm, n);
            break;
        }
        if (iter == opts_.num_iterations - 1) {
            printf("[ESKF] max_iter=%d dx=%.6f obs=%d (not converged)\n",
                   opts_.num_iterations, last_dx_norm, n);
        }
    }

    // FIX #3: Reject non-converged ICP — if dx is still large after max iterations,
    // the observation is unreliable (rank-deficient geometry, outlier-dominated).
    // Accepting a partial correction causes gravity misalignment → runaway drift.
    constexpr double kMaxAcceptableDx = 0.01;
    if (last_dx_norm > kMaxAcceptableDx) {
        state_ = state_backup;
        P_ = P_backup;
        printf("[ESKF] REJECTED: dx=%.6f > %.4f after max_iter → state restored\n",
               last_dx_norm, kMaxAcceptableDx);
        return false;
    }

    // Covariance update (Joseph form, computed once after convergence using P_pred)
    {
        Eigen::MatrixXd H;
        Eigen::VectorXd residual;
        Eigen::MatrixXd R_obs;
        // Re-evaluate at final state for covariance update
        if (obs_func(state_iter, H, residual, R_obs)) {
            int n = static_cast<int>(residual.rows());
            if (n > 0) {
                Eigen::MatrixXd PHt = P_pred * H.transpose();
                Eigen::MatrixXd S = H * PHt + R_obs;
                Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
                if (S_ldlt.info() == Eigen::Success) {
                    Eigen::MatrixXd K = PHt * S_ldlt.solve(Eigen::MatrixXd::Identity(n, n));
                    M18d I_KH = M18d::Identity() - K * H;
                    P_ = I_KH * P_pred * I_KH.transpose() + K * R_obs * K.transpose();
                    P_ = 0.5 * (P_ + P_.transpose());
                } else {
                    P_ = P_pred; // Fallback: keep prediction covariance
                }
            }
        }
    }

    // Log covariance health (position uncertainty)
    double pos_trace = P_.block<3, 3>(3, 3).trace();
    double vel_norm = state_.v.norm();
    if (pos_trace > 0.1 || vel_norm > 1.0) {
        printf("[ESKF] WARNING pos_cov_trace=%.4f vel=%.2fm/s\n", pos_trace, vel_norm);
    }

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

    // FIX #4: Correction magnitude clamping.
    // Prevents single bad ICP from causing catastrophic rotation/position jumps.
    // Limits: rotation <5° (~0.087 rad), position <10cm, velocity change <0.3 m/s
    constexpr double kMaxRotRad = 0.087;   // ~5 degrees
    constexpr double kMaxPosMeter = 0.10;  // 10cm
    constexpr double kMaxVelChange = 0.30; // 0.3 m/s

    double rot_norm = dtheta.norm();
    if (rot_norm > kMaxRotRad) {
        dtheta *= kMaxRotRad / rot_norm;
        printf("[ESKF] CLAMP rot: %.4f rad → %.4f rad\n", rot_norm, kMaxRotRad);
    }
    double pos_norm = dp.norm();
    if (pos_norm > kMaxPosMeter) {
        dp *= kMaxPosMeter / pos_norm;
        printf("[ESKF] CLAMP pos: %.4f m → %.4f m\n", pos_norm, kMaxPosMeter);
    }
    double vel_norm = dv.norm();
    if (vel_norm > kMaxVelChange) {
        dv *= kMaxVelChange / vel_norm;
        printf("[ESKF] CLAMP vel: %.4f m/s → %.4f m/s\n", vel_norm, kMaxVelChange);
    }

    // Apply rotation correction on the right: R <- R * Exp(dtheta)
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
