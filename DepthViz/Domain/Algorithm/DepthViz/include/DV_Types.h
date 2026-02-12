#ifndef DV_TYPES_H
#define DV_TYPES_H

// DV-SLAM Self-Contained Type System
// Replaces PCL, OpenCV, ROS, and external SLAM type dependencies.
// Only requires: Eigen + <cstdint> + <cmath>

#include <cstdint>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace DV {

// ============================================================================
// Eigen Aliases (float scalar for mobile perf, double for covariance)
// ============================================================================

using V3f = Eigen::Vector3f;
using V3d = Eigen::Vector3d;
using M3f = Eigen::Matrix3f;
using M3d = Eigen::Matrix3d;
using M4f = Eigen::Matrix4f;
using M4d = Eigen::Matrix4d;

template<int N>
using VNd = Eigen::Matrix<double, N, 1>;
template<int R, int C>
using MRCd = Eigen::Matrix<double, R, C>;

using V6d = VNd<6>;
using V18d = VNd<18>;
using M6d = MRCd<6, 6>;
using M18d = MRCd<18, 18>;
using M18_6d = MRCd<18, 6>;

// ============================================================================
// DVPoint3D — Core point type for iPhone LiDAR
// ============================================================================

struct DVPoint3D {
    float x = 0.f, y = 0.f, z = 0.f;
    float intensity = 0.f;
    float confidence = 0.f; // iPhone LiDAR: 0 (low), 1 (medium), 2 (high)
    uint8_t r = 128, g = 128, b = 128; // RGB color from camera
    double timestamp = 0.0;
    int64_t voxel_idx = 0;

    V3f position() const { return V3f(x, y, z); }

    void setPosition(const V3f& p) {
        x = p.x(); y = p.y(); z = p.z();
    }

    void setColor(uint8_t red, uint8_t green, uint8_t blue) {
        r = red; g = green; b = blue;
    }

    // Spatial hash for 10cm voxels (configurable)
    static int64_t computeVoxelIdx(float px, float py, float pz, float voxel_size = 0.1f) {
        int64_t ix = static_cast<int64_t>(std::floor(px / voxel_size));
        int64_t iy = static_cast<int64_t>(std::floor(py / voxel_size));
        int64_t iz = static_cast<int64_t>(std::floor(pz / voxel_size));
        // Spatial hash: interleave with large primes to reduce collisions
        return ix + iy * 10000LL + iz * 100000000LL;
    }

    void updateVoxelIdx(float voxel_size = 0.1f) {
        voxel_idx = computeVoxelIdx(x, y, z, voxel_size);
    }
};

// ============================================================================
// IMUData — Replaces sensor_msgs::Imu
// ============================================================================

struct IMUData {
    double timestamp = 0.0;
    V3d acc = V3d::Zero(); // Accelerometer (m/s^2)
    V3d gyr = V3d::Zero(); // Gyroscope (rad/s)
};

// ============================================================================
// SO3 — Rotation manifold (wraps Eigen::Matrix3d)
// ============================================================================

struct SO3 {
    M3d R = M3d::Identity();

    SO3() = default;
    explicit SO3(const M3d& rotation) : R(rotation) {}

    // Exponential map: so(3) -> SO(3) via Rodrigues
    static SO3 Exp(const V3d& omega) {
        double theta = omega.norm();
        if (theta < 1e-10) {
            return SO3(M3d::Identity() + hat(omega));
        }
        V3d axis = omega / theta;
        M3d K = hat(axis);
        M3d Rot = M3d::Identity() + std::sin(theta) * K + (1.0 - std::cos(theta)) * K * K;
        return SO3(Rot);
    }

    // Logarithmic map: SO(3) -> so(3)
    V3d Log() const {
        double cos_angle = (R.trace() - 1.0) * 0.5;
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        double theta = std::acos(cos_angle);

        // Small angle: first-order vee extraction
        if (theta < 1e-10) {
            return V3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) * 0.5;
        }

        // Near-pi: sin(theta)→0 causes division instability.
        // Use eigendecomposition of the symmetric part to extract the rotation axis.
        // At theta=pi: R = 2*n*n^T - I, so (R + I)/2 has rank-1 with eigenvector = axis.
        if (theta > M_PI - 1e-6) {
            M3d S = (R + M3d::Identity()) * 0.5;
            // Find the column with largest norm as the axis estimate
            V3d c0 = S.col(0), c1 = S.col(1), c2 = S.col(2);
            double n0 = c0.squaredNorm(), n1 = c1.squaredNorm(), n2 = c2.squaredNorm();
            V3d axis;
            if (n0 >= n1 && n0 >= n2) {
                axis = c0.normalized();
            } else if (n1 >= n0 && n1 >= n2) {
                axis = c1.normalized();
            } else {
                axis = c2.normalized();
            }
            // Resolve sign ambiguity using the skew-symmetric part
            V3d skew(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
            if (skew.dot(axis) < 0) axis = -axis;
            return axis * theta;
        }

        // General case: standard formula
        M3d lnR = (theta / (2.0 * std::sin(theta))) * (R - R.transpose());
        return V3d(lnR(2, 1), lnR(0, 2), lnR(1, 0));
    }

    // Hat operator: R^3 -> so(3) (skew-symmetric)
    static M3d hat(const V3d& v) {
        M3d m;
        m << 0, -v.z(), v.y(),
             v.z(), 0, -v.x(),
             -v.y(), v.x(), 0;
        return m;
    }

    SO3 operator*(const SO3& other) const {
        return SO3(R * other.R);
    }

    SO3 inverse() const {
        return SO3(R.transpose());
    }

    V3d operator*(const V3d& v) const {
        return R * v;
    }
};

// ============================================================================
// SE3 — Rigid body transformation (rotation + translation)
// ============================================================================

struct SE3 {
    SO3 rot;
    V3d trans = V3d::Zero();

    SE3() = default;
    SE3(const SO3& r, const V3d& t) : rot(r), trans(t) {}
    explicit SE3(const M4d& T) {
        rot = SO3(T.block<3, 3>(0, 0));
        trans = T.block<3, 1>(0, 3);
    }

    M4d matrix() const {
        M4d T = M4d::Identity();
        T.block<3, 3>(0, 0) = rot.R;
        T.block<3, 1>(0, 3) = trans;
        return T;
    }

    // Exponential map: se(3) -> SE(3)
    // xi = [rho; omega] where rho is translation part, omega is rotation part
    static SE3 Exp(const V6d& xi) {
        V3d rho = xi.head<3>();
        V3d omega = xi.tail<3>();
        SO3 R_new = SO3::Exp(omega);

        double theta = omega.norm();
        M3d J;
        if (theta < 1e-10) {
            J = M3d::Identity();
        } else {
            V3d axis = omega / theta;
            M3d K = SO3::hat(axis);
            J = M3d::Identity()
                + ((1.0 - std::cos(theta)) / theta) * K
                + ((theta - std::sin(theta)) / theta) * K * K;
        }
        return SE3(R_new, J * rho);
    }

    V6d Log() const {
        V3d omega = rot.Log();
        double theta = omega.norm();

        M3d J_inv;
        if (theta < 1e-10) {
            J_inv = M3d::Identity();
        } else {
            // Use full [omega]_x (not unit-axis K) for correct coefficient scaling.
            // J_inv = I - 0.5*[w]_x + (1/θ² - (1+cosθ)/(2θ sinθ)) * [w]_x²
            M3d W = SO3::hat(omega); // [omega]_x = theta * K
            double sin_theta = std::sin(theta);
            double beta;
            if (std::abs(sin_theta) < 1e-10) {
                // theta near pi: (1+cosθ)/(2θ sinθ) → 0 as sinθ→0 and 1+cosθ→0 (faster)
                // So beta = 1/θ² - 0 = 1/θ²
                beta = 1.0 / (theta * theta);
            } else {
                beta = 1.0 / (theta * theta) - (1.0 + std::cos(theta)) / (2.0 * theta * sin_theta);
            }
            J_inv = M3d::Identity() - 0.5 * W + beta * W * W;
        }

        V6d xi;
        xi.head<3>() = J_inv * trans;
        xi.tail<3>() = omega;
        return xi;
    }

    SE3 operator*(const SE3& other) const {
        return SE3(rot * other.rot, rot * other.trans + trans);
    }

    SE3 inverse() const {
        SO3 R_inv = rot.inverse();
        return SE3(R_inv, R_inv * (-trans));
    }

    V3d operator*(const V3d& p) const {
        return rot * p + trans;
    }
};

// ============================================================================
// SysState — Full filter state (18D)
// ============================================================================

struct SysState {
    double timestamp = 0.0;
    M3d R = M3d::Identity();  // Rotation
    V3d p = V3d::Zero();      // Position
    V3d v = V3d::Zero();      // Velocity
    V3d bg = V3d::Zero();     // Gyro bias
    V3d ba = V3d::Zero();     // Accel bias
    V3d g = V3d(0, 0, -9.81); // Gravity in world frame

    M4d poseMatrix() const {
        M4d T = M4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = p;
        return T;
    }
};

// ============================================================================
// Configuration Structs
// ============================================================================

struct BundleDiscardConfig {
    float voxel_size = 0.1f;        // 10cm voxels
    int min_density = 5;            // Min points per voxel to keep
    float min_avg_confidence = 1.5f; // Min avg confidence: requires majority high-conf (≥2) points
};

struct KeyframeConfig {
    float translation_threshold = 0.05f;    // 5cm
    float rotation_threshold_deg = 2.0f;    // 2 degrees
    float rotation_threshold_rad() const {
        return rotation_threshold_deg * static_cast<float>(M_PI) / 180.0f;
    }
};

struct ESKFOptions {
    int num_iterations = 3;
    double quit_eps = 1e-6;
    // IMU noise parameters
    double gyro_noise = 0.01;         // rad/s/sqrt(Hz)
    double accel_noise = 0.1;         // m/s^2/sqrt(Hz)
    double gyro_bias_noise = 0.001;   // rad/s^2/sqrt(Hz)
    double accel_bias_noise = 0.01;   // m/s^3/sqrt(Hz)
    // Observation noise
    double lidar_noise = 0.01;        // meters
};

} // namespace DV

#endif // DV_TYPES_H
