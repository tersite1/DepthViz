#pragma once
// ScanNet++ iPhone Data Loader
// Loads depth (16-bit PNG mm), IMU (CMDeviceMotion), poses, intrinsics
// from ScanNet++ scene directory structure.

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct IMUSample {
    double timestamp;
    Eigen::Vector3d acc;  // specific force (m/s²) = -(gravity + userAcceleration) * 9.81
    Eigen::Vector3d gyr;  // angular velocity (rad/s)
};

struct DepthFrame {
    double timestamp;
    std::string frame_name;
    std::vector<uint16_t> depth_mm;  // 256×192, uint16, millimeters
    int width = 256;
    int height = 192;
};

struct PoseEntry {
    double timestamp;
    std::string frame_name;
    Eigen::Matrix4d pose;          // raw ARKit camera-to-world
    Eigen::Matrix4d aligned_pose;  // aligned to mesh space (GT reference)
    Eigen::Matrix3d intrinsic;     // 3×3 RGB intrinsics (1920×1440)
};

struct ScanNetScene {
    std::string scene_id;
    std::string scene_path;

    // Per-frame data (sorted by timestamp)
    std::vector<PoseEntry> poses;
    std::vector<DepthFrame> depth_frames;
    std::vector<IMUSample> imu_samples;

    // Depth intrinsics (scaled from RGB)
    float fx_depth, fy_depth, cx_depth, cy_depth;

    // GT mesh path
    std::string mesh_path;

    bool valid() const { return !poses.empty() && !depth_frames.empty(); }
};

class ScanNetLoader {
public:
    // Load a ScanNet++ scene from directory
    // Expected: <scene_path>/iphone/pose_intrinsic_imu.json
    //           <scene_path>/iphone/depth/frame_XXXXXX.png
    //           <scene_path>/scans/mesh_aligned_0.05.ply
    bool load(const std::string& scene_path, ScanNetScene& scene);

private:
    bool loadPoseIntrinsicIMU(const std::string& json_path, ScanNetScene& scene);
    bool loadDepthFrames(const std::string& depth_dir, ScanNetScene& scene);
    bool loadDepthPNG(const std::string& path, DepthFrame& frame);
};
