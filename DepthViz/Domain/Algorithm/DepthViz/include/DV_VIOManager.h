#ifndef DEPTHVIZ_VIO_MANAGER_H
#define DEPTHVIZ_VIO_MANAGER_H

// DV-SLAM Visual Frontend
// Shi-Tomasi corner detection + Pyramidal Lucas-Kanade optical flow
// Provides 3D-2D reprojection observations for ESKF update
// Self-contained: only Eigen + <cstdint>

#include "DV_Types.h"
#include <vector>
#include <mutex>
#include <cstdint>

class DV_VIOManager {
public:
    struct CameraIntrinsics {
        float fx = 0, fy = 0, cx = 0, cy = 0;
    };

    struct VisualLandmark {
        DV::V3d p_world;           // 3D position in world frame
        Eigen::Vector2f uv;        // Current 2D observation (working image coords)
        int track_length = 0;
        bool valid = false;
    };

    DV_VIOManager();
    ~DV_VIOManager();

    void init();

    // Set camera intrinsics (at ORIGINAL camera resolution)
    // Will be rescaled internally to working resolution
    void setIntrinsics(float fx, float fy, float cx, float cy,
                       int full_w, int full_h, int depth_w, int depth_h);

    // Process a new camera frame for visual tracking
    // gray: downscaled grayscale at kWorkWidth x kWorkHeight
    // depth_map: raw float depth at depth resolution
    // current_pose: ESKF-predicted pose (for initializing 3D landmarks)
    // Returns number of tracked features with valid 3D
    int processFrame(const uint8_t* gray, int work_w, int work_h,
                     const float* depth_map, int depth_w, int depth_h,
                     const Eigen::Matrix4d& current_pose);

    const std::vector<VisualLandmark>& getLandmarks() const { return landmarks_; }
    const CameraIntrinsics& getWorkingIntrinsics() const { return work_K_; }
    int numTracked() const;

    // Working image dimensions
    static constexpr int kWorkWidth = 480;
    static constexpr int kWorkHeight = 360;

private:
    // Image pyramid level
    struct PyramidLevel {
        std::vector<uint8_t> data;
        int width = 0, height = 0;
    };

    // Build image pyramid from level 0
    void buildPyramid(std::vector<PyramidLevel>& pyr,
                      const uint8_t* img, int w, int h);

    // Detect Shi-Tomasi corners with grid-based distribution
    std::vector<Eigen::Vector2f> detectFeatures(
        const std::vector<PyramidLevel>& pyr, int max_count,
        const std::vector<Eigen::Vector2f>& existing);

    // Pyramidal Lucas-Kanade tracking
    void trackLK(const std::vector<PyramidLevel>& prev,
                 const std::vector<PyramidLevel>& curr,
                 const std::vector<Eigen::Vector2f>& prev_pts,
                 std::vector<Eigen::Vector2f>& curr_pts,
                 std::vector<bool>& status);

    // Bilinear interpolation
    static float sampleBilinear(const uint8_t* img, int w, int h, float x, float y);

    // State
    std::vector<PyramidLevel> prev_pyr_;
    std::vector<Eigen::Vector2f> prev_pts_;
    std::vector<VisualLandmark> landmarks_;
    CameraIntrinsics work_K_;
    CameraIntrinsics depth_K_;
    float work_to_depth_x_ = 1.0f;
    float work_to_depth_y_ = 1.0f;
    bool has_prev_ = false;
    int frame_count_ = 0;

    // Config
    static constexpr int kPyramidLevels = 3;
    static constexpr int kWinHalf = 10;     // 21x21 window
    static constexpr int kMaxIter = 30;
    static constexpr float kMinEigen = 3.0f;
    static constexpr int kMaxFeatures = 200;
    static constexpr int kMinFeatures = 50;
    static constexpr int kGridCols = 10;
    static constexpr int kGridRows = 8;
    static constexpr float kFBThreshold = 1.5f; // Forward-backward check threshold (pixels)
};

#endif // DEPTHVIZ_VIO_MANAGER_H
