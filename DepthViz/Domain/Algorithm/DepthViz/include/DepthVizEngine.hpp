#ifndef DEPTHVIZ_ENGINE_HPP
#define DEPTHVIZ_ENGINE_HPP

// DepthViz: iOS-Optimized DV-SLAM Engine
// "Quantize, Gate, then Optimize" — designed for iPhone LiDAR's noisy dToF sensor.
// Self-contained: only Eigen + Apple Accelerate + GCD.
// ARKit VIO as pose prior, refined by LIO with Bundle & Discard preprocessing.

#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <cstring>

#include "DV_Types.h"
#include "DV_VoxelHashMap.h"
#include "DV_ESKF.h"

class DV_LIOBackend;
class DV_VIOManager;

class DepthVizEngine {
public:
    DepthVizEngine();
    ~DepthVizEngine();

    void init();
    void start();
    void stop();

    // Input Interfaces (Zero-copy: raw pointers from iOS)
    void pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr);
    void pushPointCloud(double timestamp, const float* xyz, const uint8_t* conf, const uint8_t* rgb, int count);
    void pushImage(double timestamp, const void* imageData, int width, int height);
    void pushARKitPose(double timestamp, const Eigen::Matrix4d& pose);

    // Output Interfaces
    Eigen::Matrix4d getPose();
    std::vector<DV::DVPoint3D> getDisplayCloud();
    std::vector<DV::DVPoint3D> getFullMap();

private:
    void run();

    // Bundle & Discard: core preprocessing
    // 1. Gate on confidence (skip conf=0)
    // 2. Hash into 10cm voxels
    // 3. Discard sparse/low-confidence voxels
    // 4. Emit one centroid per surviving voxel with averaged color
    std::vector<DV::DVPoint3D> bundleAndDiscard(const float* xyz, const uint8_t* conf, const uint8_t* rgb, int count);

    // Keyframe check: trigger if translation >= 5cm OR rotation >= 2 deg
    bool isKeyframe(const Eigen::Matrix4d& current_pose);

    // Configuration
    DV::BundleDiscardConfig bundle_config_;
    DV::KeyframeConfig keyframe_config_;

    // Backend components
    std::shared_ptr<DV_LIOBackend> lio_;
    std::shared_ptr<DV_VIOManager> vio_;

    // State
    Eigen::Matrix4d last_optimized_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d last_keyframe_pose_ = Eigen::Matrix4d::Identity();
    std::atomic<bool> first_frame_{true};

    // Threading
    std::atomic<bool> is_running_{false};
    std::unique_ptr<std::thread> thread_;
    mutable std::mutex mtx_state_;
    std::mutex mtx_data_;

    // Buffers (ring buffer for point clouds, heap-allocated to avoid 13MB on stack)
    static constexpr int RING_BUFFER_CAPACITY = 100;
    static constexpr int MAX_POINTS_PER_FRAME = 10000;

    struct PointCloudBuffer {
        // Heap-allocated data arrays
        std::unique_ptr<float[]> xyz_data;    // [CAPACITY * MAX_POINTS * 3]
        std::unique_ptr<uint8_t[]> conf_data; // [CAPACITY * MAX_POINTS]
        std::unique_ptr<uint8_t[]> rgb_data;  // [CAPACITY * MAX_POINTS * 3] (R, G, B)
        int point_counts[RING_BUFFER_CAPACITY] = {};
        double timestamps[RING_BUFFER_CAPACITY] = {};
        int write_idx = 0;
        int read_idx = 0;
        std::atomic<int> count{0};

        PointCloudBuffer()
            : xyz_data(new float[RING_BUFFER_CAPACITY * MAX_POINTS_PER_FRAME * 3]())
            , conf_data(new uint8_t[RING_BUFFER_CAPACITY * MAX_POINTS_PER_FRAME]())
            , rgb_data(new uint8_t[RING_BUFFER_CAPACITY * MAX_POINTS_PER_FRAME * 3]()) {}

        float* xyzSlot(int idx) { return &xyz_data[idx * MAX_POINTS_PER_FRAME * 3]; }
        uint8_t* confSlot(int idx) { return &conf_data[idx * MAX_POINTS_PER_FRAME]; }
        uint8_t* rgbSlot(int idx) { return &rgb_data[idx * MAX_POINTS_PER_FRAME * 3]; }
        const float* xyzSlot(int idx) const { return &xyz_data[idx * MAX_POINTS_PER_FRAME * 3]; }
        const uint8_t* confSlot(int idx) const { return &conf_data[idx * MAX_POINTS_PER_FRAME]; }
        const uint8_t* rgbSlot(int idx) const { return &rgb_data[idx * MAX_POINTS_PER_FRAME * 3]; }

        void push(double timestamp, const float* xyz, const uint8_t* conf, const uint8_t* rgb, int n_points) {
            int idx = write_idx;
            if (n_points > MAX_POINTS_PER_FRAME) n_points = MAX_POINTS_PER_FRAME;
            std::memcpy(xyzSlot(idx), xyz, n_points * 3 * sizeof(float));
            std::memcpy(confSlot(idx), conf, n_points * sizeof(uint8_t));
            if (rgb) {
                std::memcpy(rgbSlot(idx), rgb, n_points * 3 * sizeof(uint8_t));
            } else {
                std::memset(rgbSlot(idx), 128, n_points * 3 * sizeof(uint8_t)); // Default gray
            }
            point_counts[idx] = n_points;
            timestamps[idx] = timestamp;

            write_idx = (write_idx + 1) % RING_BUFFER_CAPACITY;
            int old_count = count.load(std::memory_order_relaxed);
            if (old_count < RING_BUFFER_CAPACITY) {
                count.fetch_add(1, std::memory_order_release);
            } else {
                read_idx = (read_idx + 1) % RING_BUFFER_CAPACITY;
            }
        }

        // Copy-out pop: copies data into caller-provided buffers (safe after lock release)
        bool pop(double& timestamp, float* xyz_out, uint8_t* conf_out, uint8_t* rgb_out, int& n_points) {
            if (count.load(std::memory_order_acquire) == 0) return false;
            int idx = read_idx;
            timestamp = timestamps[idx];
            n_points = point_counts[idx];
            std::memcpy(xyz_out, xyzSlot(idx), n_points * 3 * sizeof(float));
            std::memcpy(conf_out, confSlot(idx), n_points * sizeof(uint8_t));
            std::memcpy(rgb_out, rgbSlot(idx), n_points * 3 * sizeof(uint8_t));
            read_idx = (read_idx + 1) % RING_BUFFER_CAPACITY;
            count.fetch_sub(1, std::memory_order_release);
            return true;
        }
    } cloud_ring_buf_;

    // IMU buffer
    std::deque<DV::IMUData> imu_buf_;

    // ARKit pose prior
    Eigen::Matrix4d arkit_pose_ = Eigen::Matrix4d::Identity();
    std::atomic<bool> has_arkit_pose_{false};

    // Visualization output
    std::vector<DV::DVPoint3D> display_cloud_;

    // Full map accumulator (preserves RGB colors for export)
    static constexpr size_t MAX_FULL_MAP_POINTS = 2000000; // 2M points max
    std::vector<DV::DVPoint3D> full_map_;

public:
    // ========================================================================
    // Ablation configuration (for RA-L experiments)
    // ========================================================================
    struct AblationConfig {
        bool enable_bundle_discard = true;   // B&D preprocessing
        bool enable_confidence_weight = true; // Confidence-based weighting
        bool enable_tls = true;              // Truncated Least Squares
        bool enable_imu = true;              // IMU prediction in ESKF
        bool enable_lio = true;              // LIO refinement (false = ARKit-only)
    };

    void setAblationConfig(const AblationConfig& config) { ablation_ = config; }
    const AblationConfig& getAblationConfig() const { return ablation_; }

    // ========================================================================
    // Runtime profiling stats
    // ========================================================================
    struct ProfilingStats {
        int total_frames = 0;
        int keyframes = 0;
        double total_bundle_discard_ms = 0.0;
        double total_lio_ms = 0.0;
        double total_pipeline_ms = 0.0;
        double peak_memory_kb = 0.0;
        int total_input_points = 0;
        int total_output_points = 0;   // After B&D

        double avgBundleDiscardMs() const { return total_frames > 0 ? total_bundle_discard_ms / total_frames : 0; }
        double avgLioMs() const { return keyframes > 0 ? total_lio_ms / keyframes : 0; }
        double avgPipelineMs() const { return total_frames > 0 ? total_pipeline_ms / total_frames : 0; }
        double avgReductionRatio() const { return total_input_points > 0 ? 1.0 - (double)total_output_points / total_input_points : 0; }
        double fps() const { return total_frames > 0 && total_pipeline_ms > 0 ? total_frames / (total_pipeline_ms / 1000.0) : 0; }
    };

    ProfilingStats getProfilingStats() const {
        std::lock_guard<std::mutex> lock(mtx_state_);
        return profiling_;
    }

private:
    AblationConfig ablation_;
    mutable ProfilingStats profiling_;
};

#endif // DEPTHVIZ_ENGINE_HPP
