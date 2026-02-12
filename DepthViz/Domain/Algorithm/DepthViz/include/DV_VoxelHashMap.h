#ifndef DV_VOXEL_HASH_MAP_H
#define DV_VOXEL_HASH_MAP_H

// DV-SLAM Voxel Hash Map
// Replaces LI2Sup::OctVoxMap (which depends on boost, PCL, tsl::robin_map).
// Self-contained spatial hash map for point-to-plane ICP.
// Header-only implementation.

#include "DV_Types.h"
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cstring>
#include <Eigen/Dense>

namespace DV {

// ============================================================================
// KNN Result
// ============================================================================

struct KNNResult {
    static constexpr int MAX_K = 5;
    V3f points[MAX_K];
    float distances[MAX_K];
    int count = 0;

    void clear() { count = 0; }

    void insert(const V3f& p, float dist) {
        if (count < MAX_K) {
            points[count] = p;
            distances[count] = dist;
            count++;
        } else {
            // Replace the farthest
            int worst = 0;
            for (int i = 1; i < MAX_K; i++) {
                if (distances[i] > distances[worst]) worst = i;
            }
            if (dist < distances[worst]) {
                points[worst] = p;
                distances[worst] = dist;
            }
        }
    }

    float worstDistance() const {
        if (count == 0) return 1e10f;
        float worst = distances[0];
        for (int i = 1; i < count; i++) {
            worst = std::max(worst, distances[i]);
        }
        return worst;
    }
};

// ============================================================================
// VoxelCell — stores points within a single voxel
// ============================================================================

struct VoxelCell {
    static constexpr int MAX_POINTS_PER_VOXEL = 20;
    V3f points[MAX_POINTS_PER_VOXEL];
    int count = 0;
    uint32_t last_access = 0; // For LRU eviction

    void addPoint(const V3f& p) {
        if (count < MAX_POINTS_PER_VOXEL) {
            points[count++] = p;
        }
    }

    V3f centroid() const {
        if (count == 0) return V3f::Zero();
        V3f sum = V3f::Zero();
        for (int i = 0; i < count; i++) {
            sum += points[i];
        }
        return sum / static_cast<float>(count);
    }
};

// ============================================================================
// DV_VoxelHashMap — main spatial data structure
// ============================================================================

class DV_VoxelHashMap {
public:
    explicit DV_VoxelHashMap(float voxel_size = 0.1f, size_t max_voxels = 500000)
        : voxel_size_(voxel_size)
        , inv_voxel_size_(1.0f / voxel_size)
        , max_voxels_(max_voxels)
        , access_counter_(0) {}

    // Insert world-frame points into the map
    void insert(const std::vector<V3f>& points) {
        for (const auto& p : points) {
            int64_t key = computeKey(p);
            auto it = map_.find(key);
            if (it == map_.end()) {
                if (map_.size() >= max_voxels_) {
                    evictLRU();
                }
                VoxelCell cell;
                cell.addPoint(p);
                cell.last_access = nextAccessCounter();
                map_[key] = cell;
            } else {
                it->second.addPoint(p);
                it->second.last_access = nextAccessCounter();
            }
        }
    }

    // Insert a single point
    void insertPoint(const V3f& p) {
        int64_t key = computeKey(p);
        auto it = map_.find(key);
        if (it == map_.end()) {
            if (map_.size() >= max_voxels_) {
                evictLRU();
            }
            VoxelCell cell;
            cell.addPoint(p);
            cell.last_access = nextAccessCounter();
            map_[key] = cell;
        } else {
            it->second.addPoint(p);
            it->second.last_access = nextAccessCounter();
        }
    }

    // Query approximate K nearest neighbors by searching the voxel and its 26 neighbors
    bool getTopK(const V3f& query, KNNResult& result, int K = 5) const {
        result.clear();

        int64_t qx = static_cast<int64_t>(std::floor(query.x() * inv_voxel_size_));
        int64_t qy = static_cast<int64_t>(std::floor(query.y() * inv_voxel_size_));
        int64_t qz = static_cast<int64_t>(std::floor(query.z() * inv_voxel_size_));

        // Search 3x3x3 neighborhood
        for (int64_t dx = -1; dx <= 1; dx++) {
            for (int64_t dy = -1; dy <= 1; dy++) {
                for (int64_t dz = -1; dz <= 1; dz++) {
                    int64_t key = (qx + dx) + (qy + dy) * 10000LL + (qz + dz) * 100000000LL;
                    auto it = map_.find(key);
                    if (it == map_.end()) continue;

                    const VoxelCell& cell = it->second;
                    for (int i = 0; i < cell.count; i++) {
                        float dist = (cell.points[i] - query).squaredNorm();
                        result.insert(cell.points[i], dist);
                    }
                }
            }
        }

        return result.count >= K;
    }

    // Fit a local plane to K nearest neighbors via SVD
    // Returns true if plane fit succeeded (enough points, good conditioning)
    static bool fitPlane(const KNNResult& knn, V3f& normal, V3f& centroid) {
        if (knn.count < 3) return false;

        // Compute centroid
        centroid = V3f::Zero();
        for (int i = 0; i < knn.count; i++) {
            centroid += knn.points[i];
        }
        centroid /= static_cast<float>(knn.count);

        // Build covariance matrix
        M3f cov = M3f::Zero();
        for (int i = 0; i < knn.count; i++) {
            V3f diff = knn.points[i] - centroid;
            cov += diff * diff.transpose();
        }
        cov /= static_cast<float>(knn.count);

        // SVD: smallest eigenvector = normal
        Eigen::SelfAdjointEigenSolver<M3f> solver(cov);
        if (solver.info() != Eigen::Success) return false;

        // Eigenvalues are in ascending order; smallest = normal direction
        normal = solver.eigenvectors().col(0);

        // Check planarity: smallest eigenvalue should be much smaller than second
        V3f eigenvalues = solver.eigenvalues();
        if (eigenvalues(0) < 0.f) eigenvalues(0) = 0.f;
        if (eigenvalues(1) <= 1e-6f) return false;
        float planarity = eigenvalues(0) / eigenvalues(1);
        if (planarity > 0.3f) return false; // Not planar enough

        return true;
    }

    // Get total number of voxels
    size_t size() const { return map_.size(); }

    // Get all points (for visualization)
    std::vector<V3f> getAllPoints() const {
        std::vector<V3f> all;
        all.reserve(map_.size() * 5);
        for (const auto& pair : map_) {
            const VoxelCell& cell = pair.second;
            // Return centroid per voxel for efficiency
            if (cell.count > 0) {
                all.push_back(cell.centroid());
            }
        }
        return all;
    }

    void clear() {
        map_.clear();
        access_counter_ = 0;
    }

    float voxelSize() const { return voxel_size_; }

private:
    int64_t computeKey(const V3f& p) const {
        int64_t ix = static_cast<int64_t>(std::floor(p.x() * inv_voxel_size_));
        int64_t iy = static_cast<int64_t>(std::floor(p.y() * inv_voxel_size_));
        int64_t iz = static_cast<int64_t>(std::floor(p.z() * inv_voxel_size_));
        return ix + iy * 10000LL + iz * 100000000LL;
    }

    uint32_t nextAccessCounter() {
        ++access_counter_;
        // On overflow, re-index all voxels to avoid LRU confusion
        if (access_counter_ == 0) {
            uint32_t idx = 1;
            for (auto& pair : map_) {
                pair.second.last_access = idx++;
            }
            access_counter_ = idx;
        }
        return access_counter_;
    }

    void evictLRU() {
        // Evict 10% of oldest voxels
        size_t evict_count = max_voxels_ / 10;
        if (evict_count == 0) evict_count = 1;

        // Find evict_count oldest entries
        std::vector<std::pair<uint32_t, int64_t>> entries;
        entries.reserve(map_.size());
        for (const auto& pair : map_) {
            entries.emplace_back(pair.second.last_access, pair.first);
        }

        // Partial sort to find the oldest
        size_t n = std::min(evict_count, entries.size());
        std::partial_sort(entries.begin(), entries.begin() + n, entries.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        for (size_t i = 0; i < n; i++) {
            map_.erase(entries[i].second);
        }
    }

    float voxel_size_;
    float inv_voxel_size_;
    size_t max_voxels_;
    uint32_t access_counter_;
    std::unordered_map<int64_t, VoxelCell> map_;
};

} // namespace DV

#endif // DV_VOXEL_HASH_MAP_H
