#include "DV_VIOManager.h"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <numeric>

DV_VIOManager::DV_VIOManager() = default;
DV_VIOManager::~DV_VIOManager() = default;

void DV_VIOManager::init() {
    prev_pyr_.clear();
    prev_pts_.clear();
    landmarks_.clear();
    has_prev_ = false;
    frame_count_ = 0;
}

void DV_VIOManager::setIntrinsics(float fx, float fy, float cx, float cy,
                                   int full_w, int full_h, int depth_w, int depth_h) {
    // Rescale original camera intrinsics to working image resolution
    float sx = static_cast<float>(kWorkWidth) / static_cast<float>(full_w);
    float sy = static_cast<float>(kWorkHeight) / static_cast<float>(full_h);
    work_K_.fx = fx * sx;
    work_K_.fy = fy * sy;
    work_K_.cx = cx * sx;
    work_K_.cy = cy * sy;

    // Depth map intrinsics
    float dx = static_cast<float>(depth_w) / static_cast<float>(full_w);
    float dy = static_cast<float>(depth_h) / static_cast<float>(full_h);
    depth_K_.fx = fx * dx;
    depth_K_.fy = fy * dy;
    depth_K_.cx = cx * dx;
    depth_K_.cy = cy * dy;

    // Scale from working image coords to depth map coords
    work_to_depth_x_ = static_cast<float>(depth_w) / static_cast<float>(kWorkWidth);
    work_to_depth_y_ = static_cast<float>(depth_h) / static_cast<float>(kWorkHeight);

    printf("[VIO] intrinsics set: work=(%.1f,%.1f,%.1f,%.1f) depth=(%.1f,%.1f,%.1f,%.1f)\n",
           work_K_.fx, work_K_.fy, work_K_.cx, work_K_.cy,
           depth_K_.fx, depth_K_.fy, depth_K_.cx, depth_K_.cy);
}

int DV_VIOManager::numTracked() const {
    int count = 0;
    for (const auto& lm : landmarks_) {
        if (lm.valid) count++;
    }
    return count;
}

// ============================================================================
// Image Pyramid
// ============================================================================

void DV_VIOManager::buildPyramid(std::vector<PyramidLevel>& pyr,
                                  const uint8_t* img, int w, int h) {
    pyr.resize(kPyramidLevels);

    // Level 0: copy input
    pyr[0].width = w;
    pyr[0].height = h;
    pyr[0].data.assign(img, img + w * h);

    // Subsequent levels: 2x downsample with 2x2 averaging
    for (int lv = 1; lv < kPyramidLevels; lv++) {
        int pw = pyr[lv - 1].width;
        int ph = pyr[lv - 1].height;
        int nw = pw / 2;
        int nh = ph / 2;
        pyr[lv].width = nw;
        pyr[lv].height = nh;
        pyr[lv].data.resize(nw * nh);

        const uint8_t* src = pyr[lv - 1].data.data();
        uint8_t* dst = pyr[lv].data.data();

        for (int y = 0; y < nh; y++) {
            for (int x = 0; x < nw; x++) {
                int s = src[(2 * y) * pw + (2 * x)]
                      + src[(2 * y) * pw + (2 * x + 1)]
                      + src[(2 * y + 1) * pw + (2 * x)]
                      + src[(2 * y + 1) * pw + (2 * x + 1)];
                dst[y * nw + x] = static_cast<uint8_t>(s / 4);
            }
        }
    }
}

// ============================================================================
// Bilinear Interpolation
// ============================================================================

float DV_VIOManager::sampleBilinear(const uint8_t* img, int w, int h, float x, float y) {
    // Returns NaN for out-of-bounds — propagates cleanly through arithmetic
    int x0 = static_cast<int>(std::floor(x));
    int y0 = static_cast<int>(std::floor(y));
    if (x0 < 0 || y0 < 0 || x0 >= w - 1 || y0 >= h - 1) return std::nanf("");

    float fx = x - x0;
    float fy = y - y0;
    float v00 = img[y0 * w + x0];
    float v10 = img[y0 * w + x0 + 1];
    float v01 = img[(y0 + 1) * w + x0];
    float v11 = img[(y0 + 1) * w + x0 + 1];

    return v00 * (1 - fx) * (1 - fy) + v10 * fx * (1 - fy)
         + v01 * (1 - fx) * fy + v11 * fx * fy;
}

// ============================================================================
// Shi-Tomasi Corner Detection with Grid-based Distribution
// ============================================================================

std::vector<Eigen::Vector2f> DV_VIOManager::detectFeatures(
    const std::vector<PyramidLevel>& pyr, int max_count,
    const std::vector<Eigen::Vector2f>& existing)
{
    const auto& img = pyr[0];
    int w = img.width;
    int h = img.height;
    const uint8_t* data = img.data.data();

    // Mark occupied grid cells from existing features
    int cell_w = w / kGridCols;
    int cell_h = h / kGridRows;
    std::vector<bool> occupied(kGridCols * kGridRows, false);
    for (const auto& pt : existing) {
        int gc = std::min(static_cast<int>(pt.x() / cell_w), kGridCols - 1);
        int gr = std::min(static_cast<int>(pt.y() / cell_h), kGridRows - 1);
        occupied[gr * kGridCols + gc] = true;
    }

    // Compute Sobel gradients and min-eigenvalue response
    // Skip border pixels (margin = 3)
    constexpr int margin = 3;
    constexpr int block_half = 3; // 7x7 block for structure tensor

    struct CandidatePoint {
        float response;
        float x, y;
        int grid_cell;
    };

    // Per-grid-cell best candidates
    std::vector<CandidatePoint> grid_best(kGridCols * kGridRows, {0, 0, 0, -1});

    for (int y = margin + block_half; y < h - margin - block_half; y++) {
        for (int x = margin + block_half; x < w - margin - block_half; x++) {
            // Sobel gradients
            float gx = -1.0f * data[(y-1)*w + (x-1)] + 1.0f * data[(y-1)*w + (x+1)]
                       -2.0f * data[y*w + (x-1)]     + 2.0f * data[y*w + (x+1)]
                       -1.0f * data[(y+1)*w + (x-1)] + 1.0f * data[(y+1)*w + (x+1)];
            float gy = -1.0f * data[(y-1)*w + (x-1)] - 2.0f * data[(y-1)*w + x] - 1.0f * data[(y-1)*w + (x+1)]
                       +1.0f * data[(y+1)*w + (x-1)] + 2.0f * data[(y+1)*w + x] + 1.0f * data[(y+1)*w + (x+1)];

            // Quick pre-filter: skip low-gradient areas
            if (std::abs(gx) + std::abs(gy) < 20.0f) continue;

            // Structure tensor over block
            float sxx = 0, syy = 0, sxy = 0;
            for (int dy = -block_half; dy <= block_half; dy++) {
                for (int dx = -block_half; dx <= block_half; dx++) {
                    int px = x + dx, py = y + dy;
                    float ix = -1.0f * data[(py-1)*w + (px-1)] + 1.0f * data[(py-1)*w + (px+1)]
                               -2.0f * data[py*w + (px-1)]     + 2.0f * data[py*w + (px+1)]
                               -1.0f * data[(py+1)*w + (px-1)] + 1.0f * data[(py+1)*w + (px+1)];
                    float iy = -1.0f * data[(py-1)*w + (px-1)] - 2.0f * data[(py-1)*w + px] - 1.0f * data[(py-1)*w + (px+1)]
                               +1.0f * data[(py+1)*w + (px-1)] + 2.0f * data[(py+1)*w + px] + 1.0f * data[(py+1)*w + (px+1)];
                    sxx += ix * ix;
                    syy += iy * iy;
                    sxy += ix * iy;
                }
            }

            // Min eigenvalue of structure tensor (Shi-Tomasi score)
            float trace = sxx + syy;
            float det = sxx * syy - sxy * sxy;
            float disc = trace * trace - 4.0f * det;
            if (disc < 0) disc = 0;
            float lambda_min = 0.5f * (trace - std::sqrt(disc));

            if (lambda_min < kMinEigen * 1000.0f) continue; // Scale threshold by block size

            int gc = std::min(x / cell_w, kGridCols - 1);
            int gr = std::min(y / cell_h, kGridRows - 1);
            int gi = gr * kGridCols + gc;

            if (occupied[gi]) continue; // Cell already has a feature

            if (lambda_min > grid_best[gi].response) {
                grid_best[gi] = {lambda_min, static_cast<float>(x), static_cast<float>(y), gi};
            }
        }
    }

    // Collect best feature per unoccupied cell
    std::vector<CandidatePoint> candidates;
    for (const auto& gb : grid_best) {
        if (gb.grid_cell >= 0 && gb.response > 0) {
            candidates.push_back(gb);
        }
    }

    // Sort by response (strongest first) and take up to max_count
    std::sort(candidates.begin(), candidates.end(),
              [](const CandidatePoint& a, const CandidatePoint& b) {
                  return a.response > b.response;
              });

    std::vector<Eigen::Vector2f> result;
    int to_add = std::min(static_cast<int>(candidates.size()), max_count);
    result.reserve(to_add);
    for (int i = 0; i < to_add; i++) {
        result.push_back(Eigen::Vector2f(candidates[i].x, candidates[i].y));
    }

    return result;
}

// ============================================================================
// Pyramidal Lucas-Kanade Optical Flow
// ============================================================================

void DV_VIOManager::trackLK(const std::vector<PyramidLevel>& prev,
                             const std::vector<PyramidLevel>& curr,
                             const std::vector<Eigen::Vector2f>& prev_pts,
                             std::vector<Eigen::Vector2f>& curr_pts,
                             std::vector<bool>& status)
{
    int n = static_cast<int>(prev_pts.size());
    curr_pts.resize(n);
    status.assign(n, false);

    for (int i = 0; i < n; i++) {
        float px = prev_pts[i].x();
        float py = prev_pts[i].y();
        float dx_total = 0, dy_total = 0;
        bool ok = true;

        // Coarse to fine
        for (int lv = kPyramidLevels - 1; lv >= 0; lv--) {
            float scale = 1.0f / (1 << lv);
            float spx = px * scale;
            float spy = py * scale;
            float sdx = dx_total * scale;
            float sdy = dy_total * scale;

            const uint8_t* prev_img = prev[lv].data.data();
            const uint8_t* curr_img = curr[lv].data.data();
            int w = prev[lv].width;
            int h = prev[lv].height;

            // Compute structure tensor G over window in prev image
            float gxx = 0, gyy = 0, gxy = 0;
            for (int wy = -kWinHalf; wy <= kWinHalf; wy++) {
                for (int wx = -kWinHalf; wx <= kWinHalf; wx++) {
                    float sx = spx + wx;
                    float sy = spy + wy;
                    // Central difference gradients
                    float ix = sampleBilinear(prev_img, w, h, sx + 1, sy)
                             - sampleBilinear(prev_img, w, h, sx - 1, sy);
                    float iy = sampleBilinear(prev_img, w, h, sx, sy + 1)
                             - sampleBilinear(prev_img, w, h, sx, sy - 1);
                    if (std::isnan(ix) || std::isnan(iy)) continue;
                    gxx += ix * ix;
                    gyy += iy * iy;
                    gxy += ix * iy;
                }
            }

            // Check if trackable (min eigenvalue of G)
            float trace = gxx + gyy;
            float det = gxx * gyy - gxy * gxy;
            float disc = trace * trace - 4.0f * det;
            if (disc < 0) disc = 0;
            float lambda_min = 0.5f * (trace - std::sqrt(disc));
            if (lambda_min < kMinEigen) {
                ok = false;
                break;
            }

            // Inverse of G (2x2)
            float inv_det = 1.0f / (det + 1e-10f);
            float g_inv00 = gyy * inv_det;
            float g_inv11 = gxx * inv_det;
            float g_inv01 = -gxy * inv_det;

            // Iterative refinement
            float ddx = sdx, ddy = sdy;
            for (int iter = 0; iter < kMaxIter; iter++) {
                float bx = 0, by = 0;
                for (int wy = -kWinHalf; wy <= kWinHalf; wy++) {
                    for (int wx = -kWinHalf; wx <= kWinHalf; wx++) {
                        float sx = spx + wx;
                        float sy = spy + wy;
                        float prev_val = sampleBilinear(prev_img, w, h, sx, sy);
                        float curr_val = sampleBilinear(curr_img, w, h, sx + ddx, sy + ddy);
                        if (std::isnan(prev_val) || std::isnan(curr_val)) continue;

                        float it = curr_val - prev_val;
                        float ix = sampleBilinear(prev_img, w, h, sx + 1, sy)
                                 - sampleBilinear(prev_img, w, h, sx - 1, sy);
                        float iy = sampleBilinear(prev_img, w, h, sx, sy + 1)
                                 - sampleBilinear(prev_img, w, h, sx, sy - 1);
                        if (std::isnan(ix) || std::isnan(iy)) continue;

                        bx += ix * it;
                        by += iy * it;
                    }
                }

                float update_x = -(g_inv00 * bx + g_inv01 * by);
                float update_y = -(g_inv01 * bx + g_inv11 * by);
                ddx += update_x;
                ddy += update_y;

                if (std::abs(update_x) < 0.01f && std::abs(update_y) < 0.01f) break;
            }

            // Propagate to next level
            if (lv > 0) {
                dx_total = ddx / scale;
                dy_total = ddy / scale;
            } else {
                dx_total = ddx;
                dy_total = ddy;
            }
        }

        if (!ok) continue;

        float cx = px + dx_total;
        float cy_pt = py + dy_total;

        // Bounds check at level 0
        if (cx < kWinHalf || cx >= prev[0].width - kWinHalf ||
            cy_pt < kWinHalf || cy_pt >= prev[0].height - kWinHalf) {
            continue;
        }

        curr_pts[i] = Eigen::Vector2f(cx, cy_pt);
        status[i] = true;
    }

    // Forward-backward check for outlier rejection
    // Track curr→prev and check if we get back close to the original point
    std::vector<Eigen::Vector2f> back_pts;
    std::vector<bool> back_status;
    // We need to do the reverse tracking
    std::vector<Eigen::Vector2f> tracked_curr;
    std::vector<int> tracked_indices;
    for (int i = 0; i < n; i++) {
        if (status[i]) {
            tracked_curr.push_back(curr_pts[i]);
            tracked_indices.push_back(i);
        }
    }

    if (!tracked_curr.empty()) {
        back_pts.resize(tracked_curr.size());
        back_status.assign(tracked_curr.size(), false);

        // Reverse track: same algorithm, curr→prev
        for (int ti = 0; ti < static_cast<int>(tracked_curr.size()); ti++) {
            float bpx = tracked_curr[ti].x();
            float bpy = tracked_curr[ti].y();
            float bdx = 0, bdy = 0;
            bool bok = true;

            for (int lv = kPyramidLevels - 1; lv >= 0; lv--) {
                float scale = 1.0f / (1 << lv);
                float spx = bpx * scale;
                float spy = bpy * scale;
                float sdx = bdx * scale;
                float sdy = bdy * scale;

                // Note: prev/curr swapped for backward tracking
                const uint8_t* prev_img = curr[lv].data.data();
                const uint8_t* curr_img_data = prev[lv].data.data();
                int w = curr[lv].width;
                int h = curr[lv].height;

                float gxx = 0, gyy = 0, gxy = 0;
                for (int wy = -kWinHalf; wy <= kWinHalf; wy++) {
                    for (int wx = -kWinHalf; wx <= kWinHalf; wx++) {
                        float sx = spx + wx;
                        float sy = spy + wy;
                        float ix = sampleBilinear(prev_img, w, h, sx + 1, sy)
                                 - sampleBilinear(prev_img, w, h, sx - 1, sy);
                        float iy = sampleBilinear(prev_img, w, h, sx, sy + 1)
                                 - sampleBilinear(prev_img, w, h, sx, sy - 1);
                        if (std::isnan(ix) || std::isnan(iy)) continue;
                        gxx += ix * ix;
                        gyy += iy * iy;
                        gxy += ix * iy;
                    }
                }

                float det = gxx * gyy - gxy * gxy;
                float trace = gxx + gyy;
                float disc = trace * trace - 4.0f * det;
                if (disc < 0) disc = 0;
                float lmin = 0.5f * (trace - std::sqrt(disc));
                if (lmin < kMinEigen) { bok = false; break; }

                float inv_det = 1.0f / (det + 1e-10f);
                float gi00 = gyy * inv_det, gi11 = gxx * inv_det, gi01 = -gxy * inv_det;

                float ddx = sdx, ddy = sdy;
                for (int iter = 0; iter < kMaxIter; iter++) {
                    float bxv = 0, byv = 0;
                    for (int wy = -kWinHalf; wy <= kWinHalf; wy++) {
                        for (int wx = -kWinHalf; wx <= kWinHalf; wx++) {
                            float sx = spx + wx, sy = spy + wy;
                            float pv = sampleBilinear(prev_img, w, h, sx, sy);
                            float cv = sampleBilinear(curr_img_data, w, h, sx + ddx, sy + ddy);
                            if (std::isnan(pv) || std::isnan(cv)) continue;
                            float it = cv - pv;
                            float ix = sampleBilinear(prev_img, w, h, sx + 1, sy)
                                     - sampleBilinear(prev_img, w, h, sx - 1, sy);
                            float iy = sampleBilinear(prev_img, w, h, sx, sy + 1)
                                     - sampleBilinear(prev_img, w, h, sx, sy - 1);
                            if (std::isnan(ix) || std::isnan(iy)) continue;
                            bxv += ix * it;
                            byv += iy * it;
                        }
                    }
                    float ux = -(gi00 * bxv + gi01 * byv);
                    float uy = -(gi01 * bxv + gi11 * byv);
                    ddx += ux; ddy += uy;
                    if (std::abs(ux) < 0.01f && std::abs(uy) < 0.01f) break;
                }

                if (lv > 0) { bdx = ddx / scale; bdy = ddy / scale; }
                else { bdx = ddx; bdy = ddy; }
            }

            if (bok) {
                back_pts[ti] = Eigen::Vector2f(bpx + bdx, bpy + bdy);
                back_status[ti] = true;
            }
        }

        // Check forward-backward consistency
        for (int ti = 0; ti < static_cast<int>(tracked_indices.size()); ti++) {
            int orig_i = tracked_indices[ti];
            if (!back_status[ti]) {
                status[orig_i] = false;
                continue;
            }
            float dist = (back_pts[ti] - prev_pts[orig_i]).norm();
            if (dist > kFBThreshold) {
                status[orig_i] = false;
            }
        }
    }
}

// ============================================================================
// Main Processing
// ============================================================================

int DV_VIOManager::processFrame(const uint8_t* gray, int work_w, int work_h,
                                 const float* depth_map, int depth_w, int depth_h,
                                 const Eigen::Matrix4d& current_pose)
{
    if (work_w != kWorkWidth || work_h != kWorkHeight) {
        printf("[VIO] WARNING: expected %dx%d, got %dx%d\n", kWorkWidth, kWorkHeight, work_w, work_h);
        return 0;
    }

    // Build current pyramid
    std::vector<PyramidLevel> curr_pyr;
    buildPyramid(curr_pyr, gray, work_w, work_h);

    if (!has_prev_) {
        // First frame: detect features, no tracking yet
        auto new_pts = detectFeatures(curr_pyr, kMaxFeatures, {});
        printf("[VIO] first frame: detected %zu features\n", new_pts.size());

        prev_pyr_ = std::move(curr_pyr);
        prev_pts_ = std::move(new_pts);

        // Initialize landmarks (no 3D yet, will be set on next frame)
        landmarks_.clear();
        landmarks_.resize(prev_pts_.size());
        for (size_t i = 0; i < prev_pts_.size(); i++) {
            landmarks_[i].uv = prev_pts_[i];
            landmarks_[i].track_length = 1;
            landmarks_[i].valid = false;

            // Try to get depth for initial 3D
            float du = prev_pts_[i].x() * work_to_depth_x_;
            float dv = prev_pts_[i].y() * work_to_depth_y_;
            int di = static_cast<int>(std::round(du));
            int dj = static_cast<int>(std::round(dv));
            if (di >= 0 && di < depth_w && dj >= 0 && dj < depth_h) {
                float depth = depth_map[dj * depth_w + di];
                if (depth > 0.1f && depth < 5.0f && !std::isnan(depth)) {
                    // Unproject to ARKit camera frame
                    float u = prev_pts_[i].x();
                    float v = prev_pts_[i].y();
                    double X = (u - work_K_.cx) * depth / work_K_.fx;
                    double Y = -((v - work_K_.cy) * depth / work_K_.fy); // Y-up
                    double Z = -depth; // Z-backward

                    // Transform to world frame using current pose
                    Eigen::Vector3d p_cam(X, Y, Z);
                    Eigen::Matrix3d R = current_pose.block<3, 3>(0, 0);
                    Eigen::Vector3d t = current_pose.block<3, 1>(0, 3);
                    landmarks_[i].p_world = R * p_cam + t;
                    landmarks_[i].valid = true;
                }
            }
        }

        has_prev_ = true;
        frame_count_++;
        return 0; // No tracking on first frame
    }

    // Track features from previous to current frame
    std::vector<Eigen::Vector2f> curr_pts;
    std::vector<bool> track_status;
    trackLK(prev_pyr_, curr_pyr, prev_pts_, curr_pts, track_status);

    // Update landmarks: remove failed tracks, update 2D observations
    std::vector<Eigen::Vector2f> surviving_pts;
    std::vector<VisualLandmark> surviving_lm;
    int valid_3d_count = 0;

    for (size_t i = 0; i < prev_pts_.size(); i++) {
        if (!track_status[i]) continue;
        if (i >= landmarks_.size()) continue;

        auto lm = landmarks_[i];
        lm.uv = curr_pts[i];
        lm.track_length++;

        // If no 3D yet, try to initialize from depth
        if (!lm.valid) {
            float du = curr_pts[i].x() * work_to_depth_x_;
            float dv = curr_pts[i].y() * work_to_depth_y_;
            int di = static_cast<int>(std::round(du));
            int dj = static_cast<int>(std::round(dv));
            if (di >= 0 && di < depth_w && dj >= 0 && dj < depth_h) {
                float depth = depth_map[dj * depth_w + di];
                if (depth > 0.1f && depth < 5.0f && !std::isnan(depth)) {
                    float u = curr_pts[i].x();
                    float v = curr_pts[i].y();
                    double X = (u - work_K_.cx) * depth / work_K_.fx;
                    double Y = -((v - work_K_.cy) * depth / work_K_.fy);
                    double Z = -depth;
                    Eigen::Vector3d p_cam(X, Y, Z);
                    Eigen::Matrix3d R = current_pose.block<3, 3>(0, 0);
                    Eigen::Vector3d t = current_pose.block<3, 1>(0, 3);
                    lm.p_world = R * p_cam + t;
                    lm.valid = true;
                }
            }
        }

        surviving_pts.push_back(curr_pts[i]);
        surviving_lm.push_back(lm);
        if (lm.valid) valid_3d_count++;
    }

    // Detect new features if we're running low
    if (static_cast<int>(surviving_pts.size()) < kMinFeatures) {
        int need = kMaxFeatures - static_cast<int>(surviving_pts.size());
        auto new_pts = detectFeatures(curr_pyr, need, surviving_pts);

        for (const auto& np : new_pts) {
            surviving_pts.push_back(np);
            VisualLandmark nlm;
            nlm.uv = np;
            nlm.track_length = 1;
            nlm.valid = false;

            // Try depth init
            float du = np.x() * work_to_depth_x_;
            float dv = np.y() * work_to_depth_y_;
            int di = static_cast<int>(std::round(du));
            int dj = static_cast<int>(std::round(dv));
            if (di >= 0 && di < depth_w && dj >= 0 && dj < depth_h) {
                float depth = depth_map[dj * depth_w + di];
                if (depth > 0.1f && depth < 5.0f && !std::isnan(depth)) {
                    double X = (np.x() - work_K_.cx) * depth / work_K_.fx;
                    double Y = -((np.y() - work_K_.cy) * depth / work_K_.fy);
                    double Z = -depth;
                    Eigen::Vector3d p_cam(X, Y, Z);
                    Eigen::Matrix3d R = current_pose.block<3, 3>(0, 0);
                    Eigen::Vector3d t = current_pose.block<3, 1>(0, 3);
                    nlm.p_world = R * p_cam + t;
                    nlm.valid = true;
                    valid_3d_count++;
                }
            }
            surviving_lm.push_back(nlm);
        }
    }

    // Log (first 10 frames + every 30th)
    if (frame_count_ < 10 || frame_count_ % 30 == 0) {
        printf("[VIO] frame=%d tracked=%zu valid3d=%d total=%zu\n",
               frame_count_, surviving_pts.size(), valid_3d_count, surviving_lm.size());
    }

    // Swap state
    prev_pyr_ = std::move(curr_pyr);
    prev_pts_ = std::move(surviving_pts);
    landmarks_ = std::move(surviving_lm);
    frame_count_++;

    return valid_3d_count;
}
