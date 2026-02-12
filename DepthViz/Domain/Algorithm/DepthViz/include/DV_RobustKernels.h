#ifndef DEPTHVIZ_ROBUST_KERNELS_H
#define DEPTHVIZ_ROBUST_KERNELS_H

#include <cmath>
#include <algorithm>

namespace DepthViz {

// Task 3.1: Multi-level Gating
inline float getConfidenceWeight(float confidence) {
    if (confidence >= 1.5f) return 1.0f; // High confidence (2)
    if (confidence >= 0.5f) return 0.5f; // Medium confidence (1)
    return 0.0f;                         // Low confidence (0) -> Discard
}

// Task 3.2: Truncated Least Squares (TLS)
inline float computeTLSWeight(float residual, float threshold = 0.10f) {
    if (std::abs(residual) > threshold) {
        return 0.0f; // Hard cut
    }
    return 1.0f; // Standard L2
}

} // namespace DepthViz

#endif
