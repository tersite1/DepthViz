#ifndef DISPATCH_UTILS_H
#define DISPATCH_UTILS_H

#include <dispatch/dispatch.h>
#include <functional>
#include <vector>
#include <cmath>

namespace Common {

// Helper to replace tbb::parallel_for and #pragma omp parallel for
// Uses Apple's Grand Central Dispatch (GCD)
inline void parallel_for(size_t begin, size_t end, const std::function<void(size_t)>& body) {
    if (end <= begin) return;
    size_t count = end - begin;
    
    // Use the global concurrent queue
    dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0);
    
    dispatch_apply(count, queue, ^(size_t i) {
        body(begin + i);
    });
}

// Strided version if needed (OpenMP schedule)
inline void parallel_for(size_t begin, size_t end, size_t step, const std::function<void(size_t)>& body) {
    if (end <= begin) return;
    size_t count = (end - begin + step - 1) / step;
    
    dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0);
    
    dispatch_apply(count, queue, ^(size_t i) {
        body(begin + i * step);
    });
}

}

#endif
