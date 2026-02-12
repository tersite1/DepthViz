#ifndef VIKIT_MATH_UTILS_H
#define VIKIT_MATH_UTILS_H

#include <Eigen/Core>

namespace vk {

inline double norm_max(const Eigen::VectorXd &v) {
    return v.cwiseAbs().maxCoeff();
}

}

#endif
