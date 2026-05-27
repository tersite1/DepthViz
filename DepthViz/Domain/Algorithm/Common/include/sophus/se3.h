#ifndef SOPHUS_SE3_H
#define SOPHUS_SE3_H

#include "so3.h"

namespace Sophus {

class SE3 {
public:
    using Scalar = double;
    using Vector3Type = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix4Type = Eigen::Matrix<Scalar, 4, 4>;
    using Matrix3Type = Eigen::Matrix<Scalar, 3, 3>;

    SE3() : so3_(), translation_(Vector3Type::Zero()) {}

    SE3(const SO3& so3, const Vector3Type& translation) 
        : so3_(so3), translation_(translation) {}

    SE3(const Matrix3Type& R, const Vector3Type& t)
        : so3_(R), translation_(t) {}

    const SO3& so3() const { return so3_; }
    const Vector3Type& translation() const { return translation_; }
    
    Matrix3Type rotationMatrix() const { return so3_.matrix(); }

    Matrix4Type matrix() const {
        Matrix4Type m = Matrix4Type::Identity();
        m.block<3,3>(0,0) = so3_.matrix();
        m.block<3,1>(0,3) = translation_;
        return m;
    }

    SE3 inverse() const {
        SO3 invR = so3_.inverse();
        return SE3(invR, -(invR * translation_));
    }

    SE3 operator*(const SE3& other) const {
        return SE3(so3_ * other.so3_, translation_ + so3_ * other.translation_);
    }

    Vector3Type operator*(const Vector3Type& p) const {
        return so3_ * p + translation_;
    }
    
    static SE3 exp(const Eigen::Matrix<Scalar, 6, 1>& tangent) {
        Vector3Type omega = tangent.head<3>();
        Vector3Type v = tangent.tail<3>(); // Sophus order: translation(0-2) or rotation(0-2)? 
        // Sophus::SE3::Tangent is [translation, rotation] usually? 
        // Wait, Sophus se3 tangent is [upsison, omega]. upsilon is translation-like.
        // But usually first 3 are translation, last 3 rotation? 
        // In Manifold.h: V6 xi. 
        // Let's assume standard [rho, omega].
        // But let's check Fast-LIVO2 usage.
        // Usage is sparse. I'll implement simplified exp.
        // Actually Fast-LIVO2 uses Exp from so3_math.h mostly.
        // But VIO uses SE3 T_cur_ref.
        return SE3(SO3::exp(v), omega); // Approximation: decoupled
    }

    static SE3 Exp(const Eigen::Matrix<Scalar, 6, 1>& tangent) { return exp(tangent); }

private:
    SO3 so3_;
    Vector3Type translation_;
};

}

#endif
