#ifndef SOPHUS_SO3_H
#define SOPHUS_SO3_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

namespace Sophus {

class SO3 {
public:
    using Scalar = double;
    using QuaternionType = Eigen::Quaternion<Scalar>;
    using Matrix3Type = Eigen::Matrix<Scalar, 3, 3>;
    using Vector3Type = Eigen::Matrix<Scalar, 3, 1>;

    SO3() : unit_quaternion_(QuaternionType::Identity()) {}
    
    SO3(const QuaternionType& q) : unit_quaternion_(q) {
        unit_quaternion_.normalize();
    }
    
    SO3(const Matrix3Type& R) : unit_quaternion_(R) {
        unit_quaternion_.normalize();
    }

    Matrix3Type matrix() const { return unit_quaternion_.toRotationMatrix(); }
    QuaternionType unit_quaternion() const { return unit_quaternion_; }

    SO3 inverse() const { return SO3(unit_quaternion_.inverse()); }

    SO3 operator*(const SO3& other) const {
        return SO3(unit_quaternion_ * other.unit_quaternion_);
    }

    Vector3Type operator*(const Vector3Type& p) const {
        return unit_quaternion_ * p;
    }

    static SO3 exp(const Vector3Type& omega) {
        Scalar theta = omega.norm();
        if (theta < 1e-10) {
            return SO3();
        }
        Vector3Type n = omega / theta;
        return SO3(Eigen::AngleAxis<Scalar>(theta, n).toRotationMatrix());
    }

    Vector3Type log() const {
        Eigen::AngleAxis<Scalar> aa(unit_quaternion_);
        return aa.axis() * aa.angle();
    }
    
    // Alias for Manifold compatibility if needed (Capital Exp)
    static SO3 Exp(const Vector3Type& omega) { return exp(omega); }
    Matrix3Type R() const { return matrix(); }

private:
    QuaternionType unit_quaternion_;
};

}

#endif
