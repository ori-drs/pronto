#pragma once
#include <Eigen/Dense>

namespace pronto {
namespace rotation {

Eigen::Matrix3d skewHat(const Eigen::Vector3d & vec);

Eigen::Vector3d subtractQuats(const Eigen::Quaterniond & quat1, const Eigen::Quaterniond & quat2);

double mod2pi_positive(double vin);

double mod2pi(double vin);

Eigen::Vector3d getEulerAngles(const Eigen::Quaterniond & quat);

Eigen::Quaterniond setQuatEulerAngles(const Eigen::Vector3d & eulers);

Eigen::Vector3d getEulerAngles(const Eigen::Matrix3d & rot);

inline Eigen::Vector3d getEulerAnglesDeg(const Eigen::Quaterniond& quat) {
    return getEulerAngles(quat) * 180.0 / M_PI;
}

} // namespace rotation
} // namespace pronto
