#include "pronto_core/rotations.hpp"

namespace pronto {
namespace rotation {
Eigen::Matrix3d skewHat(const Eigen::Vector3d & vec)
{
  Eigen::Matrix3d skew_hat;
  skew_hat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return skew_hat;
}

/**
 * returns the exponential coordinates of quat1 - quat2
 * (quat2.inverse() * quat1)
 */
Eigen::Vector3d subtractQuats(const Eigen::Quaterniond & quat1, const Eigen::Quaterniond & quat2)
{
  Eigen::Quaterniond quat_resid = quat2.inverse() * quat1;
  Eigen::AngleAxisd angle_axis_resid(quat_resid);

  double angle = angle_axis_resid.angle();
  angle = mod2pi(angle);
  return angle_axis_resid.axis() * angle;
}

double mod2pi_positive(double vin) {
    double q = vin / (2*M_PI) + 0.5;
    int qi = (int) q;

    return vin - qi*2*M_PI;
}

/** Map v to [-PI, PI] **/
double mod2pi(double vin) {
    if (vin < 0)
        return -mod2pi_positive(-vin);
    else
        return mod2pi_positive(vin);
}

/**
 * @brief getEulerAngles converts a quaternion in roll pitch yaw angles in radians
 * @param quat
 * @return
 */
Eigen::Vector3d getEulerAngles(const Eigen::Quaterniond & quat)
{
  return quat.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

Eigen::Quaterniond setQuatEulerAngles(const Eigen::Vector3d & eulers)
{
  Eigen::Quaterniond quat = Eigen::AngleAxisd(eulers(2), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(eulers(1), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(eulers(0), Eigen::Vector3d::UnitX());

  return quat;
}

Eigen::Vector3d getEulerAngles(const Eigen::Matrix3d & rot)
{
  return rot.eulerAngles(2, 1, 0).reverse(); //ypr
}

}
}


