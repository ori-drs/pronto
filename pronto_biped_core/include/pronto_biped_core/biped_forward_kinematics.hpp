#pragma once

#include <Eigen/Dense>
#include <vector>

namespace pronto {
namespace biped {

class BipedForwardKinematics {
public:
  using Transform = Eigen::Isometry3d;
  using JointState = std::vector<double>;

  virtual Transform getLeftFootPose(const JointState& q) = 0;
  virtual Transform getRightFootPose(const JointState& q) = 0;

  virtual bool getLeftFootPose(const JointState& q, Transform& x) = 0;
  virtual bool getRightFootPose(const JointState& q, Transform& x) = 0;

};

} // namespace biped
} // namespace pronto
