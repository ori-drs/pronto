#include "pronto_core/rigidbody.hpp"
#include "pronto_core/rotations.hpp"

using namespace Eigen;

namespace pronto {

RigidBodyState::RigidBodyState(int state_dim) :
  vec(Eigen::VectorXd::Zero(state_dim)), utime(0), quat(Eigen::Quaterniond::Identity())
{
}

RigidBodyState::RigidBodyState() :
  vec(Eigen::VectorXd::Zero((int) basic_num_states)), utime(0), quat(Eigen::Quaterniond::Identity())
{
}

RigidBodyState::RigidBodyState(const Eigen::VectorXd & arg_vec) :
  vec(arg_vec), utime(0), quat(Eigen::Quaterniond::Identity())
{
  this->chiToQuat();
}

RigidBodyState::RigidBodyState(const Eigen::VectorXd & arg_vec, const Eigen::Quaterniond & arg_quat) :
  vec(arg_vec), utime(0), quat(arg_quat)
{
}

void RigidBodyState::integrateForwardConstantVelocity(double time) {
  Eigen::Affine3d trans = getTransTwist(angularVelocity(), velocity(), time);
  this->orientation() = this->orientation() * trans.rotation();
  this->position() = this->position() + this->orientation() * trans.translation();
}

/**
   * phi, theta, psi (roll, pitch, yaw)
   */
Eigen::Vector3d RigidBodyState::getEulerAngles() const {
  return rotation::getEulerAngles(this->quat);
}

  /**
   * phi, theta, psi (roll, pitch, yaw)
   */
void RigidBodyState::setQuatEulerAngles(const Eigen::Vector3d & eulers) {
  this->quat = rotation::setQuatEulerAngles(eulers);
}

void RigidBodyState::alignVelocityXAxis(double vel_tol) {
  Eigen::Quaterniond wind_axes_to_body_axes = Eigen::Quaterniond::Identity(); //transforms vectors in wind frame to vectors in body frame
  if (velocity().norm() > vel_tol) {
    wind_axes_to_body_axes.setFromTwoVectors(Eigen::Vector3d::UnitX(), velocity());
  }

  Eigen::Matrix3d body_axes_to_wind_axes = wind_axes_to_body_axes.toRotationMatrix().transpose();

  angularVelocity() = body_axes_to_wind_axes * angularVelocity();
  velocity() = body_axes_to_wind_axes * velocity();
  acceleration() = body_axes_to_wind_axes * acceleration();
  orientation() = orientation() * wind_axes_to_body_axes;
}

void RigidBodyState::chiToQuat() {
  double chi_norm = this->chi().norm();
  if (chi_norm > .000001) { //tolerance check
    Eigen::Quaterniond dquat;
    dquat = Eigen::AngleAxisd(chi_norm, this->chi() / chi_norm);
    this->quat *= dquat;
    this->chi() = Eigen::Vector3d::Zero();
  }
}

void RigidBodyState::quatToChi() {
  this->chi() = rotation::subtractQuats(this->quat, Eigen::Quaterniond::Identity());
  this->quat = Eigen::Quaterniond::Identity();
}

void RigidBodyState::addState(const RigidBodyState & state_to_add) {
  this->vec += state_to_add.vec;
  this->chiToQuat();
  this->quat *= state_to_add.quat;
}

void RigidBodyState::subtractState(const RigidBodyState & state_to_subtract) {
  this->vec -= state_to_subtract.vec;
  this->quat = state_to_subtract.quat.inverse() * this->quat;
}

bool RigidBodyState::hasNan() const {
  for (int ii = 0; ii < vec.rows(); ii++) {
    if (std::isnan(this->vec(ii)))
      return true;
  }
  for (int ii = 0; ii < 4; ii++) {
    if (std::isnan(this->quat.coeffs()(ii)))
      return true;
  }

  return false;
}

Eigen::Affine3d RigidBodyState::getTransTwistUnscaled(const Eigen::Vector3d & unscaledAngularVelocity,
    const Eigen::Vector3d & unscailedLinearVelocity)
{
  double t = unscaledAngularVelocity.norm();
  Affine3d trans;
  if (t < 0.000000001) {
    trans = Translation3d(unscailedLinearVelocity);
  }
  else {
    Vector3d omega = unscaledAngularVelocity / t;
    Vector3d v = unscailedLinearVelocity / t;

    trans = AngleAxisd(t, unscaledAngularVelocity.normalized());
    trans.translation() = (Matrix3d::Identity() - trans.rotation()) * omega.cross(v) + omega.dot(v) * omega * t;
  }
  return trans;
}

Eigen::Affine3d RigidBodyState::getTransTwist(const Eigen::Vector3d & angularVelocity,
                              const Eigen::Vector3d & linearVelocity,
                              double time)
{
  return getTransTwistUnscaled(time * angularVelocity, time * linearVelocity);
}

std::ostream& operator<<(std::ostream& output, const RigidBodyState & state) {
  output << "angularVelocity: " << state.angularVelocity().transpose();
  output << ", velocity: " << state.velocity().transpose();
  output << ", chi: " << state.chi().transpose();
  output << ", position: " << state.position().transpose();
  output << ", acceleration: " << state.acceleration().transpose();
  output << ", RPY: " << (state.getEulerAngles().transpose()*180.0/M_PI);
  return output;
}

}

