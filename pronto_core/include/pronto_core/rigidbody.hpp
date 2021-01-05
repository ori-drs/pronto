#pragma once
#include <Eigen/Dense>

namespace pronto {

/**
 * @brief g_val scalar representing the gravitational acceleration on Earth (positive)
 */
static const double g_val = 9.80665;
/**
 * @brief rho_val air density kg/m^3
 */
static const double rho_val = 1.2;

/**
 * @brief g_vec gravitational acceleration vector (negative along Z)
 *  ENU gravity vector
 */
static const Eigen::Vector3d g_vec = -g_val * Eigen::Vector3d::UnitZ();

/**
 * Basic Rigid Body State representation
 *
 * The chi part of the state vector represents attitude perturbations (exponential coordinates)
 *
 * velocity, angular velocity, and acceleration are tracked in body coordiates
 * acceleration is the "sensed" acceleration including the gravity vector (what an IMU onboard would measure)
 */
class RigidBodyState {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  enum {
    angular_velocity_ind = 0,
    velocity_ind = 3,
    chi_ind = 6,
    position_ind = 9,
    acceleration_ind = 12,
    basic_num_states = 15
  };

  Eigen::VectorXd vec;
  int64_t utime;
  Eigen::Quaterniond quat;



protected:
  RigidBodyState(int state_dim);

public:
  RigidBodyState();

  RigidBodyState(const Eigen::VectorXd & arg_vec);

  RigidBodyState(const Eigen::VectorXd & arg_vec, const Eigen::Quaterniond & arg_quat);

  void integrateForwardConstantVelocity(double time);

  /**
   * phi, theta, psi (roll, pitch, yaw)
   */
  Eigen::Vector3d getEulerAngles() const;

  /**
   * phi, theta, psi (roll, pitch, yaw)
   */
  void setQuatEulerAngles(const Eigen::Vector3d & eulers);

  typedef Eigen::Block<Eigen::VectorXd, 3, 1> Block3Element;
  typedef const Eigen::Block<const Eigen::VectorXd, 3, 1> ConstBlock3Element;

  Block3Element velocity() {
    return vec.block<3, 1>(RigidBodyState::velocity_ind, 0);
  }

  Block3Element chi() {
    return vec.block<3, 1>(RigidBodyState::chi_ind, 0);
  }

  Block3Element position() {
    return vec.block<3, 1>(RigidBodyState::position_ind, 0);
  }

  Block3Element angularVelocity() {
    return vec.block<3, 1>(RigidBodyState::angular_velocity_ind, 0);
  }

  Block3Element acceleration() {
    return vec.block<3, 1>(RigidBodyState::acceleration_ind, 0);
  }

  Eigen::Quaterniond & orientation() {
    return this->quat;
  }

  const Eigen::Quaterniond & orientation() const {
    return this->quat;
  }

  //const returns
  ConstBlock3Element velocity() const {
    return vec.block<3, 1>(RigidBodyState::velocity_ind, 0);
  }

  ConstBlock3Element chi() const {
    return vec.block<3, 1>(RigidBodyState::chi_ind, 0);
  }

  ConstBlock3Element position() const {
    return vec.block<3, 1>(RigidBodyState::position_ind, 0);
  }

  ConstBlock3Element angularVelocity() const {
    return vec.block<3, 1>(RigidBodyState::angular_velocity_ind, 0);
  }

  ConstBlock3Element acceleration() const {
    return vec.block<3, 1>(RigidBodyState::acceleration_ind, 0);
  }

  Eigen::Vector3d accelerationGlobal() const {
    return orientation() * acceleration();
  }

  Eigen::Vector3d accelerationGlobalNoGravity() const {
    return orientation() * acceleration() + g_vec;
  }

  Eigen::Vector3d velocityGlobal() const {
    return orientation() * velocity();
  }

  /**
   * aligns the body x axis with the velocity vector
   */
  void alignVelocityXAxis(double vel_tol = .01);

  void chiToQuat();

  void quatToChi();

  /**
   * add state on right (postmultiplies orientation)
   */
  void addState(const RigidBodyState & state_to_add);

  /**
   * subtract state (premultiplies inverse of state_to_subtract.quat
   */
  void subtractState(const RigidBodyState & state_to_subtract);

  bool hasNan() const;

  static Eigen::Vector3i angularVelocityInds()
  {
    return Eigen::Vector3i::LinSpaced(angular_velocity_ind, angular_velocity_ind + 2);
  }

  static Eigen::Vector3i velocityInds()
  {
    return Eigen::Vector3i::LinSpaced(velocity_ind, velocity_ind + 2);
  }

  static Eigen::Vector3i chiInds()
  {
    return Eigen::Vector3i::LinSpaced(chi_ind, chi_ind + 2);
  }

  static Eigen::Vector3i positionInds()
  {
    return Eigen::Vector3i::LinSpaced(position_ind, position_ind + 2);
  }

  static Eigen::Affine3d getTransTwistUnscaled(const Eigen::Vector3d & unscaledAngularVelocity,
      const Eigen::Vector3d & unscailedLinearVelocity);

  static Eigen::Affine3d getTransTwist(const Eigen::Vector3d & angularVelocity, const Eigen::Vector3d & linearVelocity,
      double time);

  friend std::ostream& operator<<(std::ostream& output, const RigidBodyState & state);
};

}

