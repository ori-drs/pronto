#ifndef __RBS_estimator_h__
#define __RBS_estimator_h__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_utils/eigen_utils.hpp>
#include <lcmtypes/bot_core_ins_t.h>
#include <lcmtypes/pronto_indexed_measurement_t.h>
#include <lcmtypes/pronto_filter_state_t.h>
#include <lcmtypes/pronto/filter_state_t.hpp>
#include <bot_lcmgl_client/lcmgl.h>


namespace MavStateEst {

/**
 * Rigid body state
 */
class RBIS: public eigen_utils::RigidBodyState {

public:
  enum {
    gyro_bias_ind = 15, accel_bias_ind = 18, rbis_num_states = 21
  };

  /**
   * Rigid Body Matrix State types
   */
  typedef Eigen::Matrix<double, rbis_num_states, 1> VectorNd;
  typedef Eigen::Matrix<double, rbis_num_states, rbis_num_states> MatrixNd;

  typedef Eigen::Matrix<double, 3, rbis_num_states> Matrix3Nd;
  typedef Eigen::Matrix<double, 6, rbis_num_states> Matrix6Nd;
  typedef Eigen::Matrix<double, 9, rbis_num_states> Matrix9Nd;

  typedef Eigen::Matrix<double, rbis_num_states, 3> MatrixN3d;
  typedef Eigen::Matrix<double, rbis_num_states, 6> MatrixN6d;
  typedef Eigen::Matrix<double, rbis_num_states, 9> MatrixN9d;

  RBIS() :
      RigidBodyState(rbis_num_states)
  {

  }

  RBIS(const VectorNd & vec) :
      RigidBodyState(vec)
  {
    assert(vec.rows() == rbis_num_states);
  }

  RBIS(const VectorNd & vec, const Eigen::Quaterniond & quat) :
      RigidBodyState(vec, quat)
  {
    assert(vec.rows() == rbis_num_states);
  }

  RBIS(const pronto_filter_state_t * msg) :
      RigidBodyState(Eigen::Map<const Eigen::VectorXd>(msg->state, msg->num_states))
  {
    if (msg->num_states != rbis_num_states) {
      fprintf(stderr, "error, constructed RBIS from rbis_filter_state_t of wrong size\n");
    }

    eigen_utils::botDoubleToQuaternion(quat, msg->quat);
    this->utime = msg->utime;
  }

  RBIS(const pronto::filter_state_t & msg) :
      RigidBodyState(Eigen::Map<const Eigen::VectorXd>(&msg.state[0], msg.num_states))
  {
    if (msg.num_states != rbis_num_states) {
      fprintf(stderr, "error, constructed RBIS from rbis_filter_state_t of wrong size\n");
    }

    eigen_utils::botDoubleToQuaternion(quat, msg.quat);
    this->utime = msg.utime;
  }

  /**
   * copy everything from rigid body state and biases are set to 0
   */
  RBIS(const eigen_utils::RigidBodyState & rigid_body_state) :
      RigidBodyState(rbis_num_states)
  {
    this->vec.head(eigen_utils::RigidBodyState::basic_num_states) = rigid_body_state.vec;
    this->utime = rigid_body_state.utime;
    this->quat = rigid_body_state.quat;
  }

  inline Block3Element gyroBias()
  {
    return vec.block<3, 1>(RBIS::gyro_bias_ind, 0);
  }

  inline Block3Element accelBias()
  {
    return vec.block<3, 1>(RBIS::accel_bias_ind, 0);
  }

  inline ConstBlock3Element gyroBias() const
  {
    return vec.block<3, 1>(RBIS::gyro_bias_ind, 0);
  }

  inline ConstBlock3Element accelBias() const
  {
    return vec.block<3, 1>(RBIS::accel_bias_ind, 0);
  }

  static Eigen::Vector3i gyroBiasInds()
  {
    return Eigen::Vector3i::LinSpaced(gyro_bias_ind, gyro_bias_ind + 2);
  }

  static Eigen::Vector3i accelBiasInds()
  {
    return Eigen::Vector3i::LinSpaced(accel_bias_ind, accel_bias_ind + 2);
  }

};

typedef RBIS::MatrixNd RBIM;

void getIMUProcessLinearizationContinuous(const RBIS & state, RBIM & Ac);

void insUpdateState(const Eigen::Vector3d & gyro, const Eigen::Vector3d & accelerometer, double dt, RBIS & state);

void insUpdateCovariance(double q_gyro, double q_accel, double q_gyro_bias, double q_accel_bias, const RBIS & state,
    RBIM & cov, double dt);

double matrixMeasurementGetKandCovDelta(const Eigen::MatrixXd & R, const Eigen::MatrixXd & H, const RBIM & cov,
    const Eigen::VectorXd & z_resid, RBIM & dcov, Eigen::MatrixXd & K);

double matrixMeasurement(const Eigen::VectorXd & z, const Eigen::VectorXd & z_pred, const Eigen::MatrixXd & R,
    const Eigen::MatrixXd & H, const RBIS & state, const RBIM & cov, RBIS & dstate, RBIM & dcov);

double indexedMeasurement(const Eigen::VectorXd & z, const Eigen::MatrixXd & R, const Eigen::VectorXi & z_indices,
    const RBIS & state, const RBIM & cov, RBIS & dstate, RBIM & dcov);

double indexedPlusOrientationMeasurement(const Eigen::VectorXd & z, const Eigen::Quaterniond & quat,
    const Eigen::MatrixXd & R, const Eigen::VectorXi & z_indices, const RBIS & state, const RBIM & cov, RBIS & dstate,
    RBIM & dcov);

void rbisApplyDelta(const RBIS & prior_state, const RBIM & prior_cov, const RBIS & dstate, const RBIM & dcov,
    RBIS & posterior_state, RBIM & posterior_cov);

void ekfSmoothingStep(const RBIS & next_state_pred, const RBIM & next_cov_pred, const RBIS & next_state,
    const RBIM & next_state_cov, double dt, RBIS & cur_state, RBIM & cur_cov);

pronto_filter_state_t * rbisCreateFilterStateMessage(const RBIS & state, const RBIM & cov);

pronto::filter_state_t rbisCreateFilterStateMessageCPP(const RBIS & state, const RBIM & cov);

}
#endif
