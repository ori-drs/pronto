#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "pronto_core/rigidbody.hpp"

namespace pronto {

/**
 * Rigid body state
 */
class RBIS: public RigidBodyState {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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



  /**
   * copy everything from rigid body state and biases are set to 0
   */
  RBIS(const RigidBodyState & rigid_body_state) :
      RigidBodyState(rbis_num_states)
  {
    this->vec.head(RigidBodyState::basic_num_states) = rigid_body_state.vec;
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

  inline Eigen::Isometry3d getPoseAsIsometry3d()
  {
      Eigen::Isometry3d pose_iso;
      pose_iso.setIdentity();
      pose_iso.translate(this->position());
      pose_iso.rotate(this->orientation());
      return pose_iso;
    }
private:
  bool debug_ = false;
  uint64_t debug_utime_ = 0;

};

inline void setAlmostZeroToZero(Eigen::MatrixXd& m, double eps = 1e-5){
  m.unaryExpr([eps](double x){return (abs(x)<eps)?0.:x;});
}

typedef RBIS::MatrixNd RBIM;

void getIMUProcessLinearizationContinuous(const RBIS & state, RBIM & Ac);

/**
 * @brief insUpdateState integrates a gyro and accelerometer measurements given
 * a delta time. Optionally, it skips the acceleration integration.
 * @param[in] gyro angular velocity measurement, in rad / s, base frame
 * @param[in] accelerometer proper acceleration (= 1 g at rest) in m / s^2
 * @param[in] dt integration constant in s
 * @param[out] state resulting state. If the state had already linear velocity
 * values, they get integrated too.
 */
void insUpdateState(const Eigen::Vector3d & gyro,
                    const Eigen::Vector3d & accelerometer,
                    double dt,
                    RBIS & state);

/**
 * @brief propagates the covariance matrix
 * @param[in] q_gyro
 * @param[in] q_accel
 * @param[in] q_gyro_bias
 * @param[in] q_accel_bias
 * @param[in] state
 * @param[out] cov
 * @param[in] dt
 */
void insUpdateCovariance(double q_gyro,
                         double q_accel,
                         double q_gyro_bias,
                         double q_accel_bias,
                         const RBIS & state,
                         RBIM & cov,
                         double dt);

/**
 * @brief matrixMeasurementGetKandCovDelta
 * @param R
 * @param H
 * @param cov
 * @param z_resid
 * @param dcov
 * @param K
 * @return
 */
double matrixMeasurementGetKandCovDelta(const Eigen::MatrixXd & R,
                                        const Eigen::MatrixXd & H,
                                        const RBIM & cov,
                                        const Eigen::VectorXd & z_resid,
                                        RBIM & dcov,
                                        Eigen::MatrixXd & K);
/**
 * @brief matrixMeasurement
 * @param z
 * @param z_pred
 * @param R
 * @param H
 * @param state
 * @param cov
 * @param dstate
 * @param dcov
 * @return
 */
double matrixMeasurement(const Eigen::VectorXd & z,
                         const Eigen::VectorXd & z_pred,
                         const Eigen::MatrixXd & R,
                         const Eigen::MatrixXd & H,
                         const RBIS & state,
                         const RBIM & cov,
                         RBIS & dstate,
                         RBIM & dcov);
/**
 * @brief indexedMeasurement
 * @param z
 * @param R
 * @param z_indices
 * @param state
 * @param cov
 * @param dstate
 * @param dcov
 * @return
 */
double indexedMeasurement(const Eigen::VectorXd & z,
                          const Eigen::MatrixXd & R,
                          const Eigen::VectorXi & z_indices,
                          const RBIS & state,
                          const RBIM & cov,
                          RBIS & dstate,
                          RBIM & dcov);
/**
 * @brief indexedPlusOrientationMeasurement
 * @param z
 * @param quat
 * @param R
 * @param z_indices
 * @param state
 * @param cov
 * @param dstate
 * @param dcov
 * @param verbose
 * @return
 */
double indexedPlusOrientationMeasurement(const Eigen::VectorXd & z,
                                         const Eigen::Quaterniond & quat,
                                         const Eigen::MatrixXd & R,
                                         const Eigen::VectorXi & z_indices,
                                         const RBIS & state,
                                         const RBIM & cov,
                                         RBIS & dstate,
                                         RBIM & dcov);
/**
 * @brief rbisApplyDelta
 * @param prior_state
 * @param prior_cov
 * @param dstate
 * @param dcov
 * @param posterior_state
 * @param posterior_cov
 */
void rbisApplyDelta(const RBIS & prior_state,
                    const RBIM & prior_cov,
                    const RBIS & dstate,
                    const RBIM & dcov,
                    RBIS & posterior_state,
                    RBIM & posterior_cov);

/**
 * @brief ekfSmoothingStep
 * @param next_state_pred
 * @param next_cov_pred
 * @param next_state
 * @param next_state_cov
 * @param dt
 * @param cur_state
 * @param cur_cov
 */
void ekfSmoothingStep(const RBIS & next_state_pred,
                      const RBIM & next_cov_pred,
                      const RBIS & next_state,
                      const RBIM & next_state_cov,
                      double dt,
                      RBIS & cur_state,
                      RBIM & cur_cov);

}
