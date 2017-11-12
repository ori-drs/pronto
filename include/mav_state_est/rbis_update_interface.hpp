#ifndef __rbis_update_interface_h__
#define __rbis_update_interface_h__

#include "rbis.hpp"

namespace MavStateEst {

class RBISUpdateInterface {
public:
  typedef enum {
    ins, gps, vicon, laser, laser_gpf, scan_matcher, optical_flow, reset, invalid, rgbd, fovis, legodo, pose_meas, altimeter, airspeed, sideslip, init_message, viewer, yawlock
  } sensor_enum;

  int64_t utime;

  RBIS posterior_state;
  RBIM posterior_covariance;
  double loglikelihood; //loglikelihood of measurements up to and including the one in this update
  sensor_enum sensor_id;

  RBISUpdateInterface(sensor_enum sensor_id_, int64_t utime_) :
      posterior_state(), posterior_covariance(), sensor_id(sensor_id_), utime(utime_)
  {
  }

  virtual ~RBISUpdateInterface()
  {
  }

  /**
   * Sets the posterior state and cov by applying the update contained in this RBISUpdate from the given prior state and cov
   *
   * returns the log likelihood of the measurement
   */
  virtual void updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood) = 0;

  static const char * sensor_enum_chars;
  static const char * sensor_enum_strings[];
  sensor_enum sensor_enum_from_char(char sensor_char);
};

/**
 * Dummy update for resetting the state of the filter
 */
class RBISResetUpdate: public RBISUpdateInterface {
public:
  RBIS reset_state;
  RBIM reset_cov;
  RBISResetUpdate(const RBIS & state, const RBIM & cov, sensor_enum sensor_id_, int64_t utime) :
      RBISUpdateInterface(sensor_id_, utime), reset_state(state), reset_cov(cov)
  {
  }

  void updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood);
};

/**
 * process step driven by IMU measurement
 */
class RBISIMUProcessStep: public RBISUpdateInterface {
public:
  Eigen::Vector3d gyro;
  Eigen::Vector3d accelerometer;
  double dt;
  double q_gyro;
  double q_accel;
  double q_gyro_bias;
  double q_accel_bias;

  RBISIMUProcessStep(const Eigen::Vector3d & gyro_, const Eigen::Vector3d & accelerometer_, double q_gyro_,
      double q_accel_, double q_gyro_bias_, double q_accel_bias_, double dt_, int64_t utime) :
      RBISUpdateInterface(ins, utime), gyro(gyro_), accelerometer(accelerometer_), q_gyro(q_gyro_), q_accel(q_accel_), q_gyro_bias(
          q_gyro_bias_), q_accel_bias(q_accel_bias_), dt(dt_)
  {
  }

  void updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood);

};

/**
 * direct measurement of some piece of the state vector described by index
 */
class RBISIndexedMeasurement: public RBISUpdateInterface {
public:
  Eigen::VectorXi index;
  Eigen::VectorXd measurement;
  Eigen::MatrixXd measurement_cov;

  RBISIndexedMeasurement(const Eigen::VectorXi & index_, const Eigen::VectorXd & measurement_,
      const Eigen::MatrixXd & measurement_cov_, RBISUpdateInterface::sensor_enum sensor_id_, int64_t utime) :
      RBISUpdateInterface(sensor_id_, utime), index(index_), measurement(measurement_), measurement_cov(
          measurement_cov_)
  {

  }

  void updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood);
};

/**
 * direct measurement of some piece of the state, as well as orientation
 */
class RBISIndexedPlusOrientationMeasurement: public RBISUpdateInterface {
public:
  Eigen::VectorXi index;
  Eigen::VectorXd measurement;
  Eigen::MatrixXd measurement_cov;
  Eigen::Quaterniond orientation;

  RBISIndexedPlusOrientationMeasurement(const Eigen::VectorXi & index_, const Eigen::VectorXd & measurement_,
      const Eigen::MatrixXd & measurement_cov_, const Eigen::Quaterniond & orientation_,
      RBISUpdateInterface::sensor_enum sensor_id_, int64_t utime) :
      RBISUpdateInterface(sensor_id_, utime), index(index_), measurement(measurement_), measurement_cov(
          measurement_cov_), orientation(orientation_)
  {
  }

  void updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood);
};

/**
 * measurement computed indirectly as a function of the state - requires
 * measurement Jacobian matrix (specifically for optic flow in this case)
 */
#include <lcmtypes/pronto_optical_flow_t.h>
#include <lcm/lcm.h>
class RBISOpticalFlowMeasurement: public RBISUpdateInterface {

private:
  static const int m = 4;

  Eigen::Matrix<double, m, 1> measure(const RBIS::VectorNd & state_vec);
  void publish(const Eigen::VectorXd & z);
  Eigen::Vector3d r; // Names are to match my derivations.  Camera position in body frame
  Eigen::Vector3d zeta1, zeta2, eta; // Camera rotation from body frame
  double alpha1, alpha2, gamma;

public:
  Eigen::VectorXd z_meas;
  Eigen::MatrixXd cov_xyrs;

  RBISOpticalFlowMeasurement(const Eigen::VectorXd & z_meas_, const Eigen::MatrixXd & cov_xyrs_,
      const Eigen::VectorXd & body_to_cam_trans_, const Eigen::MatrixXd & body_to_cam_rot_, const double alpha1_,
      const double alpha2_, const double gamma_, RBISUpdateInterface::sensor_enum sensor_id_, int64_t utime) :
      RBISUpdateInterface(sensor_id_, utime), z_meas(z_meas_), cov_xyrs(cov_xyrs_), r(body_to_cam_trans_), zeta1(
          body_to_cam_rot_.col(0)), zeta2(body_to_cam_rot_.col(1)), eta(body_to_cam_rot_.col(2)), alpha1(alpha1_), alpha2(
          alpha2_), gamma(gamma_)
  {
  }

  void updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood);

};


}

#endif
