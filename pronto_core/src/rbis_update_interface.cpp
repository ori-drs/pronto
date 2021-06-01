#include "pronto_core/rbis_update_interface.hpp"
#include "pronto_core/rotations.hpp"
#include <iostream>

namespace pronto {

const char * RBISUpdateInterface::sensor_enum_chars = "igvlfsorxdukpabmtwy";
const char * RBISUpdateInterface::sensor_enum_strings[] =
    { "ins", "gps", "vicon", "laser", "laser_gpf", "scan_matcher", "optic_flow", "reset", "invalid", "rgbd", "fovis", "legodo", "pose_meas", "altimeter", "airspeed", "sideslip", "init_message", "viewer", "yawlock" };

RBISUpdateInterface::sensor_enum RBISUpdateInterface::sensor_enum_from_char(char sensor_char)
{
  const char * arg_char = strchr(sensor_enum_chars, sensor_char);
  if (arg_char == NULL) {
    return invalid;
  }

  int sensor_ind = (int) (arg_char - &sensor_enum_chars[0]);
  if (sensor_ind < 0 || sensor_ind > invalid) {
    sensor_ind = invalid;
  }
  return (sensor_enum) sensor_ind;
}

std::string RBISUpdateInterface::sensorIdToString(const sensor_enum& id){
    return std::string(sensor_enum_strings[(int)id]);
}

std::string RBISUpdateInterface::getSensorIdString() const{
    return sensorIdToString(sensor_id);
}

void RBISResetUpdate::updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood)
{
  posterior_state = reset_state;
  posterior_covariance = reset_cov;
  loglikelihood = 0;
#if DEBUG_MODE
  if(sensor_id == yawlock){
    std::cerr << "?????????????????????" << std::endl;

    std::cerr << "Prior gyro bias: " << prior_state.gyroBias().transpose() << std::endl;
    Eigen::Vector3d rpy = rotation::getEulerAngles(prior_state.orientation()) * 180.0 / M_PI;
    std::cerr << "Prior accel bias: " << prior_state.accelBias().transpose() << std::endl;
    std::cerr << "Prior roll, pitch, yaw [deg]: " << rpy.transpose() << std::endl;

    std::cerr << "Posterior gyro bias: " << posterior_state.gyroBias().transpose() << std::endl;
    std::cerr << "Posterior accel bias: " << posterior_state.accelBias().transpose() << std::endl;
    rpy = rotation::getEulerAngles(posterior_state.orientation()) * 180.0 / M_PI;
    std::cerr << "Posterior roll, pitch, yaw [deg]: " << rpy.transpose() << std::endl;


    std::cerr << "?????????????????????" << std::endl << std::endl;
  }
#endif
}

void RBISIMUProcessStep::updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood)
{
  posterior_state = prior_state;
  posterior_covariance = prior_cov;
#if DEBUG_MODE
  std::cerr << "======================================" << std::endl;
  std::cerr << "       INS process " << std::endl;
  std::cerr << "======================================"  << std::endl;
  std::cerr << "              Time: " << utime << std::endl;
  Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");
  // TODO if this was meant to measure some sort of performance
#endif
  insUpdateState(gyro, accelerometer, dt, posterior_state);
  insUpdateCovariance(q_gyro, q_accel, q_gyro_bias, q_accel_bias, prior_state, posterior_covariance, dt);
#if DEBUG_MODE
  std::cerr << "    Prior velocity: " << prior_state.velocity().transpose().format(CleanFmt) << std::endl;
  std::cerr << "Posterior velocity: " << posterior_state.velocity().transpose().format(CleanFmt) << std::endl;
  std::cerr << "    Prior position: " << prior_state.position().transpose().format(CleanFmt) << std::endl;
  std::cerr << "Posterior position: " << posterior_state.position().transpose().format(CleanFmt) << std::endl;
#endif

  loglikelihood = prior_loglikelihood;
  //    eigen_dump(prior_state);
  //    eigen_dump(prior_cov);
  //    eigen_dump(gyro.transpose());
  //    eigen_dump(q_gyro);
  //    eigen_dump(accelerometer.transpose());
  //    eigen_dump(q_accel);
  //    eigen_dump(posterior_state);
  //    eigen_dump(posterior_covariance);
  //    std::cout << std::endl;
}

void RBISIndexedMeasurement::updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood)
{
  RBIS dstate;
  RBIM dcov;

#if DEBUG_MODE
    std::cout << "mfallon a\n";
    std::cout << " meas: " << measurement.transpose() << "\n";
    std::cout << " mcov: " << measurement_cov << "\n";
    std::cout << "index: " << index.transpose() << "\n";
    std::cout << "prior: " << prior_state << "\n";
    std::cout << "pcov : " << prior_cov << "\n";
  }
#endif
  double current_loglikelihood = indexedMeasurement(measurement, measurement_cov, index, prior_state, prior_cov, dstate,
      dcov);

#if DEBUG_MODE
  std::cout << "dstat: " << dstate <<"\n";
  std::cout << " dcov: " << dcov <<"\n";
  std::cout << dstate.position() << " pos\n";

  std::cerr << "======================================" << std::endl;
  std::cerr << "       " << sensorIdToString(sensor_id) << std::endl;
  std::cerr << "======================================" << std::endl;
  Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");
  std::cerr << "              Time: " << utime << std::endl;
  std::cerr << "       Measurement: " << measurement.transpose().format(CleanFmt) << std::endl
     << "           Indices: " << index.transpose().format(CleanFmt) << std::endl
     << std::endl;
#endif


  rbisApplyDelta(prior_state, prior_cov, dstate, dcov, posterior_state, posterior_covariance);


#if DEBUG_MODE
  is << "    Prior velocity: " << prior_state.velocity().transpose().format(CleanFmt) << std::endl;
  is << "Posterior velocity: " << posterior_state.velocity().transpose().format(CleanFmt) << std::endl;
  is << "    Prior position: " << prior_state.position().transpose().format(CleanFmt) << std::endl;
  is << "Posterior position: " << posterior_state.position().transpose().format(CleanFmt) << std::endl;
#endif


#if DEBUG_MODE
    std::cout << " post: " << posterior_state <<"\n";
    if (verbose_cov) std::cout << "ptcov: " << posterior_covariance <<"\n";
    std::cout << "mfallon b\n";
#endif

  loglikelihood = prior_loglikelihood + current_loglikelihood;
  //    eigen_dump(prior_state);
  //    eigen_dump(prior_cov);
  //    eigen_dump(measurement.transpose());
  //    eigen_dump(measurement_cov);
  //    eigen_dump(posterior_state);
  //    eigen_dump(posterior_covariance);
  //    std::cout << std::endl;
}

void RBISIndexedPlusOrientationMeasurement::updateFilter(const RBIS & prior_state,
                                                         const RBIM & prior_cov,
                                                         double prior_loglikelihood)
{
  RBIS dstate;
  RBIM dcov;
  double current_likelihood = indexedPlusOrientationMeasurement(measurement,
                                                                orientation,
                                                                measurement_cov,
                                                                index,
                                                                prior_state,
                                                                prior_cov,
                                                                dstate,
                                                                dcov);
#if DEBUG_MODE
  std::cerr << "======================================" << std::endl;
  std::cerr << sensorIdToString(sensor_id)  << std::endl;
  std::cerr << "======================================" << std::endl;
  Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");
  std::cerr << "              Time: " << utime << std::endl;
  std::cerr << "       Measurement: " << measurement.transpose() << std::endl
            << "           Indices: " << index.transpose() << std::endl;
#endif
  rbisApplyDelta(prior_state, prior_cov, dstate, dcov, posterior_state, posterior_covariance);

#if DEBUG_MODE
  std::cerr << "    Prior velocity: " << prior_state.velocity().transpose().format(CleanFmt) << std::endl;
  std::cerr << "Posterior velocity: " << posterior_state.velocity().transpose().format(CleanFmt) << std::endl;
  std::cerr << "    Prior position: " << prior_state.position().transpose().format(CleanFmt) << std::endl;
  std::cerr << "Posterior position: " << posterior_state.position().transpose().format(CleanFmt) << std::endl;
#endif
  loglikelihood = prior_loglikelihood + current_likelihood;
}

const int RBISOpticalFlowMeasurement::m;

Eigen::Matrix<double, RBISOpticalFlowMeasurement::m, 1> RBISOpticalFlowMeasurement::measure(const RBIS::VectorNd & state_vec)
{
  RBIS state(state_vec);
  Eigen::Matrix<double, m, 1> meas;

  Eigen::Matrix3d R;
  R = state.orientation().inverse();
  Eigen::Vector3d w = -state.angularVelocity();
  Eigen::Vector3d v = state.velocity() + w.cross(R * r);

  double z = state.position().coeff(2);

  double lambda = z + Eigen::Vector3d::UnitZ().transpose() * R * r;

  Eigen::Matrix3d P1 = eta * zeta2.transpose() + alpha2 * zeta2 * eta.transpose();
  Eigen::Matrix3d P2 = eta * zeta1.transpose() + alpha1 * zeta1 * eta.transpose();
  Eigen::Matrix3d Pt = .5 * (gamma + 1.) * zeta2 * zeta1.transpose() + 5 * (gamma - 1.) * zeta1 * zeta2.transpose();
  Eigen::Matrix3d Ps = eta * eta.transpose() + .5 * (zeta1 * zeta1.transpose() + zeta2 * zeta2.transpose())
      + .5 * gamma * (zeta1 * zeta1.transpose() - zeta2 * zeta2.transpose());

  meas(0) = (Eigen::Vector3d::UnitZ().dot(R * P1 * R.transpose() * v)) / lambda - (alpha2 - 1) * w.dot(R * zeta2);
  meas(1) = (Eigen::Vector3d::UnitZ().dot(R * P2 * R.transpose() * v)) / lambda + (alpha1 - 1) * w.dot(R * zeta1);
  meas(2) = (Eigen::Vector3d::UnitZ().dot(R * Pt * R.transpose() * v)) / lambda - w.dot(R * eta);
  meas(3) = -(Eigen::Vector3d::UnitZ().dot(R * Ps * R.transpose() * v)) / lambda;

  return meas;

}

void RBISOpticalFlowMeasurement::updateFilter(const RBIS & prior_state,
                                              const RBIM & prior_cov,
                                              double prior_loglikelihood)
{
  loglikelihood = prior_loglikelihood;
  //    RBIS dstate;
  //    RBIM dcov;
  //
  //    Eigen::Vector3d z_xys_pred = prior_state.velocity(); // Body frame
  //    Eigen::MatrixXd jac_xys = Eigen::MatrixXd::Zero(3, prior_state.vec.rows());
  //
  //    double height = prior_state.vec(RBIS::position_ind + 2); // Fixed frame
  //
  //    // Here's where the magic happens...
  //
  //    // 1. Compute predicted flow measurement from prior state.
  //    z_xys_pred /= height; // *Only* valid for *very* small roll/pitch angles.
  //
  //    // 2. Compute flow measurement Jacobian (H matrix).
  //    jac_xys.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind).setIdentity();
  //    jac_xys.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind) /= height; //TODO: Better way to do this?
  //    jac_xys.col(RBIS::position_ind + 2) = -z_xys_pred / height;
  //
  //    // 3. Compute matrix measurement update.
  //    matrixMeasurement(z_xys, z_xys_pred, cov_xys, jac_xys, prior_state, prior_cov, dstate, dcov);
  //
  //    // 4. Apply delta update.
  //    rbisApplyDelta(prior_state, prior_cov, dstate, dcov, posterior_state, posterior_covariance);
  //
  //    // 5. Relax...
  posterior_covariance = prior_cov;
  posterior_state = prior_state;
//    if (prior_state.position().coeff(2) < 0.25)
//      return;

  //Step aside, Merlin...

  // 0. Set up parameters
  int n = prior_state.vec.size();

  double a2 = 1e-6; //=alpha^2. Tune to taste.  Param file?
  double b = 2;
  double k = 0;

  double lambda = a2 * (n + k) - n;
  double Ws0 = lambda / (n + lambda);
  double Wc0 = lambda / (n + lambda) + (1 - a2 + b);
  double Wi = 1 / (2 * (n + lambda));

  // 1. Generate sigma points
  Eigen::MatrixXd chi(n, 2 * n + 1);
  Eigen::MatrixXd chol(n, n);
  chol = prior_cov.llt().matrixL();

  //NaN check
  if ((chol.array() != chol.array()).any()) {
    std::cout << "Optical flow has broken!" << std::endl;
    return;
  }

  chi.col(0) = prior_state.vec;
  for (int i = 0; i < n; ++i) {
    chi.col(i + 1) = prior_state.vec + sqrt((n + lambda)) * chol.col(i); //sqrt(diag(i) * (n + lambda)) * chol.col(i);
    chi.col(i + n + 1) = prior_state.vec - sqrt((n + lambda)) * chol.col(i); //sqrt(diag(i) * (n + lambda)) * chol.col(i);
  }

  // 2. Predict optical flow measurements
  Eigen::MatrixXd z(m, 2 * n + 1);
  Eigen::VectorXd zhat(m);
  zhat.setZero();
  for (int i = 0; i < 2 * n + 1; ++i) {
    z.col(i) = measure(chi.col(i));
    if (i == 0)
      zhat += (Ws0 * z.col(i));
    else
      zhat += (Wi * z.col(i));
  }

  // 3. Compute Kalman gain
  Eigen::MatrixXd Pxz(n, m), Pzz(m, m);
  Eigen::VectorXd dz(m);
  Pzz = cov_xyrs;
  Pxz.setZero();
  for (int i = 0; i < (2 * n + 1); ++i) {
    dz = z.col(i) - zhat;
    if (i == 0) {
      Pzz += Wc0 * (dz * dz.transpose());
    }
    else {
      Pzz += Wi * (dz * dz.transpose());
      Pxz += Wi * (chi.col(i) - prior_state.vec) * dz.transpose();
    }
  }
  Eigen::MatrixXd K(n, m);
  K = (Pzz.ldlt().solve(Pxz.transpose())).transpose();

  // 4. Apply update
  posterior_state.vec += K * (z_meas - zhat);
  posterior_covariance -= K * Pzz * K.transpose();

}

}
