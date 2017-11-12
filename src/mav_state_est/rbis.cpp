#include "mav_state_est/rbis.hpp"

using namespace eigen_utils;
using namespace Eigen;
using namespace std;

namespace MavStateEst {

typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;

void getIMUProcessLinearizationContinuous(const RBIS & state, RBIM & Ac)
{
  Ac = RBIM::Zero();

  Matrix3d omega_hat = skewHat(state.angularVelocity());
  Matrix3d vb_hat = skewHat(state.velocity());
  Matrix3d RotationMat = state.quat.toRotationMatrix();

  Ac.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind) = -omega_hat;
  Ac.block<3, 3>(RBIS::velocity_ind, RBIS::chi_ind) = skewHat(state.quat.inverse() * g_vec);

  //Ac_chi
  Ac.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = -omega_hat;

  //Ac_Delta
  Ac.block<3, 3>(RBIS::position_ind, RBIS::velocity_ind) = RotationMat;
  Ac.block<3, 3>(RBIS::position_ind, RBIS::chi_ind).noalias() = -RotationMat * vb_hat;

  //bias stuff
  Ac.block<3, 3>(RBIS::velocity_ind, RBIS::gyro_bias_ind) = -vb_hat;
  Ac.block<3, 3>(RBIS::velocity_ind, RBIS::accel_bias_ind) = -Matrix3d::Identity();
  Ac.block<3, 3>(RBIS::chi_ind, RBIS::gyro_bias_ind) = -Matrix3d::Identity();

}

void insUpdateState(const Eigen::Vector3d & gyro, const Eigen::Vector3d & accelerometer, double dt, RBIS & state)
{
 
  bool verbose = false;
  if (verbose){
    std::cout << "mfallon insUpdateState\n";
    std::cout << state << " state prior\n";
    std::cout << gyro.transpose() << "\n";
    std::cout << accelerometer.transpose() << "\n";
    std::cout << dt << "\n";
  }
  
  //put the ins measurements into the state, correcting for bias
  state.angularVelocity() = gyro - state.gyroBias();
  state.acceleration() = accelerometer - state.accelBias();

  //compute derivatives
  RBIS dstate; //initialize everything to 0
  dstate.velocity() = -state.angularVelocity().cross(state.velocity());
  dstate.velocity().noalias() += state.quat.inverse() * g_vec + state.acceleration();

  dstate.chi() = state.angularVelocity();
  dstate.position().noalias() = state.quat * state.velocity();

  //integrate
  dstate.vec *= dt;
  dstate.chiToQuat();
  
  if (verbose){
    std::cout << dstate << " dstate\n";
  }
  
  state.addState(dstate);
  
  if (verbose){
    std::cout << dstate << " state updated\n";
    std::cout << "\n";
  }
}

void insUpdateCovariance(double q_gyro, double q_accel, double q_gyro_bias, double q_accel_bias, const RBIS & state,
    RBIM & cov, double dt)
{
  RBIM Ac;
  getIMUProcessLinearizationContinuous(state, Ac);

  //input matrix (ins has 12 inputs with biases)
  int gyro_ind = 0;
  int accelerometer_ind = 3;
  int gyro_bias_noise_ind = 6;
  int accelerometer_bias_noise_ind = 9;
  const int num_inputs = 12;

  //ins input only maps to vb and chi states
  Matrix<double, RBIS::rbis_num_states, num_inputs> Wc = Matrix<double, RBIS::rbis_num_states, num_inputs>::Zero();
  //Wc_vb
  Wc.block<3, 3>(RBIS::velocity_ind, gyro_ind) = skewHat(state.velocity());
  Wc.block<3, 3>(RBIS::velocity_ind, accelerometer_ind) = Matrix3d::Identity();

  //Wc_chi
  Wc.block<3, 3>(RBIS::chi_ind, gyro_ind) = Matrix3d::Identity();

  Wc.block<3, 3>(RBIS::gyro_bias_ind, gyro_bias_noise_ind) = Matrix3d::Identity();
  Wc.block<3, 3>(RBIS::accel_bias_ind, accelerometer_bias_noise_ind) = Matrix3d::Identity();

  Vector12d Qc_vec = Vector12d::Zero();
  Qc_vec.segment<3>(gyro_ind) = Vector3d::Ones() * q_gyro;
  Qc_vec.segment<3>(accelerometer_ind) = Vector3d::Ones() * q_accel;

  Qc_vec.segment<3>(gyro_bias_noise_ind) = Vector3d::Ones() * q_gyro_bias;
  Qc_vec.segment<3>(accelerometer_bias_noise_ind) = Vector3d::Ones() * q_accel_bias;

  Matrix12d Qc = Qc_vec.asDiagonal();

  //make discrete matrices based on approximation (16.322 notes P. 119)
  RBIM Ad, Qd;
  Ad = RBIM::Identity();
  Ad.noalias() += Ac * dt;

  Qd.noalias() = Wc * Qc * Wc.transpose() * dt;

  cov = Ad * cov * Ad.transpose() + Qd; //TODO: Can eigen handle me better?

  cov.block<3, 3>(RBIS::acceleration_ind, RBIS::acceleration_ind).noalias() = q_accel * Matrix3d::Identity();
  cov.block<3, 3>(RBIS::angular_velocity_ind, RBIS::angular_velocity_ind).noalias() = q_gyro * Matrix3d::Identity();
}

double matrixMeasurementGetKandCovDelta(const Eigen::MatrixXd & R, const Eigen::MatrixXd & C, const RBIM & cov,
    const Eigen::VectorXd & z_resid, RBIM & cov_delta, Eigen::MatrixXd & K)
{
  int m = R.rows();

  MatrixXd S(m, m);
//  eigen_dump(R);
//  eigen_dump(S);
//  eigen_dump(C);
//  eigen_dump(cov);
  S = R;
  S.noalias() += C * cov * C.transpose();

  LDLT<MatrixXd> Sldlt = LDLT<MatrixXd>(S);
  //  K = self->Sigma * H.transpose() * S.inverse();
  K.transpose() = Sldlt.solve(C * cov);
  cov_delta.noalias() = K * C * cov;

  return -log(S.determinant()) - z_resid.transpose() * Sldlt.solve(z_resid);
}

double matrixMeasurement(const Eigen::VectorXd & z, const Eigen::VectorXd & z_pred, const Eigen::MatrixXd & R,
    const Eigen::MatrixXd & C, const RBIS & state, const RBIM & cov, RBIS & dstate, RBIM & dcov)
{
  int m = z.rows();

  MatrixXd K = MatrixXd::Zero(RBIS::rbis_num_states, m);

  VectorXd z_resid = z - z_pred;

  double loglikelihood = matrixMeasurementGetKandCovDelta(R, C, cov, z_resid, dcov, K);

  dstate = RBIS(K * z_resid);
  return loglikelihood;
}

double indexedMeasurement(const Eigen::VectorXd & z, const Eigen::MatrixXd & R, const Eigen::VectorXi & z_indices,
    const RBIS & state, const RBIM & cov, RBIS & dstate, RBIM & dcov)
{
  int m = z_indices.rows();
  VectorXd z_resid(m);

  MatrixXd K = MatrixXd::Zero(RBIS::rbis_num_states, m);
  MatrixXd C = MatrixXd::Zero(m, RBIS::rbis_num_states);

  for (int ii = 0; ii < m; ii++) {
    z_resid(ii) = z(ii) - state.vec(z_indices(ii));
    C(ii, z_indices(ii)) = 1;
  }

  double loglikelihood = matrixMeasurementGetKandCovDelta(R, C, cov, z_resid, dcov, K);
  dstate = RBIS(K * z_resid);
  return loglikelihood;

}

/**
 * Residuals for any orientation error states in z (designated
 * by entries in z_indices) are computed directly as components
 * of the exponential representation of the error quaternion -
 * the corresponding values in z are *ignored*.
 *
 * Use this function only if you're certain you know what you're
 * doing.
 */
double indexedPlusOrientationMeasurement(const Eigen::VectorXd & z, const Eigen::Quaterniond & quat,
    const Eigen::MatrixXd & R, const Eigen::VectorXi & z_indices, const RBIS & state, const RBIM & cov, RBIS & dstate,
    RBIM & dcov)
{
  int m = z_indices.rows();
  VectorXd z_resid(m);

  MatrixXd K = MatrixXd::Zero(RBIS::rbis_num_states, m);
  MatrixXd C = MatrixXd::Zero(m, RBIS::rbis_num_states);

  Vector3d dquat = subtractQuats(quat, state.quat);

  // Assumes indices for orientation error state are consecutive.
  for (int ii = 0; ii < m; ii++) {
    if (z_indices(ii) >= RBIS::chi_ind && z_indices(ii) <= RBIS::chi_ind + 2) {
      z_resid(ii) = dquat(z_indices(ii) - RBIS::chi_ind);
    }
    else {
      z_resid(ii) = z(ii) - state.vec(z_indices(ii));
    }

    C(ii, z_indices(ii)) = 1;
  }

  double loglikelihood = matrixMeasurementGetKandCovDelta(R, C, cov, z_resid, dcov, K);
  dstate = RBIS(K * z_resid);

  return loglikelihood; //TODO get this right
}

void rbisApplyDelta(const RBIS & prior_state, const RBIM & prior_cov, const RBIS & dstate, const RBIM & dcov,
    RBIS & posterior_state, RBIM & posterior_cov)
{
  posterior_state = prior_state;
  posterior_cov = prior_cov;

  posterior_state.addState(dstate);
  posterior_cov -= dcov;
}

/**
 * Kalman smoother implemented using equations 15.82, 15.83 from
 * Michael Jordan's Graphical Models book (chapter 15 Kalman
 * Filtering)
 */
void ekfSmoothingStep(const RBIS & next_state_pred, const RBIM & next_cov_pred, const RBIS & next_state,
    const RBIM & next_cov, double dt, RBIS & cur_state, RBIM & cur_cov)
{
  RBIM Ac;
  getIMUProcessLinearizationContinuous(cur_state, Ac);
  RBIM Ad = RBIM::Identity();
  Ad.noalias() += Ac * dt;

  //the cov_pred calculation fails if there is 0 uncertainty in
  //the biases - this fix just gives it an identity matrix so it's invertible
  RBIM next_cov_pred_corrected = next_cov_pred;

  if ((next_cov_pred.block<3, 3>(RBIS::gyro_bias_ind, RBIS::gyro_bias_ind).diagonal().array() < .00000000001).any()) {
    next_cov_pred_corrected.block<3, 3>(RBIS::gyro_bias_ind, RBIS::gyro_bias_ind) = Matrix3d::Identity();
  }
  if ((next_cov_pred.block<3, 3>(RBIS::accel_bias_ind, RBIS::accel_bias_ind).diagonal().array() < .00000000001).any()) {
    next_cov_pred_corrected.block<3, 3>(RBIS::accel_bias_ind, RBIS::accel_bias_ind) = Matrix3d::Identity();
  }

  //RBIM L = cur_cov*Ad.transpose()/(next_cov_pred + Sigma_x_pred_pd_corr);
  RBIM L;
  L.transpose() = next_cov_pred_corrected.ldlt().solve(Ad * cur_cov);

  cur_cov = cur_cov + L * (next_cov - next_cov_pred) * L.transpose();

  RBIS smooth_resid = next_state;
  smooth_resid.subtractState(next_state_pred);
  smooth_resid.quatToChi();

  RBIS smooth_innov = RBIS(L * smooth_resid.vec);

  cur_state.addState(smooth_innov);
}

pronto_filter_state_t * rbisCreateFilterStateMessage(const RBIS & state, const RBIM & cov)
{
  pronto_filter_state_t * msg = (pronto_filter_state_t *) malloc(sizeof(pronto_filter_state_t));

  msg->utime = state.utime;

  msg->num_states = RBIS::rbis_num_states;
  msg->num_cov_elements = msg->num_states * msg->num_states;
  quaternionToBotDouble(msg->quat, state.quat);

  msg->state = (double *) malloc(msg->num_states * sizeof(double));
  msg->cov = (double *) malloc(msg->num_cov_elements * sizeof(double));

  Map<RBIS::VectorNd>(msg->state) = state.vec;
  Map<RBIM>(msg->cov) = cov;

  return msg;
}

pronto::filter_state_t rbisCreateFilterStateMessageCPP(const RBIS & state, const RBIM & cov)
{
  pronto::filter_state_t msg;
  msg.utime = state.utime;

  msg.num_states = RBIS::rbis_num_states;
  msg.num_cov_elements = msg.num_states * msg.num_states;
  quaternionToBotDouble(msg.quat, state.quat);

  msg.state = vector<double>(msg.num_states);
  msg.cov = vector<double>(msg.num_cov_elements);

  Map<RBIS::VectorNd>(&msg.state[0]) = state.vec;
  Map<RBIM>(&msg.cov[0]) = cov;

  return msg;

}
}
