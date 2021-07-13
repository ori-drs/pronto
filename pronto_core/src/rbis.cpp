#include "pronto_core/rbis.hpp"
#include "pronto_core/rotations.hpp"

using namespace pronto::rotation;
using namespace Eigen;
using namespace std;

namespace pronto {

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

#define DEBUG_MODE 0

void insUpdateState(const Eigen::Vector3d & gyro,
                    const Eigen::Vector3d & accelerometer,
                    double dt,
                    RBIS & state)
{
#if DEBUG_MODE
        std::cout << "mfallon insUpdateState\n";
        std::cout << state << " state prior\n";
        std::cout << gyro.transpose() << "\n";
        std::cout << accelerometer.transpose() << "\n";
        std::cout << dt << "\n";

        static uint64_t utime;
        std::cerr << "INS prior pos: " << state.position().transpose()        << std::endl;
        std::cerr << "INS prior vel: " << state.velocity().transpose()        << std::endl;
        std::cerr << "INS state gyro bias: " << state.gyroBias().transpose()  << std::endl;
        std::cerr << "INS state accel bias: " << state.accelBias().transpose()<< std::endl;
        double ee = (((((double)(state.utime - utime))*1e-6) / dt) -1.0)*100;
        std::cerr << "True dt: " << state.utime - utime  << " ("<< ee << "%)"<<std::endl;
        if(std::abs(ee) > 0.25){
          std::cerr << "========================" << std::endl;
          std::cerr << std::endl << std::endl;
        }
        utime = state.utime;
#endif

  //put the ins measurements into the state, correcting for bias
  state.angularVelocity() = gyro - state.gyroBias();
  state.acceleration() = accelerometer - state.accelBias();

#if DEBUG_MODE
  std::cerr << "state.acceleration() = [" << accelerometer.transpose() << "]' - [" << state.accelBias().transpose() << "]'" << std::endl;
  std::cerr << "state.angularVelocity() = [" << gyro.transpose() << "]' - " << state.gyroBias().transpose() << "]'" << std::endl;
#endif

  //compute derivatives
  RBIS dstate; //initialize everything to 0

  dstate.velocity() = -state.angularVelocity().cross(state.velocity());
  dstate.velocity().noalias() += state.quat.inverse() * g_vec + state.acceleration();
#if DEBUG_MODE
  std::cerr << "dstate.velocity() = [" << -state.angularVelocity().transpose() << "]' x [" << state.velocity().transpose()<< "]'" << std::endl;
  std::cerr << "dstate.velocity() += [" << (state.quat.inverse() * g_vec).transpose() << "]' + [" << state.acceleration().transpose()<< "]'" << std::endl;
#endif

  dstate.chi() = state.angularVelocity();
  dstate.position().noalias() = state.quat * state.velocity();
  // add second order?
  // dstate.position().noalias() += + 0.5 * state.quat.inverse() * g_vec * dt + 0.5 * state.quat * state.acceleration() * dt;

  //integrate
  dstate.vec *= dt;
  dstate.chiToQuat();
  
#if DEBUG_MODE
      std::cerr << "INS delta pos: " << dstate.position().transpose() << std::endl;
      std::cerr << "INS delta vel: " << dstate.velocity().transpose() << std::endl;

      std::cout << dstate << " dstate\n";
#endif

  state.addState(dstate);

#if DEBUG_MODE
      std::cerr << "INS posterior pos: " << state.position().transpose() << std::endl;
      std::cerr << "INS posterior vel: " << state.velocity().transpose() << std::endl;

      std::cout << dstate << " state updated\n";
      std::cout << "\n";
#endif
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

double matrixMeasurementGetKandCovDelta(const Eigen::MatrixXd & R,
                                        const Eigen::MatrixXd & C,
                                        const RBIM & cov,
                                        const Eigen::VectorXd & z_resid,
                                        RBIM & cov_delta,
                                        Eigen::MatrixXd & K)
{
  int m = R.rows();

  MatrixXd S(m, m);
  S = R;
  S.noalias() += C * cov * C.transpose();

  LDLT<MatrixXd> Sldlt = LDLT<MatrixXd>(S);
  //  K = self->Sigma * H.transpose() * S.inverse();
  K.transpose() = Sldlt.solve(C * cov);
  cov_delta.noalias() = K * C * cov;

  return -log(S.determinant()) - z_resid.transpose() * Sldlt.solve(z_resid);
}

double matrixMeasurement(const Eigen::VectorXd & z,
                         const Eigen::VectorXd & z_pred,
                         const Eigen::MatrixXd & R,
                         const Eigen::MatrixXd & C,
                         const RBIS & state,
                         const RBIM & cov,
                         RBIS & dstate,
                         RBIM & dcov)
{
  int m = z.rows();

  MatrixXd K = MatrixXd::Zero(RBIS::rbis_num_states, m);

  VectorXd z_resid = z - z_pred;

  double loglikelihood = matrixMeasurementGetKandCovDelta(R, C, cov, z_resid, dcov, K);

  dstate = RBIS(K * z_resid);
  return loglikelihood;
}

double indexedMeasurement(const Eigen::VectorXd & z,
                          const Eigen::MatrixXd & R,
                          const Eigen::VectorXi & z_indices,
                          const RBIS & state,
                          const RBIM & cov,
                          RBIS & dstate,
                          RBIM & dcov)
{
  int m = z_indices.rows();
  VectorXd z_resid(m);

  MatrixXd K = MatrixXd::Zero(RBIS::rbis_num_states, m);
  MatrixXd C = MatrixXd::Zero(m, RBIS::rbis_num_states);

  for (int ii = 0; ii < m; ii++) {
    z_resid(ii) = z(ii) - state.vec(z_indices(ii));
    C(ii, z_indices(ii)) = 1;
#if DEBUG_MODE
    std::cout << "prediction: [" << ii << "] " << state.vec(z_indices(ii)) << std::endl;
    std::cout << "update:     [" << ii << "] " << z(ii) << std::endl;
    std::cout << "residual:   [" << ii << "] " << z_resid(ii) << std::endl;
#endif
  }

  double loglikelihood = matrixMeasurementGetKandCovDelta(R, C, cov, z_resid, dcov, K);
  dstate = RBIS(K * z_resid);
#if DEBUG_MODE
  std::cout << "Kalman gain: " << K.rows()*K.cols() << std::endl;
#endif

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
double indexedPlusOrientationMeasurement(const Eigen::VectorXd & z,
                                         const Eigen::Quaterniond & quat,
                                         const Eigen::MatrixXd & R,
                                         const Eigen::VectorXi & z_indices,
                                         const RBIS & state,
                                         const RBIM & cov,
                                         RBIS & dstate,
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
#if DEBUG_MODE
    Eigen::IOFormat mformat(5, 0, ", ", "\n", "[", "]");
    std::cerr << "Indices: " << z_indices.transpose().format(mformat) << std::endl;

    std::cerr << "Measurement: " << z.transpose().format(mformat) << std::endl;

    std::cerr << "Matrix K:" << std::endl;
    std::cerr << K.format(mformat) << std::endl;

    std::cerr << "Matrix C:" << std::endl;
    std::cerr << C.format(mformat) << std::endl;

    std::cerr << "Vector residual z_resid:" << std::endl;
    std::cerr << z_resid.transpose().format(mformat) << std::endl;

    std::cerr << "Vector dstate K * z_resid:" << std::endl;
    std::cerr << (K*z_resid).transpose().format(mformat) << std::endl;
#endif

  return loglikelihood; //TODO get this right
}

void rbisApplyDelta(const RBIS & prior_state,
                    const RBIM & prior_cov,
                    const RBIS & dstate,
                    const RBIM & dcov,
                    RBIS & posterior_state,
                    RBIM & posterior_cov)
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
void ekfSmoothingStep(const RBIS & next_state_pred,
                      const RBIM & next_cov_pred,
                      const RBIS & next_state,
                      const RBIM & next_cov,
                      double dt,
                      RBIS & cur_state,
                      RBIM & cur_cov)
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
}
