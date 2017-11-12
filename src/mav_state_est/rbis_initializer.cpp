#include "mav_state_est/rbis_initializer.hpp"

using namespace std;
using namespace Eigen;

namespace MavStateEst {

RBISInitializer::RBISInitializer(LCMFrontEnd * _lcm_front, const RBIS & def_state, const RBIM & def_cov) :
    default_state(def_state), default_cov(def_cov), lcm_front(_lcm_front)
{

  initializer_created = true;
  initialized = false;

  init_cov = -RBIM::Identity();
  init_state = RBIS();

  char** init_sensor_names = bot_param_get_str_array_alloc(lcm_front->param, "state_estimator.init_sensors");
  if (init_sensor_names == NULL) {
    fprintf(stderr, "Error: must specify init sensors using key state_estimator.init_sensors\n");
    exit(1);
  }
  else {
    for (int i = 0; init_sensor_names[i]; i++) {
      std::string sensor_str = std::string(init_sensor_names[i]);
      sensors_initialized[sensor_str] = false;
    }
  }
  bot_param_str_array_free(init_sensor_names);

  char * init_msg_chan = bot_param_get_str_or_fail(lcm_front->param, "state_estimator.init_message.channel");
  init_message_channel = std::string(init_msg_chan);
  free(init_msg_chan);

}

RBISInitializer::~RBISInitializer()
{
  std::map<std::string, SensorHandlerInterface *>::iterator it;
  for (it = sensor_handlers.begin(); it != sensor_handlers.end(); it++) {
    delete it->second;
  }
}

RBIS RBISInitializer::getDefaultState(BotParam * param)
{
  RBIS def_state;

  double vec[3];

  bot_param_get_double_array_or_fail(param, "state_estimator.x0.velocity", vec, 3);
  def_state.velocity() = Eigen::Map<Eigen::Vector3d>(vec);

  bot_param_get_double_array_or_fail(param, "state_estimator.x0.position", vec, 3);
  def_state.position() = Eigen::Map<Eigen::Vector3d>(vec);

  bot_param_get_double_array_or_fail(param, "state_estimator.x0.angular_velocity", vec, 3);
  def_state.angularVelocity() = Eigen::Map<Eigen::Vector3d>(vec);

  bot_param_get_double_array_or_fail(param, "state_estimator.x0.rpy", vec, 3);
  def_state.orientation() = eigen_utils::setQuatEulerAngles(Eigen::Map<Eigen::Vector3d>(vec));

  return def_state;
}

RBIM RBISInitializer::getDefaultCov(BotParam * param)
{
  RBIM def_cov = RBIM::Zero();

  double sigma_Delta_xy_init = bot_param_get_double_or_fail(param, "state_estimator.sigma0.Delta_xy");
  double sigma_Delta_z_init = bot_param_get_double_or_fail(param, "state_estimator.sigma0.Delta_z");
  double sigma_chi_xy_init = bot_to_radians(bot_param_get_double_or_fail(param, "state_estimator.sigma0.chi_xy"));
  double sigma_chi_z_init = bot_to_radians(bot_param_get_double_or_fail(param, "state_estimator.sigma0.chi_z"));
  double sigma_vb_init = bot_param_get_double_or_fail(param, "state_estimator.sigma0.vb");
  double sigma_gyro_bias_init =
      bot_to_radians(bot_param_get_double_or_fail(param, "state_estimator.sigma0.gyro_bias"));
  double sigma_accelerometer_bias_init = bot_param_get_double_or_fail(param, "state_estimator.sigma0.accel_bias");

  Eigen::Vector3d xyz_cov_diag =
      Eigen::Array3d(sigma_Delta_xy_init, sigma_Delta_xy_init, sigma_Delta_z_init).square();

  Eigen::Vector3d init_chi_cov_diag = Eigen::Array3d(sigma_chi_xy_init, sigma_chi_xy_init, sigma_chi_z_init).square();

  //set all the sub-blocks of the covariance matrix
  def_cov.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind) = bot_sq(sigma_vb_init) * Eigen::Matrix3d::Identity();
  def_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = init_chi_cov_diag.asDiagonal();
  def_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = xyz_cov_diag.asDiagonal();
  def_cov.block<3, 3>(RBIS::gyro_bias_ind, RBIS::gyro_bias_ind) = Eigen::Matrix3d::Identity()
      * bot_sq(sigma_gyro_bias_init);
  def_cov.block<3, 3>(RBIS::accel_bias_ind, RBIS::accel_bias_ind) = Eigen::Matrix3d::Identity()
      * bot_sq(sigma_accelerometer_bias_init);

  return def_cov;
}

bool RBISInitializer::allInitializedExcept(const std::map<std::string, bool> & _sensors_initialized
    , const std::string & sensor_prefix)
{
  std::map<std::string, bool>::const_iterator it;
  for (it = _sensors_initialized.begin(); it != _sensors_initialized.end(); it++) {
    if (it->first == sensor_prefix)
      continue;
    if (!it->second)
      return false;
  }
  return true;
}

bool RBISInitializer::initializingWith(const std::map<std::string, bool> & _sensors_initialized
    ,const std::string & sensor_prefix)
{
  return _sensors_initialized.count(sensor_prefix) > 0;
}

bool RBISInitializer::initializingWith(const std::string & sensor_prefix)
{
  return initializingWith(sensors_initialized, sensor_prefix);
}

void RBISInitializer::updateInitialization(int64_t utime)
{
  if (allInitializedExcept(sensors_initialized, "")) {

    for (int ii = 0; ii < RBIS::rbis_num_states; ii++) {
      if (init_cov(ii, ii) < 0) {
        init_cov(ii, ii) = default_cov(ii, ii);
        init_state.vec(ii) = default_state.vec(ii);
      }
    }

    //premultiply so default is in global frame
    Eigen::Quaterniond init_quat = init_state.orientation();
    init_state.chiToQuat();
    init_state.orientation() = default_state.orientation() * init_quat;
    init_state.utime = utime;
    initialized = true;
  }
}

void RBISInitializer::initialize(RBIS & state, RBIM & cov)
{
  std::map<std::string, bool>::iterator it;
  for (it = sensors_initialized.begin(); it != sensors_initialized.end(); it++) {
    if (sensor_handlers.count(it->first) < 1) {
      fprintf(stderr, "Error: expected to be initializing with sensor %s, but never added\n", it->first.c_str());
      exit(1);
    }
  }

  while (!initialized) {
    lcm_front->lcm_recv->handle();
  }

  state = init_state;
  cov = init_cov;

  pronto::filter_state_t fs_msg = rbisCreateFilterStateMessageCPP(state, cov);
  lcm_front->lcm_pub->publish(init_message_channel, &fs_msg);

}

bool InitMessageHandler::processMessageInit(const pronto::filter_state_t * msg,
      const std::map<std::string, bool> & sensors_initialized, const RBIS & default_state,
      const RBIM & default_cov, RBIS & init_state, RBIM & init_cov)
{
  init_state = RBIS(*msg);
  Eigen::Map<const MatrixXd> cov_map(&msg->cov[0], msg->num_states, msg->num_states);
  init_cov = cov_map;
  fprintf(stderr, "Initialized using message\n");
  return true;
}

/**
 * When subscribed as a normal sensor, the reset message will reset the state estimator
 * on the fly.
 */
RBISUpdateInterface * InitMessageHandler::processMessage(const pronto::filter_state_t * msg, MavStateEstimator* state_estimator)
{
  Eigen::Map<const MatrixXd> cov_map(&msg->cov[0], msg->num_states, msg->num_states);
  RBIM init_cov = cov_map;

  return new RBISResetUpdate(RBIS(*msg), cov_map, RBISUpdateInterface::init_message, msg->utime);

}


}

