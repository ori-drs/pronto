#include "pronto_estimator_lcm/gps_handler.hpp"

namespace MavStateEst {

GpsHandler::GpsHandler(BotParam * _param)
{
  double r_gps_xy = bot_param_get_double_or_fail(_param, "state_estimator.gps.r_xy");
  double r_gps_z = bot_param_get_double_or_fail(_param, "state_estimator.gps.r_z");
  Eigen::Vector3d R_gps_diagonal = Eigen::Array3d(r_gps_xy, r_gps_xy, r_gps_z).pow(2);
  cov_xyz = R_gps_diagonal.asDiagonal();
}

RBISUpdateInterface * GpsHandler::processMessage(const bot_core::gps_data_t * msg, MavStateEstimator* state_estimator)
{
  if (msg->gps_lock < 3)
    return NULL;

  return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
      Eigen::Map<const Eigen::Vector3d>(msg->xyz_pos), cov_xyz, RBISUpdateInterface::gps, msg->utime);
}

bool GpsHandler::processMessageInit(const bot_core::gps_data_t * msg,
    const std::map<std::string, bool> & sensors_initialized
    ,const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov)
{
  init_state.utime = msg->utime;
  RBISIndexedMeasurement * update = dynamic_cast<RBISIndexedMeasurement *>(GpsHandler::processMessage(msg, NULL));

  if(update == NULL) {
    return false;
  }

  init_state.position() = update->measurement;
  init_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = update->measurement_cov;

  fprintf(stderr, "initialized position using GPS at xyz: %f,%f,%f\n", init_state.position()(0),
      init_state.position()(1), init_state.position()(2));

  delete update;
  return true;
}
} // namespace MavStateEst
