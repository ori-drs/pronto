#include "mav_state_est/index_meas_handler.hpp"

namespace MavStateEst {

RBISUpdateInterface * IndexedMeasurementHandler::processMessage(const pronto::indexed_measurement_t * msg, MavStateEstimator* state_estimator)
{
  return new RBISIndexedMeasurement(Eigen::Map<const Eigen::VectorXi>(&msg->z_indices[0], msg->measured_dim),
      Eigen::Map<const Eigen::VectorXd>(&msg->z_effective[0], msg->measured_dim),
      Eigen::Map<const Eigen::MatrixXd>(&msg->R_effective[0], msg->measured_dim, msg->measured_dim),
      indexed_sensor, msg->utime);
}

bool IndexedMeasurementHandler::processMessageInit(const pronto::indexed_measurement_t * msg,
    const std::map<std::string, bool> & sensors_initialized
    , const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov)
{

  RBISIndexedMeasurement * update = (RBISIndexedMeasurement *) processMessage(msg, NULL);

  if (update == NULL) {
    return false;
  }

  int m = update->index.rows();
  for (int ii = 0; ii < m; ii++) {
    for (int jj = 0; jj < m; jj++) {
      init_cov(update->index(ii), update->index(jj)) = update->measurement_cov(ii, jj);
    }
    init_state.vec(update->index(ii)) = update->measurement(ii);
  }
  init_state.chiToQuat();

  std::cout << "Initializing indices:\n" << update->index << "\n with:\n" << update->measurement << "\nand cov:\n"
      << update->measurement_cov << "\n";
  delete update;

  return true;
}

} // namespace MavStateEst
