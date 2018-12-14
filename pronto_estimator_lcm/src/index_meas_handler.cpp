#include "pronto_estimator_lcm/index_meas_handler.hpp"

namespace MavStateEst {

IndexedMeasurementHandler::IndexedMeasurementHandler(const RBISUpdateInterface::sensor_enum &this_sensor) :
    index_module_(this_sensor)
{

}

RBISUpdateInterface * IndexedMeasurementHandler::processMessage(const pronto::indexed_measurement_t * msg, MavStateEstimator* state_estimator)
{
    indexMeasurementFromLCM(*msg, index_msg_);
    return index_module_.processMessage(&index_msg_, state_estimator);
}

bool IndexedMeasurementHandler::processMessageInit(const pronto::indexed_measurement_t * msg,
                                                   const std::map<std::string, bool> & sensors_initialized,
                                                   const RBIS & default_state,
                                                   const RBIM & default_cov,
                                                   RBIS & init_state,
                                                   RBIM & init_cov)
{
    indexMeasurementFromLCM(*msg, index_msg_);
    return index_module_.processMessageInit(&index_msg_,
                                            sensors_initialized,
                                            default_state,
                                            default_cov,
                                            init_state,
                                            init_cov);
}

void IndexedMeasurementHandler::indexMeasurementFromLCM(const pronto::indexed_measurement_t &lcm_msg,
                                                        IndexedMeasurement &msg)
{
    msg.R_effective = Eigen::Map<const Eigen::MatrixXd>(&lcm_msg.R_effective[0],
                                                        lcm_msg.measured_dim,
                                                        lcm_msg.measured_dim);
    msg.state_utime = lcm_msg.state_utime;
    msg.utime = lcm_msg.utime;
    msg.z_effective = Eigen::Map<const Eigen::VectorXd>(&lcm_msg.z_effective[0],
                                                         lcm_msg.measured_dim);
    msg.z_indices = Eigen::Map<const Eigen::VectorXi>(&lcm_msg.z_indices[0],
                                                       lcm_msg.measured_dim);
}

} // namespace MavStateEst
