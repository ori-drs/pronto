#include "pronto_estimator_lcm/index_meas_handler.hpp"
#include <pronto_conversions/pronto_meas_lcm.hpp>

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



} // namespace MavStateEst
