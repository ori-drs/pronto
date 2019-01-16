#pragma once
#include <pronto_core/rbis_update_interface.hpp>
#include <lcmtypes/pronto/indexed_measurement_t.hpp>
#include <pronto_core/mav_state_est.hpp>
#include <pronto_core/sensing_module.hpp>
#include <pronto_core/definitions.hpp>
#include <pronto_core/indexed_meas_module.hpp>
namespace MavStateEst {

class IndexedMeasurementHandler : SensingModule<pronto::indexed_measurement_t>
{
public:
  IndexedMeasurementHandler(const RBISUpdateInterface::sensor_enum& this_sensor);

  RBISUpdateInterface * processMessage(const pronto::indexed_measurement_t * msg,
                                       MavStateEstimator* state_estimator);

  bool processMessageInit(const pronto::indexed_measurement_t * msg,
                          const std::map<std::string, bool> & sensors_initialized,
                          const RBIS & default_state,
                          const RBIM & default_cov,
                          RBIS & init_state,
                          RBIM & init_cov);

private:
    IndexedMeasurementModule index_module_;
    IndexedMeasurement index_msg_;
};


} // namespace MavStateEst
