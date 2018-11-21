#pragma once
#include "mav_state_est/rbis_update_interface.hpp"
#include <lcmtypes/pronto/indexed_measurement_t.hpp>
#include "mav_state_est/mav_state_est.hpp"

namespace MavStateEst {

class IndexedMeasurementHandler {
public:
  IndexedMeasurementHandler(RBISUpdateInterface::sensor_enum this_sensor)
  {
      indexed_sensor = this_sensor;
  }
  RBISUpdateInterface * processMessage(const pronto::indexed_measurement_t * msg,
                                       MavStateEstimator* state_estimator);

  bool processMessageInit(const pronto::indexed_measurement_t * msg,
                          const std::map<std::string, bool> & sensors_initialized,
                          const RBIS & default_state,
                          const RBIM & default_cov,
                          RBIS & init_state,
                          RBIM & init_cov);

private:
    RBISUpdateInterface::sensor_enum indexed_sensor;

};


} // namespace MavStateEst
