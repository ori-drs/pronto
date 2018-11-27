#pragma once
#include "mav_state_est/rbis_update_interface.hpp"
#include "mav_state_est/mav_state_est.hpp"
#include <lcmtypes/pronto/filter_state_t.hpp>
#include <map>

namespace MavStateEst {

class InitMessageHandler {
public:
  bool processMessageInit(const pronto::filter_state_t * msg,
                          const std::map<std::string, bool> & sensors_initialized,
                          const RBIS & default_state,
                          const RBIM & default_cov,
                          RBIS & init_state, RBIM & init_cov);

  RBISUpdateInterface * processMessage(const pronto::filter_state_t * msg,
                                       MavStateEstimator* state_estimator);
};
}
