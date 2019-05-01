#pragma once
#include <pronto_core/rbis_update_interface.hpp>
#include <pronto_core/mav_state_est.hpp>
#include <pronto_core/init_message_module.hpp>
#include <pronto_msgs/FilterState.h>
#include <map>

namespace MavStateEst {

class InitMessageHandlerROS : public SensingModule<pronto_msgs::FilterState>{
public:
  bool processMessageInit(const pronto_msgs::FilterState * msg,
                          const std::map<std::string, bool> & sensors_initialized,
                          const RBIS & default_state,
                          const RBIM & default_cov,
                          RBIS & init_state, RBIM & init_cov);

  RBISUpdateInterface * processMessage(const pronto_msgs::FilterState * msg,
                                       MavStateEstimator* state_estimator);
protected:
  InitMessageModule init_module_;
  FilterState init_msg_;
};
}
