#pragma once
#include "mav_state_est/rbis_update_interface.hpp"
#include "mav_state_est/mav_state_est.hpp"

namespace MavStateEst {
template <class MessageT>
class SensingModule {
public:
    virtual RBISUpdateInterface* processMessage(const MessageT* msg,
                                        MavStateEstimator* est = NULL) = 0;

    virtual bool processMessageInit(const MessageT* msg,
                                    const std::map<std::string,bool>& sensor_initialized,
                                    const RBIS & default_state,
                                    const RBIM & default_cov,
                                    RBIS & init_state,
                                    RBIM & init_cov) = 0;

};
}
