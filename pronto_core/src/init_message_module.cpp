#include "pronto_core/init_message_module.hpp"
#include <iostream>

namespace pronto {

RBISUpdateInterface* InitMessageModule::processMessage(const FilterState *msg,
                                                       StateEstimator *est)
{
    RBIS state = RBIS(msg->state, msg->quat);
    state.utime = msg->utime;

    return new RBISResetUpdate(state,
                               msg->cov,
                               RBISUpdateInterface::init_message,
                               msg->utime);
}

bool InitMessageModule::processMessageInit(const FilterState *msg,
                                           const std::map<std::string, bool> &sensor_initialized,
                                           const RBIS &default_state,
                                           const RBIM &default_cov,
                                           RBIS &init_state,
                                           RBIM &init_cov)
{
    init_state = RBIS(msg->state, msg->quat);
    init_state.utime = msg->utime;
    init_cov = msg->cov;
    std::cout << "Initialized using message" << std::endl;
    return true;
}

} // namespace pronto

