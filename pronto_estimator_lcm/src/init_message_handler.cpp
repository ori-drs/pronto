#include "pronto_estimator_lcm/init_message_handler.hpp"
#include <Eigen/Dense>
#include <pronto_conversions/pronto_meas_lcm.hpp>

using namespace Eigen;

namespace MavStateEst {

bool InitMessageHandler::processMessageInit(const pronto::filter_state_t * msg,
                                            const std::map<std::string, bool> & sensors_initialized,
                                            const RBIS & default_state,
                                            const RBIM & default_cov,
                                            RBIS & init_state,
                                            RBIM & init_cov)
{
    filterStateFromLCM(*msg, init_msg_);
    return init_module_.processMessageInit(&init_msg_,
                                           sensors_initialized,
                                           default_state,
                                           default_cov,
                                           init_state,
                                           init_cov);
}

/**
 * When subscribed as a normal sensor, the reset message will reset the state estimator
 * on the fly.
 */
RBISUpdateInterface * InitMessageHandler::processMessage(const pronto::filter_state_t * msg,
                                                         MavStateEstimator* state_estimator)
{
  filterStateFromLCM(*msg, init_msg_);
  return init_module_.processMessage(&init_msg_, state_estimator);
}

} // namespace MavStateEst

