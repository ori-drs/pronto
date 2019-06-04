#include "pronto_ros/init_message_ros_handler.hpp"
#include "pronto_ros/pronto_ros_conversions.hpp"
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <string>

using namespace Eigen;

namespace pronto {

bool InitMessageHandlerROS::processMessageInit(const pronto_msgs::FilterState * msg,
                                            const std::map<std::string, bool> & sensors_initialized,
                                            const RBIS & default_state,
                                            const RBIM & default_cov,
                                            RBIS & init_state,
                                            RBIM & init_cov)
{
    filterStateFromROS(*msg, init_msg_);
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
RBISUpdateInterface * InitMessageHandlerROS::processMessage(const pronto_msgs::FilterState * msg,
                                                         StateEstimator* state_estimator)
{
  filterStateFromROS(*msg, init_msg_);
  return init_module_.processMessage(&init_msg_, state_estimator);
}


}

