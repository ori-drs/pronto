#include "pronto_estimator_lcm/init_message_handler.hpp"
#include <Eigen/Dense>

using namespace Eigen;

namespace MavStateEst {

bool InitMessageHandler::processMessageInit(const pronto::filter_state_t * msg,
                                            const std::map<std::string, bool> & sensors_initialized,
                                            const RBIS & default_state,
                                            const RBIM & default_cov,
                                            RBIS & init_state,
                                            RBIM & init_cov)
{
    Eigen::Quaterniond init_quat;
    eigen_utils::botDoubleToQuaternion(init_quat, msg->quat);
    init_state = RBIS(Eigen::Map<const Eigen::VectorXd>(&msg->state[0], msg->num_states),
                      init_quat);
    init_state.utime = msg->utime;
    Eigen::Map<const MatrixXd> cov_map(&msg->cov[0], msg->num_states, msg->num_states);
    init_cov = cov_map;
    fprintf(stderr, "Initialized using message\n");
    return true;
}

/**
 * When subscribed as a normal sensor, the reset message will reset the state estimator
 * on the fly.
 */
RBISUpdateInterface * InitMessageHandler::processMessage(const pronto::filter_state_t * msg,
                                                         MavStateEstimator* state_estimator)
{
  Eigen::Map<const MatrixXd> cov_map(&msg->cov[0], msg->num_states, msg->num_states);
  Eigen::Quaterniond quat;
  eigen_utils::botDoubleToQuaternion(quat, msg->quat);
  RBIS state = RBIS(Eigen::Map<const Eigen::VectorXd>(&msg->state[0], msg->num_states),
                    quat);
  state.utime = msg->utime;

  return new RBISResetUpdate(state, cov_map, RBISUpdateInterface::init_message, msg->utime);

}
}

