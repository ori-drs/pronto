#include "pronto_ros/init_message_ros_handler.hpp"
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <string>

using namespace Eigen;

namespace MavStateEst {

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
                                                         MavStateEstimator* state_estimator)
{
  filterStateFromROS(*msg, init_msg_);
  return init_module_.processMessage(&init_msg_, state_estimator);
}

void InitMessageHandlerROS::filterStateFromROS(const pronto_msgs::FilterState &ros_msg,
                                               FilterState &msg)
{
    if(ros_msg.cov.size() != std::pow(ros_msg.state.size(),2))
    {
        throw std::logic_error("Covariance matrix of size " +
                               std::to_string(ros_msg.cov.size()) +
        + ". " + std::to_string(std::pow(ros_msg.state.size(),2)) + " expected.");
        return;
    }
    Eigen::Quaterniond init_quat;
    tf::Quaternion q;
    tf::quaternionMsgToTF(ros_msg.quat, q);
    tf::quaternionTFToEigen(q, init_quat);
    Eigen::Map<const Matrix<double, Dynamic, Dynamic, RowMajor>> cov_map(ros_msg.cov.data(),
                                                                         ros_msg.state.size(),
                                                                         ros_msg.state.size());
    msg.cov = cov_map;
    msg.quat = init_quat;
    msg.state = Eigen::Map<const Eigen::VectorXd>(ros_msg.state.data(), ros_msg.state.size());
    msg.utime = ros_msg.header.stamp.toNSec() / 1000;
}
}

