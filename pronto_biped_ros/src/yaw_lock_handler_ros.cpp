#include "pronto_biped_ros/yaw_lock_handler_ros.hpp"

namespace pronto {
namespace biped {

YawLockHandlerROS::YawLockHandlerROS(ros::NodeHandle &nh) : nh_(nh){

}

RBISUpdateInterface* YawLockHandlerROS::processMessage(const sensor_msgs::Imu *msg, StateEstimator *est) {
  return nullptr;
}

bool YawLockHandlerROS::processMessageInit(const sensor_msgs::Imu *msg,
                                           const std::map<std::string, bool> &sensor_initialized,
                                           const RBIS &default_state,
                                           const RBIM &default_cov,
                                           RBIS &init_state,
                                           RBIM &init_cov) {
  return true;
}

void YawLockHandlerROS::processSecondaryMessage(const sensor_msgs::JointState& msg) {

}

} // namespace biped
} // namespace pronto
