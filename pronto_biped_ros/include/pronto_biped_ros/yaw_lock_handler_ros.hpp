#pragma once

#include <pronto_core/sensing_module.hpp>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <ros/node_handle.h>

namespace pronto {
namespace biped {
class YawLockHandlerROS : public DualSensingModule<sensor_msgs::Imu, sensor_msgs::JointState> {
public:
  YawLockHandlerROS(ros::NodeHandle& nh);
  inline virtual ~YawLockHandlerROS() {}


  RBISUpdateInterface* processMessage(const sensor_msgs::Imu *msg, StateEstimator *est) override;

  bool processMessageInit(const sensor_msgs::Imu *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov) override;

  void processSecondaryMessage(const sensor_msgs::JointState& msg) override;


protected:
  ros::NodeHandle& nh_;


};

} // namespace biped
} // namespace pronto

