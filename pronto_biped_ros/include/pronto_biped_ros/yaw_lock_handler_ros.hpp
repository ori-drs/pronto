#pragma once

#include <pronto_core/sensing_module.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/node_handle.h>

namespace pronto {
namespace biped {
class YawLockHandlerROS : public SensingModule<sensor_msgs::JointState> {
public:
  YawLockHandlerROS(ros::NodeHandle& nh);
  inline virtual ~YawLockHandlerROS() {}
protected:
  ros::NodeHandle& nh_;

};

} // namespace biped
} // namespace pronto

