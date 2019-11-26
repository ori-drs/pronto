#pragma once
#include <pronto_core/sensing_module.hpp>
#include <pronto_core/visual_odometry_module.hpp>
#include <pronto_msgs/VisualOdometryUpdate.h>
#include <ros/node_handle.h>

namespace pronto {

class VisualOdometryHandlerROS : public SensingModule<pronto_msgs::VisualOdometryUpdate>{
public:
  VisualOdometryHandlerROS(ros::NodeHandle& nh);

  RBISUpdateInterface * processMessage(const pronto_msgs::VisualOdometryUpdate* msg,
                                       StateEstimator* state_estimator);

  bool processMessageInit(const pronto_msgs::VisualOdometryUpdate *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov);
private:
  std::shared_ptr<VisualOdometryModule> vo_module_;
  VisualOdometryUpdate vo_update_;
  ros::Duration msg_time_offset_;
};

}
