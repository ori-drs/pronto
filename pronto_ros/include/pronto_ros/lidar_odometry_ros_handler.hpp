#pragma once
#include <pronto_core/sensing_module.hpp>
#include <pronto_core/lidar_odometry_module.hpp>
#include <pronto_msgs/LidarOdometryUpdate.h>
#include <ros/node_handle.h>

namespace pronto {

class LidarOdometryHandlerROS : public SensingModule<pronto_msgs::LidarOdometryUpdate>{
public:
  LidarOdometryHandlerROS(ros::NodeHandle& nh);

  RBISUpdateInterface * processMessage(const pronto_msgs::LidarOdometryUpdate* msg,
                                       StateEstimator* state_estimator);

  bool processMessageInit(const pronto_msgs::LidarOdometryUpdate *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov);
private:
  std::shared_ptr<LidarOdometryModule> lidarodom_module_;
  LidarOdometryUpdate lidarodom_update_;
  ros::Duration msg_time_offset_;
};

}
