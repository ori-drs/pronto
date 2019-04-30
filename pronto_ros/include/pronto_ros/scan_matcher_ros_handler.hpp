#pragma once
#include <Eigen/Dense>
#include <pronto_core/rbis_update_interface.hpp>
#include <pronto_core/mav_state_est.hpp>
#include <pronto_core/scan_matcher_module.hpp>
#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>

namespace  MavStateEst {

class ScanMatcherHandler : SensingModule<nav_msgs::Odometry>{
public:
  ScanMatcherHandler(ros::NodeHandle& nh);

  RBISUpdateInterface * processMessage(const nav_msgs::Odometry * msg,
                                       MavStateEstimator* state_estimator) override;

  bool processMessageInit(const nav_msgs::Odometry *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov) override;

protected:
  ScanMatcherModule scan_matcher_module_;
  PoseMeasurement pose_meas_;
  ros::NodeHandle& nh_;
  tf::Quaternion tf_q;
};


} // namespace MavStateEst
