#include "pronto_quadruped_ros/force_sensor_stance_estimator_ros.hpp"


namespace pronto {
namespace quadruped {

ForceSensorStanceEstimatorROS::ForceSensorStanceEstimatorROS(double force_threshold)
  : ForceSensorStanceEstimator(force_threshold)
{
}

ForceSensorStanceEstimatorROS::ForceSensorStanceEstimatorROS(ros::NodeHandle &nh)
{
  // get parameters for the leg odometry
  std::string legodo_prefix = "legodo/";
  double stance_threshold;
  nh.getParam(legodo_prefix + "stance_threshold", stance_threshold);
  force_threshold_ = stance_threshold;
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] Stance threshold: " << force_threshold_);
}

}  // namespace quadruped
}  // namespace pronto
