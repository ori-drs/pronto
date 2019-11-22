#include "pronto_ros/visual_odometry_ros_handler.hpp"
#include <pronto_ros/pronto_ros_conversions.hpp>

namespace pronto {

VisualOdometryHandlerROS::VisualOdometryHandlerROS(ros::NodeHandle& nh) {
  std::string prefix = "fovis/";
  VisualOdometryConfig cfg;
  std::string mode_str = "pos";
  if(!nh.getParam(prefix + "mode", mode_str)){
    ROS_WARN_STREAM("Couldn't read param \"" << prefix << "mode" << "\". Using position.");
  }
  if(mode_str.compare("pos") == 0){
    cfg.mode = VisualOdometryMode::MODE_POSITION;
    cfg.cov_vo.resize(3,3);
    cfg.z_indices.resize(3);
    cfg.z_indices = RBIS::positionInds();
  } else if(mode_str.compare("pos_orient") == 0){
    cfg.mode = VisualOdometryMode::MODE_POSITION_ORIENT;
    cfg.cov_vo.resize(6,6);
    cfg.z_indices.resize(6);
    cfg.z_indices.head<3>() = RBIS::positionInds();
    cfg.z_indices.tail<3>() = RBIS::chiInds();
  } else {
    ROS_WARN_STREAM("Unsupported mode \"" << prefix << mode_str << "\". Using position.");
  }

  // unrealistic high covariances by default
  double r_pxy = 1;
  double r_pz = 1;
  double r_pang = 50;

  if(!nh.getParam(prefix + "r_pxy", r_pxy)){
    ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_pxy");
  }
  if(!nh.getParam(prefix + "r_pz", r_pz)){
    ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_pz");
  }

  // square the standard deviation
  r_pxy = std::pow(r_pxy, 2);
  r_pz  = std::pow(r_pz, 2);

  cfg.cov_vo.topLeftCorner<3,3>() = Eigen::Vector3d(r_pxy, r_pxy, r_pz).asDiagonal();

  if(cfg.mode == VisualOdometryMode::MODE_POSITION_ORIENT){
    if(!nh.getParam(prefix + "r_pang", r_pang)){
      ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_pz");
    }
    // by default the values are in degrees, convert into radians and square
    cfg.cov_vo.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity() * std::pow((r_pang * M_PI / 180.0), 2);
  }
  vo_module_ = std::make_shared<VisualOdometryModule>(cfg);
}

RBISUpdateInterface * VisualOdometryHandlerROS::processMessage(const pronto_msgs::VisualOdometryUpdate *msg,
                                                               StateEstimator *state_estimator)
{
  visualOdometryFromROS(*msg, vo_update_);
  return vo_module_->processMessage(&vo_update_, state_estimator);
}

bool VisualOdometryHandlerROS::processMessageInit(const pronto_msgs::VisualOdometryUpdate *msg,
                                                  const std::map<std::string, bool> &sensor_initialized,
                                                  const RBIS &default_state,
                                                  const RBIM &default_cov,
                                                  RBIS &init_state,
                                                  RBIM &init_cov)
{
  return true;
}

}
