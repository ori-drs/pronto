#include "pronto_ros/visual_odometry_ros_handler.hpp"
#include <pronto_ros/pronto_ros_conversions.hpp>

namespace pronto {

VisualOdometryHandlerROS::VisualOdometryHandlerROS(ros::NodeHandle& nh) {
  std::string prefix = "fovis/";
  VisualOdometryConfig cfg;
  std::string mode_str = "pos";
  if(!nh.getParam(prefix + "mode", mode_str)){
    ROS_WARN_STREAM("Couldn't read param \"" << prefix << "mode\". Using position.");
  }
  if(mode_str.compare("pos") == 0){
    cfg.mode = VisualOdometryMode::MODE_POSITION;
    cfg.cov_vo.resize(3,3);
    cfg.cov_vo = Eigen::Matrix3d::Identity();
    cfg.z_indices.resize(3);
    cfg.z_indices = RBIS::positionInds();
  } else if(mode_str.compare("pos_orient") == 0){
    cfg.mode = VisualOdometryMode::MODE_POSITION_ORIENT;
    cfg.cov_vo.resize(6,6);
    cfg.cov_vo = Eigen::Matrix<double, 6,6>::Identity();
    cfg.z_indices.resize(6);
    cfg.z_indices.head<3>() = RBIS::positionInds();
    cfg.z_indices.tail<3>() = RBIS::chiInds();
  } else {
    ROS_WARN_STREAM("Unsupported mode \"" << prefix << mode_str << "\". Using position.");
  }
  // before doing anything, set the covariance matrix to identity


  // unrealistic high covariances by default
  double r_px = 1;
  double r_py = 1;
  double r_pz = 1;
  double r_rxy = 50;
  double r_ryaw = 50;

  if(!nh.getParam(prefix + "r_px", r_px)){
    ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_px");
  }
  if(!nh.getParam(prefix + "r_py", r_py)){
    ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_py");
  }
  if(!nh.getParam(prefix + "r_pz", r_pz)){
    ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_pz");
  }
  int utime_offset = 0;
  if(!nh.getParam(prefix + "utime_offset", utime_offset)){
    ROS_WARN_STREAM("Couldnt' set time offset, set to 0.");
  }
  msg_time_offset_ = ros::Duration().fromNSec(utime_offset * 1e3);

  // square the standard deviation
  r_px = std::pow(r_px, 2);
  r_py = std::pow(r_py, 2);
  r_pz  = std::pow(r_pz, 2);

  cfg.cov_vo.topLeftCorner<3,3>() = Eigen::Vector3d(r_px, r_py, r_pz).asDiagonal();

  if(cfg.mode == VisualOdometryMode::MODE_POSITION_ORIENT){
    if(!nh.getParam(prefix + "r_rxy", r_rxy)){
      ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_rxy");
    }
    if(!nh.getParam(prefix + "r_ryaw", r_ryaw)){
      ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_ryaw");
    }
    // by default the values are in degrees, convert into radians and square
    cfg.cov_vo.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity() * std::pow((r_rxy * M_PI / 180.0), 2);
    // the yaw element is different than the rest
    cfg.cov_vo.bottomRightCorner<1,1>() << std::pow((r_ryaw * M_PI / 180.0), 2);
  }
  vo_module_ = std::make_shared<VisualOdometryModule>(cfg);
}

RBISUpdateInterface * VisualOdometryHandlerROS::processMessage(const pronto_msgs::VisualOdometryUpdate *msg,
                                                               StateEstimator *state_estimator)
{

  pronto_msgs::VisualOdometryUpdate mymsg = *msg;
  try {
    mymsg.header.stamp = msg->header.stamp + msg_time_offset_;
    mymsg.curr_timestamp = msg->curr_timestamp + msg_time_offset_;
    mymsg.prev_timestamp = msg->prev_timestamp + msg_time_offset_;
  } catch (std::runtime_error& ex) {
    ROS_WARN_STREAM("HEADER: " << msg->header.stamp.toNSec());
    ROS_WARN_STREAM("CURR: " << msg->curr_timestamp.toNSec());
    ROS_WARN_STREAM("PREV: " << msg->prev_timestamp.toNSec());
    ROS_WARN_STREAM("OFFSET: " << msg_time_offset_.toNSec());
    ROS_ERROR("Exception: [%s]", ex.what());
  }
  visualOdometryFromROS(mymsg, vo_update_);
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
