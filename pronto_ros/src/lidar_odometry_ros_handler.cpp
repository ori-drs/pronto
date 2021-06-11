#include "pronto_ros/lidar_odometry_ros_handler.hpp"
#include <pronto_ros/pronto_ros_conversions.hpp>

namespace pronto {

LidarOdometryHandlerROS::LidarOdometryHandlerROS(ros::NodeHandle& nh) : use_measurement_cov_(false) {
  std::string prefix = "scan_matcher/";
  std::string mode_str = "pos";
  if(!nh.getParam(prefix + "mode", mode_str)){
    ROS_WARN_STREAM("Couldn't read param \"" << prefix << "mode\". Using position.");
  }
  if(mode_str.compare("position") == 0){
    cfg_.mode = LidarOdometryMode::POSITION;
    cfg_.cov_vo.resize(3,3);
    cfg_.cov_vo = Eigen::Matrix3d::Identity();
    cfg_.z_indices.resize(3);
    cfg_.z_indices = RBIS::positionInds();
  } else if(mode_str.compare("position_orient") == 0){
    cfg_.mode = LidarOdometryMode::POSITION_ORIENT;
    cfg_.cov_vo.resize(6,6);
    cfg_.cov_vo = Eigen::Matrix<double, 6,6>::Identity();
    cfg_.z_indices.resize(6);
    cfg_.z_indices.head<3>() = RBIS::positionInds();
    cfg_.z_indices.tail<3>() = RBIS::chiInds();
  } else if(mode_str.compare("position_yaw") == 0){
    cfg_.mode = LidarOdometryMode::POSITION_YAW;
    cfg_.cov_vo.resize(4,4);
    cfg_.cov_vo = Eigen::Matrix<double, 4,4>::Identity();
    cfg_.z_indices.resize(4);
    cfg_.z_indices.head<3>() = RBIS::positionInds();
    cfg_.z_indices.tail<1>() << RBIS::chi_ind + 2;
  } else {
    ROS_WARN_STREAM("Unsupported mode \"" << prefix << mode_str << "\". Using position.");
  }
  // before doing anything, set the covariance matrix to identity

  // unrealistic high covariances by default
  double r_pxy = 1;
  double r_pz = 1;
  double r_rxy = 50;
  double r_ryaw = 50;

  if(!nh.getParam(prefix + "use_measurement_cov", use_measurement_cov_)){
    ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "use_measurement_cov");
  }

  if(!nh.getParam(prefix + "r_pxy", r_pxy)){
    ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_px");
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
  r_pxy = std::pow(r_pxy, 2);
  r_pz  = std::pow(r_pz, 2);

  cfg_.cov_vo.topLeftCorner<3,3>() = Eigen::Vector3d(r_pxy, r_pxy, r_pz).asDiagonal();

  if(cfg_.mode == LidarOdometryMode::POSITION_ORIENT || cfg_.mode == LidarOdometryMode::POSITION_YAW){
    if(!nh.getParam(prefix + "r_rxy", r_rxy)){
      ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_rxy");
    }
    if(!nh.getParam(prefix + "r_ryaw", r_ryaw)){
      ROS_ERROR_STREAM("Coudn't read param \"" << prefix << "r_ryaw");
    }
    if(cfg_.mode == LidarOdometryMode::POSITION_ORIENT) {
      // by default the values are in degrees, convert into radians and square
      cfg_.cov_vo.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity() * std::pow((r_rxy * M_PI / 180.0), 2);
    }
    // the yaw element is common to both modes
    // note that this overwrites the last element if POSITION_ORIENT
    cfg_.cov_vo.bottomRightCorner<1,1>() << std::pow((r_ryaw * M_PI / 180.0), 2);
  }
  lidarodom_module_ = std::make_shared<LidarOdometryModule>(cfg_);
}

RBISUpdateInterface * LidarOdometryHandlerROS::processMessage(const pronto_msgs::LidarOdometryUpdate *msg,
                                                               StateEstimator *state_estimator)
{

  pronto_msgs::LidarOdometryUpdate mymsg = *msg;

  if(msg->curr_timestamp < msg->prev_timestamp){
    ROS_ERROR("[LidarOdometryHandlerROS] curr_timestamp is less than prev_timestamp! Ignoring message.");
    return nullptr;
  }

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
  lidarOdometryFromROS(mymsg, lidarodom_update_);
  if(use_measurement_cov_ && cfg_.mode == LidarOdometryMode::POSITION){
    lidarodom_module_->setCovariance(lidarodom_update_.pose_covariance.topLeftCorner<3,3>());// position only
  }
  return lidarodom_module_->processMessage(&lidarodom_update_, state_estimator);
}

bool LidarOdometryHandlerROS::processMessageInit(const pronto_msgs::LidarOdometryUpdate *msg,
                                                  const std::map<std::string, bool> &sensor_initialized,
                                                  const RBIS &default_state,
                                                  const RBIM &default_cov,
                                                  RBIS &init_state,
                                                  RBIM &init_cov)
{
  return true;
}

}
