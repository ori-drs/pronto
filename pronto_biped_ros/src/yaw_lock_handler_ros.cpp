#include "pronto_biped_ros/yaw_lock_handler_ros.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <pronto_ros/pronto_ros_conversions.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace pronto {
namespace biped {

YawLockHandlerROS::YawLockHandlerROS(ros::NodeHandle &nh, std::string urdf_string)
  : nh_(nh)
{
  std::string prefix = "bias_lock/";
  YawLockConfig cfg;
  if(!nh_.getParam(prefix + "correction_period", cfg.correction_period)){
    ROS_WARN_STREAM("Couldn't find parameter \"correction_period\"");
  }

  if(!nh_.getParam(prefix + "yaw_slip_detect", cfg.yaw_slip_detect)){
    ROS_WARN_STREAM("Couldn't find parameter \"yaw_slip_detect\"");
  }

  if(!nh_.getParam(prefix + "yaw_slip_threshold_degrees", cfg.yaw_slip_threshold_degrees)){
    ROS_WARN_STREAM("Couldn't find parameter \"yaw_slip_threshold_degrees\"");
  }

  if(!nh_.getParam(prefix + "yaw_slip_disable_period", cfg.yaw_slip_disable_period)){
    ROS_WARN_STREAM("Couldn't find parameter \"yaw_slip_disable_period\"");
  }

  if(!nh_.getParam(prefix + "left_standing_link", cfg.left_standing_link)){
    ROS_WARN_STREAM("Couldn't find parameter \"left_standing_link\"");
  }

  if(!nh_.getParam(prefix + "right_standing_link", cfg.right_standing_link)){
    ROS_WARN_STREAM("Couldn't find parameter \"right_standing_link\"");
  }

  if(!nh_.getParam(prefix + "r_yaw_bias", cfg.r_yaw_bias)){
    ROS_WARN_STREAM("Couldn't find parameter \"r_yaw_bias\"");
  }

  if(!nh_.getParam(prefix + "r_yaw", cfg.r_yaw)){
    ROS_WARN_STREAM("Couldn't find parameter \"r_yaw\"");
  }
  std::string yawlock_mode;
  if(!nh_.getParam(prefix + "yawlock_mode", yawlock_mode)){
    ROS_WARN_STREAM("Couldn't find parameter \"yawlock_mode\"");
  }

  if(yawlock_mode.compare("yaw") == 0){
    cfg.mode = YawLockMode::YAW;
  } else if (yawlock_mode.compare("yawbias")==0){
    cfg.mode = YawLockMode::YAWBIAS;
  } else if (yawlock_mode.compare("yawbias_yaw")==0){
    cfg.mode = YawLockMode::YAWBIAS_YAW;
  } else {
    cfg.mode = YawLockMode::YAW;
  }

  Eigen::Isometry3d ins_to_body = Eigen::Isometry3d::Identity();
  std::string ins_param_prefix = "ins/";
  std::string imu_frame;
  if(!nh_.getParam(ins_param_prefix + "frame", imu_frame)){
    ROS_FATAL_STREAM("Couldn't find parameter " << ins_param_prefix << "frame");
  };
  std::string base_frame = "base";
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf_imu_to_body_listener_(tfBuffer);

  while(nh_.ok()){
    try{
      geometry_msgs::TransformStamped temp_transform = tfBuffer.lookupTransform(imu_frame, base_frame,
                                                                                ros::Time(0));

      tf::transformMsgToEigen(temp_transform.transform, ins_to_body);
      break;
    }
    catch (tf2::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  fk_.reset(new BipedForwardKinematicsROS(urdf_string, cfg.left_standing_link, cfg.right_standing_link));

  yawlock_module_.reset(new YawLockModule(*fk_, cfg, ins_to_body));

}


RBISUpdateInterface* YawLockHandlerROS::processMessage(const sensor_msgs::Imu *msg,
                                                       StateEstimator *est)
{
  msgToImuMeasurement(*msg,imu_meas_);
  return yawlock_module_->processMessage(&imu_meas_, est);
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
  jointStateFromROS(msg, joint_state_meas_);
  yawlock_module_->processSecondaryMessage(joint_state_meas_);
}

} // namespace biped
} // namespace pronto
