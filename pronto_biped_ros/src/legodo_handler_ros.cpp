#include "pronto_biped_ros/legodo_handler_ros.hpp"
#include <pronto_ros/pronto_ros_conversions.hpp>

namespace pronto {
namespace biped {

LegOdometryHandler::LegOdometryHandler(std::string urdf_string,
                                       ros::NodeHandle& nh) : nh_(nh) {
  std::string prefix = "/state_estimator_pronto/legodo/";
  if(!nh_.getParam(prefix + "torque_adjustment", legodo_cfg_.use_torque_adjustment_)){
    ROS_WARN_STREAM("Couldn't find parameter \"torque_adjustment\"."
                    << " Using default : "
                    << legodo_cfg_.use_torque_adjustment_);
  }

  if(!nh_.getParam(prefix + "adjustment_gain", legodo_cfg_.torque_adj_gains_)){
    ROS_WARN_STREAM("Couldn't find parameter \"adjustment_gain\"."
                    << " Not using torque adjustment");
    legodo_cfg_.use_torque_adjustment_ = false;
  }

  if(!nh_.getParam(prefix + "adjustment_joints", legodo_cfg_.torque_adj_names_)){
    ROS_WARN_STREAM("Couldn't find parameter \"adjustment_joints\"."
                    << " Not using torque adjustment");
    legodo_cfg_.use_torque_adjustment_ = false;
  }

  if(!nh_.getParam(prefix + "zero_initial_velocity", legodo_cfg_.zero_initial_velocity)){
    ROS_WARN_STREAM("Couldn't find parameter \"torque_adjustment\"."
                    << " Using default : "
                    << legodo_cfg_.zero_initial_velocity);
  }
  legodo_cfg_.common_cfg.mode_ = LegOdometryMode::LIN_RATE;
  std::string mode;
  if(!nh_.getParam(prefix + "mode", mode)){
    ROS_WARN_STREAM("Couldn't find parameter \"torque_adjustment\"."
                    << " Using default : lin_rate");
  } else if(mode.compare("pos_and_lin_rate") == 0){
    legodo_cfg_.common_cfg.mode_ = LegOdometryMode::POSITION_AND_LIN_RATE;
  } else if(mode.compare("lin_rot_rate") == 0){
    legodo_cfg_.common_cfg.mode_ = LegOdometryMode::LIN_AND_ROT_RATE;
  } else if(mode.compare("rot_rate") == 0){
    legodo_cfg_.common_cfg.mode_ = LegOdometryMode::ROT_RATE;
  }

  if(!nh_.getParam(prefix + "r_vang", legodo_cfg_.common_cfg.R_legodo_vang_)){
    ROS_WARN_STREAM("Couldn't find parameter \"r_vang\"."
                    << " Using default : "
                    << legodo_cfg_.common_cfg.R_legodo_vang_);
  }
  if(!nh_.getParam(prefix + "r_vang_uncertain", legodo_cfg_.common_cfg.R_legodo_vang_uncertain_))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"r_vang_uncertain\"."
                    << " Using default : "
                    << legodo_cfg_.common_cfg.R_legodo_vang_uncertain_);
  }
  if(!nh_.getParam(prefix + "r_vxyz", legodo_cfg_.common_cfg.R_legodo_vxyz_))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"r_vxyz\"."
                    << " Using default : "
                    << legodo_cfg_.common_cfg.R_legodo_vxyz_);
  }
  if(!nh_.getParam(prefix + "r_vxyz_uncertain", legodo_cfg_.common_cfg.R_legodo_vxyz_uncertain_))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"r_vxyz_uncertain\"."
                    << " Using default : "
                    << legodo_cfg_.common_cfg.R_legodo_vxyz_uncertain_);
  }
  if(!nh_.getParam(prefix + "r_xyz", legodo_cfg_.common_cfg.R_legodo_xyz_))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"r_xyz\"."
                    << " Using default : "
                    << legodo_cfg_.common_cfg.R_legodo_xyz_);
  }
  if(!nh_.getParam(prefix + "verbose", legodo_cfg_.common_cfg.verbose))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"verbose\"."
                    << " Using default : "
                    << std::boolalpha <<  legodo_cfg_.common_cfg.verbose);
  }
  if(!nh_.getParam(prefix + "verbose", legodo_cfg_.common_cfg.verbose))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"verbose\"."
                    << " Using default : "
                    << std::boolalpha << legodo_cfg_.common_cfg.verbose);
  }
  legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::NONE;
  if(!nh_.getParam(prefix + "filter_joint_positions", mode))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"filter_joint_positions\"."
                    << " Using default : none");
  } else if(mode.compare("lowpass") == 0){
    legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::LOWPASS;
  } else if (mode.compare("kalman") == 0){
    legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::KALMAN;
  }
  if(!nh_.getParam(prefix + "use_controller_input", legodo_cfg_.odometer_cfg.use_controller_input))
  {
    ROS_WARN_STREAM("Couldn't find parameter \"use_controller_input\"."
                    << " Using default: "
                    << std::boolalpha << legodo_cfg_.odometer_cfg.use_controller_input);
  }

  legodo_module_.reset(new LegOdometryModule(urdf_string, legodo_cfg_));
}

RBISUpdateInterface* LegOdometryHandler::processMessage(const sensor_msgs::JointState *msg,
                                                        StateEstimator *est)
{
  jointStateFromROS(*msg, legodo_msg_);
  return legodo_module_->processMessage(&legodo_msg_, est);
}

bool LegOdometryHandler::processMessageInit(const sensor_msgs::JointState *msg,
                                            const std::map<std::string, bool> &sensor_initialized,
                                            const RBIS &default_state,
                                            const RBIM &default_cov,
                                            RBIS &init_state,
                                            RBIM &init_cov)
{
  jointStateFromROS(*msg, legodo_msg_);
  return legodo_module_->processMessageInit(&legodo_msg_, sensor_initialized, default_state, default_cov, init_state, init_cov);
}

}
}

