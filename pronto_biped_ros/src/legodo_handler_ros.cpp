#include "pronto_biped_ros/legodo_handler_ros.hpp"
#include <pronto_ros/pronto_ros_conversions.hpp>

namespace pronto {
namespace biped {

LegOdometryHandler::LegOdometryHandler(ros::NodeHandle& nh,
                                       std::string urdf_string) :
  nh_(nh),
  legodo_msg_(30),
  urdf_string_(urdf_string)
{
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
  std::string topic_force_torque;
  if(!nh_.getParam(prefix + "topic_force_torque", topic_force_torque)){
    ROS_WARN_STREAM("Couldn't find parameter \"topic_force_torque\".");
  }
  ROS_INFO_STREAM("Subscribing to auxiliary topic: " << topic_force_torque);
  force_torque_sub_ = nh_.subscribe(topic_force_torque, 10, &LegOdometryHandler::forceTorqueCallback, this);

  std::string topic_ctrl_input;
  if(!nh_.getParam(prefix + "topic_ctrl_input", topic_ctrl_input)){
    ROS_WARN_STREAM("Couldn't find parameter \"topic_ctrl_input\".");
  }
  ROS_INFO_STREAM("Subscribing to auxiliary topic: " << topic_force_torque);
  ctrl_foot_contact_sub_ = nh_.subscribe(topic_ctrl_input, 10, &LegOdometryHandler::ctrlFootContactCallback, this);

  if(!nh_.getParam(prefix + "initialization_mode", legodo_cfg_.odometer_cfg.initialization_mode)){
    ROS_WARN_STREAM("Couldn't find parameter \"initialization_mode\".");
  }

  if(!nh_.getParam(prefix + "left_foot_name", legodo_cfg_.odometer_cfg.left_foot_name)){
    ROS_WARN_STREAM("Couldn't find parameter \"left_foot_name\".");
  }

  if(!nh_.getParam(prefix + "right_foot_name", legodo_cfg_.odometer_cfg.right_foot_name)){
    ROS_WARN_STREAM("Couldn't find parameter \"right_foot_name\".");
  }


  std::string filter_mode;

  if(!nh_.getParam(prefix + "filter_joint_positions", filter_mode)){
    ROS_WARN_STREAM("Couldn't find parameter \"filter_joint_positions\".");
  }

  if(filter_mode.compare("lowpass") == 0){
    legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::LOWPASS;
  } else if(filter_mode.compare("kalman") == 0){
    legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::KALMAN;
  } else {
    legodo_cfg_.odometer_cfg.filter_mode = FilterJointMode::NONE;
  }

  if(!nh_.getParam(prefix + "filter_contact_events", legodo_cfg_.odometer_cfg.filter_contact_events)){
    ROS_WARN_STREAM("Couldn't find parameter \"filter_contact_events\".");
  }

  if(!nh_.getParam(prefix + "publish_diagnostics", legodo_cfg_.odometer_cfg.publish_diagnostics)){
    ROS_WARN_STREAM("Couldn't find parameter \"publish_diagnostics\".");
  }

  if(!nh_.getParam(prefix + "active_joints", active_joints_)){
    ROS_WARN_STREAM("Couldn't find parameter \"active_joints\".");
  }


  fk_.reset(new BipedForwardKinematicsROS(urdf_string,
                                          legodo_cfg_.odometer_cfg.left_foot_name,
                                          legodo_cfg_.odometer_cfg.right_foot_name));

  legodo_module_.reset(new LegOdometryModule(*fk_, legodo_cfg_));
}

RBISUpdateInterface* LegOdometryHandler::processMessage(const sensor_msgs::JointState *msg,
                                                        StateEstimator *est)
{
  if(msg->position.size() < active_joints_){
    // different publisher might send messages to the /joint_state topic
    // we use the expected length of the joint vector to check that we
    // are processing the right one
    return nullptr;
  }
  if(!init){
    joint_names_ = msg->name;
    fk_->setJointNames(joint_names_);
    for(const auto& el : joint_names_){
      std::cerr << "Joint " << el << std::endl;
    }
    init = true;
  }
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
  if(!init){
    joint_names_ = msg->name;
    fk_->setJointNames(joint_names_);
    init = true;
  }
  jointStateFromROS(*msg, legodo_msg_);
  return legodo_module_->processMessageInit(&legodo_msg_, sensor_initialized, default_state, default_cov, init_state, init_cov);
}

void LegOdometryHandler::ctrlFootContactCallback(const pronto_msgs::ControllerFootContactConstPtr & msg){
  legodo_module_->setControllerInput(msg->num_left_foot_contacts, msg->num_right_foot_contacts);
}

void LegOdometryHandler::forceTorqueCallback(const pronto_msgs::BipedForceTorqueSensorsConstPtr &msg){
  forceTorqueFromROS(*msg, ft_msg_);
  legodo_module_->setForceTorque(ft_msg_);
}

}
}

