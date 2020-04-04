#pragma once
#include <pronto_core/sensing_module.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/node_handle.h>
#include <pronto_biped_core/legodo_module.hpp>
#include <pronto_msgs/BipedForceTorqueSensors.h>
#include <pronto_msgs/ControllerFootContact.h>
#include <pronto_ros/pronto_ros_conversions.hpp>


namespace pronto {
namespace biped {

class LegOdometryHandler : public pronto::SensingModule<sensor_msgs::JointState> {
public:

  LegOdometryHandler() = delete;
  LegOdometryHandler(std::string urdf_string, ros::NodeHandle& nh);

  RBISUpdateInterface* processMessage(const sensor_msgs::JointState *msg,
                                      StateEstimator *est) override;

  bool processMessageInit(const sensor_msgs::JointState *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov) override;
public:
  void forceTorqueCallback(const pronto_msgs::BipedForceTorqueSensorsConstPtr& msg);
  void ctrlFootContactCallback(const pronto_msgs::ControllerFootContactConstPtr& msg);
protected:
  ros::NodeHandle nh_;
  std::unique_ptr<LegOdometryModule> legodo_module_;
  ros::Subscriber ctrl_foot_contact_sub_;
  ros::Subscriber force_torque_sub_;
  JointState legodo_msg_;
  LegOdometryConfig legodo_cfg_;
  pronto::ForceTorqueSensorArray ft_msg_;
};

}
}
