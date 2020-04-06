#include "pronto_biped_ros/forward_kinematics_ros.hpp"
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <ros/console.h>
#include <set>

using namespace pronto::biped;

BipedForwardKinematicsROS::BipedForwardKinematicsROS(std::string urdf_string,
                                                     std::string left_foot_name,
                                                     std::string right_foot_name) :
  urdf_string_(urdf_string),
  left_foot_name_(left_foot_name),
  right_foot_name_(right_foot_name)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_string ,tree)){
    ROS_FATAL("Failed to extract kdl tree from xml robot description");
    exit(-1);
  }
  fk_solver_.reset(new KDL::TreeFkSolverPosFull_recursive(tree));


}

void BipedForwardKinematicsROS::setJointNames(const JointName &names){
  joint_names_ = names;

  // fill the joint state and cartesian state maps with zero/identity values
  for(size_t i; i < joint_names_.size(); i++){
    jointpos_in.insert(std::make_pair(joint_names_[i], 0.0));
    cartpos_out.insert(std::make_pair(joint_names_[i], KDL::Frame::Identity()));
  }

  if(!joint_names_.empty() && joint_names_.size() == 30){
    joint_names_ok_ = true;
  }
}

bool BipedForwardKinematicsROS::getLeftFootPose(const JointState &q, Transform &x){
#if DEBUG_MODE
  std::cerr <<"Joint names ok: " << std::boolalpha << joint_names_ok_ << std::endl;
  std::cerr << "nr. " << joint_names_.size() << std::endl;

  for(const auto& el : joint_names_){
    std::cerr << "name: " << el << std::endl;
  }
#endif
  return joint_names_ok_ && computeFK(q, x, left_foot_name_);
}

bool BipedForwardKinematicsROS::getRightFootPose(const JointState &q, Transform &x){
  return joint_names_ok_ && computeFK(q, x, right_foot_name_);
}

bool BipedForwardKinematicsROS::computeFK(const JointState& q, Transform& t, std::string foot_name)
{
  // fill in the joint name / position map
  for (size_t i = 0; i < joint_names_.size(); i++) {
    jointpos_in[joint_names_[i]] = q[i];
  }
#if DEBUG_MODE
  for(const auto& kv : jointpos_in){
    std::cerr << "jointpos_in[" << kv.first << "] = " << kv.second << std::endl;
  }
#endif
  // compute FK
  int res = fk_solver_->JntToCart(jointpos_in, cartpos_out, true);
  // convert to Eigen
  tf::transformKDLToEigen(cartpos_out.find(foot_name)->second, t);
  return res == 0;
}


