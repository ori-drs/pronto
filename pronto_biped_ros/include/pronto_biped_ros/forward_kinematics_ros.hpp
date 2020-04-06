#pragma once
#include <pronto_biped_core/biped_forward_kinematics.hpp>
#include "pronto_biped_ros/treefksolverposfull_recursive.hpp"
#include <kdl/frames.hpp>
#include <memory>
#include <map>

namespace pronto {
namespace biped {

class BipedForwardKinematicsROS : public BipedForwardKinematics {
public:
  using ForwardKinematicsSolver = KDL::TreeFkSolverPosFull_recursive;
  using ForwardKinematicsSolverPtr = std::unique_ptr<ForwardKinematicsSolver>;
  using JointName = std::vector<std::string>;

public:
  BipedForwardKinematicsROS() = delete;
  BipedForwardKinematicsROS(std::string urdf_string,
                            std::string left_foot_name,
                            std::string right_foot_name);

  void setJointNames(const JointName& names);

  bool getLeftFootPose(const JointState &q, Transform& x) override;
  bool getRightFootPose(const JointState &q, Transform& x) override;

  inline Transform getLeftFootPose(const JointState &q) override {
    Transform t;
    getLeftFootPose(q,t);
    return t;
  }

  inline Transform getRightFootPose(const JointState &q) override {
    Transform t;
    getRightFootPose(q, t);
    return t;
  }

private:
  std::string urdf_string_;
  std::vector<std::string> joint_names_;
  bool joint_names_ok_ = false;
  std::string left_foot_name_;
  std::string right_foot_name_;
  ForwardKinematicsSolverPtr fk_solver_;
  std::map<std::string, double> jointpos_in;
  std::map<std::string, KDL::Frame> cartpos_out;

  bool computeFK(const JointState& q, Transform& t, std::string foot_name);
};

}
}
