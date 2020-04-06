#pragma once

#include "pronto_biped_core/biped_forward_kinematics.hpp"
#include <Eigen/Dense>
#include <memory>

namespace pronto {
namespace biped {

class YawLock {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    YawLock(pronto::biped::BipedForwardKinematics& fk);

    ~YawLock(){
    }

    inline void setParameters(int correction_period_in,
                       bool yaw_slip_detect_in,
                       double yaw_slip_threshold_degrees_in,
                       double yaw_slip_disable_period_in)
    {
        correction_period_ = correction_period_in;
        yaw_slip_detect_ = yaw_slip_detect_in;
        yaw_slip_threshold_degrees_ = yaw_slip_threshold_degrees_in;
        yaw_slip_disable_period_ = yaw_slip_disable_period_in;
    }

    inline void setIsRobotStanding(bool is_robot_standing_in) {
      is_robot_standing_ = is_robot_standing_in;
    }

    inline bool isRobotStanding() const {
      return is_robot_standing_;
    }

    inline void setJointState(const std::vector<double>& joint_position_in,
                              const std::vector<std::string>& joint_name_in)
    {
      joint_angles_init_ = true;
      joint_position_ = joint_position_in;
      joint_name_ = joint_name_in;
    }

    inline bool getJointAnglesInit() const{
        return joint_angles_init_;
    }

    inline void setStandingLinks(const std::string& left_standing_link_in,
                          const std::string& right_standing_link_in)
    {
      left_standing_link_ = left_standing_link_in;
      right_standing_link_ = right_standing_link_in;
    }

    // Determine the actual yaw correction to be made.
    // Input is the current floating base
    // output bool is true if a correction should be made
    // and world_to_body_quat_correction holds the full orientation to be corrected to
    bool getCorrection(const Eigen::Isometry3d& world_to_body,
                       const int64_t& body_utime,
                       Eigen::Quaterniond &world_to_body_quat_correction);


  private:
    BipedForwardKinematics& fk_;
    std::vector<std::string> joint_name_;
    std::vector<double> joint_position_;

    // Poses of the l and r feet when robot first became standing or manipulating:
    Eigen::Isometry3d world_to_l_foot_original_;
    Eigen::Isometry3d world_to_r_foot_original_;
    bool joint_angles_init_ = false; // have received some joint angles
    bool lock_init_ = false; // is the yaw lock active

    // Used in foot slip detection:
    Eigen::Isometry3d l_foot_to_r_foot_original_;
    // don't disable at start (but will be uninitialized)
    int64_t utime_disable_until_ = 0;

    int64_t counter_ = 0;

    // Controller state set to unknonn at start
    bool is_robot_standing_ = false;
    double current_yaw_;

    // Parameters
    int correction_period_;
    bool yaw_slip_detect_;
    double yaw_slip_threshold_degrees_;
    double yaw_slip_disable_period_;

    std::string left_standing_link_;
    std::string right_standing_link_;
};
}
} // namespace pronto
