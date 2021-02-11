#include "pronto_ros/scan_matcher_ros_handler.hpp"
#include <string>
#include <tf_conversions/tf_eigen.h>
#include "pronto_ros/pronto_ros_conversions.hpp"
#include <pronto_core/rotations.hpp>

namespace pronto {

using Mode = ScanMatcherModule::ScanMatchingMode;

ScanMatcherHandler::ScanMatcherHandler(ros::NodeHandle& nh) : nh_(nh)
{
    Mode mode;
    Eigen::VectorXi z_indices;
    Eigen::MatrixXd cov_scan_match;

    std::string prefix = "scan_matcher/";
    std::string mode_str;

    if(!nh_.getParam(prefix + "mode", mode_str)){
        ROS_WARN("Couldn't get param \"mode\". Using MODE_POSITION by default.");
        mode_str = "position";
    }

  if (mode_str.compare("position") == 0) {
    mode = Mode::POSITION;
    std::cout << "Scan matcher will provide position measurements." << std::endl;
  }
  else if (mode_str.compare("position_yaw") == 0) {
    mode = Mode::POSITION_YAW;
    ROS_INFO_STREAM("Scan matcher will provide position and yaw measurements.");
  }
  else if (mode_str.compare("position_orient") == 0) {
    mode = Mode::POSITION_ORIENT;
  }
  else if (mode_str.compare("yaw") == 0) {
    mode = Mode::YAW;
    std::cout << "Scan matcher will provide yaw measurements." << std::endl;
  }
  else {
    mode = Mode::POSITION;
    ROS_WARN("Couldn't get param \"mode\". Using MODE_POSITION by default.");
  }
  Eigen::VectorXd R_scan_match;

  switch(mode){
  case Mode::POSITION:
      z_indices.resize(3);
      R_scan_match.resize(3);
    break;
  case Mode::VELOCITY:
    z_indices.resize(3);
    R_scan_match.resize(3);
    break;
  case Mode::YAW:
    z_indices.resize(1);
    R_scan_match.resize(1);
    break;
  case Mode::POSITION_YAW:
    z_indices.resize(4); // Use yaw measurements too.
    R_scan_match.resize(4);
    break;
  case Mode::VELOCITY_YAW:
    z_indices.resize(4); // Use yaw measurements too.
    R_scan_match.resize(4);
    break;
  case Mode::POSITION_ORIENT:
    z_indices.resize(6);
    R_scan_match.resize(6);
    break;
  }

  // Initialize covariance matrix based on mode.
  if (mode == Mode::POSITION || mode == Mode::POSITION_YAW || mode == Mode::POSITION_ORIENT)
  {
    double r_scan_match_pxy = 0;
    if(!nh_.getParam(prefix + "r_pxy", r_scan_match_pxy)){
        ROS_WARN("Couldn't get param \"r_pxy\". Using zero as default.");
    }
    double r_scan_match_pz = 0;
    if(!nh_.getParam(prefix + "r_pz", r_scan_match_pz)){
        ROS_WARN("Couldn't get param \"r_pz\". Using zero as default.");
    }
    R_scan_match(0) = std::pow(r_scan_match_pxy, 2); // Cleaner way?
    R_scan_match(1) = std::pow(r_scan_match_pxy, 2);
    R_scan_match(2) = std::pow(r_scan_match_pz , 2);
    z_indices.head<3>() = RigidBodyState::positionInds();
  }
  else if (mode == Mode::YAW) {
    double r_scan_match_yaw = 0;
    if(!nh_.getParam(prefix + "r_yaw", r_scan_match_yaw)){
        ROS_WARN("Couldn't get param \"r_yaw\". Using zero as default.");
    }
    R_scan_match(0) = std::pow(r_scan_match_yaw * M_PI / 180.0, 2);
    z_indices(0) = RBIS::chi_ind + 2; // z component only
  }
  else {
    double r_scan_match_vxy = 0;
    double r_scan_match_vz = 0;
    if(!nh_.getParam(prefix + "r_vxy", r_scan_match_vxy)){
        ROS_WARN("Couldn't get param \"r_vxy\". Using zero as default.");
    }
    if(!nh_.getParam(prefix + "r_vz", r_scan_match_vz)){
        ROS_WARN("Couldn't get param \"r_vz\". Using zero as default.");
    }
    R_scan_match(0) = std::pow(r_scan_match_vxy, 2); // Cleaner way?
    R_scan_match(1) = std::pow(r_scan_match_vxy, 2);
    R_scan_match(2) = std::pow(r_scan_match_vz , 2);
    z_indices.head<3>() = RigidBodyState::velocityInds();
  }

  if (mode == Mode::POSITION_YAW || mode == Mode::VELOCITY_YAW) {
    double r_scan_match_yaw = 0;
    if(!nh_.getParam(prefix + "r_yaw", r_scan_match_yaw)){
        ROS_WARN("Couldn't get param \"r_yaw\". Using zero as default.");
    }
    R_scan_match(3) = std::pow(r_scan_match_yaw * M_PI / 180.0, 2);
    z_indices(3) = RBIS::chi_ind + 2; // z component only
  }

  if(mode == Mode::POSITION_ORIENT)
  {
    double r_scan_match_rxy = 0;
    double r_scan_match_ryaw = 0;
    if(!nh_.getParam(prefix + "r_rxy", r_scan_match_rxy)){
        ROS_WARN("Couldn't get param \"r_yaw\". Using zero as default.");
    }
    if(!nh_.getParam(prefix + "r_ryaw", r_scan_match_ryaw)){
        ROS_WARN("Couldn't get param \"r_yaw\". Using zero as default.");
    }
    z_indices.tail<3>() = RigidBodyState::chiInds();
    R_scan_match(3) = std::pow(r_scan_match_rxy * M_PI / 180.0, 2);
    R_scan_match(4) = std::pow(r_scan_match_rxy * M_PI / 180.0, 2);
    R_scan_match(5) = std::pow(r_scan_match_ryaw * M_PI / 180.0, 2);
  }

  cov_scan_match = R_scan_match.asDiagonal();

  scan_matcher_module_ = ScanMatcherModule(mode, z_indices, cov_scan_match);
}

RBISUpdateInterface * ScanMatcherHandler::processMessage(const geometry_msgs::PoseWithCovarianceStamped * msg,
                                                         StateEstimator* state_estimator)
{
    poseMsgFromROS(*msg, pose_meas_);
    std::cerr << "RECEIVED POSE MEASUREMENT: " <<
                 pose_meas_.pos.transpose() <<
                 "   " <<
              rotation::getEulerAnglesDeg(pose_meas_.orientation).transpose() <<
                 std::endl;
    return scan_matcher_module_.processMessage(&pose_meas_, state_estimator);
}

bool ScanMatcherHandler::processMessageInit(const geometry_msgs::PoseWithCovarianceStamped *msg,
                                            const std::map<std::string, bool> &sensor_initialized,
                                            const RBIS &default_state,
                                            const RBIM &default_cov,
                                            RBIS &init_state,
                                            RBIM &init_cov)
{
    return true;
}

} // namespace pronto
