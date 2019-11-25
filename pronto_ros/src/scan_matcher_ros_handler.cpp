#include "pronto_ros/scan_matcher_ros_handler.hpp"
#include <string>
#include <tf_conversions/tf_eigen.h>
#include "pronto_ros/pronto_ros_conversions.hpp"

namespace pronto {

using ScanMatchingMode = ScanMatcherModule::ScanMatchingMode;

ScanMatcherHandler::ScanMatcherHandler(ros::NodeHandle& nh) : nh_(nh)
{
    ScanMatchingMode mode;
    Eigen::VectorXi z_indices;
    Eigen::MatrixXd cov_scan_match;

    std::string prefix = "scan_matcher/";
    std::string mode_str;

    if(!nh_.getParam(prefix + "mode", mode_str)){
        ROS_WARN("Couldn't get param \"mode\". Using MODE_POSITION by default.");
        mode_str = "position";
    }

  if (mode_str.compare("position") == 0) {
    mode = ScanMatchingMode::MODE_POSITION;
    std::cout << "Scan matcher will provide position measurements." << std::endl;
  }
  else if (mode_str.compare("position_yaw") == 0) {
    mode = ScanMatchingMode::MODE_POSITION_YAW;
    std::cout << "Scan matcher will provide position and yaw measurements." << std::endl;
  }
  else if (mode_str.compare("velocity") == 0) {
    mode = ScanMatchingMode::MODE_VELOCITY;
    std::cout << "Scan matcher will provide velocity measurements." << std::endl;
  }
  else if (mode_str.compare("velocity_yaw") == 0) {
    mode = ScanMatchingMode::MODE_VELOCITY_YAW;
    std::cout << "Scan matcher will provide velocity and yaw measurements." << std::endl;
  }
  else if (mode_str.compare("yaw") == 0) {
    mode = ScanMatchingMode::MODE_YAW;
    std::cout << "Scan matcher will provide yaw measurements." << std::endl;
  }
  else {
    mode = ScanMatchingMode::MODE_POSITION;
    ROS_WARN("Couldn't get param \"mode\". Using MODE_POSITION by default.");
  }
  Eigen::VectorXd R_scan_match;

  if (mode == ScanMatchingMode::MODE_POSITION || mode == ScanMatchingMode::MODE_VELOCITY) {
    z_indices.resize(3);
    R_scan_match.resize(3);
  }
  else if (mode == ScanMatchingMode::MODE_YAW) {
    z_indices.resize(1);
    R_scan_match.resize(1);
  }
  else {
    z_indices.resize(4); // Use yaw measurements too.
    R_scan_match.resize(4);
  }

  // Initialize covariance matrix based on mode.
  if (mode == ScanMatchingMode::MODE_POSITION || mode == ScanMatchingMode::MODE_POSITION_YAW)
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
    z_indices.head<3>() = eigen_utils::RigidBodyState::positionInds();
  }
  else if (mode == ScanMatchingMode::MODE_YAW) {
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
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
  }

  if (mode == ScanMatchingMode::MODE_POSITION_YAW || mode == ScanMatchingMode::MODE_VELOCITY_YAW) {
    double r_scan_match_yaw = 0;
    if(!nh_.getParam(prefix + "r_yaw", r_scan_match_yaw)){
        ROS_WARN("Couldn't get param \"r_yaw\". Using zero as default.");
    }
    R_scan_match(3) = std::pow(r_scan_match_yaw * M_PI / 180.0, 2);
    z_indices(3) = RBIS::chi_ind + 2; // z component only
  }

  cov_scan_match = R_scan_match.asDiagonal();

  scan_matcher_module_ = ScanMatcherModule(mode, z_indices, cov_scan_match);
}

RBISUpdateInterface * ScanMatcherHandler::processMessage(const geometry_msgs::PoseWithCovarianceStamped * msg,
                                                         StateEstimator* state_estimator)
{
    poseMsgFromROS(*msg, pose_meas_);
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
