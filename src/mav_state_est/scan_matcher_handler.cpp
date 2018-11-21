#include "mav_state_est/scan_matcher_handler.hpp"

namespace MavStateEst {

ScanMatcherHandler::ScanMatcherHandler(BotParam * param)
{
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.scan_matcher.mode");

  if (strcmp(mode_str, "position") == 0) {
    mode = ScanMatcherHandler::MODE_POSITION;
    std::cout << "Scan matcher will provide position measurements." << std::endl;
  }
  else if (strcmp(mode_str, "position_yaw") == 0) {
    mode = ScanMatcherHandler::MODE_POSITION_YAW;
    std::cout << "Scan matcher will provide position and yaw measurements." << std::endl;
  }
  else if (strcmp(mode_str, "velocity") == 0) {
    mode = ScanMatcherHandler::MODE_VELOCITY;
    std::cout << "Scan matcher will provide velocity measurements." << std::endl;
  }
  else if (strcmp(mode_str, "velocity_yaw") == 0) {
    mode = ScanMatcherHandler::MODE_VELOCITY_YAW;
    std::cout << "Scan matcher will provide velocity and yaw measurements." << std::endl;
  }
  else if (strcmp(mode_str, "yaw") == 0) {
    mode = ScanMatcherHandler::MODE_YAW;
    std::cout << "Scan matcher will provide yaw measurements." << std::endl;
  }
  else {
    mode = ScanMatcherHandler::MODE_VELOCITY;
    std::cout << "Unrecognized scan matcher mode. Using velocity mode by default." << std::endl;
  }

  free(mode_str);
  Eigen::VectorXd R_scan_match;

  if (mode == MODE_POSITION || mode == MODE_VELOCITY) {
    z_indices.resize(3);
    R_scan_match.resize(3);
  }
  else if (mode == MODE_YAW) {
    z_indices.resize(1);
    R_scan_match.resize(1);
  }
  else {
    z_indices.resize(4); // Use yaw measurements too.
    R_scan_match.resize(4);
  }

  // Initialize covariance matrix based on mode.
  if (mode == MODE_POSITION || mode == MODE_POSITION_YAW) {
    double r_scan_match_pxy = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_pxy");
    double r_scan_match_pz = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_pz");
    R_scan_match(0) = bot_sq(r_scan_match_pxy); // Cleaner way?
    R_scan_match(1) = bot_sq(r_scan_match_pxy);
    R_scan_match(2) = bot_sq(r_scan_match_pz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::positionInds();
  }
  else if (mode == MODE_YAW) {
    double r_scan_match_yaw = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_yaw");
    R_scan_match(0) = bot_sq(bot_to_radians(r_scan_match_yaw));
    z_indices(0) = RBIS::chi_ind + 2; // z component only
  }
  else {
    double r_scan_match_vxy = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_vxy");
    double r_scan_match_vz = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_vz");
    R_scan_match(0) = bot_sq(r_scan_match_vxy); // Cleaner way?
    R_scan_match(1) = bot_sq(r_scan_match_vxy);
    R_scan_match(2) = bot_sq(r_scan_match_vz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
  }

  if (mode == MODE_POSITION_YAW || mode == MODE_VELOCITY_YAW) {
    double r_scan_match_yaw = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_yaw");
    R_scan_match(3) = bot_sq(bot_to_radians(r_scan_match_yaw));
    z_indices(3) = RBIS::chi_ind + 2; // z component only
  }

  cov_scan_match = R_scan_match.asDiagonal();
}

RBISUpdateInterface * ScanMatcherHandler::processMessage(const bot_core::pose_t * msg, MavStateEstimator* state_estimator)
{

  if (mode == MODE_POSITION) {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
        Eigen::Map<const Eigen::Vector3d>(msg->pos), cov_scan_match, RBISUpdateInterface::scan_matcher,
        msg->utime);
  }
  else if (mode == MODE_VELOCITY) {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
        Eigen::Map<const Eigen::Vector3d>(msg->vel), cov_scan_match, RBISUpdateInterface::scan_matcher,
        msg->utime);
  }
  else if (mode == MODE_YAW) {
    Eigen::Vector4d z_meas = Eigen::Vector4d(0,0,0,0); // unused, I believe
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, msg->orientation);
    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_scan_match, quat,
        RBISUpdateInterface::scan_matcher, msg->utime);
  }
  else if (mode == MODE_POSITION_YAW || mode == MODE_VELOCITY_YAW) {
    Eigen::Vector4d z_meas;
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, msg->orientation);

    if (mode == MODE_POSITION_YAW) {
      z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->pos);
    }
    else {
      z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->vel);
    }

    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_scan_match, quat,
        RBISUpdateInterface::scan_matcher, msg->utime);
  }
}

} // namespace MavStateEst
