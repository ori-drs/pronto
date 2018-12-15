#include "pronto_estimator_lcm/scan_matcher_handler.hpp"
#include <bot_core/math_util.h>
namespace MavStateEst {

using ScanMatchingMode = ScanMatcherModule::ScanMatchingMode;

ScanMatcherHandler::ScanMatcherHandler(BotParam * param)
{
    ScanMatchingMode mode;
    Eigen::VectorXi z_indices;
    Eigen::MatrixXd cov_scan_match;

    char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.scan_matcher.mode");

  if (strcmp(mode_str, "position") == 0) {
    mode = ScanMatchingMode::MODE_POSITION;
    std::cout << "Scan matcher will provide position measurements." << std::endl;
  }
  else if (strcmp(mode_str, "position_yaw") == 0) {
    mode = ScanMatchingMode::MODE_POSITION_YAW;
    std::cout << "Scan matcher will provide position and yaw measurements." << std::endl;
  }
  else if (strcmp(mode_str, "velocity") == 0) {
    mode = ScanMatchingMode::MODE_VELOCITY;
    std::cout << "Scan matcher will provide velocity measurements." << std::endl;
  }
  else if (strcmp(mode_str, "velocity_yaw") == 0) {
    mode = ScanMatchingMode::MODE_VELOCITY_YAW;
    std::cout << "Scan matcher will provide velocity and yaw measurements." << std::endl;
  }
  else if (strcmp(mode_str, "yaw") == 0) {
    mode = ScanMatchingMode::MODE_YAW;
    std::cout << "Scan matcher will provide yaw measurements." << std::endl;
  }
  else {
    mode = ScanMatchingMode::MODE_VELOCITY;
    std::cout << "Unrecognized scan matcher mode. Using velocity mode by default." << std::endl;
  }

  free(mode_str);
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
  if (mode == ScanMatchingMode::MODE_POSITION || mode == ScanMatchingMode::MODE_POSITION_YAW) {
    double r_scan_match_pxy = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_pxy");
    double r_scan_match_pz = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_pz");
    R_scan_match(0) = bot_sq(r_scan_match_pxy); // Cleaner way?
    R_scan_match(1) = bot_sq(r_scan_match_pxy);
    R_scan_match(2) = bot_sq(r_scan_match_pz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::positionInds();
  }
  else if (mode == ScanMatchingMode::MODE_YAW) {
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

  if (mode == ScanMatchingMode::MODE_POSITION_YAW || mode == ScanMatchingMode::MODE_VELOCITY_YAW) {
    double r_scan_match_yaw = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_yaw");
    R_scan_match(3) = bot_sq(bot_to_radians(r_scan_match_yaw));
    z_indices(3) = RBIS::chi_ind + 2; // z component only
  }

  cov_scan_match = R_scan_match.asDiagonal();

  scan_matcher_module_ = ScanMatcherModule(mode,z_indices,cov_scan_match);
}

RBISUpdateInterface * ScanMatcherHandler::processMessage(const bot_core::pose_t * msg,
                                                         MavStateEstimator* state_estimator)
{
    // convert LCM to generic format
    pose_meas_.utime = msg->utime;
    pose_meas_.linear_vel = Eigen::Map<const Eigen::Vector3d>(msg->vel);
    pose_meas_.pos = Eigen::Map<const Eigen::Vector3d>(msg->pos);
    eigen_utils::botDoubleToQuaternion(pose_meas_.orientation,msg->orientation);
    return scan_matcher_module_.processMessage(&pose_meas_,state_estimator);
}

bool ScanMatcherHandler::processMessageInit(const bot_core::pose_t *msg,
                                            const std::map<std::string, bool> &sensor_initialized,
                                            const RBIS &default_state,
                                            const RBIM &default_cov,
                                            RBIS &init_state,
                                            RBIM &init_cov)
{
    return true;
}

} // namespace MavStateEst
