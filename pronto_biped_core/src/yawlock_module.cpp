#include "pronto_biped_core/yawlock_module.hpp"

namespace pronto {
namespace biped {

YawLockModule::YawLockModule(BipedForwardKinematics& fk,
                             const YawLockConfig &cfg,
                             const Transform &ins_to_body) :
    yaw_lock_(fk), mode(cfg.mode), ins_to_body_(ins_to_body)
{



  yaw_lock_.setParameters(cfg.correction_period,
                          cfg.yaw_slip_detect,
                          cfg.yaw_slip_threshold_degrees,
                          cfg.yaw_slip_disable_period);

  yaw_lock_.setStandingLinks(cfg.left_standing_link, cfg.right_standing_link);

  // TODO behaviour status should be replaced with something else
  /*
  std::string behavior_channel = bot_param_get_str_or_fail(param, "state_estimator.yawlock.behavior_channel");
  if (behavior_channel == "CONTROLLER_STATUS"){
    // MIT controller:
    lcm_recv->subscribe( "CONTROLLER_STATUS" ,&YawLockModule::controllerStatusHandler,this);
  }else if (behavior_channel == "ROBOT_BEHAVIOR") {
    // ROBOT_BEHAVIOR comes from IHMC or BDI API
    lcm_recv->subscribe( "ROBOT_BEHAVIOR" ,&YawLockModule::robotBehaviorHandler,this);
  }else{
    std::cout << "behavior_channel not recognised: CONTROLLER_STATUS or ROBOT_BEHAVIOR\n";
    exit(-1);
  }
  */

  // Bias Estimation
  // TODO replace subscription IMU with something else
  /*
  char * ins_frame = bot_param_get_str_or_fail(param, "state_estimator.ins.frame");
  bot_frames_get_trans(frames, ins_frame, "body", &ins_to_body);
  free(ins_frame);
  */

  Eigen::VectorXd R_scan_match;

  switch(mode){
  case YawLockMode::YAWBIAS:
      z_indices.resize(1);
      z_meas.resize(1);
      R_scan_match.resize(1);
      // Units (rads per sec)^2
      R_scan_match(0) = std::pow(cfg.r_yaw_bias * M_PI / 180.0, 2);
      z_indices(0) = RBIS::gyro_bias_ind + 2; // z component only
      break;
  case YawLockMode::YAW:
      z_indices.resize(1);
      z_meas.resize(1);
      R_scan_match.resize(1);
      R_scan_match(0) = std::pow(cfg.r_yaw * M_PI / 180.0, 2);
      z_indices(0) = RBIS::chi_ind + 2; // z component only
      break;
  case YawLockMode::YAWBIAS_YAW:
      z_indices.resize(2);
      // TODO understand why in the original code this data was always 1-dim
      z_meas.resize(1);
      R_scan_match.resize(2);
      // Units (rads per sec)^2
      R_scan_match(0) = std::pow((cfg.r_yaw_bias) * M_PI / 180.0, 2);
      R_scan_match(1) = std::pow((cfg.r_yaw) * M_PI / 180.0, 2);
      // Must by gyro bias followed by chi/yaw
      z_indices(0) = RBIS::gyro_bias_ind + 2; // z component only
      z_indices(1) = RBIS::chi_ind + 2; // z component only
      break;
  default:
      break;
  }
  cov_scan_match = R_scan_match.asDiagonal();
}

void YawLockModule::processSecondaryMessage(const pronto::JointState & msg){
  if (mode == YawLockMode::YAW || mode == YawLockMode::YAWBIAS_YAW)
  {
    yaw_lock_.setJointState(msg.joint_position, msg.joint_name);
  }
}

void YawLockModule::addControlStatus(const ControlStatus &ctrl_status) {
    if(ctrl_status == ControlStatus::STANDING ||
            ctrl_status == ControlStatus::MANIPULATING) {
        yaw_lock_.setIsRobotStanding(true);
    } else {
        yaw_lock_.setIsRobotStanding(false);
    }
}

RBISUpdateInterface * YawLockModule::processMessage(const ImuMeasurement *msg,
                                                    StateEstimator* state_estimator){
  body_gyro = ins_to_body_.rotation() * msg->omega;


  state_estimator->getHeadState(head_state, head_cov);

  // Get the Yaw Rate Bias Estimate:
  if (yaw_lock_.isRobotStanding()) {
    z_meas(0) = body_gyro(2);
  } else {
    z_meas(0) = head_state.gyroBias()(2);
  }

  yawLockValid = false;
  // Get the Yaw estimate:
  if (mode == YawLockMode::YAW || mode == YawLockMode::YAWBIAS_YAW)
  {
    yawLockValid = yaw_lock_.getCorrection(head_state.getPoseAsIsometry3d(),
                                           msg->utime,
                                           world_to_body_quat_lock);
  }

  switch(mode) {
  case YawLockMode::YAWBIAS:
      return new RBISIndexedMeasurement(z_indices,
                                        z_meas,
                                        cov_scan_match,
                                        RBISUpdateInterface::yawlock,
                                        msg->utime);
  case YawLockMode::YAW:
      if (yawLockValid) {
          return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                           z_meas,
                                                           cov_scan_match,
                                                           world_to_body_quat_lock,
                                                           RBISUpdateInterface::yawlock,
                                                           msg->utime);
      } else {
        return nullptr;
      }
  case YawLockMode::YAWBIAS_YAW:
      if (yawLockValid) {
        return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                         z_meas,
                                                         cov_scan_match,
                                                         world_to_body_quat_lock,
                                                         RBISUpdateInterface::yawlock,
                                                         msg->utime);
      } else {
        // If want to correct but yaw_lock is invalid,
        // only return a bias measurement
        return new RBISIndexedMeasurement(z_indices.head<1>(),
                                          z_meas.head<1>(),
                                          cov_scan_match.topLeftCorner<1,1>(),
                                          RBISUpdateInterface::yawlock,
                                          msg->utime);
      }
  }
}

bool YawLockModule::processMessageInit(const ImuMeasurement* msg,
                                       const std::map<std::string, bool> &sensor_initialized,
                                       const RBIS &default_state,
                                       const RBIM &default_cov,
                                       RBIS &init_state,
                                       RBIM &init_cov)
{
    // do nothing for now
    return true;
}


}
} // end of namespace
