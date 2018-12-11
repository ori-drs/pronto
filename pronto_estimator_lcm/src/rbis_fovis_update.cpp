#include "pronto_estimator_lcm/rbis_fovis_update.hpp"
#include <lcm/lcm-cpp.hpp>

namespace MavStateEst {

FovisHandler::FovisHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub,
                           BotParam * param, BotFrames * frames): lcm_recv(lcm_recv), lcm_pub(lcm_pub), frames(frames){
  
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.fovis.mode");

  VisualOdometryConfig cfg;

  if (strcmp(mode_str, "position") == 0){
    cfg.mode = VisualOdometryMode::MODE_POSITION;
    std::cout << "FOVIS will provide position corrections." << std::endl;
  }
  else if (strcmp(mode_str, "position_orient") == 0){
    cfg.mode = VisualOdometryMode::MODE_POSITION_ORIENT;
    std::cout << "FOVIS will provide position and orientation corrections." << std::endl;
  }
  else{
    std::cout << "FOVIS mode not understood "<< mode_str << ", exiting\n";
    exit(-1);
    // ... incomplete...
  }
  free(mode_str);
  
  Eigen::VectorXd R_fovis;
  if (cfg.mode == VisualOdometryMode::MODE_POSITION){
    cfg.z_indices.resize(3);
    R_fovis.resize(3);
  }
  else if (cfg.mode == VisualOdometryMode::MODE_POSITION_ORIENT){
    cfg.z_indices.resize(6);
    R_fovis.resize(6);
  }
  else{
    // ... incomplete...
  }

  // Initialize covariance matrix based on mode.
  if (cfg.mode == VisualOdometryMode::MODE_POSITION){
    double R_fovis_pxyz = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_pxyz");
    R_fovis(0) = std::pow(R_fovis_pxyz, 2);
    R_fovis(1) = std::pow(R_fovis_pxyz, 2);
    R_fovis(2) = std::pow(R_fovis_pxyz, 2);
    cfg.z_indices.head<3>() = eigen_utils::RigidBodyState::positionInds();

  }else if (cfg.mode == VisualOdometryMode::MODE_POSITION_ORIENT){
    double R_fovis_pxyz = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_pxyz");
    R_fovis(0) = std::pow(R_fovis_pxyz, 2);
    R_fovis(1) = std::pow(R_fovis_pxyz, 2);
    R_fovis(2) = std::pow(R_fovis_pxyz, 2);
    cfg.z_indices.head<3>() = eigen_utils::RigidBodyState::positionInds();

    double R_fovis_chi = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_chi");
    R_fovis(3) = std::pow(R_fovis_chi, 2);
    R_fovis(4) = std::pow(R_fovis_chi, 2);
    R_fovis(5) = std::pow(R_fovis_chi, 2);
    cfg.z_indices.tail<3>() = eigen_utils::RigidBodyState::chiInds();

  }else{
    // ..incomplete
  }

  cfg.cov_vo = R_fovis.asDiagonal();

  vo_module_.reset(new VisualOdometryModule(cfg));

}


int get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        Eigen::Isometry3d & mat){
  int status;
  double matx[16];
  status = bot_frames_get_trans_mat_4x4_with_utime( bot_frames, from_frame,  to_frame, utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i,j) = matx[i*4+j];
    }
  }
  return status;
}

// NOTE: this inserts the BotTrans trans_vec 
// as the velocity components in the pose 
// [duplicated in rbis_legodo_common.cpp]
bot_core::pose_t getBotTransAsBotPoseVelocity(BotTrans bt, int64_t utime ){
  bot_core::pose_t pose;
  pose.utime = utime;
  pose.pos[0] = 0;
  pose.pos[1] = 0;
  pose.pos[2] = 0;
  pose.orientation[0] = 0;
  pose.orientation[1] = 0;
  pose.orientation[2] = 0;
  pose.orientation[3] = 0;
  pose.vel[0] = bt.trans_vec[0];
  pose.vel[1] = bt.trans_vec[1];
  pose.vel[2] = bt.trans_vec[2];
  pose.rotation_rate[0] = 0;
  pose.rotation_rate[1] = 0;
  pose.rotation_rate[2] = 0;
  return pose;
}



RBISUpdateInterface * FovisHandler::processMessage(const pronto::update_t * msg, MavStateEstimator* state_estimator)
{
    if(!get_trans_with_utime(frames, "body" , "local", msg->prev_timestamp, t0_body)){
        // if the transform can't be retrieved, we quietly return an invalid update
        return NULL;
    }
    // we will pass to the module both the messate and the transform
    // the vo_update contains also t0_body (the pose of the robot at the previous
    // time
    getVisualOdometryUpdateFromLCM(*msg, t0_body, vo_update_);
    return vo_module_->processMessage(&vo_update_, state_estimator);
}

bool FovisHandler::processMessageInit(const pronto::update_t *msg,
                                      const std::map<std::string, bool> &sensor_initialized,
                                      const RBIS &default_state,
                                      const RBIM &default_cov,
                                      RBIS &init_state,
                                      RBIM &init_cov){
    // we don't use this module to initialize for now
    return true;
}


/// Everything below is duplicated in rbis_legodo_common.cpp
void printTrans(BotTrans bt, std::string message){
  std::cout << message << ": "
      << bt.trans_vec[0] << ", " << bt.trans_vec[1] << ", " << bt.trans_vec[2] << " | "
      << bt.rot_quat[0] << ", " << bt.rot_quat[1] << ", " << bt.rot_quat[2] << ", " << bt.rot_quat[3] << "\n";
}


// Difference the transform and scale by elapsed time:
BotTrans FovisHandler::getTransAsVelocityTrans(BotTrans msgT, int64_t utime, int64_t prev_utime){
  Eigen::Isometry3d msgE = pronto::getBotTransAsEigen(msgT);
  Eigen::Isometry3d msgE_vel = pronto::getDeltaAsVelocity(msgE, (utime-prev_utime) );
  BotTrans msgT_vel;
  msgT_vel = pronto::getEigenAsBotTrans(msgE_vel);
 
  return msgT_vel;
}


/// Publishing Functions 
// Convert the delta position into a velocity 
// as a bot_pose message for visualization with signal scope:
void FovisHandler::sendTransAsVelocityPose(BotTrans msgT, int64_t utime, int64_t prev_utime, std::string channel){
  BotTrans msgT_vel = getTransAsVelocityTrans(msgT, utime, prev_utime);
  bot_core::pose_t vel_pose = getBotTransAsBotPoseVelocity(msgT_vel, utime)  ;
  lcm_pub->publish(channel, &vel_pose );
}

void FovisHandler::getVisualOdometryUpdateFromLCM(const pronto::update_t &lcm_update,
                                             const Eigen::Affine3d &body_to_local,
                                             VisualOdometryUpdate &vo_update)
{
    vo_update.curr_utime = lcm_update.timestamp;
    vo_update.pose_covariance = Eigen::Map<const Eigen::Matrix<double,6,6>>(&lcm_update.covariance[0][0]);

    vo_update.prev_pose = body_to_local;
    vo_update.prev_utime = lcm_update.prev_timestamp;
    vo_update.relative_pose.setIdentity();
    vo_update.relative_pose.translate(Eigen::Map<const Eigen::Vector3d>(lcm_update.translation));
    temp_quat = Eigen::Quaterniond::Identity();
    temp_quat.w() = lcm_update.rotation[0];
    temp_quat.x() = lcm_update.rotation[1];
    temp_quat.y() = lcm_update.rotation[2];
    temp_quat.z() = lcm_update.rotation[3];
    vo_update.relative_pose.rotate(temp_quat);
    vo_update.status = static_cast<VisualOdometryUpdate::Status>(lcm_update.estimate_status);
}

} // end of namespace
