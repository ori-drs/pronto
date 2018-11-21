#include "mav_state_est/vicon_handler.hpp"

using namespace Eigen;

namespace MavStateEst {

ViconHandler::ViconHandler(BotParam * param, BotFrames * frames)
{
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.vicon.mode");

  if (strcmp(mode_str, "position") == 0) {
    mode = ViconHandler::MODE_POSITION;
    std::cout << "Vicon will provide position measurements." << std::endl;
  }
  else if (strcmp(mode_str, "position_orient") == 0) {
    mode = ViconHandler::MODE_POSITION_ORIENT;
    std::cout << "Vicon will provide position and orientation measurements." << std::endl;
  }
  else if (strcmp(mode_str, "orientation") == 0) {
    mode = ViconHandler::MODE_ORIENTATION;
    std::cout << "Vicon will provide orientation measurements." << std::endl;
  }
  else if (strcmp(mode_str, "yaw") == 0) {
    mode = ViconHandler::MODE_YAW;
    std::cout << "Vicon will provide yaw orientation measurements." << std::endl;
  }
  else {
    mode = ViconHandler::MODE_POSITION;
    std::cout << "Unrecognized Vicon mode. Using position mode by default." << std::endl;
  }

  apply_frame = bot_param_get_boolean_or_fail(param, "state_estimator.vicon.apply_frame");
  if (apply_frame){
    char* frame_from = bot_param_get_str_or_fail(param, "state_estimator.vicon.frame_from");
    char* frame_to   = bot_param_get_str_or_fail(param, "state_estimator.vicon.frame_to");
    bot_frames_get_trans(frames, frame_from, frame_to, &body_to_vicon);
  }

  free(mode_str);
  init(param);
}

ViconHandler::ViconHandler(BotParam * param, ViconMode vicon_mode)
{
  init(param);
}

void ViconHandler::init(BotParam * param)
{
  // Build full covariance matrix - we may only use part of it.
  double r_vicon_xyz = bot_param_get_double_or_fail(param, "state_estimator.vicon.r_xyz");
  double r_vicon_chi = bot_param_get_double_or_fail(param, "state_estimator.vicon.r_chi");

  cov_vicon = Eigen::MatrixXd::Zero(6, 6);
  cov_vicon.topLeftCorner<3, 3>() = pow(r_vicon_xyz, 2) * Eigen::Matrix3d::Identity();
  cov_vicon.bottomRightCorner<3, 3>() = pow(bot_to_radians(r_vicon_chi), 2) * Eigen::Matrix3d::Identity();

  if (mode == MODE_POSITION) {
    z_indices = RBIS::positionInds();
  } else if (mode == MODE_YAW){
      z_indices.resize(1);
      z_indices(0) = RBIS::chi_ind + 2; // z component only
      cov_vicon.resize(1,1);
      cov_vicon(0,0) = pow(bot_to_radians(r_vicon_chi), 2);
  } else if (mode == MODE_ORIENTATION){
      z_indices.resize(3);
      z_indices = RBIS::chiInds();
      cov_vicon.resize(3,3);
      cov_vicon = pow(bot_to_radians(r_vicon_chi), 2) * Eigen::Matrix3d::Identity();
  } else {
    z_indices.resize(6);
    z_indices.head<3>() = RBIS::positionInds();
    z_indices.tail<3>() = RBIS::chiInds();
  }
}

RBISUpdateInterface * ViconHandler::processMessage(const bot_core::rigid_transform_t * msg, MavStateEstimator* state_estimator)
{

  BotTrans local_to_vicon;
  memset(&local_to_vicon, 0, sizeof(local_to_vicon));
  memcpy(local_to_vicon.trans_vec, msg->trans, 3*sizeof(double));
  memcpy(local_to_vicon.rot_quat, msg->quat, 4*sizeof(double));

  BotTrans local_to_body;
  memset(&local_to_body, 0, sizeof(local_to_body));
  if (apply_frame){
    bot_trans_apply_trans_to( &local_to_vicon,&body_to_vicon, &local_to_body);
  }else{
    bot_trans_copy(&local_to_body, &local_to_vicon);
  }
  ////////////////////////////////////

  if ((Eigen::Map<const Eigen::Array3d>( local_to_vicon.trans_vec ).abs() < 1e-5).all())
    return NULL;

  if (mode == MODE_POSITION) {
      return new RBISIndexedMeasurement(z_indices, Eigen::Map<const Eigen::Vector3d>( local_to_body.trans_vec ),
                                        cov_vicon.block<3, 3>(0, 0), RBISUpdateInterface::vicon, msg->utime);
  } else if(mode == MODE_YAW) {

      Eigen::VectorXd z_meas(1);
      Eigen::Quaterniond quat;
      eigen_utils::botDoubleToQuaternion(quat, local_to_body.rot_quat);

      return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                       z_meas,
                                                       cov_vicon,
                                                       quat,
                                                       RBISUpdateInterface::vicon,
                                                       msg->utime);
  }  else if(mode == MODE_ORIENTATION){
      Eigen::VectorXd z_meas(3);
      Eigen::Quaterniond quat;
      eigen_utils::botDoubleToQuaternion(quat, local_to_body.rot_quat);

      return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                       z_meas,
                                                       cov_vicon,
                                                       quat,
                                                       RBISUpdateInterface::vicon,
                                                       msg->utime);
  }  else if(mode == MODE_POSITION_ORIENT){
      Eigen::VectorXd z_meas(6);
      Eigen::Quaterniond quat;
      eigen_utils::botDoubleToQuaternion(quat, local_to_body.rot_quat );

      z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(local_to_body.trans_vec);

      return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                       z_meas,
                                                       cov_vicon,
                                                       quat,
                                                       RBISUpdateInterface::vicon,
                                                       msg->utime);
  }
}

bool ViconHandler::processMessageInit(const bot_core::rigid_transform_t * msg,
    const std::map<std::string, bool> & sensors_initialized
    , const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov)
{

  BotTrans local_to_vicon;
  memset(&local_to_vicon, 0, sizeof(local_to_vicon));
  memcpy(local_to_vicon.trans_vec, msg->trans, 3*sizeof(double));
  memcpy(local_to_vicon.rot_quat, msg->quat, 4*sizeof(double));

  BotTrans local_to_body;
  memset(&local_to_body, 0, sizeof(local_to_body));
  if (apply_frame){
    bot_trans_apply_trans_to( &local_to_vicon,&body_to_vicon, &local_to_body);
  }else{
    bot_trans_copy(&local_to_body, &local_to_vicon);
  }
  ////////////////////////////////////

  init_state.utime = msg->utime;

  init_state.position() = Eigen::Map<const Eigen::Vector3d>(local_to_body.trans_vec);
  eigen_utils::botDoubleToQuaternion(init_state.orientation(), local_to_body.rot_quat);

  init_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = cov_vicon.topLeftCorner<3, 3>();
  init_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = cov_vicon.bottomRightCorner<3, 3>();

  Vector3d init_rpy_deg = bot_to_degrees(init_state.getEulerAngles());

  fprintf(stderr, "initialized position using VICON at xyz: %f,%f,%f\n", init_state.position()(0),
      init_state.position()(1), init_state.position()(2));
  fprintf(stderr, "initialized orientation using VICON at rpy: %f,%f,%f\n", init_rpy_deg(0),
      init_rpy_deg(1), init_rpy_deg(2));

  return true;
}
} // namespace MavStateEst
