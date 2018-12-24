#include "pronto_estimator_lcm/vicon_handler.hpp"
#include <pronto_conversions/pronto_conversions_bot_core.hpp>
#include <pronto_conversions/pronto_meas_lcm.hpp>

using namespace Eigen;

namespace MavStateEst {

ViconHandler::ViconHandler(BotParam * param, BotFrames * frames)
{
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.vicon.mode");
  ViconConfig cfg;
  if (strcmp(mode_str, "position") == 0) {
    cfg.mode = ViconMode::MODE_POSITION;
    std::cout << "Vicon will provide position measurements." << std::endl;
  }
  else if (strcmp(mode_str, "position_orient") == 0) {
    cfg.mode = ViconMode::MODE_POSITION_ORIENT;
    std::cout << "Vicon will provide position and orientation measurements." << std::endl;
  }
  else if (strcmp(mode_str, "orientation") == 0) {
    cfg.mode = ViconMode::MODE_ORIENTATION;
    std::cout << "Vicon will provide orientation measurements." << std::endl;
  }
  else if (strcmp(mode_str, "yaw") == 0) {
    cfg.mode = ViconMode::MODE_YAW;
    std::cout << "Vicon will provide yaw orientation measurements." << std::endl;
  }
  else {
    cfg.mode = ViconMode::MODE_POSITION;
    std::cout << "Unrecognized Vicon mode. Using position mode by default." << std::endl;
  }

  bool apply_frame = bot_param_get_boolean_or_fail(param, "state_estimator.vicon.apply_frame");
  if (apply_frame){
    char* frame_from = bot_param_get_str_or_fail(param, "state_estimator.vicon.frame_from");
    char* frame_to   = bot_param_get_str_or_fail(param, "state_estimator.vicon.frame_to");
    bot_frames_get_trans(frames, frame_from, frame_to, &body_to_vicon);
    cfg.body_to_vicon = pronto::getBotTransAsEigen(body_to_vicon);
  } else {
      cfg.body_to_vicon.setIdentity();
  }

  free(mode_str);
  cfg.r_vicon_xyz = bot_param_get_double_or_fail(param, "state_estimator.vicon.r_xyz");
  cfg.r_vicon_chi = bot_param_get_double_or_fail(param, "state_estimator.vicon.r_chi");
  vicon_module_.reset(new ViconModule(cfg));
}

RBISUpdateInterface * ViconHandler::processMessage(const bot_core::rigid_transform_t * msg, MavStateEstimator* state_estimator)
{
    rigidTransformFromLCM(*msg, vicon_transform_);
    return vicon_module_->processMessage(&vicon_transform_,state_estimator);
}

bool ViconHandler::processMessageInit(const bot_core::rigid_transform_t * msg,
                                      const std::map<std::string, bool> & sensors_initialized,
                                      const RBIS & default_state,
                                      const RBIM & default_cov,
                                      RBIS & init_state,
                                      RBIM & init_cov)
{
    rigidTransformFromLCM(*msg, vicon_transform_);
    return vicon_module_->processMessageInit(&vicon_transform_,
                                            sensors_initialized,
                                            default_state,
                                            default_cov,
                                            init_state,
                                            init_cov);
}


} // namespace MavStateEst
