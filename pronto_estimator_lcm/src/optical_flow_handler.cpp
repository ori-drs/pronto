#include "pronto_estimator_lcm/optical_flow_handler.hpp"

namespace MavStateEst {

OpticalFlowHandler::OpticalFlowHandler(BotParam * param, BotFrames * frames)
{
  bot_frames_get_trans(frames, "body", "camera", &body_to_cam);
  bot_trans_get_trans_vec(&body_to_cam, body_to_cam_trans.data());
  bot_trans_get_rot_mat_3x3(&body_to_cam, body_to_cam_rot.data());
//  cam_to_body_trans = cam_to_body_rot * cam_to_body_trans;

  Eigen::Vector4d R_optical_flow_xyrs;

  double r_optical_flow_x = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_ux");
  double r_optical_flow_y = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_uy");
  double r_optical_flow_r = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_r");
  double r_optical_flow_s = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_s");

  R_optical_flow_xyrs
  << Eigen::Array4d(r_optical_flow_x, r_optical_flow_y, r_optical_flow_r, r_optical_flow_s).square();
  cov_xyrs = R_optical_flow_xyrs.asDiagonal();
}

RBISUpdateInterface * OpticalFlowHandler::processMessage(const pronto::optical_flow_t * msg, MavStateEstimator* state_estimator)
{
// Camera frame to body frame transform.
  z_xyrs(0) = msg->ux;
  z_xyrs(1) = msg->uy;
  z_xyrs(2) = msg->theta;
  z_xyrs(3) = msg->scale;

  double alpha1 = msg->alpha1;
  double alpha2 = msg->alpha2;
  double gamma = msg->gamma;

//  eigen_dump(z_xyrs);
//  eigen_dump(cov_xyrs);
//  eigen_dump(cam_to_body_trans);
//  eigen_dump(cam_to_body_rot);
  return new RBISOpticalFlowMeasurement(z_xyrs, cov_xyrs, body_to_cam_trans, body_to_cam_rot, alpha1, alpha2, gamma,
      RBISUpdateInterface::optical_flow, msg->utime);
}

} // namespace MavStateEst
