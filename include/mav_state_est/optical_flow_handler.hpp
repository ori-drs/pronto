#pragma once

#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include "mav_state_est/rbis_update_interface.hpp"
#include <lcmtypes/pronto/optical_flow_t.hpp>
#include "mav_state_est/mav_state_est.hpp"

namespace MavStateEst {
class OpticalFlowHandler {
public:
  OpticalFlowHandler(BotParam * param, BotFrames * frames);
  RBISUpdateInterface * processMessage(const pronto::optical_flow_t * msg,
                                       MavStateEstimator* state_estimator);

  BotTrans body_to_cam;
  Eigen::Vector3d body_to_cam_trans; // In body frame, not camera frame
  Eigen::Matrix3d body_to_cam_rot;

  Eigen::Vector4d z_xyrs; // data storage vector
  Eigen::Matrix4d cov_xyrs; // x, y, rot, scale
};
} // namespace MavStateEst
