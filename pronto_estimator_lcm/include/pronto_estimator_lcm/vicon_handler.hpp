#pragma once
#include <pronto_estimator_core/rbis_update_interface.hpp>
#include <pronto_estimator_core/mav_state_est.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>


namespace MavStateEst {
class ViconHandler {
public:
    // required by all classes with an Eigen type member
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  typedef enum {
    MODE_POSITION, MODE_POSITION_ORIENT, MODE_ORIENTATION, MODE_YAW
  } ViconMode;

  ViconHandler(BotParam * param, BotFrames *frames);

  ViconHandler(BotParam * param, ViconMode vicon_mode);

  void init(BotParam * param);

  RBISUpdateInterface * processMessage(const bot_core::rigid_transform_t * msg,
                                       MavStateEstimator* state_estimator);

  bool processMessageInit(const bot_core::rigid_transform_t * msg,
                          const std::map<std::string, bool> & sensors_initialized,
                          const RBIS & default_state,
                          const RBIM & default_cov,
                          RBIS & init_state,
                          RBIM & init_cov);

private:
  // added mfallon, to allow a plate-to-body transform
  BotTrans body_to_vicon;
  bool apply_frame;

  ViconMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_vicon;
};
}
