#ifndef RBIS_FOVIS_LIB_UPDATE_HPP_
#define RBIS_FOVIS_LIB_UPDATE_HPP_

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

// #include <pronto_vis/pronto_vis.hpp>
#include <pronto_conversions/pronto_conversions_lcm.hpp>
#include <pronto_conversions/pronto_conversions_bot_core.hpp>
#include <pronto_estimator_core/rbis_update_interface.hpp>
#include <pronto_estimator_lcm/sensor_handlers.hpp>

#include <lcmtypes/pronto/update_t.hpp>
#include <lcm/lcm-cpp.hpp>

namespace MavStateEst {

class FovisHandler {
public:
  // Typical mode is MODE_POSITION OR MODE_
  typedef enum {
    MODE_VELOCITY, MODE_ROTATION_RATE, MODE_VELOCITY_ROTATION_RATE, MODE_POSITION, MODE_POSITION_ORIENT
  } FovisMode;

  FovisHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub,
               BotParam * param, BotFrames * frames);

  RBISUpdateInterface * processMessage(const pronto::update_t  * msg, MavStateEstimator* state_estimator);

  FovisMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_fovis;
  
  lcm::LCM* lcm_pub;
  lcm::LCM* lcm_recv;
  BotFrames* frames;
  // pronto_vis* pc_vis_;
  
  // both duplicated in leg odom
  BotTrans getTransAsVelocityTrans(BotTrans msgT,
           int64_t utime, int64_t prev_utime);  
  
  void sendTransAsVelocityPose(BotTrans msgT, int64_t utime, int64_t prev_utime, std::string channel);
  
  // Publish Debug Data to LCM
  bool publish_diagnostics_;  

  Eigen::Isometry3d prev_t0_body_;
  Eigen::Isometry3d prev_t0_body_internal_;
  int64_t prev_t0_body_utime_;
};


}
#endif /* RBIS_FOVIS_LIB_UPDATE_HPP_ */

