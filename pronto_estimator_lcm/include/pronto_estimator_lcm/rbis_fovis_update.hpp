#ifndef RBIS_FOVIS_LIB_UPDATE_HPP_
#define RBIS_FOVIS_LIB_UPDATE_HPP_

#include <lcm/lcm.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

// #include <pronto_vis/pronto_vis.hpp>
#include <pronto_conversions/pronto_conversions_lcm.hpp>
#include <pronto_conversions/pronto_conversions_bot_core.hpp>
#include <pronto_estimator_core/rbis_update_interface.hpp>
#include <pronto_estimator_lcm/sensor_handlers.hpp>

#include <lcmtypes/pronto/update_t.hpp>
#include <lcm/lcm-cpp.hpp>

#include <pronto_estimator_core/visual_odometry_module.hpp>
#include <pronto_estimator_core/definitions.hpp>
#include <memory>

namespace MavStateEst {

class FovisHandler : SensingModule<pronto::update_t>{
public:
  FovisHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub,
               BotParam * param, BotFrames * frames);

  RBISUpdateInterface * processMessage(const pronto::update_t  * msg, MavStateEstimator* state_estimator);
  bool processMessageInit(const pronto::update_t *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov);
private:
  lcm::LCM* lcm_pub;
  lcm::LCM* lcm_recv;
  BotFrames* frames;
  // pronto_vis* pc_vis_;
  
  // both duplicated in leg odom
  BotTrans getTransAsVelocityTrans(BotTrans msgT,
           int64_t utime, int64_t prev_utime);  
  
  void sendTransAsVelocityPose(BotTrans msgT, int64_t utime, int64_t prev_utime, std::string channel);
  Eigen::Isometry3d t0_body = Eigen::Isometry3d::Identity();
  std::shared_ptr<VisualOdometryModule> vo_module_;
  VisualOdometryUpdate vo_update_;
  Eigen::Quaterniond temp_quat;

  /**
   * @brief getVisualOdometryUpdateFromLCM takes an LCM message and a pose
   * and converts them into a VisualOdometryUpdate, which contains the time t0
   * the pose of the robot at t0, the time t1 and the relative pose between
   * t0 and t1, plus covariance and status.
   * @param[in] lcm_update
   * @param[in] body_to_local
   * @param[out] vo_update
   */
  void getVisualOdometryUpdateFromLCM(const pronto::update_t& lcm_update,
                                        const Eigen::Affine3d& body_to_local,
                                        VisualOdometryUpdate& vo_update);




};


}
#endif /* RBIS_FOVIS_LIB_UPDATE_HPP_ */

