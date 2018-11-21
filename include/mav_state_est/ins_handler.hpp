#pragma once
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <string>
#include "mav_state_est/rbis_update_interface.hpp"
#include <lcmtypes/bot_core/ins_t.hpp>
#include "mav_state_est/mav_state_est.hpp"


namespace MavStateEst {

class InsHandler {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  void create(BotParam * _param, BotFrames * _frames);
  public:
  InsHandler(BotParam * _param, BotFrames * _frames);

  // channel is used to determine which signal to subscribe to:
  std::string channel;

  // Microstrain Functions:
  RBISUpdateInterface * processMessage(const bot_core::ins_t * msg,
                                       MavStateEstimator* state_estimator);

  bool processMessageInit(const bot_core::ins_t * msg,
                          const std::map<std::string, bool> & sensors_initialized,
                          const RBIS & default_state,
                          const RBIM & default_cov,
                          RBIS & init_state,
                          RBIM & init_cov);
  inline double getTimeStep() {
      return dt;
  }

protected:
  // Common Initialization Function:
  bool processMessageInitCommon(const std::map<std::string, bool> & sensors_initialized,
                                const RBIS & default_state,
                                const RBIM & default_cov,
                                RBIS & init_state,
                                RBIM & init_cov,
                                RBISIMUProcessStep * update,
                                Eigen::Vector3d mag_vec);
protected:
  BotTrans ins_to_body;

  double cov_accel;
  double cov_gyro;
  double cov_accel_bias;
  double cov_gyro_bias;

  double dt;

  //initialization
  int num_to_init;
  double max_initial_gyro_bias;
  int init_counter;

  Eigen::Vector3d g_vec_sum;
  Eigen::Vector3d mag_vec_sum;
  Eigen::Vector3d gyro_bias_sum;

  ////////////////////////
  bool gyro_bias_update_online;
  Eigen::Vector3d gyro_bias_initial;
  bool gyro_bias_recalc_at_start;

  bool accel_bias_update_online;
  Eigen::Vector3d accel_bias_initial;
  bool accel_bias_recalc_at_start;

};

} // namespace MavStateEst
