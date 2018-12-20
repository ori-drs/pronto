#pragma once

#include "pronto_estimator_lcm/lcm_front_end.hpp"
#include <pronto_estimator_core/sensing_module.hpp>
#include <pronto_estimator_core/pose_meas_module.hpp>
#include <eigen_utils/eigen_utils.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <memory>

namespace MavStateEst {

class PoseMeasHandler : SensingModule<bot_core::pose_t>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  PoseMeasHandler(BotParam * param);

  RBISUpdateInterface * processMessage(const bot_core::pose_t * msg,
                                       MavStateEstimator* state_estimator);

  bool processMessageInit(const bot_core::pose_t * msg,
                          const std::map<std::string, bool> & sensors_initialized,
                          const RBIS & default_state,
                          const RBIM & default_cov,
                          RBIS & init_state,
                          RBIM & init_cov);
protected:
  void poseMeasurementFromLCM(const bot_core::pose_t& lcm_msg,
                              PoseMeasurement& msg);

  void init(BotParam * param);
  std::shared_ptr<PoseMeasModule> pose_module_;
  PoseMeasurement pose_msg_;
};

}//namespace
