#pragma once

#include <lcm/lcm-cpp.hpp>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <map>
#include <set>

#include <Eigen/Dense>

#include <lcmtypes/bot_core/utime_t.hpp>
#include <lcmtypes/pronto/filter_state_t.hpp>

//TODO remove when front end is templated on FilterState and Update
#include "mav_state_est/rbis.hpp"
#include "mav_state_est/rbis_update_interface.hpp"
#include "mav_state_est/mav_state_est.hpp"
#include "mav_state_est/sensor_handler_interface.hpp"


// this forward declaration is necessary because there is a circular dependency
// between SensorHandler and LCMFrontEnd
template<class lcmType, class SensorHandlerClass>
class SensorHandler;

namespace MavStateEst {
class LCMFrontEnd {
public:
  /**
   * Constructor initializes all the static variables
   *
   * @param[in]  in_log_fname   log file name to run from, "" means live lcm subscribing
   * @param[in]  out_log_fname  log file name to run to, "" means live lcm publishing
   * @param[in]  param_fname    param file name to run from, "" means get from lcm
   * @param[in]  param_override_str    override param string
   * @param[in]  begin_timestamp lcm logfile message timestamp to start from. 0= start (ie from unixtime=0)
   */
  LCMFrontEnd(const std::string & in_log_fname,
              const std::string & out_log_fname = "",
              const std::string & param_fname = "",
              const std::string & param_override_str = "",
              const std::string & begin_timestamp = "0",
              double processing_rate = 1.0);

  ~LCMFrontEnd();

  template<class lcmType, class SensorHandlerClass>
  void addSensor(const std::string & sensor_prefix,
      RBISUpdateInterface * (SensorHandlerClass::*_handler_method)(const lcmType* msg, MavStateEstimator* state_estimator),
      SensorHandlerClass * handler_class);

  void setStateEstimator(MavStateEstimator * _state_estimator);
  void run();
  bool isActive(const std::string & sensor_prefix);
  void outputLogLikeLihood(const std::string & output_fname = "");
  void smooth(double dt);
  void publishHead();
  void publishState(const RBIS & state, const RBIM & cov);

  //LCM/libbot stuff
  lcm::LCM * lcm_pub;
  lcm::LCM * lcm_recv;
  BotParam * param;
  BotFrames * frames;

  //params
  std::string pose_channel;
  std::string filter_state_channel;
  std::string init_message_channel;
  std::string init_complete_channel; // used to publish a message after reset
  bool publish_filter_state;
  bool publish_pose;
  bool republish_sensors;
  std::map<std::string, SensorHandlerInterface *> sensor_handlers; //tracks active sensors whose handlers have registered
  std::set<std::string> active_sensors;
  //filter
  MavStateEstimator * state_estimator;

  bool exit_estimator; // mfallon added. when true, stops estimation
private:
  pronto::filter_state_t rbisCreateFilterStateMessageCPP(const RBIS & state,
                                                         const RBIM & cov) const
  {
    pronto::filter_state_t msg;
    msg.utime = state.utime;

    msg.num_states = RBIS::rbis_num_states;
    msg.num_cov_elements = msg.num_states * msg.num_states;
    eigen_utils::quaternionToBotDouble(msg.quat, state.quat);

    msg.state = std::vector<double>(msg.num_states);
    msg.cov = std::vector<double>(msg.num_cov_elements);

    Eigen::Map<RBIS::VectorNd>(&msg.state[0]) = state.vec;
    Eigen::Map<RBIM>(&msg.cov[0]) = cov;

    return msg;

  }
};
} // namespace MavStateEst

#include "mav_state_est/sensor_handler.hpp"

namespace MavStateEst {
template<class lcmType, class SensorHandlerClass>
void LCMFrontEnd::addSensor(const std::string & sensor_prefix,
    RBISUpdateInterface * (SensorHandlerClass::*_handler_method)(const lcmType* msg, MavStateEstimator* state_estimator), SensorHandlerClass * handler_class)
{
      if (!isActive(sensor_prefix)) {
        fprintf(stderr, "Sensor \"%s\" inactive, not subscribing\n", sensor_prefix.c_str());
        return;
      }

      sensor_handlers[sensor_prefix] = new SensorHandler<lcmType, SensorHandlerClass>(this, sensor_prefix,
          _handler_method, handler_class);
}

} // namespace MavStateEst
