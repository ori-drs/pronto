#ifndef MAV_STATE_ESTIMATOR_LCMFRONT_HPP_
#define MAV_STATE_ESTIMATOR_LCMFRONT_HPP_

#include <lcm/lcm-cpp.hpp>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <map>
#include <set>

#include <Eigen/Dense>

#include <lcmtypes/bot_core/utime_t.hpp>

//TODO remove when front end is templated on FilterState and Update
#include "rbis.hpp"
#include "rbis_update_interface.hpp"
#include "mav_state_est.hpp"

namespace MavStateEst {

class SensorHandlerInterface {
public:
  virtual ~SensorHandlerInterface()
  {
  }
};

class LCMFrontEnd;

template<class lcmType, class SensorHandlerClass>
class SensorHandler: public SensorHandlerInterface {
public:
  typedef RBISUpdateInterface * (SensorHandlerClass::*HandlerFunction)(const lcmType* msg, MavStateEstimator* state_estimator);

  SensorHandler(LCMFrontEnd * _lcm_front, const std::string & _sensor_prefix, HandlerFunction _handler_function,
      SensorHandlerClass * handler_class);
  void lcm_message_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
      const lcmType * msg);
  ~SensorHandler();

private:
  void loadParams(BotParam * param);

  //front end pointer
  LCMFrontEnd * lcm_front;

  //sensor subscription
  SensorHandlerClass* handler;
  HandlerFunction handler_function;
  lcm::Subscription * lcm_subscription;

  //sensor handling variables
  std::string channel;
  std::string sensor_prefix;
  int64_t utime_delay; //TODO make some of these private so inherited classes don't get confused?
  bool publish_head_on_message;
  bool roll_forward_on_receive;
  int downsample_factor;
  int counter;
};

class LCMFrontEnd {
protected:
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
  LCMFrontEnd(const std::string & in_log_fname, const std::string & out_log_fname = "",
      const std::string & param_fname = "", const std::string & param_override_str = "",
      const std::string & begin_timestamp = "0", double processing_rate = 1.0);
  ~LCMFrontEnd();

  template<class lcmType, class SensorHandlerClass>
  void addSensor(const std::string & sensor_prefix,
      RBISUpdateInterface * (SensorHandlerClass::*_handler_method)(const lcmType* msg, MavStateEstimator* state_estimator),
      SensorHandlerClass * handler_class)
  {
    if (!isActive(sensor_prefix)) {
      fprintf(stderr, "Sensor \"%s\" inactive, not subscribing\n", sensor_prefix.c_str());
      return;
    }

    sensor_handlers[sensor_prefix] = new SensorHandler<lcmType, SensorHandlerClass>(this, sensor_prefix,
        _handler_method, handler_class);
  }

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
};

template<class lcmType, class SensorHandlerClass>
SensorHandler<lcmType, SensorHandlerClass>::SensorHandler(LCMFrontEnd * _lcm_front, const std::string & _sensor_prefix,
    HandlerFunction _handler_function, SensorHandlerClass * handler_class) :
    handler(handler_class), handler_function(_handler_function), lcm_front(_lcm_front), sensor_prefix(_sensor_prefix)
{
  loadParams(_lcm_front->param);

  lcm_subscription = lcm_front->lcm_recv->subscribe(
      channel, &SensorHandler<lcmType, SensorHandlerClass>::lcm_message_handler , this);
  fprintf(stderr, "subscribed to %s for sensor \"%s\"\n", channel.c_str(), sensor_prefix.c_str());
}

template<class lcmType, class SensorHandlerClass>
void SensorHandler<lcmType, SensorHandlerClass>::lcm_message_handler(const lcm::ReceiveBuffer* rbuf,
    const std::string& channel,
    const lcmType * msg)
{
  if (lcm_front->state_estimator == NULL) {
    return;
  }

  if (counter++ % downsample_factor != 0)
    return;

//  (subs->handler->*subs->handlerMethod)
//  RBIS head_state;
//  RBIM head_cov;
//  lcm_front->state_estimator->getHeadState(head_state, head_cov);

  RBISUpdateInterface * update = (handler->*handler_function)(msg, lcm_front->state_estimator);
  if (update != NULL) {
    update->utime -= utime_delay;
    lcm_front->state_estimator->addUpdate(update, roll_forward_on_receive);
  }

  if (lcm_front->init_message_channel != "" && lcm_front->init_complete_channel != "" && channel == lcm_front->init_message_channel) {
      // send a message informing the world that we have just performed a reset
      bot_core::utime_t init_complete_msg;

      RBIS head_state;
      RBIM head_cov;

      lcm_front->state_estimator->getHeadState(head_state, head_cov);

      init_complete_msg.utime = head_state.utime;

      lcm_front->lcm_pub->publish(lcm_front->init_complete_channel, &init_complete_msg);
  }

  if (lcm_front->lcm_pub != lcm_front->lcm_recv && lcm_front->republish_sensors) {
    lcm_front->lcm_pub->publish(channel, msg);
  }
  if (publish_head_on_message) {
    lcm_front->publishHead();
  }
}

template<class lcmType, class SensorHandlerClass>
SensorHandler<lcmType, SensorHandlerClass>::~SensorHandler()
{
  if (lcm_subscription != NULL) {
    lcm_front->lcm_recv->unsubscribe(lcm_subscription);
  }
}

template<class lcmType, class SensorHandlerClass>
void SensorHandler<lcmType, SensorHandlerClass>::loadParams(BotParam * param)
{
  std::string param_prefix = "state_estimator." + sensor_prefix;
  downsample_factor = bot_param_get_int_or_fail(param, (param_prefix + ".downsample_factor").c_str());
  roll_forward_on_receive = bot_param_get_boolean_or_fail(param,
      (param_prefix + ".roll_forward_on_receive").c_str());
  char * chan = bot_param_get_str_or_fail(param, (param_prefix + ".channel").c_str());
  channel = chan;
  free(chan);
  utime_delay = bot_param_get_int_or_fail(param, (param_prefix + ".utime_offset").c_str());
  publish_head_on_message = bot_param_get_boolean_or_fail(param, (param_prefix + ".publish_head_on_message").c_str());
}

} //namespace

#endif

