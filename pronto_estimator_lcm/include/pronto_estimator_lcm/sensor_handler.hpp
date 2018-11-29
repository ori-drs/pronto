#pragma once
#include "pronto_estimator_lcm/sensor_handler_interface.hpp"

#include <lcm/lcm-cpp.hpp>


class LCMFrontend;

namespace MavStateEst {

template<class lcmType, class SensorHandlerClass>
class SensorHandler: public SensorHandlerInterface {
public:
  typedef RBISUpdateInterface * (SensorHandlerClass::*HandlerFunction)(const lcmType* msg, MavStateEstimator* state_estimator);

  SensorHandler(LCMFrontEnd* _lcm_front,
                const std::string& _sensor_prefix,
                HandlerFunction _handler_function,
                SensorHandlerClass * handler_class);

  void lcm_message_handler(const lcm::ReceiveBuffer* rbuf,
                           const std::string& channel,
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
}

#include "pronto_estimator_lcm/lcm_front_end.hpp"

namespace MavStateEst {
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
} // namespace MavStateEst
