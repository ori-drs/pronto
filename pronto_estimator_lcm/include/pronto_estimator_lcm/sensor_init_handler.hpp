#pragma once
#include <string>
#include <lcm/lcm-cpp.hpp>

namespace MavStateEst {

class RBISInitializer;

template<typename lcmType, typename HandlerClass>
class SensorInitHandler: public SensorHandlerInterface {
public:
  typedef bool (HandlerClass::*HandlerFunction)(lcmType * msg,
      const std::map<std::string, bool> & sensors_initialized, const RBIS & default_state,
      const RBIM & default_cov, RBIS & init_state, RBIM & init_cov);

  SensorInitHandler(
      const std::string & _sensor_prefix, RBISInitializer * _initializer, HandlerFunction _handler_func,
      HandlerClass * _handler);
  ~SensorInitHandler();

  void message_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const lcmType * msg);

private:

  HandlerFunction handler_function;
  HandlerClass * handler;

  RBISInitializer * initializer;

  std::string sensor_prefix;
  std::string channel;

  lcm::Subscription * lcm_subscription;
};
}

#include "pronto_estimator_lcm/rbis_initializer.hpp"

namespace MavStateEst {
template<typename lcmType, typename HandlerClass>
SensorInitHandler<lcmType, HandlerClass>::SensorInitHandler(
    const std::string & _sensor_prefix, RBISInitializer * _initializer, HandlerFunction _handler_func,
    HandlerClass * _handler)
{
  handler_function = _handler_func;
  handler = _handler;

  initializer = _initializer;
  sensor_prefix = _sensor_prefix;

  char * chan = bot_param_get_str_or_fail(initializer->lcm_front->param,
      ("state_estimator." + sensor_prefix + ".channel").c_str());
  channel = chan;
  free(chan);

  lcm_subscription = initializer->lcm_front->lcm_recv->subscribe(channel,
      &SensorInitHandler<lcmType, HandlerClass>::message_handler, this);
  fprintf(stderr, "subscribed to %s for sensor \"%s\" for initialization\n", channel.c_str(),
      sensor_prefix.c_str());
}

template<typename lcmType, typename HandlerClass>
SensorInitHandler<lcmType, HandlerClass>::~SensorInitHandler()
{
  if (lcm_subscription != NULL) {
    initializer->lcm_front->lcm_recv->unsubscribe(lcm_subscription);
  }
}

template<typename lcmType, typename HandlerClass>
void SensorInitHandler<lcmType, HandlerClass>::message_handler(const lcm::ReceiveBuffer* rbuf,
    const std::string& channel, const lcmType * msg)
{
  int64_t msg_utime;
  initializer->sensors_initialized[sensor_prefix] = (handler->*handler_function)(msg, initializer->sensors_initialized,
      initializer->default_state, initializer->default_cov, initializer->init_state, initializer->init_cov);
  if (initializer->sensors_initialized[sensor_prefix]) {
    initializer->lcm_front->lcm_recv->unsubscribe(lcm_subscription);
  }
  initializer->updateInitialization(msg_utime);
}
}
