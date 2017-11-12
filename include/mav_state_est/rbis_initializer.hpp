#ifndef RBIS_INITIALIZER_HPP_
#define RBIS_INITIALIZER_HPP_
#include "lcm_front_end.hpp"
#include "rbis.hpp"
#include <map>

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

class InitMessageHandler {
public:
  bool processMessageInit(const pronto::filter_state_t * msg,
      const std::map<std::string, bool> & sensors_initialized, const RBIS & default_state,
      const RBIM & default_cov, RBIS & init_state, RBIM & init_cov);

  RBISUpdateInterface * processMessage(const pronto::filter_state_t * msg, MavStateEstimator* state_estimator);
};

class RBISInitializer {
public:

  std::string init_message_channel;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LCMFrontEnd * lcm_front;

  //default Sigma values
  RBIS default_state;
  RBIM default_cov;

  bool initializer_created;

  RBIS init_state;
  RBIM init_cov;
  std::map<std::string, bool> sensors_initialized;
  std::map<std::string, SensorHandlerInterface *> sensor_handlers;
  bool initialized;

  RBISInitializer(LCMFrontEnd * _lcm_front, const RBIS & def_state, const RBIM & def_cov);
  ~RBISInitializer();

  template<typename lcmType, typename HandlerClass>
  void addSensor(const std::string & sensor_prefix,
      bool(HandlerClass::*HandlerFunction)(lcmType * msg,
          const std::map<std::string, bool> & sensors_initialized, const RBIS & default_state,
          const RBIM & default_cov, RBIS & init_state, RBIM & init_cov),
      HandlerClass * handler_class)
  {
    if (sensors_initialized.count(sensor_prefix) < 1) {
      fprintf(stderr, "Sensor %s not in initialization string, not subscribing\n", sensor_prefix.c_str());
      return;
    }

    sensor_handlers[sensor_prefix] = new SensorInitHandler<lcmType, HandlerClass>(sensor_prefix, this, HandlerFunction,
        handler_class);
  }

  static RBIM getDefaultCov(BotParam * param);
  static RBIS getDefaultState(BotParam * param);
  static bool allInitializedExcept(const std::map<std::string, bool> & _sensors_initialized
      , const std::string & sensor_prefix);
  static bool initializingWith(const std::map<std::string, bool> & _sensors_initialized
      ,const std::string & sensor_prefix);
  bool initializingWith(const std::string & sensor_prefix);
  void updateInitialization(int64_t utime);
  void initialize(RBIS & state, RBIM & cov);

};

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



#endif /* INITIALIZER_HPP_ */
