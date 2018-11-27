#ifndef RBIS_INITIALIZER_HPP_
#define RBIS_INITIALIZER_HPP_
#include "lcm_front_end.hpp"
#include "rbis.hpp"
#include <map>

namespace MavStateEst {


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
      HandlerClass * handler_class);



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
}

#include "mav_state_est/sensor_init_handler.hpp"

namespace MavStateEst {
template<typename lcmType, typename HandlerClass>
void RBISInitializer::addSensor(const std::string & sensor_prefix,
                                bool(HandlerClass::*HandlerFunction)(lcmType * msg,
                                                                     const std::map<std::string, bool> & sensors_initialized,
                                                                     const RBIS & default_state,
                                                                     const RBIM & default_cov,
                                                                     RBIS & init_state,
                                                                     RBIM & init_cov),
                                HandlerClass * handler_class)
{
  if (sensors_initialized.count(sensor_prefix) < 1) {
    fprintf(stderr, "Sensor %s not in initialization string, not subscribing\n", sensor_prefix.c_str());
    return;
  }

  sensor_handlers[sensor_prefix] = new SensorInitHandler<lcmType, HandlerClass>(sensor_prefix, this, HandlerFunction,
      handler_class);
}


}



#endif /* INITIALIZER_HPP_ */
