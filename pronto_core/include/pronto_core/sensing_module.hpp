#pragma once
#include "pronto_core/rbis_update_interface.hpp"
#include "pronto_core/state_est.hpp"

namespace pronto {

template <class MessageT>
class SensingModule {
public:
    virtual ~SensingModule() {}

    virtual RBISUpdateInterface* processMessage(const MessageT* msg,
                                                StateEstimator* est = nullptr) = 0;

    virtual bool processMessageInit(const MessageT* msg,
                                    const std::map<std::string,bool>& sensor_initialized,
                                    const RBIS & default_state,
                                    const RBIM & default_cov,
                                    RBIS & init_state,
                                    RBIM & init_cov) = 0;
};


template <class PrimaryMsgT, class SecondaryMsgT>
class DualSensingModule : public SensingModule<PrimaryMsgT>{
public:
  virtual void processSecondaryMessage(const SecondaryMsgT& msg) = 0;
};

}
