#pragma once

#include "pronto_core/sensing_module.hpp"
#include "pronto_core/definitions.hpp"

namespace pronto {
class InitMessageModule : public SensingModule<FilterState> {
public:

    RBISUpdateInterface* processMessage(const FilterState *msg,
                                        StateEstimator *est) override;

    bool processMessageInit(const FilterState *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;


};
} // namespace pronto

