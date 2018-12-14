#pragma once

#include "pronto_estimator_core/sensing_module.hpp"
#include "pronto_estimator_core/definitions.hpp"

namespace MavStateEst {
class InitMessageModule : public SensingModule<FilterState> {
public:

    RBISUpdateInterface* processMessage(const FilterState *msg,
                                        MavStateEstimator *est);

    bool processMessageInit(const FilterState *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov);


};
} // namespace MavStateEst

