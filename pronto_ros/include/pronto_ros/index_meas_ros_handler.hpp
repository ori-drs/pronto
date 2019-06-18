#pragma once
#include <pronto_core/indexed_meas_module.hpp>
#include <pronto_core/sensing_module.hpp>
#include <pronto_msgs/IndexedMeasurement.h>


namespace pronto {

class IndexedMeasurementHandlerROS : public SensingModule<pronto_msgs::IndexedMeasurement>
{
public:
    IndexedMeasurementHandlerROS(const RBISUpdateInterface::sensor_enum& this_sensor);

    RBISUpdateInterface * processMessage(const pronto_msgs::IndexedMeasurement *msg,
                                         StateEstimator* state_estimator);

    bool processMessageInit(const pronto_msgs::IndexedMeasurement *msg,
                            const std::map<std::string, bool> & sensors_initialized,
                            const RBIS & default_state,
                            const RBIM & default_cov,
                            RBIS & init_state,
                            RBIM & init_cov);

protected:
    IndexedMeasurementModule index_module_;
    IndexedMeasurement index_msg_;

};

} // namespace pronto
