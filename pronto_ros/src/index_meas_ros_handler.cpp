#include "pronto_ros/index_meas_ros_handler.hpp"
#include "pronto_ros/pronto_ros_conversions.hpp"

namespace pronto {

IndexedMeasurementHandlerROS::IndexedMeasurementHandlerROS(const RBISUpdateInterface::sensor_enum &this_sensor) :
    index_module_(this_sensor)
{

}

RBISUpdateInterface * IndexedMeasurementHandlerROS::processMessage(const pronto_msgs::IndexedMeasurement * msg,
                                                                   StateEstimator* state_estimator)
{
    indexMeasurementFromROS(*msg, index_msg_);
    return index_module_.processMessage(&index_msg_, state_estimator);
}

bool IndexedMeasurementHandlerROS::processMessageInit(const pronto_msgs::IndexedMeasurement * msg,
                                                   const std::map<std::string, bool> & sensors_initialized,
                                                   const RBIS & default_state,
                                                   const RBIM & default_cov,
                                                   RBIS & init_state,
                                                   RBIM & init_cov)
{
    indexMeasurementFromROS(*msg, index_msg_);
    return index_module_.processMessageInit(&index_msg_,
                                            sensors_initialized,
                                            default_state,
                                            default_cov,
                                            init_state,
                                            init_cov);
}
} // namespace pronto


