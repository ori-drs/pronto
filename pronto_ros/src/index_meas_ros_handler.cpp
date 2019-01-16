#include "pronto_ros/index_meas_ros_handler.hpp"

namespace MavStateEst {

IndexedMeasurementHandlerROS::IndexedMeasurementHandlerROS(const RBISUpdateInterface::sensor_enum &this_sensor) :
    index_module_(this_sensor)
{

}

RBISUpdateInterface * IndexedMeasurementHandlerROS::processMessage(const pronto_msgs::IndexedMeasurement * msg,
                                                                   MavStateEstimator* state_estimator)
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

void IndexedMeasurementHandlerROS::indexMeasurementFromROS(const pronto_msgs::IndexedMeasurement &ros_msg,
                                                           IndexedMeasurement &msg)
{
    // check that the size of z_indices == z_effective and R_effective is its square
    if(ros_msg.R_effective.size() != ros_msg.z_effective.size() * ros_msg.z_indices.size()){
        return;
    }

    // TODO check that the data are rowmajor
    msg.R_effective = Eigen::Map<const Eigen::MatrixXd>(ros_msg.R_effective.data(),
                                                        ros_msg.z_effective.size(),
                                                        ros_msg.z_effective.size());
    msg.state_utime = ros_msg.state_utime;
    msg.utime = ros_msg.utime;
    msg.z_effective = Eigen::Map<const Eigen::VectorXd>(ros_msg.z_effective.data(),
                                                        ros_msg.z_effective.size());
    msg.z_indices = Eigen::Map<const Eigen::VectorXi>(ros_msg.z_indices.data(),
                                                      ros_msg.z_indices.size());
}

} // namespace MavStateEst


