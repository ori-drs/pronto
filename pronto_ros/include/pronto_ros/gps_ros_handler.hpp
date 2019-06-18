#pragma once

#include <pronto_core/gps_module.hpp>
#include <pronto_core/sensing_module.hpp>
#include <pronto_msgs/GPSData.h>
#include <ros/node_handle.h>
#include <pronto_ros/pronto_ros_conversions.hpp>

namespace pronto {
class GPSHandlerROS : public SensingModule<pronto_msgs::GPSData> {
public:
    GPSHandlerROS(ros::NodeHandle& nh);
    RBISUpdateInterface* processMessage(const pronto_msgs::GPSData *msg,
                                        StateEstimator *est) override;

    bool processMessageInit(const pronto_msgs::GPSData *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov);

protected:
    ros::NodeHandle nh_;
    std::shared_ptr<GPSModule> gps_module_;
    GPSMeasurement gps_meas_;
};
} // namespace pronto
