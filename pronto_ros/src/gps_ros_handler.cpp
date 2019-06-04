#include "pronto_ros/gps_ros_handler.hpp"
#include "pronto_ros/pronto_ros_conversions.hpp"

namespace pronto {

GPSHandlerROS::GPSHandlerROS(ros::NodeHandle &nh) : nh_(nh) {
    std::string prefix = "gps/";
    GPSConfig cfg;
    nh_.getParam(prefix + "r_xy", cfg.r_gps_xy);
    nh_.getParam(prefix + "r_z", cfg.r_gps_z);
    gps_module_.reset(new GPSModule(cfg));
}

RBISUpdateInterface* GPSHandlerROS::processMessage(const pronto_msgs::GPSData *msg,
                                                   StateEstimator *est)
{
    gpsDataFromROS(*msg, gps_meas_);
    return gps_module_->processMessage(&gps_meas_,
                                      est);

}

bool GPSHandlerROS::processMessageInit(const pronto_msgs::GPSData *msg,
                                       const std::map<std::string, bool> &sensor_initialized,
                                       const RBIS &default_state,
                                       const RBIM &default_cov,
                                       RBIS &init_state,
                                       RBIM &init_cov)
{
    gpsDataFromROS(*msg, gps_meas_);
    return gps_module_->processMessageInit(&gps_meas_,
                                           sensor_initialized,
                                           default_state,
                                           default_cov,
                                           init_state,
                                           init_cov);
}



}
