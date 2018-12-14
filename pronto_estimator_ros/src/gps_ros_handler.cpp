#include "pronto_estimator_ros/gps_ros_handler.hpp"

namespace MavStateEst {

GPSHandlerROS::GPSHandlerROS(ros::NodeHandle &nh) : nh_(nh) {
    std::string prefix = "gps/";
    GPSConfig cfg;
    nh_.getParam(prefix + "r_xy", cfg.r_gps_xy);
    nh_.getParam(prefix + "r_z", cfg.r_gps_z);
    gps_module_.reset(new GPSModule(cfg));
}

RBISUpdateInterface* GPSHandlerROS::processMessage(const pronto_msgs::GPSData *msg,
                                                   MavStateEstimator *est)
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

void GPSHandlerROS::gpsDataFromROS(const pronto_msgs::GPSData &ros_msg,
                              GPSMeasurement &msg)
{
    msg.elev = ros_msg.elev;
    msg.gps_lock = ros_msg.gps_lock;
    msg.gps_time = ros_msg.gps_time;
    msg.heading = ros_msg.heading;
    msg.horizontal_accuracy = ros_msg.horizontal_accuracy;
    msg.latitude = ros_msg.latitude;
    msg.longitude = ros_msg.longitude;
    msg.numSatellites = ros_msg.num_satellites;
    msg.speed = ros_msg.speed;
    msg.utime = ros_msg.utime;
    msg.vertical_accuracy = ros_msg.vertical_accuracy;
    msg.xyz_pos = Eigen::Map<const Eigen::Vector3d>(ros_msg.xyz_pos.data());
}

}
