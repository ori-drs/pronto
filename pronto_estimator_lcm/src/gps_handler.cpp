#include "pronto_estimator_lcm/gps_handler.hpp"

namespace MavStateEst {

GpsHandler::GpsHandler(BotParam * _param)
{
    GPSConfig cfg;
    cfg.r_gps_xy = bot_param_get_double_or_fail(_param, "state_estimator.gps.r_xy");
    cfg.r_gps_z = bot_param_get_double_or_fail(_param, "state_estimator.gps.r_z");
    gps_module_.reset(new GPSModule(cfg));
}

RBISUpdateInterface * GpsHandler::processMessage(const bot_core::gps_data_t * msg, MavStateEstimator* state_estimator)
{
    gpsDataFromLCM(*msg, gps_meas_);
    return gps_module_->processMessage(&gps_meas_, state_estimator);
}

bool GpsHandler::processMessageInit(const bot_core::gps_data_t * msg,
    const std::map<std::string, bool> & sensors_initialized
    ,const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov)
{
    gpsDataFromLCM(*msg, gps_meas_);
    return gps_module_->processMessageInit(&gps_meas_,
                                           sensors_initialized,
                                           default_state,
                                           default_cov,
                                           init_state,
                                           init_cov);
}

void GpsHandler::gpsDataFromLCM(const bot_core::gps_data_t &lcm_msg,
                                GPSMeasurement &msg)
{
    gps_meas_.elev = lcm_msg.elev;
    gps_meas_.gps_lock = lcm_msg.gps_lock;
    gps_meas_.gps_time = lcm_msg.gps_time;
    gps_meas_.heading = lcm_msg.heading;
    gps_meas_.horizontal_accuracy = lcm_msg.horizontal_accuracy;
    gps_meas_.latitude = lcm_msg.latitude;
    gps_meas_.longitude = lcm_msg.longitude;
    gps_meas_.numSatellites = lcm_msg.numSatellites;
    gps_meas_.speed = lcm_msg.speed;
    gps_meas_.utime = lcm_msg.utime;
    gps_meas_.vertical_accuracy = lcm_msg.vertical_accuracy;
    gps_meas_.xyz_pos = Eigen::Map<const Eigen::Vector3d>(lcm_msg.xyz_pos);
}
} // namespace MavStateEst
