#include "pronto_estimator_lcm/gps_handler.hpp"
#include <pronto_conversions/pronto_meas_lcm.hpp>

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


} // namespace MavStateEst
