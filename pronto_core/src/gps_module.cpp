#include "pronto_core/gps_module.hpp"
#include <iostream>

namespace pronto {

GPSModule::GPSModule(const GPSConfig &cfg){
    Eigen::Vector3d R_gps_diagonal = Eigen::Array3d(cfg.r_gps_xy, cfg.r_gps_xy, cfg.r_gps_z).pow(2);
    cov_xyz = R_gps_diagonal.asDiagonal();
}

RBISUpdateInterface* GPSModule::processMessage(const GPSMeasurement *msg,
                                               StateEstimator *est)
{
    if (msg->gps_lock < 3) {
      return NULL;
    }
    return new RBISIndexedMeasurement(RigidBodyState::positionInds(),
                                      msg->xyz_pos,
                                      cov_xyz,
                                      RBISUpdateInterface::gps,
                                      msg->utime);

}

bool GPSModule::processMessageInit(const GPSMeasurement *msg,
                                   const std::map<std::string, bool> &sensor_initialized,
                                   const RBIS &default_state,
                                   const RBIM &default_cov,
                                   RBIS &init_state,
                                   RBIM &init_cov)
{
    init_state.utime = msg->utime;
    RBISIndexedMeasurement * update = dynamic_cast<RBISIndexedMeasurement *>(GPSModule::processMessage(msg, NULL));

    if(update == NULL) {
      return false;
    }

    init_state.position() = update->measurement;
    init_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = update->measurement_cov;

    std::cout << "initialized position using GPS at xyz: "
              << init_state.position().transpose() << std::endl;

    delete update;
    return true;
}

} // namespace pronto 
