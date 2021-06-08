#include "pronto_core/vicon_module.hpp"
#include <iostream>

namespace pronto {

ViconModule::ViconModule(const ViconConfig &cfg) :
    mode(cfg.mode),
    body_to_vicon(cfg.body_to_vicon)
{

    switch(mode){
    case ViconMode::MODE_POSITION:
        z_indices = RBIS::positionInds();
        z_meas.resize(3); // will not be used anyway
        cov_vicon.resize(3,3);
        cov_vicon.setZero();
        cov_vicon = std::pow(cfg.r_vicon_xyz, 2) * Eigen::Matrix3d::Identity();
        break;
    case ViconMode::MODE_YAW:
        z_indices.resize(1);
        z_meas.resize(1);
        z_indices(0) = RBIS::chi_ind + 2; // z component only
        cov_vicon.resize(1,1);
        cov_vicon(0,0) = std::pow((cfg.r_vicon_chi*M_PI/180.0), 2);
        break;
    case ViconMode::MODE_ORIENTATION:
        z_indices.resize(3);
        z_indices = RBIS::chiInds();
        cov_vicon.resize(3,3);
        cov_vicon = std::pow((cfg.r_vicon_chi)*M_PI/180.0,2) * Eigen::Matrix3d::Identity();
        break;
    case ViconMode::MODE_POSITION_ORIENT:
        z_indices.resize(6);
        z_meas.resize(6);
        z_indices.head<3>() = RBIS::positionInds();
        z_indices.tail<3>() = RBIS::chiInds();
        cov_vicon.resize(6,6);
        cov_vicon.setZero();
        cov_vicon.topLeftCorner<3, 3>() = std::pow(cfg.r_vicon_xyz, 2) * Eigen::Matrix3d::Identity();
        cov_vicon.bottomRightCorner<3, 3>() = std::pow((cfg.r_vicon_chi*M_PI/180.0), 2) * Eigen::Matrix3d::Identity();
        break;
    default:
        std::cerr << "Unrecognized mode" << std::endl;
    }

}

RBISUpdateInterface* ViconModule::processMessage(const RigidTransform *msg,
                                                 StateEstimator *est)
{
    local_to_vicon = msg->transform;
    // TODO check that this is correct
    local_to_body = body_to_vicon.inverse() * local_to_vicon;

    // mild check for invalid vicon data. If the translation is very small
    // we send an invalid update
    if ((local_to_vicon.translation().array().abs() < 1e-5).all()){
      return nullptr;
    }

    // no need to break because we return at each case
    switch(mode) {
    case ViconMode::MODE_POSITION:
        return new RBISIndexedMeasurement(z_indices,
                                          local_to_body.translation(),
                                          cov_vicon,
                                          RBISUpdateInterface::vicon,
                                          msg->utime);
    case ViconMode::MODE_YAW:

        return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                         z_meas,
                                                         cov_vicon,
                                                         Eigen::Quaterniond(local_to_body.rotation()),
                                                         RBISUpdateInterface::vicon,
                                                         msg->utime);
    case ViconMode::MODE_ORIENTATION:
        return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                         z_meas,
                                                         cov_vicon,
                                                         Eigen::Quaterniond(local_to_body.rotation()),
                                                         RBISUpdateInterface::vicon,
                                                         msg->utime);
    case ViconMode::MODE_POSITION_ORIENT:
        z_meas.head<3>() = local_to_body.translation();

        return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                         z_meas,
                                                         cov_vicon,
                                                         Eigen::Quaterniond(local_to_body.rotation()),
                                                         RBISUpdateInterface::vicon,
                                                         msg->utime);
    default:
        return nullptr;
    }

}

bool ViconModule::processMessageInit(const RigidTransform *msg,
                                     const std::map<std::string, bool> &sensor_initialized,
                                     const RBIS &default_state,
                                     const RBIM &default_cov,
                                     RBIS &init_state,
                                     RBIM &init_cov)
{
    local_to_vicon = msg->transform;
    // TODO check that this is correct
    local_to_body = local_to_vicon * body_to_vicon.inverse();
    init_state.utime = msg->utime;
    init_state.position() = msg->transform.translation();
    init_state.orientation() = Eigen::Quaterniond(msg->transform.rotation());
    init_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = cov_vicon.topLeftCorner<3, 3>();
    init_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = cov_vicon.bottomRightCorner<3, 3>();
    Eigen::Vector3d init_rpy_deg = (init_state.getEulerAngles())*180.0/M_PI;

    std::cout << "initialized position using VICON at xyz: "
              << init_state.position().transpose() << std::endl;

    std::cout << "initialized orientation using VICON at rpy: "
              << init_rpy_deg.transpose() << std::endl;
    return true;
}

}// namespace pronto


