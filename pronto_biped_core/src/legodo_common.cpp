#include "pronto_biped_core/legodo_common.hpp"

namespace pronto {
namespace biped {

LegOdoCommon::LegOdoCommon(const LegOdoCommonConfig &cfg) :
    mode_(cfg.mode_),
    R_legodo_xyz_(cfg.R_legodo_xyz_),
    R_legodo_vxyz_(cfg.R_legodo_vxyz_),
    R_legodo_vang_(cfg.R_legodo_vang_),
    R_legodo_vxyz_uncertain_(cfg.R_legodo_vxyz_uncertain_),
    R_legodo_vang_uncertain_(cfg.R_legodo_vang_uncertain_)
{


}

void LegOdoCommon::getCovariance(const LegOdometryMode &mode_current,
                            bool delta_certain,
                            Eigen::MatrixXd &cov_legodo,
                            Eigen::VectorXi &z_indices)
{
    // Determine which velocity variance to use
    double R_legodo_vxyz_current = R_legodo_vxyz_;
    double R_legodo_vang_current = R_legodo_vang_;

    if (!delta_certain){
      R_legodo_vxyz_current = R_legodo_vxyz_uncertain_;
      R_legodo_vang_current = R_legodo_vang_uncertain_;
    }

    Eigen::VectorXd R_legodo;

    switch(mode_current) {
    case LegOdometryMode::LIN_AND_ROT_RATE :
        z_indices.resize(6);
        R_legodo.resize(6);

        R_legodo(0) = std::pow(R_legodo_vxyz_current, 2);
        R_legodo(1) = std::pow(R_legodo_vxyz_current, 2);
        R_legodo(2) = std::pow(R_legodo_vxyz_current, 2);
        R_legodo(3) = std::pow(R_legodo_vang_current, 2);
        R_legodo(4) = std::pow(R_legodo_vang_current, 2);
        R_legodo(5) = std::pow(R_legodo_vang_current, 2);

        z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
        z_indices.tail<3>() = eigen_utils::RigidBodyState::angularVelocityInds();
        break;
    case LegOdometryMode::LIN_RATE :
        z_indices.resize(3);
        R_legodo.resize(3);
        R_legodo(0) = std::pow(R_legodo_vxyz_current, 2);
        R_legodo(1) = std::pow(R_legodo_vxyz_current, 2);
        R_legodo(2) = std::pow(R_legodo_vxyz_current, 2);

        z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
        break;
    case LegOdometryMode::POSITION_AND_LIN_RATE :
        z_indices.resize(6);
        R_legodo.resize(6);
        R_legodo(0) = std::pow(R_legodo_xyz_, 2);
        R_legodo(1) = std::pow(R_legodo_xyz_, 2);
        R_legodo(2) = std::pow(R_legodo_xyz_, 2);
        R_legodo(3) = std::pow(R_legodo_vxyz_current, 2);
        R_legodo(4) = std::pow(R_legodo_vxyz_current, 2);
        R_legodo(5) = std::pow(R_legodo_vxyz_current, 2);

        z_indices.head<3>() = eigen_utils::RigidBodyState::positionInds();
        z_indices.tail<3>() = eigen_utils::RigidBodyState::velocityInds();
        break;
    default:
        break;
    }
    cov_legodo = R_legodo.asDiagonal();
}

LegOdoCommon::Transform LegOdoCommon::getTransAsVelocityTrans(const Transform &msgT,
                                                              int64_t utime,
                                                              int64_t prev_utime) const
{
  Transform msgT_vel(Transform::Identity());

  Eigen::Vector3d rpy = eigen_utils::getEulerAngles(Eigen::Quaterniond(msgT.rotation()));

  double elapsed_time = ((double)(utime - prev_utime)) / 1e6;
  Eigen::Vector3d rpy_rate = rpy / elapsed_time;

  msgT_vel.translate(msgT.translation() / elapsed_time);

  Eigen::Quaterniond quat_vel = eigen_utils::setQuatEulerAngles(rpy_rate);

  msgT_vel.rotate(quat_vel);

  return msgT_vel;
}

RBISUpdateInterface * LegOdoCommon::createMeasurement(const Transform &odo_positionT,
                                                      const Transform &delta_odoT,
                                                      const uint64_t &utime,
                                                      const uint64_t &prev_utime,
                                                      const int &odo_position_status,
                                                      const float &odo_delta_status)
{
    Transform odo_velT = getTransAsVelocityTrans(delta_odoT, utime, prev_utime);
    //Eigen::MatrixXd cov_legodo_use;

    LegOdometryMode mode_current = mode_;
    if ((mode_current == LegOdometryMode::POSITION_AND_LIN_RATE) && (!odo_position_status) ){
      if (verbose){
          std::cout << "LegOdo Mode is MODE_POSITION_AND_LIN_RATE but position is not suitable\n";
          std::cout << "Falling back to lin rate only for this iteration\n";
      }
      mode_current = LegOdometryMode::LIN_RATE;
    }

    bool delta_certain = true;
    if (odo_delta_status < 0.5){ // low variance, high reliable
      delta_certain = true;
    } else { // high variance, low reliable e.g. breaking contact
      delta_certain = false;
    }

    Eigen::MatrixXd cov_legodo;
    Eigen::VectorXi z_indices;
    getCovariance(mode_current, delta_certain, cov_legodo, z_indices );

    Eigen::VectorXd z_meas;
    switch(mode_current) {

    case LegOdometryMode::LIN_AND_ROT_RATE:
    {
        // Apply a linear and rotation rate correction
        // This was finished in Feb 2015 but not tested on the robot
        z_meas.resize(6);

        Eigen::Vector3d rpy = eigen_utils::getEulerAngles(delta_odoT.rotation());
        double elapsed_time =  ((double)(utime -  prev_utime)) / 1000000.0;
        double rpy_rate[3];
        rpy_rate[0] = rpy[0] / elapsed_time;
        rpy_rate[1] = rpy[1] / elapsed_time;
        rpy_rate[2] = rpy[2] / elapsed_time;

        z_meas.head<3>() = odo_velT.translation();
        z_meas.tail<3>() = Eigen::Map<const Eigen::Vector3d>(rpy_rate);
        return new RBISIndexedMeasurement(z_indices,
                                          z_meas,
                                          cov_legodo,
                                          RBISUpdateInterface::legodo,
                                          utime);
    }
    case LegOdometryMode::LIN_RATE:
    {
        return new RBISIndexedMeasurement(z_indices,
                                          odo_velT.translation(),
                                          cov_legodo,
                                          RBISUpdateInterface::legodo,
                                          utime);
    }
    case LegOdometryMode::POSITION_AND_LIN_RATE:
    {
        // Newly added mode
        if (verbose){
            std::cout << "LegOdometry Mode update both xyz position and rate\n";
        }
        z_meas.resize(6);
        z_meas.head<3>() = odo_positionT.translation();
        z_meas.tail<3>() = odo_velT.translation();
        return new RBISIndexedMeasurement(z_indices,
                                          z_meas,
                                          cov_legodo,
                                          RBISUpdateInterface::legodo,
                                          utime);
    }
    default:
    {
        std::cout << "LegOdometry Mode not supported, exiting\n";
        return nullptr;
    }
    } // end switch
}

} // namespace biped
} // namespace pronto
