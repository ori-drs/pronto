#include "pronto_estimator_core/scan_matcher_module.hpp"
#include "eigen_utils/eigen_rigidbody.hpp"

namespace MavStateEst {

// using MODE_POSITION     = ScanMatchingMode::MODE_POSITION;
// using MODE_POSITION_YAW = ScanMatchingMode::MODE_POSITION_YAW;
// using MODE_VELOCITY     = ScanMatchingMode::MODE_VELOCITY;
// using MODE_VELOCITY_YAW = ScanMatchingMode::MODE_VELOCITY_YAW;
// using MODE_YAW          = ScanMatchingMode::MODE_YAW;

//using ScanMatchingMode;

ScanMatcherModule::ScanMatcherModule() : ScanMatcherModule(MODE_POSITION,
                                                           eigen_utils::RigidBodyState::positionInds(),
                                                           Eigen::Matrix3d::Identity())
{

}

ScanMatcherModule::ScanMatcherModule(const ScanMatchingMode &mode,
                                     const Eigen::VectorXi &z_indices,
                                     const Eigen::MatrixXd &cov_scan_match) :
    mode(mode), z_indices(z_indices), cov_scan_match(cov_scan_match)
{

}

RBISUpdateInterface * ScanMatcherModule::processMessage(const PoseMeasurement *msg,
                                                        MavStateEstimator *state_estimator)
{
    if (mode == MODE_POSITION) {
      return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
                                        msg->pos,
                                        cov_scan_match,
                                        RBISUpdateInterface::scan_matcher,
                                        msg->utime);
    }
    else if (mode == MODE_VELOCITY) {
      return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
                                        msg->linear_vel,
                                        cov_scan_match,
                                        RBISUpdateInterface::scan_matcher,
                                        msg->utime);
    }
    else if (mode == MODE_YAW) {
      Eigen::Vector4d z_meas = Eigen::Vector4d(0,0,0,0); // unused, I believe
      return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                       z_meas,
                                                       cov_scan_match,
                                                       msg->orientation,
                                                       RBISUpdateInterface::scan_matcher,
                                                       msg->utime);
    }
    else if (mode == MODE_POSITION_YAW || mode == MODE_VELOCITY_YAW) {
      Eigen::Vector4d z_meas;
      if (mode == MODE_POSITION_YAW) {
        z_meas.head<3>() = msg->pos;
      }
      else {
        z_meas.head<3>() = msg->linear_vel;
      }
      return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                       z_meas,
                                                       cov_scan_match,
                                                       msg->orientation,
                                                       RBISUpdateInterface::scan_matcher,
                                                       msg->utime);
    }
}


bool ScanMatcherModule::processMessageInit(const PoseMeasurement *msg,
                                           const std::map<std::string, bool> &sensor_initialized,
                                           const RBIS &default_state,
                                           const RBIM &default_cov,
                                           RBIS &init_state,
                                           RBIM &init_cov)
{
    // doing nothing
    return true;
}
}
