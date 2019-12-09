#include "pronto_core/scan_matcher_module.hpp"
#include "eigen_utils/eigen_rigidbody.hpp"

namespace pronto {

// using MODE_POSITION     = ScanMatchingMode::MODE_POSITION;
// using MODE_POSITION_YAW = ScanMatchingMode::MODE_POSITION_YAW;
// using MODE_VELOCITY     = ScanMatchingMode::MODE_VELOCITY;
// using MODE_VELOCITY_YAW = ScanMatchingMode::MODE_VELOCITY_YAW;
// using MODE_YAW          = ScanMatchingMode::MODE_YAW;

using Mode = ScanMatcherModule::ScanMatchingMode;

ScanMatcherModule::ScanMatcherModule() : ScanMatcherModule(Mode::POSITION,
                                                           eigen_utils::RigidBodyState::positionInds(),
                                                           Eigen::Matrix3d::Identity())
{

}

ScanMatcherModule::ScanMatcherModule(const ScanMatchingMode &mode,
                                     const Eigen::VectorXi &z_indices,
                                     const Eigen::MatrixXd &cov_scan_match) :
    mode(mode), z_indices(z_indices), cov_scan_match_(cov_scan_match)
{
  std::cout << "[ ScanMatcherModule ] covariance: " << std::endl
            << cov_scan_match_ << std::endl;

}

RBISUpdateInterface * ScanMatcherModule::processMessage(const PoseMeasurement *msg,
                                                        StateEstimator *state_estimator)
{
  // always return, no need to break
  switch(mode) {
  case Mode::POSITION :
  {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
                                      msg->pos,
                                      cov_scan_match_,
                                      RBISUpdateInterface::scan_matcher,
                                      msg->utime);
  }
  case Mode::VELOCITY :
  {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
                                      msg->linear_vel,
                                      cov_scan_match_,
                                      RBISUpdateInterface::scan_matcher,
                                      msg->utime);
  }
  case Mode::YAW:
  {
    return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                     Eigen::Vector4d(0, 0, 0, 0),
                                                     cov_scan_match_,
                                                     msg->orientation,
                                                     RBISUpdateInterface::scan_matcher,
                                                     msg->utime);
  }
  case Mode::POSITION_YAW :
  {
    Eigen::Vector4d z_meas(Eigen::Vector4d::Zero());
    z_meas.head<3>() = msg->pos;
    return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                     z_meas,
                                                     cov_scan_match_,
                                                     msg->orientation,
                                                     RBISUpdateInterface::scan_matcher,
                                                     msg->utime);
  }
  case Mode::VELOCITY_YAW :
  {
    Eigen::Vector4d z_meas(Eigen::Vector4d::Zero());
    z_meas.head<3>() = msg->linear_vel;
    return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                     z_meas,
                                                     cov_scan_match_,
                                                     msg->orientation,
                                                     RBISUpdateInterface::scan_matcher,
                                                     msg->utime);
  }
  case Mode::POSITION_ORIENT :
  {
    Eigen::Matrix<double,6,1> z_meas;
    z_meas.head<3>() = msg->pos;
    z_meas.tail<3>() = Eigen::Vector3d::Zero();
    return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                     z_meas,
                                                     cov_scan_match_,
                                                     msg->orientation,
                                                     RBISUpdateInterface::scan_matcher,
                                                     msg->utime);
  }
  default:
    return nullptr;
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
