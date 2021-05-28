#include "pronto_core/lidar_odometry_module.hpp"
#include <iostream>

using Update = pronto::RBISUpdateInterface;
using namespace std;
namespace pronto {

LidarOdometryModule::LidarOdometryModule(const LidarOdometryConfig &cfg) :
    mode_(cfg.mode), z_indices(cfg.z_indices), cov_vo_(cfg.cov_vo)
{
  z_meas.resize(z_indices.rows());
  z_meas.setZero();
  std::cerr << "[ LidarOdometryModule ] Covariance: " << std::endl
            << cov_vo_ << std::endl;
}

void LidarOdometryModule::setCovariance(const Eigen::Matrix3d& covariance){
  cov_vo_ = covariance;
  std::cerr << 
    "[ LidarOdometryModule ] Updated Covariance " << std::endl <<  cov_vo_ << std::endl;
}

Update* LidarOdometryModule::processMessage(const LidarOdometryUpdate *msg,
                                             StateEstimator *est)
{
  std::cerr << "RECEIVED LIDAR ODOMETRY UPDATE" << std::endl;
    // TODO more appropriate check to distinguish different situations
    if(msg->status != LidarOdometryUpdate::REGISTRATION_VALID){
        return nullptr;
    }

    // if there are NaN or inf in the translation, we leave.
    if(!msg->relative_pose.translation().allFinite()){
        cerr << "Visual Odometry: NaN or inf in the translation." << endl;
        return nullptr;
    }

    // supported modalities are only position or position and orientation
    if((mode_ != LidarOdometryMode::POSITION) &&
       (mode_ != LidarOdometryMode::POSITION_ORIENT) &&
       (mode_ != LidarOdometryMode::POSITION_YAW))
    {
        cerr << "Visual Odometry: unsupported mode." << endl;
        return nullptr;
    }

    // get the robot pose at time t0 according to the filter
    if(!est->getInterpolatedPose(msg->prev_utime, t0_body_filter_)){
      std::cerr << "No pose found at requested time" << msg->prev_utime << std::endl;
      auto it = est->history.updateMap.begin();
      std::cerr << "begin time: " << it->first << std::endl;
      it = est->history.updateMap.end();
      it--;
      std::cerr << "end time: " << it->first << std::endl;
      return nullptr;
    }
#if DEBUG_MODE
    std::cerr << "++++++++++++++++++++++++ UPDATE " << std::endl
              << msg->relative_pose.translation().transpose() << std::endl
    << eigen_utils::getEulerAnglesDeg(msg->relative_pose.rotation()).transpose() << std::endl;
    std::cerr << "+++++++++++++++++++++++++ FILTER T0 " << std::endl
          << t0_body_filter_.translation().transpose() << std::endl
          << eigen_utils::getEulerAnglesDeg(t0_body_filter_.rotation()).transpose() << std::endl;
#endif

    // get VO estimate at time t1 as relative motion from time t0 to t1
    t1_body_vo_ = t0_body_filter_ * msg->relative_pose;
#if DEBUG_MODE
    std::cerr << "+++++++++++++++++++++++++ FILTER T1 "
          << t1_body_vo_.translation().transpose() << std::endl
             << eigen_utils::getEulerAnglesDeg(t1_body_vo_.rotation()).transpose() << std::endl;
#endif
    // at this point, the mode is either position or position_orient
    if (mode_ == LidarOdometryMode::POSITION) {
      z_meas.head<3>() = t1_body_vo_.translation();
      // z_meas.head<3>() = t0_body_filter_.translation() + msg->relative_pose.translation();
      std::cerr << "SENDING LIDAR ODOMETRY POSITION UPDATE" << std::endl;
      return new RBISIndexedMeasurement(RigidBodyState::positionInds(),
                                        z_meas,
                                        cov_vo_,
                                        RBISUpdateInterface::scan_matcher,
                                        msg->curr_utime);
    } else {
      std::cerr << "SENDING LIDAR ODOMETRY ELSE UPDATE" << std::endl;
      quat = Eigen::Quaterniond(t1_body_vo_.rotation());
      z_meas.head<3>() = t1_body_vo_.translation();

      return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                       z_meas,
                                                       cov_vo_,
                                                       quat,
                                                       RBISUpdateInterface::scan_matcher,
                                                       msg->curr_utime);
    }
    std::cerr << "NOT SENDING LIDAR ODOMETRY UPDATE" << std::endl;
    return nullptr;
}

bool LidarOdometryModule::processMessageInit(const LidarOdometryUpdate *msg,
                                              const std::map<std::string, bool> &sensor_initialized,
                                              const RBIS &default_state,
                                              const RBIM &default_cov,
                                              RBIS &init_state,
                                              RBIM &init_cov)
{
    // we shouldn't initialize from VO
    return true;
}
}
