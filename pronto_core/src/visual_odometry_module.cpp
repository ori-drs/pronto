#include "pronto_core/visual_odometry_module.hpp"

using Update = pronto::RBISUpdateInterface;
using namespace std;
namespace pronto {

VisualOdometryModule::VisualOdometryModule(const VisualOdometryConfig &cfg) :
    mode_(cfg.mode), z_indices(cfg.z_indices), cov_vo_(cfg.cov_vo)
{
  z_meas.resize(3);
}

Update* VisualOdometryModule::processMessage(const VisualOdometryUpdate *msg,
                                             StateEstimator *est)
{
    // TODO more appropriate check to distinguish different situations
    if(msg->status != VisualOdometryUpdate::ESTIMATE_VALID){
        return NULL;
    }

    // if there are NaN or inf in the translation, we leave.
    if(!msg->relative_pose.translation().allFinite()){
        cerr << "Visual Odometry: NaN or inf in the translation." << endl;
        return NULL;
    }

    // supported modalities are only position or position and orientation
    if((mode_ != VisualOdometryMode::MODE_POSITION) && (mode_ != VisualOdometryMode::MODE_POSITION_ORIENT)){
        cerr << "Visual Odometry: unsupported mode." << endl;
        return NULL;
    }



    // get the robot pose at time t0 according to the filter
    if(!est->getInterpolatedPose(msg->prev_utime, t0_body_filter_)){
      return nullptr;
    }

    // get VO estimate at time t1 as relative motion from time t0 to t1
    t1_body_vo_ = t0_body_filter_ * msg->relative_pose;

    // at this point, the mode is either position or position_orient
    if (mode_ == VisualOdometryMode::MODE_POSITION) {
      z_meas.head<3>() = Eigen::Vector3d(t1_body_vo_.translation().x(),
                                         t1_body_vo_.translation().y(),
                                         t1_body_vo_.translation().z());

      return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
                                        z_meas,
                                        cov_vo_,
                                        RBISUpdateInterface::fovis,
                                        msg->curr_utime);
    } else {
      quat = Eigen::Quaterniond(t1_body_vo_.rotation());
      z_meas.head<3>() = Eigen::Vector3d(t1_body_vo_.translation().x(),
                                         t1_body_vo_.translation().y(),
                                         t1_body_vo_.translation().z());

      return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                       z_meas,
                                                       cov_vo_,
                                                       quat,
                                                       RBISUpdateInterface::fovis,
                                                       msg->curr_utime);
    }
    return nullptr;
}

bool VisualOdometryModule::processMessageInit(const VisualOdometryUpdate *msg,
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
