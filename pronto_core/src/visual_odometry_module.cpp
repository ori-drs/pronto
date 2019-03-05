#include "pronto_core/visual_odometry_module.hpp"

using Update = MavStateEst::RBISUpdateInterface;
using namespace std;
namespace MavStateEst {

VisualOdometryModule::VisualOdometryModule(const VisualOdometryConfig &cfg) :
    mode_(cfg.mode), z_indices(cfg.z_indices), cov_vo_(cfg.cov_vo)
{

}

Update* VisualOdometryModule::processMessage(const VisualOdometryUpdate *msg,
                                             MavStateEstimator *est)
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

    t0_body = msg->prev_pose;
    t0_body_internal = Eigen::Isometry3d::Identity();

    if (msg->prev_utime != prev_t0_body_utime_){
        // TODO fill t0_body with the position of the robot in world frame at that time
        /////////////////////////////////////////////////////////////////////////////////
        //updateHistory::historyMapIterator prev_it = est->history.updateMap.find(msg->prev_utime);
        lower_it = est->history.updateMap.lower_bound(msg->prev_utime);
        diff_utime = ( lower_it->first - msg->prev_utime ) *1E-6;
        if (diff_utime > 0.025){
          cout << "Time difference for VO delta root pose is too great ("<< diff_utime <<" sec). Will not use" << endl;
          return NULL;
        }

        // The following check is not properly debugged:
        if (lower_it == est->history.updateMap.end()){
          std::cout << msg->prev_utime <<  " at the end\n";
          return NULL;
        }else{
          std::cout << msg->prev_utime <<  " not at the end - (successful delta change)\n";
        }

        t0_body_RBISInterface = lower_it->second;
        t0_body_RBIS = t0_body_RBISInterface->posterior_state;
        t0_body_internal.translation() = Eigen::Vector3d( t0_body_RBIS.position()[0], t0_body_RBIS.position()[1], t0_body_RBIS.position()[2] );
        t0_body_internal.rotate( Eigen::Quaterniond( t0_body_RBIS.quat.w(), t0_body_RBIS.quat.x(), t0_body_RBIS.quat.y(), t0_body_RBIS.quat.z()) );
        /////////////////////////////////////////////////////////////////////////////////

        prev_t0_body_ = t0_body;
        prev_t0_body_internal_ = t0_body_internal;
        prev_t0_body_utime_ = msg->prev_utime;


      }else{
        t0_body = prev_t0_body_;
        t0_body_internal = prev_t0_body_internal_;
      }


      t1_body = Eigen::Isometry3d::Identity();
      //get_trans_with_utime(frames, "body" , "local", msg->curr_utime, t1_body);

      t0t1_body_vo = msg->relative_pose;

      t1_body_vo = t0_body_internal * t0t1_body_vo; // the pose of the robot as estimated by applying the VO translation on top of t0 state

      // at this point, the mode is either position or position_orient
      if (mode_ == VisualOdometryMode::MODE_POSITION) {
          z_meas.resize(3);
          z_meas.head<3>() = Eigen::Vector3d( t1_body_vo.translation().x()  , t1_body_vo.translation().y() , t1_body_vo.translation().z() );
          return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
                                            z_meas, cov_vo_, RBISUpdateInterface::fovis,
                                            msg->curr_utime);

      } else {
          z_meas.resize(3);
          quat = Eigen::Quaterniond(t1_body_vo.rotation());
          z_meas.head<3>() = Eigen::Vector3d( t1_body_vo.translation().x()  , t1_body_vo.translation().y() , t1_body_vo.translation().z() );
          return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                           z_meas, cov_vo_, quat, RBISUpdateInterface::fovis,
                                                           msg->curr_utime);
      }
      return NULL;
}

bool VisualOdometryModule::processMessageInit(const VisualOdometryUpdate *msg,
                                              const std::map<std::string, bool> &sensor_initialized,
                                              const RBIS &default_state,
                                              const RBIM &default_cov,
                                              RBIS &init_state,
                                              RBIM &init_cov)
{
    // we shouldn't initialize with him
    return true;
}
}
