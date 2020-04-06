#include "pronto_biped_core/yawlock_common.hpp"
#include <eigen_utils/eigen_utils.hpp>

// if transitioned to standing, capture yaw
// there after, issue this yaw as a correction to
// to the state estimate

/*

inputs are:
- core robot state
- pose body
- controller status

when standing or manipulating
- and haven't seen a slip recently determine correction (at about 1Hz)

using current state (pose body and joints):
- find pose of each foot

have we just started standing:
- capture the positions of the feet
  [return]

if a yaw jump is detected disable for 5 seconds
- this is done by comparing the average yaw of the feet
  not to earlier. it shouldn't have changed
- to avoid closed loop feedback on controller

else:

assuming the two feet havent slipped since standing
- get reverse FK from each foot to pelvis
- average to give pelvis orientation
- only use the yaw element in the state correction


TODO: look at atlas logs and see if yaw slip detection
occurred during the finals
TODO: one piece of info we dont currently use is the
relative foot velocity in either estimation or failure
detection
*/

namespace pronto {
namespace biped {

YawLock::YawLock(BipedForwardKinematics& fk) : fk_(fk), is_robot_standing_(false)
{
}

bool YawLock::getCorrection(const Eigen::Isometry3d& world_to_body,
                            const int64_t& body_utime,
                            Eigen::Quaterniond &world_to_body_quat_correction){

  // Occasionally send a correction when in either standing or manipulating mode
  if (counter_ % correction_period_ != 0){ // Correct the yaw drift every X tics
    counter_++;
    return false;
  }else{
    counter_++;
  }
  if (!is_robot_standing_){
    std::cout << "YAWLOCK: " <<  body_utime
              << " not in standing or manipulation mode, not correcting\n";
    lock_init_ = false;
    return false;
  }

  if (yaw_slip_detect_){
    if (body_utime < utime_disable_until_){
      std::cout << "YAWLOCK: yaw lock disabled until "
                << utime_disable_until_ << " ("
                << ((double) (utime_disable_until_ - body_utime)*1E-6)
                << " secs more)\n";
      return false;
    }
  }

  Eigen::Isometry3d body_to_l_foot(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d body_to_r_foot(Eigen::Isometry3d::Identity());

  // true = flatten tree to absolute transforms
  bool kinematics_status = fk_.getLeftFootPose(joint_position_, body_to_l_foot)
                           && fk_.getRightFootPose(joint_position_, body_to_r_foot);
  if(!kinematics_status) {
    std::cerr << "Error: could not calculate forward kinematics!" <<std::endl;
    exit(-1);
  }

  Eigen::Isometry3d l_foot_to_r_foot = body_to_l_foot.inverse() *body_to_r_foot ;

  // If not initialized then do so:
  if ( !lock_init_ ){
    world_to_l_foot_original_ =  world_to_body * body_to_l_foot;
    world_to_r_foot_original_ =  world_to_body * body_to_r_foot;
    l_foot_to_r_foot_original_ = l_foot_to_r_foot;

    // TODO replace print_Isometry3d from pronto_math
    /*
    std::cout << "==============================\n\n\n";
    std::string string0 = print_Isometry3d(world_to_body);
    std::cout << string0 << " captured world body\n";
    std::string string1 = print_Isometry3d(world_to_l_foot_original_);
    std::cout << string1 << " captured world lfoot\n";
    std::string string2 = print_Isometry3d(world_to_r_foot_original_);
    std::cout << string2 << " captured world rfoot\n";
    */
    lock_init_ = true;
    return false;
  }


  std::cout << "\n";
  if (yaw_slip_detect_){
    Eigen::Vector3d l_foot_to_r_foot_rpy = eigen_utils::getEulerAngles(Eigen::Quaterniond(l_foot_to_r_foot.rotation()));
    Eigen::Vector3d l_foot_to_r_foot_original_rpy = eigen_utils::getEulerAngles(Eigen::Quaterniond(l_foot_to_r_foot_original_.rotation()));
    double yaw_diff_change = fabs(l_foot_to_r_foot_rpy(2) - l_foot_to_r_foot_original_rpy(2));
    std::cout <<  "YAWLOCK: left-right yaw angle: "
               << l_foot_to_r_foot_original_rpy(2) * 180.0 / M_PI
               << " original | " << l_foot_to_r_foot_rpy(2) * 180.0 / M_PI
               << " now | " << yaw_diff_change * 180.0 / M_PI
               << " change (deg)" << std::endl;

    // If a yaw change of more than XX degrees is detected in the kinematics, don't do yaw lock
    if (yaw_diff_change * 180.0 / M_PI >  yaw_slip_threshold_degrees_) {
      utime_disable_until_ = body_utime + yaw_slip_disable_period_ * 1E6;
      std::stringstream message;
      message << "YAWLOCK: yaw slippage of "
              << (yaw_diff_change * 180.0 / M_PI)
              << " degrees detected. Resetting and disabling the yaw lock until "
              << utime_disable_until_;

      std::cout << message.str() << std::endl;
      // TODO find a way to send this message
      /*
      bot_core::utime_t warning_message;
      warning_message.utime = body_utime;
      lcm_pub->publish(("YAW_SLIP_DETECTED"), &warning_message);
      bot_core::system_status_t stat_msg;
      stat_msg.utime = 0;
      stat_msg.system = stat_msg.MOTION_ESTIMATION;
      stat_msg.importance = stat_msg.VERY_IMPORTANT;
      stat_msg.frequency = stat_msg.LOW_FREQUENCY;
      stat_msg.value = message.str();
      lcm_pub->publish(("SYSTEM_STATUS"), &stat_msg);
      */
      lock_init_ = false;
      return false;
    }
  }

  // Calculated the mean of orientations inferred by the feet:
  Eigen::Isometry3d world_to_body_using_left = world_to_l_foot_original_ * body_to_l_foot.inverse();
  Eigen::Isometry3d world_to_body_using_right = world_to_r_foot_original_ * body_to_r_foot.inverse();

  // Note: the two above frames could be used to infer footslippage
  Eigen::Quaterniond world_to_body_using_left_quat(world_to_body_using_left.rotation());
  Eigen::Quaterniond world_to_body_using_right_quat(world_to_body_using_right.rotation());
  world_to_body_quat_correction = world_to_body_using_left_quat.slerp(0.5, world_to_body_using_right_quat);


//   std::cout << "==============================\n\n\n";
//   std::string l_string = print_Isometry3d(body_to_l_foot);
//   std::cout << l_string << " lfoot\n";
//   std::string r_string = print_Isometry3d(body_to_r_foot);
//   std::cout << r_string << " rfoot\n";
//
//   std::string l_string2 = print_Isometry3d(world_to_body_using_left);
//   std::cout << l_string2 << " world lfoot\n";
//   std::string r_string2 = print_Isometry3d(world_to_body_using_right);
//   std::cout << r_string2 << " world rfoot\n";
//   std::cout << world_to_body_quat_update.w() << ", " << world_to_body_quat_update.w() << ", " << world_to_body_quat_update.w()
//             << ", " << world_to_body_quat_update.w() <<" output\n";


  Eigen::Vector3d rpy_update;

  rpy_update = eigen_utils::getEulerAngles(world_to_body_quat_correction);
  std::cout << "YAWLOCK: " << rpy_update.transpose() << " output rpy [rad]"
            << std::endl;
  std::cout << "YAWLOCK: " << rpy_update.transpose() * 180.0 / M_PI
            << " output rpy [deg]" << std::endl;
  std::cout  << std::endl;

  return true;
}

} // namespace biped
} // namespace pronto
