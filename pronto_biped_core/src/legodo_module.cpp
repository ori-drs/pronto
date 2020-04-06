#include "pronto_biped_core/legodo_module.hpp"

namespace pronto {
namespace biped {

LegOdometryModule::LegOdometryModule(BipedForwardKinematics& fk, const LegOdometryConfig &cfg) :
    leg_est_(new LegEstimator(fk, cfg.odometer_cfg)),
    leg_odo_common_(new LegOdoCommon(cfg.common_cfg)),
    use_torque_adjustment_(cfg.use_torque_adjustment_),
    zero_initial_velocity(cfg.zero_initial_velocity)

{
    std::cout << "LegOdo will compute directly, in thread" << std::endl;

    if (use_torque_adjustment_){
      std::cout << "Torque-based joint angle adjustment: Using\n";

      torque_adjustment_.reset(new EstimateTools::TorqueAdjustment(cfg.torque_adj_names_,
                                                                   cfg.torque_adj_gains_));

    } else {
      std::cout << "Not using torque-based joint angle adjustment." << std::endl;
    }

    std::cout << "Will assume zero kinematic velocity for first "
              <<  zero_initial_velocity << " tics" << std::endl;
}

RBISUpdateInterface* LegOdometryModule::processMessage(const JointState *msg,
                                                       StateEstimator *est)
{

    if (!force_torque_init_){
      std::cout << "Force/Torque message not received yet,"
                << " not integrating leg odometry =========================\n";
      return nullptr;
    }

    // TODO: this was changed to allow head_state to be calculated here,
    // but are the assumptions still valid?
    RBIS head_state;
    RBIM head_cov;
    est->getHeadState(head_state, head_cov);
    leg_est_->setPoseBody(head_state.getPoseAsIsometry3d());

    std::cerr << "force_torque_.sensors[0].force[2] = " <<  force_torque_.sensors[0].force[2] << std::endl;

    // 1. Do the Leg Odometry Integration
    leg_est_->setFootSensing(FootSensing(fabs(force_torque_.sensors[0].force[2]),
                                              force_torque_.sensors[0].moment[0],
                                              force_torque_.sensors[0].moment[1]),
                             FootSensing(fabs(force_torque_.sensors[1].force[2]),
                                              force_torque_.sensors[1].moment[0],
                                              force_torque_.sensors[1].moment[1]));

    leg_est_->setControlContacts(n_control_contacts_left_,
                                 n_control_contacts_right_);

    // 1.1 Apply the joint torque-to-angle adjustment
    // TODO: this should probably be done inside the leg_est class and not here
    std::vector<double> mod_position = msg->joint_position;;
    std::vector<double> mod_effort = msg->joint_effort;

    if (use_torque_adjustment_) {
      torque_adjustment_->processSample(msg->joint_name, mod_position, mod_effort);
    }

    float odo_delta_status = leg_est_->updateOdometry(msg->joint_name,
                                                      mod_position,
                                                      msg->joint_velocity,
                                                      msg->utime);
    if (odo_delta_status < 0){
      if (verbose_ >= 3){
          std::cout << "Leg Odometry is not valid"
                    << " not integrating =========================\n";
      }
      return nullptr;
    }

    // 2. If successful make a RBIS Measurement
    Eigen::Isometry3d odo_delta;
    Eigen::Isometry3d odo_position;
    int64_t utime, prev_utime;
    leg_est_->getLegOdometryDelta(odo_delta, utime, prev_utime);
    int64_t temp;
    bool odo_position_status = leg_est_->getLegOdometryWorldConstraint(odo_position,temp);

    // Ignore the calculated velocity at launch:
    zero_initial_velocity--;

    if (zero_initial_velocity > 0) {
      odo_delta.setIdentity();
      odo_position.setIdentity();
    }


    return leg_odo_common_->createMeasurement(odo_position,
                                              odo_delta,
                                              utime,
                                              prev_utime,
                                              odo_position_status,
                                              odo_delta_status);
}

void LegOdometryModule::setControllerInput(const int &n_contacts_left,
                                           const int &n_contacts_right)
{
    n_control_contacts_left_ = n_contacts_left;
    n_control_contacts_right_ = n_contacts_right;
}

bool LegOdometryModule::processMessageInit(const JointState *msg,
                                           const std::map<std::string, bool> &sensor_initialized,
                                           const RBIS &default_state,
                                           const RBIM &default_cov,
                                           RBIS &init_state,
                                           RBIM &init_cov)
{
    // do nothing for now
    return true;
}

void LegOdometryModule::setForceTorque(const ForceTorqueSensorArray &array){
    force_torque_ = array;
    force_torque_init_ = true;
}

} // namespace biped
} // namespace pronto
