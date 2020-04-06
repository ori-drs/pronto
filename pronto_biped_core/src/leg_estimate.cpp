#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>
#include <fstream>

#include "pronto_biped_core/leg_estimate.hpp"
#include "pronto_biped_core/biped_forward_kinematics.hpp"


using namespace std;
using namespace pronto::biped;

// TODO: Won't link if placed in header file. find out why!
inline std::string control_mode_strings(ControlMode ctrl_mode)  {
    switch(ctrl_mode) {
    case ControlMode::CONTROLLER_UNKNOWN:
        return "UNKNOWN";
    case ControlMode::CONTROLLER_STANDING:
        return "STANDING";
    case ControlMode::CONTROLLER_WALKING:
        return "WALKING";
    case ControlMode::CONTROLLER_TOE_OFF:
        return "TOE_OFF";
    default:
        return "=====";
    }
}

LegEstimator::~LegEstimator(){
    // deallocate the pointers contained in the vector
    auto it = lpfilter_.begin();
    for(; it != lpfilter_.end(); ++it){
        delete (*it);
    }

    auto kit = joint_kf_.begin();
    for(; kit != joint_kf_.end(); ++kit){
        delete (*kit);
    }
}

LegEstimator::LegEstimator(BipedForwardKinematics& fk, const LegOdometerConfig& cfg) :
  initialization_mode_(cfg.initialization_mode),
  l_standing_link_(cfg.left_foot_name),
  r_standing_link_(cfg.right_foot_name),
  filter_joint_positions_(cfg.filter_mode),
  filter_contact_events_(cfg.filter_contact_events),
  publish_diagnostics_(cfg.publish_diagnostics),
  fk_(fk),
  lfoot_sensing_(0,0,0),
  rfoot_sensing_(0,0,0),
  n_control_contacts_left_(-1),
  n_control_contacts_right_(-1),
  control_mode_(cfg.control_mode),
  use_controller_input_(cfg.use_controller_input)
{

  //TODO create a config data structure to be passed to the constructor
  /* the caller of the constructor will pass the data structure filled as he
     likes (e.g. from a nodehandle).
     this class does the actual computation, it should not get a nodehandle itself
     otherwise it would become ROS-dependent and we would have the same problem
     as we are dealing with now.
    */
  std::cout << "Leg Odometry Initialize Mode: " << initialization_mode_ << " \n";
  std::cout << "Leg Odometry Standing Links: " << l_standing_link_ << " "<< r_standing_link_ << " \n";


  // TODO fill "filter_joint_positions_" from data structure

  if (filter_joint_positions_ == FilterJointMode::LOWPASS) {
      std::cout << "Leg Odometry Filter Joints: lowpass" << std::endl;
    for (size_t i=0;i < NUM_FILT_JOINTS; i++){
      LowPassFilter* a_filter = new LowPassFilter ();
      lpfilter_.push_back(a_filter);
    }
  }else if (filter_joint_positions_ == FilterJointMode::KALMAN) {
    // TODO fill with data structure taken from constructor
    double joint_process_noise  = 0;//bot_param_get_double_or_fail(botparam_, "state_estimator.legodo.joint_process_noise"); // 0.01;
    double joint_observation_noise   = 0;//bot_param_get_double_or_fail(botparam_, "state_estimator.legodo.joint_observation_noise"); // 5E-4;
    for (size_t i=0;i < NUM_FILT_JOINTS; i++){
      EstimateTools::SimpleKalmanFilter* a_filter = new EstimateTools::SimpleKalmanFilter (joint_process_noise, joint_observation_noise); // uses Eigen2d
      joint_kf_.push_back(a_filter);
    }
  }else{
    filter_joint_positions_ = FilterJointMode::NONE;
  }

  filtered_joint_position_.resize(NUM_FILT_JOINTS);
  filtered_joint_velocity_.resize(NUM_FILT_JOINTS);
  filtered_joint_effort_.resize(NUM_FILT_JOINTS);

  std::cout << "Leg Odometry Filter Contact Events: " << filter_contact_events_ << " \n";

  // actually more like 1540N when standing still in Jan 2014, but don't change

  // originally foot shift was when s_foot - 1400*0.65  > p_foot   ... typically s_foot = 1180 | p_foot =200 (~75%)
  foot_contact_logic_.reset(new biped::FootContact(cfg.total_force, cfg.standing_schmitt_level));
  foot_contact_logic_->setStandingFoot( FootID::LEFT );

  foot_contact_logic_alt_.reset(new biped::FootContactAlt(false,
                                                          cfg.schmitt_low_threshold,
                                                          cfg.schmitt_high_threshold,
                                                          cfg.schmitt_low_delay,
                                                          cfg.schmitt_high_delay));
  foot_contact_logic_alt_->setStandingFoot(FootID::LEFT);

  // Should I use a very heavy contact classifier (standing) or one that allows toe off (typical)?
  std::cout << "Leg Odometry Contact Mode: " << control_mode_strings(control_mode_) << "\n";

  // Should I use information from the controller to decide the stationary foot?
  use_controller_input_ = 0;//bot_param_get_boolean_or_fail(botparam_, "state_estimator.legodo.use_controller_input");
  std::cout << "Leg Odometry using input from controller about contact: " << (int) use_controller_input_ << "\n";


  // these two variables are probably duplicate - need to clean this up...
  primary_foot_ = FootID::LEFT; // ie left
  standing_foot_ = FootID::LEFT; //
  leg_odo_init_ = false;

  //TODO class names with CamelCase: FootContactClassifier
  foot_contact_classify_.reset(new FootContactClassifier(publish_diagnostics_));

  verbose_ = 1;

  previous_utime_ = 0; // Set utimes to known values
  current_utime_ = 0;

  odom_to_body_.setIdentity();
  world_to_body_.setIdentity();
  world_to_body_init_ = false;
  world_to_primary_foot_transition_init_ = false;
  world_to_body_constraint_init_ = false;
}

bool LegEstimator::getLegOdometryDelta(Eigen::Isometry3d &odom_to_body_delta,
                                       int64_t &current_utime,
                                       int64_t &previous_utime)
{
  odom_to_body_delta = odom_to_body_delta_;
  current_utime = current_utime_;
  previous_utime = previous_utime_;
  return true;
}

bool LegEstimator::getLegOdometryWorldConstraint(Eigen::Isometry3d &world_to_body_constraint,
                                                 int64_t &current_utime)
{
  world_to_body_constraint = world_to_body_constraint_;
  current_utime = current_utime_;
  return world_to_body_constraint_init_;
}

// Very Basic switch to determine FK of primary and secondary foot
// By providing the id, I can use this flexiably
// TODO: use this in the integration do reduce all the cases
Eigen::Isometry3d getPrimaryFootFK(FootID main_id, Eigen::Isometry3d body_to_l_foot, Eigen::Isometry3d body_to_r_foot){
  if (main_id == FootID::LEFT){
    return body_to_l_foot;
  }else if(main_id == FootID::RIGHT){
    return body_to_r_foot;
  }else{
    std::cout << "ERROR: foot id out of range! getPrimaryFootFK\n";
    return Eigen::Isometry3d::Identity();
  }
}

Eigen::Isometry3d getSecondaryFootFK(FootID main_id, Eigen::Isometry3d body_to_l_foot, Eigen::Isometry3d body_to_r_foot){
  if (main_id == FootID::RIGHT){
    return body_to_l_foot;
  }else if(main_id == FootID::LEFT){
    return body_to_r_foot;
  }else{
    std::cout << "ERROR: foot id out of range! getSecondaryFootFK\n";
    return Eigen::Isometry3d::Identity();
  }
}


// TODO: need to move this function outside of the class, down to app
bool LegEstimator::initializePose(const Eigen::Isometry3d& body_to_foot)
{
  if (initialization_mode_.compare("zero") == 0){
    // Initialize with primary foot at (0,0,0)
    // Otherwise, there is a discontinuity at the very start
    Eigen::Quaterniond q_slaved( world_to_body_.rotation() );
    Eigen::Isometry3d odom_to_body_at_zero;
    odom_to_body_at_zero.setIdentity(); // ... Dont need to use the translation, so not filling it in
    odom_to_body_at_zero.rotate(q_slaved);
    Eigen::Isometry3d odom_to_r_foot_at_zero = odom_to_body_at_zero*body_to_foot;
    Eigen::Quaterniond q_foot_new( odom_to_r_foot_at_zero.rotation() );
    odom_to_primary_foot_fixed_ = Eigen::Isometry3d::Identity();
    odom_to_primary_foot_fixed_.rotate( q_foot_new );

    // was...
    //odom_to_primary_foot_fixed_ = Eigen::Isometry3d::Identity();
    odom_to_body_ =odom_to_primary_foot_fixed_*body_to_foot.inverse();
  }
  return true;
}

bool LegEstimator::prepInitialization(const Eigen::Isometry3d &body_to_l_foot,
                                      const Eigen::Isometry3d &body_to_r_foot,
                                      ContactStatusID contact_status)
{
  bool init_this_iteration = false;
  if (contact_status == ContactStatusID::LEFT_FIXED){
    std::cout << "Initialize Leg Odometry using left foot\n";
    bool success = initializePose(body_to_l_foot); // typical init mode =0
    if (success){
      // if successful, complete initialization
      primary_foot_ = FootID::LEFT; // left
      odom_to_secondary_foot_ = odom_to_body_*body_to_r_foot;
      leg_odo_init_ = true;
      init_this_iteration = true;
    }
  }else if  (contact_status == ContactStatusID::RIGHT_FIXED){
    std::cout << "Initialize Leg Odometry using left foot\n";
    bool success = initializePose(body_to_r_foot); // typical init mode =0
    if (success){
      // if successful, complete initialization
      primary_foot_ = FootID::RIGHT; // right
      odom_to_secondary_foot_ = odom_to_body_*body_to_l_foot;
      leg_odo_init_ = true;
      init_this_iteration = true;
    }
  }

  return init_this_iteration;
}


bool LegEstimator::legOdometryGravitySlavedAlways(const Eigen::Isometry3d& body_to_l_foot,
                                                      const Eigen::Isometry3d& body_to_r_foot,
                                                      const ContactStatusID& contact_status)
{
  bool init_this_iteration= false;

  if (!leg_odo_init_){
    init_this_iteration = prepInitialization(body_to_l_foot, body_to_r_foot, contact_status);
    return init_this_iteration;
  }

  if (contact_status == ContactStatusID::LEFT_FIXED && primary_foot_ == FootID::LEFT){
    if (verbose_>2) std::cout << "Using fixed Left foot, update pelvis position\n";

    Eigen::Isometry3d odom_to_body_at_zero = Eigen::Isometry3d::Identity(); // ... Dont need to use the translation, so not filling it in
    odom_to_body_at_zero.rotate( Eigen::Quaterniond(world_to_body_.rotation()) );

    Eigen::Isometry3d odom_to_r_foot_at_zero = odom_to_body_at_zero*body_to_l_foot;
    Eigen::Quaterniond q_foot_new( odom_to_r_foot_at_zero.rotation() );
    Eigen::Vector3d foot_new_trans = odom_to_primary_foot_fixed_.translation();

    // the foot has now been rotated to agree with the pelvis orientation:
    odom_to_primary_foot_fixed_.setIdentity();
    odom_to_primary_foot_fixed_.translation() = foot_new_trans;
    odom_to_primary_foot_fixed_.rotate( q_foot_new );

    odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_l_foot.inverse() ;
    odom_to_secondary_foot_ = odom_to_body_ * body_to_r_foot;
  }else if (contact_status == ContactStatusID::RIGHT_NEW && primary_foot_ == FootID::LEFT){
    std::cout << "2 Transition Odometry to right foot. Fix foot, update pelvis position\n";
    // When transitioning, take the passive position of the other foot
    // from the previous iteration. this will now be the fixed foot
    // At the instant of transition, slave the pelvis position to gravity:
    // - retain the xyz position.
    // Then do FK and fix that position as the foot position
    Eigen::Isometry3d odom_to_body_switch = Eigen::Isometry3d::Identity();
    odom_to_body_switch.translation() = odom_to_body_.translation();
    odom_to_body_switch.rotate( Eigen::Quaterniond(world_to_body_.rotation()) );
    odom_to_primary_foot_fixed_ = odom_to_body_switch * body_to_r_foot;

    odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_r_foot.inverse();
    odom_to_secondary_foot_ = odom_to_body_ * body_to_l_foot;
    primary_foot_ = FootID::RIGHT;
  }else if (contact_status == ContactStatusID::RIGHT_FIXED && primary_foot_ == FootID::RIGHT){
    if (verbose_>2) std::cout << "Using fixed Right foot, update pelvis position\n";

    Eigen::Isometry3d odom_to_body_at_zero = Eigen::Isometry3d::Identity(); // ... Dont need to use the translation, so not filling it in
    odom_to_body_at_zero.rotate( Eigen::Quaterniond(world_to_body_.rotation()) );

    Eigen::Isometry3d odom_to_r_foot_at_zero = odom_to_body_at_zero*body_to_r_foot;
    Eigen::Quaterniond q_foot_new( odom_to_r_foot_at_zero.rotation() );
    Eigen::Vector3d foot_new_trans = odom_to_primary_foot_fixed_.translation();

    // the foot has now been rotated to agree with the pelvis orientation:
    odom_to_primary_foot_fixed_.setIdentity();
    odom_to_primary_foot_fixed_.translation() = foot_new_trans;
    odom_to_primary_foot_fixed_.rotate( q_foot_new );

    odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_r_foot.inverse() ;
    odom_to_secondary_foot_ = odom_to_body_ * body_to_l_foot;
  }else if (contact_status == ContactStatusID::LEFT_NEW && primary_foot_ == FootID::RIGHT){
    std::cout << "2 Transition Odometry to left foot. Fix foot, update pelvis position\n";

    // When transitioning, take the passive position of the other foot
    // from the previous iteration. this will now be the fixed foot
    // At the instant of transition, slave the pelvis position to gravity:
    // - retain the xyz position.
    // Then do FK and fix that position as the foot position
    Eigen::Isometry3d odom_to_body_switch = Eigen::Isometry3d::Identity();
    odom_to_body_switch.translation() = odom_to_body_.translation();
    odom_to_body_switch.rotate( Eigen::Quaterniond(world_to_body_.rotation()) );
    odom_to_primary_foot_fixed_ = odom_to_body_switch * body_to_l_foot;

    odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_l_foot.inverse();
    odom_to_secondary_foot_ = odom_to_body_ * body_to_r_foot;
    primary_foot_ = FootID::LEFT;
  }else{
    std::cout << "initialized but unknown update: " << static_cast<int>(contact_status) << " and " << (int) primary_foot_ << "\n";
  }

  return init_this_iteration;
}



void LegEstimator::determinePositionConstraintSlavedAlways(const Eigen::Isometry3d& body_to_l_foot,
                                                               const Eigen::Isometry3d& body_to_r_foot)
{
  // Take the CURRENT quaternion from [[[POSE_BODY]]] - via FK.
  // Then update the pelvis position
  Eigen::Isometry3d world_to_body_at_zero = Eigen::Isometry3d::Identity(); // ... Dont need to use the translation, so not filling it in
  world_to_body_at_zero.rotate( Eigen::Quaterniond(world_to_body_.rotation()) );

  Eigen::Isometry3d world_to_primary_at_zero = world_to_body_at_zero* getPrimaryFootFK(primary_foot_,body_to_l_foot, body_to_r_foot) ;
  Eigen::Quaterniond q_foot_new( world_to_primary_at_zero.rotation() ); // assumed to be current foot orientation (in world)
  Eigen::Vector3d foot_new_trans = world_to_primary_foot_transition_.translation();

  // the foot has now been rotated to agree with the pelvis orientation and translated to express the constraint
  world_to_primary_foot_constraint_.setIdentity();
  world_to_primary_foot_constraint_.translation() = foot_new_trans;
  world_to_primary_foot_constraint_.rotate( q_foot_new );

  world_to_body_constraint_ = world_to_primary_foot_constraint_ * getPrimaryFootFK(primary_foot_,body_to_l_foot, body_to_r_foot).inverse() ;
  world_to_secondary_foot_constraint_ = world_to_body_constraint_ * getSecondaryFootFK(primary_foot_,body_to_l_foot, body_to_r_foot);
  world_to_body_constraint_init_ = true;
}


ContactStatusID LegEstimator::footTransition(){
  ContactStatusID contact_status = ContactStatusID::UNKNOWN;

  //std::cout << lfoot_sensing_.force_z <<  " | " << rfoot_sensing_.force_z << "\n";
  FootID newstep = foot_contact_logic_->detectFootTransition(current_utime_, lfoot_sensing_.force_z, rfoot_sensing_.force_z);
  if (newstep == FootID::LEFT || newstep == FootID::RIGHT) {
    foot_contact_logic_->setStandingFoot(newstep);
  }
  if (newstep != FootID::UNKNOWN){
    std::cout << std::endl;
    std::cout << "1 NEW STEP | STANDING ON " << ((foot_contact_logic_->getStandingFoot()==FootID::UNKNOWN) ? "LEFT" : "RIGHT")  << std::endl;
    if ( foot_contact_logic_->getStandingFoot() == FootID::UNKNOWN ){
      contact_status = ContactStatusID::LEFT_NEW;
    }else if ( foot_contact_logic_->getStandingFoot() == FootID::RIGHT ){
      contact_status = ContactStatusID::RIGHT_NEW;
    }else{
      std::cout << "Foot Contact Error "<< static_cast<int>(foot_contact_logic_->getStandingFoot()) << " (switch)\n";
      int blah;
      cin >> blah;
    }
  }else{
    if ( foot_contact_logic_->getStandingFoot() == FootID::UNKNOWN ){
      contact_status = ContactStatusID::LEFT_FIXED;
    }else if ( foot_contact_logic_->getStandingFoot() == FootID::RIGHT ){
      contact_status = ContactStatusID::RIGHT_FIXED;
    }else{
      std::cout << "Foot Contact Error "<< static_cast<int>(foot_contact_logic_->getStandingFoot()) << " \n";
      int blah;
      cin >> blah;
    }
  }

  standing_foot_ = foot_contact_logic_->getStandingFoot();

  return contact_status;
}


ContactStatusID LegEstimator::footTransitionAlt(){
  ContactStatusID contact_status = foot_contact_logic_alt_->detectFootTransition(current_utime_, lfoot_sensing_.force_z, rfoot_sensing_.force_z);

  standing_foot_ = FootID(static_cast<int>(foot_contact_logic_alt_->getStandingFoot()));

  if ( use_controller_input_ ){
  // when standing foot is left but controller is only
  // putting ~2 points of contact, then set right to be standing_foot
  // ContactStatusID::F_LEFT_NEW   = 0, // just decided that left is in (primary) contact
  // ContactStatusID::F_RIGHT_NEW   = 1, // just decided that right is in (primary) contact
  // ContactStatusID::F_LEFT_FIXED = 2, // left continues to be in primary contact
  // ContactStatusID::F_RIGHT_FIXED = 3, // right continues to be in primary contact
  if ((isEqualTo(standing_foot_, ContactStatusID::LEFT_NEW)) || (isEqualTo(standing_foot_, ContactStatusID::LEFT_FIXED))){
    if (n_control_contacts_left_ > -1 && n_control_contacts_left_ < 3 && n_control_contacts_right_ >= 3){
      std::cout << "Signals: " << static_cast<int>(contact_status) << " | Control: " << n_control_contacts_left_ << "  " << n_control_contacts_right_ <<" | Overruling left signals with right control **************\n";
      contact_status = ContactStatusID::RIGHT_NEW;
      foot_contact_logic_alt_->forceRightStandingFoot();
      standing_foot_ = FootID(static_cast<int>(FootID::RIGHT));
    }
  } else if ((isEqualTo(standing_foot_, ContactStatusID::RIGHT_NEW)) || (isEqualTo(standing_foot_, ContactStatusID::RIGHT_FIXED))){
    if (n_control_contacts_right_ > -1 && n_control_contacts_right_ < 3 && n_control_contacts_left_ >= 3){
      std::cout << "Signals: " << static_cast<int>(contact_status) << " | Control: " << n_control_contacts_left_ << "  " << n_control_contacts_right_ <<" | Overruling right signals with left control **************\n";
      contact_status = ContactStatusID::LEFT_NEW;
      foot_contact_logic_alt_->forceLeftStandingFoot();
      standing_foot_ = FootID(static_cast<int>(FootID::LEFT));
    }
  }
  }

  // std::cout << "Signals: " << contact_status << " | " << n_control_contacts_left_ << "  " << n_control_contacts_right_ <<"\n";

  return contact_status;
}


float LegEstimator::updateOdometry(const std::vector<std::string>& joint_name,
                                   const std::vector<double>& joint_position,
                                   const std::vector<double>& joint_velocity,
                                   const int64_t& utime)
{
  
  previous_utime_ = current_utime_;
  previous_odom_to_body_ = odom_to_body_;
  current_utime_ = utime;

  if ( (current_utime_ - previous_utime_)*1E-6 > 30E-3){
    double odo_dt = (current_utime_ - previous_utime_)*1E-6;
    std::cout << "extended time since last update: " <<  odo_dt << "\n";
    std::cout << "resetting the leg odometry\n";

    leg_odo_init_ = false;
  }


  // 0. Filter Joints
  // Two filters: low pass or kalman. low pass adds latency.
  // KF should replace it when I can get a good set of testing logs

  switch(filter_joint_positions_){
  case FilterJointMode::LOWPASS:
      for (size_t i = 0; i < NUM_FILT_JOINTS; i++) {
        filtered_joint_position_[i]  = lpfilter_[i]->processSample(joint_position[i]);
      }
      break;
  case FilterJointMode::KALMAN:
      for (size_t i = 0; i < NUM_FILT_JOINTS; i++) {
          double x_filtered;
          double x_dot_filtered;

          joint_kf_[i]->processSample(((double)utime * 1E-6),
                                      joint_position[i],
                                      joint_velocity[i],
                                      x_filtered,
                                      x_dot_filtered);

          filtered_joint_position_[i] = x_filtered;
          //joint_velocity[ i ] = x_dot_filtered; // not used
      }
      break;
  default:
      // else no filtering
      filtered_joint_position_ = joint_position;
  }


  // 1. Solve for Forward Kinematics:
  Eigen::Isometry3d body_to_l_foot = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d body_to_r_foot = Eigen::Isometry3d::Identity();

  bool kinematics_status = fk_.getLeftFootPose(filtered_joint_position_, body_to_l_foot);
  kinematics_status = kinematics_status & fk_.getRightFootPose(filtered_joint_position_, body_to_r_foot);


  if (!kinematics_status) {
    cerr << "Error: could not calculate forward kinematics!" << endl;
    exit(-1);
  }
  // 2. Determine Primary Foot State

  // 5. Analyse signals to infer covariance
  // Classify/Detect Contact events: strike, break, swing, sway
  foot_contact_classify_->setFootSensing(lfoot_sensing_, rfoot_sensing_);

  float contact_classification = foot_contact_classify_->update(current_utime_,
                                                                odom_to_primary_foot_fixed_,
                                                                odom_to_secondary_foot_,
                                                                standing_foot_);
  ContactStatusID contact_status = ContactStatusID::UNKNOWN;

  if (control_mode_ == ControlMode::CONTROLLER_STANDING){
    contact_status = footTransition(); // original method from Dehann, now set to be very conservative
  } else {
    contact_status = footTransitionAlt();
  }

  bool init_this_iteration = legOdometryGravitySlavedAlways(body_to_l_foot,
                                                                body_to_r_foot,
                                                                contact_status);

  // NB: the corresponding variables are sent

  if (world_to_body_init_) {
    world_to_primary_foot_slide_ = world_to_body_ * getPrimaryFootFK(primary_foot_,
                                                                     body_to_l_foot,
                                                                     body_to_r_foot);

    world_to_secondary_foot_ = world_to_body_ * getSecondaryFootFK(primary_foot_,
                                                                   body_to_l_foot,
                                                                   body_to_r_foot);

    if ((contact_status == ContactStatusID::LEFT_NEW) || (contact_status == ContactStatusID::RIGHT_NEW) )
    {
      std::cout << "3 Leg Estimate: Changing Foot Constraint\n";
      world_to_primary_foot_transition_ = world_to_primary_foot_slide_;
      world_to_primary_foot_transition_init_ = true;
      if (publish_diagnostics_) { // this was enabled by default for a long time
        Isometry3dTime world_to_primary_trans_T = Isometry3dTime(current_utime_ , world_to_primary_foot_transition_ ) ;
        //TODO no pc vis anymore
        //pc_vis_->pose_to_lcm_from_list(1014, world_to_primary_trans_T);
      }
    }
  }

  // 4. Determine a valid kinematic deltaodom_to_body_delta_
  float estimate_status = -1.0; // if odometry is not valid, then use -1 to indicate it
  if (leg_odo_init_) {
    if (!init_this_iteration){
      // Calculate and publish the position delta:
      odom_to_body_delta_ =  previous_odom_to_body_.inverse() * odom_to_body_;
      estimate_status = 0.0; // assume very accurate to begin with

      if (world_to_body_init_ && world_to_primary_foot_transition_init_){
        determinePositionConstraintSlavedAlways(body_to_l_foot, body_to_r_foot);
      }else{
        world_to_body_constraint_init_ = false;
      }


      // TODO WE DONT PUBLISH DIAGNOSTICS ANYMORE because LCM
      if (publish_diagnostics_){
          /*
        Eigen::Vector3d motion_T = odom_to_body_delta_.translation();
        Eigen::Quaterniond motion_R = Eigen::Quaterniond(odom_to_body_delta_.rotation());
        pronto::pose_transform_t legodo_msg;
        legodo_msg.utime = current_utime_;
        legodo_msg.prev_utime = previous_utime_;
        legodo_msg.translation[0] = motion_T(0);
        legodo_msg.translation[1] = motion_T(1);
        legodo_msg.translation[2] = motion_T(2);
        legodo_msg.rotation[0] = motion_R.w();
        legodo_msg.rotation[1] = motion_R.x();
        legodo_msg.rotation[2] = motion_R.y();
        legodo_msg.rotation[3] = motion_R.z();
        //lcm_publish_->publish("LEG_ODOMETRY_DELTA", &legodo_msg); // Outputting this message should enable out of core integration
        */
      }
    }
  }


    // TODO remove diagnostic publishing

    if (publish_diagnostics_){
        /*
      // TODO: pass Isometry3dTime by reference
      Isometry3dTime isoT = Isometry3dTime(current_utime_ , Eigen::Isometry3d::Identity() );
      isoT= Isometry3dTime(current_utime_ , odom_to_body_ ) ;
      pc_vis_->pose_to_lcm_from_list(1001, isoT);
      isoT = Isometry3dTime(current_utime_ , odom_to_primary_foot_fixed_ ) ;
      pc_vis_->pose_to_lcm_from_list(1002, isoT);
      isoT = Isometry3dTime(current_utime_ , odom_to_secondary_foot_ ) ;
      pc_vis_->pose_to_lcm_from_list(1003, isoT);
      // Primary (green) and Secondary (red) Contact points:
      pc_vis_->ptcld_to_lcm_from_list(1004, *foot_contact_classify_->getContactPoints() , current_utime_, current_utime_);
      pc_vis_->ptcld_to_lcm_from_list(1005, *foot_contact_classify_->getContactPoints() , current_utime_, current_utime_);

      if (world_to_body_init_ && world_to_primary_foot_transition_init_){
        isoT = Isometry3dTime(current_utime_ , world_to_body_ ) ;
        pc_vis_->pose_to_lcm_from_list(1011, isoT);
        isoT = Isometry3dTime(current_utime_ , world_to_primary_foot_slide_ ) ;
        pc_vis_->pose_to_lcm_from_list(1012, isoT);
        isoT = Isometry3dTime(current_utime_ , world_to_secondary_foot_ ) ;
        pc_vis_->pose_to_lcm_from_list(1013, isoT);

        isoT = Isometry3dTime(current_utime_ , world_to_body_constraint_ ) ;
        pc_vis_->pose_to_lcm_from_list(1021, isoT);
        isoT = Isometry3dTime(current_utime_ , world_to_primary_foot_constraint_ ) ;
        pc_vis_->pose_to_lcm_from_list(1022, isoT);
        isoT = Isometry3dTime(current_utime_ , world_to_secondary_foot_constraint_ ) ;
        pc_vis_->pose_to_lcm_from_list(1023, isoT);
      }
    }*/
  }


  if (filter_contact_events_){
    if (estimate_status > -1){
      // if the estimator reports a suitable estimate, use the classifier
      // Skip integration of odometry deemed to be unsuitable
      estimate_status = contact_classification;
    }
  }

  previous_body_to_l_foot_ = body_to_l_foot;
  previous_body_to_r_foot_ = body_to_r_foot;
  return estimate_status;
}
