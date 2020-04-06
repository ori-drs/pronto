#include "pronto_biped_core/foot_contact_classify.hpp"


using namespace std;
using namespace pronto::biped;

FootContactClassifier::~FootContactClassifier() {
    delete left_contact_state_strong_;
    delete right_contact_state_strong_;
    delete left_contact_state_weak_;
    delete right_contact_state_weak_;
}

FootContactClassifier::FootContactClassifier(bool publish_diagnostics_) :
     print_diagnostics_(publish_diagnostics_),
     lfoot_sensing_(0,0,0),
     rfoot_sensing_(0,0,0)
{
  initialized_ = false;
  mode_ = WalkMode::UNKNOWN;
  previous_mode_ = WalkMode::UNKNOWN;
  verbose_ = 3; // 3 lots, 2 some, 1 v.important, 0 typical for debug, -1 typical for operation

  // defaults dehann used: 0, 5, 5000 but foot doesnt 'hang' like in VRC
  left_contact_state_weak_  = new SchmittTrigger(20.0, 30.0, 5000, 5000);
  right_contact_state_weak_ = new SchmittTrigger(20.0, 30.0, 5000, 5000);
  left_contact_state_strong_  = new SchmittTrigger(275.0, 375.0, 7000, 7000);
  right_contact_state_strong_ = new SchmittTrigger(275.0, 375.0, 7000, 7000);

  last_strike_utime_ = 0;
  last_break_utime_ = 0;
  strike_blackout_duration_ = 95000;// 75ms //was 10.000
  break_blackout_duration_ = 800000; // 750ms

  foot_to_sole_z_ = 0.081119; // hardcoded from the urdf

  contact_points_.resize(4, Eigen::NoChange);

  contact_points_.row(0) << -0.082, 0.0624435, -foot_to_sole_z_;
  contact_points_.row(1) << -0.082, -0.0624435, -foot_to_sole_z_;
  contact_points_.row(2) << 0.178, 0.0624435, -foot_to_sole_z_;
  contact_points_.row(3) << 0.178, -0.0624435, -foot_to_sole_z_;

  foot_to_sole_ = Eigen::Isometry3d( Eigen::Isometry3d::Identity() );
  foot_to_sole_.translation().z() = foot_to_sole_z_;
}



float FootContactClassifier::update(const int64_t &utime,
                                    const Eigen::Isometry3d &primary_foot,
                                    const Eigen::Isometry3d &secondary_foot,
                                    FootID standing_foot)
{
  // 0. Filter the Foot Forces:
  // TODO: correct none-unity coeff sum at source: (a bug in dehann's code)
  float left_force_filtered  = lfoot_sensing_.force_z;
  float right_force_filtered  = rfoot_sensing_.force_z;

  // low pass filter filtering?
  if (apply_low_pass_filter_){
    left_force_filtered  = lpfilter_lfoot_.processSample(lfoot_sensing_.force_z);
    right_force_filtered  = lpfilter_rfoot_.processSample(rfoot_sensing_.force_z);
  }

  // 1. Determine Weak or Strong Contact:
  left_contact_state_weak_->UpdateState(utime, left_force_filtered);
  right_contact_state_weak_->UpdateState(utime, right_force_filtered);

  bool lf_state = (bool) left_contact_state_weak_->getState();
  bool rf_state = (bool) right_contact_state_weak_->getState();

  left_contact_state_strong_->UpdateState(utime, left_force_filtered);
  right_contact_state_strong_->UpdateState(utime, right_force_filtered);

  bool lf_state_high = (bool) left_contact_state_strong_->getState();
  bool rf_state_high = (bool) right_contact_state_strong_->getState();

  //std::cout << utime << " " << left_force_filtered << " " << lf_state << " " << lf_state_high << "\n";
  //std::cout << utime << " " << right_force_filtered << " " << rf_state << " " << rf_state_high << "\n";

  // 2. if the walking state mode:
  updateWalkingPhase (utime, lf_state , rf_state, lf_state_high, rf_state_high);

  // 3. Determine the contact status:
  bool recent_left_strike = false;
  bool recent_right_strike = false;
  bool recent_left_break = false;
  bool recent_right_break = false;

  if (utime - last_strike_utime_  < strike_blackout_duration_){
    if (verbose_ >= 3) {
        std::cout << utime << " recent contact strike\n";
    }

    if (static_cast<int>(mode_) < 4) {
      recent_right_strike = true;
    }
    else {
      recent_left_strike = true;
    }
  }

  if (utime - last_break_utime_  < break_blackout_duration_){
    if (verbose_ >= 3) {
        std::cout << utime << " recent contact break\n";
    }
    if (static_cast<int>(mode_) < 4) {
      recent_right_break = true;
    }
    else {
      recent_left_break = true;
    }
  }

  float odometry_status = 0.0; // by default assume accurate
  if (recent_left_strike || recent_right_strike) {
    odometry_status = -1.0; // unuseable
  } else if (recent_left_break || recent_right_break) {
    odometry_status = 1.0; // very inaccurate
  }

  if (print_diagnostics_){
    // 4. Output Foot Contact Estimates for visualization:
    std::cout << "[ " << utime << " us ] "
              << "FOOT_CONTACT_ESTIMATE (L, R) = "
              <<  lf_state << ", " << rf_state << std::endl;

    bool recent_left_contact = false;
    bool recent_right_contact = false;

    if ( recent_left_break || recent_left_strike) {
      recent_left_contact = true;
    }
    if ( recent_right_break || recent_right_strike) {
      recent_right_contact = true;
    }

    std::cout << "[ " << utime << " us ] "
              << "FOOT_CONTACT_CLASSIFY (L, R) = "
              <<  recent_left_contact << ", " << recent_right_contact << std::endl;
  }

  // Determine which points are in contact with the ground (stub)
  // determineContactPoints(utime, primary_foot, secondary_foot);

  // Determine center of pressure, works but i am not using it currently
  if (verbose_>=3) {
      Eigen::Vector3d cop;
      determineCenterOfPressure(utime,
                                primary_foot,
                                secondary_foot,
                                standing_foot,
                                cop);
      std::cout << "CoP: " << cop.transpose() << std::endl;
  }

  return odometry_status;
}


std::string printVariables(WalkMode mode_,
                            bool left_contact,
                            bool right_contact,
                            bool left_contact_break,
                            bool right_contact_break)
{
  std::stringstream ss;
  ss << left_contact << "" << right_contact << "  " << left_contact_break
     << "" << right_contact_break << " | " << static_cast<int>(mode_);
  return ss.str();
}

int FootContactClassifier::updateWalkingPhase(int64_t utime,
                                              bool left_contact,
                                              bool right_contact,
                                              bool left_contact_strong,
                                              bool right_contact_strong)
{
  string pv = printVariables(mode_,
                             left_contact,
                             right_contact,
                             left_contact_strong,
                             right_contact_strong);

  previous_mode_ = mode_;

  if (!initialized_){
    if (left_contact && right_contact){
      std::cout << pv << ">0 | Initializing. both in contact with left foot as primary\n";
      mode_ = WalkMode::LEFT_PRIME_RIGHT_STAND;
      initialized_ = true;
      return 2;
    }else{
      std::cout << pv << ">8 | Not initialized yet: both feet are not in contact\n";
      return -1;
    }
  }

  switch(mode_){
  case WalkMode::LEFT_PRIME_RIGHT_STAND:
      if (left_contact && !right_contact_strong) {
        if (verbose_ >= 2) {
            std::cout << pv << ">1 | primary left. right weak [LEFT_PRIME_RIGHT_BREAK]\n";
        }
        mode_ = WalkMode::LEFT_PRIME_RIGHT_BREAK;
        last_break_utime_ = utime;
        return 2;
      } else if (!left_contact_strong  && right_contact) {
        if (verbose_ >= 1) {
            std::cout << pv << ">5 | SWITCHING primary to right. left breaking [LEFT_BREAK_RIGHT_PRIME]\n"; //LEG ODOM SWITCH
        }
        mode_ = WalkMode::LEFT_BREAK_RIGHT_PRIME;
        last_break_utime_ = utime;
        return 1;
      } else if (left_contact  && right_contact) {
        if (verbose_ >= 3) std::cout << pv << ">0 | primary left. both in contact. still [LEFT_PRIME_RIGHT_STAND]\n";
        return 2;
      } else {
        std::cout << "Unknown LEFT_PRIME_RIGHT_STAND Transition: " << pv<< "\n";
        return -2;
      }
      break;
  case WalkMode::LEFT_PRIME_RIGHT_BREAK:
      if (left_contact  && !right_contact){
        if (verbose_ >= 0) std::cout << pv << ">2 | primary left but right now raised. [LEFT_PRIME_RIGHT_SWING]\n";
        mode_ = WalkMode::LEFT_PRIME_RIGHT_SWING;
        return -1;
      }else if (left_contact  && right_contact_strong){
        // is this possible?
        if (verbose_ >= 0) std::cout << pv << ">0 | primary left. right now in strong contact. [LEFT_PRIME_RIGHT_STAND]\n";
        mode_ = WalkMode::LEFT_PRIME_RIGHT_STAND;
        return -1;
      }else if (left_contact  && !right_contact_strong){
        if (verbose_ >= 3) std::cout << pv << ">1 | primary left still in contact but right weak. still\n";
        return -1;
      }else{
        std::cout << "Unknown LEFT_PRIME_RIGHT_BREAK Transition: " << pv<< "\n";
        return -2;
      }
      break;
  case WalkMode::LEFT_PRIME_RIGHT_SWING:
      if (left_contact  && !right_contact){
        if (verbose_ >= 3) std::cout << pv << ">2 | primary left. right raised. still [LEFT_PRIME_RIGHT_SWING]\n";
        return 2;
      }else if (left_contact  && right_contact){
        if (verbose_ >= 2) std::cout << pv << ">3 | primary left. right now in contact [LEFT_PRIME_RIGHT_STRIKE]\n";
        mode_ = WalkMode::LEFT_PRIME_RIGHT_STRIKE;
        last_strike_utime_ = utime;
        return 2;
      // Corner Case of momentary "flight"
      }else if (!left_contact  && !right_contact){
        if (verbose_ >= 1) std::cout << pv << ">2 | Error: neither foot in contact! Staying in [LEFT_PRIME_RIGHT_SWING]\n";
        return 2;
      }else{
        std::cout << "Unknown LEFT_PRIME_RIGHT_SWING Transition: " << pv<< "\n";
        return -2;
      }
      break;
  case WalkMode::LEFT_PRIME_RIGHT_STRIKE:
      if (left_contact  && right_contact_strong){
        if (verbose_ >= 1) std::cout << pv << ">0 | primary left. right now in strong contact [LEFT_PRIME_RIGHT_STAND] \n";
        mode_ = WalkMode::LEFT_PRIME_RIGHT_STAND;
        return -1;
      }else if (left_contact  && !right_contact_strong){
        if (verbose_ >= 3) std::cout << pv << ">3 | primary left. right weak. still [LEFT_PRIME_RIGHT_STRIKE] \n";
        return -1;
      }else{
        std::cout << "Unknown LEFT_PRIME_RIGHT_STRIKE Transition: " << pv<< "\n";
        return -2;
      }
      break;
  case WalkMode::LEFT_STAND_RIGHT_PRIME:
      if (!left_contact_strong  && right_contact){
        if (verbose_ >= 2) std::cout << pv << ">5 | primary right. left weak [LEFT_BREAK_RIGHT_PRIME]\n";
        mode_ = WalkMode::LEFT_BREAK_RIGHT_PRIME;
        last_break_utime_ = utime;
        return 2;
      }else if (left_contact && !right_contact_strong){
        if (verbose_ >= 1) std::cout << pv << ">1 | SWITCHING primary to left. right breaking [LEFT_PRIME_RIGHT_BREAK]\n"; //LEG ODOM SWITCH
        mode_ = WalkMode::LEFT_PRIME_RIGHT_BREAK;
        last_break_utime_ = utime;
        return 1;
      }else if (left_contact  && right_contact){
        if (verbose_ >= 3) std::cout << pv << ">4 | primary left. both in contact. still [LEFT_STAND_RIGHT_PRIME]\n";
        return 2;
      }else{
        std::cout << "Unknown LEFT_STAND_RIGHT_PRIME Transition: " << pv<< "\n";
        return -2;
      }
      break;
  case WalkMode::LEFT_BREAK_RIGHT_PRIME:
      if (!left_contact  && right_contact){
        if (verbose_ >= 0) std::cout << pv << ">6 | primary right. left now raised [LEFT_SWING_RIGHT_PRIME]\n";
        mode_ = WalkMode::LEFT_SWING_RIGHT_PRIME;
        return -1;
      }else if (left_contact_strong  && right_contact){
        std::cout << pv << ">4 | primary right. left strong contact [LEFT_STAND_RIGHT_PRIME]\n";
        mode_ = WalkMode::LEFT_STAND_RIGHT_PRIME;
        return -1;
      }else if (!left_contact_strong  && right_contact){
        if (verbose_ >= 3) std::cout << pv << ">5 | primary right. left weak. still [LEFT_BREAK_RIGHT_PRIME]\n";
        return -1;
      }else{
        std::cout << "Unknown LEFT_BREAK_RIGHT_PRIME Transition: " << pv<< "\n";
        return -2;
      }
      break;
  case  WalkMode::LEFT_SWING_RIGHT_PRIME:
      if (!left_contact  && right_contact){
        if (verbose_ >= 3) std::cout << pv << ">6 | primary right. left raised. still. [LEFT_SWING_RIGHT_PRIME]\n";
        return 3;
      }else if (left_contact  && right_contact){
        if (verbose_ >= 2) std::cout << pv << ">7 | primary right. left now in contact [LEFT_STRIKE_RIGHT_PRIME]\n";
        mode_ = WalkMode::LEFT_STRIKE_RIGHT_PRIME;
        last_strike_utime_ = utime;
        return 3;
      // Corner Case of momentary "flight"
      }else if (!left_contact  && !right_contact){
        if (verbose_ >= 1) std::cout << pv << ">6 | Error: neither foot in contact! Staying in [LEFT_SWING_RIGHT_PRIME]\n";
        return 3;
      }else{
        std::cout << "Unknown LEFT_SWING_RIGHT_PRIME Transition: " << pv<< "\n";
        return -2;
      }
      break;
  case WalkMode::LEFT_STRIKE_RIGHT_PRIME:
      if (left_contact_strong  && right_contact){
        if (verbose_ >= 2) std::cout << pv << ">4 | primary right. left now in strong contact [LEFT_STAND_RIGHT_PRIME] \n";
        mode_ = WalkMode::LEFT_STAND_RIGHT_PRIME;
        return -1;
      }else if (!left_contact_strong  && right_contact){
        if (verbose_ >= 3) std::cout << pv << ">7 | primary right. left weak. still [LEFT_STRIKE_RIGHT_PRIME] \n";
        return -1;
      }else{
        std::cout << "Unknown LEFT_STRIKE_RIGHT_PRIME Transition: " << pv<< "\n";
        return -2;
      }
      break;
  default:
      std::cout << "Unknown fallthrough: " << static_cast<int>(mode_) << " > " << left_contact << " " << right_contact << "\n";
      return -2;
  }
}

bool FootContactClassifier::determineContactPoints(const int64_t &utime,
                                                   const Eigen::Isometry3d &primary_foot,
                                                   const Eigen::Isometry3d &secondary_foot)
{
  // Determine moving contact points in stationary foot's sole frame:
  // I define the sole frame as a frame directly below the foot frame on the sole
  Eigen::Matrix<double, Eigen::Dynamic, 3> cp_moving;
  cp_moving.resize(contact_points_.rows(), Eigen::NoChange);

  Eigen::Isometry3d foot_to_foot =  primary_foot.inverse() * secondary_foot * foot_to_sole_;

  for(size_t i = 0; i < contact_points_.rows(); ++i){
      cp_moving.row(i) = (foot_to_foot * contact_points_.row(i).transpose()).transpose();
  }

  if (cp_moving_prev_.rows() != 4){
    std::cout << "Previous contact points not four - we have a problem\n";
    return false;
  } else {

    int n_points_in_contact = 0;
    for (size_t i=0; i < 4 ; i++){
      Eigen::Vector3d cp = cp_moving.row(i).head<3>();
      Eigen::Vector3d cp_prev = cp_moving_prev_.row(i);

      // TODO understand why cp_prev is not used. My guess is that raise
      // should be fabs(cp(2) - cp_prev(2)) instead of just fabs(cp(2))
      double raise = fabs(cp(2));

      if (raise < 0.02){
        n_points_in_contact++;
        //std::cout <<utime << " "<< raise << " " << (int)i << " cp in contact\n";
      } else{
        //std::cout <<utime << " "<< raise << " " << (int)i << " cp NOT in contact\n";
      }
    }

    if (n_points_in_contact > 0){
      std::cout << utime << " " << n_points_in_contact << "\n";
    }
  }

  cp_moving_prev_ = cp_moving;
  
  // determine the velocity of the SF CPs onto the PFCP plane
  // infer the time to contact by differencing

  // If the distance of the foot to the plane is less than a certain amount
  // and the time to contact is less than a certain amount
  // then contact is likely
  return true;
}

void FootContactClassifier::determineCenterOfPressure(const int64_t& utime,
                                                      const Eigen::Isometry3d& primary_foot,
                                                      const Eigen::Isometry3d& secondary_foot,
                                                      FootID standing_foot,
                                                      Eigen::Vector3d& cop) const
{
  Eigen::Isometry3d left_foot = secondary_foot;
  Eigen::Isometry3d right_foot = primary_foot;
  if (standing_foot == FootID::LEFT){
    //std::cout << "on the left\n";
    left_foot = primary_foot;
    right_foot = secondary_foot;
  }else{
    //std::cout << "on the right\n";
  }

  Eigen::Vector3d l_cpressure = Eigen::Vector3d(-lfoot_sensing_.torque_y/lfoot_sensing_.force_z,
                                                 lfoot_sensing_.torque_x/lfoot_sensing_.force_z,
                                                -foot_to_sole_z_);

  Eigen::Vector3d r_cpressure = Eigen::Vector3d(-rfoot_sensing_.torque_y/rfoot_sensing_.force_z,
                                                 rfoot_sensing_.torque_x/rfoot_sensing_.force_z,
                                                -foot_to_sole_z_);
  l_cpressure = left_foot * l_cpressure;
  r_cpressure = right_foot * r_cpressure;

  cop = Eigen::Vector3d(lfoot_sensing_.force_z  * l_cpressure + rfoot_sensing_.force_z * r_cpressure) / ( lfoot_sensing_.force_z + rfoot_sensing_.force_z);
}

