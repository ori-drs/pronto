#include "pronto_biped_core/FootContactAlt.hpp"
#include <iostream>
using namespace pronto::biped;
using namespace std;

FootContactAlt::~FootContactAlt(){
    delete left_contact_state_strong_;
    delete right_contact_state_strong_;
}

FootContactAlt::FootContactAlt(bool _log_data_files,
                               const float &schmitt_low_threshold,
                               const float &schmitt_high_threshold,
                               const int &schmitt_low_delay,
                               const int &schmitt_high_delay)
{
  cout << "A new FootContactAlt object was created" << endl;

  standing_foot = FootID::UNKNOWN;

  lcmutime = 0;
  deltautime = 0;
  
  foottransitionintermediateflag = true;
  
  l_foot_force_z = 0.f;
  r_foot_force_z = 0.f;
  
  transition_timespan = 0;
  
  // actual noticable toe push off occurs with about 50:50 ratio: 760:760 or even later
  // To reduce tic-tocing you can reduce the first number
  // originally 275/375 was ok, but some upward drift (3/4 of a block height) and backward drift (1/3 of a block length)
  // later 475/525 was better, upward drift (1/2 of block) and backward drift (about the same)
  // in Autumn 2014 used 525/575 and it worked very well
  left_contact_state_strong_  = new SchmittTrigger(schmitt_low_threshold, schmitt_high_threshold, schmitt_low_delay, schmitt_high_delay);
  right_contact_state_strong_ = new SchmittTrigger(schmitt_low_threshold, schmitt_high_threshold, schmitt_low_delay, schmitt_high_delay);
  
  left_contact_state_strong_->forceHigh();
  right_contact_state_strong_->forceHigh();
  
  verbose_ =1; // 3 lots, 2 some, 1 v.important
}


ContactStatusID FootContactAlt::detectFootTransition(int64_t utime, float leftz, float rightz) {
  bool lf_state_last = (bool) left_contact_state_strong_->getState();
  bool rf_state_last = (bool) right_contact_state_strong_->getState();
  
  left_contact_state_strong_->UpdateState(utime, leftz);
  right_contact_state_strong_->UpdateState(utime, rightz);  
  
  bool lf_state = (bool) left_contact_state_strong_->getState();
  bool rf_state = (bool) right_contact_state_strong_->getState();
  
  std::stringstream ss;
  ss << (int)standing_foot << " | " << lf_state << " " << rf_state << " ";
  
  if (!lf_state_last && lf_state){
    if (verbose_ >= 1) std::cout << ss.str() << "Left has gone high\n"; 
    standing_foot = FootID::LEFT;
    return ContactStatusID::LEFT_NEW;
  }else if(!rf_state_last && rf_state){
    if (verbose_ >= 1) std::cout << ss.str() << "Right has gone high\n"; 
    standing_foot = FootID::RIGHT;
    return ContactStatusID::RIGHT_NEW;
  }else if(lf_state_last && !lf_state){
    if (verbose_ >= 3) std::cout << ss.str() << "Left has gone low\n"; 
    if (standing_foot==FootID::LEFT){
      if (verbose_ >= 1) std::cout << ss.str() << "Left has gone low when used as standing, force switch to right "<< leftz << " | "<<  rightz <<"\n"; 
      standing_foot = FootID::RIGHT;
      return ContactStatusID::RIGHT_NEW;
    }else{
      if (verbose_ >= 3) std::cout << ss.str() << "Left has gone low when used not used as standing. Continue with right\n"; 
      return ContactStatusID::RIGHT_FIXED;
    }
  }else if(rf_state_last && !rf_state){
    if (verbose_ >= 3) std::cout << ss.str() << "Right has gone low\n"; 
    if (standing_foot==FootID::RIGHT){
      if (verbose_ >= 1) std::cout << ss.str() << "Right has gone low when used as standing, force switch to left "<< leftz << " | "<<  rightz <<"\n"; 
      standing_foot = FootID::LEFT;
      return ContactStatusID::LEFT_NEW;
    }else{
      if (verbose_ >= 3) std::cout << ss.str() << "Right has gone low when used not used as standing. Continue with left\n"; 
      return ContactStatusID::LEFT_FIXED;
    }    
  }else{
    if (standing_foot==FootID::LEFT){
      if (verbose_ >= 3) std::cout << ss.str() << "Left No change\n";
      return ContactStatusID::LEFT_FIXED;
    }else if (standing_foot==FootID::RIGHT){
      if (verbose_ >= 3) std::cout << ss.str() << "Right No change\n";
      return ContactStatusID::RIGHT_FIXED;
    }
  }
  
  std::cout << ss.str() << "Situation unknown. Error\n";
  exit(-1);
  return ContactStatusID::UNKNOWN;
}

void FootContactAlt::setStandingFoot(FootID foot) {
  standing_foot = foot;
}

FootID FootContactAlt::getStandingFoot() {
  return standing_foot;
}

FootID FootContactAlt::getSecondaryFoot() {
  if (standing_foot == FootID::LEFT)
    return FootID::RIGHT;
  if (standing_foot == FootID::RIGHT)
    return FootID::LEFT;
  std::cout << "FootContactAlt::secondary_foot(): THIS SHOULD NOT HAPPEN THE FOOT NUMBERS ARE INCONSISTENT\n";
  return FootID::UNKNOWN;
}

float FootContactAlt::getPrimaryFootZforce() {
  if (standing_foot == FootID::LEFT)
    return l_foot_force_z;
  return r_foot_force_z;
}

float FootContactAlt::getSecondaryFootZforce() {
  if (standing_foot == FootID::LEFT)
    return r_foot_force_z;
  return l_foot_force_z;
}

void FootContactAlt::forceLeftStandingFoot() {
  standing_foot = FootID::LEFT;
  left_contact_state_strong_->forceHigh();
  right_contact_state_strong_->forceLow();
}

void FootContactAlt::forceRightStandingFoot() {
  left_contact_state_strong_->forceLow();
  right_contact_state_strong_->forceHigh();
  standing_foot = FootID::RIGHT;
}

