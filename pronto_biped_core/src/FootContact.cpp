#include "pronto_biped_core/FootContact.hpp"
#include <iostream>

using namespace pronto::biped;
using namespace std;

FootContact::FootContact(float total_force_, float schmitt_level_) :
  total_force_(total_force_), schmitt_level_(schmitt_level_)
{
  cout << "A new FootContact object was created" << endl;

  // was 1400*0.65 for a long time, but this didn't work with toe-off
  //schmitt_level_ = 0.95;//0.65; 
  transition_timeout_ = 4000;
  
  standing_foot_ = FootID::UNKNOWN;

  prev_utime_ = 0;
  delta_utime_ = 0;
  
  
  is_foot_transition_intermediate_ = true;
  
  l_foot_force_z_ = 0.f;
  r_foot_force_z_ = 0.f;
  
  transition_timespan_ = 0;
}

FootID FootContact::detectFootTransition(int64_t utime, float left_z_force, float right_z_force)
{
  delta_utime_ =  utime - prev_utime_;
  prev_utime_ = utime;
  l_foot_force_z_ = left_z_force;
  r_foot_force_z_ = right_z_force;

  FootID new_footstep = FootID::UNKNOWN;

  if (getSecondaryFootZforce() - schmitt_level_*total_force_ > getPrimaryFootZforce()) {
    transition_timespan_ += delta_utime_;
  }else{
    transition_timespan_ = 0.;
    is_foot_transition_intermediate_ = true;
  }

  if (transition_timespan_ > transition_timeout_ && is_foot_transition_intermediate_)   {
    is_foot_transition_intermediate_ = false;
    new_footstep = getSecondaryFoot();
  }else{
    new_footstep = FootID::UNKNOWN;
  }

  return new_footstep;
}

void FootContact::setStandingFoot(FootID foot) {
  standing_foot_ = foot;
}

FootID FootContact::getStandingFoot() {
  return standing_foot_;
}

FootID FootContact::getSecondaryFoot() {
    switch (standing_foot_) {
    case FootID::LEFT:
        return FootID::RIGHT;
    case FootID::RIGHT:
        return FootID::LEFT;
    default:
        std::cerr << "FootContact::getSecondaryFoot():"
                     " FOOT NUMBERS ARE INCONSISTENT" << std::endl;
        return FootID::UNKNOWN;
    }
}

float FootContact::getPrimaryFootZforce() {
  if (standing_foot_ == FootID::LEFT){
    return l_foot_force_z_;
  }
  return r_foot_force_z_;
}

float FootContact::getSecondaryFootZforce() {
  if (standing_foot_ == FootID::LEFT){
    return r_foot_force_z_;
  }
  return l_foot_force_z_;
}

