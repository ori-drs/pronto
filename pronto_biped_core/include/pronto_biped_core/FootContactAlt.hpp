#pragma once

#include <filter_tools/SignalTap.hpp>
#include <cstdint>

#include "pronto_biped_core/definitions.hpp"

namespace pronto {
namespace biped {

class FootContactAlt {
  public:     
    FootContactAlt (bool _log_data_files,
                    const float& schmitt_low_threshold,
                    const float& schmitt_high_threshold,
                    const int& schmitt_low_delay,
                    const int& schmitt_high_delay);

    virtual ~FootContactAlt();

    void terminate();

    ContactStatusID detectFootTransition(int64_t utime, float leftz, float rightz);

    FootID getStandingFoot();
    // Used internally to change the active foot for motion estimation
    void setStandingFoot(FootID foot);

    // Return which foot is which
    FootID getSecondaryFoot();

    float leftContactStatus();
    float rightContactStatus();

    void updateSingleFootContactStates(long utime, const double left_force, const double right_force);

    // Added to help force FootContactAlt into a different state:
    void forceRightStandingFoot();
    void forceLeftStandingFoot();

private:
    float getPrimaryFootZforce();
    float getSecondaryFootZforce();

private:
  FootID standing_foot;
  int64_t transition_timespan;
  bool foottransitionintermediateflag;

  float l_foot_force_z;
  float r_foot_force_z;

  int64_t lcmutime;
  int64_t deltautime;


  SchmittTrigger* left_contact_state_strong_;
  SchmittTrigger* right_contact_state_strong_;

  bool verbose_;

};

}
}
