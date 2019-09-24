#pragma once

#include <cstdint>
#include "pronto_biped_core/definitions.hpp"

namespace pronto {
namespace biped {

class FootContact {
  private:
    // Parameters:
    float schmitt_level_;
    int64_t transition_timeout_;

    FootID standing_foot_;
    float total_force_;
    int64_t transition_timespan_;
    bool is_foot_transition_intermediate_;

    float l_foot_force_z_;
    float r_foot_force_z_;

    int64_t prev_utime_;
    int64_t delta_utime_;

    float getPrimaryFootZforce();
    float getSecondaryFootZforce();


  public:     
    FootContact (float total_force_, float schmitt_level_);

    void terminate();

    FootID detectFootTransition(int64_t utime,
                                float left_z_force,
                                float right_z_force);

    FootID getStandingFoot();
    // Used internally to change the active foot for motion estimation
    void setStandingFoot(FootID foot);

    // Return which foot is which
    FootID getSecondaryFoot();

    float leftContactStatus();
    float rightContactStatus();

    void updateSingleFootContactStates(const long& utime,
                                       const double& left_force,
                                       const double& right_force);
};

}
}
