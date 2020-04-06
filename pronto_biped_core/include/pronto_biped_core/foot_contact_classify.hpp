#pragma once

// 1. I think I saw drc-foot-state publish that both feet were in contact when it was actually in the air
// 2. Obviously the foot will not make perfectly flat contact with the ground - esp with BDI's walking mode
//    Thus integrating JK or even just using constraints between consequtive JK measurements seems quite flawed

#include <iostream>
#include <sstream>      // std::stringstream
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <boost/shared_ptr.hpp>
#include <pronto_math/pronto_math.hpp> // just for the Isometry3dTime
#include <filter_tools/Filter.hpp>
#include <filter_tools/SignalTap.hpp> // SchmittTrigger

#include "pronto_biped_core/definitions.hpp"

namespace pronto {
namespace biped {

class FootSensing { 
public:
  FootSensing(double force_z_in, double torque_x_in, double torque_y_in) :
    force_z (force_z_in), torque_x (torque_x_in), torque_y(torque_y_in)
  {

  }
  float force_z;
  float torque_x;
  float torque_y;
};




class FootContactClassifier {
  public:
    FootContactClassifier (bool print_diagnostics_);
    virtual ~FootContactClassifier();

    // Set the classifier from the 
//    void setFootForces(float left_force_in, float right_force_in  ){
//      left_force_ = left_force_in;
//      right_force_ = right_force_in;
//    }
    void setFootSensing(const FootSensing& lfoot_sensing_in,
                        const FootSensing& rfoot_sensing_in ){
      lfoot_sensing_ = lfoot_sensing_in; 
      rfoot_sensing_ = rfoot_sensing_in;
      std::cerr << "Setting Foot Sensing : " << std::endl;
      std::cerr << "Left Fz Tx Ty"<< lfoot_sensing_.force_z << " " << lfoot_sensing_.torque_x << " " << lfoot_sensing_.torque_y << std::endl;
      std::cerr << "Right Fz Tx Ty"<< rfoot_sensing_.force_z << " " << rfoot_sensing_.torque_x << " " << rfoot_sensing_.torque_y << std::endl;

    }
    
    inline WalkMode getMode() const{
        return mode_;
    }

    inline WalkMode getPreviousMode() const {
        return previous_mode_;
    }
    
    
    // update foot classification.
    // returns: odometry_status
    // 0 -> 1 float
    // 0 is very accurate
    // 1 very inaccuracy
    // -1 unuseable/invalid
    float update (const int64_t& utime,
                  const Eigen::Isometry3d& primary_foot,
                  const Eigen::Isometry3d& secondary_foot,
                  FootID standing_foot);
    
    
    // TODO: return variable is not functioning currently [feb 2014]
    // returns: 
    // -2 logic in error 
    // -1 not initialized yet
    // 0 transitioning onto left foot now. switch leg odom to left
    // 1 transitioning onto right foot now. switch leg odom to right
    // 2 continuing to have left as primary foot [system an initialize from here]
    // 3 continuing to have right as primary foot
    int updateWalkingPhase (int64_t utime,
                            bool left_contact,
                            bool right_contact,
                            bool left_contact_break,
                            bool right_contact_break);
    
    
    // Determine which points are in contact with the ground
    // Currently only a stub which determines distance of points off of plane of standing foot
    bool determineContactPoints(const int64_t& utime,
                                const Eigen::Isometry3d& primary_foot,
                                const Eigen::Isometry3d& secondary_foot);
    
    void determineCenterOfPressure(const int64_t &utime,
                                   const Eigen::Isometry3d &primary_foot,
                                   const Eigen::Isometry3d &secondary_foot,
                                   FootID standing_foot,
                                   Eigen::Vector3d &cop) const;
    
    // pronto::PointCloud* getContactPoints(){ return contact_points_; }
    


  private:
    // the schmitt trigger detector for force-based contact classication:
    
    FootSensing lfoot_sensing_;
    FootSensing rfoot_sensing_; // un filtered
    
    // low pass filters for the feet contact forces
    LowPassFilter lpfilter_lfoot_;
    LowPassFilter lpfilter_rfoot_;

    SchmittTrigger* left_contact_state_weak_;
    SchmittTrigger* right_contact_state_weak_;
    SchmittTrigger* left_contact_state_strong_;
    SchmittTrigger* right_contact_state_strong_;
    
    Eigen::Matrix<double, Eigen::Dynamic, 3> contact_points_;
    // Transform from foot frame origin to a point on the sole below the feet
    double foot_to_sole_z_;
    Eigen::Isometry3d foot_to_sole_;
    
    Eigen::Matrix<double, Eigen::Dynamic, 3> cp_moving_prev_;
    
    // initialization condition is both feet in contact with the ground
    bool initialized_;
    // Publish Debug Data e.g. kinematic velocities and foot contacts
    bool print_diagnostics_ = true;
    
    WalkMode mode_;
    // the mode in the most previous iteration.
    // NOT: the mode that we came from some time ago
    WalkMode previous_mode_;
    
    // time when the last mode break or strike transition occurred
    int64_t last_strike_utime_;
    int64_t last_break_utime_; 
    // Parameters:
    int64_t strike_blackout_duration_; // amount of time to black out when a foot contacts the ground
    int64_t break_blackout_duration_;  // amount of time to black out when a foot breaks the ground
    
    // level of verbosity:
    // 0 - say nothing except pre-initialization
    // 1 - say when the primary foot is switched and pre-initialization
    // 2 - say when a mode transition
    // 3 - say something each iteration
    int verbose_;
    bool apply_low_pass_filter_ = false;
};


}
}
