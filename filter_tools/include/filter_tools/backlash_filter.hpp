#ifndef __BACKLASH_FILTER_HPP__
#define __BACKLASH_FILTER_HPP__

#include <iostream>
#include <inttypes.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include "filter_tools/simple_kalman_filter.hpp" // Eigen::Vector2f KF

namespace EstimateTools {

class BacklashFilter{
  public:
    BacklashFilter(double process_noise_pos_= 0.01, double process_noise_vel_= 0.01, double observation_noise_ = 5E-4);
    
    ~BacklashFilter(){
    }    
    
    void setAlpha(double alpha_in){ alpha_ = alpha_in; }
    void setCrossingTimeMax(double t_crossing_max_in){ t_crossing_max_ = t_crossing_max_in; }

    void processSample(double t,  double x,  double x_dot,
                       double &x_filtered, double &x_dot_filtered);

  private:
    
    EstimateTools::SimpleKalmanFilter* simple_kf_;
    float process_noise_pos_, process_noise_vel_;
    float observation_noise_;

    // last time stamp in seconds
    double t_prev_;
    // Previous raw sensed position
    double x_prev_;
    // Previous output position and velocity:
    double x_dot_filtered_prev_;
    double x_dot_crossing_prev_;
    
    // last velocity zero crossing, seconds:
    double t_crossing_;
    
    // weighting parameter for velocity
    double alpha_; 
    // expiry time of crossing fix (sec)
    double t_crossing_max_;
    
    // have we got our first sample?
    bool init_;
    bool verbose_;    
};


}

#endif

