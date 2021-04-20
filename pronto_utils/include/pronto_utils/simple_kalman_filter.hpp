#ifndef __SIMPLE_KALMAN_FILTER_HPP__
#define __SIMPLE_KALMAN_FILTER_HPP__

#include <iostream>
#include <inttypes.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Core>

namespace EstimateTools {

class SimpleKalmanFilter{
  public:
    SimpleKalmanFilter(double process_noise_pos_= 0.01, double process_noise_vel_= 0.01, double observation_noise_ = 5E-4);
    
    ~SimpleKalmanFilter(){
    }    
    
    void processSample(double t,  double x,  double x_dot,
                       double &x_filtered, double &x_dot_filtered);

  private:
    
    ///////////////////////
    Eigen::Vector2d Hk;
    float R;
    Eigen::Matrix2d P;
    Eigen::Vector2d x_est;

    
    Eigen::Matrix2d F, Q;
    float meas_resid;
    Eigen::Matrix2d Pprior;
    float S;
    Eigen::Vector2d K; // 2x1
    Eigen::Vector2d jprior;
    
    float process_noise_pos_, process_noise_vel_;
    float observation_noise_;
    
    // last time stamp in seconds
    double tlast_;
    // have we got our first sample?
    bool init_;
    bool verbose_;    
};


}

#endif
