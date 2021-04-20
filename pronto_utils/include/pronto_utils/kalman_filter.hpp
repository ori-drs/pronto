#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include <iostream>
#include <inttypes.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Core>

namespace EstimateTools {
  
class KalmanFilter{
  public:
    KalmanFilter(int nq_, double process_noise_= 0.01, double observation_noise_ = 5E-4);
    
    ~KalmanFilter(){
    }    
    
    void processSample(double t,  Eigen::VectorXf& x,  Eigen::VectorXf& x_dot,
                       Eigen::VectorXf &x_filtered, Eigen::VectorXf &x_dot_filtered);

  private:
    // static matrices
    Eigen::MatrixXf Hk_;
    Eigen::MatrixXf R_;
    Eigen::MatrixXf P_;
    Eigen::VectorXf x_est_;
    
    // pre allocations (updated):
    Eigen::MatrixXf F, Q;
    Eigen::VectorXf meas_resid;
    Eigen::MatrixXf Pprior;
    Eigen::MatrixXf S;
    Eigen::MatrixXf K;
    Eigen::VectorXf jprior;
    
    double process_noise_;
    double observation_noise_;
    
    // number of states (typically 1 but can support N, but will be inefficent with N^2):
    int nq_;
    // last time stamp in seconds
    double tlast_;
    // have we got our first sample?
    bool init_;
    bool verbose_;    
};

}

#endif
