// This class implements the backlash filtering 
// fix that was suggested by IHMC 
// essentially, when a velocity crossing occurs
// it is ignored initially and then gradually fed back in
// otherwise the filter is a single tap filter

#include "pronto_utils/backlash_filter.hpp"
using namespace Eigen;
using namespace std;

namespace EstimateTools {

BacklashFilter::BacklashFilter(double process_noise_pos_ ,double process_noise_vel_ , double observation_noise_ ):
     process_noise_pos_(process_noise_pos_), process_noise_vel_(process_noise_vel_), observation_noise_(observation_noise_){
  init_ = false;
  verbose_ = false;

  simple_kf_ = new EstimateTools::SimpleKalmanFilter (process_noise_pos_, process_noise_vel_, observation_noise_);
  
  // Default Parameters:
  alpha_ =0.5;// 0.9042; // Sylvian: "computed to get a 16Hz break freq", value from Scott
  t_crossing_max_ = 0.05; // expiry time of crossing fix (sec), IHMC said they used 30msec
  
  // Initial value of t_crossing was long ago
  t_crossing_ = 0;
}


void BacklashFilter::processSample(double t,  double x,  double x_dot,
                    double& x_filtered, double& x_dot_filtered){
  
  // 1. Filter the Kalman filter
  // velocity output is used to determine crossings
  // position output is used directly
  double x_filtered_kf, x_dot_filtered_kf;
  simple_kf_->processSample( t,  x , x_dot , x_filtered_kf, x_dot_filtered_kf );
  double x_dot_crossing = x_dot_filtered_kf; // use the kalman filtered velocity to infer zero crossings
    
  
  if (!init_){
    init_ = true;
    x_filtered = x;
    x_dot_filtered = x_dot;
    
    // retain for differencing
    t_prev_ = t;
    x_prev_ = x; 
    x_dot_filtered_prev_ = x_dot_filtered;
    return;
  }
  
  double dt = t - t_prev_;
  
  
  // 2. Find zero crossings:
  bool curr_neg = signbit( x_dot_crossing );
  bool prev_neg = signbit( x_dot_crossing_prev_ );
  // std::cout << (int) curr_neg << " and " << (int) prev_neg << "\n";
  if (curr_neg != prev_neg){
    // std::cout << t << " crossing: " << (int) curr_neg << " and " << (int) prev_neg << "\n";
    t_crossing_ = t;
  }
  
  
  // 3. Determine eta - the innovation scaling
  double eta = 1;
  if (t - t_crossing_ < t_crossing_max_)//{   // if crossing has recently happened
     eta = ( t - t_crossing_ )/ t_crossing_max_;
     //std::cout << "recent crossing: " << (t-t_crossing_) << " | " << eta <<"\n";
  //}else{
     //std::cout << "not  a crossing: " << eta << "\n"; 
  //}
  
     
  // 4. Set outputs
  // Velocity is a combination of the previous value and a weight numerical difference
  //double x_dot_diff = (x - x_prev_ )/(t- t_prev_);
  x_filtered =  x_filtered_kf;
  x_dot_filtered = alpha_*x_dot_filtered_prev_  + (1.0 - alpha_)*eta*x_dot_filtered_kf; // velocity

  
  x_prev_ = x;
  t_prev_ = t;
  x_dot_filtered_prev_ = x_dot_filtered;
  x_dot_crossing_prev_ = x_dot_crossing;
}

}
