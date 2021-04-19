// 12 joints - 0.15 msec
// 28 joints - 0.46 msec


#include "pronto_utils/simple_kalman_filter.hpp"

using namespace Eigen;
using namespace std;

namespace EstimateTools {

SimpleKalmanFilter::SimpleKalmanFilter(double process_noise_pos_ ,double process_noise_vel_ , double observation_noise_ ):
     process_noise_pos_(process_noise_pos_), process_noise_vel_(process_noise_vel_), observation_noise_(observation_noise_){
  init_ = false;
  verbose_ = false;

  tlast_ = 0;
  Hk << 1 , 0;
  R = observation_noise_;
  P = Matrix2d::Identity();
  x_est << 0,0;  
}
  
  
void SimpleKalmanFilter::processSample(double t,  double x,  double x_dot,
                    double& x_filtered, double& x_dot_filtered){
  if (!init_){
    init_ = true;
    x_est << x , x_dot;
    x_filtered = x;
    x_dot_filtered = x_dot;
    tlast_ = t;
    return;
  }
  
  double dt = t - tlast_;
  
  F << 1 , dt , 0, 1;
  Q << process_noise_pos_*dt , 0, 0, process_noise_vel_/dt;
  jprior = F*x_est;
  Pprior = F*P*F.transpose() + Q;
  meas_resid = x - Hk.transpose()*jprior;
  S = Hk.transpose()*Pprior*Hk + R;
  K = ( P*Hk )  / S ;
  x_est = jprior + K*meas_resid;
  P = ( Matrix2d::Identity() - K*Hk.transpose()   ) * Pprior;

  x_filtered = x_est(0) ;  
  x_dot_filtered = x_est(1) ;  
  tlast_ = t;
  
}

}
