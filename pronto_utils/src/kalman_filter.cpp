// 12 joints - 0.55 msec
// 28 joints - 1.3 msec



// x = 68 = 34 states and 34 velocities
// q = states
// qd = velocities
// nq = 34
//
// static (except for dt):
// F - 68x68
// Q - 68x68

#include "pronto_utils/kalman_filter.hpp"
using namespace Eigen;
using namespace std;


namespace EstimateTools {
  
KalmanFilter::KalmanFilter(int nq_, double process_noise_ , double observation_noise_ ): 
     nq_(nq_), process_noise_(process_noise_), observation_noise_(observation_noise_){
  init_ = false;
  verbose_ = false;

  tlast_ = 0;

  Hk_ = MatrixXf (nq_,2*nq_);
  Hk_ << MatrixXf::Identity(nq_,nq_),
         MatrixXf::Zero(nq_,nq_);

  R_ = MatrixXf (nq_,nq_);
  R_ << observation_noise_*MatrixXf::Identity (nq_,nq_);

  P_ = MatrixXf (2*nq_,2*nq_);
  P_ << MatrixXf::Identity (2*nq_,2*nq_);
  
  x_est_ = VectorXf (2*nq_);
  x_est_.setZero(2*nq_);
  
  if (verbose_){
    cout << Hk_ << " Hk_" << endl;
    cout << R_ << " R_" << endl;
    cout << x_est_ << " x_est_" << endl;
  }
       
  // Pre-allocations:
  F = MatrixXf (2*nq_,2*nq_);    
  Q = MatrixXf (2*nq_,2*nq_);
  
  meas_resid = VectorXf (nq_);
  Pprior = MatrixXf(2*nq_,2*nq_);
  S = MatrixXf(nq_,nq_);
  K = MatrixXf(2*nq_,nq_);
  jprior = VectorXf (2*nq_);
  
}
  
  
void KalmanFilter::processSample(double t,  Eigen::VectorXf& x,  Eigen::VectorXf& x_dot,
                    Eigen::VectorXf &x_filtered, Eigen::VectorXf &x_dot_filtered){
  if (!init_){
    init_ = true;
    tlast_ = t;
    //cout << "init\n======\n";
    
    // Set Initial State and input to be current values - to avoid any spikes
    x_est_ << x , x_dot;
    x_filtered = x;
    x_dot_filtered = x_dot;
    
    return;
  }
  
  double dt = t - tlast_;
  
  /*
  if (verbose_){
  std::stringstream ss;  
  ss << std::fixed << t ;  

  cout << x.transpose() << "\n";
  cout << ss.str() << " t\n";
  cout << 1/dt << " fs\n";
  cout << dt << " dt\n";
  }
  */
  
  
  
  //MatrixXf F(2*nq_,2*nq_);
  F << MatrixXf::Identity(nq_,nq_),
       dt*MatrixXf::Identity(nq_,nq_),
       MatrixXf::Zero(nq_,nq_)  ,
       MatrixXf::Identity(nq_,nq_);

  // MatrixXf Q(2*nq_,2*nq_);
  Q << process_noise_*dt*MatrixXf::Identity(nq_,nq_),
       MatrixXf::Zero(nq_,nq_) ,
       MatrixXf::Zero(nq_,nq_)  ,
       process_noise_*MatrixXf::Identity(nq_,nq_);
       
  
  jprior = F*x_est_;
  
  //Eigen::MatrixXf Pprior = MatrixXf(2*nq_,2*nq_);
  Pprior =  F*P_*F.transpose() + Q;
  
  //Eigen::VectorXf meas_resid = VectorXf (nq_);
  meas_resid = x -   Hk_*jprior;
  
  //Eigen::MatrixXf S = MatrixXf(nq_,nq_);
  S = Hk_*Pprior*Hk_.transpose() + R_;
  
  //Eigen::MatrixXf K = MatrixXf(2*nq_,nq_);
  K = ( P_*Hk_.transpose() ) * S.inverse()  ;
  
  x_est_ = jprior + K*meas_resid;
  
  P_ = ( MatrixXf::Identity(2*nq_,2*nq_) - K*Hk_ ) * Pprior;
  
  
  
  // cout << F << endl;
  // cout << Q << endl;
  // cout << x_est_ << endl;
  
  x_filtered << x_est_.head(nq_) ;  
  x_dot_filtered << x_est_.tail(nq_) ;  
  
  /*
  if (1==0){
  std::stringstream ss;  
  ss << std::fixed << t ;  
  cout << ss.str() << " t\n";    
    
    cout << x_filtered.transpose() << " qout "<< endl;
    cout << "processed\n";
  }
  */
//  int dog;
//  cin >> dog;  
  
  tlast_ = t;
}


// template void KalmanFilter::processSample(double t, MatrixBase< VectorXf > &x, MatrixBase< VectorXf > &x_dot, MatrixBase< VectorXf > &x_filtered, MatrixBase< VectorXf > &x_dot_filtered);

//template void KalmanFilter::processSample(MatrixBase< MatrixXf > &,const set<int> &);

}
