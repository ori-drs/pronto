#include <estimate_tools/iir_notch.hpp>

IIRNotch::IIRNotch(double notch_freq_, double fs_):notch_freq_(notch_freq_), fs_(fs_){
  // as of april 2014:
  // fs_ = 1000, notch_freq_ = 87 (fundemental freq)
  double Wo = notch_freq_/(fs_/2); 
  double Bw = Wo;
  secondOrderNotch(Wo,Bw, b, a);  
  
  x = Eigen::VectorXd(2);
  x << 0,0;
  y = Eigen::VectorXd(2);
  y << 0,0;
}


void IIRNotch::secondOrderNotch(double Wo, double BW, Eigen::Vector3d &num, Eigen::Vector3d &den ){
  double Ab = fabs(10*log10(.5));

  // Inputs are normalized by pi.
  BW = BW*M_PI;
  Wo = Wo*M_PI;

  double Gb   = pow(10, -Ab/20.);
  double beta = (sqrt(1.0 - Gb*Gb)/Gb)*tan(BW/2.0);
  double gain = 1/(1+beta);

  // 1x3 vectors
  num  = gain*Eigen::Vector3d( 1.0, -2.0*cos(Wo), 1 );
  den  = Eigen::Vector3d( 1.0, -2*gain*cos(Wo), 2*gain-1  );  
  
}
  
double IIRNotch::processSample(double input){
  bool v = false;
  
  Eigen::Vector3d x_temp ( input , x(0),  x(1) );
  Eigen::Vector3d y_temp (      0, y(0),  y(1) );
  if(v)  std::cout << input << "\n";
  if(v)  std::cout << x_temp.transpose() << " x_temp\n";
  if(v)  std::cout << y_temp.transpose() << " y_temp\n";
  if(v)  std::cout << b.transpose() << " b\n";
  if(v)  std::cout << a.transpose() << " a\n";
  
  if(v){  
    Eigen::Vector3d bit =  x_temp.cross(b);
    std::cout << bit.transpose() << " bit\n";  
  }
  
  
  double output =  (x_temp.dot(b)) -  (y_temp.dot(a));
  double temp_x = x(0);
  x << input , temp_x  ;
  double temp_y = y(0);
  y <<  output, temp_y ; 
  
  if(v)  std::cout << x.transpose() << " x\n";
  if(v)  std::cout << y.transpose() << " y\n\n";
  
  return output;
}
