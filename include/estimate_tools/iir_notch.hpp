#include <iostream>
#include <inttypes.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

class IIRNotch{
  public:
    IIRNotch(double notch_freq_, double fs_);
    
    ~IIRNotch(){
    }    
    
    double fs_;
    double notch_freq_;
    
    // iir filter cooeffs
    Eigen::Vector3d b;
    Eigen::Vector3d a;
    
    // carry over inputs/outputs
    Eigen::VectorXd x;
    Eigen::VectorXd y;
    
    double processSample(double input);
    
    // Re-implementation of Matlab's IIR notch filter
    void secondOrderNotch(double Wo, double BW, Eigen::Vector3d &num, Eigen::Vector3d &den );

  private:
};