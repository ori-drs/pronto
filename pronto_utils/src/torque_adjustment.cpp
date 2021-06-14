// This class implements a torque adjustment suggested by IHMC

#include "pronto_utils/torque_adjustment.hpp"
#include <cmath>
#include <algorithm>    // std::find


namespace EstimateTools {

TorqueAdjustment::TorqueAdjustment(const std::vector<std::string>& jointsToFilter_,
                                   const std::vector<float>& filterGains_) :
    jointsToFilter_(jointsToFilter_), filterGains_(filterGains_)
{
  // Construct a torque adjustment tool with the given spring constants. The
  // vector filterGains_ should be the same length as the number of efforts and
  // positions (and in the same order), and is measured in units of radians
  // per Newton-meter. To prevent torque adjustment on a joint, set its
  // associated value of filterGains_ to inf (since a joint with no deflection can be
  // thought of as an infinitely stiff spring).
  std::cout << "TorqueAdjustment gains: ";
  for( size_t i=0; i<filterGains_.size(); i++)
    std::cout << jointsToFilter_[i] << " " << filterGains_[i] << ", ";
  std::cout << "\n";

  max_adjustment_ = 0.1; // 0.1 was always used
}


float TorqueAdjustment::magnitudeLimit(float val_in){
  if (val_in > max_adjustment_){
    return max_adjustment_;
  }else if (val_in < -max_adjustment_){
    return -max_adjustment_;
  }
  return val_in;
}

void TorqueAdjustment::processSample(const std::vector<std::string>& names,
                                     std::vector<double> &positions,
                                     std::vector<double> &efforts)
{

  // Loop through list of joints to be filtered:
  for (size_t i=0; i< jointsToFilter_.size(); i++){

    // Find the joint to be filtered
    std::string this_joint = jointsToFilter_[i];
    int pos = std::find(names.begin(), names.end(), this_joint) - names.begin();

    if(pos >= names.size()) {
      std::cerr << "TorqueAdjustment: " << this_joint << " joint not found\n";
      std::cerr << "Names given: " << std::endl;
      for(const auto& el : names){
        std::cerr << el << std::endl;
      }
      exit(-1);
    }else{

      // std::cout << "adjust " << names[pos] << " " << pos << " using " << jointsToFilter[i] << " " << i << "\n";
      // Apply correction. don't do the correction if filterGains_[i] is zero, NaN, or infinite.
      if (std::isnormal(filterGains_[i])) {
        positions[pos] -= magnitudeLimit( efforts[pos] / filterGains_[i]);
      }

    }
  }

  return;

  /*
  // Used for v3:
  //float k_hpz = 7000;
  //float k_x     = 10000;

  // Huge effect - for testing:
  //double k_hpz = 700;
  //double k     = 1000;
  */

}

}
