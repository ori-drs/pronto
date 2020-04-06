#ifndef __TORQUE_ADJUSTMENT_HPP__
#define __TORQUE_ADJUSTMENT_HPP__

#include <iostream>
#include <inttypes.h>
#include <vector>

namespace EstimateTools {

class TorqueAdjustment{
  public:
    TorqueAdjustment(const std::vector<std::string>& jointsToFilter_,
                     const std::vector<float>& filterGains_);

    virtual ~TorqueAdjustment(){
    }

    void processSample(const std::vector<std::string>& names,
                       std::vector<double> &positions,
                       std::vector<double> &efforts);

  private:

    float magnitudeLimit(float val_in);
    float max_adjustment_;

    std::vector<float> filterGains_;
    std::vector<std::string> jointsToFilter_;

};


}

#endif

