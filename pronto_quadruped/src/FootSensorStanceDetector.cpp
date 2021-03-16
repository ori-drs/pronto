#include "pronto_quadruped/FootSensorStanceDetector.hpp"

namespace pronto {
namespace quadruped {

void FootSensorStanceDetector::setStance(const LegBoolMap& stance){
  stance_ = stance;
}

bool FootSensorStanceDetector::getStance(LegBoolMap& stance) {
  // ignore all the inputs, we get the stance from an external source
  stance = stance_;
  return true;
}

bool FootSensorStanceDetector::getGRF(LegVectorMap& grf){
  double count = 0.0;
  for(size_t i = 0; i < 4; i++){
    if(stance_[LegID(i)]){
      count += 1.0;
    }
  }

  double weight = (50.0 * iit::rbd::g) / count;  // TODO: This is a hard-coded value(!)

  // add a fictitious vertical force equal to a robot weight (of 50 kg mass)
  // divided by the number of legs in contact
  for(size_t i = 0; i < 4; i++){
    grf[LegID(i)] = Eigen::Vector3d(0,0, stance_[LegID(i)]*weight);
  }
  return true;
}

bool FootSensorStanceDetector::isStance(LegID leg) const{
  return stance_[leg];
}

}  // namespace quadruped
}  // namespace pronto
