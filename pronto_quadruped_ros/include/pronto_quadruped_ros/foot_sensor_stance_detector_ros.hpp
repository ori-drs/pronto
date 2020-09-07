#pragma once

#include <pronto_msgs/QuadrupedStance.h>
#include <pronto_quadruped/FootSensorStanceDetector.hpp>

namespace pronto {
namespace quadruped {

class FootSensorStanceDetectorROS : public FootSensorStanceDetector {
public:
  FootSensorStanceDetectorROS() = default;

};

}
}
