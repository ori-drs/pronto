#pragma once

#include <pronto_quadruped/ForceSensorStanceEstimator.hpp>
#include <ros/node_handle.h>

namespace pronto {
namespace quadruped {

class ForceSensorStanceEstimatorROS : public ForceSensorStanceEstimator {
public:
  ForceSensorStanceEstimatorROS(double force_threshold = 50);
  ForceSensorStanceEstimatorROS(ros::NodeHandle& nh);


};

}
}
