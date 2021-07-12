#pragma once
#include "pronto_quadruped/StanceEstimatorBase.hpp"

namespace pronto {

namespace quadruped {

class ForceSensorStanceEstimator : public StanceEstimatorBase {
public:
  ForceSensorStanceEstimator(double /*force_threshold*/ = 50) {
  }

  bool getStance(LegBoolMap &stance) override;

  bool getStance(LegBoolMap& stance,
                 LegScalarMap& stance_probability) override;

  bool getGRF(LegVectorMap& grf) override;

  void setGRF(const LegVectorMap& grf) override;

  bool isStance(LegID leg) const override;

protected:
  LegBoolMap stance_;
  LegVectorMap grf_;
  double force_threshold_;
};

}  // namespace quadruped
}  // namespace pronto
