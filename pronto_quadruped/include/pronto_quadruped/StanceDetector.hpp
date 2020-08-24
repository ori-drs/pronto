#pragma once
#include "pronto_quadruped/StanceEstimatorBase.hpp"

namespace pronto {
namespace quadruped {

class StanceDetector : StanceEstimatorBase {
public:
  LegBoolMap getStance(const double time,
                       const JointState &q,
                       const JointState &qd,
                       const JointState &tau,
                       const Quaterniond & orient,
                       const JointState &qdd = JointState::Constant(0),
                       const Vector3d& xd = Vector3d(0, 0, 0),
                       const Vector3d & xdd  = Vector3d(0, 0, 0),
                       const Vector3d & omega  = Vector3d(0, 0, 0),
                       const Vector3d & omegad = Vector3d(0, 0, 0)) override;

  bool getStance(const double time,
                 const JointState &q,
                 const JointState &qd,
                 const JointState &tau,
                 const Quaterniond & orient,
                 LegBoolMap& stance,
                 const JointState &qdd = JointState::Constant(0),
                 const Vector3d& xd = Vector3d(0, 0, 0),
                 const Vector3d & xdd  = Vector3d(0, 0, 0),
                 const Vector3d & omega  = Vector3d(0, 0, 0),
                 const Vector3d & omegad = Vector3d(0, 0, 0)) override;

  bool getStance(const double time,
                 const JointState &q,
                 const JointState &qd,
                 const JointState &tau,
                 const Quaterniond & orient,
                 LegBoolMap& stance,
                 LegScalarMap& stance_probability,
                 const JointState &qdd = JointState::Constant(0),
                 const Vector3d& xd = Vector3d(0, 0, 0),
                 const Vector3d & xdd  = Vector3d(0, 0, 0),
                 const Vector3d & omega  = Vector3d(0, 0, 0),
                 const Vector3d & omegad = Vector3d(0, 0, 0)) override;

  LegVectorMap getGRF() override;

protected:
  void setStance(bool lf, bool rf, bool lh, bool rh);

private:
  LegBoolMap stance_;
};

}
}
