#include "pronto_quadruped/StanceDetector.hpp"

namespace pronto {
namespace quadruped {

void StanceDetector::setStance(bool lf, bool rf, bool lh, bool rh){
    stance_[LF] = lf;
    stance_[RH] = rf;
    stance_[LF] = lh;
    stance_[RH] = rh;
}

LegBoolMap StanceDetector::getStance(const double time,
                                     const JointState &q,
                                     const JointState &qd,
                                     const JointState &tau,
                                     const Quaterniond & orient,
                                     const JointState &qdd,
                                     const Vector3d& xd,
                                     const Vector3d & xdd,
                                     const Vector3d & omega,
                                     const Vector3d & omegad) {
  LegBoolMap stance;
  getStance(time, q, qd, tau, orient, stance, qdd, xd, xdd, omega, omegad);
  return stance;
}

bool StanceDetector::getStance(const double time,
                               const JointState &q,
                               const JointState &qd,
                               const JointState &tau,
                               const Quaterniond & orient,
                               LegBoolMap& stance,
                               const JointState &qdd,
                               const Vector3d& xd,
                               const Vector3d & xdd,
                               const Vector3d & omega,
                               const Vector3d & omegad) {
  LegScalarMap l;
  return getStance(time, q, qd, tau, orient, stance, l, qdd, xd, xdd, omega, omegad);
}

bool StanceDetector::getStance(const double time,
                               const JointState &q,
                               const JointState &qd,
                               const JointState &tau,
                               const Quaterniond & orient,
                               LegBoolMap& stance,
                               LegScalarMap& stance_probability,
                               const JointState &qdd,
                               const Vector3d& xd,
                               const Vector3d & xdd,
                               const Vector3d & omega,
                               const Vector3d & omegad) {

  // ignore all the inputs, we get the stance from an external source
  stance = stance_;
  for(int i = 0; i < 4; i++){
    stance_probability[LegID(i)] = static_cast<double>(stance_[LegID(i)]);
  }
  return true;
}

StanceDetector::LegVectorMap StanceDetector::getGRF(){
  LegVectorMap map;
  // assume perfect vertical forces of 100N on the stance legs
  for(int i = 0; i < 4; i++){
    map[LegID(i)] = Eigen::Vector3d(0,0, stance_[LegID(i)]*100);
  }
  return map;
}

}
}


