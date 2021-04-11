#include "pronto_anymal_b_commons/feet_jacobians.hpp"

using namespace pronto::quadruped;
using namespace iit::rbd;

namespace pronto {
namespace anymal {


FootJac FeetJacobians::getFootJacobian(const JointState& q,
                                               const LegID& leg)
{
    switch(leg){
    case LF:
        return getFootJacobianLF(q);
    case RF:
        return getFootJacobianRF(q);
    case LH:
        return getFootJacobianLH(q);
    case RH:
        return getFootJacobianRH(q);
    default:
        return Matrix33d::Identity();
    }
}

FootJac FeetJacobians::getFootJacobianLF(const JointState& q){
    return jacs_.fr_base_J_LF_FOOT(q).block<3,3>(LX,0);
}
FootJac FeetJacobians::getFootJacobianRF(const JointState& q){
    return jacs_.fr_base_J_RF_FOOT(q).block<3,3>(LX,0);
}
FootJac FeetJacobians::getFootJacobianLH(const JointState& q){
    return jacs_.fr_base_J_LH_FOOT(q).block<3,3>(LX,0);
}
FootJac FeetJacobians::getFootJacobianRH(const JointState& q){
    return jacs_.fr_base_J_RH_FOOT(q).block<3,3>(LX,0);
}

FootJac FeetJacobians::getFootJacobianAngular(const JointState &q, const LegID &leg){
  switch(leg){
  case LF:
      return jacs_.fr_base_J_LF_FOOT(q).block<3,3>(AX,0);
  case RF:
      return jacs_.fr_base_J_RF_FOOT(q).block<3,3>(AX,0);
  case LH:
      return jacs_.fr_base_J_LH_FOOT(q).block<3,3>(AX,0);
  case RH:
      return jacs_.fr_base_J_RH_FOOT(q).block<3,3>(AX,0);
  default:
      return Matrix33d::Identity();
  }
}

}
}



