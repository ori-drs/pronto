#include "pronto_anymal_b_commons/forward_kinematics.hpp"
#include <iit/rbd/utils.h>

using namespace pronto::quadruped;
using namespace iit::rbd;

namespace pronto {
namespace anymal {

ForwardKinematics::ForwardKinematics()
{
}

Vector3d ForwardKinematics::getFootPosLF(const JointState& q){
    return Utils::positionVector(ht_.fr_base_X_LF_FOOT(q));
}

Vector3d ForwardKinematics::getFootPosRF(const JointState& q){
    return Utils::positionVector(ht_.fr_base_X_RF_FOOT(q));
}

Vector3d ForwardKinematics::getFootPosLH(const JointState& q){
    return Utils::positionVector(ht_.fr_base_X_LH_FOOT(q));
}

Vector3d ForwardKinematics::getFootPosRH(const JointState& q){
    return Utils::positionVector(ht_.fr_base_X_RH_FOOT(q));
}

Vector3d ForwardKinematics::getFootPos(const JointState& q, const LegID& leg){
    switch(leg){
    case LegID::LF:
        return getFootPosLF(q);
    case LegID::RF:
        return getFootPosRF(q);
    case LegID::LH:
        return getFootPosLH(q);
    case LegID::RH:
        return getFootPosRH(q);
    default:
        return Vector3d::Zero();
    }
}

Matrix3d ForwardKinematics::getFootOrientation(const JointState &q, const LegID &leg){

    switch(leg){
    case LegID::LF:
        return Utils::rotationMx(ht_.fr_base_X_LF_FOOT);
    case LegID::RF:
        return Utils::rotationMx(ht_.fr_base_X_RF_FOOT);
    case LegID::LH:
        return Utils::rotationMx(ht_.fr_base_X_LH_FOOT);
    case LegID::RH:
        return Utils::rotationMx(ht_.fr_base_X_RH_FOOT);
    default:
        std::cerr << "[ ForwardKinematics::getFootOrientation(...) ] "
                  << "ERROR: legID not recognized. Returning identity."
                  << std::endl;
        return Matrix3d::Identity();
    }
}

Vector3d ForwardKinematics::getShinPos(const JointState& q,
                                               const double& contact_pos,
                                               const LegID& leg){
    return getFootPos(q, leg);
}


}
}

