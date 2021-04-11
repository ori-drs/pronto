#include "jacobians.h"

pronto::anymal::Jacobians::Jacobians()
:    fr_base_J_LF_FOOT(), 
    fr_base_J_RF_FOOT(), 
    fr_base_J_LH_FOOT(), 
    fr_base_J_RH_FOOT(), 
    imu_link_J_LF_FOOT(), 
    imu_link_J_RF_FOOT(), 
    imu_link_J_LH_FOOT(), 
    imu_link_J_RH_FOOT()
{}

void pronto::anymal::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles)
{
    params.lengths = _lengths;
    params.angles = _angles;
    params.trig.update();
}

pronto::anymal::Jacobians::Type_fr_base_J_LF_FOOT::Type_fr_base_J_LF_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const pronto::anymal::Jacobians::Type_fr_base_J_LF_FOOT& pronto::anymal::Jacobians::Type_fr_base_J_LF_FOOT::update(const JointState& q)
{
    Scalar sin_q_LF_HAA  = ScalarTraits::sin( q(LF_HAA) );
    Scalar cos_q_LF_HAA  = ScalarTraits::cos( q(LF_HAA) );
    Scalar sin_q_LF_HFE  = ScalarTraits::sin( q(LF_HFE) );
    Scalar cos_q_LF_HFE  = ScalarTraits::cos( q(LF_HFE) );
    Scalar sin_q_LF_KFE  = ScalarTraits::sin( q(LF_KFE) );
    Scalar cos_q_LF_KFE  = ScalarTraits::cos( q(LF_KFE) );
    (*this)(1,1) = cos_q_LF_HAA;
    (*this)(1,2) = cos_q_LF_HAA;
    (*this)(2,1) = sin_q_LF_HAA;
    (*this)(2,2) = sin_q_LF_HAA;
    (*this)(3,1) = ((( tx_LF_FOOT * sin_q_LF_HFE)+( ty_LF_FOOT * cos_q_LF_HFE)) * sin_q_LF_KFE)+((( ty_LF_FOOT * sin_q_LF_HFE)-( tx_LF_FOOT * cos_q_LF_HFE)) * cos_q_LF_KFE)-( tx_LF_KFE * cos_q_LF_HFE);
    (*this)(3,2) = ((( tx_LF_FOOT * sin_q_LF_HFE)+( ty_LF_FOOT * cos_q_LF_HFE)) * sin_q_LF_KFE)+((( ty_LF_FOOT * sin_q_LF_HFE)-( tx_LF_FOOT * cos_q_LF_HFE)) * cos_q_LF_KFE);
    (*this)(4,0) = (((- tx_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)-( ty_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)) * sin_q_LF_KFE)+((( tx_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)-( ty_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)) * cos_q_LF_KFE)+( tx_LF_KFE * cos_q_LF_HAA * cos_q_LF_HFE)+((- tz_LF_KFE- tz_LF_FOOT- ty_LF_HFE) * sin_q_LF_HAA);
    (*this)(4,1) = ((( ty_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)-( tx_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * sin_q_LF_KFE)+(((- tx_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)-( ty_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * cos_q_LF_KFE)-( tx_LF_KFE * sin_q_LF_HAA * sin_q_LF_HFE);
    (*this)(4,2) = ((( ty_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)-( tx_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * sin_q_LF_KFE)+(((- tx_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)-( ty_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * cos_q_LF_KFE);
    (*this)(5,0) = (((- tx_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)-( ty_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * sin_q_LF_KFE)+((( tx_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)-( ty_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)) * cos_q_LF_KFE)+( tx_LF_KFE * sin_q_LF_HAA * cos_q_LF_HFE)+(( tz_LF_KFE+ tz_LF_FOOT+ ty_LF_HFE) * cos_q_LF_HAA);
    (*this)(5,1) = ((( tx_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)-( ty_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)) * sin_q_LF_KFE)+((( tx_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)+( ty_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)) * cos_q_LF_KFE)+( tx_LF_KFE * cos_q_LF_HAA * sin_q_LF_HFE);
    (*this)(5,2) = ((( tx_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)-( ty_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)) * sin_q_LF_KFE)+((( tx_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)+( ty_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)) * cos_q_LF_KFE);
    return *this;
}

pronto::anymal::Jacobians::Type_fr_base_J_RF_FOOT::Type_fr_base_J_RF_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const pronto::anymal::Jacobians::Type_fr_base_J_RF_FOOT& pronto::anymal::Jacobians::Type_fr_base_J_RF_FOOT::update(const JointState& q)
{
    Scalar sin_q_RF_HAA  = ScalarTraits::sin( q(RF_HAA) );
    Scalar cos_q_RF_HAA  = ScalarTraits::cos( q(RF_HAA) );
    Scalar sin_q_RF_HFE  = ScalarTraits::sin( q(RF_HFE) );
    Scalar cos_q_RF_HFE  = ScalarTraits::cos( q(RF_HFE) );
    Scalar sin_q_RF_KFE  = ScalarTraits::sin( q(RF_KFE) );
    Scalar cos_q_RF_KFE  = ScalarTraits::cos( q(RF_KFE) );
    (*this)(1,1) = cos_q_RF_HAA;
    (*this)(1,2) = cos_q_RF_HAA;
    (*this)(2,1) = sin_q_RF_HAA;
    (*this)(2,2) = sin_q_RF_HAA;
    (*this)(3,1) = ((( tx_RF_FOOT * sin_q_RF_HFE)+( ty_RF_FOOT * cos_q_RF_HFE)) * sin_q_RF_KFE)+((( ty_RF_FOOT * sin_q_RF_HFE)-( tx_RF_FOOT * cos_q_RF_HFE)) * cos_q_RF_KFE)-( tx_RF_KFE * cos_q_RF_HFE);
    (*this)(3,2) = ((( tx_RF_FOOT * sin_q_RF_HFE)+( ty_RF_FOOT * cos_q_RF_HFE)) * sin_q_RF_KFE)+((( ty_RF_FOOT * sin_q_RF_HFE)-( tx_RF_FOOT * cos_q_RF_HFE)) * cos_q_RF_KFE);
    (*this)(4,0) = (((- tx_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)-( ty_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)) * sin_q_RF_KFE)+((( tx_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)-( ty_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)) * cos_q_RF_KFE)+( tx_RF_KFE * cos_q_RF_HAA * cos_q_RF_HFE)+((- tz_RF_KFE- tz_RF_FOOT- ty_RF_HFE) * sin_q_RF_HAA);
    (*this)(4,1) = ((( ty_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)-( tx_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * sin_q_RF_KFE)+(((- tx_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)-( ty_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * cos_q_RF_KFE)-( tx_RF_KFE * sin_q_RF_HAA * sin_q_RF_HFE);
    (*this)(4,2) = ((( ty_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)-( tx_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * sin_q_RF_KFE)+(((- tx_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)-( ty_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * cos_q_RF_KFE);
    (*this)(5,0) = (((- tx_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)-( ty_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * sin_q_RF_KFE)+((( tx_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)-( ty_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)) * cos_q_RF_KFE)+( tx_RF_KFE * sin_q_RF_HAA * cos_q_RF_HFE)+(( tz_RF_KFE+ tz_RF_FOOT+ ty_RF_HFE) * cos_q_RF_HAA);
    (*this)(5,1) = ((( tx_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)-( ty_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)) * sin_q_RF_KFE)+((( tx_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)+( ty_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)) * cos_q_RF_KFE)+( tx_RF_KFE * cos_q_RF_HAA * sin_q_RF_HFE);
    (*this)(5,2) = ((( tx_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)-( ty_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)) * sin_q_RF_KFE)+((( tx_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)+( ty_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)) * cos_q_RF_KFE);
    return *this;
}

pronto::anymal::Jacobians::Type_fr_base_J_LH_FOOT::Type_fr_base_J_LH_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const pronto::anymal::Jacobians::Type_fr_base_J_LH_FOOT& pronto::anymal::Jacobians::Type_fr_base_J_LH_FOOT::update(const JointState& q)
{
    Scalar sin_q_LH_HAA  = ScalarTraits::sin( q(LH_HAA) );
    Scalar cos_q_LH_HAA  = ScalarTraits::cos( q(LH_HAA) );
    Scalar sin_q_LH_HFE  = ScalarTraits::sin( q(LH_HFE) );
    Scalar cos_q_LH_HFE  = ScalarTraits::cos( q(LH_HFE) );
    Scalar sin_q_LH_KFE  = ScalarTraits::sin( q(LH_KFE) );
    Scalar cos_q_LH_KFE  = ScalarTraits::cos( q(LH_KFE) );
    (*this)(1,1) = cos_q_LH_HAA;
    (*this)(1,2) = cos_q_LH_HAA;
    (*this)(2,1) = sin_q_LH_HAA;
    (*this)(2,2) = sin_q_LH_HAA;
    (*this)(3,1) = ((( tx_LH_FOOT * sin_q_LH_HFE)+( ty_LH_FOOT * cos_q_LH_HFE)) * sin_q_LH_KFE)+((( ty_LH_FOOT * sin_q_LH_HFE)-( tx_LH_FOOT * cos_q_LH_HFE)) * cos_q_LH_KFE)-( tx_LH_KFE * cos_q_LH_HFE);
    (*this)(3,2) = ((( tx_LH_FOOT * sin_q_LH_HFE)+( ty_LH_FOOT * cos_q_LH_HFE)) * sin_q_LH_KFE)+((( ty_LH_FOOT * sin_q_LH_HFE)-( tx_LH_FOOT * cos_q_LH_HFE)) * cos_q_LH_KFE);
    (*this)(4,0) = (((- tx_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)-( ty_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)) * sin_q_LH_KFE)+((( tx_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)-( ty_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)) * cos_q_LH_KFE)+( tx_LH_KFE * cos_q_LH_HAA * cos_q_LH_HFE)+((- tz_LH_KFE- tz_LH_FOOT- ty_LH_HFE) * sin_q_LH_HAA);
    (*this)(4,1) = ((( ty_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)-( tx_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * sin_q_LH_KFE)+(((- tx_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)-( ty_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * cos_q_LH_KFE)-( tx_LH_KFE * sin_q_LH_HAA * sin_q_LH_HFE);
    (*this)(4,2) = ((( ty_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)-( tx_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * sin_q_LH_KFE)+(((- tx_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)-( ty_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * cos_q_LH_KFE);
    (*this)(5,0) = (((- tx_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)-( ty_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * sin_q_LH_KFE)+((( tx_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)-( ty_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)) * cos_q_LH_KFE)+( tx_LH_KFE * sin_q_LH_HAA * cos_q_LH_HFE)+(( tz_LH_KFE+ tz_LH_FOOT+ ty_LH_HFE) * cos_q_LH_HAA);
    (*this)(5,1) = ((( tx_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)-( ty_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)) * sin_q_LH_KFE)+((( tx_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)+( ty_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)) * cos_q_LH_KFE)+( tx_LH_KFE * cos_q_LH_HAA * sin_q_LH_HFE);
    (*this)(5,2) = ((( tx_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)-( ty_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)) * sin_q_LH_KFE)+((( tx_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)+( ty_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)) * cos_q_LH_KFE);
    return *this;
}

pronto::anymal::Jacobians::Type_fr_base_J_RH_FOOT::Type_fr_base_J_RH_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const pronto::anymal::Jacobians::Type_fr_base_J_RH_FOOT& pronto::anymal::Jacobians::Type_fr_base_J_RH_FOOT::update(const JointState& q)
{
    Scalar sin_q_RH_HAA  = ScalarTraits::sin( q(RH_HAA) );
    Scalar cos_q_RH_HAA  = ScalarTraits::cos( q(RH_HAA) );
    Scalar sin_q_RH_HFE  = ScalarTraits::sin( q(RH_HFE) );
    Scalar cos_q_RH_HFE  = ScalarTraits::cos( q(RH_HFE) );
    Scalar sin_q_RH_KFE  = ScalarTraits::sin( q(RH_KFE) );
    Scalar cos_q_RH_KFE  = ScalarTraits::cos( q(RH_KFE) );
    (*this)(1,1) = cos_q_RH_HAA;
    (*this)(1,2) = cos_q_RH_HAA;
    (*this)(2,1) = sin_q_RH_HAA;
    (*this)(2,2) = sin_q_RH_HAA;
    (*this)(3,1) = ((( tx_RH_FOOT * sin_q_RH_HFE)+( ty_RH_FOOT * cos_q_RH_HFE)) * sin_q_RH_KFE)+((( ty_RH_FOOT * sin_q_RH_HFE)-( tx_RH_FOOT * cos_q_RH_HFE)) * cos_q_RH_KFE)-( tx_RH_KFE * cos_q_RH_HFE);
    (*this)(3,2) = ((( tx_RH_FOOT * sin_q_RH_HFE)+( ty_RH_FOOT * cos_q_RH_HFE)) * sin_q_RH_KFE)+((( ty_RH_FOOT * sin_q_RH_HFE)-( tx_RH_FOOT * cos_q_RH_HFE)) * cos_q_RH_KFE);
    (*this)(4,0) = (((- tx_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)-( ty_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)) * sin_q_RH_KFE)+((( tx_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)-( ty_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)) * cos_q_RH_KFE)+( tx_RH_KFE * cos_q_RH_HAA * cos_q_RH_HFE)+((- tz_RH_KFE- tz_RH_FOOT- ty_RH_HFE) * sin_q_RH_HAA);
    (*this)(4,1) = ((( ty_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)-( tx_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * sin_q_RH_KFE)+(((- tx_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)-( ty_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * cos_q_RH_KFE)-( tx_RH_KFE * sin_q_RH_HAA * sin_q_RH_HFE);
    (*this)(4,2) = ((( ty_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)-( tx_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * sin_q_RH_KFE)+(((- tx_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)-( ty_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * cos_q_RH_KFE);
    (*this)(5,0) = (((- tx_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)-( ty_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * sin_q_RH_KFE)+((( tx_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)-( ty_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)) * cos_q_RH_KFE)+( tx_RH_KFE * sin_q_RH_HAA * cos_q_RH_HFE)+(( tz_RH_KFE+ tz_RH_FOOT+ ty_RH_HFE) * cos_q_RH_HAA);
    (*this)(5,1) = ((( tx_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)-( ty_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)) * sin_q_RH_KFE)+((( tx_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)+( ty_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)) * cos_q_RH_KFE)+( tx_RH_KFE * cos_q_RH_HAA * sin_q_RH_HFE);
    (*this)(5,2) = ((( tx_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)-( ty_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)) * sin_q_RH_KFE)+((( tx_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)+( ty_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)) * cos_q_RH_KFE);
    return *this;
}

pronto::anymal::Jacobians::Type_imu_link_J_LF_FOOT::Type_imu_link_J_LF_FOOT()
{
    (*this)(0,0) = -1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const pronto::anymal::Jacobians::Type_imu_link_J_LF_FOOT& pronto::anymal::Jacobians::Type_imu_link_J_LF_FOOT::update(const JointState& q)
{
    Scalar sin_q_LF_HAA  = ScalarTraits::sin( q(LF_HAA) );
    Scalar cos_q_LF_HAA  = ScalarTraits::cos( q(LF_HAA) );
    Scalar sin_q_LF_HFE  = ScalarTraits::sin( q(LF_HFE) );
    Scalar cos_q_LF_HFE  = ScalarTraits::cos( q(LF_HFE) );
    Scalar sin_q_LF_KFE  = ScalarTraits::sin( q(LF_KFE) );
    Scalar cos_q_LF_KFE  = ScalarTraits::cos( q(LF_KFE) );
    (*this)(1,1) = cos_q_LF_HAA;
    (*this)(1,2) = cos_q_LF_HAA;
    (*this)(2,1) = -sin_q_LF_HAA;
    (*this)(2,2) = -sin_q_LF_HAA;
    (*this)(3,1) = (((- tx_LF_FOOT * sin_q_LF_HFE)-( ty_LF_FOOT * cos_q_LF_HFE)) * sin_q_LF_KFE)+((( tx_LF_FOOT * cos_q_LF_HFE)-( ty_LF_FOOT * sin_q_LF_HFE)) * cos_q_LF_KFE)+( tx_LF_KFE * cos_q_LF_HFE);
    (*this)(3,2) = (((- tx_LF_FOOT * sin_q_LF_HFE)-( ty_LF_FOOT * cos_q_LF_HFE)) * sin_q_LF_KFE)+((( tx_LF_FOOT * cos_q_LF_HFE)-( ty_LF_FOOT * sin_q_LF_HFE)) * cos_q_LF_KFE);
    (*this)(4,0) = (((- tx_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)-( ty_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)) * sin_q_LF_KFE)+((( tx_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)-( ty_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)) * cos_q_LF_KFE)+( tx_LF_KFE * cos_q_LF_HAA * cos_q_LF_HFE)+((- tz_LF_KFE- tz_LF_FOOT- ty_LF_HFE) * sin_q_LF_HAA);
    (*this)(4,1) = ((( ty_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)-( tx_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * sin_q_LF_KFE)+(((- tx_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)-( ty_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * cos_q_LF_KFE)-( tx_LF_KFE * sin_q_LF_HAA * sin_q_LF_HFE);
    (*this)(4,2) = ((( ty_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)-( tx_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * sin_q_LF_KFE)+(((- tx_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)-( ty_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * cos_q_LF_KFE);
    (*this)(5,0) = ((( tx_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)+( ty_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * sin_q_LF_KFE)+((( ty_LF_FOOT * sin_q_LF_HAA * sin_q_LF_HFE)-( tx_LF_FOOT * sin_q_LF_HAA * cos_q_LF_HFE)) * cos_q_LF_KFE)-( tx_LF_KFE * sin_q_LF_HAA * cos_q_LF_HFE)+((- tz_LF_KFE- tz_LF_FOOT- ty_LF_HFE) * cos_q_LF_HAA);
    (*this)(5,1) = ((( ty_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)-( tx_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)) * sin_q_LF_KFE)+(((- tx_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)-( ty_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)) * cos_q_LF_KFE)-( tx_LF_KFE * cos_q_LF_HAA * sin_q_LF_HFE);
    (*this)(5,2) = ((( ty_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)-( tx_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)) * sin_q_LF_KFE)+(((- tx_LF_FOOT * cos_q_LF_HAA * sin_q_LF_HFE)-( ty_LF_FOOT * cos_q_LF_HAA * cos_q_LF_HFE)) * cos_q_LF_KFE);
    return *this;
}

pronto::anymal::Jacobians::Type_imu_link_J_RF_FOOT::Type_imu_link_J_RF_FOOT()
{
    (*this)(0,0) = -1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const pronto::anymal::Jacobians::Type_imu_link_J_RF_FOOT& pronto::anymal::Jacobians::Type_imu_link_J_RF_FOOT::update(const JointState& q)
{
    Scalar sin_q_RF_HAA  = ScalarTraits::sin( q(RF_HAA) );
    Scalar cos_q_RF_HAA  = ScalarTraits::cos( q(RF_HAA) );
    Scalar sin_q_RF_HFE  = ScalarTraits::sin( q(RF_HFE) );
    Scalar cos_q_RF_HFE  = ScalarTraits::cos( q(RF_HFE) );
    Scalar sin_q_RF_KFE  = ScalarTraits::sin( q(RF_KFE) );
    Scalar cos_q_RF_KFE  = ScalarTraits::cos( q(RF_KFE) );
    (*this)(1,1) = cos_q_RF_HAA;
    (*this)(1,2) = cos_q_RF_HAA;
    (*this)(2,1) = -sin_q_RF_HAA;
    (*this)(2,2) = -sin_q_RF_HAA;
    (*this)(3,1) = (((- tx_RF_FOOT * sin_q_RF_HFE)-( ty_RF_FOOT * cos_q_RF_HFE)) * sin_q_RF_KFE)+((( tx_RF_FOOT * cos_q_RF_HFE)-( ty_RF_FOOT * sin_q_RF_HFE)) * cos_q_RF_KFE)+( tx_RF_KFE * cos_q_RF_HFE);
    (*this)(3,2) = (((- tx_RF_FOOT * sin_q_RF_HFE)-( ty_RF_FOOT * cos_q_RF_HFE)) * sin_q_RF_KFE)+((( tx_RF_FOOT * cos_q_RF_HFE)-( ty_RF_FOOT * sin_q_RF_HFE)) * cos_q_RF_KFE);
    (*this)(4,0) = (((- tx_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)-( ty_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)) * sin_q_RF_KFE)+((( tx_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)-( ty_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)) * cos_q_RF_KFE)+( tx_RF_KFE * cos_q_RF_HAA * cos_q_RF_HFE)+((- tz_RF_KFE- tz_RF_FOOT- ty_RF_HFE) * sin_q_RF_HAA);
    (*this)(4,1) = ((( ty_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)-( tx_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * sin_q_RF_KFE)+(((- tx_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)-( ty_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * cos_q_RF_KFE)-( tx_RF_KFE * sin_q_RF_HAA * sin_q_RF_HFE);
    (*this)(4,2) = ((( ty_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)-( tx_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * sin_q_RF_KFE)+(((- tx_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)-( ty_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * cos_q_RF_KFE);
    (*this)(5,0) = ((( tx_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)+( ty_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * sin_q_RF_KFE)+((( ty_RF_FOOT * sin_q_RF_HAA * sin_q_RF_HFE)-( tx_RF_FOOT * sin_q_RF_HAA * cos_q_RF_HFE)) * cos_q_RF_KFE)-( tx_RF_KFE * sin_q_RF_HAA * cos_q_RF_HFE)+((- tz_RF_KFE- tz_RF_FOOT- ty_RF_HFE) * cos_q_RF_HAA);
    (*this)(5,1) = ((( ty_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)-( tx_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)) * sin_q_RF_KFE)+(((- tx_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)-( ty_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)) * cos_q_RF_KFE)-( tx_RF_KFE * cos_q_RF_HAA * sin_q_RF_HFE);
    (*this)(5,2) = ((( ty_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)-( tx_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)) * sin_q_RF_KFE)+(((- tx_RF_FOOT * cos_q_RF_HAA * sin_q_RF_HFE)-( ty_RF_FOOT * cos_q_RF_HAA * cos_q_RF_HFE)) * cos_q_RF_KFE);
    return *this;
}

pronto::anymal::Jacobians::Type_imu_link_J_LH_FOOT::Type_imu_link_J_LH_FOOT()
{
    (*this)(0,0) = -1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const pronto::anymal::Jacobians::Type_imu_link_J_LH_FOOT& pronto::anymal::Jacobians::Type_imu_link_J_LH_FOOT::update(const JointState& q)
{
    Scalar sin_q_LH_HAA  = ScalarTraits::sin( q(LH_HAA) );
    Scalar cos_q_LH_HAA  = ScalarTraits::cos( q(LH_HAA) );
    Scalar sin_q_LH_HFE  = ScalarTraits::sin( q(LH_HFE) );
    Scalar cos_q_LH_HFE  = ScalarTraits::cos( q(LH_HFE) );
    Scalar sin_q_LH_KFE  = ScalarTraits::sin( q(LH_KFE) );
    Scalar cos_q_LH_KFE  = ScalarTraits::cos( q(LH_KFE) );
    (*this)(1,1) = cos_q_LH_HAA;
    (*this)(1,2) = cos_q_LH_HAA;
    (*this)(2,1) = -sin_q_LH_HAA;
    (*this)(2,2) = -sin_q_LH_HAA;
    (*this)(3,1) = (((- tx_LH_FOOT * sin_q_LH_HFE)-( ty_LH_FOOT * cos_q_LH_HFE)) * sin_q_LH_KFE)+((( tx_LH_FOOT * cos_q_LH_HFE)-( ty_LH_FOOT * sin_q_LH_HFE)) * cos_q_LH_KFE)+( tx_LH_KFE * cos_q_LH_HFE);
    (*this)(3,2) = (((- tx_LH_FOOT * sin_q_LH_HFE)-( ty_LH_FOOT * cos_q_LH_HFE)) * sin_q_LH_KFE)+((( tx_LH_FOOT * cos_q_LH_HFE)-( ty_LH_FOOT * sin_q_LH_HFE)) * cos_q_LH_KFE);
    (*this)(4,0) = (((- tx_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)-( ty_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)) * sin_q_LH_KFE)+((( tx_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)-( ty_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)) * cos_q_LH_KFE)+( tx_LH_KFE * cos_q_LH_HAA * cos_q_LH_HFE)+((- tz_LH_KFE- tz_LH_FOOT- ty_LH_HFE) * sin_q_LH_HAA);
    (*this)(4,1) = ((( ty_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)-( tx_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * sin_q_LH_KFE)+(((- tx_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)-( ty_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * cos_q_LH_KFE)-( tx_LH_KFE * sin_q_LH_HAA * sin_q_LH_HFE);
    (*this)(4,2) = ((( ty_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)-( tx_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * sin_q_LH_KFE)+(((- tx_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)-( ty_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * cos_q_LH_KFE);
    (*this)(5,0) = ((( tx_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)+( ty_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * sin_q_LH_KFE)+((( ty_LH_FOOT * sin_q_LH_HAA * sin_q_LH_HFE)-( tx_LH_FOOT * sin_q_LH_HAA * cos_q_LH_HFE)) * cos_q_LH_KFE)-( tx_LH_KFE * sin_q_LH_HAA * cos_q_LH_HFE)+((- tz_LH_KFE- tz_LH_FOOT- ty_LH_HFE) * cos_q_LH_HAA);
    (*this)(5,1) = ((( ty_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)-( tx_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)) * sin_q_LH_KFE)+(((- tx_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)-( ty_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)) * cos_q_LH_KFE)-( tx_LH_KFE * cos_q_LH_HAA * sin_q_LH_HFE);
    (*this)(5,2) = ((( ty_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)-( tx_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)) * sin_q_LH_KFE)+(((- tx_LH_FOOT * cos_q_LH_HAA * sin_q_LH_HFE)-( ty_LH_FOOT * cos_q_LH_HAA * cos_q_LH_HFE)) * cos_q_LH_KFE);
    return *this;
}

pronto::anymal::Jacobians::Type_imu_link_J_RH_FOOT::Type_imu_link_J_RH_FOOT()
{
    (*this)(0,0) = -1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const pronto::anymal::Jacobians::Type_imu_link_J_RH_FOOT& pronto::anymal::Jacobians::Type_imu_link_J_RH_FOOT::update(const JointState& q)
{
    Scalar sin_q_RH_HAA  = ScalarTraits::sin( q(RH_HAA) );
    Scalar cos_q_RH_HAA  = ScalarTraits::cos( q(RH_HAA) );
    Scalar sin_q_RH_HFE  = ScalarTraits::sin( q(RH_HFE) );
    Scalar cos_q_RH_HFE  = ScalarTraits::cos( q(RH_HFE) );
    Scalar sin_q_RH_KFE  = ScalarTraits::sin( q(RH_KFE) );
    Scalar cos_q_RH_KFE  = ScalarTraits::cos( q(RH_KFE) );
    (*this)(1,1) = cos_q_RH_HAA;
    (*this)(1,2) = cos_q_RH_HAA;
    (*this)(2,1) = -sin_q_RH_HAA;
    (*this)(2,2) = -sin_q_RH_HAA;
    (*this)(3,1) = (((- tx_RH_FOOT * sin_q_RH_HFE)-( ty_RH_FOOT * cos_q_RH_HFE)) * sin_q_RH_KFE)+((( tx_RH_FOOT * cos_q_RH_HFE)-( ty_RH_FOOT * sin_q_RH_HFE)) * cos_q_RH_KFE)+( tx_RH_KFE * cos_q_RH_HFE);
    (*this)(3,2) = (((- tx_RH_FOOT * sin_q_RH_HFE)-( ty_RH_FOOT * cos_q_RH_HFE)) * sin_q_RH_KFE)+((( tx_RH_FOOT * cos_q_RH_HFE)-( ty_RH_FOOT * sin_q_RH_HFE)) * cos_q_RH_KFE);
    (*this)(4,0) = (((- tx_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)-( ty_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)) * sin_q_RH_KFE)+((( tx_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)-( ty_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)) * cos_q_RH_KFE)+( tx_RH_KFE * cos_q_RH_HAA * cos_q_RH_HFE)+((- tz_RH_KFE- tz_RH_FOOT- ty_RH_HFE) * sin_q_RH_HAA);
    (*this)(4,1) = ((( ty_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)-( tx_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * sin_q_RH_KFE)+(((- tx_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)-( ty_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * cos_q_RH_KFE)-( tx_RH_KFE * sin_q_RH_HAA * sin_q_RH_HFE);
    (*this)(4,2) = ((( ty_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)-( tx_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * sin_q_RH_KFE)+(((- tx_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)-( ty_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * cos_q_RH_KFE);
    (*this)(5,0) = ((( tx_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)+( ty_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * sin_q_RH_KFE)+((( ty_RH_FOOT * sin_q_RH_HAA * sin_q_RH_HFE)-( tx_RH_FOOT * sin_q_RH_HAA * cos_q_RH_HFE)) * cos_q_RH_KFE)-( tx_RH_KFE * sin_q_RH_HAA * cos_q_RH_HFE)+((- tz_RH_KFE- tz_RH_FOOT- ty_RH_HFE) * cos_q_RH_HAA);
    (*this)(5,1) = ((( ty_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)-( tx_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)) * sin_q_RH_KFE)+(((- tx_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)-( ty_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)) * cos_q_RH_KFE)-( tx_RH_KFE * cos_q_RH_HAA * sin_q_RH_HFE);
    (*this)(5,2) = ((( ty_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)-( tx_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)) * sin_q_RH_KFE)+(((- tx_RH_FOOT * cos_q_RH_HAA * sin_q_RH_HFE)-( ty_RH_FOOT * cos_q_RH_HAA * cos_q_RH_HFE)) * cos_q_RH_KFE);
    return *this;
}

