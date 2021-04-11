#pragma once

#include <pronto_quadruped_commons/feet_jacobians.h>
#include <anymal_b_robcogen/jacobians.h>

namespace pronto {
namespace anymal {

class FeetJacobians : public pronto::quadruped::FeetJacobians {
public:
    typedef pronto::quadruped::FootJac FootJac;
    typedef pronto::quadruped::JointState JointState;
    typedef pronto::quadruped::LegID LegID;

    inline FeetJacobians() {}

    virtual ~FeetJacobians() {}

    FootJac getFootJacobian(const JointState& q, const LegID& leg) override;
    FootJac getFootJacobianLF(const JointState& q) override;
    FootJac getFootJacobianRF(const JointState& q) override;
    FootJac getFootJacobianLH(const JointState& q) override;
    FootJac getFootJacobianRH(const JointState& q) override;

    FootJac getFootJacobianAngular(const JointState &q, const LegID &leg) override;

private:
    pronto::anymal::Jacobians jacs_;
};

}
}

