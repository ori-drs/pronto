#ifndef RCG__ANYMAL_TRAITS_H_
#define RCG__ANYMAL_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace pronto {
namespace anymal {
struct Traits {
    typedef typename pronto::anymal::ScalarTraits ScalarTraits;

    typedef typename pronto::anymal::JointState JointState;

    typedef typename pronto::anymal::JointIdentifiers JointID;
    typedef typename pronto::anymal::LinkIdentifiers  LinkID;

    typedef typename pronto::anymal::HomogeneousTransforms HomogeneousTransforms;
    typedef typename pronto::anymal::MotionTransforms MotionTransforms;
    typedef typename pronto::anymal::ForceTransforms ForceTransforms;

    typedef typename pronto::anymal::dyn::InertiaProperties InertiaProperties;
    typedef typename pronto::anymal::dyn::ForwardDynamics FwdDynEngine;
    typedef typename pronto::anymal::dyn::InverseDynamics InvDynEngine;
    typedef typename pronto::anymal::dyn::JSIM JSIM;

    static const int joints_count = pronto::anymal::jointsCount;
    static const int links_count  = pronto::anymal::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return pronto::anymal::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return pronto::anymal::orderedLinkIDs;
}

}
}

#endif
