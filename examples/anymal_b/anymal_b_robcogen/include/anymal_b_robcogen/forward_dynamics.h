#ifndef RCG_ANYMAL_FORWARD_DYNAMICS_H_
#define RCG_ANYMAL_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace pronto {
namespace anymal {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot anymal.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot anymal, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param base_a
     * \param base_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& base_a, // output parameters,
       const Velocity& base_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& base_a, // output parameters,
        const Velocity& base_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    Matrix66 vcross; // support variable
    Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'base'
    Matrix66 base_AI;
    Force base_p;

    // Link 'LF_HIP' :
    Matrix66 LF_HIP_AI;
    Velocity LF_HIP_a;
    Velocity LF_HIP_v;
    Velocity LF_HIP_c;
    Force    LF_HIP_p;

    Column6 LF_HIP_U;
    Scalar LF_HIP_D;
    Scalar LF_HIP_u;
    // Link 'LF_THIGH' :
    Matrix66 LF_THIGH_AI;
    Velocity LF_THIGH_a;
    Velocity LF_THIGH_v;
    Velocity LF_THIGH_c;
    Force    LF_THIGH_p;

    Column6 LF_THIGH_U;
    Scalar LF_THIGH_D;
    Scalar LF_THIGH_u;
    // Link 'LF_SHANK' :
    Matrix66 LF_SHANK_AI;
    Velocity LF_SHANK_a;
    Velocity LF_SHANK_v;
    Velocity LF_SHANK_c;
    Force    LF_SHANK_p;

    Column6 LF_SHANK_U;
    Scalar LF_SHANK_D;
    Scalar LF_SHANK_u;
    // Link 'RF_HIP' :
    Matrix66 RF_HIP_AI;
    Velocity RF_HIP_a;
    Velocity RF_HIP_v;
    Velocity RF_HIP_c;
    Force    RF_HIP_p;

    Column6 RF_HIP_U;
    Scalar RF_HIP_D;
    Scalar RF_HIP_u;
    // Link 'RF_THIGH' :
    Matrix66 RF_THIGH_AI;
    Velocity RF_THIGH_a;
    Velocity RF_THIGH_v;
    Velocity RF_THIGH_c;
    Force    RF_THIGH_p;

    Column6 RF_THIGH_U;
    Scalar RF_THIGH_D;
    Scalar RF_THIGH_u;
    // Link 'RF_SHANK' :
    Matrix66 RF_SHANK_AI;
    Velocity RF_SHANK_a;
    Velocity RF_SHANK_v;
    Velocity RF_SHANK_c;
    Force    RF_SHANK_p;

    Column6 RF_SHANK_U;
    Scalar RF_SHANK_D;
    Scalar RF_SHANK_u;
    // Link 'LH_HIP' :
    Matrix66 LH_HIP_AI;
    Velocity LH_HIP_a;
    Velocity LH_HIP_v;
    Velocity LH_HIP_c;
    Force    LH_HIP_p;

    Column6 LH_HIP_U;
    Scalar LH_HIP_D;
    Scalar LH_HIP_u;
    // Link 'LH_THIGH' :
    Matrix66 LH_THIGH_AI;
    Velocity LH_THIGH_a;
    Velocity LH_THIGH_v;
    Velocity LH_THIGH_c;
    Force    LH_THIGH_p;

    Column6 LH_THIGH_U;
    Scalar LH_THIGH_D;
    Scalar LH_THIGH_u;
    // Link 'LH_SHANK' :
    Matrix66 LH_SHANK_AI;
    Velocity LH_SHANK_a;
    Velocity LH_SHANK_v;
    Velocity LH_SHANK_c;
    Force    LH_SHANK_p;

    Column6 LH_SHANK_U;
    Scalar LH_SHANK_D;
    Scalar LH_SHANK_u;
    // Link 'RH_HIP' :
    Matrix66 RH_HIP_AI;
    Velocity RH_HIP_a;
    Velocity RH_HIP_v;
    Velocity RH_HIP_c;
    Force    RH_HIP_p;

    Column6 RH_HIP_U;
    Scalar RH_HIP_D;
    Scalar RH_HIP_u;
    // Link 'RH_THIGH' :
    Matrix66 RH_THIGH_AI;
    Velocity RH_THIGH_a;
    Velocity RH_THIGH_v;
    Velocity RH_THIGH_c;
    Force    RH_THIGH_p;

    Column6 RH_THIGH_U;
    Scalar RH_THIGH_D;
    Scalar RH_THIGH_u;
    // Link 'RH_SHANK' :
    Matrix66 RH_SHANK_AI;
    Velocity RH_SHANK_a;
    Velocity RH_SHANK_v;
    Velocity RH_SHANK_c;
    Force    RH_SHANK_p;

    Column6 RH_SHANK_U;
    Scalar RH_SHANK_D;
    Scalar RH_SHANK_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_LF_HIP_X_fr_base)(q);
    (motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP)(q);
    (motionTransforms-> fr_LF_SHANK_X_fr_LF_THIGH)(q);
    (motionTransforms-> fr_RF_HIP_X_fr_base)(q);
    (motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP)(q);
    (motionTransforms-> fr_RF_SHANK_X_fr_RF_THIGH)(q);
    (motionTransforms-> fr_LH_HIP_X_fr_base)(q);
    (motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP)(q);
    (motionTransforms-> fr_LH_SHANK_X_fr_LH_THIGH)(q);
    (motionTransforms-> fr_RH_HIP_X_fr_base)(q);
    (motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP)(q);
    (motionTransforms-> fr_RH_SHANK_X_fr_RH_THIGH)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd, Acceleration& base_a, // output parameters,
    const Velocity& base_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, base_a, base_v, g, qd, tau, fext);
}

}
}
}

#endif
