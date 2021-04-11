#ifndef RCG_ANYMAL_INVERSE_DYNAMICS_H_
#define RCG_ANYMAL_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace pronto {
namespace anymal {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot anymal.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */
class InverseDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot anymal, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(InertiaProperties& in, MotionTransforms& tr);

    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[out] baseAccel the spatial acceleration of the robot base
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_v the spatial velocity of the base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        JointState& jForces, Acceleration& base_a,
        const Acceleration& g, const Velocity& base_v,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces, Acceleration& base_a,
        const Acceleration& g, const Velocity& base_v,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] baseWrench the spatial force to be applied to the robot base to achieve
     *             the desired accelerations
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_v the spatial velocity of the base
     * \param[in] baseAccel the desired spatial acceleration of the robot base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const JointState& q);
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g);
    ///@}
    /** \name Centrifugal and Coriolis terms, fully actuated base
     *
     * These functions take only velocity inputs, that is, they assume
     * a zero spatial acceleration of the base (in addition to zero acceleration
     * at the actuated joints).
     * Note that this is NOT the same as imposing zero acceleration
     * at the virtual 6-dof-floting-base joint, which would result, in general,
     * in a non-zero spatial acceleration of the base, due to velocity
     * product terms.
     */
    ///@{
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_v, const JointState& q, const JointState& qd);
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_v, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Force& getForce_base() const { return base_f; }
    const Velocity& getVelocity_LF_HIP() const { return LF_HIP_v; }
    const Acceleration& getAcceleration_LF_HIP() const { return LF_HIP_a; }
    const Force& getForce_LF_HIP() const { return LF_HIP_f; }
    const Velocity& getVelocity_LF_THIGH() const { return LF_THIGH_v; }
    const Acceleration& getAcceleration_LF_THIGH() const { return LF_THIGH_a; }
    const Force& getForce_LF_THIGH() const { return LF_THIGH_f; }
    const Velocity& getVelocity_LF_SHANK() const { return LF_SHANK_v; }
    const Acceleration& getAcceleration_LF_SHANK() const { return LF_SHANK_a; }
    const Force& getForce_LF_SHANK() const { return LF_SHANK_f; }
    const Velocity& getVelocity_RF_HIP() const { return RF_HIP_v; }
    const Acceleration& getAcceleration_RF_HIP() const { return RF_HIP_a; }
    const Force& getForce_RF_HIP() const { return RF_HIP_f; }
    const Velocity& getVelocity_RF_THIGH() const { return RF_THIGH_v; }
    const Acceleration& getAcceleration_RF_THIGH() const { return RF_THIGH_a; }
    const Force& getForce_RF_THIGH() const { return RF_THIGH_f; }
    const Velocity& getVelocity_RF_SHANK() const { return RF_SHANK_v; }
    const Acceleration& getAcceleration_RF_SHANK() const { return RF_SHANK_a; }
    const Force& getForce_RF_SHANK() const { return RF_SHANK_f; }
    const Velocity& getVelocity_LH_HIP() const { return LH_HIP_v; }
    const Acceleration& getAcceleration_LH_HIP() const { return LH_HIP_a; }
    const Force& getForce_LH_HIP() const { return LH_HIP_f; }
    const Velocity& getVelocity_LH_THIGH() const { return LH_THIGH_v; }
    const Acceleration& getAcceleration_LH_THIGH() const { return LH_THIGH_a; }
    const Force& getForce_LH_THIGH() const { return LH_THIGH_f; }
    const Velocity& getVelocity_LH_SHANK() const { return LH_SHANK_v; }
    const Acceleration& getAcceleration_LH_SHANK() const { return LH_SHANK_a; }
    const Force& getForce_LH_SHANK() const { return LH_SHANK_f; }
    const Velocity& getVelocity_RH_HIP() const { return RH_HIP_v; }
    const Acceleration& getAcceleration_RH_HIP() const { return RH_HIP_a; }
    const Force& getForce_RH_HIP() const { return RH_HIP_f; }
    const Velocity& getVelocity_RH_THIGH() const { return RH_THIGH_v; }
    const Acceleration& getAcceleration_RH_THIGH() const { return RH_THIGH_a; }
    const Force& getForce_RH_THIGH() const { return RH_THIGH_f; }
    const Velocity& getVelocity_RH_SHANK() const { return RH_SHANK_v; }
    const Acceleration& getAcceleration_RH_SHANK() const { return RH_SHANK_a; }
    const Force& getForce_RH_SHANK() const { return RH_SHANK_f; }
    ///@}
protected:
    void secondPass_fullyActuated(JointState& jForces);

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* xm;
private:
    Matrix66 vcross; // support variable
    // Link 'LF_HIP' :
    const InertiaMatrix& LF_HIP_I;
    Velocity      LF_HIP_v;
    Acceleration  LF_HIP_a;
    Force         LF_HIP_f;
    // Link 'LF_THIGH' :
    const InertiaMatrix& LF_THIGH_I;
    Velocity      LF_THIGH_v;
    Acceleration  LF_THIGH_a;
    Force         LF_THIGH_f;
    // Link 'LF_SHANK' :
    const InertiaMatrix& LF_SHANK_I;
    Velocity      LF_SHANK_v;
    Acceleration  LF_SHANK_a;
    Force         LF_SHANK_f;
    // Link 'RF_HIP' :
    const InertiaMatrix& RF_HIP_I;
    Velocity      RF_HIP_v;
    Acceleration  RF_HIP_a;
    Force         RF_HIP_f;
    // Link 'RF_THIGH' :
    const InertiaMatrix& RF_THIGH_I;
    Velocity      RF_THIGH_v;
    Acceleration  RF_THIGH_a;
    Force         RF_THIGH_f;
    // Link 'RF_SHANK' :
    const InertiaMatrix& RF_SHANK_I;
    Velocity      RF_SHANK_v;
    Acceleration  RF_SHANK_a;
    Force         RF_SHANK_f;
    // Link 'LH_HIP' :
    const InertiaMatrix& LH_HIP_I;
    Velocity      LH_HIP_v;
    Acceleration  LH_HIP_a;
    Force         LH_HIP_f;
    // Link 'LH_THIGH' :
    const InertiaMatrix& LH_THIGH_I;
    Velocity      LH_THIGH_v;
    Acceleration  LH_THIGH_a;
    Force         LH_THIGH_f;
    // Link 'LH_SHANK' :
    const InertiaMatrix& LH_SHANK_I;
    Velocity      LH_SHANK_v;
    Acceleration  LH_SHANK_a;
    Force         LH_SHANK_f;
    // Link 'RH_HIP' :
    const InertiaMatrix& RH_HIP_I;
    Velocity      RH_HIP_v;
    Acceleration  RH_HIP_a;
    Force         RH_HIP_f;
    // Link 'RH_THIGH' :
    const InertiaMatrix& RH_THIGH_I;
    Velocity      RH_THIGH_v;
    Acceleration  RH_THIGH_a;
    Force         RH_THIGH_f;
    // Link 'RH_SHANK' :
    const InertiaMatrix& RH_SHANK_I;
    Velocity      RH_SHANK_v;
    Acceleration  RH_SHANK_a;
    Force         RH_SHANK_f;

    // The robot base
    const InertiaMatrix& base_I;
    InertiaMatrix base_Ic;
    Force         base_f;
    // The composite inertia tensors
    InertiaMatrix LF_HIP_Ic;
    InertiaMatrix LF_THIGH_Ic;
    const InertiaMatrix& LF_SHANK_Ic;
    InertiaMatrix RF_HIP_Ic;
    InertiaMatrix RF_THIGH_Ic;
    const InertiaMatrix& RF_SHANK_Ic;
    InertiaMatrix LH_HIP_Ic;
    InertiaMatrix LH_THIGH_Ic;
    const InertiaMatrix& LH_SHANK_Ic;
    InertiaMatrix RH_HIP_Ic;
    InertiaMatrix RH_THIGH_Ic;
    const InertiaMatrix& RH_SHANK_Ic;

private:
    static const ExtForces zeroExtForces;
};

inline void InverseDynamics::setJointStatus(const JointState& q) const
{
    (xm->fr_LF_HIP_X_fr_base)(q);
    (xm->fr_LF_THIGH_X_fr_LF_HIP)(q);
    (xm->fr_LF_SHANK_X_fr_LF_THIGH)(q);
    (xm->fr_RF_HIP_X_fr_base)(q);
    (xm->fr_RF_THIGH_X_fr_RF_HIP)(q);
    (xm->fr_RF_SHANK_X_fr_RF_THIGH)(q);
    (xm->fr_LH_HIP_X_fr_base)(q);
    (xm->fr_LH_THIGH_X_fr_LH_HIP)(q);
    (xm->fr_LH_SHANK_X_fr_LH_THIGH)(q);
    (xm->fr_RH_HIP_X_fr_base)(q);
    (xm->fr_RH_THIGH_X_fr_RH_HIP)(q);
    (xm->fr_RH_SHANK_X_fr_RH_THIGH)(q);
}

inline void InverseDynamics::id(
    JointState& jForces, Acceleration& base_a,
    const Acceleration& g, const Velocity& base_v,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, base_a, g, base_v,
       qd, qdd, fext);
}

inline void InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g, const JointState& q)
{
    setJointStatus(q);
    G_terms_fully_actuated(baseWrench, jForces, g);
}

inline void InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_v, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms_fully_actuated(baseWrench, jForces, base_v, qd);
}

inline void InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    setJointStatus(q);
    id_fully_actuated(baseWrench, jForces, g, base_v,
        baseAccel, qd, qdd, fext);
}

}
}
}

#endif
