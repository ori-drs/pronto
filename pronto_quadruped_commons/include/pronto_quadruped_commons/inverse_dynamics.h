/*
 * Copyright (c) 2015-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * Authors: Marco Frigerio, Michele Focchi, Marco Camurri
 *
 * This file is part of pronto_quadruped_commons, a library for
 * algebra, kinematics and dynamics for quadruped robots.
 * This library is a fork of iit_commons.
 * For more information see:
 * https://github.com/iit-DLSLab/iit_commons
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */



#pragma once

#include <Eigen/Dense>
#include <pronto_quadruped_commons/rbd/rbd.h>

#include "declarations.h"
#include "link_data_map.h"

namespace pronto {
namespace quadruped {

/**
 * The Inverse Dynamics routine for quadruped robots
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the accelerations \c qdd. The \c jForces parameter
 * will be filled with the computed values.
 */
class InverseDynamicsBase {
public:
    typedef iit::rbd::ForceVector Force;
    typedef iit::rbd::VelocityVector Velocity;
    typedef iit::rbd::VelocityVector Acceleration;
    typedef LinkDataMap<iit::rbd::ForceVector> ExtForces;

public:

    ///@}

    /** \name Inverse dynamics
     * The full algorithm for inverse dynamics for this robot
     */ ///@{
    virtual void id(JointState& jForces,
                    Acceleration& trunk_a,
                    const Acceleration& g,
                    const Velocity& trunk_v,
                    const JointState& q,
                    const JointState& qd,
                    const JointState& qdd,
                    const ExtForces& fext = LinkDataMap<iit::rbd::ForceVector>(Force::Zero()) ) = 0;

    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base
     */ ///@{
    virtual void id_fully_actuated(Force& baseWrench,
                                   JointState& jForces,
                                   const Acceleration& g,
                                   const Velocity& trunk_v,
                                   const Acceleration& baseAccel,
                                   const JointState& q,
                                   const JointState& qd,
                                   const JointState& qdd,
                                   const ExtForces& fext = LinkDataMap<iit::rbd::ForceVector>(Force::Zero())) = 0;

    ///@}
    /** \name Gravity terms, fully actuated base
     */
    ///@{
    virtual void G_terms_fully_actuated(Force& baseWrench,
                                        JointState& jForces,
                                        const Acceleration& g,
                                        const JointState& q) = 0;

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
    virtual void C_terms_fully_actuated(Force& baseWrench,
                                        JointState& jForces,
                                        const Velocity& trunk_v,
                                        const JointState& q,
                                        const JointState& qd) = 0;

    virtual Velocity getLinkVelocity(const JointState& q,
                                     const JointState& qd,
                                     const JointState& qdd,
                                     const Acceleration& gravity_acceleration,
                                     const Velocity& trunk_velocity,
                                     const LinkIdentifiers& link_id) = 0;


    virtual Acceleration getLinkAcceleration(const JointState& q,
                                             const JointState& qd,
                                             const JointState& qdd,
                                             const Acceleration& gravity_acceleration,
                                             const Velocity& trunk_velocity,
                                             const LinkIdentifiers& link_id) = 0;

protected:
    /** \name propagate link velocity and accelerations
     * The first pass forward propagation of velocity/accelerations, sets internal variables
     */ ///@{
    virtual void propagateVelAcc(
        const Acceleration& g, const Velocity& trunk_v,
        const JointState& qd, const JointState& qdd) = 0;
    ///@}

    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    virtual void setJointStatus(const JointState& q) const = 0;
};
}
}

