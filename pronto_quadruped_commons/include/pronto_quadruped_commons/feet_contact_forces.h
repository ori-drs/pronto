/* Copyright (c) 2015-2021
 * Istituto Italiano di Tecnologia (IIT), University of Oxford
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

#include "declarations.h"
#include "leg_data_map.h"
#include "leg_vector_map.h"

#include <Eigen/Dense>

namespace pronto {
namespace quadruped {

/**
 * This class computes the ground reaction forces at the feet, according to the
 * actuated part of the Dynamics Equation of Motion:
 * \f[
 *  \begin{bmatrix}
  I_c & F \\
  F^T & M \\
   \end{bmatrix}
   \begin{bmatrix}
   {}_b{\ddot{\mathbf{x}}}{_b} \\
   {}_b{\dot{\boldsymbol{\omega}}}{_b} \\
   \ddot{\mathbf{q}} \\
   \end{bmatrix} +
   \begin{bmatrix} \mathbf{h}_b \\ \mathbf{h}_q \\ \end{bmatrix} =
   \begin{bmatrix} J_{cb}^T \\- J_{cq}^T \cr \end{bmatrix} \mathbf{f} +
   \begin{bmatrix} \mathbf{0} \\ \boldsymbol{\tau} \\ \end{bmatrix}
 * \f]
 */
class FeetContactForces {
public:
    typedef typename Eigen::Quaterniond Quaterniond;
public:
    FeetContactForces() {}
    virtual ~FeetContactForces() {}

    /**
     * @brief Same as getFootGRF() but with returns Vector3d object. Overloaded by default.
     *
     * @param[in] q Joint position, (measured by encoders)
     * @param[in] qd Joint velocity (computed from encoder positions)
     * @param[in] tau Joint torque (computed/measured by force/torque sensors)
     * @param[in] orient Base orientation, expressed in world frame as Quaternion
     * @param[in] leg The leg in which we wish to compute the GRF
     * @param[in] qdd Joint acceleration (computed from encoder position)
     * @param[in] xd Base linear velocity (estimated)
     * @param[in] xdd Base absolute acceleration (computed from IMU).
     * Note that this is NOT the proper acceleration (i.e., the IMU values)!
     * @param[in] omega Base angular velocity (measured from IMU)
     * @param[in] omegad Base angular acceleration (derived from IMU gyro)
     * @return A Vector3d expressing the GRF at the foot, expressed in the base
     * frame
     */
    virtual Vector3d getFootGRF(const JointState& q,
                                const JointState& qd,
                                const JointState& tau,
                                const Quaterniond& orient,
                                const LegID& leg,
                                const JointState& qdd = JointState::Zero(),
                                const Vector3d& xd = Vector3d::Zero(),
                                const Vector3d& xdd = Vector3d::Zero(),
                                const Vector3d& omega = Vector3d::Zero(),
                                const Vector3d& omegad = Vector3d::Zero()) {
        Vector3d res;
        (void)getFootGRF(q, qd, tau, orient, leg, res, qdd, xd, xdd, omega, omegad);
        return res;
    }

    /**
     * @brief Same as getFootGRF() but returns GRF for all legs
     * 
     * @param[in] q Joint position, (measured by encoders)
     * @param[in] qd Joint velocity (computed from encoder positions)
     * @param[in] tau Joint torque (computed/measured by force/torque sensors)
     * @param[in] orient Base orientation, expressed in world frame as Quaternion
     * @param[out] feet_grf data structure which associates the force at the
     * end effector for each leg
     * @param[in] qdd Joint acceleration (computed from encoder position)
     * @param[in] xd Base linear velocity (estimated)
     * @param[in] xdd Base absolute acceleration (computed from IMU).
     * Note that this is NOT the proper acceleration (i.e., the IMU values)!
     * @param[in] omega Base angular velocity (measured from IMU)
     * @param[in] omegad Base angular acceleration (derived from IMU gyro)
     * @return a boolean indicating whether the computation was successful or
     * not
     */
    virtual bool getFeetGRF(const JointState& q,
                            const JointState& qd,
                            const JointState& tau,
                            const Quaterniond& orient,
                            LegVectorMap& feet_grf,
                            const JointState& qdd = JointState::Zero(),
                            const Vector3d& xd = Vector3d::Zero(),
                            const Vector3d& xdd = Vector3d::Zero(),
                            const Vector3d& omega = Vector3d::Zero(),
                            const Vector3d& omegad = Vector3d::Zero()) {
        bool res_lf = getFootGRF(q, qd, tau, orient, LegID::LF, feet_grf[LegID::LF], qdd, xd, xdd, omega, omegad);
        bool res_rf = getFootGRF(q, qd, tau, orient, LegID::RF, feet_grf[LegID::RF], qdd, xd, xdd, omega, omegad);
        bool res_lh = getFootGRF(q, qd, tau, orient, LegID::LH, feet_grf[LegID::LH], qdd, xd, xdd, omega, omegad);
        bool res_rh = getFootGRF(q, qd, tau, orient, LegID::RH, feet_grf[LegID::RH], qdd, xd, xdd, omega, omegad);

        return (res_lf && res_rf && res_lh && res_rh);
    }

    /**
     * @brief Same as getFootGRF() but returns GRF for all legs as a LegVectorMap
     * 
     * @param[in] q Joint position, (measured by encoders)
     * @param[in] qd Joint velocity (computed from encoder positions)
     * @param[in] tau Joint torque (computed/measured by force/torque sensors)
     * @param[in] orient Base orientation, expressed in world frame as Quaternion
     * @param[out] feet_grf data structure which associates the force at the
     * end effector for each leg
     * @param[in] qdd Joint acceleration (computed from encoder position)
     * @param[in] xd Base linear velocity (estimated)
     * @param[in] xdd Base absolute acceleration (computed from IMU).
     * Note that this is NOT the proper acceleration (i.e., the IMU values)!
     * @param[in] omega Base angular velocity (measured from IMU)
     * @param[in] omegad Base angular acceleration (derived from IMU gyro)
     * @return a data structure which associates the force at the
     * end effector for each leg
     */
    virtual LegVectorMap getFeetGRF(const JointState& q,
                                    const JointState& qd,
                                    const JointState& tau,
                                    const Quaterniond& orient,
                                    const JointState& qdd = JointState::Zero(),
                                    const Vector3d& xd = Vector3d::Zero(),
                                    const Vector3d& xdd = Vector3d::Zero(),
                                    const Vector3d& omega = Vector3d::Zero(),
                                    const Vector3d& omegad = Vector3d::Zero()) {
        LegVectorMap res;
        (void)getFeetGRF(q, qd, tau, orient, res, qdd, xd, xdd, omega, omegad);
        return res;
    }

    /**
     * @brief getFootGRF computes the Ground Reaction Forces at the foot of a
     * specific leg, according to the dynamic equation:
     * \f[
     * \mathbf{f} =\Big(J_{c,q}\Big)^{\dagger} \Big(-{\tau} +
     * h_q(\mathbf{q}, \dot{\mathbf{q}}) + F^T(\mathbf{q})
     * [\ddot{\mathbf{x}}\;\; \dot{{\omega}}]^T +
     *  M(\mathbf{q})\ddot{\mathbf{q}}\Big)
     * \f]
     * 
     * @param[in] q Joint position, (measured by encoders)
     * @param[in] qd Joint velocity (computed from encoder positions)
     * @param[in] tau Joint torque (computed/measured by force/torque sensors)
     * @param[in] orient Base orientation, expressed in world frame as Quaternion
     * @param[in] leg The leg in which we wish to compute the GRF
     * @param[out] foot_grf
     * @param[in] qdd Joint acceleration (computed from encoder position)
     * @param[in] xd Base linear velocity (estimated)
     * @param[in] xdd Base absolute acceleration (computed from IMU).
     * Note that this is NOT the proper acceleration (i.e., the IMU values)!
     * @param[in] omega Base angular velocity (measured from IMU)
     * @param[in] omegad Base angular acceleration (derived from IMU gyro)
     * @return a boolean indicating whether the computation was successful or
     * not
     */
    virtual bool getFootGRF(const JointState& q,
                            const JointState& qd,
                            const JointState& tau,
                            const Quaterniond& orient,
                            const LegID& leg,
                            Vector3d& foot_grf,
                            const JointState& qdd = JointState::Zero(),
                            const Vector3d& xd = Vector3d::Zero(),
                            const Vector3d& xdd = Vector3d::Zero(),
                            const Vector3d& omega = Vector3d::Zero(),
                            const Vector3d& omegad = Vector3d::Zero()) = 0;
};
}  // namespace quadruped
}  // namespace pronto
