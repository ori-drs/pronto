/* Copyright (c) 2015-2019
 * Istituto Italiano di Tecnologia (IIT), University of Oxford
 * All rights reserved.
 *
 * Author: Marco Camurri (mcamurri@robots.ox.ac.uk)
 *
 * This file is part of pronto_quadruped,
 * a library for leg odometry on quadruped robots.
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
#include <pronto_quadruped_commons/declarations.h>
#include <pronto_quadruped_commons/leg_data_map.h>
#include <pronto_quadruped_commons/leg_bool_map.h>
#include <pronto_quadruped_commons/rbd/utils.h>

namespace pronto {
/**
 * @brief The StanceEstimatorBase class computes the stance state of a
 * legged robot with point feet. The stance state is described by a LegBoolMap
 * data structure which contains a boolean per leg. In addition the class
 * can compute the probability of stance. These values are computed from
 * different inputs, including: time, joint states, torques, base orientation,
 * base velocity, base acceleration.
 */
class StanceEstimatorBase {
public:
    typedef typename pronto::quadruped::JointState JointState;
    typedef typename Eigen::Vector3d Vector3d;
    typedef typename Eigen::Matrix3d Matrix3d;
    typedef typename Eigen::Quaterniond Quaterniond;

    template <class T>
    using LegDataMap = pronto::quadruped::LegDataMap<T>;
    using LegBoolMap = pronto::quadruped::LegBoolMap;
    using LegScalarMap = LegDataMap<double>;
    using LegVectorMap = LegDataMap<Vector3d>;


public:
    /**
     * @brief getStance computes the stance state from several inputs
     * @param[in] time an absolute time, measured in seconds
     * @param[in] q joint position
     * @param[in] qd joint velocity
     * @param[in] tau joint torque
     * @param[in] orient absolute base orientation
     * @param[in] qdd joint acceleration (optional)
     * @param[in] xd base linear velocity (optionl)
     * @param[in] xdd base absolute linear acceleration (optional)
     * @param[in] omega base angular velocity (optional)
     * @param[in] omegad base angular acceleration (optional)
     * @return a data structure which associates a boolean to each leg,
     * indicating whether the foot is in stance (true) or in swing (false).
     */
    virtual LegBoolMap getStance(const double time,
                                 const JointState &q,
                                 const JointState &qd,
                                 const JointState &tau,
                                 const Quaterniond & orient,
                                 const JointState &qdd = JointState::Constant(0),
                                 const Vector3d& xd = Vector3d(0, 0, 0),
                                 const Vector3d & xdd  = Vector3d(0, 0, 0),
                                 const Vector3d & omega  = Vector3d(0, 0, 0),
                                 const Vector3d & omegad = Vector3d(0, 0, 0)) = 0;

    /**
     * @brief same as getStance(), but with return as parameter
     * @param[in] time an absolute time, measured in seconds
     * @param[in] q joint position
     * @param[in] qd joint velocity
     * @param[in] tau joint torque
     * @param[in] orient absolute base orientation
     * @param[out] stance a data structure which associates a boolean to each leg,
     * indicating whether the foot is in stance (true) or in swing (false).
     * @param[in] qdd joint acceleration (optional)
     * @param[in] xd base linear velocity (optionl)
     * @param[in] xdd base absolute linear acceleration (optional)
     * @param[in] omega base angular velocity (optional)
     * @param[in] omegad base angular acceleration (optional)
     * @return true if the stance was computed with no errors, false otherwise
     */
    virtual bool getStance(const double time,
                           const JointState &q,
                           const JointState &qd,
                           const JointState &tau,
                           const Quaterniond & orient,
                           LegBoolMap& stance,
                           const JointState &qdd = JointState::Constant(0),
                           const Vector3d& xd = Vector3d(0, 0, 0),
                           const Vector3d & xdd  = Vector3d(0, 0, 0),
                           const Vector3d & omega  = Vector3d(0, 0, 0),
                           const Vector3d & omegad = Vector3d(0, 0, 0)) = 0;
    /**
     * @brief same as getStance(), but with stance status and stance probability
     * as parameters
     * @param[in] time an absolute time, measured in seconds
     * @param[in] q joint position
     * @param[in] qd joint velocity
     * @param[in] tau joint torque
     * @param[in] orient absolute base orientation
     * @param[out] stance a data structure which associates a boolean to each leg,
     * indicating whether the foot is in stance (true) or in swing (false).
     * @param[out] stance_probability a data structure which associates a
     * number between 0 and 1 indicating the probability for a specific foot to
     * be in stance or not
     * @param[in] qdd joint acceleration (optional)
     * @param[in] xd base linear velocity (optionl)
     * @param[in] xdd base absolute linear acceleration (optional)
     * @param[in] omega base angular velocity (optional)
     * @param[in] omegad base angular acceleration (optional)
     * @return true if the stance was computed with no errors, false otherwise
     */
    virtual bool getStance(const double time,
                           const JointState &q,
                           const JointState &qd,
                           const JointState &tau,
                           const Quaterniond & orient,
                           LegBoolMap& stance,
                           LegScalarMap& stance_probability,
                           const JointState &qdd = JointState::Constant(0),
                           const Vector3d& xd = Vector3d(0, 0, 0),
                           const Vector3d & xdd  = Vector3d(0, 0, 0),
                           const Vector3d & omega  = Vector3d(0, 0, 0),
                           const Vector3d & omegad = Vector3d(0, 0, 0)) = 0;

    virtual LegVectorMap getGRF() = 0;
};
} // namespace pronto
