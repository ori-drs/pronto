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

#include "declarations.h"
#include "leg_data_map.h"
#include "leg_bool_map.h"
#include "joint_bool_map.h"


namespace pronto {
namespace quadruped {

using FootPositions     = LegDataMap<rbd::Vector3d>;
using FootVelocities    = LegDataMap<rbd::Vector3d>;
using FootAccelerations = LegDataMap<rbd::Vector3d>;

using FootPosition      = rbd::Vector3d;
using FootVelocity      = rbd::Vector3d;
using FootAcceleration  = rbd::Vector3d;
using LegJointState     = rbd::Vector3d;

/**
 * An inverse kinematics interface for quadrupeds.
 * The class InverseKinematics computes the Inverse Kinematics for a generic
 * quadruped robots. It can compute:
 *   - joint positions
 *   - joint velocities
 *   - joint positions and velocities
 *   - joint positions, velocities and accelerations
 *
 * These quantities can be computed for one leg or for all legs at once.
 *
 * The class can compute these quantities taking in consideration the kinematic
 * limits and the knee configuration of the quadruped.
 */
class InverseKinematics {
public:
    virtual ~InverseKinematics() {}

    /**
     * @brief getJointPosition computes the position of the joints for one leg
     * given the foot position
     * @param[in] foot_position position of the foot, expressed in the base frame
     * @param[in] leg_id leg identifier (e.g., quadruped::LF)
     * @param[out] q three-dimensional vector expressing the joint positions of
     * the three joints of a leg
     * @param[in] clamp optional parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @return a boolean indicating if the position is correctly computed
     */
    virtual bool getJointPosition(const FootPosition& foot_position,
                                  const LegID& leg_id,
                                  LegJointState& q,
                                  bool clamp = false) const = 0;

    /**
     * @brief same as getJointPosition(const FootPosition& foot_position,
                                  const LegID& leg_id,
                                  LegJointState& q,
                                  bool clamp) but with an additional parameter
                                  to indicate if the joint limits are hit
     * @param[in] foot_position position of the foot, expressed in the base frame
     * @param[in] leg_id leg identifier (e.g., quadruped::LF)
     * @param[out] q three-dimensional vector expressing the joint positions of
     * the three joints of a leg
     * @param[in] clamp  parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @param[out] q_violation a three-dimentional vector of booleans indicating
     * which indicate whether a specific joint violated the
     * kinematic limits, regardless of the clamp option
     * @return a boolean indicating if the position is correctly computed
     * @sa getJointPosition(const FootPosition& foot_position,
                                  const LegID& leg_id,
                                  LegJointState& q,
                                  bool clamp)
     */
    virtual bool getJointPosition(const FootPosition& foot_position,
                                  const LegID& leg_id,
                                  LegJointState& q,
                                  bool clamp,
                                  LegJointBool& q_violation) const = 0;

    /**
     * @brief same as getJointPosition(const FootPosition& foot_position,
                                  const LegID& leg_id,
                                  LegJointState& q,
                                  bool clamp) but for all legs
     * @param[in] foot_positions positions of the feet, expressed in the base frame
     * @param[out] q a quadruped::JointState vector expressing the joint positions
     * @param[in] clamp optional parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @return a boolean indicating if the position is correctly computed
     * @sa getJointPosition(const FootPosition& foot_position,
                                  const LegID& leg_id,
                                  LegJointState& q,
                                  bool clamp)
     */
    virtual bool getJointPosition(const FootPositions& foot_positions,
                                  JointState& q,                                  
                                  bool clamp = false) const = 0;

    /**
     * @brief same as getJointPosition(const FootPosition& foot_position,
                                  const LegID& leg_id,
                                  LegJointState& q,
                                  bool clamp,
                                  LegJointBool& q_violation) but for all legs
     * @param[in] foot_positions positions of the feet, expressed in the base frame
     * @param[out] q a quadruped::JointState vector expressing the joint positions
     * @param[in] clamp parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @param[out] q_violation  a quadruped::JointState vector of booleans indicating
     * which indicate whether a specific joint violated the
     * kinematic limits, regardless of the clamp option
     * @return a boolean indicating if all the joint positions are correctly
     * computed
     * @sa getJointPosition(const FootPosition& foot_position,
                                  const LegID& leg_id,
                                  LegJointState& q,
                                  bool clamp,
                                  LegJointBool& q_violation)
     */
    virtual bool getJointPosition(const FootPositions& foot_positions,
                                  JointState& q,
                                  bool clamp,
                                  JointBoolMap & q_violation) const = 0;

    /**
     * @brief getJointVelocity computes the velocity of the joints for one leg
     * given the foot velocity for the same leg
     * @param[in] foot_velocity velocity of the foot with respect to the base
     * frame, espressed in the base frame
     * @param[in] leg_id leg identifier (e.g., quadruped::LF)
     * @param[in] q three-dimensional vector expressing the joint positions of
     * the three joints of a leg
     * @param[out] qd  three-dimensional vector expressing the joint velocities
     * of the three joints of a leg
     * @return true if the velocity is correctly computed, false otherwise
     */
    virtual bool getJointVelocity(const FootVelocity& foot_velocity,
                                  const LegID& leg_id,
                                  const LegJointState& q,
                                  LegJointState& qd) = 0;

    /**
     * @brief same as getJointVelocity(), but for all legs
     * @param[in] foot_velocities velocities of the feet with respect to the base
     * frame, espressed in the base frame
     * @param[in] q a quadruped::JointState vector expressing the joint positions
     * @param[out] qd a quadruped::JointState vector expressing the joint velocities
     * @return true if the velocities of all legs are correctly computed,
     * false otherwise
     * @sa getJointVelocityconst FootVelocity& foot_velocity,
                                  const LegID& leg_id,
                                  const LegJointState& q,
                                  LegJointState& qd)
     */
    virtual bool getJointVelocity(const FootVelocities& foot_velocities,
                                  const JointState& q,
                                  JointState& qd) = 0;

    /**
     * @brief getJointState jointly computes position and velocity of the
     * joints, given the foot position and velocity expressed in the base frame
     * @param[in] foot_position foot position expressed in the base frame
     * @param[in] foot_velocity velocity of the foot with respect to the base
     * frame, espressed in the base frame
     * @param[in] leg_id leg identifier (e.g., quadruped::LF)
     * @param[out] q three-dimensional vector expressing the joint positions of
     * the three joints of a leg
     * @param[out] qd three-dimensional vector expressing the joint velocities
     * of the three joints of a leg
     * @param[in] clamp optional parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @return true if both position and velocity are computed correctly,
     * false otherwise
     */
    virtual bool getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,                               
                               bool clamp = false) = 0;

    /**
     * @brief same as getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               bool clamp) but with an extra parameter which
                               stores whether some of the joints has been
                               saturated to the joint limits
     * @param[in] foot_position foot position expressed in the base frame
     * @param[in] foot_velocity velocity of the foot with respect to the base
     * @param[in] leg_id leg identifier (e.g., quadruped::LF)
     * @param[out] q three-dimensional vector expressing the joint positions of
     * the three joints of a leg
     * @param[out] qd three-dimensional vector expressing the joint velocities
     * of the three joints of a leg
     * @param[in] clamp parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @param[in] q_violation a three-dimensional data structure storing
     * three booleans which indicate whether a specific joint violated the
     * kinematic limits, regardless of the clamp option
     * @return true if all the positions and velocities are computed correctly
     * @sa getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               bool clamp)
     */
    virtual bool getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               bool clamp,
                               LegJointBool& q_violation) = 0;

    /**
     * @brief same as  getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               bool clamp) but computing the joint
                               accelerations also
     * @param[in] foot_position foot position expressed in the base frame
     * @param[in] foot_velocity velocity of the foot with respect to the base
     * frame, espressed in the base frame
     * @param[in] leg_id leg identifier (e.g., quadruped::LF)
     * @param[out] q three-dimensional vector expressing the joint positions of
     * the three joints of a leg
     * @param[out] qd three-dimensional vector expressing the joint velocities
     * of the three joints of a leg
     * @param[out] qdd three-dimensional vector expressing the joint accelerations
     * of the three joints of a leg
     * @param[in] clamp optional parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @return true if position, velocity and acceleration are computed
     * correctly for all the joints, false otherwise
     * @sa getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               bool clamp)
     */
    virtual bool getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const FootAcceleration& foot_acceleration,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               LegJointState& qdd,                               
                               bool clamp = false) = 0;


    /**
     * @brief same as
     * @param[in] foot_position foot position expressed in the base frame
     * @param[in] foot_velocity velocity of the foot with respect to the base
     * @param[in] leg_id leg identifier (e.g., quadruped::LF)
     * @param[out] q three-dimensional vector expressing the joint positions of
     * the three joints of a leg
     * @param[out] qd three-dimensional vector expressing the joint velocities
     * of the three joints of a leg
     * @param[out] qdd three-dimensional vector expressing the joint accelerations
     * of the three joints of a leg
     * @param[in] clamp parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @param[in] q_violation a three-dimensional data structure storing
     * three booleans which indicate whether a specific joint violated the
     * kinematic limits, regardless of the clamp option
     * correctly for all the joints, false otherwise
     * @sa getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               bool clamp)
     */
    virtual bool getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const FootAcceleration& foot_acceleration,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               LegJointState& qdd,
                               bool clamp,
                               LegJointBool& q_violation) = 0;

    /**
     * @brief same as getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               bool clamp), but for all legs
     * @param[in] foot_positions positions of the feet, expressed in the base frame
     * @param[in] foot_velocities velocities of the feet with respect to the base
     * frame, espressed in the base frame
     * @param[out] q a quadruped::JointState vector expressing the joint positions
     * @param[out] qd a quadruped::JointState vector expressing the joint velocities
     * @param[in] clamp parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @param[in] q_violation a 12-dimensional data structure storing
     * 12 booleans which indicate whether a specific joint violated the
     * kinematic limits, regardless of the clamp option
     * @return true if position, velocity and acceleration are computed
     * correctly for all the joints, false otherwise
     * @sa getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               bool clamp)
     */
    virtual bool getJointState(const FootPositions& foot_positions,
                               const FootVelocities& foot_velocities,
                               JointState& q,
                               JointState& qd,                               
                               bool clamp,
                               JointBoolMap& q_violation) = 0;

    /**
     * @brief same as getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const FootAcceleration& foot_acceleration,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               LegJointState& qdd,
                               bool clamp) but for all legs
     * @param[in] foot_positions positions of the feet, expressed in the base frame
     * @param[in] foot_velocities velocities of the feet with respect to the base
     * frame, espressed in the base frame
     * @param[in] foot_accelerations accelerations of the feet with respect to
     * the base frame, espressed in the base frame
     * @param[out] q a quadruped::JointState vector expressing the joint positions
     * @param[out] qd a quadruped::JointState vector expressing the joint velocities
     * @param[out] qdd a quadruped::JointState vector expressing the joint accelerations
     * @param[in] clamp optional parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @return true if position, velocity and acceleration are computed
     * correctly for all the joints, false otherwise
     * @sa getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const FootAcceleration& foot_acceleration,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               LegJointState& qdd,
                               bool clamp)
     */
    virtual bool getJointState(const FootPositions& foot_positions,
                               const FootVelocities& foot_velocities,
                               const FootAccelerations& foot_accelerations,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               bool clamp = false) = 0;


    /**
     * @brief same as getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const FootAcceleration& foot_acceleration,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               bool clamp) but for all legs
     * @param[in] foot_positions positions of the feet, expressed in the base frame
     * @param[in] foot_velocities velocities of the feet with respect to the base
     * frame, espressed in the base frame
     * @param[out] q a quadruped::JointState vector expressing the joint positions
     * @param[out] qd a quadruped::JointState vector expressing the joint velocities
     * @param clamp optional parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @return true if position, velocity and acceleration are computed
     * correctly for all the joints, false otherwise
     * @sa getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const FootAcceleration& foot_acceleration,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               bool clamp)
     */
    virtual bool getJointState(const FootPositions& foot_positions,
                               const FootVelocities& foot_velocities,
                               JointState& q,
                               JointState& qd,
                               bool clamp = false) = 0;

    /**
     * @brief same as getJointState(const FootPosition& foot_position,
                               const FootVelocity& foot_velocity,
                               const FootAcceleration& foot_acceleration,
                               const LegID& leg_id,
                               LegJointState& q,
                               LegJointState& qd,
                               LegJointState& qdd,
                               bool clamp,
                               LegJointBool& q_violation) but for all legs
     * @param[in] foot_positions positions of the feet, expressed in the base frame
     * @param[in] foot_velocities velocities of the feet with respect to the base
     * frame, espressed in the base frame
     * @param[in] foot_accelerations
     * @param[out] q a quadruped::JointState vector expressing the joint positions
     * @param[out] qd a quadruped::JointState vector expressing the joint velocities
     * @param[in] clamp parameter to indicate whether the values returned
     * for the joints should be saturated to the joint limits
     * @param[in] q_violation a 12-dimensional data structure storing
     * 12 booleans which indicate whether a specific joint violated the
     * kinematic limits, regardless of the clamp option
     * @return true if position, velocity and acceleration are computed
     * correctly for all the joints, false otherwise

     * @returntrue if position, velocity and acceleration are computed
     * correctly for all the joints, false otherwise
     */
    virtual bool getJointState(const FootPositions& foot_positions,
                               const FootVelocities& foot_velocities,
                               const FootAccelerations& foot_accelerations,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               bool clamp,
                               JointBoolMap& q_violation) = 0;

    /**
     * @brief method to get the kinematic limits from the user
     * @param[in] q_min lower kinematic limits
     * @param[in] q_max upper kinematic limits
     */
    virtual void setKinematicLimits(const JointState& q_min,
                                    const JointState& q_max) = 0;

    /**
     * @brief method to get the knee configuration from the user.
     * @param[in] is_knee_backward a data structure of four booleans, each one
     * being true if the knee is pointing backwars, false otherwise
     */
    virtual void setKneeConfiguration(const LegBoolMap& is_knee_backward) = 0;

    virtual void setTimePeriod(const double& dt) = 0;
};


}
}

