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
#include <pronto_quadruped_commons/leg_data_map.h>
#include <pronto_quadruped_commons/leg_bool_map.h>
#include <pronto_quadruped_commons/declarations.h>

namespace pronto {

/**
 * @brief The LegOdometerBase class is the base class (interface) to implement
 * leg odometry. It provides methods to estimate the pose and the velocity of
 * a legged robot given the encoder values and knowledge about the stance
 * status of the robot.
 */
class LegOdometerBase {
public:
    // iit-commons aliases
    typedef typename quadruped::JointState JointState;
    template <class T>
    using LegDataMap = quadruped::LegDataMap<T>;
    typedef LegDataMap<double> LegScalarMap;
    typedef typename pronto::quadruped::LegBoolMap LegBoolMap;
    // Eigen aliases
    typedef Eigen::Vector3d Vector3d;
    typedef Eigen::Matrix3d Matrix3d;
    typedef Eigen::Quaterniond Quaterniond;
    typedef LegDataMap<Vector3d> LegVectorMap;

public:

    /**
     * @brief estimates the pose of the robot from time, joint states, and
     * contact state of the feet
     * @param[in] utime an absolute time (epoch) in microseconds
     * @param[in] q joint positions
     * @param[in] stance_legs a data structure indicating if a leg is on the
     * ground or not
     * @param[in] stance_prob a data structure indicating how likely a leg is
     * on the ground (from 0 to 1)
     * @param[out] position robot absolute position, expressed in an absolute
     * in a absolute reference frame
     * @param[out] pos_covariance covariance associated to the position
     * @param[out] orientation quaternion describing the absolute attitude of
     * the robot, expressed in world frame
     * @param[out] orient_covariance the covariance associated to this
     * measurement. The covariance is expressed using delta angles.
     * @return true if the returned pose is valid, false otherwise. If the
     * pose is invalid, the output parameters are left unchanged.
     */
    virtual bool estimatePose(const uint64_t utime,
                              const JointState& q,
                              const LegBoolMap& stance_legs,
                              const LegScalarMap& stance_prob,
                              Vector3d& position,
                              Matrix3d& pos_covariance,
                              Quaterniond& orientation,
                              Matrix3d& orient_covariance) = 0;

    /**
     * @brief estimates the linear velocity of the robot from time,
     * joint states, and contact state of the feet, according to the relation:
     * \f[
     * {}_b{\dot{\mathbf{x}}}{_{b_l}} = - {}_b{\dot{\mathbf{x}}}{_{f_l}}
     * - {}_b{\boldsymbol{\omega}}{_b} \times {}_b{\mathbf{x}}{_{f_l}}
     * \f]
     * where \f${}_b{\dot{\mathbf{x}}}{_{b_l}}\f$ is the velocity of the base
     * espressed in the base frame, computed from the velocity of a leg
     * \f${}_b{\dot{\mathbf{x}}}{_{f_l}}\f$ in contact with the ground, the
     * angular velocity of the base \f${}_b{\boldsymbol{\omega}}{_b}\f$, and
     * the foot position (expressed in base frame)
     * \f${}_b{\mathbf{x}}{_{f_l}}\f$.
     * @param[in] utime an absolute time (epoch) in microseconds
     * @param[in] q joint positions
     * @param[in] qd joint velocity
     * @param[in] omega angular velocity
     * @param[in] stance_legs a data structure indicating if a leg is on the
     * ground or not
     * @param[in] stance_prob a data structure indicating how likely a leg is
     * on the ground (from 0 to 1)
     * @param[out] velocity
     * @param[out] covariance
     * @return true if the returned velocity is valid, false otherwise. If the
     * velocity is invalid, the output parameters are left unchanged.
     */
    virtual bool estimateVelocity(const uint64_t utime,
                                  const JointState& q,
                                  const JointState& qd,
                                  const Vector3d& omega,
                                  const LegBoolMap& stance_legs,
                                  const LegScalarMap& stance_prob,
                                  Vector3d& velocity,
                                  Matrix3d& covariance) = 0;

    // virtual bool estimateAngularVelocity(const uint64_t utime,
    //                                      const JointState& q,
    //                                      const JointState& qd,
    //                                      const LegBoolMap& stance_legs,
    //                                      const LegScalarMap& stance_prob,
    //                                      Vector3d& omega,
    //                                      Matrix3d& covariance) = 0;

    /**
     * @brief returns the latest estimated velocity and covariance
     * @param[out] velocity
     * @param[out] covariance
     */
    virtual void getVelocity(Vector3d& velocity, Matrix3d& covariance) = 0;
    /**
     * @brief returns the latest estimated position and covariance
     * @param[out] position
     * @param[out] covariance
     */
    virtual void getPosition(Vector3d& position, Matrix3d& covariance) = 0;
    /**
     * @brief returns the latest estimated orientation and covariance
     * @param[out] orientation
     * @param[out] covariance
     */
    virtual void getOrientation(Quaterniond& orientation,
                                Matrix3d& covariance) = 0;

    virtual void setInitVelocityCov(const Matrix3d& vel_cov) = 0;
    virtual void setInitVelocityStd(const Vector3d& vel_std) = 0;
    virtual void setInitPositionCov(const Matrix3d& pos_cov) = 0;

    // virtual void getAngularVelocity(Vector3d& omega, Matrix3d& covariance) = 0;

    virtual void getVelocitiesFromLegs(LegVectorMap & vd) = 0;
    virtual void getFeetPositions(LegVectorMap & jd) = 0;
};

} // namespace pronto
