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

namespace pronto {
namespace quadruped {

/**
 * An interface for the Jacobian of the feet of quadrupeds.
 */
class FeetJacobians
{
public:
    virtual ~FeetJacobians() {};

    /**
     * \name Foot Jacobian getters
     * These functions shall return the 3x3 Jacobian that multiplied by the
     * velocity of the leg joints yields the linear velocity of the foot,
     * expressed in the base reference frame.
     *
     * There are five getters, one for each leg (LF, RF, LH, RH) and one that
     * takes the identifier of the leg of interest.
     */
    ///@{
    virtual FootJac getFootJacobianLF(const JointState& q) = 0;
    virtual FootJac getFootJacobianRF(const JointState& q) = 0;
    virtual FootJac getFootJacobianLH(const JointState& q) = 0;
    virtual FootJac getFootJacobianRH(const JointState& q) = 0;
    virtual FootJac getFootJacobian(const JointState& q, const LegID& leg) = 0;
    /**
     * @brief overload to get the Jacobian on a point of the foot sphere
     * @param q
     * @param leg
     * @param foot_x
     * @param foot_y
     * @return
     */
    virtual FootJac getFootJacobian(const pronto::quadruped::JointState& q,
                                    const pronto::quadruped::LegID& leg,
                                    const double& foot_x,
                                    const double& foot_y) = 0;
    ///@}
};


}
}

