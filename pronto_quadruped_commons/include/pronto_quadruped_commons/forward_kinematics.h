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
#include "leg_vector_map.h"

namespace pronto {
namespace quadruped {

/**
 * A forward kinematics interface for quadrupeds.
 */
class ForwardKinematics
{
public:
    ForwardKinematics() = default;
    virtual ~ForwardKinematics() = default;

    virtual Vector3d getFootPos(const JointState& q, const LegID& leg) = 0;
    inline virtual LegVectorMap getFeetPos(const JointState& q)
    {
        LegVectorMap feetPos(Vector3d::Zero());
        feetPos[quadruped::LF] = getFootPos(q, LegID::LF);
        feetPos[quadruped::RF] = getFootPos(q, LegID::RF);
        feetPos[quadruped::LH] = getFootPos(q, LegID::LH);
        feetPos[quadruped::RH] = getFootPos(q, LegID::RH);
        return feetPos;
    }
    virtual Matrix3d getFootOrientation(const JointState& q, const LegID& leg) = 0;
};

}  // namespace quadruped
}  // namespace pronto
