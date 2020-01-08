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

#include <pronto_quadruped_commons/rbd/rbd.h>

#include "declarations.h"
#include "leg_data_map.h"


namespace pronto {
namespace quadruped {

class InertiaPropertiesBase {
public:
    virtual ~InertiaPropertiesBase() {}

    virtual const iit::rbd::Vector3d& getTrunkCOM() = 0;
    virtual double getTrunkMass()  = 0;

    virtual double getHipAssemblyMass(const LegID& leg) const = 0;
    virtual double getUpperLegMass(const LegID& leg) const = 0;
    virtual double getLowerLegMass(const LegID& leg) const = 0;

    virtual Vector3d getHipAssemblyCOM(const LegID& leg) const = 0;
    virtual Vector3d getUpperLegCOM(const LegID& leg) const = 0;
    virtual Vector3d getLowerLegCOM(const LegID& leg) const = 0;

    virtual double getTotalMass() const = 0;
    /**
     * @brief getWholeBodyCOM  computes the Center Of Mass (COM) position of
     * the whole robot, in  base coordinates.
     * @param q the joint status vector describing the configuration of the robot
     * @return the position of the Center Of Mass of the whole robot, expressed
               in base coordinates
     */
    virtual Vector3d getWholeBodyCOM(const JointState& q) = 0;

};
}
}

