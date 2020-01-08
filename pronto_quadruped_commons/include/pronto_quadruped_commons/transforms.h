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

#include <pronto_quadruped_commons/rbd/TransformsBase.h>
#include <pronto_quadruped_commons/declarations.h>
#include <pronto_quadruped_commons/rbd/rbd.h>

namespace pronto {

namespace quadruped {

typedef typename iit::rbd::PlainMatrix<double, 6, 6> SpatialTransformPlain ;
typedef typename iit::rbd::PlainMatrix<double, 4, 4> HomogeneousTransformPlain ;

enum class DestFrame {TRUNK = 0, LF_LOWERLEG, RF_LOWERLEG, LH_LOWERLEG, RH_LOWERLEG,
                      LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT,
                      LF_HIPASSEMBLY, RF_HIPASSEMBLY, LH_HIPASSEMBLY, RH_HIPASSEMBLY,
                      LF_UPPERLEG, RF_UPPERLEG, LH_UPPERLEG, RH_UPPERLEG};

enum class OriginFrame {TRUNK = 0,
                        LF_HAA, LF_HFE, LF_KFE,
                        RF_HAA, RF_HFE, RF_KFE,
                        LH_HAA, LH_HFE, LH_KFE,
                        RH_HAA, RH_HFE, RH_KFE,
                        LF_LOWERLEG, RF_LOWERLEG, LH_LOWERLEG, RH_LOWERLEG,
                        LF_HIPASSEMBLY_COM, RF_HIPASSEMBLY_COM, LH_HIPASSEMBLY_COM, RH_HIPASSEMBLY_COM,
                        LF_HIPASSEMBLY, RF_HIPASSEMBLY, LH_HIPASSEMBLY, RH_HIPASSEMBLY,
                        LF_UPPERLEG_COM,RF_UPPERLEG_COM,LH_UPPERLEG_COM,RH_UPPERLEG_COM,
                        LF_UPPERLEG,RF_UPPERLEG,LH_UPPERLEG,RH_UPPERLEG,
                        LF_LOWERLEG_COM,RF_LOWERLEG_COM,LH_LOWERLEG_COM,RH_LOWERLEG_COM,
                        LF_SHIN,RF_SHIN,LH_SHIN,RH_SHIN,
                        LF_FOOT,RF_FOOT,LH_FOOT,RH_FOOT};

class HomogeneousTransformsBase {
public:
    virtual ~HomogeneousTransformsBase() {}

    virtual HomogeneousTransformPlain getTransform(const JointState& q,
                                                        const OriginFrame& orig,
                                                        const DestFrame& dest) = 0;
};

class MotionTransformsBase {
public:
    virtual ~MotionTransformsBase() {}

    virtual SpatialTransformPlain getTransform(const JointState& q,
                                                        const OriginFrame& orig,
                                                        const DestFrame& dest) = 0;
};


class ForceTransformsBase {
public:
    virtual ~ForceTransformsBase() {}

    virtual SpatialTransformPlain getTransform(const JointState& q,
                                                        const OriginFrame& orig,
                                                        const DestFrame& dest) = 0;
};

}

}

