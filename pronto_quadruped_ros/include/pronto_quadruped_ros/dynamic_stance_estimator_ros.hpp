/* Copyright (c) 2018-2019 University of Oxford
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
#include <pronto_quadruped_commons/inverse_dynamics.h>
#include <pronto_quadruped/DynamicStanceEstimator.hpp>
#include <ros/node_handle.h>

namespace pronto {
namespace quadruped {

class DynamicStanceEstimatorROS : public DynamicStanceEstimator {
public:
    typedef pronto::quadruped::InverseDynamicsBase InverseDynamicsBase;
    typedef pronto::quadruped::JSIMBase JSIMBase;
    typedef pronto::quadruped::FeetContactForces FeetContactForces;
    typedef pronto::quadruped::ForwardKinematics ForwardKinematics;

public:
    DynamicStanceEstimatorROS(ros::NodeHandle& nh,
                              InverseDynamicsBase& inverse_dynamics,
                              JSIMBase& jsim,
                              FeetContactForces& feet_contact_forces,
                              ForwardKinematics& forward_kinematics);
private:
    ros::NodeHandle& nh_;
};

} // namespace quadruped
} // namespace pronto
