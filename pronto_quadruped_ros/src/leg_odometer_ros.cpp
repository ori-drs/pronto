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

#include "pronto_quadruped_ros/leg_odometer_ros.hpp"

namespace pronto {
namespace quadruped {

LegOdometerROS::LegOdometerROS(ros::NodeHandle &nh,
                               FeetJacobians &feet_jacobians,
                               ForwardKinematics &forward_kinematics) :
    pronto::LegOdometer(feet_jacobians, forward_kinematics), nh_(nh)
{
    // get parameters for the leg odometry
    std::string legodo_prefix = "legodo/";
    int legodo_mode;
    nh_.getParam(legodo_prefix + "legodo_mode", legodo_mode);
    setMode(static_cast<LegOdometer::Mode>(legodo_mode));
}

}
}
