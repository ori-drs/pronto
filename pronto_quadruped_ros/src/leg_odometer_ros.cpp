/* Copyright (c) 2018-2021 University of Oxford
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
    LegOdometer(feet_jacobians, forward_kinematics), nh_(nh)
{
    // get parameters for the leg odometry
    std::string legodo_prefix = "legodo/";
    int legodo_mode;
    if(!nh_.getParam(legodo_prefix + "legodo_mode", legodo_mode)){
        ROS_WARN_STREAM("Could not get param \"" << legodo_prefix + "legodo_mode\"" << ". Using default.");
    }

    // # STATIC_SIGMA 0x00, VAR_SIGMA 0x01, IMPACT_SIGMA 0x02, WEIGHTED_AVG 0x04, ALPHA_FILTER : 0x08, KALMAN_FILTER : 0x10
    switch(legodo_mode){
    case 0: // 000
      setMode(SigmaMode::STATIC_SIGMA, AverageMode::SIMPLE_AVG);
      break;
    case 1: // 001
      setMode(SigmaMode::VAR_SIGMA, AverageMode::SIMPLE_AVG);
      break;
    case 2: // 010
      setMode(SigmaMode::IMPACT_SIGMA, AverageMode::SIMPLE_AVG);
      break;
    case 3: // 011
      setMode(SigmaMode::VAR_AND_IMPACT_SIGMA, AverageMode::SIMPLE_AVG);
      break;
    case 4: // 100
      setMode(SigmaMode::STATIC_SIGMA, AverageMode::WEIGHTED_AVG);
      break;
    case 5: // 101
      setMode(SigmaMode::VAR_SIGMA, AverageMode::WEIGHTED_AVG);
      break;
    case 6: // 110
      setMode(SigmaMode::IMPACT_SIGMA, AverageMode::WEIGHTED_AVG);
      break;
    case 7: // 111
      setMode(SigmaMode::VAR_AND_IMPACT_SIGMA, AverageMode::WEIGHTED_AVG);
      break;
    default:
      setMode(SigmaMode::STATIC_SIGMA, AverageMode::SIMPLE_AVG);
    }
}

}  // namespace quadruped
}  // namespace pronto
