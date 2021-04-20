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

#include "pronto_quadruped_ros/stance_estimator_ros.hpp"

namespace pronto {
namespace quadruped {

StanceEstimatorROS::StanceEstimatorROS(ros::NodeHandle &nh,
                                       FeetContactForces &feet_forces) :
     StanceEstimator(feet_forces), nh_(nh)
{
    // get parameters for the leg odometry
    std::string legodo_prefix = "legodo/";

    // stance estimator parameters
    std::vector<double> beta;

    double hysteresis_low = 50;
    uint64_t stance_hysteresis_delay_low  = 0;
    uint64_t stance_hysteresis_delay_high = 0;
    int stance_hysteresis_delay_low_int = 0;
    int stance_hysteresis_delay_high_int = 0;
    double hysteresis_high = 50;
    double stance_threshold = 50;

    int stance_mode;
    if(!nh_.getParam(legodo_prefix + "stance_mode", stance_mode)){
        ROS_WARN("Could not read the stance mode from param server. Using threshold with default 50 N.");
        setMode(Mode::THRESHOLD);
    } else if(stance_mode < 3){
      setMode(static_cast<StanceEstimator::Mode>(stance_mode));
    } else {
      ROS_WARN("Invalid stance mode from param server. Using threshold with default 50 N.");
      setMode(Mode::THRESHOLD);
    }

    switch(mode_){
    case Mode::THRESHOLD:
      if(!nh_.getParam(legodo_prefix + "stance_threshold", stance_threshold)){
        ROS_WARN("Could not read the stance threshold from param server. Using default 50 N.");
      }
      break;
    case Mode::HYSTERESIS:
      if(!nh_.getParam(legodo_prefix + "stance_hysteresis_low", hysteresis_low)){
        ROS_WARN("Could not read the stance_hysteresis_low from param server. Using default 50 N.");
      }
      if(!nh_.getParam(legodo_prefix + "stance_hysteresis_high", hysteresis_high)){
        ROS_WARN("Could not read the stance_hysteresis_high from param server. Using default 50 N.");
      }
      if(!nh_.getParam(legodo_prefix + "stance_hysteresis_delay_low", stance_hysteresis_delay_low_int)){
        ROS_WARN("Could not read the stance_hysteresis_delay_low from param server. Using default 0 ns.");
      }
      if(!nh_.getParam(legodo_prefix + "stance_hysteresis_delay_high", stance_hysteresis_delay_high_int)){
        ROS_WARN("Could not read the stance_hysteresis_delay_high from param server. Using default 0 ns.");
      }
      stance_hysteresis_delay_low  = stance_hysteresis_delay_low_int;
      stance_hysteresis_delay_high = stance_hysteresis_delay_high_int;
      break;
    case Mode::REGRESSION:
      if(!nh_.getParam(legodo_prefix + "stance_regression_beta", beta)){
        ROS_WARN("Could not read the stance_regression_beta from param server. Setting mode to THRESHOLD with default value of 50 N.");
        setMode(Mode::THRESHOLD);
      }

    }
    setParams(beta, stance_threshold, hysteresis_low, hysteresis_high, stance_hysteresis_delay_low, stance_hysteresis_delay_high);
}

}
}
