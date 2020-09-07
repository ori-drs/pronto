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
    nh_.getParam(legodo_prefix + "stance_regression_beta", beta);
    double hysteresis_low;
    nh_.getParam(legodo_prefix + "stance_hysteresis_low", hysteresis_low);
    double hysteresis_high;
    nh_.getParam(legodo_prefix + "stance_hysteresis_high", hysteresis_high);
    double stance_threshold;
    nh_.getParam(legodo_prefix + "stance_threshold", stance_threshold);

    int stance_mode;
    nh_.getParam(legodo_prefix + "stance_mode", stance_mode);
    setMode(static_cast<StanceEstimator::Mode>(stance_mode));

    setParams(beta,
              stance_threshold,
              hysteresis_low,
              hysteresis_high);
}

}
}
