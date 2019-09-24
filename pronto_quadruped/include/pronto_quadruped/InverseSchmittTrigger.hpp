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

namespace pronto {

template <typename T>
using LegDataMap = pronto::quadruped::LegDataMap<T>;

typedef pronto::quadruped::LegBoolMap LegBoolMap;
typedef pronto::quadruped::LegDataMap<Eigen::Vector3d> LegVector3Map;

/**
 * @brief The InverseSchmittTrigger class implements a Schmitt trigger which
 * behaves the opposite of an ordinary Schmitt trigger.
 */
class InverseSchmittTrigger {
public:
    InverseSchmittTrigger(const unsigned int& low_utime = 250000, // 250 ms
                          const unsigned int& high_utime = 250000, // 250 ms
                          const float& low_threshold = 60, // 60 newton
                          const float& high_threshold = 200); // 200 newton

    LegBoolMap getStance(const uint64_t utime,
                         const LegDataMap<Eigen::Vector3d>& grf);

private:
    unsigned int low_utime_;
    unsigned int high_utime_;
    float low_threshold_;
    float high_threshold_;

    LegBoolMap old_stance_;
    LegDataMap<double> old_grf_z_;
    float high_timer_;
    uint64_t high_start_utime_;
    float low_timer_;
    uint64_t low_start_utime_;

    void updateTimer(const uint64_t& utime);
};

} // namespace pronto
