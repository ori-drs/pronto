/*
 * Copyright (c) 2021 University of Oxford 
 * All rights reserved.
 *
 * Author: Marco Camurri (mcamurri@robots.ox.ac.uk)
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

#include <pronto_quadruped_commons/leg_data_map.h>
#include <Eigen/Dense>

namespace pronto {
namespace quadruped {

/**
 * @brief The LegVectorMap class specializes the LegDataMap with extended
 * functionalities for operations with Vector3 elements
 */
class LegVectorMap : public LegDataMap<Eigen::Vector3d> {
public:
  using Vector = Eigen::Vector3d;
public:
    LegVectorMap() : LegDataMap<Vector>(Vector::Zero()) {}
    LegVectorMap(const Vector& defaultValue) : LegDataMap<Vector>(defaultValue) {}
    LegVectorMap(const LegVectorMap& rhs) : LegDataMap<Vector>(rhs) {}
};

inline std::ostream& operator<<(std::ostream& out, const LegVectorMap& map) {
    const Eigen::IOFormat clean_format_ = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");
    out << "LF = " << map[LF].transpose().format(clean_format_) << "' " << std::endl;
    out << "RF = " << map[RF].transpose().format(clean_format_) << "' " << std::endl;
    out << "LH = " << map[LH].transpose().format(clean_format_) << "' " << std::endl;
    out << "RH = " << map[RH].transpose().format(clean_format_) << "' " << std::endl;
    return out;
}

} // namespace quadruped
} // namespace pronto

