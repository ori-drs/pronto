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

#include <pronto_quadruped_commons/leg_data_map.h>

namespace pronto {

namespace quadruped {

/**
 * @brief The LegBoolMap class specializes the LegDataMap with extended
 * functionalities for logic operations when the type is a bool, including:
 * negation, or, and, cwise or and cwise and.
 */
class LegBoolMap : public LegDataMap<bool> {

public:
    LegBoolMap() : LegDataMap<bool>(false) {}
    LegBoolMap(const bool& defaultValue) : LegDataMap<bool>(defaultValue) {}
    LegBoolMap(const LegBoolMap& rhs) : LegDataMap<bool>(rhs) {}

    inline LegBoolMap operator!() const {
        LegBoolMap res;
        res[LF] = !(*this)[LF];
        res[RF] = !(*this)[RF];
        res[LH] = !(*this)[LH];
        res[RH] = !(*this)[RH];
        return res;
    }

    inline LegBoolMap operator || (const LegBoolMap& rhs) const {
        LegBoolMap res;
        res[LF] = (*this)[LF] || rhs[LF];
        res[RF] = (*this)[RF] || rhs[RF];
        res[LH] = (*this)[LH] || rhs[LH];
        res[RH] = (*this)[RH] || rhs[RH];
        return res;
    }

    inline LegBoolMap operator && (const LegBoolMap& rhs) const {
        LegBoolMap res;
        res[LF] = (*this)[LF] && rhs[LF];
        res[RF] = (*this)[RF] && rhs[RF];
        res[LH] = (*this)[LH] && rhs[LH];
        res[RH] = (*this)[RH] && rhs[RH];
        return res;
    }

    static inline bool OR(const LegDataMap<bool>& rhs) {
        return(rhs[LF] || rhs[RF] || rhs[LH] || rhs[RH]);
    }

    inline bool OR() const {
        return ((*this)[LF] || (*this)[RF] || (*this)[LH] || (*this)[RH]);
    }


    static inline bool AND(const LegDataMap<bool>& rhs) {
        return(rhs[LF] && rhs[RF] && rhs[LH] && rhs[RH]);
    }

    inline bool AND() const {
        return ((*this)[LF] && (*this)[RF] && (*this)[LH] && (*this)[RH]);
    }
};

}  // namespace quadruped
}  // namespace pronto
