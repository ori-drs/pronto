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

#include <pronto_quadruped_commons/joint_data_map.h>

namespace pronto {

namespace quadruped {

/**
 * @brief The JointBoolMap class specializes the JointDataMap with extended
 * functionalities for logic operations when the type is a bool, including:
 * negation, or, and, cwise or and cwise and.
 */
class JointBoolMap : public JointDataMap<bool> {

public:
    JointBoolMap() : JointDataMap<bool>(false) {}
    JointBoolMap(const bool& defaultValue) : JointDataMap<bool>(defaultValue) {}
    JointBoolMap(const JointBoolMap& rhs) : JointDataMap<bool>(rhs) {}

    inline JointBoolMap operator!() const {
        JointBoolMap res;
        res[LF_HAA] = !(*this)[LF_HAA];
        res[LF_HFE] = !(*this)[LF_HFE];
        res[LF_KFE] = !(*this)[LF_KFE];
        res[RF_HAA] = !(*this)[RF_HAA];
        res[RF_HFE] = !(*this)[RF_HFE];
        res[RF_KFE] = !(*this)[RF_KFE];
        res[LH_HAA] = !(*this)[LH_HAA];
        res[LH_HFE] = !(*this)[LH_HFE];
        res[LH_KFE] = !(*this)[LH_KFE];
        res[RH_HAA] = !(*this)[RH_HAA];
        res[RH_HFE] = !(*this)[RH_HFE];
        res[RH_KFE] = !(*this)[RH_KFE];
        return res;
    }

    inline JointBoolMap operator || (const JointBoolMap& rhs) const {
        JointBoolMap res;
        res[LF_HAA] = (*this)[LF_HAA] || rhs[LF_HAA];
        res[LF_HFE] = (*this)[LF_HFE] || rhs[LF_HFE];
        res[LF_KFE] = (*this)[LF_KFE] || rhs[LF_KFE];
        res[RF_HAA] = (*this)[RF_HAA] || rhs[RF_HAA];
        res[RF_HFE] = (*this)[RF_HFE] || rhs[RF_HFE];
        res[RF_KFE] = (*this)[RF_KFE] || rhs[RF_KFE];
        res[LH_HAA] = (*this)[LH_HAA] || rhs[LH_HAA];
        res[LH_HFE] = (*this)[LH_HFE] || rhs[LH_HFE];
        res[LH_KFE] = (*this)[LH_KFE] || rhs[LH_KFE];
        res[RH_HAA] = (*this)[RH_HAA] || rhs[RH_HAA];
        res[RH_HFE] = (*this)[RH_HFE] || rhs[RH_HFE];
        res[RH_KFE] = (*this)[RH_KFE] || rhs[RH_KFE];
        return res;
    }

    inline JointBoolMap operator && (const JointBoolMap& rhs) const {
        JointBoolMap res;
        res[LF_HAA] = (*this)[LF_HAA] && rhs[LF_HAA];
        res[LF_HFE] = (*this)[LF_HFE] && rhs[LF_HFE];
        res[LF_KFE] = (*this)[LF_KFE] && rhs[LF_KFE];
        res[RF_HAA] = (*this)[RF_HAA] && rhs[RF_HAA];
        res[RF_HFE] = (*this)[RF_HFE] && rhs[RF_HFE];
        res[RF_KFE] = (*this)[RF_KFE] && rhs[RF_KFE];
        res[LH_HAA] = (*this)[LH_HAA] && rhs[LH_HAA];
        res[LH_HFE] = (*this)[LH_HFE] && rhs[LH_HFE];
        res[LH_KFE] = (*this)[LH_KFE] && rhs[LH_KFE];
        res[RH_HAA] = (*this)[RH_HAA] && rhs[RH_HAA];
        res[RH_HFE] = (*this)[RH_HFE] && rhs[RH_HFE];
        res[RH_KFE] = (*this)[RH_KFE] && rhs[RH_KFE];
        return res;
    }

    static inline bool OR(const JointDataMap<bool>& rhs) {
        return(rhs[LF_HAA] || rhs[LF_HFE] || rhs[LF_KFE] ||
               rhs[RF_HAA] || rhs[RF_HFE] || rhs[RF_KFE] ||
               rhs[LH_HAA] || rhs[LH_HFE] || rhs[LH_KFE] ||
               rhs[RH_HAA] || rhs[RH_HFE] || rhs[RH_KFE]);
    }

    inline bool OR() const {
        return ((*this)[LF_HAA] || (*this)[LF_HFE] || (*this)[LF_KFE] ||
                (*this)[RF_HAA] || (*this)[RF_HFE] || (*this)[RF_KFE] ||
                (*this)[LH_HAA] || (*this)[LH_HFE] || (*this)[LH_KFE] ||
                (*this)[RH_HAA] || (*this)[RH_HFE] || (*this)[RH_KFE]);
    }


    static inline bool AND(const JointDataMap<bool>& rhs) {
        return(rhs[LF_HAA] && rhs[LF_HFE] && rhs[LF_KFE] &&
               rhs[RF_HAA] && rhs[RF_HFE] && rhs[RF_KFE] &&
               rhs[LH_HAA] && rhs[LH_HFE] && rhs[LH_KFE] &&
               rhs[RH_HAA] && rhs[RH_HFE] && rhs[RH_KFE]);
    }

    inline bool AND() const {
        return ((*this)[LF_HAA] && (*this)[LF_HFE] && (*this)[LF_KFE] &&
                (*this)[RF_HAA] && (*this)[RF_HFE] && (*this)[RF_KFE] &&
                (*this)[LH_HAA] && (*this)[LH_HFE] && (*this)[LH_KFE] &&
                (*this)[RH_HAA] && (*this)[RH_HFE] && (*this)[RH_KFE]);
    }
};

}

}

