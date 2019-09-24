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

#include "joint_id_tricks.h"

namespace pronto {
namespace quadruped {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[LF_HAA] = rhs[LF_HAA];
    data[LF_HFE] = rhs[LF_HFE];
    data[LF_KFE] = rhs[LF_KFE];
    data[RF_HAA] = rhs[RF_HAA];
    data[RF_HFE] = rhs[RF_HFE];
    data[RF_KFE] = rhs[RF_KFE];
    data[LH_HAA] = rhs[LH_HAA];
    data[LH_HFE] = rhs[LH_HFE];
    data[LH_KFE] = rhs[LH_KFE];
    data[RH_HAA] = rhs[RH_HAA];
    data[RH_HFE] = rhs[RH_HFE];
    data[RH_KFE] = rhs[RH_KFE];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[LF_HAA] = value;
    data[LF_HFE] = value;
    data[LF_KFE] = value;
    data[RF_HAA] = value;
    data[RF_HFE] = value;
    data[RF_KFE] = value;
    data[LH_HAA] = value;
    data[LH_HFE] = value;
    data[LH_KFE] = value;
    data[RH_HAA] = value;
    data[RH_HFE] = value;
    data[RH_KFE] = value;
}


//operator overload defined out of the class
template<typename T> inline
JointDataMap<T> operator+(const JointDataMap<T>& lhs, const JointDataMap<T>& rhs)
{
	JointDataMap<T> out;
	out[LF_HAA] = lhs[LF_HAA] + rhs[LF_HAA];
	out[LF_HFE] = lhs[LF_HFE] + rhs[LF_HFE];
	out[LF_KFE] = lhs[LF_KFE] + rhs[LF_KFE];
	out[RF_HAA] = lhs[RF_HAA] + rhs[RF_HAA];
	out[RF_HFE] = lhs[RF_HFE] + rhs[RF_HFE];
	out[RF_KFE] = lhs[RF_KFE] + rhs[RF_KFE];
	out[LH_HAA] = lhs[LH_HAA] + rhs[LH_HAA];
	out[LH_HFE] = lhs[LH_HFE] + rhs[LH_HFE];
	out[LH_KFE] = lhs[LH_KFE] + rhs[LH_KFE];
	out[RH_HAA] = lhs[RH_HAA] + rhs[RH_HAA];
	out[RH_HFE] = lhs[RH_HFE] + rhs[RH_HFE];
	out[RH_KFE] = lhs[RH_KFE] + rhs[RH_KFE];
	return out;
}

//operator overload defined out of the class
template<typename T> inline
JointDataMap<T> operator-(const JointDataMap<T>& lhs, const JointDataMap<T>& rhs)
{
	JointDataMap<T> out;
	out[LF_HAA] = lhs[LF_HAA] - rhs[LF_HAA];
	out[LF_HFE] = lhs[LF_HFE] - rhs[LF_HFE];
	out[LF_KFE] = lhs[LF_KFE] - rhs[LF_KFE];
	out[RF_HAA] = lhs[RF_HAA] - rhs[RF_HAA];
	out[RF_HFE] = lhs[RF_HFE] - rhs[RF_HFE];
	out[RF_KFE] = lhs[RF_KFE] - rhs[RF_KFE];
	out[LH_HAA] = lhs[LH_HAA] - rhs[LH_HAA];
	out[LH_HFE] = lhs[LH_HFE] - rhs[LH_HFE];
	out[LH_KFE] = lhs[LH_KFE] - rhs[LH_KFE];
	out[RH_HAA] = lhs[RH_HAA] - rhs[RH_HAA];
	out[RH_HFE] = lhs[RH_HFE] - rhs[RH_HFE];
	out[RH_KFE] = lhs[RH_KFE] - rhs[RH_KFE];
	return out;
}

//operator overload defined out of the class
template<typename T> inline
JointDataMap<T> operator*(const JointDataMap<T>& lhs, const double  rhs)
{
	JointDataMap<T> out;
	out[LF_HAA] = lhs[LF_HAA]*rhs;
	out[LF_HFE] = lhs[LF_HFE]*rhs;
	out[LF_KFE] = lhs[LF_KFE]*rhs;
	out[RF_HAA] = lhs[RF_HAA]*rhs;
	out[RF_HFE] = lhs[RF_HFE]*rhs;
	out[RF_KFE] = lhs[RF_KFE]*rhs;
	out[LH_HAA] = lhs[LH_HAA]*rhs;
	out[LH_HFE] = lhs[LH_HFE]*rhs;
	out[LH_KFE] = lhs[LH_KFE]*rhs;
	out[RH_HAA] = lhs[RH_HAA]*rhs;
	out[RH_HFE] = lhs[RH_HFE]*rhs;
	out[RH_KFE] = lhs[RH_KFE]*rhs;
	return out;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   LF_HAA = "
    << map[LF_HAA]
    << "   LF_HFE = "
    << map[LF_HFE]
    << "   LF_KFE = "
    << map[LF_KFE]
    << "   RF_HAA = "
    << map[RF_HAA]
    << "   RF_HFE = "
    << map[RF_HFE]
    << "   RF_KFE = "
    << map[RF_KFE]
    << "   LH_HAA = "
    << map[LH_HAA]
    << "   LH_HFE = "
    << map[LH_HFE]
    << "   LH_KFE = "
    << map[LH_KFE]
    << "   RH_HAA = "
    << map[RH_HAA]
    << "   RH_HFE = "
    << map[RH_HFE]
    << "   RH_KFE = "
    << map[RH_KFE]
    ;
    return out;
}

}
}

