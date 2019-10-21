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

#include <stdexcept>
#include <iostream>


namespace pronto {
/**
 * This namespace holds interfaces and data types suitable to be used with
 * quadruped robots.
 *
 * More or less everything is based on the assumption of having four legs and
 * three joints per leg.
 */
namespace quadruped {

static const int _LEGS_COUNT = 4;
enum LegID{LF=0, RF, LH, RH};
enum LegJoints{HAA=0, HFE=1, KFE=2};


/**
 * A very simple container to associate a generic data item to each leg
 * (or anything related to a leg, e.g. a hip)
 *
 * \tparam T the type of the items that will be stored in the data map.
 */
template<typename T> class LegDataMap {
	private:
	    T data[_LEGS_COUNT];
	public:
	    LegDataMap() {};
	    LegDataMap(const T& defaultValue);
	    LegDataMap(const LegDataMap& rhs);
	    LegDataMap& operator=(const LegDataMap& rhs);
	    LegDataMap& operator=(const T& defaultValue);
	    T& operator[](LegID index);
	    T& operator[](int index);
	    const T& operator[](LegID index) const;
	    const T& operator[](int index) const;
	private:
	    void copydata(const LegDataMap& rhs);
	    void assignAll(const T& value);
	};

template<typename T> inline
LegDataMap<T>::LegDataMap(const T& defaultValue) {
    assignAll(defaultValue);
}

template<typename T> inline
LegDataMap<T>::LegDataMap(const LegDataMap& rhs) //cannot use initializer list for arrays?
{
    copydata(rhs);
}

template<typename T> inline
LegDataMap<T>& LegDataMap<T>::operator=(const LegDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LegDataMap<T>& LegDataMap<T>::operator=(const T& defaultValue)
{
    assignAll(defaultValue);
    return *this;
}

template<typename T> inline
T& LegDataMap<T>::operator[](LegID hip) {
    return data[hip];
}

template<typename T> inline
T& LegDataMap<T>::operator[](int index) {
    if(index<0 || index>_LEGS_COUNT) {
        throw(std::runtime_error("Leg index out of bounds"));
    }
    return data[index];
}

template<typename T> inline
const T& LegDataMap<T>::operator[](LegID hip) const {
    return data[hip];
}

template<typename T> inline
const T& LegDataMap<T>::operator[](int index) const {
    if(index<0 || index>_LEGS_COUNT) {
        throw(std::runtime_error("Leg index out of bounds"));
    }
    return data[index];
}

template<typename T> inline
void LegDataMap<T>::copydata(const LegDataMap& rhs) {
    data[LF] = rhs[LF];
    data[RF] = rhs[RF];
    data[LH] = rhs[LH];
    data[RH] = rhs[RH];
}

template<typename T> inline
void  LegDataMap<T>::assignAll(const T& value) {
    data[LF] = value;
    data[RF] = value;
    data[LH] = value;
    data[RH] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LegDataMap<T>& map) {
    out << "LF = " << map[LF] << "  RF = " << map[RF];
    out << "  LH = " << map[LH] << "  RH = " << map[RH];
    return out;
}

//operator overload defined out of the class legdatamap
template<typename T> inline
LegDataMap<T> operator+(const LegDataMap<T>& lhs, const LegDataMap<T>& rhs) {
    LegDataMap<T> out;
    out[LF] = lhs[LF] + rhs[LF];
    out[RF] = lhs[RF] + rhs[RF];
    out[LH] = lhs[LH] + rhs[LH];
    out[RH] = lhs[RH] + rhs[RH];
    return out;
}

template<typename T> inline
LegDataMap<T> operator-(const LegDataMap<T>& lhs, const LegDataMap<T>& rhs) {
    LegDataMap<T> out;
    out[LF] = lhs[LF] - rhs[LF];
    out[RF] = lhs[RF] - rhs[RF];
    out[LH] = lhs[LH] - rhs[LH];
    out[RH] = lhs[RH] - rhs[RH];
    return out;
}

}
}

