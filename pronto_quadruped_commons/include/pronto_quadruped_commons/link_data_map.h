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

#include "declarations.h"

namespace pronto {
namespace quadruped {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[TRUNK] = rhs[TRUNK];
    data[LF_HIPASSEMBLY] = rhs[LF_HIPASSEMBLY];
    data[LF_UPPERLEG] = rhs[LF_UPPERLEG];
    data[LF_LOWERLEG] = rhs[LF_LOWERLEG];
    data[RF_HIPASSEMBLY] = rhs[RF_HIPASSEMBLY];
    data[RF_UPPERLEG] = rhs[RF_UPPERLEG];
    data[RF_LOWERLEG] = rhs[RF_LOWERLEG];
    data[LH_HIPASSEMBLY] = rhs[LH_HIPASSEMBLY];
    data[LH_UPPERLEG] = rhs[LH_UPPERLEG];
    data[LH_LOWERLEG] = rhs[LH_LOWERLEG];
    data[RH_HIPASSEMBLY] = rhs[RH_HIPASSEMBLY];
    data[RH_UPPERLEG] = rhs[RH_UPPERLEG];
    data[RH_LOWERLEG] = rhs[RH_LOWERLEG];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[TRUNK] = value;
    data[LF_HIPASSEMBLY] = value;
    data[LF_UPPERLEG] = value;
    data[LF_LOWERLEG] = value;
    data[RF_HIPASSEMBLY] = value;
    data[RF_UPPERLEG] = value;
    data[RF_LOWERLEG] = value;
    data[LH_HIPASSEMBLY] = value;
    data[LH_UPPERLEG] = value;
    data[LH_LOWERLEG] = value;
    data[RH_HIPASSEMBLY] = value;
    data[RH_UPPERLEG] = value;
    data[RH_LOWERLEG] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   trunk = "
    << map[TRUNK]
    << "   LF_hipassembly = "
    << map[LF_HIPASSEMBLY]
    << "   LF_upperleg = "
    << map[LF_UPPERLEG]
    << "   LF_lowerleg = "
    << map[LF_LOWERLEG]
    << "   RF_hipassembly = "
    << map[RF_HIPASSEMBLY]
    << "   RF_upperleg = "
    << map[RF_UPPERLEG]
    << "   RF_lowerleg = "
    << map[RF_LOWERLEG]
    << "   LH_hipassembly = "
    << map[LH_HIPASSEMBLY]
    << "   LH_upperleg = "
    << map[LH_UPPERLEG]
    << "   LH_lowerleg = "
    << map[LH_LOWERLEG]
    << "   RH_hipassembly = "
    << map[RH_HIPASSEMBLY]
    << "   RH_upperleg = "
    << map[RH_UPPERLEG]
    << "   RH_lowerleg = "
    << map[RH_LOWERLEG]
    ;
    return out;
}

}
}

