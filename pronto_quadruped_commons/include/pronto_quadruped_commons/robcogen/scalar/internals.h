/*
 * Copyright (c) 2015-2018, Marco Frigerio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <type_traits>
#include "pronto_quadruped_commons/rbd/scalar_traits.h"

namespace pronto {
namespace robcogen {

namespace internal {

// ScalarTraitsSelector is a metafunction that extracts the ScalarTraits
// from a given a robot-traits. If the robot-traits does not define a
// ScalarTraits field, the metafunction resolves to the default traits for the
// double numeric type.
// This selector is provided purely for backwards compatibility with older
// versions of RobCoGen, which generate a robot-traits struct that does not
// include the ScalarTraits field.

// Usage example:
// typedef typename internal::ScalarTraitsSelector< ROBOT_TRAITS >::trait RobotScalarTrait;

// For some explanation of the mechanism, see Wikipedia's page about SFINAE,


// This alias always resolves to 'void'
template<typename ... > using void_t = void;

// This alias resolves to void if the template argument defines a 'ScalarTraits'
// field; otherwise the template argument substitution would fail.
template<typename ROBOT_TRAITS> using VoidIfDef = void_t<typename ROBOT_TRAITS::ScalarTraits>;


// #1: Primary template, "returns" the default traits
template <typename ROBOT_TRAITS, typename = void>
struct ScalarTraitsSelector
{
    typedef typename iit::rbd::DoubleTraits trait; // "default value"
};

// #2: A _partial specialization_ that resolves to ...<ROBOT_TRAITS, void> when
// ROBOT_TRAITS::ScalarTraits exists.
template <typename ROBOT_TRAITS>
struct ScalarTraitsSelector<ROBOT_TRAITS, VoidIfDef<ROBOT_TRAITS>  >
{
    typedef typename ROBOT_TRAITS::ScalarTraits trait;
};

// In my understanding, the trick works because an explicit template argument
// is preferred over a default argument.
// This client code:
//
//   ScalarTraitsSelector<RT>
//
// results in the primary template match with <RT, =void>
// If the robot-traits RT does define 'ScalarTraits', the partial specialization
// is <RT, void>, therefore it also matches and it is selected because preferred
// over <RT, =void>
// If RT does _not_ define 'ScalarTraits', then matching #2 fails in the first
// place, and the only candidate is the primary template #1.


}
}
}
