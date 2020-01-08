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

#include "rbd.h"

namespace iit {
namespace rbd {

/**
 * A sort of minimal interface to model the dependency on some kind of state
 * variable.
 * The type of such a state variable is the first template parameter.
 *
 * The second parameter must be a class inheriting from this one, i.e., this
 * class is supposed to be used according to the "curiously recurring template
 * pattern".
 * The sub-class shall implement the update() function with the actual,
 * class-specific logic to update the instance, because the implementation
 * of the operator(State const&) relies on update(State const&).
 */
template<class State, class Actual>
class StateDependentBase
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /**
         * Updates this object according to the given state.
         * The actual logic of the update must be implemented in the update()
         * function.
         */
        const Actual& operator()(State const& state) {
            return static_cast<Actual*>(this) -> update(state);
        }
        /**
         * Updates this object according to the given state variable.
         * Subclasses shall implement this method since the implementation of
         * operator()(State const&) relies on it.
         */
        const Actual& update(State const& state);
};

}
}
