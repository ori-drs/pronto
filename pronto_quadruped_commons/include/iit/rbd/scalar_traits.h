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

#include "iit/rbd/rbd.h"

namespace iit {
namespace rbd {


template<typename T>
struct BasicTraits
{
    typedef T ValueType;
    typedef T Scalar;
};

template struct BasicTraits<double>;
template struct BasicTraits<float>;


template<typename BasicTrait>
struct ScalarTraitsCommons
{
    typedef typename BasicTrait::Scalar    Scalar;
    typedef typename BasicTrait::ValueType ValueType;

    typedef typename rbd::Core<Scalar> CoreTypes;
    template<int R, int C> using PlainMatrix = rbd::PlainMatrix<Scalar, R, C>;

    template <int Dims>
    inline static PlainMatrix<Dims, 1> solve(
            const PlainMatrix<Dims, Dims>& A,
            const PlainMatrix<Dims, 1>& b)
    {
        return A.inverse()*b;
    }
};


namespace internal {

template<typename FLOAT_t>
struct FloatPointFuncs
{
    typedef FLOAT_t Float_t;
    inline static Float_t sin (const Float_t& x) { return std::sin(x);  }
    inline static Float_t cos (const Float_t& x) { return std::cos(x);  }
    inline static Float_t tan (const Float_t& x) { return std::tan(x);  }
    inline static Float_t sinh(const Float_t& x) { return std::sinh(x); }
    inline static Float_t cosh(const Float_t& x) { return std::cosh(x); }
    inline static Float_t tanh(const Float_t& x) { return std::tanh(x); }
    inline static Float_t exp (const Float_t& x) { return std::exp(x);  }
    inline static Float_t abs (const Float_t& x) { return std::abs(x);  }
    inline static Float_t fabs(const Float_t& x) { return std::fabs(x); }
    inline static Float_t sqrt(const Float_t& x) { return std::sqrt(x); }
};

} // namespace internal



template<typename SCALAR>
struct ScalarTraits : public ScalarTraitsCommons< BasicTraits<SCALAR> >,
                      public internal::FloatPointFuncs<SCALAR>
{
};

// Explicit _instantiation_ of the traits for double and float
template struct ScalarTraits<double>;
template struct ScalarTraits<float>;

typedef ScalarTraits<double> DoubleTraits;
typedef ScalarTraits<float>  FloatTraits;



}
}
