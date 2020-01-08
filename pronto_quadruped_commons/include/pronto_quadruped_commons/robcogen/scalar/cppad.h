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

#include <cppad/cppad.hpp>

#include "pronto_quadruped_commons/rbd/rbd.h"
#include "pronto_quadruped_commons/rbd/scalar_traits.h"


namespace iit {
namespace robcogen {

namespace cppad {


typedef CppAD::AD<double> Double;
typedef CppAD::AD<float>  Float;

template<typename Scalar>
struct TraitFuncs
{
    inline static Scalar sin (const Scalar& x) { return CppAD::sin(x); }
    inline static Scalar cos (const Scalar& x) { return CppAD::cos(x); }
    inline static Scalar sqrt(const Scalar& x) { return CppAD::sqrt(x); }
    inline static Scalar abs (const Scalar& x) { return CppAD::abs(x); }
};


} //namespace 'cppad'



template<typename CppADFloat>
struct BasicTraits
{
    typedef typename CppADFloat::value_type ValueType;
    typedef CppADFloat Scalar;
};


template<typename CppADFloat>
struct ScalarTraits : public rbd::ScalarTraitsCommons< BasicTraits<CppADFloat> >,
                      public cppad::TraitFuncs< typename BasicTraits<CppADFloat>::Scalar >
{};

typedef ScalarTraits< cppad::Double > CppADDoubleTraits;
typedef ScalarTraits< cppad::Float  > CppADFloatTraits;


} // namespace robcogen

namespace rbd {

template<> struct ScalarTraits<robcogen::cppad::Float> : public robcogen::CppADFloatTraits {};
template<> struct ScalarTraits<robcogen::cppad::Double>: public robcogen::CppADDoubleTraits {};

}

} // namespace iit




namespace Eigen {

template<> struct NumTraits<iit::robcogen::cppad::Float>  : NumTraits<float> {};
template<> struct NumTraits<iit::robcogen::cppad::Double> : NumTraits<double> {};

}

