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

#include <iostream>

#include "rbd.h"
#include "StateDependentBase.h"

namespace iit {
namespace rbd {


/**
 * A matrix that exposes a dependency on some kind of state variable.
 * The type of such a variable is the first template parameter.
 * The second and third parameters are the number of rows and columns.
 *
 * The last parameter must be a class inheriting from this class, i.e., we
 * are using here the curiously recurring template pattern.
 * Such a sub-class shall implement the update() function. See the documentation
 * of StateDependentBase.
 *
 * This class was created explicitly to support code generation
 * e.g. of coordinate transforms.
 */
template<class State, int Rows, int Cols, class ActualMatrix>
class StateDependentMatrix  :
        public StateDependentBase<State, ActualMatrix>,
        public PlainMatrix<typename State::Scalar, Rows, Cols>
{
    private:
        typedef PlainMatrix<typename State::Scalar, Rows, Cols> Base;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /** The type of the coefficients of this matrix */
        typedef typename Base::Scalar Scalar;
        /** The type of row/column indices of this matrix */
        typedef typename Base::Index  Index;
        /** The regular matrix type this class inherits from */
        typedef Base MatrixType;
    public:
        StateDependentMatrix() {};
        ~StateDependentMatrix() {}

        template<typename OtherDerived>
         StateDependentMatrix& operator= (const MatrixBase<OtherDerived>& other) {
             this->Base::operator=(other);
             return *this;
         }

        using StateDependentBase<State, ActualMatrix>::operator();
        using MatrixType::operator();
};


}
}
