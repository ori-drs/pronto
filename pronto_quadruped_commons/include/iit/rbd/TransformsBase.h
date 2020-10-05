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

#include "StateDependentMatrix.h"

namespace iit {
namespace rbd {


/**
 * A 3x3 specialization of StateDependentMatrix, to be used as a base class for
 * rotation matrices that depend on a state variable.
 */
template<class State, class ActualMatrix>
class RotationTransformBase : public StateDependentMatrix<State, 3, 3, ActualMatrix>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * A 4x4 specialization of StateDependentMatrix, to be used as a base class for
 * homogeneous transformation matrices that depend on a state variable.
 */
template<class State, class ActualMatrix>
class HomogeneousTransformBase : public StateDependentMatrix<State, 4, 4, ActualMatrix>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * A 6x6 specialization of StateDependentMatrix, to be used as a base class for
 * spatial transformation matrices that depend on a state variable.
 */
template<class State, class ActualMatrix>
class SpatialTransformBase : public StateDependentMatrix<State, 6, 6, ActualMatrix>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * A 6xCols specialization of StateDependentMatrix, to be used as a base class
 * for geometric Jacobians that depend on a state variable.
 */
template<class State, int Cols, class ActualMatrix>
class JacobianBase : public StateDependentMatrix<State, 6, Cols, ActualMatrix>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}
}
