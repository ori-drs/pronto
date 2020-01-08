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
#include "TransformsBase.h"

/**
 * \file
 * This header file contains some instantiations of the \c traits template (in
 * the \c Eigen::internal namespace) for matrix types defined in \c iit::rbd
 */

namespace Eigen {
namespace internal {

/**
 * The Eigen traits for the iit::rbd::HomogeneousTransformBase type
 */
template<typename State, typename M>
struct traits< iit::rbd::HomogeneousTransformBase<State, M> >
{
        typedef typename iit::rbd::HomogeneousTransformBase<State, M>::MatrixType MxType;
        typedef traits<MxType> Traits;
        typedef typename Traits::Scalar Scalar;
        typedef typename Traits::StorageKind StorageKind;
        typedef typename Traits::Index Index;
        typedef typename Traits::XprKind XprKind;
        enum {
            RowsAtCompileTime    = Traits::RowsAtCompileTime,
            ColsAtCompileTime    = Traits::ColsAtCompileTime,
            MaxRowsAtCompileTime = Traits::MaxRowsAtCompileTime,
            MaxColsAtCompileTime = Traits::MaxColsAtCompileTime,
            Options = Traits::Options,
            Flags   = Traits::Flags,
            CoeffReadCost = Traits::CoeffReadCost,
            InnerStrideAtCompileTime = Traits::InnerStrideAtCompileTime,
            OuterStrideAtCompileTime = Traits::OuterStrideAtCompileTime
        };
};

/**
 * The Eigen traits for the iit::rbd::RotationTransformBase type
 */
template<typename State, typename M>
struct traits< iit::rbd::RotationTransformBase<State, M> >
{
        typedef typename iit::rbd::RotationTransformBase<State, M>::MatrixType MxType;
        typedef traits<MxType> Traits;
        typedef typename Traits::Scalar Scalar;
        typedef typename Traits::StorageKind StorageKind;
        typedef typename Traits::Index Index;
        typedef typename Traits::XprKind XprKind;
        enum {
            RowsAtCompileTime    = Traits::RowsAtCompileTime,
            ColsAtCompileTime    = Traits::ColsAtCompileTime,
            MaxRowsAtCompileTime = Traits::MaxRowsAtCompileTime,
            MaxColsAtCompileTime = Traits::MaxColsAtCompileTime,
            Options = Traits::Options,
            Flags   = Traits::Flags,
            CoeffReadCost = Traits::CoeffReadCost,
            InnerStrideAtCompileTime = Traits::InnerStrideAtCompileTime,
            OuterStrideAtCompileTime = Traits::OuterStrideAtCompileTime
        };
};


}
}
