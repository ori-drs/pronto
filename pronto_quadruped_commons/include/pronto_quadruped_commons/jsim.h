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

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>
#include <pronto_quadruped_commons/declarations.h>
#include <iit/rbd/InertiaMatrix.h>

namespace pronto {
namespace quadruped {

typedef iit::rbd::InertiaMatrixDense InertiaMatrix;
/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot HyQ.
 */
class JSIMBase : public iit::rbd::StateDependentMatrix<pronto::quadruped::JointState, 18, 18, JSIMBase>
{
public:
    typedef Eigen::Matrix<double,18,18> MatrixType;
    /** The type of the F sub-block of the floating-base JSIM */
    typedef const Eigen::Block<const MatrixType,6,12> BlockF_t;
    /** The type of the fixed-base sub-block of the JSIM */
    typedef const Eigen::Block<const MatrixType,12,12> BlockFixedBase_t;

public:
    virtual const JSIMBase& update(const pronto::quadruped::JointState&) = 0;

    /**
         * Computes and saves the matrix L of the L^T L factorization of this JSIM.
         */
    virtual void computeL() = 0;
    /**
         * Computes and saves the inverse of this JSIM.
         * This function assumes that computeL() has been called already, since it
         * uses L to compute the inverse. The algorithm takes advantage of the branch
         * induced sparsity of the robot, if any.
         */
    virtual void computeInverse() = 0;
    /**
         * Returns an unmodifiable reference to the matrix L. See also computeL()
         */
    virtual const MatrixType& getL() const = 0;
    /**
         * Returns an unmodifiable reference to the inverse of this JSIM
         */
    virtual const MatrixType& getInverse() const = 0;

    /**
         * The spatial composite-inertia tensor of the robot base,
         * ie the inertia of the whole robot for the current configuration.
         * According to the convention of this class about the layout of the
         * floating-base JSIM, this tensor is the 6x6 upper left corner of
         * the JSIM itself.
         * \return the 6x6 InertiaMatrix that correspond to the spatial inertia
         *   tensor of the whole robot, according to the last joints configuration
         *   used to update this JSIM
         */
    virtual const InertiaMatrix& getWholeBodyInertia() const = 0;
    /**
         * The matrix that maps accelerations in the actual joints of the robot
         * to the spatial force acting on the floating-base of the robot.
         * This matrix is the F sub-block of the JSIM in Featherstone's notation.
         * \return the 6x12 upper right block of this JSIM
         */
    virtual const BlockF_t getF() const = 0;
    /**
         * The submatrix of this JSIM related only to the actual joints of the
         * robot (as for a fixed-base robot).
         * This matrix is the H sub-block of the JSIM in Featherstone's notation.
         * \return the 12x12 lower right block of this JSIM,
         *   which correspond to the fixed-base JSIM
         */
    virtual const BlockFixedBase_t getFixedBaseBlock() const = 0;
protected:
    /**
         * Computes and saves the inverse of the matrix L. See also computeL()
         */
    virtual void computeLInverse() = 0;
};

} // namespace quadruped
} // namespace pronto

