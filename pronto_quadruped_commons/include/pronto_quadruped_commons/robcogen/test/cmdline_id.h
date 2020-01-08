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

#include <cmath>
#include <iostream>
#include <fstream>

#include <sstream>
#include <string>

#include "fext.h"

#include "pronto_quadruped_commons/rbd/rbd.h"
#include "pronto_quadruped_commons/robcogen/utils.h"
#include "pronto_quadruped_commons/robcogen/scalar/internals.h"


namespace pronto {
namespace robcogen {
namespace test {

/**
 *
 */
template<class ROB>
void cmdline_id(int argc, char** argv, typename ROB::InvDynEngine& id)
{
    //
    typename ROB::JointState q, qd, qdd, tau;
    robcogen::utils::cmdlineargs_jstate<ROB>(argc-1, &(argv[1]), q, qd, qdd);

    typedef typename internal::ScalarTraitsSelector< ROB >::trait::Scalar Scalar;
    //
    int c = ROB::joints_count * 3 + 1;
    typename ROB::FwdDynEngine::ExtForces fext(rbd::Force<Scalar>::Zero());
    if(argc > c) {
        std::string extForcesFile(argv[c]);
        robcogen::test::readExtForces<ROB>(extForcesFile, fext);
    }

    id.id(tau, q, qd, qdd, fext);
	std::cout << tau << std::endl;
}

template<class ROB>
void cmdline_id(int argc, char** argv)
{
    typename ROB::MotionTransforms  xm;
    typename ROB::InertiaProperties ip;
    typename ROB::InvDynEngine      id(ip, xm);
    cmdline_id<ROB> (argc, argv, id);
}

/**
 *
 */
template<class ROB>
void cmdline_id_fb(int argc, char** argv, typename ROB::InvDynEngine& id)
{
    //
    typename ROB::JointState q, qd, qdd, tau, tau2;
    robcogen::utils::cmdlineargs_jstate<ROB>(argc-1, &(argv[1]), q, qd, qdd);

    //
    rbd::VelocityVector v0, a0, grav;
    a0.setZero();
    v0.setZero();
    grav.setZero();
    grav(rbd::LZ) = -iit::rbd::g;

    // the number of "consumed" arguments so far
    int arg = ROB::joints_count * 3 + 1;

    v0(rbd::AX) = std::atof(argv[arg++]);
    v0(rbd::AY) = std::atof(argv[arg++]);
    v0(rbd::AZ) = std::atof(argv[arg++]);
    v0(rbd::LX) = std::atof(argv[arg++]);
    v0(rbd::LY) = std::atof(argv[arg++]);
    v0(rbd::LZ) = std::atof(argv[arg++]);

    //
    typename ROB::InvDynEngine::ExtForces fext(rbd::ForceVector::Zero());
    if(argc > arg) {
        std::string extForcesFile(argv[arg]);
        robcogen::test::readExtForces<ROB>(extForcesFile, fext);
    }

    id.id(tau, a0, grav, v0, q, qd, qdd, fext);
    std::cout << a0 << std::endl;
    std::cout << tau << std::endl;
}

template<class ROB>
void cmdline_id_fb(int argc, char** argv)
{
    typename ROB::MotionTransforms  xm;
    typename ROB::InertiaProperties ip;
    typename ROB::InvDynEngine      id(ip, xm);
    cmdline_id_fb<ROB> (argc, argv, id);
}

}
}
}
