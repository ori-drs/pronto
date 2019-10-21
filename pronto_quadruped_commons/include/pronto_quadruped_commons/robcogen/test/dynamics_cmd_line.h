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

#include "pronto_quadruped_commons/rbd/utils.h"

namespace pronto {
namespace robcogen {
namespace test {

/**
 * \name Command-line tests of the dynamics.
 *
 * Very simple "tests", which parse the argc/argv arguments available in a
 * main(), and print to standard output the result of a dynamics algorithm.
 *
 * The command line arguments are expected to be enough to initialize all the
 * joint-status vectors required as input by the dynamics algorithm.
 */
///@{
template<class RT>
void cmdline_fixedBase_invdyn(
        int argc,
        char** argv,
        typename RT::InvDynEngine& id)
{
    typename RT::JointState q, qd, qdd, tau;
    utils::cmdlineargs_jstate<RT>(argc-1, &(argv[1]), q, qd, qdd);
    id.id(tau, q,qd,qdd);
    std::cout << tau << std::endl;
}

template<class RT>
void cmdline_fixedBase_fwddyn(
        int argc,
        char** argv,
        typename RT::FwdDynEngine& fd)
{
    typename RT::JointState q, qd, qdd, tau;
    utils::cmdlineargs_jstate<RT>(argc-1, &(argv[1]), q, qd, tau);
    fd.fd(qdd, q,qd,tau);
    std::cout << qdd << std::endl;
}

template<class RT>
void cmdline_jsim(
        int argc,
        char** argv,
        typename RT::JSIM& jsim)
{
    typename RT::JointState q;
    utils::cmdlineargs_jstate<RT>(argc-1, &(argv[1]), q);
    std::cout << jsim(q) << std::endl;
}
///@}

}
}
}
