/* Copyright (c) 2015-2019
 * Istituto Italiano di Tecnologia (IIT), University of Oxford
 * All rights reserved.
 *
 * Author: Marco Camurri (mcamurri@robots.ox.ac.uk)
 *
 * This file is part of pronto_quadruped,
 * a library for leg odometry on quadruped robots.
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

#include "pronto_quadruped/DataLogger.hpp"
#include "pronto_quadruped/StanceEstimatorBase.hpp"

// Eigen
#include <Eigen/Dense>

// iit-rbd
#include <pronto_quadruped_commons/rbd/rbd.h>
#include <pronto_quadruped_commons/rbd/utils.h>

// iit-commons
#include <pronto_quadruped_commons/declarations.h>
#include <pronto_quadruped_commons/feet_contact_forces.h>

// std
#include <memory>
#include <iostream>

namespace pronto {

using pronto::quadruped::LF;
using pronto::quadruped::RF;
using pronto::quadruped::LH;
using pronto::quadruped::RH;

using iit::rbd::X;
using iit::rbd::Y;
using iit::rbd::Z;

struct GRFStat {
    uint64_t tic_stance;
    uint64_t tic_swing;
    Eigen::ArrayXd stance_cluster;
    Eigen::ArrayXd swing_cluster;
    uint32_t cluster_size;
};

struct GRFShortStat {
    double mu_swing;
    double sigma_swing;
    double pi_swing;
    double delta_swing;
    double M2_swing;
    uint64_t n_swing;

    double mu_stance;
    double sigma_stance;
    double pi_stance;
    double delta_stance;
    double M2_stance;
    uint64_t n_stance;
};

class StanceEstimator : public StanceEstimatorBase {
    // typedef to remove ugly and verbose traited templated code
public:
    typedef typename pronto::quadruped::JointState JointState;
    typedef typename pronto::quadruped::FeetContactForces FeetContactForces;
    typedef typename pronto::quadruped::LegID LegID;



    enum Mode {THRESHOLD = 0, HYSTERESIS, REGRESSION};

public:
    StanceEstimator(FeetContactForces& feet_contact_forces,
                    double force_threshold = 50);

    StanceEstimator(FeetContactForces& feet_contact_forces,
                    const Mode& mode,
                    const std::vector<double>& beta = std::vector<double>(4, 0.25),
                    const double& force_threshold = 50,
                    const double& hysteresis_low = 50,
                    const double& hysteresis_high = 150);

    virtual ~StanceEstimator();

    inline LegBoolMap getStance(const double time,
                                const JointState &q,
                                const JointState &qd,
                                const JointState &tau,
                                const Quaterniond & orient,
                                const JointState &qdd = JointState::Constant(0),
                                const Vector3d& xd = Vector3d(0, 0, 0),
                                const Vector3d & xdd  = Vector3d(0, 0, 0),
                                const Vector3d & omega  = Vector3d(0, 0, 0),
                                const Vector3d & omegad = Vector3d(0, 0, 0)) {
        LegBoolMap stance;
        getStance(time, q, qd, tau, orient, stance, qdd, xd, xdd, omega, omegad);
        return stance;
    }

    inline bool getStance(const double time,
                          const JointState &q,
                          const JointState &qd,
                          const JointState &tau,
                          const Quaterniond & orient,
                          LegBoolMap& stance,
                          const JointState &qdd = JointState::Constant(0),
                          const Vector3d& xd = Vector3d(0, 0, 0),
                          const Vector3d & xdd  = Vector3d(0, 0, 0),
                          const Vector3d & omega  = Vector3d(0, 0, 0),
                          const Vector3d & omegad = Vector3d(0, 0, 0)) {
        LegScalarMap l;
        return getStance(time, q, qd, tau, orient, stance, l, qdd, xd, xdd, omega, omegad);
    }

    bool getStance(const double time,
                   const JointState &q,
                   const JointState &qd,
                   const JointState &tau,
                   const Quaterniond & orient,
                   LegBoolMap& stance,
                   LegScalarMap& stance_probability,
                   const JointState &qdd = JointState::Constant(0),
                   const Vector3d& xd = Vector3d(0, 0, 0),
                   const Vector3d & xdd  = Vector3d(0, 0, 0),
                   const Vector3d & omega  = Vector3d(0, 0, 0),
                   const Vector3d & omegad = Vector3d(0, 0, 0));

    virtual bool isStance(LegID leg);
    LegVectorMap getGRF();
    virtual void getGrf_W(LegDataMap<Eigen::Vector3d> & leg_status);
    virtual inline void getGrfDelta(LegDataMap<double>& grfDelta){
        grfDelta = grForceDelta;
    }

    virtual void getNormalizedGRF(Eigen::Vector4d& normgrf);
    virtual void updateStat(double sample,
                    bool is_stance,
                    int index);

    virtual void setMode(const Mode& mode);

    virtual void setParams(const std::vector<double>& beta,
                   const double& force_threshold = 50,
                   const double& hysteresis_low = 50,
                   const double& hysteresis_high = 150,
                   const int& hysteresis_delay_low = 250,
                   const int& hysteresis_delay_high = 250);

protected:
    FeetContactForces& feet_contact_forces_;
    LegDataMap<Eigen::Vector3d> grf_;
    LegDataMap<Eigen::Vector3d> grForce_W;
    LegDataMap<Eigen::Vector3d> grForce_des;
    LegDataMap<double> grForceDelta;

    Eigen::Vector3d violation_;

    Mode mode_ = THRESHOLD;

    double force_threshold_;
    double hysteresis_low_;
    double hysteresis_high_;
    double hysteresis_delay_low_;
    double hysteresis_delay_high_;

    std::vector<double>beta_;
    Eigen::Vector4d stance_weights_ = Eigen::Vector4d::Zero();

    LegBoolMap leg_status_;

    GRFStat stat;
    GRFShortStat gss[4];
};

} // namespace pronto
