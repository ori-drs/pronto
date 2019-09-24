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

#include <pronto_quadruped_commons/leg_data_map.h>
#include <pronto_quadruped_commons/leg_bool_map.h>
#include <cmath>
#include <vector>

namespace pronto {

typedef pronto::quadruped::LegBoolMap StanceState;
typedef pronto::quadruped::LegDataMap<double> HipState;
typedef pronto::quadruped::LegDataMap<double> HipParam;
typedef pronto::quadruped::LegDataMap<std::string> HipName;


struct HipStates {
    uint64_t utime;
    HipName names;
    HipState position;
    HipState velocity;
    HipState effort;
    int num_joints = 4;
};

/**
 * @brief The HipParams struct contains Matlab's system identification toolbox
 * parameters for the identified system.

 */
struct HipParams {
    HipParam Kp = 0.0017329;
    HipParam Tw = 0.037571;
    HipParam Zeta = 1.6298;
    HipParam Tz = 0.1095;
};


/**
 * @brief The FlexEstimator class compensate for velocity errors due to leg
 *  flexibility. It treats the flexibility as rotational spring in the HAA
 * joints, and tries to capture the system parameter with system identification.
 * The dynamics is modeled as follows:
 * \f[
     \ddot{\mathbf{q}}(k+1) = \frac{T_z}{T_w^2}\Big(K_p(T_z
     \dot{\boldsymbol{\tau}}(k)+\boldsymbol{\tau})
     - (\mathbf{q}(k) - \mathbf{q}_0) - 2\; Z\; T_w\; \dot{\mathbf{q}}(k)\Big)
   \f]
 *
 * where \f$K_p,\;T_w,\;Z,\;T_z\f$ are described by the struct HipParams
 */
class FlexEstimator {
public:
    /**
     * @brief init sets the initial time for
     * @param time
     */
    void init(const double& time);
    /**
     * @brief computes the hip state (i.e. joint velocity) from the commanded
     * and actual joint positions, and current (uncompensated) joint velocity
     * @param[in] s stance state of the robot
     * @param[in] u commanded joint velocity
     * @param[in] q actual joint position
     * @param[in] qd actual joint velocity
     * @param[in] time absolute time (in seconds)
     * @param[out] qd_flex compensated joint velocity
     */
    void update(const StanceState& s,
                const HipState& u,
                const HipState& q,
                const HipState& qd,
                const double& time,
                HipState& qd_flex);

    /**
     * @brief set the estimator in debug mode (prints more)
     * @param debug whether we should debug or not
     */
    void setDebug(const bool &debug);

    /**
     * @brief getHipStates
     * @return returns the HipStates of the estimator.
     */
    const HipStates& getHipStates();

private:
    bool has_init_ = false;
    bool debug_ = false;
    double now_;
    double before_;
    double dt_;

    // HAA joint statuses (estimated)
    HipState q_; // joint position (estimated)
    HipState q0_; // initial joint position (measured)
    HipState qd_; // joint velocity (estimated)
    HipState qdd_; // joint acceleration (estimated)

    HipState qd_prev_; // joint velocity (estimated)
    HipState qdd_prev_; // joint acceleration (estimated)

    HipState tau_; // joint effort (measured)

    StanceState s_ = false; // current stance status

    // parameters obtained from system identification
    //HipParam Kp = 0.0017646;
    //HipParam Tw = 0.012707;
    //HipParam one_Tw_2 = 1/pow(Tw[0],2);
    //HipParam Zeta = 0.34733;
    //HipParam Tz = 0.0079444;

    HipParams hp_;

    HipStates hip_states_;
};

} // namespace pronto
