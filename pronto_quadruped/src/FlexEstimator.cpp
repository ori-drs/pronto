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

#include "pronto_quadruped/FlexEstimator.hpp"

namespace pronto {
void FlexEstimator::init(const double &time) {
    before_ = time;

    hip_states_.names[0] = "lf_haa_joint";
    hip_states_.names[1] = "rf_haa_joint";
    hip_states_.names[2] = "lh_haa_joint";
    hip_states_.names[3] = "rh_haa_joint";

    hip_states_.num_joints = 4;
}

void FlexEstimator::setDebug(const bool& debug) {
    debug_ = debug;
}

void FlexEstimator::update(const StanceState &s,
                           const HipState &tau,
                           const HipState &q,
                           const HipState &qd,
                           const double &time,
                           HipState &qd_flex) {
    if(!has_init_) {
        init(time);
        has_init_ = true;
        return;
    }
    now_ = time;
    dt_ = now_ - before_;

    for(int i = 0; i < 4; i++) {
        // leg transition from stance to swing OR swing period
        if(!s[i]) {
            // if we switch from stance to swing, or we are in swing period,
            // we apply no correction
            q_[i] = q[i];
            qd_[i] = qd[i];
            qd_flex[i] = qd[i];
            // leg transition from swing to stance, we re-initialize the counter
        } else if(!s_[i] && s[i]) {
            q_[i] = q[i];
            q0_[i] = q[i];
            qd_[i] = qd[i];
            qd_prev_[i] = qd[i];
            qdd_[i] = 0;
            qdd_prev_[i] = 0;
            qd_flex[i] = qd[i];
            // stance period
        } else if(s_[i] && s[i]) {
            // taud(k) = (tau(k) - tau(k-1))/dt
            double taud = (tau[i] - tau_[i]) / dt_;

            // qdd_prev = qdd_(k)
            qdd_prev_[i] = qdd_[i];
            //               1Tz
            // qdd(k + 1) = --- (Kp (Tz taud(k) + tau(k)) - (q(k) - q0) - 2 Zeta Tw qd(k))
            //                2
            //              Tw

            qdd_[i] = 1/(hp_.Tw[i]*hp_.Tw[i]) * (hp_.Kp[i] * (hp_.Tz[i] * taud + tau[i]) - (q_[i] - q0_[i]) - 2 * hp_.Zeta[i] * hp_.Tw[i] * qd_[i]);
            // qd(k)
            qd_prev_ = qd_[i];
            // qd(k+1) = (qd(k)+qd(
            qd_[i] += (qdd_prev_[i] + qdd_[i]) * dt_ / 2;
            q_[i] += (qd_[i] + qd_prev_[i]) * dt_ / 2;
            qd_flex[i] = qd_[i];

            // preparing for next cycle
            tau_[i] = tau[i];
        }
        hip_states_.position[i] = q_[i];
        hip_states_.velocity[i] = qd_[i];
        qd_flex[i] = qd[i] - qd_[i];
        hip_states_.effort[i] = qdd_[i];

        hip_states_.utime = time * 1000000;
    }

    // setting the stance status
    s_ = s;
    before_ = now_;
}

const HipStates& FlexEstimator::getHipStates(){
    return hip_states_;
}
} // namespace pronto
