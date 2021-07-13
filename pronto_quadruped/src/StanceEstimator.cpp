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

#include "pronto_quadruped/StanceEstimator.hpp"
#include <cmath>

namespace pronto {
namespace quadruped {

StanceEstimator::StanceEstimator(FeetContactForces& feet_contact_forces,
                                 double force_threshold) :
  force_threshold_(force_threshold),
  feet_contact_forces_(feet_contact_forces),
  mode_(Mode::THRESHOLD)
{
    // Initializing the statistics member
    stat.cluster_size = 16;
    for(int i = 0; i < 4; i++) {
        gss[i].mu_stance = 400;
        gss[i].sigma_stance = 50;
        gss[i].mu_swing = 0;
        gss[i].sigma_swing = 250;
        grf_[i] = Eigen::Vector3d::Zero();
        grForceDelta[i] = 0.0;
        grForce_des[i] = Eigen::Vector3d::Zero();
        grForce_W[i] = Eigen::Vector3d::Zero();
    }
}

StanceEstimator::StanceEstimator(FeetContactForces& feet_contact_forces,
                                 const Mode &mode,
                                 const std::vector<double> &beta,
                                 const double &force_threshold,
                                 const double &hysteresis_low,
                                 const double &hysteresis_high) :
  force_threshold_(force_threshold),
  falling_edge_threshold_(hysteresis_low),
  rising_edge_threshold_(hysteresis_high),
  beta_(beta),
  feet_contact_forces_(feet_contact_forces),
  mode_(mode)
{
}

void StanceEstimator::setParams(const std::vector<double> &beta,
                                const double &force_threshold,
                                const double &hysteresis_low,
                                const double &hysteresis_high,
                                const uint64_t &hysteresis_delay_low,
                                const uint64_t &hysteresis_delay_high)
{
    if(beta.size() == 2) {
        beta_.resize(8);
        for(int i = 0; i < 4; i = i + 2) {
            beta_[i] = beta[0];
            beta_[i + 1] = beta[1];
        }
    } else if(beta.size() == 8) {
        beta_ = beta;
    }

    force_threshold_ = force_threshold;
    falling_edge_threshold_ = hysteresis_low;
    rising_edge_threshold_ = hysteresis_high;
    falling_edge_delay_ = hysteresis_delay_low;
    rising_edge_delay_ = hysteresis_delay_high;

    switch(mode_) {
    case Mode::THRESHOLD:
        std::cout << "[ StanceEst ] Mode: THRESHOLD" << std::endl;
        std::cout << "[ StanceEst ] Force threshold: " << force_threshold_ << std::endl;
        break;
    case Mode::HYSTERESIS:
        std::cout << "[ StanceEst ] Mode: HYSTERESIS" << std::endl;
        std::cout << "[ StanceEst ] Hysteresis low: " << falling_edge_threshold_ << std::endl;
        std::cout << "[ StanceEst ] Hysteresis high: " << rising_edge_threshold_ << std::endl;
        std::cout << "[ StanceEst ] Hysteresis delay low (ns): " << falling_edge_delay_ << std::endl;
        std::cout << "[ StanceEst ] Hysteresis delay high (ns): " << rising_edge_delay_ << std::endl;
        for(size_t i = 0; i < 4; i++){
          force_triggers_[LegID(i)].setParameters(falling_edge_threshold_,
                                                  rising_edge_threshold_,
                                                  falling_edge_delay_,
                                                  rising_edge_delay_);
        }
        break;
    case Mode::REGRESSION:
        std::cout << "[ StanceEst ] Mode: REGRESSION" << std::endl;
        std::cout << "[ StanceEst ] Beta: [";
        for(size_t i = 0; i < beta.size(); i++) {
            std::cout << beta_[i];
            if(i == beta.size() - 1) {
                std::cout << "]" << std::endl;
            } else {
                std::cout << ", ";
            }
        }
        break;
    }
}

void StanceEstimator::updateStat(double sample,
                                 bool is_stance,
                                 int index) {
    if(is_stance) {
        gss[index].n_stance += 1;
        gss[index].delta_stance = sample - gss[index].mu_stance;
        gss[index].mu_stance += gss[index].delta_stance / gss[index].n_stance;
        gss[index].M2_stance += gss[index].delta_stance * (sample - gss[index].mu_stance);
        if(gss[index].n_stance < 2) {
            gss[index].sigma_stance = 50;
        } else {
            gss[index].sigma_stance  = sqrt(gss[index].M2_stance / (gss[index].n_stance - 1));
        }
    } else {
        gss[index].n_swing += 1;
        gss[index].delta_swing = sample - gss[index].mu_swing;
        gss[index].mu_swing += gss[index].delta_swing / gss[index].n_swing;
        gss[index].M2_swing += gss[index].delta_swing * (sample - gss[index].mu_swing);
        if(gss[index].n_swing < 2) {
            gss[index].sigma_stance = 250;
        } else {
            gss[index].sigma_swing  = sqrt(gss[index].M2_swing / (gss[index].n_swing - 1));
        }
    }
}

void StanceEstimator::setJointStates(const uint64_t& nsec,
                                     const JointState &q,
                                     const JointState &qd,
                                     const JointState &tau,
                                     const Quaterniond &orient,
                                     const JointState &qdd,
                                     const Vector3d &xd,
                                     const Vector3d &xdd,
                                     const Vector3d &omega,
                                     const Vector3d &omegad)
{
  nsec_ = nsec;
  setJointStates(q, qd, tau, orient, qdd, xd, xdd, omega, omegad);
}

void StanceEstimator::setJointStates(const JointState &q,
                                     const JointState &qd,
                                     const JointState &tau,
                                     const Quaterniond &orient,
                                     const JointState &qdd,
                                     const Vector3d &xd,
                                     const Vector3d &xdd,
                                     const Vector3d &omega,
                                     const Vector3d &omegad) {
  q_ = q;
  qd_ = qd;
  qdd_ = qdd;
  tau_ = tau;
  orient_ = orient;
  xd_ = xd;
  xdd_ = xdd;
  omega_ = omega;
  omegad_ = omegad;
}

bool StanceEstimator::getStance(LegBoolMap &stance) {
  LegScalarMap stance_probability;
  return getStance(stance, stance_probability);
}

bool StanceEstimator::getStance(LegBoolMap &stance,
                                LegScalarMap &stance_probability)
{
    if(!getGRF(grf_)){
        return false;
    }
    // get the Ground Reaction Forces at the feet, expressed in the base frame
    for(int leg_id  = 0; leg_id < _LEGS_COUNT; leg_id++) {
        grForceDelta[leg_id] = -grf_[leg_id](Z);

        grForceDelta[leg_id] += grf_[leg_id](Z);
        grForce_W[leg_id] = grf_[leg_id]; // FIXME retrieve orientation

        switch(mode_) {
        case Mode::THRESHOLD:
            stance[leg_id] = grf_[leg_id](Z) > force_threshold_ ? true : false;
            stance_probability[leg_id] = stance[leg_id];
            break;
        case Mode::HYSTERESIS:
            force_triggers_[leg_id].updateState(nsec_, grf_[leg_id](Z));
            stance[leg_id] = force_triggers_[leg_id].getState();
            stance_probability[leg_id] = stance[leg_id];
            break;
        case Mode::REGRESSION:
            stance_probability[leg_id] = 1.0 - 1.0 / (1.0 + exp(-(beta_[0 + leg_id * 2] + grf_[leg_id](Z) * beta_[1 + leg_id * 2])));
            stance[leg_id] = (stance_probability[leg_id] > 0.5 ? true : false);
            break;
        }
    }
    return true;
}

void StanceEstimator::setMode(const Mode &mode) {
    mode_ = mode;
}

void StanceEstimator::getNormalizedGRF(Eigen::Vector4d &normgrf) {
    normgrf << grf_[0](Z), grf_[1](Z), grf_[2](Z), grf_[3](Z);
    double max = normgrf.maxCoeff();
    if(max > 200) {
        normgrf /= max;
    } else {
        normgrf = Eigen::Vector4d::Zero();
    }

}

bool StanceEstimator::isStance(LegID leg) const{
    return stance_[leg];
}

LegVectorMap StanceEstimator::getGRF() {
    return grf_;
}

bool StanceEstimator::getGRF(LegVectorMap& grf){
 return feet_contact_forces_.getFeetGRF(q_, qd_, tau_, orient_, grf, qdd_, xd_, xdd_, omega_, omegad_);
}

void StanceEstimator::getGrf_W(LegVectorMap & grf) {
    grf = grForce_W;
}

}  // namespace quadruped
}  // namespace pronto
