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

#include "pronto_quadruped/LegOdometer.hpp"
#include <pronto_quadruped_commons/rbd/utils.h>
#include <boost/math/distributions/normal.hpp>


namespace pronto {

using namespace pronto::quadruped;

LegOdometer::LegOdometer(FeetJacobians &feet_jacobians,
                                     ForwardKinematics& forward_kinematics,
                                     bool debug, Mode mode) :
    feet_jacobians_(feet_jacobians),
    forward_kinematics_(forward_kinematics),
    mode_(mode),
    xd_b_(Eigen::Vector3d::Zero()),
    old_xd_b_(Eigen::Vector3d::Zero()),
    debug_(debug) {
}

LegOdometer::~LegOdometer() {
}

void LegOdometer::setMode(const uint8_t mode) {
    std::cout << "[ KSE ] Mode changed to: " << (int) mode << std::endl;
    if(((mode & VAR_SIGMA) | (mode & IMPACT_SIGMA)) == STATIC_SIGMA) {
        std::cout << "[ KSE ] Static sigma" << std::endl;
    }
    if(mode & VAR_SIGMA) {
        std::cout << "[ KSE ] Var sigma" << std::endl;
    }
    if((mode & IMPACT_SIGMA) == IMPACT_SIGMA) {
        std::cout << "[ KSE ] Impact sigma" << std::endl;
    }
    if(mode & WEIGHTED_AVG) {
        std::cout << "[ KSE ] Weighted avg" << std::endl;
    }
    mode_ = mode;
}

void LegOdometer::setInitVelocityCov(const Eigen::Matrix3d& vel_cov){
    initial_vel_cov_ = vel_cov;
}

void LegOdometer::setInitVelocityStd(const Eigen::Vector3d& vel_std){
    initial_vel_std_ = vel_std;
}

void LegOdometer::setInitPositionCov(const Eigen::Matrix3d& pos_cov){
    pos_cov_ = pos_cov;
}

void LegOdometer::getVelocitiesFromLegs(LegVector3Map &vd) {
    vd = base_vel_leg_;
}

void LegOdometer::getFeetPositions(LegVector3Map &jd) {
    jd = foot_pos_;
}

void LegOdometer::setGrfDelta(const LegScalarMap &grf_delta){
    grf_delta_ << grf_delta[0] , grf_delta[1] , grf_delta[2], grf_delta[3];
}

bool LegOdometer::estimateVelocity(const uint64_t utime,
                                   const JointState &q,
                                   const JointState &qd,
                                   const Vector3d &omega,
                                   const LegBoolMap &stance_legs,
                                   const LegScalarMap &stance_prob,
                                   Vector3d &velocity,
                                   Matrix3d &covariance)
{
    vel_cov_ = initial_vel_cov_;

    // Recording foot position and base velocity from legs
    for(int leg = LF; leg <= RH; leg++){
        foot_pos_[LegID(leg)] =  forward_kinematics_.getFootPos(q, LegID(leg));
        base_vel_leg_[LegID(leg)] = - feet_jacobians_.getFootJacobian(q, LegID(leg))
                            * qd.block<3,1>(leg * 3, 0)
                            - omega.cross(foot_pos_[LegID(leg)]);
    }

    Eigen::Vector3d old_xd_b = xd_b_;
    xd_b_ = Eigen::Vector3d::Zero();
    // If we want to perform weighted average over legs depending on the
    // probabilities of contact
    int leg_count = 0;

    Eigen::Vector3d var_velocity = Eigen::Vector3d::Zero();

    if((mode_ & WEIGHTED_AVG)) {
        double sum = 0;
        for(int i = 0; i < 4; i++) {
            if(stance_legs[i]) {
                leg_count++;
                sum += stance_prob[i];
            }
        }

        if(leg_count == 0) {
            return false;
        }

        // Computing weighted average velocity
        for(int i = 0; i < 4; i++) {
            if(stance_legs[i]) {
                xd_b_ += stance_prob[i] * base_vel_leg_[i] / sum;
            }
        }

        for(int i = 0; i < 4; i++) {
            if(stance_legs[i]) {
                // Computing weighted the standard deviation
                for(int j = 0; j < 3; j++) {
                    var_velocity(j) += stance_prob[i] * pow(xd_b_(j) - base_vel_leg_[i](j), 2) / sum;
                }
            }
        }

    } else {
        // Computing average velocity
        for(int i = 0; i < 4; i++) {
            if(stance_legs[i]) {
                xd_b_ += base_vel_leg_[i];
                leg_count++;
            }
        }
        if(leg_count == 0) {
            return false;
        }

        xd_b_ /= (double)leg_count;

        for(int i = 0; i < 4; i++) {
            if(stance_legs[i]) {
                // Computing the standard deviation
                for(int j = 0; j < 3; j++) {
                    var_velocity(j) += pow(xd_b_(j) - base_vel_leg_[i](j), 2);
                }
            }
        }
        var_velocity /= (double)leg_count;
    }



    double alpha = 0.4;
    double beta = 0.3;
    double gamma = 0.8;
    double delta = 0.5;

    if((mode_ & VAR_SIGMA) && !(mode_ & IMPACT_SIGMA)) {
        // Compute the new sigma based on the covariance over stance legs.
        // leave unchanged if only one leg is on the ground!
        if(leg_count != 1) {
            vel_std_ << vel_std_(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) +  (1 - beta) * sqrt(var_velocity(0))),
                  vel_std_(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma) * sqrt(var_velocity(1))),
                  vel_std_(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma) * sqrt(var_velocity(2)));

            var_velocity << vel_std_(0) * vel_std_(0),
                  vel_std_(1) * vel_std_(1),
                  vel_std_(2) * vel_std_(2);
        }
    }

    if((mode_ & IMPACT_SIGMA)) {
        // Featuring Jean-Claude Van Damme, twice
        double impact =  2 * 0.00109375 * abs(grf_delta_.mean());
        if(impact < 0.001 || std::isnan(impact)) {
            impact = 0.0;
            beta = 1;
            gamma = 1;
        }
        if(mode_ & VAR_SIGMA) {
            vel_std_ << vel_std_(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) + (1 - beta) * (delta * impact + (1 - delta) * sqrt(var_velocity(0)))),
                  vel_std_(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma) * (delta * impact + (1 - delta) * sqrt(var_velocity(1)))),
                  vel_std_(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma) * (delta * impact + (1 - delta) * sqrt(var_velocity(2))));

        } else {
            vel_std_ << vel_std_(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) + (1 - beta)* impact),
                  vel_std_(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma)* impact),
                  vel_std_(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma)* impact);

        }

        if((mode_ & WEIGHTED_AVG) && ((old_xd_b - xd_b_)(0) > initial_vel_std_(0) || (old_xd_b - xd_b_)(0) < -initial_vel_std_(0))) {
            old_xd_b = xd_b_;
            return false;
        }

        var_velocity << vel_std_(0) * vel_std_(0),
              vel_std_(1) * vel_std_(1),
              vel_std_(2) * vel_std_(2);
    }
    if(debug_) {
        double impact =  2 * 0.00109375 * abs(grf_delta_.mean());
        if(impact < 0.001 || std::isnan(impact)) {
            impact = 0.0;
            beta = 1;
            gamma = 1;
        }

        r_kse_var_debug << r_kse_var_debug(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) +  (1 - beta) * sqrt(var_velocity(0))),
                        r_kse_var_debug(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma) * sqrt(var_velocity(1))),
                        r_kse_var_debug(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma) * sqrt(var_velocity(2)));

        r_kse_impact_debug << r_kse_impact_debug(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) + (1 - beta)* impact),
                           r_kse_impact_debug(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma)* impact),
                           r_kse_impact_debug(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma)* impact);

        r_kse_var_impact_debug << r_kse_var_impact_debug(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) + (1 - beta) * (delta * impact + (1 - delta) * sqrt(var_velocity(0)))),
                               r_kse_var_impact_debug(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma) * (delta * impact + (1 - delta) * sqrt(var_velocity(1)))),
                               r_kse_var_impact_debug(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma) * (delta * impact + (1 - delta) * sqrt(var_velocity(2))));
    }


    vel_cov_ = var_velocity.asDiagonal();

    // Checks if the computed values are all finite before using them
    if(!xd_b_.allFinite() ){
        return false;
    }

    // Checks if the computed values are all finite before using them
    if(!vel_cov_.allFinite()){
        vel_cov_ = initial_vel_cov_;
    }
    // the old value will stay the same if the just computed one is not finite
    old_xd_b_ = xd_b_;

    velocity = xd_b_;
    covariance = vel_cov_;
    return true;
}

LegOdometer::LegVector3Map LegOdometer::getFootPos() {
    return foot_pos_;
}

}

