/* Copyright (c) 2015-2021
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
#include <iit/rbd/utils.h>
#include <boost/math/distributions/normal.hpp>
#include <sstream>

namespace pronto {

using namespace pronto::quadruped;

LegOdometer::LegOdometer(FeetJacobians &feet_jacobians,
                         ForwardKinematics& forward_kinematics,
                         bool debug, SigmaMode s_mode, AverageMode a_mode) :
    feet_jacobians_(feet_jacobians),
    forward_kinematics_(forward_kinematics),
    debug_(debug),
    s_mode_(s_mode),
    a_mode_(a_mode),
    xd_b_(Eigen::Vector3d::Zero()),
    speed_limit_(12.42) // set speed limit to Usain Bolt's sprint record, Berlin August 16th, 2009
{
}

LegOdometer::~LegOdometer() {
}

void LegOdometer::setMode(const SigmaMode s_mode, const AverageMode a_mode) {
  std::cout << "[ LegOdometer ] Sigma Mode changed to: ";

  switch(s_mode){
  case SigmaMode::STATIC_SIGMA:
    std::cout << "Static Sigma" << std::endl;
    break;
  case SigmaMode::VAR_SIGMA:
    std::cout << "Var Sigma" << std::endl;
    break;
  case SigmaMode::VAR_AND_IMPACT_SIGMA:
    std::cout << "Var and Impact Sigma" << std::endl;
    break;
  case SigmaMode::IMPACT_SIGMA:
    std::cout << "Impact Sigma" << std::endl;
    break;
  default:
    throw std::runtime_error("Unknown SigmaMode");
    break;
  }

  s_mode_ = s_mode;
  std::cout << "[ LegOdometer ] Average Mode changed to: ";
  switch (a_mode) {
  case AverageMode::SIMPLE_AVG:
    std::cout << "Simple Average" << std::endl;
    break;
  case AverageMode::WEIGHTED_AVG:
    std::cout << "Weighted Average" << std::endl;
    break;
  default:
    throw std::runtime_error("Unknown AverageMode");
    break;
  }
  a_mode_ = a_mode;
}

void LegOdometer::getMode(SigmaMode& s_mode, AverageMode& a_mode) {
    s_mode = s_mode_;
    a_mode = a_mode_;
}
std::string LegOdometer::printMode() {
    std::stringstream ss;

    ss << "[ LegOdometer ] Sigma Mode: ";
    switch(s_mode_){
    case SigmaMode::STATIC_SIGMA:
      ss << "Static Sigma" << std::endl;
      break;
    case SigmaMode::VAR_SIGMA:
      ss << "Var Sigma" << std::endl;
      break;
    case SigmaMode::VAR_AND_IMPACT_SIGMA:
      ss << "Var and Impact Sigma" << std::endl;
      break;
    case SigmaMode::IMPACT_SIGMA:
      ss << "Impact Sigma" << std::endl;
      break;
    default:
      break;
    }

    ss << "[ LegOdometer ] Average Mode: ";
    switch (a_mode_) {
    case AverageMode::SIMPLE_AVG:
      ss << "Simple Average" << std::endl;
      break;
    case AverageMode::WEIGHTED_AVG:
      ss << "Weighted Average" << std::endl;
      break;
    default:
      break;
    }
    return ss.str();
}

void LegOdometer::setInitVelocityCov(const Eigen::Matrix3d& vel_cov){
    initial_vel_cov_ = vel_cov;
}

void LegOdometer::setInitVelocityStd(const Eigen::Vector3d& vel_std){
    initial_vel_std_ = vel_std;
    vel_std_ = initial_vel_std_;
    initial_vel_cov_ = vel_std.array().square().matrix().asDiagonal();
    Eigen::IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "Set Initial standard deviation: " << std::endl;
    std::cout << initial_vel_std_.format(clean_fmt) << std::endl;
    std::cout << "Set Initial covariance: " << std::endl;
    std::cout << initial_vel_cov_.format(clean_fmt) << std::endl;
}

void LegOdometer::setInitPositionCov(const Eigen::Matrix3d& pos_cov){
    pos_cov_ = pos_cov;
}

void LegOdometer::getVelocitiesFromLegs(LegVectorMap &vd) {
    vd = base_vel_leg_;
}

void LegOdometer::getFeetPositions(LegVectorMap &jd) {
    jd = foot_pos_;
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
    foot_pos_ = forward_kinematics_.getFeetPos(q);
    for(int leg = LF; leg <= RH; leg++){
        base_vel_leg_[LegID(leg)] = - feet_jacobians_.getFootJacobian(q, LegID(leg))
                            * qd.block<3,1>(leg * 3, 0)
                            - omega.cross(foot_pos_[LegID(leg)]);
    }

    Eigen::Vector3d old_xd_b = xd_b_;
    xd_b_.setZero();
    // If we want to perform weighted average over legs depending on the
    // probabilities of contact
    int leg_count = 0;

    Eigen::Vector3d var_velocity = Eigen::Vector3d::Zero();

    if(a_mode_ == AverageMode::WEIGHTED_AVG) {
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

    } else if(a_mode_ == AverageMode::SIMPLE_AVG){
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
        if(xd_b_.norm() > 10){
          std::cerr << "+++++++++++++++++++++ABNORMAL VELOCITY: " << std::endl;
          Eigen::IOFormat clean(4, 0, ", ", "\n", "[", "]");
          std::cerr << xd_b_.format(clean) << std::endl;
          std::cerr << "Stance: " << stance_legs << std::endl;
          std::cerr << "Leg velocities: " << base_vel_leg_ << std::endl;
        }

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

    if(s_mode_ == SigmaMode::VAR_SIGMA) {
        // Compute the new sigma based on the covariance over stance legs.
        // leave unchanged if only one leg is on the ground!
        if(leg_count != 1) {
            vel_std_ << vel_std_(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) +  (1 - beta) * sqrt(var_velocity(0))),
                  vel_std_(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma) * sqrt(var_velocity(1))),
                  vel_std_(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma) * sqrt(var_velocity(2)));
        }
    }

    if(s_mode_ == SigmaMode::IMPACT_SIGMA || s_mode_ == SigmaMode::VAR_AND_IMPACT_SIGMA) {
        // Featuring Jean-Claude Van Damme, twice
        double impact =  2 * 0.00109375 * grf_delta_.abs().mean();
        if(impact < 0.001 || std::isnan(impact)) {
            impact = 0.0;
            beta = 1;
            gamma = 1;
        }
        if(s_mode_ == SigmaMode::VAR_AND_IMPACT_SIGMA) {
            vel_std_ << vel_std_(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) + (1 - beta) * (delta * impact + (1 - delta) * sqrt(var_velocity(0)))),
                  vel_std_(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma) * (delta * impact + (1 - delta) * sqrt(var_velocity(1)))),
                  vel_std_(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma) * (delta * impact + (1 - delta) * sqrt(var_velocity(2))));

        } else {
            vel_std_ << vel_std_(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) + (1 - beta)* impact),
                  vel_std_(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma)* impact),
                  vel_std_(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma)* impact);
        }

        // if the difference with the previous velocity is beyond the initial
        // value for the standard deviation, reject the measurement
        if((a_mode_ == AverageMode::WEIGHTED_AVG) && ((old_xd_b - xd_b_)(0) > initial_vel_std_(0) || (old_xd_b - xd_b_)(0) < -initial_vel_std_(0))) {
            old_xd_b = xd_b_;
            return false;
        }
    }

    var_velocity << vel_std_(0) * vel_std_(0), vel_std_(1) * vel_std_(1), vel_std_(2) * vel_std_(2);

    if(debug_) {
        double impact =  2 * 0.00109375 * std::abs(grf_delta_.mean());  // TODO: What is the magic number
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
    if(!xd_b_.allFinite()){
      return false;
    }

    // if we hit the speed limit, we reject the measurement
    if(xd_b_.norm() > speed_limit_){
      std::cerr << "Speed limit hit: " << xd_b_.norm() << " > " << speed_limit_ << std::endl;
      return false;
    }

    // Checks if the computed values are all finite before using them
    if(!vel_cov_.allFinite()){
        vel_cov_ = initial_vel_cov_;
    }

    velocity = xd_b_;
    covariance = vel_cov_;
    return true;
}

LegVectorMap LegOdometer::getFootPos() {
    return foot_pos_;
}

void LegOdometer::setSpeedLimit(const double &limit){
  speed_limit_ = limit;
}

void LegOdometer::setGrf(const LegVectorMap &grf){
  Eigen::Array4d prev_grf_ = grf_;
  grf_ << grf[LF](2), grf[RF](2), grf[LH](2), grf[RH](2);
  grf_delta_ = grf_ - prev_grf_;
}

}  // namespace pronto
