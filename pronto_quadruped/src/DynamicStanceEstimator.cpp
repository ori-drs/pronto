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

#include "pronto_quadruped/DynamicStanceEstimator.hpp"
#include <pronto_quadruped_commons/geometry/rotations.h>
#include <pronto_quadruped_commons/joint_id_tricks.h>
#include <pronto_quadruped_commons/forward_kinematics.h>
#include <pronto_quadruped_commons/geometry/algebra.h>
#include <bitset>

using namespace iit;
using namespace pronto;
using namespace pronto::commons;
using namespace pronto::quadruped;

namespace pronto {
DynamicStanceEstimator::DynamicStanceEstimator(InverseDynamics &inverse_dynamics,
        JSIM &jsim,
        FeetContactForces &feet_contact_forces,
        ForwardKinematics &forward_kinematics) :
  StanceEstimator(feet_contact_forces),
      forward_kinematics_(forward_kinematics),
    dynamics_violation_(std::vector<Wrench>(15)),
    wrench_legs(std::vector<Wrench>(15)),
    inverse_dynamics_(inverse_dynamics),
    jsim_(jsim)
 {
}

bool DynamicStanceEstimator::getStance(LegBoolMap &stance,
                                       LegScalarMap &stance_probability) {
    Eigen::Matrix<double, Eigen::Dynamic, 6, 0, 12, 6> Jcb;
    // Vector of GRFs
    Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 12, 1> grf_legs;

    wrench_base = getBaseWrench(q_, qd_, orient_, qdd_, xd_, xdd_, omega_, omegad_);
    grf_ = feet_contact_forces_.getFeetGRF(q_, qd_, tau_, orient_, qdd_, xd_, xdd_, omega_, omegad_);

    std::vector<std::pair<double,unsigned char> > items(15);

    for(unsigned char leg_conf = 1; leg_conf < 16; leg_conf++) {
        LegBoolMap my_stance;

        my_stance[LF] = leg_conf & 0b00001000; // the most significant bit is LF
        my_stance[RF] = leg_conf & 0b00000100;
        my_stance[LH] = leg_conf & 0b00000010;
        my_stance[RH] = leg_conf & 0b00000001;

        int cleg_count = 0;
        for (int i = 0; i < quadruped::_LEGS_COUNT; i++) {
            if (my_stance[quadruped::LegID(i)]) {
                cleg_count++;
            }
        }
        // Resizing the dynamic matrices depending on the number of contact legs
        Jcb.resize(cleg_count * 3, 6);
        grf_legs.resize(cleg_count * 3, 1);

        // resetting the number of contact legs to access the data for every
        // contact leg
        cleg_count = 0;

        for (int i = 0; i < quadruped::_LEGS_COUNT; i++) {
            // if leg is constrained then leg is in stance,
            // ie no need to check all eff_dof

            if (my_stance[quadruped::LegID(i)]) {
                //base contribution to
                //need to map from base to world frame cause feet is in base frame
                Jcb.block(cleg_count * 3, rbd::AX, 3, 3) = -commons::skew_sim(forward_kinematics_.getFootPos(q_, quadruped::LegID(i)));
                //base linear velocity is assumed in the base frame
                Jcb.block(cleg_count * 3, rbd::LX, 3, 3) = Eigen::Matrix3d::Identity();
                grf_legs.block<3, 1>(3 * cleg_count, 0) = grf_[LegID(i)];
                cleg_count++;
            }
        }
        wrench_legs[leg_conf-1] = Jcb.transpose() * grf_legs;

        dynamics_violation_[leg_conf-1] = wrench_base - wrench_legs[leg_conf-1];

        double peso = std::bitset<4>(leg_conf).count()*50;



        items[leg_conf-1] = std::pair<double, unsigned char>(peso+rbd::linearPart(dynamics_violation_[leg_conf-1]).norm(),leg_conf);

    }

    std::sort(items.begin(),items.end());

    //std::cout << "----------" << std::endl;
    //for(int i=0; i<items.size();i++){
    //    std::cout << std::bitset<4>(items[i].second) << ": " << items[i].first << std::endl;
    //}
    //std::cout << "----------" << std::endl;

    stance[LF] = items[0].second & 0b00001000;
    stance[RF] = items[0].second & 0b00000100;
    stance[LH] = items[0].second & 0b00000010;
    stance[RH] = items[0].second & 0b00000001;

    //std::cout << stance << std::endl;

    stance_probability[LF] = (double)stance[LF]*(1.0 - items[0].first/1000.0);
    stance_probability[RF] = (double)stance[RF]*(1.0 - items[0].first/1000.0);
    stance_probability[LH] = (double)stance[LH]*(1.0 - items[0].first/1000.0);
    stance_probability[RH] = (double)stance[RH]*(1.0 - items[0].first/1000.0);

    stance_probability[LF] = (double)stance[LF];
    stance_probability[RF] = (double)stance[RF];
    stance_probability[LH] = (double)stance[LH];
    stance_probability[RH] = (double)stance[RH];

    return true;
}

std::vector<Wrench> DynamicStanceEstimator::getDynamicsViolation() {
    return dynamics_violation_;
}

std::vector<Wrench> DynamicStanceEstimator::getGRFnormLegs() {
    return wrench_legs;
}

Wrench DynamicStanceEstimator::getGRFnormBase() {
    return wrench_base;
}

Wrench DynamicStanceEstimator::getBaseWrench(const JointState &q,
                                             const JointState &qd,
                                             const Quaterniond &orient,
                                             const JointState &qdd,
                                             const Vector3d &xd,
                                             const Vector3d &xdd,
                                             const Vector3d &omega,
                                             const Vector3d &omegad)
{
    jsim_(q);

    rbd::Vector6D gravity_world = rbd::Vector6D::Zero();
    gravity_world(5) = -rbd::g;
    rbd::Vector6D gravity_base = rbd::Vector6D::Zero();
    rbd::Vector6D base_acceleration = rbd::Vector6D::Zero();
    rbd::VelocityVector base_twist = rbd::Vector6D::Zero();

    Eigen::Matrix3d R = pronto::commons::quatToRotMat(orient);

    gravity_base.segment(rbd::LX, 3) = R * gravity_world.segment(rbd::LX, 3);

    base_acceleration.segment(rbd::LX, 3) = xdd + gravity_base.segment(rbd::LX, 3);
    base_acceleration.segment(rbd::AX, 3) = omegad;

    base_twist.segment(rbd::AX, 3) = omega;
    base_twist.segment(rbd::LX, 3) = xd;

    rbd::ForceVector h_base;
    JointState  h_joints;
    quadruped::LinkDataMap<rbd::ForceVector> zero_ext_forces(rbd::ForceVector::Zero());

    // If we set the accelerations to zero, we basically compute the h_base
    // and h_joints components of the dynamics equation
    inverse_dynamics_.id_fully_actuated(h_base,
                                        h_joints,
                                        gravity_base,
                                        base_twist,
                                        rbd::Vector6D::Zero(),
                                        q,
                                        qd,
                                        JointState::Zero(),
                                        zero_ext_forces);
    // Vector of GRFs
    Wrench w;
    w << ((jsim_.getWholeBodyInertia()) * (base_acceleration) + jsim_.getF()*qdd + h_base);

    return w;
}

}


