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

// Eigen
#include <Eigen/Dense>
#include <Eigen/SVD>

// DLS State Estimator
#include "pronto_quadruped/DataLogger.hpp"
#include "pronto_quadruped/LegOdometerBase.hpp"
#include "pronto_quadruped/StanceEstimatorBase.hpp"
#include "pronto_quadruped/FlexEstimator.hpp"

// iit-commons
#include <pronto_quadruped_commons/leg_data_map.h>
#include <pronto_quadruped_commons/forward_kinematics.h>
#include <pronto_quadruped_commons/feet_jacobians.h>

// std
#include <memory>

namespace pronto {
namespace quadruped {

using pronto::quadruped::LF;
using pronto::quadruped::RF;
using pronto::quadruped::LH;
using pronto::quadruped::RH;

/**
 * @brief The LegOdometer class computes the velocity of a floating base
 * legged robot given its joint state, the stance feet, and other data.
 */
class LegOdometer  : public LegOdometerBase {
public:
    template <class T>
    using LegDataMap = pronto::quadruped::LegDataMap<T>;

    typedef typename pronto::quadruped::FeetJacobians FeetJacobians;
    typedef typename pronto::quadruped::ForwardKinematics ForwardKinematics;
    typedef typename pronto::quadruped::JointState JointState;
    typedef typename pronto::quadruped::LegBoolMap LegBoolMap;
    typedef typename pronto::quadruped::LegDataMap<Eigen::Vector3d> LegVector3Map;

    enum Mode {STATIC_SIGMA = 0,/*!< use constant covariance */
               VAR_SIGMA, /*!< compute covariance from stance legs */
               IMPACT_SIGMA, /*!< compute covariance from impact information */
               WEIGHTED_AVG /*!< velocity is a weighted average on stance
                                        probability */
              };

public:
    LegOdometer(FeetJacobians &feet_jacobians,
                ForwardKinematics &forward_kinematics,
                bool debug = true,
                Mode mode = STATIC_SIGMA);
    virtual ~LegOdometer();


public:
    inline bool estimatePose(const uint64_t utime,
                      const JointState& q,
                      const LegBoolMap& stance_legs,
                      const LegScalarMap& stance_prob,
                      Vector3d& position,
                      Matrix3d& pos_covariance,
                      Quaterniond& orientation,
                      Matrix3d& orient_covariance) {
        std::cerr << "Function not implemented yet!" << std::endl;
	return false;
    }

    bool estimateVelocity(const uint64_t utime,
                          const JointState& q,
                          const JointState& qd,
                          const Vector3d& omega,
                          const LegBoolMap& stance_legs,
                          const LegScalarMap& stance_prob,
                          Vector3d& velocity,
                          Matrix3d& covariance);

    inline void getVelocity(Vector3d& velocity, Matrix3d& covariance) override
    {
        velocity = xd_b_;
        covariance = vel_cov_;
    }
    inline void getPosition(Vector3d& position, Matrix3d& covariance){
        std::cerr << "Function not implemented yet!" << std::endl;
    }
    inline void getOrientation(Quaterniond& orientation, Matrix3d& covariance){
        std::cerr << "Function not implemented yet!" << std::endl;
    }

    /**
     * @brief setGrfDelta sets the latest computed Delta between the current
     * and the previous GRF. This is relevant to compute the covariance
     * if the mode IMPACT_SIGMA is used.
     * @param grf_delta
     */
    virtual void setGrfDelta(const LegScalarMap& grf_delta);

    // Debugging methods
    void getVelocitiesFromLegs(LegVector3Map & vd) override;
    void getFeetPositions(LegVector3Map & jd) override;
    virtual LegVector3Map getFootPos();


    void setInitVelocityCov(const Matrix3d& vel_cov) override;
    void setInitVelocityStd(const Vector3d& vel_std) override;
    void setInitPositionCov(const Matrix3d& pos_cov) override;

    // Configuration methods
    virtual void setMode(const uint8_t mode);

protected:
    FeetJacobians& feet_jacobians_;
    ForwardKinematics& forward_kinematics_;
    bool debug_;

    uint8_t mode_;
    Eigen::Vector3d vel_std_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d vel_cov_ = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d pos_cov_ = Eigen::Matrix3d::Zero();

    Eigen::Vector3d initial_vel_std_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d initial_vel_cov_ = Eigen::Matrix3d::Zero();

    Eigen::Vector3d r_kse_var_debug = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_kse_impact_debug = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_kse_var_impact_debug = Eigen::Vector3d::Zero();

    /**
     * @brief base_vel_leg_ this is the base velocity as estimated from the leg
     * kinematics.
     * <b>NOTE</b>: these are NOT the velocity of the legs.
     */
    LegVector3Map base_vel_leg_;
    LegVector3Map foot_pos_;

    Eigen::Vector3d xd_b_; // estimated velocity, base frame
    Eigen::Vector3d old_xd_b_; // previous estimated velocity, base frame

    Eigen::Array4d grf_delta_;
};
} // namespace quadruped
} // namespace pronto
