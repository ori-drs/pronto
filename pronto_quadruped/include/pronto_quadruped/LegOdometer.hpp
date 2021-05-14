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
#include <pronto_quadruped_commons/leg_vector_map.h>
#include <pronto_quadruped_commons/forward_kinematics.h>
#include <pronto_quadruped_commons/feet_jacobians.h>

// std
#include <memory>

namespace pronto {
namespace quadruped {

/**
 * @brief The LegOdometer class computes the velocity of a floating base
 * legged robot given its joint state, the stance feet, and other data.
 */
class LegOdometer  : public LegOdometerBase {
public:
    template <class T>
    using LegDataMap = pronto::quadruped::LegDataMap<T>;

    enum class SigmaMode {STATIC_SIGMA = 0,/*!< use constant covariance */
                          VAR_SIGMA, /*!< compute covariance from stance legs */
                          IMPACT_SIGMA, /*!< compute covariance from impact information */
                          VAR_AND_IMPACT_SIGMA};

    enum class AverageMode {SIMPLE_AVG,
                            WEIGHTED_AVG}; /*!< velocity is a weighted average on stance
                                                              probability */

public:
    LegOdometer(FeetJacobians &feet_jacobians,
                ForwardKinematics &forward_kinematics,
                bool debug = true,
                SigmaMode s_mode = SigmaMode::STATIC_SIGMA,
                AverageMode a_mode = AverageMode::SIMPLE_AVG);
    virtual ~LegOdometer();


public:
    bool estimatePose(const uint64_t /*utime*/,
                      const JointState& /*q*/,
                      const LegBoolMap& /*stance_legs*/,
                      const LegScalarMap& /*stance_prob*/,
                      Vector3d& /*position*/,
                      Matrix3d& /*pos_covariance*/,
                      Quaterniond& /*orientation*/,
                      Matrix3d& /*orient_covariance*/) override {
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
                          Matrix3d& covariance) override;

    void getVelocity(Vector3d& velocity, Matrix3d& covariance) override
    {
        velocity = xd_b_;
        covariance = vel_cov_;
    }
    void getPosition(Vector3d& /*position*/, Matrix3d& /*covariance*/) override
    {
        std::cerr << "[LegOdometer::getPosition] Function not implemented yet!" << std::endl;
    }
    void getOrientation(Quaterniond& /*orientation*/, Matrix3d& /*covariance*/) override
    {
        std::cerr << "[LegOdometer::getOrientation] Function not implemented yet!" << std::endl;
    }


    // Debugging methods
    void getVelocitiesFromLegs(LegVectorMap & vd) override;
    void getFeetPositions(LegVectorMap & jd) override;
    virtual LegVectorMap getFootPos();


    void setInitVelocityCov(const Matrix3d& vel_cov) override;
    void setInitVelocityStd(const Vector3d& vel_std) override;
    void setInitPositionCov(const Matrix3d& pos_cov) override;
    void setGrf(const LegVectorMap& grf) override;

    // Configuration methods
    virtual void setMode(const SigmaMode s_mode, const AverageMode a_mode);
    virtual void getMode(SigmaMode& s_mode, AverageMode& a_mode);
    virtual std::string printMode();
    void setSpeedLimit(const double& limit) override;

protected:
    FeetJacobians& feet_jacobians_;
    ForwardKinematics& forward_kinematics_;
    bool debug_;

    SigmaMode s_mode_;
    AverageMode a_mode_;
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
    LegVectorMap base_vel_leg_;
    LegVectorMap foot_pos_;

    Eigen::Vector3d xd_b_;  ///< estimated velocity, base frame

    Eigen::Array4d grf_delta_;
    Eigen::Array4d grf_;

    double speed_limit_; // upper limit of the absolute norm of the linear velocity [m/s]
};
} // namespace quadruped
} // namespace pronto
