/* Copyright (c) 2018-2020 University of Oxford
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

#include <ros/node_handle.h>
#include <pronto_core/sensing_module.hpp>

#include <pronto_quadruped/StanceEstimatorBase.hpp>
#include <pronto_quadruped/LegOdometerBase.hpp>
#include <pronto_quadruped/DataLogger.hpp>

#include <pronto_msgs/QuadrupedStance.h>
#include <pronto_msgs/QuadrupedForceTorqueSensors.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>



namespace pronto {
namespace quadruped {

class LegodoHandlerBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    typedef typename pronto::LegOdometerBase LegOdometerBase;
    template <class T>
    using LegDataMap = pronto::quadruped::LegDataMap<T>;
    typedef typename pronto::quadruped::LegBoolMap LegBoolMap;
    using  LegScalarMap = LegDataMap<double>;
    using Update = RBISUpdateInterface;

public:
    LegodoHandlerBase(ros::NodeHandle& nh,
                      StanceEstimatorBase& fcf,
                      LegOdometerBase& fj);

protected:
    StanceEstimatorBase& stance_estimator_;
    LegOdometerBase& leg_odometer_;

    Eigen::Vector3d R_legodo;
    Eigen::Vector3d r_legodo;
    Eigen::Vector3d R_legodo_init;
    Eigen::Vector3d r_legodo_init;
    Eigen::Matrix3d cov_legodo;

    JointState q_;
    JointState qd_;
    JointState qdd_;
    JointState tau_;

    RBIS head_state_;
    RBIM head_cov_;

    Eigen::Vector3d xd_;
    Eigen::Vector3d xdd_;
    Eigen::Vector3d omega_;
    Eigen::Vector3d omegad_;
    Eigen::Quaterniond orientation_;

    LegBoolMap stance_;
    LegScalarMap stance_prob_;

    Eigen::Affine3d imu_to_body_;

    uint64_t utime_;

    uint16_t downsample_factor_;
    uint64_t utime_offset_;

    std::vector<ros::Publisher> vel_debug_;
    std::vector<ros::Publisher> grf_debug_;
    ros::Publisher vel_raw_;
    bool debug_ = true;
    geometry_msgs::WrenchStamped wrench_msg_;
    pronto_msgs::QuadrupedStance stance_msg_;
    ros::Publisher stance_pub_;

    std::unique_ptr<pronto::DataLogger> dl_pose_;
    std::unique_ptr<pronto::DataLogger> dl_vel_;
    std::unique_ptr<pronto::DataLogger> dl_vel_sigma_;
protected:
    virtual Update* computeVelocity();
    virtual void getPreviousState (const StateEstimator *est);
};

class LegodoHandlerROS : public pronto::SensingModule<sensor_msgs::JointState>,
                         public LegodoHandlerBase
{


public:
    LegodoHandlerROS(ros::NodeHandle& nh,
                     StanceEstimatorBase &stance_est,
                     LegOdometerBase &legodo);

    Update * processMessage(const sensor_msgs::JointState *msg, StateEstimator *est) override;

    bool processMessageInit(const sensor_msgs::JointState *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;
};

class ForceSensorLegodoHandlerROS : public LegodoHandlerBase,
                                    public pronto::DualSensingModule<sensor_msgs::JointState,
                                                                     pronto_msgs::QuadrupedForceTorqueSensors>
{
public:
  ForceSensorLegodoHandlerROS(ros::NodeHandle& nh,
                              StanceEstimatorBase& stance_est,
                              LegOdometerBase& legodo);

  Update * processMessage(const sensor_msgs::JointState *msg, StateEstimator *est) override;

  bool processMessageInit(const sensor_msgs::JointState *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov) override;

  void processSecondaryMessage(const pronto_msgs::QuadrupedForceTorqueSensors &msg) override;

};

class FootSensorLegodoHandlerROS : public LegodoHandlerBase,
                                   public pronto::DualSensingModule<sensor_msgs::JointState,
                                                                    pronto_msgs::QuadrupedStance>
{
public:
  FootSensorLegodoHandlerROS(ros::NodeHandle& nh,
                             StanceEstimatorBase& stance_est,
                             LegOdometerBase& legodo);

  Update * processMessage(const sensor_msgs::JointState *msg, StateEstimator *est) override;

  bool processMessageInit(const sensor_msgs::JointState *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov) override;

  void processSecondaryMessage(const pronto_msgs::QuadrupedStance &msg) override;

};


}
}
