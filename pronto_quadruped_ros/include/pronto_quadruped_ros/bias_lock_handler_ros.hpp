/* Copyright (c) 2019 University of Oxford
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
#include <memory>
#include <pronto_quadruped/ImuBiasLock.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <pronto_ros/pronto_ros_conversions.hpp>
#include <ros/node_handle.h>

namespace pronto {
namespace quadruped {

class ImuBiasLockROS : public DualSensingModule<sensor_msgs::Imu,sensor_msgs::JointState>
{
public:
    ImuBiasLockROS(ros::NodeHandle& nh);
    virtual ~ImuBiasLockROS() = default;


    RBISUpdateInterface* processMessage(const sensor_msgs::Imu *msg,
                                        StateEstimator *est) override;

    bool processMessageInit(const sensor_msgs::Imu *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov, RBIS &init_state,
                            RBIM &init_cov) override;


    void processSecondaryMessage(const sensor_msgs::JointState& msg) override;

protected:
    std::shared_ptr<ImuBiasLock> bias_lock_module_;
    pronto::JointState bias_lock_js_msg_;
    pronto::ImuMeasurement bias_lock_imu_msg_;
    ros::NodeHandle& nh_;
};

}
}
