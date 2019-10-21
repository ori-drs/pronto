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

#include "pronto_quadruped_ros/bias_lock_handler_ros.hpp"

using namespace pronto;
using namespace pronto::quadruped;

ImuBiasLockROS::ImuBiasLockROS(ros::NodeHandle& nh) : nh_(nh)
{
  ImuBiasLockConfig cfg;

  if(!nh_.getParam("torque_threshold", cfg.torque_threshold_)){
    ROS_WARN("Param torque treshold not found setting to default");
  }
  if(!nh_.getParam("velocity_threshold", cfg.velocity_threshold_)){
    ROS_WARN("Param velocity treshold not found setting to default");
  }

  bias_lock_module_ = std::make_shared<ImuBiasLock>(Eigen::Isometry3d::Identity(),cfg);

}

RBISUpdateInterface* ImuBiasLockROS::processMessage(const sensor_msgs::Imu *msg,
                                                    StateEstimator *est)
{
  msgToImuMeasurement(*msg, bias_lock_imu_msg_);
  return bias_lock_module_->processMessage(&bias_lock_imu_msg_, est);
}

bool ImuBiasLockROS::processMessageInit(const sensor_msgs::Imu *msg,
                                        const std::map<std::string, bool> &sensor_initialized,
                                        const RBIS &default_state,
                                        const RBIM &default_cov,
                                        RBIS &init_state,
                                        RBIM &init_cov)
{
  msgToImuMeasurement(*msg, bias_lock_imu_msg_);
  return bias_lock_module_->processMessageInit(&bias_lock_imu_msg_,
                                               sensor_initialized,
                                               default_state,
                                               default_cov,
                                               init_state,
                                               init_cov);
}

void ImuBiasLockROS::processSecondaryMessage(const sensor_msgs::JointState &msg) {
  jointStateFromROS(msg, bias_lock_js_msg_);
  bias_lock_module_->processSecondaryMessage(bias_lock_js_msg_);
}
