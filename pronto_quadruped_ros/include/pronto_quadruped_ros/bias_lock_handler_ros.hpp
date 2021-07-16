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
#include <pronto_core/rigidbody.hpp>  // for g_val
#include <pronto_quadruped/ImuBiasLock.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <pronto_ros/pronto_ros_conversions.hpp>
#include <ros/node_handle.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <pronto_msgs/JointStateWithAcceleration.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

namespace pronto {
namespace quadruped {

template <class JointStateT>
class ImuBiasLockBaseROS : public DualSensingModule<sensor_msgs::Imu, JointStateT>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  ImuBiasLockBaseROS() = delete;
  ImuBiasLockBaseROS(ros::NodeHandle& nh);
  RBISUpdateInterface* processMessage(const sensor_msgs::Imu *msg,
                                      StateEstimator *est) override;

  bool processMessageInit(const sensor_msgs::Imu *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov, RBIS &init_state,
                          RBIM &init_cov) override;

protected:
  ros::NodeHandle& nh_;
  std::unique_ptr<quadruped::ImuBiasLock> bias_lock_module_;
  pronto::JointState bias_lock_js_msg_;
  pronto::ImuMeasurement bias_lock_imu_msg_;
  visualization_msgs::Marker imu_arrow_;
  visualization_msgs::Marker base_arrow_;
  visualization_msgs::Marker base_more_arrow_;
  ros::Publisher status_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher base_marker_pub_;
  ros::Publisher base_more_marker_pub_;
  visualization_msgs::MarkerArray frame_markers_;
  visualization_msgs::Marker axis_marker_;
  tf2_ros::TransformBroadcaster broadcaster_;

  bool publish_debug_topics_ = true;
  bool publish_transforms_ = true;
};

template <class JointStateT>
ImuBiasLockBaseROS<JointStateT>::ImuBiasLockBaseROS(ros::NodeHandle& nh) : nh_(nh)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf_imu_to_body_listener_(tfBuffer);

  std::string ins_param_prefix = "ins/";
  std::string lock_param_prefix = "bias_lock/";

  std::string imu_frame = "imu";

  nh_.getParam(ins_param_prefix + "frame", imu_frame);
  std::string base_frame = "base";
  nh.param<std::string>("base_link_name", base_frame, "base");
  ROS_INFO_STREAM("[ImuBiasLockBaseROS] Name of base_link: '" << base_frame << "'");
  Eigen::Isometry3d ins_to_body = Eigen::Isometry3d::Identity();
  while(nh_.ok()) {
    try {
      // lookupTransform API is : target_frame, source_frame
      geometry_msgs::TransformStamped temp_transform = tfBuffer.lookupTransform(base_frame, imu_frame, ros::Time(0));
      tf::transformMsgToEigen(temp_transform.transform, ins_to_body);
      ROS_INFO_STREAM("IMU (" << imu_frame <<") to base (" << base_frame << ") transform: translation=(" << ins_to_body.translation().transpose() << "), rotation=(" << ins_to_body.rotation() << ")");
      break;
    }
    catch (const tf2::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  quadruped::ImuBiasLockConfig cfg;
  nh_.getParam(lock_param_prefix + "torque_threshold", cfg.torque_threshold_);
  nh_.getParam(lock_param_prefix + "velocity_threshold", cfg.velocity_threshold_);
  nh_.param<bool>(lock_param_prefix + "verbose", cfg.verbose_, false);

  if(!nh_.getParam(ins_param_prefix + "timestep_dt", cfg.dt_)){
    ROS_WARN_STREAM("Couldn't read dt. Using default: " << cfg.dt_);
  }

  // Determine what to publish:
  nh_.param<bool>(lock_param_prefix + "publish_debug_topics", publish_debug_topics_, true);  // Default to 'true' to preserve previous behaviour
  nh_.param<bool>(lock_param_prefix + "publish_transforms", publish_transforms_, true);    // Default to 'true' to preserve previous behaviour
  ROS_INFO_STREAM("[ImuBiasLockBaseROS] Publishing debug topics:  " << std::boolalpha << publish_debug_topics_ << "\n" <<
                    "                   Publishing transforms:    " << publish_transforms_);

  if (publish_debug_topics_) {
    status_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/state_estimator_pronto/recording_bias", 100);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/state_estimator_pronto/imu_arrows", 100);
    base_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/state_estimator_pronto/base_arrow", 100);
    base_more_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/state_estimator_pronto/base_more_arrow", 100);
    imu_arrow_.color.a = 1;
    imu_arrow_.color.r = 1;
    imu_arrow_.color.g = 0;
    imu_arrow_.color.b = 0;

    imu_arrow_.scale.x = 0.05;
    imu_arrow_.scale.y = 0.07;
    imu_arrow_.scale.z = 0.1;

    imu_arrow_.type = visualization_msgs::Marker::ARROW;

    imu_arrow_.action = visualization_msgs::Marker::ADD;
    imu_arrow_.header.frame_id = imu_frame;
    imu_arrow_.points.resize(2);
    imu_arrow_.points[0].x = 0;
    imu_arrow_.points[0].y = 0;
    imu_arrow_.points[0].z = 0;

    base_arrow_ = imu_arrow_;
    base_arrow_.header.frame_id = base_frame;
    base_arrow_.color.r = 0;
    base_arrow_.color.g = 1;
  }

  bias_lock_module_.reset(new quadruped::ImuBiasLock(ins_to_body, cfg));
}

template <class JointStateT>
RBISUpdateInterface* ImuBiasLockBaseROS<JointStateT>::processMessage(const sensor_msgs::Imu *msg,
                                                                    StateEstimator *est)
{
  msgToImuMeasurement(*msg, bias_lock_imu_msg_);

  if (publish_debug_topics_) {
    imu_arrow_.header.stamp = msg->header.stamp;

    imu_arrow_.points[1].x = 0.1*msg->linear_acceleration.x;
    imu_arrow_.points[1].y = 0.1*msg->linear_acceleration.y;
    imu_arrow_.points[1].z = 0.1*msg->linear_acceleration.z;

    marker_pub_.publish(imu_arrow_);

    // const Eigen::Vector3d& accel_ = bias_lock_module_->getCurrentAccel();
    // Eigen::Vector3d accel_corrected_ = bias_lock_module_->getCurrentCorrectedAccel();
    base_arrow_.points.resize(2);
    base_arrow_.points[0].x = 0;
    base_arrow_.points[0].y = 0;
    base_arrow_.points[0].z = 0;

    // Gravity vector (note, g_vec is in ENU notation so need to take the negative)
    base_arrow_.points[1].x = 0.1 * -g_vec.x();
    base_arrow_.points[1].y = 0.1 * -g_vec.y();
    base_arrow_.points[1].z = 0.1 * -g_vec.z();

    base_marker_pub_.publish(base_arrow_);

    base_more_arrow_ = base_arrow_;
    const Eigen::Vector3d& bias = bias_lock_module_->getCurrentProperAccelBias();
    base_more_arrow_.points.resize(2);
    base_more_arrow_.points[0].x = 0;
    base_more_arrow_.points[0].y = 0;
    base_more_arrow_.points[0].z = 0;

    base_more_arrow_.points[1].x = 0.1 * bias.x();
    base_more_arrow_.points[1].y = 0.1 * bias.y();
    base_more_arrow_.points[1].z = 0.1 * bias.z();

    base_more_arrow_.color.r = 0;
    base_more_arrow_.color.g = 0;
    base_more_arrow_.color.b = 1;

    base_more_marker_pub_.publish(base_more_arrow_);

    geometry_msgs::PointStamped state_msg;
    state_msg.header.stamp = msg->header.stamp;
    state_msg.point.x = int(bias_lock_module_->getRecordStatus());
    state_msg.point.y = state_msg.point.x;
    state_msg.point.z = state_msg.point.x;
    status_pub_.publish(state_msg);
  }

  if (publish_transforms_) {
    const Eigen::Isometry3d& gravity_transform = bias_lock_module_->getGravityTransform();
    const Eigen::Isometry3d& bias_transform = bias_lock_module_->getBiasTransform();

    geometry_msgs::TransformStamped msg_temp = tf2::eigenToTransform(gravity_transform);
    msg_temp.child_frame_id = "gravity";
    msg_temp.header.frame_id = base_arrow_.header.frame_id;
    msg_temp.header.stamp = msg->header.stamp;

    broadcaster_.sendTransform(msg_temp);

    msg_temp = tf2::eigenToTransform(bias_transform);
    msg_temp.child_frame_id = "bias";
    msg_temp.header.frame_id = base_arrow_.header.frame_id;
    msg_temp.header.stamp = msg->header.stamp;

    broadcaster_.sendTransform(msg_temp);
  }

  return bias_lock_module_->processMessage(&bias_lock_imu_msg_, est);
}

template <class JointStateT>
bool ImuBiasLockBaseROS<JointStateT>::processMessageInit(const sensor_msgs::Imu *msg,
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



class ImuBiasLockROS : public ImuBiasLockBaseROS<sensor_msgs::JointState>
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
};

class ImuBiasLockWithAccelerationROS : public ImuBiasLockBaseROS<pronto_msgs::JointStateWithAcceleration>
{
public:
    ImuBiasLockWithAccelerationROS(ros::NodeHandle& nh) : ImuBiasLockBaseROS<pronto_msgs::JointStateWithAcceleration>(nh) {}
    virtual ~ImuBiasLockWithAccelerationROS() = default;

    void processSecondaryMessage(const pronto_msgs::JointStateWithAcceleration& msg) override;
};

}  // namespace quadruped
}  // namespace pronto
