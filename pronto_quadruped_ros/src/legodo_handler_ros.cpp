/* Copyright (c) 2018-2019 University of Oxford
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

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include "pronto_quadruped_ros/legodo_handler_ros.hpp"
#include "pronto_quadruped_ros/conversions.hpp"
#include "pronto_quadruped/LegOdometer.hpp"

namespace pronto {
namespace quadruped {

LegodoHandlerBase::LegodoHandlerBase(ros::NodeHandle &nh,
                                     StanceEstimatorBase& stance_est,
                                     LegOdometerBase& legodo) :
    stance_estimator_(stance_est),
    leg_odometer_(legodo)
{
    std::string prefix = "legodo/";

    nh.getParam(prefix + "downsample_factor", (int&)downsample_factor_);
    nh.getParam(prefix + "utime_offset", (int&)utime_offset_);

    double r_vx;
    double r_vy;
    double r_vz;
    if(!nh.getParam(prefix + "r_vx", r_vx)){
        ROS_WARN_STREAM("Could not retrieve r_vx from parameter server."
                        <<  " Setting to default.");
        r_vx = 0.1;
    }
    if(!nh.getParam(prefix + "r_vy", r_vy)){
        ROS_WARN_STREAM("Could not retrieve r_vy from parameter server."
                        <<  " Setting to default.");
        r_vy = 0.1;
    }
    if(!nh.getParam(prefix + "r_vz", r_vz)){
        ROS_WARN_STREAM("Could not retrieve r_vz from parameter server."
                        <<  " Setting to default.");
        r_vz = 0.1;
    }

    r_legodo_init << r_vx, r_vy, r_vz;
    leg_odometer_.setInitVelocityStd(r_legodo_init);

    // not subscribing to IMU messages to get omega and stuff for now
    // if(!nh_.getParam(prefix + "imu_topic", imu_topic_)){
    //     ROS_WARN_STREAM("Couldn't get the IMU topic. Using default: " << imu_topic_);
    // }
    // imu_sub_ = nh_.subscribe(imu_topic_, 100, &LegodoHandlerROS::imuCallback, this);

    std::vector<std::string> leg_names = {"lf", "rf", "lh", "rh"};
    if(debug_){
        for(int i=0; i<4; i++){
            vel_debug_.push_back(nh.advertise<geometry_msgs::TwistStamped>(leg_names[i] + "_veldebug",10));
            grf_debug_.push_back(nh.advertise<geometry_msgs::WrenchStamped>(leg_names[i]+ "_grf", 10));
        }

        vel_raw_ = nh.advertise<geometry_msgs::TwistStamped>("vel_raw", 10);
        stance_pub_ = nh.advertise<pronto_msgs::QuadrupedStance>("stance", 10);
        prior_accel_debug_ = nh.advertise<geometry_msgs::AccelStamped>("prior_accel", 10);
        prior_joint_accel_debug_ = nh.advertise<sensor_msgs::JointState>("prior_joint_accel", 10);
        prior_velocity_debug_ = nh.advertise<geometry_msgs::TwistStamped>("prior_vel", 10);
        vel_sigma_bounds_pub_ = nh.advertise<pronto_msgs::VelocityWithSigmaBounds>("vel_sigma_bounds", 10);

        dl_pose_ = std::make_unique<pronto::DataLogger>("prontopos.txt");
        dl_pose_->setStartFromZero(false);
        dl_vel_ = std::make_unique<pronto::DataLogger>("prontovel.txt");
        dl_vel_->setStartFromZero(false);

        dl_vel_sigma_ = std::make_unique<pronto::DataLogger>("velsigma.txt");
        dl_vel_->setStartFromZero(false);

        wrench_msg_.wrench.torque.x = 0;
        wrench_msg_.wrench.torque.y = 0;
        wrench_msg_.wrench.torque.z = 0;
    }
}

void LegodoHandlerBase::getPreviousState(const StateEstimator *est)
{
    // TODO try not to use the state estimator object
    est->getHeadState(head_state_,head_cov_);

    // take the acceleration, rotational rate and orientation from the current
    // state of the filter
    xd_ = head_state_.velocity();
    xdd_ = head_state_.acceleration() - head_state_.orientation().inverse()*Eigen::Vector3d::UnitZ()*9.80655;  // TODO: Standardise gravitational acceleration

    //std::cerr << xdd_.transpose() << std::endl;
    omega_ = head_state_.angularVelocity();
    omegad_ = Eigen::Vector3d::Zero(); // TODO retrieve angular acceleration

    orientation_ = head_state_.orientation();
    if(debug_){
      double time = ((double)head_state_.utime)*1e-6;
        dl_pose_->addSampleCSV(time, head_state_.position(), orientation_);
        dl_vel_->addSample(time, head_state_.velocity(), head_state_.angularVelocity());

        Matrix3d vel_cov = head_cov_.block<3,3>(RBIS::velocity_ind, RBIS::velocity_ind);
        Matrix3d omega_cov = head_cov_.block<3,3>(RBIS::angular_velocity_ind, RBIS::angular_velocity_ind);

        Vector3d vel_sigma = vel_cov.diagonal().array().sqrt().matrix();
        Vector3d omega_sigma = omega_cov.diagonal().array().sqrt().matrix();

        dl_vel_sigma_->addSample(time, vel_sigma, omega_sigma);
    }
}

LegodoHandlerBase::Update* LegodoHandlerBase::computeVelocity(){
  if(debug_){
      // Publish GRF
      LegVectorMap grf = stance_estimator_.getGRF();
      wrench_msg_.header.stamp = ros::Time().fromNSec(nsec_);
      stance_msg_.header.stamp = wrench_msg_.header.stamp;
      for(int i = 0; i<4; i++){
          wrench_msg_.wrench.force.x = grf[LegID(i)](0);
          wrench_msg_.wrench.force.y = grf[LegID(i)](1);
          wrench_msg_.wrench.force.z = grf[LegID(i)](2);
          grf_debug_[i].publish(wrench_msg_);
      }
      stance_msg_.lf = stance_[LegID::LF] * 0.4;
      stance_msg_.rf = stance_[LegID::RF] * 0.3;
      stance_msg_.lh = stance_[LegID::LH] * 0.2;
      stance_msg_.rh = stance_[LegID::RH] * 0.1;
      stance_pub_.publish(stance_msg_);

      // Publish prior accel
      geometry_msgs::AccelStamped prior_accel_msg;

      prior_accel_msg.header.frame_id = "base";
      prior_accel_msg.header.stamp = ros::Time().fromNSec(nsec_);
      prior_accel_msg.accel.linear.x = xdd_(0);
      prior_accel_msg.accel.linear.y = xdd_(1);
      prior_accel_msg.accel.linear.z = xdd_(2);

      prior_accel_msg.accel.angular.x = omegad_(0);
      prior_accel_msg.accel.angular.y = omegad_(1);
      prior_accel_msg.accel.angular.z = omegad_(2);

      prior_accel_debug_.publish(prior_accel_msg);

      // Publish prior joint acceleration
      sensor_msgs::JointState prior_joint_accel_msg;
      prior_joint_accel_msg.name = {"LF_HAA", "LF_HFE", "LF_KFE",
                                    "RF_HAA", "RF_HFE", "RF_KFE",
                                    "LH_HAA", "LH_HFE", "LH_KFE",
                                    "RH_HAA", "RH_HFE", "RH_KFE"};
      prior_joint_accel_msg.position = std::vector<double>(12, 0.0);
      prior_joint_accel_msg.velocity = std::vector<double>(12, 0.0);
      // qdd_, xd_, xdd_, omega_, omegad_
      prior_joint_accel_msg.effort = {qdd_(0), qdd_( 1), qdd_( 2),
                                      qdd_(3), qdd_( 4), qdd_( 5),
                                      qdd_(6), qdd_( 7), qdd_( 8),
                                      qdd_(9), qdd_(10), qdd_(11)};

      prior_joint_accel_debug_.publish(prior_joint_accel_msg);

      // Publish prior velocity
      geometry_msgs::TwistStamped prior_vel_msg;
      prior_vel_msg.header.frame_id = "base";
      prior_vel_msg.header.stamp = ros::Time().fromNSec(nsec_);
      prior_vel_msg.twist.linear.x = xd_(0);
      prior_vel_msg.twist.linear.y = xd_(1);
      prior_vel_msg.twist.linear.z = xd_(2);

      prior_vel_msg.twist.angular.x = omega_(0);
      prior_vel_msg.twist.angular.y = omega_(1);
      prior_vel_msg.twist.angular.z = omega_(2);
      prior_velocity_debug_.publish(prior_vel_msg);

  }

  omega_ = head_state_.angularVelocity();
  // TODO add support for the dynamic stance estimator

  // get the ground reaction forces from the stance estimator
  stance_estimator_.getGRF(grf_);

  // and pass them over to the leg odometer
  leg_odometer_.setGrf(grf_);

  if(leg_odometer_.estimateVelocity(utime_,
                                    q_,
                                    qd_,
                                    omega_,
                                    stance_,
                                    stance_prob_,
                                    xd_,
                                    cov_legodo))
  {
      if(debug_){
          // get the 1 sigma bound from the diagonal
          r_legodo = cov_legodo.diagonal().array().sqrt().matrix();

          vel_sigma_bound_msg_.header.stamp = ros::Time().fromNSec(nsec_);
          tf::vectorEigenToMsg(r_legodo, vel_sigma_bound_msg_.plus_one_sigma);
          tf::vectorEigenToMsg(xd_ + r_legodo, vel_sigma_bound_msg_.velocity_plus_one_sigma);
          tf::vectorEigenToMsg(xd_ - r_legodo, vel_sigma_bound_msg_.velocity_minus_one_sigma);
          vel_sigma_bounds_pub_.publish(vel_sigma_bound_msg_);
          LegVectorMap veldebug;
          leg_odometer_.getVelocitiesFromLegs(veldebug);
          geometry_msgs::TwistStamped twist;
          twist.header.stamp = ros::Time().fromNSec(nsec_);
          twist.twist.angular.x = 0;
          twist.twist.angular.y = 0;
          twist.twist.angular.z = 0;

          // publish the estimated velocity for each individual leg
          for(int i=0; i<4; i++){
              twist.twist.linear.x = veldebug[LegID(i)](0);
              twist.twist.linear.y = veldebug[LegID(i)](1);
              twist.twist.linear.z = veldebug[LegID(i)](2);
              vel_debug_[i].publish(twist);
          }
          // publish the estimated velocity from the leg odometer
          // before it gets passed to the filter
          twist.twist.linear.x = xd_(0);
          twist.twist.linear.y = xd_(1);
          twist.twist.linear.z = xd_(2);

          vel_raw_.publish(twist);
      }

      return new pronto::RBISIndexedMeasurement(RigidBodyState::velocityInds(),
                                                xd_,
                                                cov_legodo,
                                                Update::legodo,
                                                utime_);
  }
  // std::cerr << "[LegodoHandlerBase::computeVelocity] Something went wrong!" << std::endl;
  return nullptr;
}

LegodoHandlerROS::LegodoHandlerROS(ros::NodeHandle &nh,
                                   StanceEstimatorBase& stance_est,
                                   LegOdometerBase& legodo) :
    LegodoHandlerBase(nh, stance_est, legodo)
{
}

LegodoHandlerROS::Update* LegodoHandlerROS::processMessage(const sensor_msgs::JointState *msg,
                                                           StateEstimator *est)
{
    nsec_ = msg->header.stamp.toNSec(); // save nsecs for later.
    utime_ = nsec_ / 1000;  // A lot of internals still assume microseconds
    // TODO: transition from microseconds to nanoseconds everywhere
    if(!jointStateFromROS(*msg, utime_, q_, qd_, qdd_, tau_)){
      ROS_WARN_STREAM("[LegodoHandlerROS::processMessage] Could not process joint state from ROS!");
      return nullptr;
    }
    getPreviousState(est);

    stance_estimator_.setJointStates(q_, qd_, tau_, orientation_,
                                     // optional arguments starts from here
                                     // note passing previous value for velocity
                                     qdd_, xd_, xdd_, omega_, omegad_);

    stance_estimator_.getStance(stance_, stance_prob_);

    return computeVelocity();
}

bool LegodoHandlerROS::processMessageInit(const sensor_msgs::JointState *msg,
                                          const std::map<std::string, bool> &sensor_initialized,
                                          const RBIS &default_state,
                                          const RBIM &default_cov,
                                          RBIS &init_state,
                                          RBIM &init_cov)
{
    // do nothing for now, we don't expect to initialize with the leg odometry
    return true;
}

FootSensorLegodoHandlerROS::FootSensorLegodoHandlerROS(ros::NodeHandle& nh,
                                                       StanceEstimatorBase& stance_est,
                                                       LegOdometerBase& legodo)
  : LegodoHandlerBase(nh, stance_est, legodo)
{
}

bool FootSensorLegodoHandlerROS::processMessageInit(const sensor_msgs::JointState *msg,
                                                    const std::map<std::string, bool> &sensor_initialized,
                                                    const RBIS &default_state,
                                                    const RBIM &default_cov,
                                                    RBIS &init_state,
                                                    RBIM &init_cov)
{
  return true;
}

LegodoHandlerBase::Update * FootSensorLegodoHandlerROS::processMessage(const sensor_msgs::JointState *msg, StateEstimator *est){
  nsec_ = msg->header.stamp.toNSec(); // save nsecs for later.
  utime_ = nsec_ / 1000;  // A lot of internals still assume microseconds
  // TODO: transition from microseconds to nanoseconds everywhere
  if(!jointStateFromROS(*msg, utime_, q_, qd_, qdd_, tau_)){
    ROS_WARN_STREAM("[FootSensorLegodoHandlerROS::processMessage] Could not process joint state from ROS!");
    return nullptr;
  }
  getPreviousState(est);
  // the data to compute the stance are processed in processSecondaryMessage()
  stance_estimator_.getStance(stance_, stance_prob_);
  return computeVelocity();
}

void FootSensorLegodoHandlerROS::processSecondaryMessage(const pronto_msgs::QuadrupedStance &msg){
  LegBoolMap stance;
  // Boolean: true = in contact.
  // We assume that "0.0" means not-in-contact and >=1.0 is in contact (different legs = different floats)
  // In some places 0.1 is used for "stance" so we want to be fairly small around 0.0 for "not in contact"
  stance[LF] = std::abs(msg.lf) > 0.01;
  stance[RF] = std::abs(msg.rf) > 0.01;
  stance[LH] = std::abs(msg.lh) > 0.01;
  stance[RH] = std::abs(msg.rh) > 0.01;

  stance_estimator_.setStance(stance);
}

ForceSensorLegodoHandlerROS::ForceSensorLegodoHandlerROS(ros::NodeHandle& nh,
                                                         StanceEstimatorBase& stance_est,
                                                         LegOdometerBase& legodo)
  : LegodoHandlerBase(nh, stance_est, legodo)
{
}

LegodoHandlerBase::Update * ForceSensorLegodoHandlerROS::processMessage(const sensor_msgs::JointState *msg, StateEstimator *est){
  nsec_ = msg->header.stamp.toNSec(); // save nsecs for later.
  utime_ = nsec_ / 1000;  // A lot of internals still assume microseconds
  // TODO: transition from microseconds to nanoseconds everywhere
  if(!jointStateFromROS(*msg, utime_, q_, qd_, qdd_, tau_)){
    ROS_WARN_STREAM("[ForceSensorLegodoHandlerROS::processMessage] Could not extract joint states from ROS message!");
    return nullptr;
  }
  getPreviousState(est);
  // the data to compute the stance are processed in processSecondaryMessage()
  stance_estimator_.getStance(stance_, stance_prob_);
  return computeVelocity();
}

bool ForceSensorLegodoHandlerROS::processMessageInit(const sensor_msgs::JointState *msg, const std::map<std::string, bool> &sensor_initialized, const RBIS &default_state, const RBIM &default_cov, RBIS &init_state, RBIM &init_cov){
  return true;
}

void ForceSensorLegodoHandlerROS::processSecondaryMessage(const pronto_msgs::QuadrupedForceTorqueSensors &msg){
  LegVectorMap grf;
  grf[LF] << msg.lf.force.x, msg.lf.force.y, msg.lf.force.z;
  grf[RF] << msg.rf.force.x, msg.rf.force.y, msg.rf.force.z;
  grf[LH] << msg.lh.force.x, msg.lh.force.y, msg.lh.force.z;
  grf[RH] << msg.rh.force.x, msg.rh.force.y, msg.rh.force.z;

  stance_estimator_.setGRF(grf);
}

} // namespace quadruped
} // namespace pronto
