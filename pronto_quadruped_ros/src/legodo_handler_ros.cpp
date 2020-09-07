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

#include "pronto_quadruped_ros/legodo_handler_ros.hpp"
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "pronto_quadruped_ros/conversions.hpp"

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

    double r_kse_vx;
    double r_kse_vy;
    double r_kse_vz;
    nh.getParam(prefix + "r_vx", r_kse_vx);
    nh.getParam(prefix + "r_vy", r_kse_vy);
    nh.getParam(prefix + "r_vz", r_kse_vz);

    r_legodo_init << r_kse_vx, r_kse_vy,r_kse_vz;
    R_legodo = r_legodo_init.array().square().matrix();
    R_legodo_init = R_legodo;
    cov_legodo = R_legodo.asDiagonal();

    leg_odometer_.setInitVelocityCov(cov_legodo);
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
    xdd_ = head_state_.acceleration() - head_state_.orientation().inverse()*Eigen::Vector3d::UnitZ()*9.80655;


    std::cerr << xdd_.transpose() << std::endl;
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
      StanceEstimatorBase::LegVectorMap grf = stance_estimator_.getGRF();
      wrench_msg_.header.stamp = ros::Time().fromNSec(utime_*1e3);
      stance_msg_.header.stamp = wrench_msg_.header.stamp;
      for(int i = 0; i<4; i++){
          wrench_msg_.wrench.force.x = grf[pronto::quadruped::LegID(i)](0);
          wrench_msg_.wrench.force.y = grf[pronto::quadruped::LegID(i)](1);
          wrench_msg_.wrench.force.z = grf[pronto::quadruped::LegID(i)](2);
          grf_debug_[i].publish(wrench_msg_);
      }
      stance_msg_.lf = stance_[pronto::quadruped::LegID::LF] * 0.4;
      stance_msg_.rf = stance_[pronto::quadruped::LegID::RF] * 0.3;
      stance_msg_.lh = stance_[pronto::quadruped::LegID::LH] * 0.2;
      stance_msg_.rh = stance_[pronto::quadruped::LegID::RH] * 0.1;
      stance_pub_.publish(stance_msg_);
  }

  omega_ = head_state_.angularVelocity();
  // TODO add support for the dynamic stance estimator

  if(leg_odometer_.estimateVelocity(utime_,
                                    q_,
                                    qd_,
                                    omega_,
                                    stance_,
                                    stance_prob_,
                                    xd_,
                                    cov_legodo))
  {
      // save the diagonal
      R_legodo = cov_legodo.diagonal();

      if(debug_){
          LegOdometerBase::LegVectorMap veldebug;
          leg_odometer_.getVelocitiesFromLegs(veldebug);
          geometry_msgs::TwistStamped twist;
          twist.header.stamp = ros::Time().fromNSec(utime_*1000);
          twist.twist.angular.x = 0;
          twist.twist.angular.y = 0;
          twist.twist.angular.z = 0;

          // publish the estimated velocity for each individual leg
          for(int i=0; i<4; i++){
              twist.twist.linear.x = veldebug[pronto::quadruped::LegID(i)](0);
              twist.twist.linear.y = veldebug[pronto::quadruped::LegID(i)](1);
              twist.twist.linear.z = veldebug[pronto::quadruped::LegID(i)](2);
              vel_debug_[i].publish(twist);
          }
          // publish the estimated velocity from the leg odometer
          // before it gets passed to the filter
          twist.twist.linear.x = xd_(0);
          twist.twist.linear.y = xd_(1);
          twist.twist.linear.z = xd_(2);

          vel_raw_.publish(twist);
      }
      return new pronto::RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
                                                     xd_,
                                                     cov_legodo,
                                                     Update::legodo,
                                                     utime_);

  }
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
    if(!jointStateFromROS(*msg, utime_, q_, qd_, qdd_, tau_)){
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
  if(!jointStateFromROS(*msg, utime_, q_, qd_, qdd_, tau_)){
    return nullptr;
  }
  getPreviousState(est);
  // the data to compute the stance are processed in processSecondaryMessage()
  stance_estimator_.getStance(stance_, stance_prob_);
  return computeVelocity();
}

void FootSensorLegodoHandlerROS::processSecondaryMessage(const pronto_msgs::QuadrupedStance &msg){
  LegBoolMap stance;
  stance[LF] = msg.lf != 0.0;
  stance[RF] = msg.rf != 0.0;
  stance[LH] = msg.lh != 0.0;
  stance[RH] = msg.rh != 0.0;

  stance_estimator_.setStance(stance);
}

ForceSensorLegodoHandlerROS::ForceSensorLegodoHandlerROS(ros::NodeHandle& nh,
                                                         StanceEstimatorBase& stance_est,
                                                         LegOdometerBase& legodo)
  : LegodoHandlerBase(nh, stance_est, legodo)
{

}

LegodoHandlerBase::Update * ForceSensorLegodoHandlerROS::processMessage(const sensor_msgs::JointState *msg, StateEstimator *est){
  if(!jointStateFromROS(*msg, utime_, q_, qd_, qdd_, tau_)){
    return nullptr;
  }
  getPreviousState(est);
  // the data to compute the stance are processed in processSecondaryMessage()
  stance_estimator_.getStance(stance_, stance_prob_);
}

bool ForceSensorLegodoHandlerROS::processMessageInit(const sensor_msgs::JointState *msg, const std::map<std::string, bool> &sensor_initialized, const RBIS &default_state, const RBIM &default_cov, RBIS &init_state, RBIM &init_cov){
  return true;
}

void ForceSensorLegodoHandlerROS::processSecondaryMessage(const pronto_msgs::QuadrupedForceTorqueSensors &msg){
  LegDataMap<Vector3d> grf;
  grf[LF] << msg.lf.force.x, msg.lf.force.y, msg.lf.force.z;
  grf[RF] << msg.rf.force.x, msg.rf.force.y, msg.rf.force.z;
  grf[LH] << msg.lh.force.x, msg.lh.force.y, msg.lh.force.z;
  grf[RH] << msg.rh.force.x, msg.rh.force.y, msg.rh.force.z;

  stance_estimator_.setGRF(grf);

}

} // namespace quadruped
} // namespace pronto
