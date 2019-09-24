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
#include "pronto_quadruped/ImuBiasLock.hpp"

namespace pronto {
namespace quadruped {

ImuBiasLock::ImuBiasLock(const Eigen::Isometry3d& ins_to_body,
                         const ImuBiasLockConfig& cfg) :
  ins_to_body_(ins_to_body)
{
  // gyro indices
  z_indices.head<3>() = RBIS::gyroBiasInds();
  // accel bias indices
  z_indices.block<3,1>(3,0) = RBIS::accelBiasInds();
  // roll and pitch indices
  z_indices.tail<2>(0) << RBIS::chi_ind, RBIS::chi_ind+1;
  gravity_vector_ = Eigen::Vector3d::UnitZ()*9.80665;
  //9.81207
      //9.80655;

  z_covariance = CovMatrix::Zero();

  eps_ = cfg.velocity_threshold_;
  torque_threshold_ = cfg.torque_threshold_;
}

RBISUpdateInterface* ImuBiasLock::processMessage(const ImuMeasurement *msg,
                                                 StateEstimator *est)
{

  RBIS prior;
  RBIM prior_cov;
  est->getHeadState(prior, prior_cov);

//  RBIS posterior = prior;
//  RBIM posterior_cov = prior_cov;


//  Eigen::Vector3d prior_euler = prior.getEulerAngles();

  current_omega_ = ins_to_body_.rotation()*msg->omega;
  current_accel_ = ins_to_body_.rotation()*msg->acceleration;
  current_accel_corrected_ = current_accel_ - (eigen_utils::skewHat((current_omega_ - previous_omega_) / 0.0025) + eigen_utils::skewHat(current_omega_)*eigen_utils::skewHat(current_omega_))*ins_to_body_.translation();
  //std::cerr << "++++++++++++++++++++++++++++++ " << (- (eigen_utils::skewHat((current_omega_ - previous_omega_) / 0.0025) + eigen_utils::skewHat(current_omega_)*eigen_utils::skewHat(current_omega_))*ins_to_body_.translation()).transpose() << std::endl;
//  Eigen::Vector3d dalla_mia_tesi = -((current_omega_ - previous_omega_) / 0.0025).cross(ins_to_body_.translation())-current_omega_.cross(current_omega_.cross(ins_to_body_.translation()));
  //std::cerr << "+++++++++?????++++++++++++++++ " << dalla_mia_tesi.transpose() << std::endl << std::endl;
  previous_omega_ = current_omega_;
  if(do_record){
    gyro_bias_history_.push_back(current_omega_);
    accel_bias_history_.push_back(current_accel_corrected_);
    if(gyro_bias_history_.size() > max_size){
      std::cout << "Stop recording (size too big)" << std::endl;
      std::cout << gyro_bias_history_.size() << std::endl;
      do_record = false;
    } else {
      return nullptr;
    }
  }

  // if we stop recording and the history is not empty but too short,
  // we just forget it
  if(!do_record && !gyro_bias_history_.empty()) {
    if(gyro_bias_history_.size() < min_size)
    {
      std::cerr << "Cleaning too short history " << gyro_bias_history_.size() << " < " << min_size << std::endl;
      gyro_bias_history_.clear();
      accel_bias_history_.clear();
      return nullptr;
    }

    // if we stop recording and history is not empty or history is too big
    // it's time to compute the bias and free it up


    gyro_bias_ = getBias(gyro_bias_history_);
    accel_bias_ = getBias(accel_bias_history_);


//    proper_accel_bias_ = accel_bias_;

    //the gravity vector points in the negative z axis
    quat_g_vec.setFromTwoVectors(accel_bias_.normalized(), gravity_vector_.normalized());


    accel_bias_ -= prior.orientation().inverse()*gravity_vector_;
    //accel_bias_ = quat_g_vec.inverse()*(quat_g_vec* - gravity_vector_);
//    Eigen::Quaterniond quat_g_vec2;
//    Eigen::Quaterniond quat_g_vec3;
//    quat_g_vec2.setFromTwoVectors(gravity_vector_, accel_bias_);

//    Eigen::Vector3d accel_bias_normalized_ = accel_bias_.normalized();
//    std::cerr << "ACCEL BIAS NORMALIZED: " << accel_bias_normalized_.transpose() << std::endl;

//    Eigen::Isometry3d bias_transform(Eigen::AngleAxisd(prior_euler(2), accel_bias_.normalized()));
//    Eigen::Isometry3d gravity_transform(Eigen::AngleAxisd(prior_euler(2), gravity_vector_.normalized()));

//    gravity_transform_ = gravity_transform;
//    bias_transform_ = bias_transform;

//    quat_g_vec2=Eigen::Quaterniond(((bias_transform)*gravity_transform).rotation());
//    quat_g_vec3=Eigen::Quaterniond((bias_transform.inverse()*gravity_transform).rotation());

    //quat_g_vec2 = quat_g_vec2.inverse();
/*
    Eigen::Vector3d g_vec_rpy = eigen_utils::getEulerAngles(quat_g_vec);
    Eigen::Vector3d g_vec_rpy2 = eigen_utils::getEulerAngles(quat_g_vec2);
    Eigen::Vector3d g_vec_rpy3 = eigen_utils::getEulerAngles(quat_g_vec3);
    std::cerr << std::endl;
    std::cerr << std::endl;
    std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

    std::cerr <<  "Roll, Pitch Initialized from INS: "
               << g_vec_rpy(0) * 180.0 / M_PI << " "
               << g_vec_rpy(1)* 180.0 / M_PI << std::endl;

    std::cerr <<  "Yaw from INS: " <<  g_vec_rpy(2)* 180.0 / M_PI << std::endl;

    std::cerr <<  "Roll, Pitch Inversed: "
               << g_vec_rpy2(0) * 180.0 / M_PI << " "
               << g_vec_rpy2(1)* 180.0 / M_PI << std::endl;

    std::cerr <<  "Yaw INVERSED INS: " <<  g_vec_rpy2(2)* 180.0 / M_PI << std::endl;

    std::cerr <<  "Roll, Pitch Inversed: "
               << g_vec_rpy3(0) * 180.0 / M_PI << " "
               << g_vec_rpy3(1)* 180.0 / M_PI << std::endl;

    std::cerr <<  "Yaw INVERSED INS: " <<  g_vec_rpy3(2)* 180.0 / M_PI << std::endl;
// s

// std::cerr << "Gravity 1?? "  << (quat_g_vec*accel_bias_).transpose() << std::endl;
//gravity_vector_ << 0, 0, (quat_g_vec*accel_bias_).tail<1>();
    std::cerr << "Gyro bias is: " << gyro_bias_.transpose() << std::endl;
    z_meas.block<3,1>(0,0) = gyro_bias_;
    std::cerr << "Accel bias is: " << accel_bias_.transpose() << std::endl;
    z_meas.block<3,1>(3,0) = accel_bias_ - quat_g_vec.inverse()*gravity_vector_;
    accel_bias_ = z_meas.block<3,1>(3,0);
    std::cerr << "True accel bias is: " << z_meas.block<3,1>(3,0).transpose() << std::endl;
    z_meas.block<2,1>(6,0) << g_vec_rpy(0), g_vec_rpy(1);



double banana = 0;
    z_covariance.block<3,3>(0,0) = banana * Eigen::Matrix3d::Identity();//getBiasCovariance(gyro_bias_history_);
    z_covariance.block<3,3>(3,3) = banana * Eigen::Matrix3d::Identity();//getBiasCovariance(accel_bias_history_);
    z_covariance.block<2,2>(6,6) = banana * Eigen::Matrix2d::Identity();//Eigen::Matrix2d::Identity() * std::pow(3 * M_PI / 180.0, 2);
*/
    //std::cerr << "covariance : " << std::endl << z_covariance << std::endl;
    gyro_bias_history_.clear();
    accel_bias_history_.clear();

//    std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    //std::cerr << std::endl;
    //std::cerr << std::endl;


//    Eigen::Vector3d prior_yaw_euler;
//    prior_yaw_euler << 0,0, prior_euler.tail<1>();
//    Eigen::Quaterniond prior_yaw_quaternion = eigen_utils::setQuatEulerAngles(prior_yaw_euler);
//    prior.orientation() = prior.orientation() * quat_g_vec;


    //std::cerr << "Prior orient: " << prior.orientation().w() << " " << prior.orientation().x() << " " << prior.orientation().y() << " " << prior.orientation().z() << std::endl;
    ////std::cerr << prior_euler.transpose() * 180.0 / M_PI << std::endl;
    //
    //posterior.setQuatEulerAngles(post_euler);
    //std::cerr << "Posterior orient: " << posterior.orientation().w() << " " << posterior.orientation().x() << " " << posterior.orientation().y() << " " << posterior.orientation().z() << std::endl;
    //std::cerr << post_euler.transpose() * 180.0 / M_PI << std::endl;
    //posterior.orientation() = prior.orientation();
    prior.accelBias() = accel_bias_;
    prior.gyroBias() = gyro_bias_;
//    gyro_bias_history_.clear();
//    accel_bias_history_.clear();

    return new RBISResetUpdate(prior,prior_cov, RBISUpdateInterface::yawlock, prior.utime);
/*
    return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                     z_meas,
                                                     z_covariance,
                                                     quat_g_vec,
                                                     RBISUpdateInterface::yawlock,
                                                     msg->utime);*/
  }
  return nullptr;
}

bool ImuBiasLock::processMessageInit(const ImuMeasurement *msg,
                                     const std::map<std::string, bool> &sensor_initialized,
                                     const RBIS &default_state,
                                     const RBIM &default_cov,
                                     RBIS &init_state,
                                     RBIM &init_cov)
{
  return true;
}

void ImuBiasLock::processSecondaryMessage(const pronto::JointState &msg){

  is_static = isStatic(msg);


  if(do_record && !is_static){
//    std::cerr << " history is " << gyro_bias_history_.size() << " long" << std::endl;
//    std::cerr << "+++++++++++++++++++ STOP RECORDING" << std::endl;
      do_record = false;
  } else if (!do_record && is_static){
//    std::cerr << "+++++++++++++++++++ START RECORDING" << std::endl;
      do_record = true;
  }
}

bool ImuBiasLock::isStatic(const pronto::JointState &state)
{
  // check if we are in four contact (poor's man version, knee torque threshoold)
  if(state.joint_effort.size() < 12){
//    std::cerr << "++++++++++++++ not enough joints " << state.joint_effort.size() << " < " << torque_threshold_<< std::endl;
    return false;
  }

  if(std::abs(state.joint_effort[2]) < torque_threshold_){
//    std::cerr << "++++++++++++++ not enough torque " << std::abs(state.joint_effort[2]) << " < " << torque_threshold_ << std::endl;
    return false;
  }
  if(std::abs(state.joint_effort[5]) < torque_threshold_){
//    std::cerr << "++++++++++++++ not enough torque " << std::abs(state.joint_effort[5]) << " < " << torque_threshold_ << std::endl;
    return false;
  }
  if(std::abs(state.joint_effort[8]) < torque_threshold_){
//    std::cerr << "++++++++++++++ not enough torque " << std::abs(state.joint_effort[8]) << " < " << torque_threshold_ << std::endl;
    return false;
  }
  if(std::abs(state.joint_effort[11]) < torque_threshold_){
//    std::cerr << "++++++++++++++ not enough torque " << std::abs(state.joint_effort[11]) << " < " << torque_threshold_ << std::endl;
    return false;
  }

  // check that joint velocities are not bigger than eps
  for (auto el : state.joint_velocity){
    if (std::abs(el) > eps_){
//      std::cerr << "++++++++++++++ too much velocity " << std::abs(el) << " > " << eps_ << std::endl;
      return false;
    }
  }
  return true;
}

Eigen::Matrix3d ImuBiasLock::getBiasCovariance(const std::vector<Eigen::Vector3d> &history) const{
  {
      Eigen::Vector3d mean = getBias(history);
      Eigen::Matrix3d covariance(Eigen::Matrix3d::Zero());

      for( auto el : history){
          covariance += (el - mean) * (el - mean).transpose();
      }

      return covariance / ((double)(history.size() - 1));
  }
}

Eigen::Vector3d ImuBiasLock::getBias(const std::vector<Eigen::Vector3d> &history) const
{
       Eigen::Vector3d bias(Eigen::Vector3d::Zero());
       for(auto& el : history) {
           bias += el;
       }
       return bias / ((double)history.size());
}



}
}


