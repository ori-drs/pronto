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
#include <map>
#include <pronto_core/sensing_module.hpp>
#include <pronto_core/definitions.hpp>

namespace pronto {
namespace quadruped {

struct ImuBiasLockConfig {
  double torque_threshold_ = 13;
  double velocity_threshold_ = 0.006;
  double dt_ = 0.0025;
  bool verbose_ = false;
};

class ImuBiasLock : public DualSensingModule<ImuMeasurement,pronto::JointState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    // the measurement and index sizes is 8: 3 for gyro bias, 3 for accel bias
    // and 2 for roll/pitch estimation
    using MeasVector = Eigen::Matrix<double, 8, 1>;
    using IndexVector = Eigen::Matrix<int, 8, 1>;
    using CovMatrix = Eigen::Matrix<double, 8, 8>;
public:
    ImuBiasLock(const Eigen::Isometry3d& ins_to_body_ = Eigen::Isometry3d::Identity(),
                const ImuBiasLockConfig& cfg = ImuBiasLockConfig());
    virtual ~ImuBiasLock() {}

    RBISUpdateInterface* processMessage(const ImuMeasurement *msg,
                                        StateEstimator *est) override;

    bool processMessageInit(const ImuMeasurement *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;

    void processSecondaryMessage(const pronto::JointState& msg) override;
public:
    const Eigen::Vector3d& getCurrentOmega() const {
      return current_omega_;
    }
    const Eigen::Vector3d& getCurrentAccel() const {
      return current_accel_;
    }
    const Eigen::Vector3d& getCurrentCorrectedAccel() const {
      return current_accel_corrected_;
    }

    const Eigen::Vector3d& getCurrentAccelBias() const {
      return accel_bias_;
    }
    const Eigen::Vector3d& getCurrentProperAccelBias() const {
      return proper_accel_bias_;
    }
    const Eigen::Quaterniond& getGVec() const {
      return quat_g_vec;
    }

    const Eigen::Isometry3d& getGravityTransform() const {
      return gravity_transform_;
    }

    const Eigen::Isometry3d& getBiasTransform() const {
      return bias_transform_;
    }

    inline bool getRecordStatus() const {
      return do_record_;
    }

protected:
    bool debug_ = false;
    std::vector<Eigen::Vector3d> gyro_bias_history_;
    std::vector<Eigen::Vector3d> accel_bias_history_;
    bool do_record_ = true;
    bool is_static_ = false;
    size_t max_size = 3000;
    size_t min_size = 500;

    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();

    MeasVector z_meas;
    IndexVector z_indices;
    CovMatrix z_covariance;
    Eigen::Vector3d gravity_vector_;
    Eigen::Vector3d proper_accel_bias_  = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_omega_;
    Eigen::Vector3d current_accel_;
    Eigen::Vector3d current_accel_corrected_;
    Eigen::Vector3d previous_omega_ = Eigen::Vector3d::Zero();
    Eigen::Isometry3d gravity_transform_;
    Eigen::Isometry3d bias_transform_;

    Eigen::Quaterniond quat_g_vec;
    double torque_threshold_ = 13;
    double eps_ = 0.006;
    double dt_ = 0.0025;
    Eigen::Isometry3d ins_to_body_;
protected:
    bool isStatic(const pronto::JointState& state);
    Eigen::Vector3d getBias(const std::vector<Eigen::Vector3d>& history) const;
    Eigen::Matrix3d getBiasCovariance(const std::vector<Eigen::Vector3d>& history) const;
};
}  // namespace quadruped
}  // namespace pronto
