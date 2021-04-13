#pragma once
#include "pronto_core/sensing_module.hpp"
#include "pronto_core/definitions.hpp"

namespace pronto {

enum class LidarOdometryMode { POSITION,
                               POSITION_YAW,
                               POSITION_ORIENT };

struct LidarOdometryConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LidarOdometryMode mode;
    Eigen::VectorXi z_indices;
    Eigen::MatrixXd cov_vo;
};


class LidarOdometryModule : public SensingModule<LidarOdometryUpdate> {
public:
    typedef Eigen::Isometry3d Transform;
    typedef Eigen::Quaterniond Quaternion;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:

public:
    LidarOdometryModule(const LidarOdometryConfig& cfg);

    RBISUpdateInterface* processMessage(const LidarOdometryUpdate *msg,
                                        StateEstimator *est) override;

    bool processMessageInit(const LidarOdometryUpdate *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;
    void setCovariance(const Eigen::Matrix3d& covariance);
protected:
    LidarOdometryMode mode_;
    Eigen::VectorXi z_indices;
    Eigen::MatrixXd cov_vo_;

    Transform t0_body_filter_ = Transform::Identity();
    Transform t1_body_vo_ = Transform::Identity();
    Eigen::VectorXd z_meas;
    Quaternion quat;
};


} // namespace
