#pragma once
#include "pronto_core/sensing_module.hpp"
#include "pronto_core/definitions.hpp"

namespace pronto {

enum class VisualOdometryMode {MODE_VELOCITY,
                               MODE_ROTATION_RATE,
                               MODE_VELOCITY_ROTATION_RATE,
                               MODE_POSITION,
                               MODE_POSITION_ORIENT
};

struct VisualOdometryConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VisualOdometryMode mode = VisualOdometryMode::MODE_POSITION;
    Eigen::VectorXi z_indices = RBIS::positionInds();
    Eigen::MatrixXd cov_vo = Eigen::Matrix3d::Identity();
};


class VisualOdometryModule : public SensingModule<VisualOdometryUpdate> {
public:
    typedef Eigen::Isometry3d Transform;
    typedef Eigen::Quaterniond Quaternion;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:

public:
    VisualOdometryModule(const VisualOdometryConfig& cfg);

    RBISUpdateInterface* processMessage(const VisualOdometryUpdate *msg,
                                        StateEstimator *est) override;

    bool processMessageInit(const VisualOdometryUpdate *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;
protected:
    VisualOdometryMode mode_;
    Eigen::VectorXi z_indices;
    Eigen::MatrixXd cov_vo_;

    Transform t0_body_filter_ = Transform::Identity();
    Transform t1_body_vo_ = Transform::Identity();
    Eigen::VectorXd z_meas;
    Quaternion quat;
};


} // namespace
