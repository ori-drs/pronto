#pragma once
#include "pronto_estimator_core/sensing_module.hpp"
#include "pronto_estimator_core/definitions.hpp"

namespace MavStateEst {

enum class VisualOdometryMode {MODE_VELOCITY,
                               MODE_ROTATION_RATE,
                               MODE_VELOCITY_ROTATION_RATE,
                               MODE_POSITION,
                               MODE_POSITION_ORIENT
};

struct VisualOdometryConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VisualOdometryMode mode;
    Eigen::VectorXi z_indices;
    Eigen::MatrixXd cov_vo;
};


class VisualOdometryModule : public SensingModule<VisualOdometryUpdate> {
public:
    typedef Eigen::Affine3d Transform;
    typedef Eigen::Quaterniond Quaternion;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:

public:
    VisualOdometryModule(const VisualOdometryConfig& cfg);

    RBISUpdateInterface* processMessage(const VisualOdometryUpdate *msg, MavStateEstimator *est);

    bool processMessageInit(const VisualOdometryUpdate *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov);
protected:
    VisualOdometryMode mode_;
    Eigen::VectorXi z_indices;
    Eigen::MatrixXd cov_vo_;
    Transform prev_t0_body_;
    Transform prev_t0_body_internal_;
    uint64_t prev_t0_body_utime_;

    Transform t0_body;
    Transform t0_body_internal;

    updateHistory::historyMapIterator lower_it;
    double diff_utime;
    RBISUpdateInterface * t0_body_RBISInterface;
    RBIS t0_body_RBIS;
    Transform t1_body;
    Transform t0t1_body_vo;
    Transform t1_body_vo;
    Eigen::VectorXd z_meas;
    Quaternion quat;


};


} // namespace
