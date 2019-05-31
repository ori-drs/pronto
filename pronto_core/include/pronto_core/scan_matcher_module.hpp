#pragma once
#include <Eigen/Dense>
#include "pronto_core/rbis_update_interface.hpp"
#include "pronto_core/definitions.hpp"
#include "pronto_core/sensing_module.hpp"
#include "pronto_core/state_est.hpp"

namespace pronto {

//enum ScanMatchingMode {
//    MODE_POSITION, MODE_POSITION_YAW, MODE_VELOCITY, MODE_VELOCITY_YAW, MODE_YAW
//};

class ScanMatcherModule : public SensingModule<PoseMeasurement> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
        enum ScanMatchingMode {
            MODE_POSITION, MODE_POSITION_YAW, MODE_VELOCITY, MODE_VELOCITY_YAW, MODE_YAW
        };
    ScanMatcherModule();

  ScanMatcherModule(const ScanMatchingMode& mode,
                    const Eigen::VectorXi& z_indices,
                    const Eigen::MatrixXd& cov_scan_match);

  RBISUpdateInterface * processMessage(const PoseMeasurement * msg,
                                       StateEstimator* state_estimator) override;

  bool processMessageInit(const PoseMeasurement *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state, RBIM &init_cov) override;
protected:
    ScanMatchingMode mode;
    Eigen::VectorXi z_indices;
    Eigen::MatrixXd cov_scan_match;
};

}
