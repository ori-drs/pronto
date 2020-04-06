#pragma once
#include <string>
#include <pronto_core/sensing_module.hpp>
#include <pronto_core/rbis_update_interface.hpp>
#include <pronto_core/definitions.hpp>

#include "pronto_biped_core/biped_forward_kinematics.hpp"
#include "pronto_biped_core/yawlock_common.hpp"

namespace  pronto {
namespace biped {

enum class YawLockMode {YAWBIAS = 0, YAW, YAWBIAS_YAW};

struct YawLockConfig {
    int correction_period;
    bool yaw_slip_detect;
    double yaw_slip_threshold_degrees;
    double yaw_slip_disable_period;
    std::string left_standing_link;
    std::string right_standing_link;
    double r_yaw_bias;
    double r_yaw;
    YawLockMode mode;
};

enum class ControlStatus {
    UNKNOWN = 0,
    STANDING,
    WALKING,
    HARNESSED,
    QUASISTATIC,
    BRACING,
    CRAWLING,
    DUMMY,
    MANIPULATING
};

class YawLockModule : public DualSensingModule<ImuMeasurement, pronto::JointState> {
public:
    // max size in memory will be 2
    typedef Eigen::Matrix<int, Eigen::Dynamic, 1, 0, 2> MeasIndices;
    typedef Eigen::Matrix<double, Eigen::Dynamic,  1, 0, 2> MeasVector;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 2, 2> MeasCovMatrix;
public:
  YawLockModule(BipedForwardKinematics& fk,
                const YawLockConfig& cfg,
                const Transform& ins_to_body);

  RBISUpdateInterface * processMessage(const ImuMeasurement* imu_msg,
                                       StateEstimator* state_estimator) override;

  bool processMessageInit(const ImuMeasurement *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov) override;

  void processSecondaryMessage(const JointState &msg) override;
  void addControlStatus(const ControlStatus& ctrl_msg);
protected:
  YawLock yaw_lock_;
  YawLockMode mode;

  MeasIndices z_indices;
  MeasVector z_meas;
  MeasCovMatrix cov_scan_match;

  Transform ins_to_body_;
  Eigen::Vector3d body_gyro;

  RBIS head_state;
  RBIM head_cov;
  bool yawLockValid = false;
  Eigen::Quaterniond world_to_body_quat_lock;
};
} // namespace biped
} // namespace pronto
