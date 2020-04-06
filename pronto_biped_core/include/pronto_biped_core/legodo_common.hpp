#pragma once
#include <Eigen/Dense>
#include <pronto_core/rbis_update_interface.hpp>

namespace pronto {
namespace biped {

// Typical mode is LIN_RATE
enum class LegOdometryMode { LIN_RATE = 0,
                             ROT_RATE,
                             LIN_AND_ROT_RATE,
                             POSITION_AND_LIN_RATE };

struct LegOdoCommonConfig {
    LegOdometryMode mode_ = LegOdometryMode::LIN_RATE;
    bool verbose = false;
    double R_legodo_xyz_ = 1;
    double R_legodo_vxyz_ = 0.25;
    double R_legodo_vang_ = 0.25;
    double R_legodo_vxyz_uncertain_ = 0.5;
    double R_legodo_vang_uncertain_ = 0.5;
};

class LegOdoCommon {
public:
    using Transform = Eigen::Isometry3d;
public:
  LegOdoCommon(const LegOdoCommonConfig& cfg);

  Transform getTransAsVelocityTrans(const Transform& msgT,
                                    int64_t utime,
                                    int64_t prev_utime) const;

  // Determine the relevent covariance:
  void getCovariance(const LegOdometryMode& mode_current,
                     bool delta_certain,
                     Eigen::MatrixXd &cov_legodo,
                     Eigen::VectorXi &z_indices);

  // Converts the Pelvis Position and Delta Translation into a combined RBIS measurement
  // which is then passed to the estimator
  // odo_position_status is a position boolean validity flag
  // odo_delta_status is a measure 0-1 of the reliability of the delta odometry
  RBISUpdateInterface * createMeasurement(const Transform & odo_positionT,
                                          const Transform & delta_odoT,
                                          const uint64_t& utime,
                                          const uint64_t& prev_utime,
                                          const int& odo_position_status,
                                          const float& odo_delta_status);
private:
  LegOdometryMode mode_;
  bool verbose = false;

  double R_legodo_xyz_;
  double R_legodo_vxyz_;
  double R_legodo_vang_;
  double R_legodo_vxyz_uncertain_;
  double R_legodo_vang_uncertain_;

};
} // namespace biped
} // namespace pronto
