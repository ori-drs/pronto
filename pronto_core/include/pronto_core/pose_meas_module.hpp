#pragma once

#include "pronto_core/sensing_module.hpp"
#include "pronto_core/definitions.hpp"

namespace pronto {

enum class PoseMeasMode { MODE_POSITION = 0, MODE_POSITION_ORIENT };

struct PoseMeasConfig {
    PoseMeasMode mode;
    int number_of_corrections;
    double r_pose_meas_xyz;
    double r_pose_meas_chi;
};

class PoseMeasModule : SensingModule<PoseMeasurement> {

public:
    // the indices and matrix will be 6 dimensional tops
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 6, 1> MeasVector;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 6> MeasCovMatrix;
    typedef Eigen::Matrix<int, Eigen::Dynamic, 1, 0, 6, 1> MeasIndices;

public:
    PoseMeasModule(const PoseMeasConfig& cfg);

    RBISUpdateInterface* processMessage(const PoseMeasurement *msg,
                                        StateEstimator *est);

    bool processMessageInit(const PoseMeasurement *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov);

protected:
  int no_corrections = 0; // no of corrections to make before going silent
  PoseMeasMode mode = PoseMeasMode::MODE_POSITION;
  MeasIndices z_indices;
  MeasVector z_meas;
  MeasCovMatrix cov_pose_meas;
};
} // namespace pronto

