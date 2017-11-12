#ifndef __SENSOR_HANDLERS_HPP__
#define __SENSOR_HANDLERS_HPP__

#include "rbis_update_interface.hpp"
#include "lcm_front_end.hpp"
#include "rbis_initializer.hpp"

#include <eigen_utils/eigen_utils.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/ins_t.hpp>
#include <lcmtypes/bot_core/gps_data_t.hpp>
#include <lcmtypes/pronto/optical_flow_t.hpp>
#include <lcmtypes/pronto/indexed_measurement_t.hpp>
#include <lcmtypes/bot_core/kvh_raw_imu_batch_t.hpp>

#include <estimate_tools/imu_stream.hpp>
#include <estimate_tools/iir_notch.hpp>

namespace MavStateEst {

class InsHandler {
protected:

  void create(BotParam * _param, BotFrames * _frames);
  public:
  InsHandler(BotParam * _param, BotFrames * _frames);

  // channel is used to determine which signal to subscribe to:
  std::string channel;

  // Microstrain Functions:
  RBISUpdateInterface * processMessage(const bot_core::ins_t * msg, MavStateEstimator* state_estimator);
  bool processMessageInit(const bot_core::ins_t * msg, const std::map<std::string, bool> & sensors_initialized
      , const RBIS & default_state, const RBIM & default_cov, RBIS & init_state, RBIM & init_cov);

  // Compariable Atlas Functions:
  RBISUpdateInterface * processMessageAtlas(const bot_core::kvh_raw_imu_batch_t * msg, MavStateEstimator* state_estimator);
  bool processMessageInitAtlas(const bot_core::kvh_raw_imu_batch_t * msg, const std::map<std::string, bool> & sensors_initialized
      , const RBIS & default_state, const RBIM & default_cov, RBIS & init_state, RBIM & init_cov);
  //////////// Members Particular to Atlas:
  double prev_utime_atlas; // cached previous time
  IMUStream imu_data_;
  void doFilter(IMUPacket &raw);
  // An cascade of 3 notch filters in xyz
  IIRNotch* notchfilter_x[3];
  IIRNotch* notchfilter_y[3];
  IIRNotch* notchfilter_z[3];
  bool atlas_filter;
  ////////////

  // Common Initialization Function:
  bool processMessageInitCommon(const std::map<std::string, bool> & sensors_initialized
      , const RBIS & default_state, const RBIM & default_cov,
      RBIS & init_state, RBIM & init_cov,
      RBISIMUProcessStep * update, Eigen::Vector3d mag_vec);


  BotTrans ins_to_body;

  double cov_accel;
  double cov_gyro;
  double cov_accel_bias;
  double cov_gyro_bias;

  double dt;

  //initialization
  int num_to_init;
  double max_initial_gyro_bias;
  int init_counter;

  Eigen::Vector3d g_vec_sum;
  Eigen::Vector3d mag_vec_sum;
  Eigen::Vector3d gyro_bias_sum;

  ////////////////////////
  bool gyro_bias_update_online;
  Eigen::Vector3d gyro_bias_initial;
  bool gyro_bias_recalc_at_start;

  bool accel_bias_update_online;
  Eigen::Vector3d accel_bias_initial;
  bool accel_bias_recalc_at_start;

};


class GpsHandler {
protected:
  public:
  GpsHandler(BotParam * _param);
  RBISUpdateInterface * processMessage(const bot_core::gps_data_t * msg, MavStateEstimator* state_estimator);
  bool processMessageInit(const bot_core::gps_data_t * msg, const std::map<std::string, bool> & sensors_initialized
      , const RBIS & default_state, const RBIM & default_cov,
      RBIS & init_state, RBIM & init_cov);

  Eigen::Matrix3d cov_xyz;
};

class ViconHandler {
public:
  typedef enum {
    MODE_POSITION, MODE_POSITION_ORIENT, MODE_ORIENTATION, MODE_YAW
  } ViconMode;

  ViconHandler(BotParam * param, BotFrames *frames);
  ViconHandler(BotParam * param, ViconMode vicon_mode);
  void init(BotParam * param);
  RBISUpdateInterface * processMessage(const bot_core::rigid_transform_t * msg, MavStateEstimator* state_estimator);
  bool processMessageInit(const bot_core::rigid_transform_t * msg, const std::map<std::string, bool> & sensors_initialized
        , const RBIS & default_state, const RBIM & default_cov,
        RBIS & init_state, RBIM & init_cov);


  // added mfallon, to allow a plate-to-body transform
  BotTrans body_to_vicon;
  bool apply_frame;

  ViconMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_vicon;
};


class IndexedMeasurementHandler {
public:
  IndexedMeasurementHandler(RBISUpdateInterface::sensor_enum this_sensor)
  {
      indexed_sensor = this_sensor;
  }
  RBISUpdateInterface * processMessage(const pronto::indexed_measurement_t * msg, MavStateEstimator* state_estimator);
  bool processMessageInit(const pronto::indexed_measurement_t * msg, const std::map<std::string, bool> & sensors_initialized
        , const RBIS & default_state, const RBIM & default_cov,
        RBIS & init_state, RBIM & init_cov);

private:
    RBISUpdateInterface::sensor_enum indexed_sensor;

};

class ScanMatcherHandler {
public:
  typedef enum {
    MODE_POSITION, MODE_POSITION_YAW, MODE_VELOCITY, MODE_VELOCITY_YAW, MODE_YAW
  } ScanMatchingMode;

  ScanMatcherHandler(BotParam * param);
  RBISUpdateInterface * processMessage(const bot_core::pose_t * msg, MavStateEstimator* state_estimator);

  ScanMatchingMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_scan_match;
};

class OpticalFlowHandler {
public:
  OpticalFlowHandler(BotParam * param, BotFrames * frames);
  RBISUpdateInterface * processMessage(const pronto::optical_flow_t * msg, MavStateEstimator* state_estimator);

  BotTrans body_to_cam;
  Eigen::Vector3d body_to_cam_trans; // In body frame, not camera frame
  Eigen::Matrix3d body_to_cam_rot;

  Eigen::Vector4d z_xyrs; // data storage vector
  Eigen::Matrix4d cov_xyrs; // x, y, rot, scale
};

}//namespace

#endif /* __SENSOR_HANDLERS_HPP__ */
