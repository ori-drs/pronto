#pragma once
#include "pronto_core/definitions.hpp"
#include "pronto_core/sensing_module.hpp"
namespace pronto {

struct InsConfig {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    double cov_accel = 0;
    double cov_gyro = 0;
    double cov_accel_bias = 0;
    double cov_gyro_bias = 0;

    double dt = 0.025;

    //initialization
    int num_to_init = 100;
    double max_initial_gyro_bias = 0.015;

    bool gyro_bias_update_online = true;
    Eigen::Vector3d gyro_bias_initial = Eigen::Vector3d::Zero();
    bool gyro_bias_recalc_at_start = true;

    bool accel_bias_update_online = true;
    Eigen::Vector3d accel_bias_initial = Eigen::Vector3d::Zero();
    bool accel_bias_recalc_at_start = true;

};


class InsModule : public SensingModule<ImuMeasurement> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
        InsModule();
    InsModule(const InsConfig& config, const Eigen::Affine3d& ins_to_body);
    ~InsModule();

  RBISUpdateInterface * processMessage(const ImuMeasurement *msg,
                                       StateEstimator* state_estimator) override;

  bool processMessageInit(const ImuMeasurement * msg,
                          const std::map<std::string, bool> & sensors_initialized,
                          const RBIS & default_state,
                          const RBIM & default_cov,
                          RBIS & init_state,
                          RBIM & init_cov) override;
  inline double getTimeStep() {
      return dt;
  }
  inline void setTimeStep(const double& dt){
      this->dt = dt;
  }
protected:
  bool allInitializedExcept(const std::map<std::string, bool> & _sensors_initialized,
                            const std::string & sensor_prefix);

  // Common Initialization Function:
  bool processMessageInitCommon(const std::map<std::string, bool> & sensors_initialized,
                                const RBIS & default_state,
                                const RBIM & default_cov,
                                RBIS & init_state,
                                RBIM & init_cov,
                                RBISIMUProcessStep * update,
                                const Eigen::Vector3d& mag_vec);



  Eigen::Affine3d ins_to_body_;

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

  Eigen::Vector3d current_omega_;
  bool gyro_bias_update_online;
  Eigen::Vector3d gyro_bias_initial;
  bool gyro_bias_recalc_at_start;

  bool accel_bias_update_online;
  Eigen::Vector3d accel_bias_initial;
  bool accel_bias_recalc_at_start;
};
}
