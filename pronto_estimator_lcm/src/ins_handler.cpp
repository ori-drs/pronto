#include "pronto_estimator_lcm/ins_handler.hpp"
#include <map>
#include "pronto_estimator_lcm/rbis_initializer.hpp"

namespace MavStateEst {

InsHandler::InsHandler()
{

}

InsHandler::~InsHandler(){

}

InsHandler::InsHandler(BotParam * _param, BotFrames * _frames) {

  // mfallon: this chooses between MICROSTRAIN and ATLAS_IMU_BATCH
  channel = bot_param_get_str_or_fail(_param, "state_estimator.ins.channel");

  InsConfig config;

  config.cov_gyro = bot_param_get_double_or_fail(_param, "state_estimator.ins.q_gyro");
  config.cov_accel = bot_param_get_double_or_fail(_param, "state_estimator.ins.q_accel");
  config.cov_gyro_bias = bot_param_get_double_or_fail(_param, "state_estimator.ins.q_gyro_bias");
  config.cov_accel_bias = bot_param_get_double_or_fail(_param, "state_estimator.ins.q_accel_bias");

  config.dt = bot_param_get_double_or_fail(_param, "state_estimator.ins.timestep_dt"); // nominally dt = 0.01 for 100 Hz IMU messages




  char * ins_frame = bot_param_get_str_or_fail(_param, "state_estimator.ins.frame");
  BotTrans bot_ins_to_body_temp;
  bot_frames_get_trans(_frames, ins_frame, "body", &bot_ins_to_body_temp);
  Eigen::Affine3d ins_to_body;
  ins_to_body.setIdentity();
  ins_to_body.translate(Eigen::Map<Eigen::Vector3d>(bot_ins_to_body_temp.trans_vec));
  Eigen::Quaterniond temp_quat;
  eigen_utils::botDoubleToQuaternion(temp_quat,bot_ins_to_body_temp.rot_quat);
  ins_to_body.rotate(temp_quat);
  free(ins_frame);



  config.num_to_init = bot_param_get_int_or_fail(_param, "state_estimator.ins.num_to_init");
  config.max_initial_gyro_bias = bot_param_get_double_or_fail(_param, "state_estimator.ins.max_initial_gyro_bias");


  // Accel Biases
  double accel_bias_initial_array[3];
  bot_param_get_double_array_or_fail(_param, "state_estimator.ins.accel_bias_initial", accel_bias_initial_array, 3);
  config.accel_bias_initial << accel_bias_initial_array[0], accel_bias_initial_array[1], accel_bias_initial_array[2];
  std::cout << "INS setting initial accel bias to: " << config.accel_bias_initial.transpose() << std::endl;

  config.accel_bias_recalc_at_start = bot_param_get_boolean_or_fail(_param, "state_estimator.ins.accel_bias_recalc_at_start");
  std::cout << "INS will " << (config.accel_bias_recalc_at_start ? "" : "NOT ")
            << "recompute initial accel bias at init" << std::endl;

  config.accel_bias_update_online = bot_param_get_boolean_or_fail(_param, "state_estimator.ins.accel_bias_update_online");
  std::cout << "INS will " << (config.accel_bias_update_online ? "" : "NOT ") << "update accel bias online" << std::endl;
  // If we don't want to update the bias, the covariance must be 0
  if(!config.accel_bias_update_online){
      config.cov_accel_bias = 0.0;
  }

  // Gyro Biases
  double gyro_bias_initial_array[3];
  bot_param_get_double_array_or_fail(_param, "state_estimator.ins.gyro_bias_initial", gyro_bias_initial_array, 3);
  config.gyro_bias_initial << gyro_bias_initial_array[0], gyro_bias_initial_array[1], gyro_bias_initial_array[2];
  std::cout << "INS setting initial gyro bias to: " << config.gyro_bias_initial.transpose() << std::endl;


  config.gyro_bias_recalc_at_start = bot_param_get_boolean_or_fail(_param, "state_estimator.ins.gyro_bias_recalc_at_start");
  std::cout << "INS will " << (config.gyro_bias_recalc_at_start ? "" : "NOT ")
            << "recompute initial gyro bias at init" << std::endl;

  config.gyro_bias_update_online = bot_param_get_boolean_or_fail(_param, "state_estimator.ins.gyro_bias_update_online");
  std::cout << "INS will " << (config.gyro_bias_update_online ? "" : "NOT ") << "update gyro bias online" << std::endl;
  // If we don't want to update the bias, the covariance must be 0
  if(!config.gyro_bias_update_online){
      config.cov_gyro_bias = 0.0;
  }
  ins_module_ = InsModule(config, ins_to_body);

}

RBISUpdateInterface * InsHandler::processMessage(const bot_core::ins_t * msg,
                                                 MavStateEstimator* state_estimator)
{
    imu_meas_.acceleration = Eigen::Map<const Eigen::Vector3d>(msg->accel);
    imu_meas_.utime = msg->utime;
    imu_meas_.omega = Eigen::Map<const Eigen::Vector3d>(msg->gyro);
    return ins_module_.processMessage(&imu_meas_, state_estimator);
}

bool InsHandler::processMessageInit(const bot_core::ins_t * msg,
                                    const std::map<std::string, bool> & sensors_initialized,
                                    const RBIS & default_state,
                                    const RBIM & default_cov,
                                    RBIS & init_state, RBIM & init_cov)
{
    imu_meas_.acceleration = Eigen::Map<const Eigen::Vector3d>(msg->accel);
    imu_meas_.utime = msg->utime;
    imu_meas_.omega = Eigen::Map<const Eigen::Vector3d>(msg->gyro);

    return ins_module_.processMessageInit(&imu_meas_,
                                          sensors_initialized,
                                          default_state,
                                          default_cov,
                                          init_state,
                                          init_cov);


}








}
