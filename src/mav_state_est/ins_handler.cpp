#include "mav_state_est/ins_handler.hpp"
#include <map>
#include "mav_state_est/rbis_initializer.hpp"

namespace MavStateEst {

InsHandler::InsHandler(BotParam * _param, BotFrames * _frames) :
    accel_bias_update_online(true),
    gyro_bias_update_online(true),
    accel_bias_initial(Eigen::Vector3d::Zero()),
    gyro_bias_initial(Eigen::Vector3d::Zero()),
    accel_bias_recalc_at_start(true),
    gyro_bias_recalc_at_start(true){

  // mfallon: this chooses between MICROSTRAIN and ATLAS_IMU_BATCH
  channel = bot_param_get_str_or_fail(_param, "state_estimator.ins.channel");

  cov_gyro = bot_param_get_double_or_fail(_param, "state_estimator.ins.q_gyro");
  cov_gyro = bot_sq(bot_to_radians(cov_gyro));
  cov_accel = bot_param_get_double_or_fail(_param, "state_estimator.ins.q_accel");
  cov_accel = bot_sq(cov_accel);
  cov_gyro_bias = bot_param_get_double_or_fail(_param, "state_estimator.ins.q_gyro_bias");
  cov_gyro_bias = bot_sq(bot_to_radians(cov_gyro_bias));
  cov_accel_bias = bot_param_get_double_or_fail(_param, "state_estimator.ins.q_accel_bias");
  cov_accel_bias = bot_sq(cov_accel_bias);

  dt = bot_param_get_double_or_fail(_param, "state_estimator.ins.timestep_dt"); // nominally dt = 0.01 for 100 Hz IMU messages




  char * ins_frame = bot_param_get_str_or_fail(_param, "state_estimator.ins.frame");
  bot_frames_get_trans(_frames, ins_frame, "body", &ins_to_body);
  free(ins_frame);

  num_to_init = bot_param_get_int_or_fail(_param, "state_estimator.ins.num_to_init");
  max_initial_gyro_bias = bot_param_get_double_or_fail(_param, "state_estimator.ins.max_initial_gyro_bias");
  init_counter = 0;
  g_vec_sum.setZero();
  mag_vec_sum.setZero();
  gyro_bias_sum.setZero();


  // Accel Biases
  double accel_bias_initial_array[3];
  bot_param_get_double_array_or_fail(_param, "state_estimator.ins.accel_bias_initial", accel_bias_initial_array, 3);
  accel_bias_initial << accel_bias_initial_array[0], accel_bias_initial_array[1], accel_bias_initial_array[2];
  std::cout << "INS setting initial accel bias to: " << accel_bias_initial.transpose() << std::endl;

  accel_bias_recalc_at_start = bot_param_get_boolean_or_fail(_param, "state_estimator.ins.accel_bias_recalc_at_start");
  std::cout << "INS will " << (accel_bias_recalc_at_start ? "" : "NOT ")
            << "recompute initial accel bias at init" << std::endl;

  accel_bias_update_online = bot_param_get_boolean_or_fail(_param, "state_estimator.ins.accel_bias_update_online");
  std::cout << "INS will " << (accel_bias_update_online ? "" : "NOT ") << "update accel bias online" << std::endl;
  // If we don't want to update the bias, the covariance must be 0
  if(!accel_bias_update_online){
      cov_accel_bias = 0.0;
  }

  // Gyro Biases
  double gyro_bias_initial_array[3];
  bot_param_get_double_array_or_fail(_param, "state_estimator.ins.gyro_bias_initial", gyro_bias_initial_array, 3);
  gyro_bias_initial << gyro_bias_initial_array[0], gyro_bias_initial_array[1], gyro_bias_initial_array[2];
  std::cout << "INS setting initial gyro bias to: " << gyro_bias_initial.transpose() << std::endl;

  gyro_bias_recalc_at_start = bot_param_get_boolean_or_fail(_param, "state_estimator.ins.gyro_bias_recalc_at_start");
  std::cout << "INS will " << (gyro_bias_recalc_at_start ? "" : "NOT ")
            << "recompute initial gyro bias at init" << std::endl;

  gyro_bias_update_online = bot_param_get_boolean_or_fail(_param, "state_estimator.ins.gyro_bias_update_online");
  std::cout << "INS will " << (gyro_bias_update_online ? "" : "NOT ") << "update gyro bias online" << std::endl;
  // If we don't want to update the bias, the covariance must be 0
  if(!gyro_bias_update_online){
      cov_gyro_bias = 0.0;
  }

}

RBISUpdateInterface * InsHandler::processMessage(const bot_core::ins_t * msg,
                                                 MavStateEstimator* state_estimator)
{

  double body_accel[3];
  //bot_trans_apply_vec(&ins_to_body, msg->accel, body_accel);
  bot_quat_rotate_to(ins_to_body.rot_quat, msg->accel, body_accel);
  Eigen::Map<Eigen::Vector3d> accelerometer(body_accel);


  // mfallon thinks this was incorrect as the addition of the translation seems wrong:
  // experimentally the bias estimator estimates the body-imu translation (fixed may 2014):
  // bot_trans_apply_vec(&ins_to_body, msg->gyro, body_gyro);
  double body_gyro[3];
  bot_quat_rotate_to(ins_to_body.rot_quat, msg->gyro, body_gyro);
  Eigen::Map<Eigen::Vector3d> gyro(body_gyro);

  RBISIMUProcessStep* update = new RBISIMUProcessStep(gyro,
          accelerometer,
          cov_gyro,
          cov_accel,
          cov_gyro_bias,
          cov_accel_bias,
          dt,
          msg->utime);

    // Reset the bias values to the original value, if requested
    if(!gyro_bias_update_online) {
        update->posterior_state.gyroBias() = gyro_bias_initial;
    }

    if(!accel_bias_update_online) {
        update->posterior_state.accelBias() = accel_bias_initial;
    }

    return update;
}

bool InsHandler::processMessageInit(const bot_core::ins_t * msg,
    const std::map<std::string, bool> & sensors_initialized
    , const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov)
{
  init_state.utime = msg->utime;

  RBISIMUProcessStep * update = dynamic_cast<RBISIMUProcessStep *>(processMessage(msg, NULL));

  if(  !RBISInitializer::allInitializedExcept(sensors_initialized, "ins")) //force the INS to go last
    return false;

  init_counter++;

  double mag[3]; //not used
  bot_trans_apply_vec(&ins_to_body, msg->mag, mag);
  Eigen::Map<Eigen::Vector3d> mag_vec(mag);

  return processMessageInitCommon(sensors_initialized, default_state, default_cov, init_state, init_cov, update, mag_vec);
}





////////////////// Common ///////////////////
bool InsHandler::processMessageInitCommon(const std::map<std::string, bool> & sensors_initialized
    , const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov,
    RBISIMUProcessStep * update, Eigen::Vector3d mag_vec)
{

  g_vec_sum += -update->accelerometer;
  mag_vec_sum += mag_vec;
  gyro_bias_sum += update->gyro;

  delete update;

  if (init_counter < num_to_init) {
    return false;
  }
  else {
    if ((init_cov.diagonal().segment<2>(RBIS::chi_ind).array() > 0).any()) {
      fprintf(stderr, "Warning: overriding initial roll, pitch with IMU values\n");
    }

    Eigen::Vector3d ins_g_vec_est = g_vec_sum / (double) init_counter;
    Eigen::Vector3d ins_gyro_bias_est = gyro_bias_sum / (double) init_counter;

    if ( (  (fabs(ins_gyro_bias_est(0))>max_initial_gyro_bias)
          ||(fabs(ins_gyro_bias_est(1))>max_initial_gyro_bias))
         || (fabs(ins_gyro_bias_est(2))>max_initial_gyro_bias) ){
      fprintf(stderr, "Initial gyro bias estimates: %f,%f,%f\n", ins_gyro_bias_est(0),
              ins_gyro_bias_est(1), ins_gyro_bias_est(2));
      fprintf(stderr, "Warning: initial gyro bias estimates exceed max (%f), setting to (0,0,0)\n",
              max_initial_gyro_bias);
      ins_gyro_bias_est = Eigen::Vector3d(0,0,0);
    }

    //set orientation
    Eigen::Quaterniond quat_g_vec;
    quat_g_vec.setFromTwoVectors(ins_g_vec_est, -Eigen::Vector3d::UnitZ()); //the gravity vector points in the negative z axis

    Eigen::Vector3d g_vec_rpy = bot_to_degrees(eigen_utils::getEulerAngles(quat_g_vec));
    fprintf(stderr, "Roll, Pitch Initialized from INS: %f, %f \n", g_vec_rpy(0), g_vec_rpy(1));

    init_state.orientation() = init_state.orientation() * quat_g_vec;
    init_cov.block<2, 2>(RBIS::chi_ind, RBIS::chi_ind) = default_cov.block<2, 2>(
        RBIS::chi_ind, RBIS::chi_ind);

    init_state.gyroBias() = ins_gyro_bias_est;
    init_cov.block<3, 3>(RBIS::gyro_bias_ind, RBIS::gyro_bias_ind) = default_cov.block<3, 3>(
        RBIS::gyro_bias_ind, RBIS::gyro_bias_ind);
    fprintf(stderr, "gyro bias using INS: %f,%f,%f\n", init_state.gyroBias()(0),
        init_state.gyroBias()(1), init_state.gyroBias()(2));

    if (RBISInitializer::initializingWith(sensors_initialized, "gps")) {
      Eigen::Vector3d ins_mag_vec_est = mag_vec_sum / (double) init_counter;
      ins_mag_vec_est(2) = 0; //make sure we're only trying to align in the xy plane

      Eigen::Quaterniond quat_mag;
      quat_mag.setFromTwoVectors(ins_mag_vec_est, Eigen::Vector3d::UnitY()); //in ENU, the magnetic vector should be aligned with Y axis

      init_state.orientation() = quat_mag * init_state.orientation();

      init_cov(RBIS::chi_ind + 2, RBIS::chi_ind + 2) = default_cov(RBIS::chi_ind + 2,
          RBIS::chi_ind + 2);

      Eigen::Vector3d mag_vec_rpy = bot_to_degrees(eigen_utils::getEulerAngles(quat_mag));
      fprintf(stderr, "Yaw Initialized from INS: %f\n", mag_vec_rpy(2));
    }

    if(accel_bias_recalc_at_start) {
        std::cout << "Estimated initial accel bias as: " << init_state.accelBias().transpose() << std::endl;
        accel_bias_initial = init_state.accelBias();
    }

    if(gyro_bias_recalc_at_start){
        std::cout << "Estimated initial gyro bias as: " << init_state.gyroBias().transpose() << std::endl;
        gyro_bias_initial = init_state.gyroBias();
    }

    init_state.gyroBias() = gyro_bias_initial;
    init_state.accelBias() = accel_bias_initial;

    return true;
  }
}


}
