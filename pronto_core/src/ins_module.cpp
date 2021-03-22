#include "pronto_core/ins_module.hpp"
#include "pronto_core/rigidbody.hpp"
#include "pronto_core/rotations.hpp"
#include <iostream>

namespace pronto {

InsModule::InsModule() : InsModule(InsConfig(), Eigen::Affine3d::Identity())
{
}

InsModule::~InsModule() {

}

InsModule::InsModule(const InsConfig &config, const Eigen::Affine3d &ins_to_body) :
    ins_to_body_(ins_to_body),
    dt(config.dt),
    num_to_init(config.num_to_init),
    max_initial_gyro_bias(config.max_initial_gyro_bias),
    init_counter(0),
    g_vec_sum(Eigen::Vector3d::Zero()),
    mag_vec_sum(Eigen::Vector3d::Zero()),
    gyro_bias_sum(Eigen::Vector3d::Zero()),
    gyro_bias_update_online(config.gyro_bias_update_online),
    gyro_bias_initial(config.gyro_bias_initial),
    gyro_bias_recalc_at_start(config.gyro_bias_recalc_at_start),
    accel_bias_update_online(config.accel_bias_update_online),
    accel_bias_initial(config.accel_bias_initial),
    accel_bias_recalc_at_start(config.accel_bias_recalc_at_start)
{
    cov_accel = std::pow(config.cov_accel, 2);
    cov_gyro = std::pow(config.cov_gyro * M_PI / 180.0, 2);
    cov_accel_bias = std::pow(config.cov_accel_bias, 2);
    cov_gyro_bias = std::pow(config.cov_gyro_bias * M_PI / 180.0,2);
}

bool InsModule::allInitializedExcept(const std::map<std::string, bool> &_sensors_initialized,
                                     const std::string &sensor_prefix)
{
    std::map<std::string, bool>::const_iterator it;
    for (it = _sensors_initialized.begin(); it != _sensors_initialized.end(); it++) {
      if (it->first == sensor_prefix)
        continue;
      if (!it->second)
        return false;
    }
    return true;
}

RBISUpdateInterface * InsModule::processMessage(const ImuMeasurement * msg,
                                                 StateEstimator* state_estimator)
{

  Eigen::Vector3d accelerometer(ins_to_body_.rotation() * msg->acceleration);
  current_omega_ = (ins_to_body_.rotation() * msg->omega);

  // mfallon thinks this was incorrect as the addition of the translation seems wrong:
  // experimentally the bias estimator estimates the body-imu translation (fixed may 2014):
  Eigen::Vector3d gyro(ins_to_body_.rotation() * msg->omega);


  RBISIMUProcessStep* update = new RBISIMUProcessStep(current_omega_,
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

bool InsModule::processMessageInit(const ImuMeasurement * msg,
                                    const std::map<std::string, bool> & sensors_initialized,
                                    const RBIS & default_state,
                                    const RBIM & default_cov,
                                    RBIS & init_state,
                                   RBIM & init_cov)
{
    init_state.utime = msg->utime;

    RBISIMUProcessStep * update = dynamic_cast<RBISIMUProcessStep *>(processMessage(msg, NULL));

    //force the INS to go last
    if(!allInitializedExcept(sensors_initialized, "ins")){
      return false;
    }

    init_counter++;
    // TODO not using magnetometer, sending zero
    // Eigen::Vector3d mag_vec(ins_to_body.rotation() * Eigen::Map<const Eigen::Vector3d>(msg->mag));

    return processMessageInitCommon(sensors_initialized,
                                    default_state,
                                    default_cov,
                                    init_state,
                                    init_cov,
                                    update,
                                    Eigen::Vector3d::Zero());

}
bool InsModule::processMessageInitCommon(const std::map<std::string, bool> & sensors_initialized,
                                          const RBIS & default_state,
                                          const RBIM & default_cov,
                                          RBIS & init_state,
                                          RBIM & init_cov,
                                          RBISIMUProcessStep * update,
                                          const Eigen::Vector3d& mag_vec)
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

    Eigen::Vector3d g_vec_rpy = (rotation::getEulerAngles(quat_g_vec) * 180.0 / M_PI);
    fprintf(stderr, "Roll, Pitch Initialized from INS: %f, %f \n", g_vec_rpy(0), g_vec_rpy(1));
    fprintf(stderr, "Yaw from INS: %f, \n", g_vec_rpy(2));

    init_state.orientation() = init_state.orientation() * quat_g_vec;
    init_cov.block<2, 2>(RBIS::chi_ind, RBIS::chi_ind) = default_cov.block<2, 2>(
        RBIS::chi_ind, RBIS::chi_ind);

    init_state.gyroBias() = ins_gyro_bias_est;
    init_cov.block<3, 3>(RBIS::gyro_bias_ind, RBIS::gyro_bias_ind) = default_cov.block<3, 3>(
        RBIS::gyro_bias_ind, RBIS::gyro_bias_ind);
    fprintf(stderr, "gyro bias using INS: %f,%f,%f\n", init_state.gyroBias()(0),
        init_state.gyroBias()(1), init_state.gyroBias()(2));

    // if the sensor is initialized with the gps we align the yaw from it
    if (sensors_initialized.count("gps") > 0 ) {
      Eigen::Vector3d ins_mag_vec_est = mag_vec_sum / (double) init_counter;
      ins_mag_vec_est(2) = 0; //make sure we're only trying to align in the xy plane

      Eigen::Quaterniond quat_mag;
      quat_mag.setFromTwoVectors(ins_mag_vec_est, Eigen::Vector3d::UnitY()); //in ENU, the magnetic vector should be aligned with Y axis

      init_state.orientation() = quat_mag * init_state.orientation();

      init_cov(RBIS::chi_ind + 2, RBIS::chi_ind + 2) = default_cov(RBIS::chi_ind + 2,
          RBIS::chi_ind + 2);

      Eigen::Vector3d mag_vec_rpy = (rotation::getEulerAngles(quat_mag) * 180.0 / M_PI);
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
