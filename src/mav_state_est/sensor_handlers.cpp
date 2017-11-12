#include "mav_state_est/sensor_handlers.hpp"

using namespace Eigen;

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

  // added for atlas: 3 notch filters
  atlas_filter = bot_param_get_boolean_or_fail(_param, "state_estimator.ins.atlas_filter");
  double notch_freq = bot_param_get_double_or_fail(_param, "state_estimator.ins.atlas_filter_freq");
  double fs = 1000;
  for (int i=0; i < 3 ; i++){
    IIRNotch* x_filter = new IIRNotch ( notch_freq*pow(2,i) , fs );
    notchfilter_x[i] = x_filter;

    IIRNotch* y_filter = new IIRNotch ( notch_freq*pow(2,i) , fs);
    notchfilter_y[i] = y_filter;

    IIRNotch* z_filter = new IIRNotch ( notch_freq*pow(2,i) , fs);
    notchfilter_z[i] = z_filter;
  }


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

////////// Typical Micro Strain INS /////////////////
RBISUpdateInterface * InsHandler::processMessage(const bot_core::ins_t * msg, MavStateEstimator* state_estimator)
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

////////// Atlas KVH INS /////////////////
void InsHandler::doFilter(IMUPacket &raw){
  // notch 85Hz, 170, 340Hz in cascade
  for (int i=0; i <3; i++){
    raw.linear_acceleration[0]  = notchfilter_x[i]->processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = notchfilter_y[i]->processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = notchfilter_z[i]->processSample( raw.linear_acceleration[2] );
  }
}


RBISUpdateInterface * InsHandler::processMessageAtlas(const bot_core::kvh_raw_imu_batch_t * msg, MavStateEstimator* state_estimator)
{

  double linear_acceleration[3];
  double delta_rotation[3];
  double raw_dt;
  int64_t utime;

  if (atlas_filter){
    // Decode the data and split out the new packets:
    bot_core::kvh_raw_imu_batch_t msg_copy = bot_core::kvh_raw_imu_batch_t(*msg);
    IMUBatch batch = imu_data_.convertFromLCMBatch(&msg_copy);
    // Only filter new packets:
    for (int i=0 ; i < batch.packets.size() ; i++ ){
      doFilter( batch.packets[i] );
    }
    if (batch.packets.size() ==0){
      //std::cout << msg->utime << " IMPORTANT: No new IMU packets detected - shouldn't happen. Skipping iteration\n";
      // mfallon: I observed this once or twice in my logs from 2014-01-21: exactly the
      // same imu data from two consecutive batch messages
      // update: happens all the time at 1kHz
      return NULL;
    }//else{
    //  std::cout << batch.packets.size() << " new packets\n";
    //}

    // Get the most recent filtered packet:
    IMUPacket p = batch.packets[batch.packets.size() -1 ];

    raw_dt =  p.utime_delta*1E-6;
    memcpy(linear_acceleration, p.linear_acceleration.data(),p.linear_acceleration.size() * sizeof(double));
    memcpy(delta_rotation, p.delta_rotation.data(),p.delta_rotation.size() * sizeof(double));
    utime = msg->utime;

  }else{
    memcpy( linear_acceleration, msg->raw_imu[0].linear_acceleration, 3*sizeof(double));
    memcpy( delta_rotation, msg->raw_imu[0].delta_rotation, 3*sizeof(double));
    raw_dt = (msg->raw_imu[0].utime - msg->raw_imu[1].utime)*1E-6;
    utime = msg->utime;
  }



  // Convert Rotation Amounts to rotation rates:
  double sensor_gyro[3];
  sensor_gyro[0] = (delta_rotation[0])/raw_dt;
  sensor_gyro[1] = (delta_rotation[1])/raw_dt;
  sensor_gyro[2] = (delta_rotation[2])/raw_dt;

  if (1==0){
    std::cout << "===\n";
    std::cout << "rotation\n";
    std::cout << delta_rotation[0] << ", " << delta_rotation[1] << ", "
              << delta_rotation[2] << " delta_rot\n";
    std::cout << "rotation rate\n";
    std::cout << raw_dt << "\n";
    std::cout << sensor_gyro[0] << ", " << sensor_gyro[1] << ", "
              << sensor_gyro[2] << " gyro\n";
  }

  //    get everything into the right frame
  double body_accel[3];
  bot_trans_apply_vec(&ins_to_body, linear_acceleration, body_accel);
  Eigen::Map<Eigen::Vector3d> accelerometer(body_accel);

  double body_gyro[3];
  // was this. mfallon thinks this is incorrect as the addition of the trans seems wrong:
  // experimentally the bias estimator estimates the body-imu translation (fixed may 2014)
  //bot_trans_apply_vec(&ins_to_body, sensor_gyro, body_gyro);
  bot_quat_rotate_to(ins_to_body.rot_quat, sensor_gyro, body_gyro);
  Eigen::Map<Eigen::Vector3d> gyro(body_gyro);

  // Use message timestamp for dt after initialization
  // TODO: quantify if this is useful versus just using 1/nominal_hz
  double integration_dt;
  if (prev_utime_atlas==0){
    integration_dt = dt; // use default
  }else{
    integration_dt = (utime - prev_utime_atlas)*1E-6;
  }
  if (integration_dt > 0.1){ // until dt integrity is confirmed..
    std::cout << "dt was : " << integration_dt << " - there is an issue with timestamps\n";
    //exit(-1);
  }
  prev_utime_atlas = utime;
  // std::cout << utime << " ins utime\n";
  return new RBISIMUProcessStep(gyro, accelerometer, cov_gyro, cov_accel, cov_gyro_bias, cov_accel_bias, integration_dt, utime);
}

bool InsHandler::processMessageInitAtlas(const bot_core::kvh_raw_imu_batch_t * msg,
    const std::map<std::string, bool> & sensors_initialized
    , const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov)
{
  init_state.utime = msg->utime;

  RBISIMUProcessStep * update = dynamic_cast<RBISIMUProcessStep *>(processMessageAtlas(msg, NULL));

  if (update == NULL){
    // ... this happens regularly at 1000Hz, so nothing to worry about
    //std::cout << "Didn't get a new Atlas packet during initialization, skipping\n";
    return false;
  }

  if(  !RBISInitializer::allInitializedExcept(sensors_initialized, "ins")) //force the INS to go last
    return false;

  init_counter++;

  double mag[3] = {0.0,0.0,0.0}; //not used
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

GpsHandler::GpsHandler(BotParam * _param)
{
  double r_gps_xy = bot_param_get_double_or_fail(_param, "state_estimator.gps.r_xy");
  double r_gps_z = bot_param_get_double_or_fail(_param, "state_estimator.gps.r_z");
  Eigen::Vector3d R_gps_diagonal = Eigen::Array3d(r_gps_xy, r_gps_xy, r_gps_z).pow(2);
  cov_xyz = R_gps_diagonal.asDiagonal();
}

RBISUpdateInterface * GpsHandler::processMessage(const bot_core::gps_data_t * msg, MavStateEstimator* state_estimator)
{
  if (msg->gps_lock < 3)
    return NULL;

  return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
      Eigen::Map<const Eigen::Vector3d>(msg->xyz_pos), cov_xyz, RBISUpdateInterface::gps, msg->utime);
}

bool GpsHandler::processMessageInit(const bot_core::gps_data_t * msg,
    const std::map<std::string, bool> & sensors_initialized
    ,const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov)
{
  init_state.utime = msg->utime;
  RBISIndexedMeasurement * update = dynamic_cast<RBISIndexedMeasurement *>(GpsHandler::processMessage(msg, NULL));

  if(update == NULL) {
    return false;
  }

  init_state.position() = update->measurement;
  init_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = update->measurement_cov;

  fprintf(stderr, "initialized position using GPS at xyz: %f,%f,%f\n", init_state.position()(0),
      init_state.position()(1), init_state.position()(2));

  delete update;
  return true;
}


ViconHandler::ViconHandler(BotParam * param, BotFrames * frames)
{
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.vicon.mode");

  if (strcmp(mode_str, "position") == 0) {
    mode = ViconHandler::MODE_POSITION;
    std::cout << "Vicon will provide position measurements." << std::endl;
  }
  else if (strcmp(mode_str, "position_orient") == 0) {
    mode = ViconHandler::MODE_POSITION_ORIENT;
    std::cout << "Vicon will provide position and orientation measurements." << std::endl;
  }
  else if (strcmp(mode_str, "orientation") == 0) {
    mode = ViconHandler::MODE_ORIENTATION;
    std::cout << "Vicon will provide orientation measurements." << std::endl;
  }
  else if (strcmp(mode_str, "yaw") == 0) {
    mode = ViconHandler::MODE_YAW;
    std::cout << "Vicon will provide yaw orientation measurements." << std::endl;
  }
  else {
    mode = ViconHandler::MODE_POSITION;
    std::cout << "Unrecognized Vicon mode. Using position mode by default." << std::endl;
  }

  apply_frame = bot_param_get_boolean_or_fail(param, "state_estimator.vicon.apply_frame");
  if (apply_frame){
    char* frame_from = bot_param_get_str_or_fail(param, "state_estimator.vicon.frame_from");
    char* frame_to   = bot_param_get_str_or_fail(param, "state_estimator.vicon.frame_to");
    bot_frames_get_trans(frames, frame_from, frame_to, &body_to_vicon);
  }

  free(mode_str);
  init(param);
}

ViconHandler::ViconHandler(BotParam * param, ViconMode vicon_mode)
{
  init(param);
}

void ViconHandler::init(BotParam * param)
{
  // Build full covariance matrix - we may only use part of it.
  double r_vicon_xyz = bot_param_get_double_or_fail(param, "state_estimator.vicon.r_xyz");
  double r_vicon_chi = bot_param_get_double_or_fail(param, "state_estimator.vicon.r_chi");

  cov_vicon = Eigen::MatrixXd::Zero(6, 6);
  cov_vicon.topLeftCorner<3, 3>() = pow(r_vicon_xyz, 2) * Eigen::Matrix3d::Identity();
  cov_vicon.bottomRightCorner<3, 3>() = pow(bot_to_radians(r_vicon_chi), 2) * Eigen::Matrix3d::Identity();

  if (mode == MODE_POSITION) {
    z_indices = RBIS::positionInds();
  } else if (mode == MODE_YAW){
      z_indices.resize(1);
      z_indices(0) = RBIS::chi_ind + 2; // z component only
      cov_vicon.resize(1,1);
      cov_vicon(0,0) = pow(bot_to_radians(r_vicon_chi), 2);
  } else if (mode == MODE_ORIENTATION){
      z_indices.resize(3);
      z_indices = RBIS::chiInds();
      cov_vicon.resize(3,3);
      cov_vicon = pow(bot_to_radians(r_vicon_chi), 2) * Eigen::Matrix3d::Identity();
  } else {
    z_indices.resize(6);
    z_indices.head<3>() = RBIS::positionInds();
    z_indices.tail<3>() = RBIS::chiInds();
  }
}

RBISUpdateInterface * ViconHandler::processMessage(const bot_core::rigid_transform_t * msg, MavStateEstimator* state_estimator)
{

  BotTrans local_to_vicon;
  memset(&local_to_vicon, 0, sizeof(local_to_vicon));
  memcpy(local_to_vicon.trans_vec, msg->trans, 3*sizeof(double));
  memcpy(local_to_vicon.rot_quat, msg->quat, 4*sizeof(double));

  BotTrans local_to_body;
  memset(&local_to_body, 0, sizeof(local_to_body));
  if (apply_frame){
    bot_trans_apply_trans_to( &local_to_vicon,&body_to_vicon, &local_to_body);
  }else{
    bot_trans_copy(&local_to_body, &local_to_vicon);
  }
  ////////////////////////////////////

  if ((Eigen::Map<const Eigen::Array3d>( local_to_vicon.trans_vec ).abs() < 1e-5).all())
    return NULL;

  if (mode == MODE_POSITION) {
      return new RBISIndexedMeasurement(z_indices, Eigen::Map<const Eigen::Vector3d>( local_to_body.trans_vec ),
                                        cov_vicon.block<3, 3>(0, 0), RBISUpdateInterface::vicon, msg->utime);
  } else if(mode == MODE_YAW) {

      Eigen::VectorXd z_meas(1);
      Eigen::Quaterniond quat;
      eigen_utils::botDoubleToQuaternion(quat, local_to_body.rot_quat);

      return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                       z_meas,
                                                       cov_vicon,
                                                       quat,
                                                       RBISUpdateInterface::vicon,
                                                       msg->utime);
  }  else if(mode == MODE_ORIENTATION){
      Eigen::VectorXd z_meas(3);
      Eigen::Quaterniond quat;
      eigen_utils::botDoubleToQuaternion(quat, local_to_body.rot_quat);

      return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                       z_meas,
                                                       cov_vicon,
                                                       quat,
                                                       RBISUpdateInterface::vicon,
                                                       msg->utime);
  }  else if(mode == MODE_POSITION_ORIENT){
      Eigen::VectorXd z_meas(6);
      Eigen::Quaterniond quat;
      eigen_utils::botDoubleToQuaternion(quat, local_to_body.rot_quat );

      z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(local_to_body.trans_vec);

      return new RBISIndexedPlusOrientationMeasurement(z_indices,
                                                       z_meas,
                                                       cov_vicon,
                                                       quat,
                                                       RBISUpdateInterface::vicon,
                                                       msg->utime);
  }
}

bool ViconHandler::processMessageInit(const bot_core::rigid_transform_t * msg,
    const std::map<std::string, bool> & sensors_initialized
    , const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov)
{

  BotTrans local_to_vicon;
  memset(&local_to_vicon, 0, sizeof(local_to_vicon));
  memcpy(local_to_vicon.trans_vec, msg->trans, 3*sizeof(double));
  memcpy(local_to_vicon.rot_quat, msg->quat, 4*sizeof(double));

  BotTrans local_to_body;
  memset(&local_to_body, 0, sizeof(local_to_body));
  if (apply_frame){
    bot_trans_apply_trans_to( &local_to_vicon,&body_to_vicon, &local_to_body);
  }else{
    bot_trans_copy(&local_to_body, &local_to_vicon);
  }
  ////////////////////////////////////

  init_state.utime = msg->utime;

  init_state.position() = Eigen::Map<const Eigen::Vector3d>(local_to_body.trans_vec);
  eigen_utils::botDoubleToQuaternion(init_state.orientation(), local_to_body.rot_quat);

  init_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = cov_vicon.topLeftCorner<3, 3>();
  init_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = cov_vicon.bottomRightCorner<3, 3>();

  Vector3d init_rpy_deg = bot_to_degrees(init_state.getEulerAngles());

  fprintf(stderr, "initialized position using VICON at xyz: %f,%f,%f\n", init_state.position()(0),
      init_state.position()(1), init_state.position()(2));
  fprintf(stderr, "initialized orientation using VICON at rpy: %f,%f,%f\n", init_rpy_deg(0),
      init_rpy_deg(1), init_rpy_deg(2));

  return true;
}

RBISUpdateInterface * IndexedMeasurementHandler::processMessage(const pronto::indexed_measurement_t * msg, MavStateEstimator* state_estimator)
{
  return new RBISIndexedMeasurement(Eigen::Map<const Eigen::VectorXi>(&msg->z_indices[0], msg->measured_dim),
      Eigen::Map<const Eigen::VectorXd>(&msg->z_effective[0], msg->measured_dim),
      Eigen::Map<const Eigen::MatrixXd>(&msg->R_effective[0], msg->measured_dim, msg->measured_dim),
      indexed_sensor, msg->utime);
}

bool IndexedMeasurementHandler::processMessageInit(const pronto::indexed_measurement_t * msg,
    const std::map<std::string, bool> & sensors_initialized
    , const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov)
{

  RBISIndexedMeasurement * update = (RBISIndexedMeasurement *) processMessage(msg, NULL);

  if (update == NULL) {
    return false;
  }

  int m = update->index.rows();
  for (int ii = 0; ii < m; ii++) {
    for (int jj = 0; jj < m; jj++) {
      init_cov(update->index(ii), update->index(jj)) = update->measurement_cov(ii, jj);
    }
    init_state.vec(update->index(ii)) = update->measurement(ii);
  }
  init_state.chiToQuat();

  std::cout << "Initializing indices:\n" << update->index << "\n with:\n" << update->measurement << "\nand cov:\n"
      << update->measurement_cov << "\n";
  delete update;

  return true;
}

ScanMatcherHandler::ScanMatcherHandler(BotParam * param)
{
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.scan_matcher.mode");

  if (strcmp(mode_str, "position") == 0) {
    mode = ScanMatcherHandler::MODE_POSITION;
    std::cout << "Scan matcher will provide position measurements." << std::endl;
  }
  else if (strcmp(mode_str, "position_yaw") == 0) {
    mode = ScanMatcherHandler::MODE_POSITION_YAW;
    std::cout << "Scan matcher will provide position and yaw measurements." << std::endl;
  }
  else if (strcmp(mode_str, "velocity") == 0) {
    mode = ScanMatcherHandler::MODE_VELOCITY;
    std::cout << "Scan matcher will provide velocity measurements." << std::endl;
  }
  else if (strcmp(mode_str, "velocity_yaw") == 0) {
    mode = ScanMatcherHandler::MODE_VELOCITY_YAW;
    std::cout << "Scan matcher will provide velocity and yaw measurements." << std::endl;
  }
  else if (strcmp(mode_str, "yaw") == 0) {
    mode = ScanMatcherHandler::MODE_YAW;
    std::cout << "Scan matcher will provide yaw measurements." << std::endl;
  }
  else {
    mode = ScanMatcherHandler::MODE_VELOCITY;
    std::cout << "Unrecognized scan matcher mode. Using velocity mode by default." << std::endl;
  }

  free(mode_str);
  Eigen::VectorXd R_scan_match;

  if (mode == MODE_POSITION || mode == MODE_VELOCITY) {
    z_indices.resize(3);
    R_scan_match.resize(3);
  }
  else if (mode == MODE_YAW) {
    z_indices.resize(1);
    R_scan_match.resize(1);
  }
  else {
    z_indices.resize(4); // Use yaw measurements too.
    R_scan_match.resize(4);
  }

  // Initialize covariance matrix based on mode.
  if (mode == MODE_POSITION || mode == MODE_POSITION_YAW) {
    double r_scan_match_pxy = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_pxy");
    double r_scan_match_pz = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_pz");
    R_scan_match(0) = bot_sq(r_scan_match_pxy); // Cleaner way?
    R_scan_match(1) = bot_sq(r_scan_match_pxy);
    R_scan_match(2) = bot_sq(r_scan_match_pz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::positionInds();
  }
  else if (mode == MODE_YAW) {
    double r_scan_match_yaw = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_yaw");
    R_scan_match(0) = bot_sq(bot_to_radians(r_scan_match_yaw));
    z_indices(0) = RBIS::chi_ind + 2; // z component only
  }
  else {
    double r_scan_match_vxy = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_vxy");
    double r_scan_match_vz = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_vz");
    R_scan_match(0) = bot_sq(r_scan_match_vxy); // Cleaner way?
    R_scan_match(1) = bot_sq(r_scan_match_vxy);
    R_scan_match(2) = bot_sq(r_scan_match_vz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
  }

  if (mode == MODE_POSITION_YAW || mode == MODE_VELOCITY_YAW) {
    double r_scan_match_yaw = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_yaw");
    R_scan_match(3) = bot_sq(bot_to_radians(r_scan_match_yaw));
    z_indices(3) = RBIS::chi_ind + 2; // z component only
  }

  cov_scan_match = R_scan_match.asDiagonal();
}

RBISUpdateInterface * ScanMatcherHandler::processMessage(const bot_core::pose_t * msg, MavStateEstimator* state_estimator)
{

  if (mode == MODE_POSITION) {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
        Eigen::Map<const Eigen::Vector3d>(msg->pos), cov_scan_match, RBISUpdateInterface::scan_matcher,
        msg->utime);
  }
  else if (mode == MODE_VELOCITY) {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
        Eigen::Map<const Eigen::Vector3d>(msg->vel), cov_scan_match, RBISUpdateInterface::scan_matcher,
        msg->utime);
  }
  else if (mode == MODE_YAW) {
    Eigen::Vector4d z_meas = Eigen::Vector4d(0,0,0,0); // unused, I believe
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, msg->orientation);
    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_scan_match, quat,
        RBISUpdateInterface::scan_matcher, msg->utime);
  }
  else if (mode == MODE_POSITION_YAW || mode == MODE_VELOCITY_YAW) {
    Eigen::Vector4d z_meas;
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, msg->orientation);

    if (mode == MODE_POSITION_YAW) {
      z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->pos);
    }
    else {
      z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->vel);
    }

    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_scan_match, quat,
        RBISUpdateInterface::scan_matcher, msg->utime);
  }
}

OpticalFlowHandler::OpticalFlowHandler(BotParam * param, BotFrames * frames)
{
  bot_frames_get_trans(frames, "body", "camera", &body_to_cam);
  bot_trans_get_trans_vec(&body_to_cam, body_to_cam_trans.data());
  bot_trans_get_rot_mat_3x3(&body_to_cam, body_to_cam_rot.data());
//  cam_to_body_trans = cam_to_body_rot * cam_to_body_trans;

  Eigen::Vector4d R_optical_flow_xyrs;

  double r_optical_flow_x = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_ux");
  double r_optical_flow_y = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_uy");
  double r_optical_flow_r = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_r");
  double r_optical_flow_s = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_s");

  R_optical_flow_xyrs
  << Eigen::Array4d(r_optical_flow_x, r_optical_flow_y, r_optical_flow_r, r_optical_flow_s).square();
  cov_xyrs = R_optical_flow_xyrs.asDiagonal();
}

RBISUpdateInterface * OpticalFlowHandler::processMessage(const pronto::optical_flow_t * msg, MavStateEstimator* state_estimator)
{
// Camera frame to body frame transform.
  z_xyrs(0) = msg->ux;
  z_xyrs(1) = msg->uy;
  z_xyrs(2) = msg->theta;
  z_xyrs(3) = msg->scale;

  double alpha1 = msg->alpha1;
  double alpha2 = msg->alpha2;
  double gamma = msg->gamma;

//  eigen_dump(z_xyrs);
//  eigen_dump(cov_xyrs);
//  eigen_dump(cam_to_body_trans);
//  eigen_dump(cam_to_body_rot);
  return new RBISOpticalFlowMeasurement(z_xyrs, cov_xyrs, body_to_cam_trans, body_to_cam_rot, alpha1, alpha2, gamma,
      RBISUpdateInterface::optical_flow, msg->utime);
}

}

