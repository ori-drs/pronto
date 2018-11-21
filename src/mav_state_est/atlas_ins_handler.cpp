#include "mav_state_est/atlas_ins_handler.hpp"
#include "mav_state_est/rbis_initializer.hpp"

namespace MavStateEst {

AtlasInsHandler::AtlasInsHandler(BotParam *_param, BotFrames *_frames) : InsHandler(_param, _frames) {
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
}

////////// Atlas KVH INS /////////////////
void AtlasInsHandler::doFilter(IMUPacket &raw){
  // notch 85Hz, 170, 340Hz in cascade
  for (int i=0; i <3; i++){
    raw.linear_acceleration[0]  = notchfilter_x[i]->processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = notchfilter_y[i]->processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = notchfilter_z[i]->processSample( raw.linear_acceleration[2] );
  }
}


RBISUpdateInterface * AtlasInsHandler::processMessageAtlas(const bot_core::kvh_raw_imu_batch_t * msg, MavStateEstimator* state_estimator)
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

bool AtlasInsHandler::processMessageInitAtlas(const bot_core::kvh_raw_imu_batch_t * msg,
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
} // namespace MavStateEst
