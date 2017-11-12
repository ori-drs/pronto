#include <estimate_tools/imu_stream.hpp>


IMUStream::IMUStream(){
  last_packet_ = -1;
  last_packet_utime_ = 0;  
  counter_= 0 ;
}

bot_core::kvh_raw_imu_t IMUStream::convertToLCMPacket(IMUPacket packet){

  bot_core::kvh_raw_imu_t m;
  m.utime = packet.utime;
  m.packet_count = packet.packet_count;
  m.delta_rotation[0] = packet.delta_rotation[0];
  m.delta_rotation[1] = packet.delta_rotation[1];
  m.delta_rotation[2] = packet.delta_rotation[2];

  m.linear_acceleration[0] = packet.linear_acceleration[0];
  m.linear_acceleration[1] = packet.linear_acceleration[1];
  m.linear_acceleration[2] = packet.linear_acceleration[2];
  return m;
}

IMUPacket IMUStream::convertFromLCMPacket(bot_core::kvh_raw_imu_t msg, int64_t last_packet_utime, int64_t batch_utime){
      IMUPacket raw;
      raw.utime_raw = msg.utime;
      raw.utime_batch = batch_utime; // the main incoming utime
      raw.utime_delta = raw.utime_raw - last_packet_utime;
      raw.utime = raw.utime_raw; // for now use raw timestamp

      raw.packet_count = msg.packet_count;
      raw.delta_rotation = Eigen::Vector3d(msg.delta_rotation);
      raw.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration);
  return raw;
}

bot_core::kvh_raw_imu_batch_t IMUStream::convertToLCMBatch(IMUBatch batch){

  bot_core::kvh_raw_imu_batch_t m;
  m.utime = batch.utime;

  // Ouput the packets in the original order:
  //     newest new packet
  // 2nd newest new packet
  // ...
  //     newest old packet (that appeared in a previous message
  // 2nd newest old packet
  // ...
  
  for (int i= batch.packets.size()-1 ; i >= 0 ; i-- ){
    m.raw_imu.push_back( convertToLCMPacket( batch.packets[i] ) );
  }
  
  for (int i= batch.packets_old.size()-1 ; i >= 0 ; i-- ){
    m.raw_imu.push_back( convertToLCMPacket( batch.packets_old[i] ) );
  }
  m.num_packets = m.raw_imu.size();
  return m;
}

IMUBatch IMUStream::convertFromLCMBatch(bot_core::kvh_raw_imu_batch_t* msg){
  if (msg->raw_imu[0].packet_count < last_packet_){
    std::cout << "Detected time skip, resetting IMUStream\n";
    last_packet_ = -1;
    last_packet_utime_ = 0;
    counter_ =0 ;
  }

  int num_new = 0;
  bool verbose = false;

  IMUBatch batch;
  batch.utime = msg->utime;

  for (int i=msg->num_packets-1 ; i >= 0 ; i--){
    if (msg->raw_imu[i].packet_count > last_packet_){
      if (verbose) std::cout << "new " <<", "<<last_packet_ <<", "<< i <<", "<< msg->raw_imu[i].packet_count << ", " << msg->raw_imu[i].utime << "\n";

      IMUPacket raw = convertFromLCMPacket(msg->raw_imu[i], last_packet_utime_, msg->utime);
      batch.packets.push_back(raw);

      last_packet_ = msg->raw_imu[i].packet_count;
      last_packet_utime_ = msg->raw_imu[i].utime;
      num_new++;
    }else{
      // delibrately obsficate the delta field (we dont know it here)
      IMUPacket raw = convertFromLCMPacket(msg->raw_imu[i], -msg->raw_imu[i].utime, msg->utime); 
      batch.packets_old.push_back(raw);
      if (verbose) std::cout << "old " <<", "<<last_packet_ <<", "<< i <<", "<< msg->raw_imu[i].packet_count << ", " << msg->raw_imu[i].utime << "\n";
    }
  }

  if (verbose) std::cout << num_new << " in " << msg->utime  << " ---\n\n";

  counter_++;
  return batch;
}
