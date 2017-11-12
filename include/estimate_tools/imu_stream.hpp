#ifndef IMU_STREAM_
#define IMU_STREAM_

#include <iostream>
#include <inttypes.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <lcmtypes/bot_core/kvh_raw_imu_batch_t.hpp>

struct IMUPacket
{
  int64_t utime_raw; // raw utime in message, deltas between this utime will be very accurate
  // utime of the batch packet that contained this message: will not drift relative to other messages
  // increments at ~333Hz. created on robot machine, with bot sync
  int64_t utime_batch; 
  // Combinaton of utime_raw and utime_batch
  int64_t utime;
  // difference between utime_raw and previous measurement
  int64_t utime_delta; 

  int64_t packet_count;
  Eigen::Vector3d delta_rotation;
  Eigen::Vector3d linear_acceleration;

};

struct IMUBatch
{
  // Original Batch Packet Utime
  int64_t utime;
  // The new packets that were received with this batch
  std::vector<IMUPacket> packets; // ordered [0] oldest [end] newest
  // The old packets that were received with this batch
  std::vector<IMUPacket> packets_old; // ordered [0] oldest [end] newest
};


class IMUStream{
  public:
    IMUStream();
    
    ~IMUStream(){
    }    
    
    bot_core::kvh_raw_imu_batch_t convertToLCMBatch(IMUBatch batch);  
    IMUBatch  convertFromLCMBatch(bot_core::kvh_raw_imu_batch_t* msg);
    
    bot_core::kvh_raw_imu_t convertToLCMPacket(IMUPacket packet);
    IMUPacket convertFromLCMPacket(bot_core::kvh_raw_imu_t msg, int64_t last_packet_utime, int64_t batch_utime);
    
    int getNumberBatchSinceReset(){ return counter_; };
  private:
    
    int64_t last_packet_;
    int64_t last_packet_utime_;
    int counter_; // don't publish if only just started filtering
    // number of packets since restart (can be used to determine recent restart
};

#endif /*IMU_STREAM_*/
