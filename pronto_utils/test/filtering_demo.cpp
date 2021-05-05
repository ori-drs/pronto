#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

//#include <ConciseArgs>

#include <lcmtypes/bot_core/kvh_raw_imu_batch_t.hpp>

#include <filter_tools/Filter.hpp>
#include <filter_tools/HeavyLowPassFilter.hpp>
#include <pronto_estimate_tools/imu_stream.hpp>
#include <pronto_estimate_tools/iir_notch.hpp>

using namespace std;

struct CommandLineConfig
{
    int filter_type;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);
    
    ~App(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    
    void batchHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::kvh_raw_imu_batch_t* msg); 
    void doFilter(IMUPacket &raw);
    const CommandLineConfig cl_cfg_;  
    
    IMUStream imu_data_;
    
    LowPassFilter lpfilter[3];
    HeavyLowPassFilter hlpfilter[3];
    
    // An cascade of 3 notch filters in xyz
    IIRNotch* notchfilter_x[3];
    IIRNotch* notchfilter_y[3];
    IIRNotch* notchfilter_z[3];
};




App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){
  lcm_->subscribe( "ATLAS_IMU_BATCH" ,&App::batchHandler,this);
  
  double notch_freq = 87;
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


void App::doFilter(IMUPacket &raw){
  // std::cout <<   raw.linear_acceleration[2] << "\n";

  if (cl_cfg_.filter_type==0){
    // "light low pass filter"
    raw.linear_acceleration[0]  = (double)lpfilter[0].processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = (double)lpfilter[1].processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = (double)lpfilter[2].processSample( raw.linear_acceleration[2] );
  }else if (cl_cfg_.filter_type ==1){ // reasonable high [ass gilter
    // "heavy low pass filter"
    raw.linear_acceleration[0]  = hlpfilter[0].processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = hlpfilter[1].processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = hlpfilter[2].processSample( raw.linear_acceleration[2] );
  }else if(cl_cfg_.filter_type==2){
    // notch 85Hz
    raw.linear_acceleration[0]  = notchfilter_x[0]->processSample( raw.linear_acceleration[0] );
    raw.linear_acceleration[1]  = notchfilter_y[0]->processSample( raw.linear_acceleration[1] );
    raw.linear_acceleration[2]  = notchfilter_z[0]->processSample( raw.linear_acceleration[2] );
  }else{
    // notch 85Hz, 170, 340Hz in cascade
    for (int i=0; i <3; i++){
      raw.linear_acceleration[0]  = notchfilter_x[i]->processSample( raw.linear_acceleration[0] );
      raw.linear_acceleration[1]  = notchfilter_y[i]->processSample( raw.linear_acceleration[1] );
      raw.linear_acceleration[2]  = notchfilter_z[i]->processSample( raw.linear_acceleration[2] );
    }    
  }
}


void App::batchHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::kvh_raw_imu_batch_t* msg){
  // 1. Convert incoming message to an IMU batch
  //    this will split the old packets from the new ones
  bot_core::kvh_raw_imu_batch_t msg_copy = bot_core::kvh_raw_imu_batch_t(*msg);
  IMUBatch batch = imu_data_.convertFromLCMBatch(&msg_copy);

  // 2. Republish (only) the new packets individually (so that signal scope can plot)
  for (int i=0 ; i < batch.packets.size() ; i++ ){ //oldest to newest
    bot_core::kvh_raw_imu_t raw_msg = imu_data_.convertToLCMPacket( batch.packets[i] );
    lcm_->publish("ATLAS_IMU_PACKET", &raw_msg); 
  }

  // 3 . Only filter new packets:
  for (int i=0 ; i < batch.packets.size() ; i++ ){
    doFilter( batch.packets[i] );
  }

  // 4. Output (only) the new packets (so that signal scope can plot)  
  for (int i=0 ; i < batch.packets.size() ; i++ ){ //oldest to newest
    bot_core::kvh_raw_imu_t imu_filtered_msg = imu_data_.convertToLCMPacket( batch.packets[i]);
    lcm_->publish("ATLAS_IMU_PACKET_FILTERED", &imu_filtered_msg);
  }

  // 5. If the filter hasn't been reset recently, republish:
  if (imu_data_.getNumberBatchSinceReset()  > 50){
    bot_core::kvh_raw_imu_batch_t batch_filtered_msg = imu_data_.convertToLCMBatch(batch);
    lcm_->publish("ATLAS_IMU_BATCH_FILTERED", &batch_filtered_msg); 
  }
}


int main(int argc, char ** argv) {
  //CommandLineConfig cl_cfg;
  cl_cfg.filter_type = 0; // 0 lp, 1 hlp, 2 notch
  //ConciseArgs opt(argc, (char**)argv);
  //opt.add(cl_cfg.filter_type, "f", "filter_type","Filter Type");
  //opt.parse();  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App app(lcm, cl_cfg);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
