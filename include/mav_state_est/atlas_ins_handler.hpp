#pragma once
#include "mav_state_est/ins_handler.hpp"
#include "estimate_tools/imu_stream.hpp"
#include "estimate_tools/iir_notch.hpp"

namespace MavStateEst {

class AtlasInsHandler : public InsHandler {
public:
    AtlasInsHandler(BotParam * _param, BotFrames * _frames);
    // Compariable Atlas Functions:
    RBISUpdateInterface * processMessageAtlas(const bot_core::kvh_raw_imu_batch_t * msg,
                                              MavStateEstimator* state_estimator);

    bool processMessageInitAtlas(const bot_core::kvh_raw_imu_batch_t * msg,
                                 const std::map<std::string, bool> & sensors_initialized,
                                 const RBIS & default_state,
                                 const RBIM & default_cov,
                                 RBIS & init_state,
                                 RBIM & init_cov);
private:
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

};

} // namespace MavStateEst
