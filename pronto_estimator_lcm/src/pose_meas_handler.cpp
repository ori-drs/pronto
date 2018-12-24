#include "pronto_estimator_lcm/pose_meas_handler.hpp"
#include <pronto_conversions/pronto_meas_lcm.hpp>

using namespace Eigen;

namespace MavStateEst {

PoseMeasHandler::PoseMeasHandler(BotParam * param)
{
    PoseMeasConfig cfg;
    char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.pose_meas.mode");

    if (strcmp(mode_str, "position") == 0) {
        cfg.mode = PoseMeasMode::MODE_POSITION;
        std::cout << "PoseMeas will provide position measurements." << std::endl;
    }
    else if (strcmp(mode_str, "position_orient") == 0) {
        cfg.mode = PoseMeasMode::MODE_POSITION_ORIENT;
        std::cout << "PoseMeas will provide position and orientation measurements." << std::endl;
    }
    else {
        cfg.mode = PoseMeasMode::MODE_POSITION;
        std::cout << "Unrecognized PoseMeas mode. Using position mode by default." << std::endl;
    }

    cfg.number_of_corrections = bot_param_get_int_or_fail(param, "state_estimator.pose_meas.no_corrections");
    std::cout << "Apply " << cfg.number_of_corrections << " PoseMeas corrections before going silent." << std::endl;

    free(mode_str);
    cfg.r_pose_meas_xyz = bot_param_get_double_or_fail(param, "state_estimator.pose_meas.r_xyz");
    cfg.r_pose_meas_chi = bot_param_get_double_or_fail(param, "state_estimator.pose_meas.r_chi");

    pose_module_.reset(new PoseMeasModule(cfg));
}


RBISUpdateInterface * PoseMeasHandler::processMessage(const bot_core::pose_t * msg, MavStateEstimator* state_estimator)
{
    poseMeasurementFromLCM(*msg, pose_msg_);
    return pose_module_->processMessage(&pose_msg_,
                                        state_estimator);
}

bool PoseMeasHandler::processMessageInit(const bot_core::pose_t * msg,
                                         const std::map<std::string, bool> & sensors_initialized,
                                         const RBIS & default_state,
                                         const RBIM & default_cov,
                                         RBIS & init_state,
                                         RBIM & init_cov)
{

    poseMeasurementFromLCM(*msg, pose_msg_);
    return pose_module_->processMessageInit(&pose_msg_,
                                            sensors_initialized,
                                            default_state,
                                            default_cov,
                                            init_state,
                                            init_cov);
}

} // namespace MavStateEst

