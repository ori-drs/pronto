#include "mav_state_est/ros_frontend.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

using SensorKey = MavStateEst::ROSFrontEnd::SensorKey;

namespace MavStateEst {
ROSFrontEnd::ROSFrontEnd(ros::NodeHandle &nh) : nh_(nh) {
    std::string prefix = "/state_estimator_pronto/";
    bool publish_pose = false;
    ROS_ASSERT(nh_.getParam(prefix + "publish_pose", publish_pose));

    if(publish_pose){
        std::string pose_topic = "POSE_BODY";
        ROS_ASSERT(nh_.getParam(prefix + "pose_topic", pose_topic));
        std::string twist_topic = "POSE_BODY";
        ROS_ASSERT(nh_.getParam(prefix + "twist_topic", twist_topic));
        pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 200);
        twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>(twist_topic, 200);
    }
    uint64_t history_span;
    ROS_ASSERT(nh_.getParam(prefix + "utime_history_span", history_span));
    RBISResetUpdate * update;
    // TODO fill in reset update
    state_est_.reset(new MavStateEstimator(update, history_span));
}


ROSFrontEnd::~ROSFrontEnd()
{
}

template <class MsgT, SensorKey K>
void ROSFrontEnd::addInitModule(SensingModule<MsgT>& module){
    std::pair<SensorKey, bool> pair(K, false);
    init_modules_.insert(pair);
}

template<class MsgT, SensorKey K>
void ROSFrontEnd::addSensingModule(SensingModule<MsgT>& module)
{
    std::pair<SensorKey, void*> pair(K, (void*) &module);
    sensing_modules_.insert(pair);
}


template <class MsgT, SensorKey Key>
void ROSFrontEnd::callback(boost::shared_ptr<MsgT const> msg)
{
    if(!init_modules_[Key]){
        init_modules_[Key] = dynamic_cast<SensingModule<MsgT>*>(sensing_modules_[Key])->processMessageInit(msg.get(),
                                                                                                           init_modules_,
                                                                                                           default_state,
                                                                                                           default_cov,
                                                                                                           init_state,
                                                                                                           init_cov);
    } else {
    RBISUpdateInterface* update = dynamic_cast<SensingModule<MsgT>*>(sensing_modules_[Key])->processMessage(msg.get(), &state_est_);

    if(update != NULL){
        state_est_->addUpdate(update, true);
    }
    }


}


}
