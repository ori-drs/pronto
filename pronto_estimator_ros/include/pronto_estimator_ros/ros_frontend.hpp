#pragma once

#include "pronto_estimator_core/sensing_module.hpp"
#include "pronto_estimator_core/mav_state_est.hpp"
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf_conversions/tf_eigen.h>


namespace MavStateEst {
class ROSFrontEnd {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    using SensorId = std::string;

    ROSFrontEnd(ros::NodeHandle &nh);
    virtual ~ROSFrontEnd();

    void addInitModule(const std::string& sensor_id);

    template<class MsgT>
    void addSensingModule(SensingModule<MsgT>& module,
                          const SensorId& Key,
                          bool is_init_module,
                          bool roll_forward,
                          bool publish_head,
                          const std::string& topic);

    bool areModulesInitialized();

    bool isFilterInitialized();


protected:
    template <class MsgT>
    void callback(boost::shared_ptr<MsgT const> msg,
                  const SensorId& Key);

    bool initializeFilter();


private:
    ros::NodeHandle& nh_;
    std::shared_ptr<MavStateEstimator> state_est_;
    std::map<SensorId, ros::Subscriber> subscribers_;
    std::map<SensorId, void*> sensing_modules_;
    std::map<SensorId, bool> init_modules_;
    std::map<SensorId, bool> roll_forward_;
    std::map<SensorId, bool> publish_head_;

    RBIS default_state;
    RBIM default_cov;

    RBIS init_state;
    RBIM init_cov;

    RBIS head_state;
    RBIM head_cov;

    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;

    geometry_msgs::PoseWithCovarianceStamped pose_msg_;
    geometry_msgs::TwistWithCovarianceStamped twist_msg_;

    uint64_t history_span_;

    tf::Vector3 temp_v3;
    tf::Quaternion temp_q;

    bool filter_initialized_ = false;


};
}

namespace MavStateEst {
using SensorId = MavStateEst::ROSFrontEnd::SensorId;

ROSFrontEnd::ROSFrontEnd(ros::NodeHandle& nh) : nh_(nh) {
    std::string prefix = "/state_estimator_pronto/";
    bool publish_pose;
    if(nh_.getParam(prefix + "publish_pose", publish_pose)){
        if(publish_pose){
            std::string pose_topic = "POSE_BODY";
            if(nh_.getParam(prefix + "pose_topic", pose_topic)){
                pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/" + pose_topic, 200);
            }
            std::string twist_topic = "TWIST_BODY";
            if(nh_.getParam(prefix + "twist_topic", twist_topic)){
                twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/" + twist_topic, 200);
            }
        }
        int history_span;
        nh_.getParam(prefix + "utime_history_span", history_span);
        history_span_ = (uint64_t)history_span;
    }
}


ROSFrontEnd::~ROSFrontEnd()
{
}

void ROSFrontEnd::addInitModule(const std::string& sensor_id)
{
    // add the sensor to the list of sensor that require initialization
    std::pair<SensorId, bool> init_id_pair(sensor_id, false);
    init_modules_.insert(init_id_pair);
}

template<class MsgT>
void ROSFrontEnd::addSensingModule(SensingModule<MsgT>& module,
                                   const SensorId& sensor_id,
                                   bool is_init_module,
                                   bool roll_forward,
                                   bool publish_head,
                                   const std::string& topic)
{
    // int this implementation we allow only one different type of module
    if(sensing_modules_.count(sensor_id) > 0){
        ROS_WARN_STREAM("Sensing Module \"" << sensor_id << "\" already added. Skipping.");
        return;
    }

    ROS_INFO_STREAM("Sensor id: " << sensor_id);
    ROS_INFO_STREAM("Init module: "<< (is_init_module? "yes" : "no"));
    ROS_INFO_STREAM("Roll forward: "<< (roll_forward? "yes" : "no"));
    ROS_INFO_STREAM("Publish head: "<< (publish_head? "yes" : "no"));
    ROS_INFO_STREAM("Topic: " << topic);

    if(is_init_module){
        addInitModule(sensor_id);
    }

    // store the will to roll forward when the message is received
    std::pair<SensorId, bool> roll_pair(sensor_id, roll_forward);
    roll_forward_.insert(roll_pair);

    // store the will to publish the estimator state when the message is received
    std::pair<SensorId, bool> publish_pair(sensor_id, publish_head);
    publish_head_.insert(publish_pair);

    // store the module as void*, to allow for different types of module to stay
    // in the same container. The type will be known when the message arrives
    // so we can properly cast back to the right type.
    std::pair<SensorId, void*> pair(sensor_id, (void*) &module);
    sensing_modules_.insert(pair);
    // subscribe the generic templated callback for all modules
    subscribers_[sensor_id] = nh_.subscribe<MsgT>(topic, 100, boost::bind(&ROSFrontEnd::callback<MsgT>, this, _1, sensor_id));


}

bool ROSFrontEnd::initializeFilter()
{
    // if the modules are not ready we return false
    if(!areModulesInitialized())
    {
        return false;
    }
    // if the filter is already initialized we quietly return
    if(isFilterInitialized()){
        return true;
    }
    state_est_.reset(new MavStateEstimator(new RBISResetUpdate(init_state,
                                                               init_cov,
                                                               RBISUpdateInterface::reset,
                                                               init_state.utime),
                                           history_span_));

    filter_initialized_ = true;
    return true;
}

bool ROSFrontEnd::areModulesInitialized(){
    std::map<SensorId,bool>::iterator it = init_modules_.begin();
    for(; it != init_modules_.end(); ++it){
        if(!it->second){
            return false;
        }
    }
    return true;
}

bool ROSFrontEnd::isFilterInitialized(){
    return filter_initialized_;
}


template <class MsgT>
void ROSFrontEnd::callback(boost::shared_ptr<MsgT const> msg, const SensorId& sensor_id)
{
    // this is a generic templated callback that does the same for every module:
    // If the module is in the initialization list and hasn't been initialized yet:
    // 1) execute the initialization callback
    // 2) attempt to inizialize the filter
    // if the module is initialized and the filter is ready
    // 1) take the measurement update and pass it to the filter if valid
    // 2) publish the filter state if the module wants to
    // if the module is in the initialization list but it is not initialized yet
    if(init_modules_.count(sensor_id) > 0 && !init_modules_[sensor_id])
    {
        init_modules_[sensor_id] = static_cast<SensingModule<MsgT>*>(sensing_modules_[sensor_id])->processMessageInit(msg.get(),
                                                                                                           init_modules_,
                                                                                                           default_state,
                                                                                                           default_cov,
                                                                                                           init_state,
                                                                                                           init_cov);
        // attempt to initialize the filter
        // (all the init modules should have initialized first)
        initializeFilter();
    } else if(isFilterInitialized()) {
        // appropriate casting to the right type and call to the process message
        // function to get the update
        RBISUpdateInterface* update = static_cast<SensingModule<MsgT>*>(sensing_modules_[sensor_id])->processMessage(msg.get(), state_est_.get());

        // if the update is valid, pass it to the filter
        if(update != NULL){
            // tell also the filter if we need to roll forward
            state_est_->addUpdate(update, roll_forward_[sensor_id]);
        }

        if(publish_head_[sensor_id]){

            state_est_->getHeadState(head_state, head_cov);

            // fill in linear velocity
            tf::vectorEigenToTF(head_state.velocity(),temp_v3);
            tf::vector3TFToMsg(temp_v3,twist_msg_.twist.twist.linear);
            // fill in angular velocity
            tf::vectorEigenToTF(head_state.angularVelocity(),temp_v3);
            tf::vector3TFToMsg(temp_v3,twist_msg_.twist.twist.angular);

            // fill in time
            twist_msg_.header.stamp = ros::Time().fromNSec(head_state.utime * 1000);

            // TODO insert appropriate covariance into the message
            // publish the twist
            twist_pub_.publish(twist_msg_);




            // fill in message position
            tf::vectorEigenToTF(head_state.position(), temp_v3);
            tf::pointTFToMsg(temp_v3, pose_msg_.pose.pose.position);

            // fill in message orientation
            tf::quaternionEigenToTF(head_state.orientation(), temp_q);
            tf::quaternionTFToMsg(temp_q,pose_msg_.pose.pose.orientation);

            // fill in time
            pose_msg_.header.stamp = ros::Time().fromNSec(head_state.utime * 1000);

            // TODO insert appropriate covariance into the message
            // publish the pose
            pose_pub_.publish(pose_msg_);
        }

    }


}
} // namespace MavStateEst
