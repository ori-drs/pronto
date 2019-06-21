#pragma once

#include <pronto_core/sensing_module.hpp>
#include <pronto_core/state_est.hpp>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <chrono>
#include <tf/transform_broadcaster.h>

namespace pronto {
class ROSFrontEnd {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    using SensorId = std::string;

    ROSFrontEnd(ros::NodeHandle &nh, bool verbose = false);
    virtual ~ROSFrontEnd();

    template<class MsgT>
    void addSensingModule(SensingModule<MsgT>& module,
                          const SensorId& sensor_id,
                          bool roll_forward,
                          bool publish_head,
                          const std::string& topic,
                          bool subscribe = true);
    template <class MsgT>
    void addInitModule(SensingModule<MsgT>& module,
                       const SensorId& sensor_id,
                       const std::string& topic,
                       bool subscribe = true);

    bool areModulesInitialized();

    bool isFilterInitialized();

    inline void getState(RBIS& state, RBIM& cov) const{
        state_est_->getHeadState(state, cov);
    }

    inline bool reset(const RBIS& state, const RBIM& cov) {
        state_est_->addUpdate(new pronto::RBISResetUpdate(state,
                                                               cov,
                                                               RBISUpdateInterface::reset,
                                                               state.utime), true);
    }
    template <class MsgT>
    void initCallback(boost::shared_ptr<MsgT const> msg,
                      const SensorId& Key);

    template <class MsgT>
    void callback(boost::shared_ptr<MsgT const> msg,
                  const SensorId& Key);
protected:
    bool initializeFilter();

    void initializeState();
    void initializeCovariance();


private:
    ros::NodeHandle& nh_;
    std::shared_ptr<StateEstimator> state_est_;
    std::map<SensorId, ros::Subscriber> sensors_subscribers_;
    std::map<SensorId, ros::Subscriber> init_subscribers_;
    std::map<SensorId, void*> active_modules_;
    std::map<SensorId, void*> init_modules_;
    std::map<SensorId, bool> initialized_list_;
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
    tf::TransformBroadcaster tf_broadcaster_;
    tf::StampedTransform tf_pose_;
    bool publish_tf_ = false;

    geometry_msgs::PoseWithCovarianceStamped pose_msg_;
    geometry_msgs::TwistWithCovarianceStamped twist_msg_;

    uint64_t history_span_;

    tf::Vector3 temp_v3;
    tf::Quaternion temp_q;

    bool filter_initialized_ = false;
    bool verbose_ = false;


};
}

namespace pronto {


template <class MsgT>
void ROSFrontEnd::addInitModule(SensingModule<MsgT>& module,
                                const SensorId& sensor_id,
                                const std::string& topic,
                                bool subscribe)
{
    if(init_modules_.count(sensor_id) > 0){
        ROS_WARN_STREAM("Init Module \"" << sensor_id << "\" already added. Skipping.");
        return;
    }
    ROS_INFO_STREAM("Sensor init id: " << sensor_id);
    ROS_INFO_STREAM("Topic: " << topic);

    // add the sensor to the list of sensor that require initialization
    std::pair<SensorId, bool> init_id_pair(sensor_id, false);
    initialized_list_.insert(init_id_pair);
    // store the module as void*, to allow for different types of module to stay
    // in the same container. The type will be known when the message arrives
    // so we can properly cast back to the right type.
    std::pair<SensorId, void*> pair(sensor_id, (void*) &module);
    init_modules_.insert(pair);
    if(subscribe){
        init_subscribers_[sensor_id] = nh_.subscribe<MsgT>(topic,
                                                           10000,
                                                           boost::bind(&ROSFrontEnd::initCallback<MsgT>,
                                                                       this,
                                                                       _1,
                                                                       sensor_id),
                                                           ros::VoidConstPtr(),
                                                           ros::TransportHints().tcpNoDelay());
    }
}

template<class MsgT>
void ROSFrontEnd::addSensingModule(SensingModule<MsgT>& module,
                                   const SensorId& sensor_id,
                                   bool roll_forward,
                                   bool publish_head,
                                   const std::string& topic,
                                   bool subscribe)
{
    // int this implementation we allow only one different type of module
    if(active_modules_.count(sensor_id) > 0){
        ROS_WARN_STREAM("Sensing Module \"" << sensor_id << "\" already added. Skipping.");
        return;
    }

    ROS_INFO_STREAM("Sensor id: " << sensor_id);
    ROS_INFO_STREAM("Roll forward: "<< (roll_forward? "yes" : "no"));
    ROS_INFO_STREAM("Publish head: "<< (publish_head? "yes" : "no"));
    ROS_INFO_STREAM("Topic: " << topic);

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
    active_modules_.insert(pair);
    // subscribe the generic templated callback for all modules
    if(subscribe){
        sensors_subscribers_[sensor_id] = nh_.subscribe<MsgT>(topic,
                                                              10000,
                                                              boost::bind(&ROSFrontEnd::callback<MsgT>,
                                                                          this, _1, sensor_id),
                                                              ros::VoidConstPtr(),
                                                              ros::TransportHints().tcpNoDelay());
    }
}



template <class MsgT>
void ROSFrontEnd::initCallback(boost::shared_ptr<MsgT const> msg, const SensorId& sensor_id){
    if(verbose_){
        ROS_INFO_STREAM("Init callback for sensor " << sensor_id);
    }
    if(initialized_list_.count(sensor_id) > 0 && !initialized_list_[sensor_id])
    {
        initialized_list_[sensor_id] = static_cast<SensingModule<MsgT>*>(init_modules_[sensor_id])->processMessageInit(msg.get(),
                                                                                                                       initialized_list_,
                                                                                                                       default_state,
                                                                                                                       default_cov,
                                                                                                                       init_state,
                                                                                                                       init_cov);

        // if the sensor has been successfully initialized, we unsubscribe.
        // This happens only for the sensors which are only for initialization.
        // The sensor which are for initialization and active will continue to listen
        if(initialized_list_[sensor_id]){
            init_subscribers_[sensor_id].shutdown();
            // attempt to initialize the filter, because the value has changed
            // in the list
            initializeFilter();
        }
    } else {
        // if we are here it means that the module is not in the list of
        // initialized modules or that the module is already initialized
        // in both cases we don't want to subscribe to this topic anymore,
        // unless there is no subscriber because we are processing a rosbag.
        if(init_subscribers_.count(sensor_id) > 0){
            init_subscribers_[sensor_id].shutdown();
        }
    }
}

//TODO come up with a better way to activate / deactivate debug mode
#define DEBUG_MODE 0

template <class MsgT>
void ROSFrontEnd::callback(boost::shared_ptr<MsgT const> msg, const SensorId& sensor_id)
{
#if DEBUG_MODE
        ROS_INFO_STREAM("Callback for sensor " << sensor_id);
#endif
    // this is a generic templated callback that does the same for every module:
    // if the module is initialized and the filter is ready
    // 1) take the measurement update and pass it to the filter if valid
    // 2) publish the filter state if the module wants to
    if(isFilterInitialized()) {
        // appropriate casting to the right type and call to the process message
        // function to get the update
        // Record start time
#if DEBUG_MODE
        auto start = std::chrono::high_resolution_clock::now();
#endif
        RBISUpdateInterface* update = static_cast<SensingModule<MsgT>*>(active_modules_[sensor_id])->processMessage(msg.get(), state_est_.get());
#if DEBUG_MODE
        auto end = std::chrono::high_resolution_clock::now();
        ROS_INFO_STREAM("Time elapsed process message: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        start = end;
#endif
        // if the update is invalid, we leave
        if(update == nullptr){
            ROS_INFO_STREAM("Invalid " << sensor_id << " update" << std::endl);
            // special case for pose meas, it returns null when it does not want
            // to process data anymore
            if(sensor_id.compare("pose_meas") == 0){
                sensors_subscribers_["pose_meas"].shutdown();
            }
            return;
        }
        // tell also the filter if we need to roll forward
        state_est_->addUpdate(update, roll_forward_[sensor_id]);
#if DEBUG_MODE
        end = std::chrono::high_resolution_clock::now();
        ROS_INFO_STREAM("Time elapsed process addupdate: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        start = end;
#endif
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
            if(publish_tf_){
                tf_pose_.setOrigin(temp_v3);
                tf_pose_.setRotation(temp_q);
                tf_pose_.stamp_ = ros::Time::now();
                tf_broadcaster_.sendTransform(tf_pose_);
            }

            // TODO insert appropriate covariance into the message
            // publish the pose
            pose_pub_.publish(pose_msg_);
        }
#if DEBUG_MODE
        end = std::chrono::high_resolution_clock::now();

        ROS_INFO_STREAM("Time elapsed till the end: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        std::cout << std::endl;
#endif
    }
}

} // namespace pronto
