#pragma once

#include <pronto_core/sensing_module.hpp>
#include <pronto_core/state_est.hpp>
#include <pronto_core/rotations.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <chrono>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <cxxabi.h>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Eigen>
#include <rclcpp/qos.hpp>

template<typename T>
std::string type_name()
{
    int status;
    std::string tname = typeid(T).name();
    char *demangled_name = abi::__cxa_demangle(tname.c_str(), nullptr, nullptr, &status);
    if (status == 0) {
        tname = demangled_name;
        std::free(demangled_name);
    }
    return tname;
}

namespace pronto {

inline void BlockToVector3d(const Eigen::Block<Eigen::VectorXd, 3, 1> in, geometry_msgs::msg::Vector3 & out)
{

    out.x = in(0);
    out.y = in(1);
    out.z = in(2);
}

inline void BlockToPoint(const Eigen::Block<Eigen::VectorXd, 3, 1> in, geometry_msgs::msg::Point & out)
{
    out.set__x(in(0));
    out.set__y(in(1));
    out.set__z(in(2));
}

inline void QuaternionToMsg(const Eigen::Quaterniond & in , geometry_msgs::msg::Quaternion & out)
{
    out.set__w(in.w());
    out.set__x(in.x());
    out.set__y(in.y());
    out.set__z(in.z());
}
class ROSFrontEnd {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    using SensorId = std::string;

    ROSFrontEnd(rclcpp::Node::SharedPtr nh, bool verbose = false);
    virtual ~ROSFrontEnd();

    template<class MsgT>
    void addSensingModule(SensingModule<MsgT>& module,
                          const SensorId& sensor_id,
                          bool roll_forward,
                          bool publish_head,
                          const std::string& topic,
                          bool subscribe = true);

    template<class MsgT, class SecondaryMsgT>
    inline void addSecondarySensingModule(DualSensingModule<MsgT, SecondaryMsgT>& /*module*/,
                                          const SensorId& sensor_id,
                                          const std::string& topic,
                                          bool subscribe)
    {
        if (!subscribe) {
            return;
        }

        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        RCLCPP_INFO_STREAM(nh_->get_logger(), sensor_id << " subscribing to " << topic
                                                      << " with SecondaryMsgT = " << type_name<SecondaryMsgT>());
        secondary_subscribers_[sensor_id] = nh_->create_subscription<SecondaryMsgT>(
            topic, qos,
            [this, sensor_id](typename SecondaryMsgT::UniquePtr msg) {
                this->secondaryCallback<MsgT, SecondaryMsgT>(std::move(msg), sensor_id);
            });
    }

    template <class MsgT>
    void addInitModule(SensingModule<MsgT>& module,
                       const SensorId& sensor_id,
                       const std::string& topic,
                       bool subscribe = true);

    bool areModulesInitialized();

    bool isFilterInitialized();

    inline void getState(RBIS& state, RBIM& cov) const {
        state_est_->getHeadState(state, cov);
    }

    inline bool reset(const RBIS& state, const RBIM& cov) {
        state_est_->addUpdate(new pronto::RBISResetUpdate(state,
                                                          cov,
                                                          RBISUpdateInterface::reset,
                                                          state.utime), true);
        return true;
    }

    template <class MsgT>
    void initCallback(std::shared_ptr<MsgT > msg,
                        const SensorId& Key);

    template <class PrimaryMsgT, class SecondaryMsgT>
    void secondaryCallback(std::shared_ptr<SecondaryMsgT > msg,
                            const SensorId& sensor_id);

    template <class MsgT>
    void callback(std::shared_ptr<MsgT > msg, const SensorId& Key);

protected:
    bool initializeFilter();

    void initializeState();
    void initializeCovariance();

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<StateEstimator> state_est_;
    std::map<SensorId, rclcpp::SubscriptionBase::SharedPtr> sensors_subscribers_;
    std::map<SensorId, rclcpp::SubscriptionBase::SharedPtr> secondary_subscribers_;
    std::map<SensorId, rclcpp::SubscriptionBase::SharedPtr> init_subscribers_;
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

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_msg_;
    bool publish_tf_ = false;

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg_;
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg_;

    nav_msgs::msg::Path aicp_path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr aicp_path_publisher;

    uint64_t history_span_;

    tf2::Vector3 temp_v3_;
    tf2::Quaternion temp_q_;

    bool filter_initialized_ = false;
    bool verbose_ = false;
};
}  // namespace pronto

namespace pronto {
template <class MsgT>
void ROSFrontEnd::addInitModule(SensingModule<MsgT>& module,
                                const SensorId& sensor_id,
                                const std::string& topic,
                                bool subscribe)
{
    if (init_modules_.count(sensor_id) > 0) {
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Init Module \"" << sensor_id << "\" already added. Skipping.");
        return;
    }
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Sensor init id: " << sensor_id);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Topic: " << topic);
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    // add the sensor to the list of sensor that require initialization
    std::pair<SensorId, bool> init_id_pair(sensor_id, false);
    initialized_list_.insert(init_id_pair);
    // store the module as void*, to allow for different types of module to stay
    // in the same container. The type will be known when the message arrives
    // so we can properly cast back to the right type.
    std::pair<SensorId, void*> pair(sensor_id, (void*)&module);
    init_modules_.insert(pair);
    if (subscribe) {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), sensor_id << " subscribing to " << topic);
        RCLCPP_ERROR_STREAM(nh_->get_logger(), " with MsgT = " << type_name<MsgT>());
        init_subscribers_[sensor_id] = nh_->create_subscription<MsgT>(
            topic, qos,
            [this, sensor_id](typename MsgT::UniquePtr msg) {
                this->initCallback<MsgT>(std::move(msg), sensor_id);
            });
    }
}

template <class MsgT>
void ROSFrontEnd::addSensingModule(SensingModule<MsgT>& module,
                                   const SensorId& sensor_id,
                                   bool roll_forward,
                                   bool publish_head,
                                   const std::string& topic,
                                   bool subscribe)
{
    // int this implementation we allow only one different type of module
    if (active_modules_.count(sensor_id) > 0) {
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Sensing Module \"" << sensor_id << "\" already added. Skipping.");
        return;
    }

    RCLCPP_INFO_STREAM(nh_->get_logger(), "Sensor id: " << sensor_id);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Roll forward: " << (roll_forward ? "yes" : "no"));
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Publish head: " << (publish_head ? "yes" : "no"));
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Topic: " << topic);
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    // store the will to roll forward when the message is received
    std::pair<SensorId, bool> roll_pair(sensor_id, roll_forward);
    roll_forward_.insert(roll_pair);

    // store the will to publish the estimator state when the message is received
    std::pair<SensorId, bool> publish_pair(sensor_id, publish_head);
    publish_head_.insert(publish_pair);

    // store the module as void*, to allow for different types of module to stay
    // in the same container. The type will be known when the message arrives
    // so we can properly cast back to the right type.
    std::pair<SensorId, void*> pair(sensor_id, (SensingModule<MsgT>*)&module);
    active_modules_.insert(pair);
    // subscribe the generic templated callback for all modules
    if (subscribe) {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), sensor_id << " subscribing to " << topic
                                                         << " with MsgT = " << type_name<MsgT>());
        sensors_subscribers_[sensor_id] = nh_->create_subscription<MsgT>(
            topic, qos,
            [this, sensor_id](typename MsgT::UniquePtr msg) {
                this->callback<MsgT>(std::move(msg), sensor_id);
            });
    }
}


template <class MsgT>
void ROSFrontEnd::initCallback(std::shared_ptr<MsgT> msg, const SensorId& sensor_id)
{
    if(verbose_){
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Init callback for sensor " << sensor_id);
    }
    if(initialized_list_.count(sensor_id) > 0 && !initialized_list_[sensor_id])
    {
        initialized_list_[sensor_id] = static_cast<SensingModule<MsgT>*>(init_modules_[sensor_id])->processMessageInit(
            msg.get(),
            initialized_list_,
            default_state,
            default_cov,
            init_state,
            init_cov);

        // if the sensor has been successfully initialized, we unsubscribe.
        // This happens only for the sensors which are only for initialization.
        // The sensor which are for initialization and active will continue to listen
        if(initialized_list_[sensor_id]){
            init_subscribers_[sensor_id].reset();
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
            init_subscribers_[sensor_id].reset();
        }
    }
}

// TODO come up with a better way to activate / deactivate debug mode
#define DEBUG_MODE 0

template <class MsgT>
void ROSFrontEnd::callback(std::shared_ptr<MsgT> msg, const SensorId& sensor_id)
{
#if DEBUG_MODE
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Callback for sensor " << sensor_id);
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
        RBISUpdateInterface* update = static_cast<SensingModule<MsgT>*>(active_modules_[sensor_id])->processMessage(
            msg.get(),
            state_est_.get());
#if DEBUG_MODE
        auto end = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Time elapsed process message: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        start = end;
#endif
        // if the update is invalid, we leave
        if(update == nullptr){
#if DEBUG_MODE
            RCLCPP_INFO_STREAM(nh_->get_logger(), "Invalid " << sensor_id << " update" << std::endl);
#endif
            // special case for pose meas, it returns null when it does not want
            // to process data anymore
            if(sensor_id.compare("pose_meas") == 0){
                sensors_subscribers_["pose_meas"].reset();
            }
            return;
        }
#if DEBUG_MODE
        if(sensor_id.compare("fovis") == 0){
          RCLCPP_INFO_STREAM(nh_->get_logger(), "fovis update posterior: " << update->posterior_state.position().transpose());
        }
#endif
#define DEBUG_AICP 0
#if DEBUG_AICP

        if(sensor_id.compare("scan_matcher") == 0){
          aicp_path.header.frame_id = "odom";
          Eigen::Vector3d p = dynamic_cast<RBISIndexedPlusOrientationMeasurement*>(update)->measurement.head<3>();
          Eigen::Quaterniond q = dynamic_cast<RBISIndexedPlusOrientationMeasurement*>(update)->orientation;

          std::cerr << "MEASR    : " << p.transpose() << "   " << rotation::getEulerAnglesDeg(q).transpose() << std::endl;

         // Eigen::Vector3d p = dynamic_cast<RBISIndexedMeasurement*>(update)->measurement.head<3>();

          // std::cerr << "ABOUT TO SEND TO FILTER THE FOLLOWING: " << p.transpose() << std::endl;

        }
#endif
        RBIS prior;
        RBIS posterior;
        RBIM prior_cov;
        RBIM posterior_cov;
        state_est_->getHeadState(prior,prior_cov);
        // tell also the filter if we need to roll forward
        state_est_->addUpdate(update, roll_forward_[sensor_id]);
        state_est_->getHeadState(posterior,posterior_cov);

#if DEBUG_MODE
        if(sensor_id.compare("scan_matcher") == 0){
            RCLCPP_INFO_STREAM(nh_->get_logger(), "PRIOR    : " << prior.position().transpose() << " " << rotation::getEulerAnglesDeg(prior.orientation()).transpose());
            RCLCPP_INFO_STREAM(nh_->get_logger(), "POSTERIOR: " << posterior.position().transpose() << " " << rotation::getEulerAnglesDeg(posterior.orientation()).transpose());
            RCLCPP_INFO_STREAM(nh_->get_logger(), ":::::::");
        }
#endif
        if(publish_head_[sensor_id]){
            state_est_->getHeadState(head_state, head_cov);

            // fill in linear velocity
            BlockToVector3d(head_state.velocity(),  twist_msg_.twist.twist.linear);
            // tf2::convert(head_state.velocity,twist_msg_.twist.twist.linear);
            // fill in angular velocity
            BlockToVector3d(head_state.angularVelocity(), twist_msg_.twist.twist.angular);
            // tf2::convert(head_state.angularVelocity, twist_msg_.twist.twist.angular);

            // fill in time
            twist_msg_.header.stamp = rclcpp::Time(head_state.utime * 1000);

            // TODO insert appropriate covariance into the message

            // set twist covariance to zero
            twist_msg_.twist.covariance.fill(0);

            Eigen::Block<RBIM, 3, 3> vel_cov = head_cov.block<3,3>(RBIS::velocity_ind,RBIS::velocity_ind);
            Eigen::Block<RBIM, 3, 3> omega_cov = head_cov.block<3,3>(RBIS::angular_velocity_ind,RBIS::angular_velocity_ind);

            for(int i=0; i<3; i++){
              for(int j=0; j<3; j++){
                twist_msg_.twist.covariance[6*i+j] = vel_cov(i,j);
                twist_msg_.twist.covariance[6*(i+3)+j+3] = omega_cov(i,j);
              }
            }

            // publish the twist
            twist_pub_->publish(twist_msg_);

            // make sure stuff is non-NAN before publishing
            assert(head_state.position().allFinite());

            // fill in message position
            BlockToPoint(head_state.position(), pose_msg_.pose.pose.position);

            // fill in message orientation
            // pose_msg_.pose.pose.orientation = tf2::toMsg(head_state.orientation()); // this is a quaternion
            QuaternionToMsg(head_state.orientation(), pose_msg_.pose.pose.orientation);

            // fill in time
           pose_msg_.header.stamp = rclcpp::Time(head_state.utime * 1000);
            if(publish_tf_){
                // Only publish the pose if the timestamp is different:
                // This prevents issues in Noetic where repeated warnings of type:
                // "TF_REPEATED_DATA ignoring data with redundant timestamp for frame base at time"
                // are otherwise printed to the terminal.
                // Cf. https://github.com/ros/geometry2/issues/467#issuecomment-751572836
                rclcpp::Time new_stamp = pose_msg_.header.stamp;
                if (new_stamp > transform_msg_.header.stamp) {
                    transform_msg_.transform.translation.x = pose_msg_.pose.pose.position.x;
                    transform_msg_.transform.translation.y = pose_msg_.pose.pose.position.y;
                    transform_msg_.transform.translation.z = pose_msg_.pose.pose.position.z;
                    transform_msg_.transform.rotation = pose_msg_.pose.pose.orientation;
                    transform_msg_.header.stamp = new_stamp;
                    tf2_broadcaster_->sendTransform(transform_msg_);
                }
            }

            // TODO insert appropriate covariance into the message
            // publish the pose
            pose_pub_->publish(pose_msg_);
        }
#if DEBUG_MODE
        else {
            RCLCPP_WARN(nh_->get_logger(), "NOT Publish head sensor ID");
        }
        end = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO_STREAM(nh_->get_logger(), "Time elapsed till the end: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        std::cout << std::endl;
#endif
    }
}

template <class PrimaryMsgT, class SecondaryMsgT>
void ROSFrontEnd::secondaryCallback(std::shared_ptr<SecondaryMsgT > msg, const SensorId& sensor_id)
{
    auto a = dynamic_cast<DualSensingModule<PrimaryMsgT,SecondaryMsgT>*>(static_cast<SensingModule<PrimaryMsgT>*>(active_modules_[sensor_id]));
    a->processSecondaryMessage(*msg);
}

} // namespace pronto
