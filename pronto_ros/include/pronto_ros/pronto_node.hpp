#pragma once

#include <ros/node_handle.h>
#include "pronto_ros/ros_frontend.hpp"
#include "pronto_ros/ins_ros_handler.hpp"
#include "pronto_ros/vicon_ros_handler.hpp"
#include "pronto_ros/pose_msg_ros_handler.hpp"
#include "pronto_ros/visual_odometry_ros_handler.hpp"
#include "pronto_ros/lidar_odometry_ros_handler.hpp"
#include "pronto_ros/scan_matcher_ros_handler.hpp"

namespace pronto {


template <class MsgT>
struct is_dummy_msg {
  static const bool value = false;
};

template <>
struct is_dummy_msg<std_msgs::Header>{
  static const bool value = true;
};

// use the std_msgs::Header as a placeholder for a dummy message type
template <class JointStateMsgT, class ContactStateMsgT = std_msgs::Header>
class ProntoNode {
public:
    using SensorList = std::vector<std::string>;
    using SensorSet = std::set<std::string>;
public:
    ProntoNode(ros::NodeHandle& nh,
               SensingModule<JointStateMsgT>& legodo_handler,
               DualSensingModule<sensor_msgs::Imu, JointStateMsgT>& imu_bias_lock);
    virtual void init(bool subscribe = true);
    virtual void run();

protected:
    ros::NodeHandle& nh_;
    SensingModule<JointStateMsgT>& legodo_handler_;
    DualSensingModule<sensor_msgs::Imu, JointStateMsgT>& bias_lock_handler_;
    ROSFrontEnd front_end;
    SensorList init_sensors;
    SensorList active_sensors;
    SensorSet all_sensors; // sets have unique elements
    // pointers to the modules we might want to initialize
    std::shared_ptr<InsHandlerROS> ins_handler_;
    std::shared_ptr<PoseHandlerROS> pose_handler_;
    std::shared_ptr<ViconHandlerROS> vicon_handler_;
    std::shared_ptr<VisualOdometryHandlerROS> vo_handler_;
    std::shared_ptr<LidarOdometryHandlerROS> sm_handler_;
    std::shared_ptr<ScanMatcherHandler> sm2_handler_;
};

template <class JointStateMsgT, class ContactStateMsgT>
ProntoNode<JointStateMsgT, ContactStateMsgT>::ProntoNode(ros::NodeHandle &nh,
                             SensingModule<JointStateMsgT>& legodo_handler,
                             DualSensingModule<sensor_msgs::Imu, JointStateMsgT>& imu_bias_lock) :
    nh_(nh), legodo_handler_(legodo_handler), bias_lock_handler_(imu_bias_lock), front_end(nh_)
{
    // get the list of active and init sensors from the param server
    if(!nh_.getParam("init_sensors", init_sensors)){
        ROS_ERROR("Not able to get init_sensors param");
    }

    if(!nh_.getParam("active_sensors", active_sensors)){
        ROS_ERROR("Not able to get active_sensors param");
    }
    bool publish_pose = false;
    if(!nh_.getParam("publish_pose", publish_pose)){
        ROS_WARN("Not able to get publish_pose param. Not publishing pose.");
    }
}

template <class JointStateMsgT, class ContactStateMsgT>
void ProntoNode<JointStateMsgT, ContactStateMsgT>::init(bool subscribe) {
    // parameters:
    // is the module used for init?
    // do we need to move forward the filter once computed the update?
    // do we want to publish the filter state after the message is received?
    // which topic are we listening to for this module?
    bool init = false;
    bool active = false;
    bool roll_forward = false;
    bool publish_head = false;
    std::string topic;
    std::string secondary_topic;

    for(SensorList::iterator it = active_sensors.begin(); it != active_sensors.end(); ++it){
        all_sensors.insert(*it);
    }
    for(SensorList::iterator it = init_sensors.begin(); it != init_sensors.end(); ++it){
        all_sensors.insert(*it);
    }

    // now all_sensors contain a list which is the union (without repetition)
    // of the init sensors and active sensors
    // iterate over the sensors
    for(SensorSet::iterator it = all_sensors.begin(); it != all_sensors.end(); ++it)
    {
        if(!nh_.getParam(*it + "/roll_forward_on_receive", roll_forward)){
            ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
            ROS_WARN_STREAM ("Param \"roll_forward_on_receive\" not available.");
            continue;
        }
        if(!nh_.getParam(*it + "/publish_head_on_message", publish_head)){
            ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
            ROS_WARN_STREAM ("Param \"publish_head_on_message\" not available.");
            continue;
        }
        if(!nh_.getParam(*it + "/topic", topic)){
            ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
            ROS_WARN_STREAM ("Param \"topic\" not available.");
            continue;
        }
        // check if the sensor is also used to initialize
        init = (std::find(init_sensors.begin(), init_sensors.end(), *it) != init_sensors.end());
        active = (std::find(active_sensors.begin(), active_sensors.end(), *it) != active_sensors.end());
        // is the IMU module in the list? Typically yes.
        if(it->compare("ins") == 0)
        {
            ins_handler_ = std::make_shared<InsHandlerROS>(nh_);
            if(active){
                front_end.addSensingModule(*ins_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
            if(init){
                front_end.addInitModule(*ins_handler_, *it, topic, subscribe);
            }
        }
        // is the leg odometry module in the list?
        // TODO the leg odometry handler object is costructed anyway, because
        // it is passed to the constructor as a reference. It should be
        // constructed only if we want to use it, but it is hard do handle this
        // from here.
        if(it->compare("legodo") == 0) {
            if(active){
                front_end.addSensingModule(legodo_handler_, *it, roll_forward, publish_head, topic, subscribe);
                // if secondary topic is provided, and the second message is not dummy,
                // attempt to cast the leg odometry handler as a DualHandler instead of SingleHandler
                if(!is_dummy_msg<ContactStateMsgT>::value &&
                   nh_.getParam(*it + "/secondary_topic", secondary_topic))
                {
                  try {
                    front_end.addSecondarySensingModule(dynamic_cast<DualSensingModule<JointStateMsgT,ContactStateMsgT>&>(legodo_handler_),
                                                        *it,
                                                        secondary_topic,
                                                        subscribe);
                  } catch(std::bad_cast e){
                    ROS_WARN_STREAM("Could not use the provided Leg Odometry handler as DualSensingModule<"
                                    << type_name<JointStateMsgT>() << ", " << type_name<ContactStateMsgT>() << ">.");
                    ROS_WARN_STREAM(e.what());
                  }
                }
            }
            if(init){
                front_end.addInitModule(legodo_handler_, *it, topic, subscribe);
            }
        }
        if(it->compare("pose_meas") == 0){
            pose_handler_ = std::make_shared<PoseHandlerROS>(nh_);
            if(active){
                front_end.addSensingModule(*pose_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
            if(init){
                front_end.addInitModule(*pose_handler_, *it, topic, subscribe);
            }

        }
        if(it->compare("vicon") == 0 ){
            vicon_handler_ = std::make_shared<ViconHandlerROS>(nh_);
            if(active){
                front_end.addSensingModule(*vicon_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
            if(init){
                front_end.addInitModule(*vicon_handler_, *it, topic, subscribe);
            }
        }
        if(it->compare("bias_lock") == 0){
          if(!nh_.getParam(*it + "/secondary_topic", secondary_topic)){
              ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
              ROS_WARN_STREAM ("Param \"secondary_topic\" not available.");
              continue;
          }
          if(active){
            front_end.addSensingModule(bias_lock_handler_, *it, roll_forward, publish_head, topic, subscribe);
            front_end.addSecondarySensingModule(bias_lock_handler_, *it, secondary_topic, subscribe);
          }
        }
        if(it->compare("fovis") == 0 ){
            vo_handler_ = std::make_shared<VisualOdometryHandlerROS>(nh_);
            if(active){
                front_end.addSensingModule(*vo_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
            if(init){
                front_end.addInitModule(*vo_handler_, *it, topic, subscribe);
            }
        }
        if(it->compare("scan_matcher") == 0 ){
          bool use_relative_pose = true;
          nh_.getParam(*it + "/relative_pose", use_relative_pose);
          ROS_WARN_STREAM("Scan matcher will use " << (use_relative_pose ? "relative " : "absolute ") << "pose");

          if(use_relative_pose){
            sm_handler_ = std::make_shared<LidarOdometryHandlerROS>(nh_);
            if(active){
                front_end.addSensingModule(*sm_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
            if(init){
                front_end.addInitModule(*sm_handler_, *it, topic, subscribe);
            }
          } else {
            sm2_handler_ = std::make_shared<ScanMatcherHandler>(nh_);
            if(active){
                front_end.addSensingModule(*sm2_handler_, *it, roll_forward, publish_head, topic, subscribe);
            }
            if(init){
                front_end.addInitModule(*sm2_handler_, *it, topic, subscribe);
            }
          }
        }
    }
}

template <class JointStateMsgT, class ContactStateMsgT>
void ProntoNode<JointStateMsgT, ContactStateMsgT>::run() {
    init(true);
    // you ...
    ros::spin();
    // ... me round (like a record)!
}
}
