#include <ros/node_handle.h>
#include "pronto_estimator_ros/ros_frontend.hpp"
#include "pronto_estimator_ros/ins_ros_handler.hpp"

using namespace MavStateEst;


int main(int argc, char** argv) {
    ros::init(argc, argv, "pronto_estimator_ros_node");
    std::string prefix = "/state_estimator_pronto";
    ros::NodeHandle nh(prefix);
    ROSFrontEnd front_end(nh);

    typedef std::vector<std::string> SensorList;
    SensorList init_sensors;
    SensorList active_sensors;

    if(!nh.getParam("init_sensors", init_sensors)){
        ROS_ERROR("Not able to get init_sensors param");
    }

    if(!nh.getParam("active_sensors", active_sensors)){
        ROS_ERROR("Not able to get active_sensors param");
    }
    bool publish_pose = false;
    if(!nh.getParam("publish_pose", publish_pose)){
        ROS_WARN("Not able to get publish_pose param. Not publishing pose.");
    }
    std::vector<std::string> param_names;
    nh.getParamNames(param_names);

    std::shared_ptr<InsHandlerROS> ins_handler_;
    // std::shared_ptr<LegOdometryHandlerROS> legodo_handler_;

    bool init = false;
    bool roll_forward = false;
    bool publish_head = false;
    std::string topic;

    for(SensorList::iterator it = active_sensors.begin(); it != active_sensors.end(); ++it)
    {
        if(!nh.getParam(*it + "/roll_forward_on_receive", roll_forward)){
            ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
            ROS_WARN_STREAM ("Param \"roll_forward_on_receive\" not available.");
            continue;
        }
        if(!nh.getParam(*it + "/publish_head_on_message", publish_head)){
            ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
            ROS_WARN_STREAM ("Param \"publish_head_on_message\" not available.");
            continue;
        }
        if(!nh.getParam(*it + "/topic", topic)){
            ROS_WARN_STREAM("Not adding sensor \"" << *it << "\".");
            ROS_WARN_STREAM ("Param \"topic\" not available.");
            continue;
        }
        // check if the sensor is also used to initialize
        if(std::find(init_sensors.begin(), init_sensors.end(), *it) != init_sensors.end()){
            init = true;
        }
        if(it->compare("ins") == 0)
        {
            ins_handler_.reset(new InsHandlerROS(nh));
            front_end.addSensingModule(*ins_handler_, *it, init, roll_forward, publish_head, topic);
        }
        if(it->compare("legodo") == 0){

        }
    }

    ros::spin();
}
