#include <ros/node_handle.h>
#include "pronto_estimator_ros/ros_frontend.hpp"
#include <pronto_estimator_ros/ins_ros_handler.hpp>

using namespace MavStateEst;

int main(int argc, char** argv) {

    ros::init(argc, argv, "pronto_estimator_ros_node");
    ros::NodeHandle nh;
    ROSFrontEnd front_end(nh);

    InsHandlerROS ins(nh);

    front_end.addSensingModule<sensor_msgs::Imu, RBISUpdateInterface::ins>(ins,
                                                         true,
                                                         true,
                                                         "/sensors/imu");
    front_end.addInitModule<RBISUpdateInterface::ins>("ins");

    ros::spin();


}
