#include <pronto_msgs/JointStateWithAcceleration.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

ros::Publisher pub;

void jointStateWithAccelerationCallback(const pronto_msgs::JointStateWithAcceleration::ConstPtr& msg)
{
    sensor_msgs::JointState out;
    out.header = msg->header;
    out.name = msg->name;
    out.position = msg->position;
    out.velocity = msg->velocity;
    out.effort = msg->effort;

    pub.publish(out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_state_with_acceleration_republisher");

    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe<pronto_msgs::JointStateWithAcceleration>("joint_states_with_acceleration", 10, jointStateWithAccelerationCallback, ros::TransportHints().tcpNoDelay());
    pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    ros::spin();

    return EXIT_SUCCESS;
}
