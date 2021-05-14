#include "pronto_quadruped_ros/conversions.hpp"
#include <ros/console.h>

namespace pronto {
namespace quadruped {

bool jointStateFromROS(const sensor_msgs::JointState& msg,
                       uint64_t& utime,
                       JointState& q,
                       JointState& qd,
                       JointState& qdd,
                       JointState& tau)
{
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    if (static_cast<int>(static_cast<const sensor_msgs::JointState&>(msg).position.size()) != q.size() ||
        static_cast<int>(static_cast<const sensor_msgs::JointState&>(msg).velocity.size()) != q.size() ||
        static_cast<int>(static_cast<const sensor_msgs::JointState&>(msg).effort.size()) != q.size()){
        ROS_WARN_STREAM_THROTTLE(1, "Joint State is expected " << \
                                 q.size() << " joints but "\
                                 << msg.position.size() << " / " << msg.velocity.size() << " / " << msg.effort.size()
                                 << " are provided.");
        return false;
    }
    // store message time in microseconds
    utime = msg.header.stamp.toNSec() / 1000;
    for(int i=0; i<12; i++){
      q(i) = msg.position[i];
      qd(i) = msg.velocity[i];
      tau(i) = msg.effort[i];
    }
    //q = Eigen::Map<const JointState>(msg.position.data());
    //qd = Eigen::Map<const JointState>(msg.velocity.data());
    //tau = Eigen::Map<const JointState>(msg.effort.data());

    qdd.setZero(); // TODO compute the acceleration

    return true;
}

}  // namespace quadruped
}  // namespace pronto
