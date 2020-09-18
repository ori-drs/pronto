#pragma once
#include <sensor_msgs/JointState.h>
#include <pronto_quadruped_commons/declarations.h>

namespace pronto {
namespace quadruped {

bool jointStateFromROS(const sensor_msgs::JointState& msg,
                       uint64_t& utime,
                       JointState& q,
                       JointState& qd,
                       JointState& qdd,
                       JointState& tau);

}
}
