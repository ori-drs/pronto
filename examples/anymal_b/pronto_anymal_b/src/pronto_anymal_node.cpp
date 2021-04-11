#include <ros/node_handle.h>
#include <pronto_quadruped_ros/stance_estimator_ros.hpp>
#include <pronto_quadruped_ros/leg_odometer_ros.hpp>
#include <sensor_msgs/JointState.h>
#include <pronto_anymal_b_commons/feet_contact_forces.hpp>
#include <pronto_anymal_b_commons/feet_jacobians.hpp>
#include <pronto_anymal_b_commons/forward_kinematics.hpp>

#include <pronto_quadruped_ros/bias_lock_handler_ros.hpp>
#include <pronto_quadruped_ros/legodo_handler_ros.hpp>

#include <pronto_ros/pronto_node.hpp>

using namespace pronto;
using namespace pronto::anymal;
using namespace pronto::quadruped;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pronto_anymal_node");
    std::string prefix = "/state_estimator_pronto";
    ros::NodeHandle nh(prefix);

    anymal::FeetContactForces feet_forces;
    anymal::FeetJacobians feet_jacs;
    anymal::ForwardKinematics fwd_kin;

    // stance estimator and leg odometer from RA-L 2017
    StanceEstimatorROS stance_estimator_(nh, feet_forces);
    LegOdometerROS leg_odometer_(nh, feet_jacs, fwd_kin);
    ImuBiasLockROS bias_lock(nh);

    LegodoHandlerROS legodo_handler(nh, stance_estimator_, leg_odometer_);
    ProntoNode<sensor_msgs::JointState> node(nh, legodo_handler, bias_lock);
    node.run();
}

