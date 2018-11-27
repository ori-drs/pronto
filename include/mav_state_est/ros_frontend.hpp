#pragma once

#include "mav_state_est/sensing_module.hpp"
#include "mav_state_est/mav_state_est.hpp"
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>


namespace MavStateEst {
class ROSFrontEnd {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    using SensorKey = RBISUpdateInterface::sensor_enum;

    ROSFrontEnd(ros::NodeHandle& nh);
    virtual ~ROSFrontEnd();

    template <class MsgT, SensorKey K>
    void addInitModule(SensingModule<MsgT>& module);

    template<class MsgT, SensorKey K>
    void addSensingModule(SensingModule<MsgT>& module);



    template <class MsgT, SensorKey K>
    void callback(boost::shared_ptr<MsgT const> msg);

    template <class MsgT, SensorKey K>
    void initCallback(boost::shared_ptr<MsgT const> msg);

private:
    ros::NodeHandle& nh_;
    std::shared_ptr<MavStateEstimator> state_est_;
    std::vector<ros::Subscriber> subscribers_;
    std::map<SensorKey, void*> sensing_modules_;
    std::map<SensorKey, bool> init_modules_;

    RBIS default_state;
    RBIM default_cov;

    RBIS init_state;
    RBIM init_cov;

    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;

};
}
