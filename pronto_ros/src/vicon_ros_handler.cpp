#include "pronto_ros/vicon_ros_handler.hpp"
#include "pronto_ros/pronto_ros_conversions.hpp"
#include <tf/transform_listener.h>

namespace pronto {

ViconHandlerROS::ViconHandlerROS(ros::NodeHandle &nh) :
nh_(nh)
{
    std::string prefix = "vicon/";
    ViconConfig cfg;
    std::string mode_str;
    nh_.getParam(prefix + "mode", mode_str);
    if (mode_str.compare("position") == 0) {
      cfg.mode = ViconMode::MODE_POSITION;
      std::cout << "Vicon will provide position measurements." << std::endl;
    }
    else if (mode_str.compare("position_orient") == 0) {
      cfg.mode = ViconMode::MODE_POSITION_ORIENT;
      std::cout << "Vicon will provide position and orientation measurements." << std::endl;
    }
    else if (mode_str.compare("orientation") == 0) {
      cfg.mode = ViconMode::MODE_ORIENTATION;
      std::cout << "Vicon will provide orientation measurements." << std::endl;
    }
    else if (mode_str.compare("yaw") == 0) {
      cfg.mode = ViconMode::MODE_YAW;
      std::cout << "Vicon will provide yaw orientation measurements." << std::endl;
    }
    else {
      cfg.mode = ViconMode::MODE_POSITION;
      std::cout << "Unrecognized Vicon mode. Using position mode by default." << std::endl;
    }

    bool apply_frame = false;
    std::string frame_from;
    std::string frame_to;
    if(nh_.getParam(prefix + "apply_frame", apply_frame) && apply_frame){
        if(nh_.getParam(prefix + "frame_from", frame_from)){
            if(nh_.getParam(prefix + "frame_to", frame_to)){
                tf::TransformListener listener;
                tf::StampedTransform transform;
                try {
                    listener.waitForTransform(frame_to, frame_from, ros::Time(0), ros::Duration(10.0) );
                    listener.lookupTransform(frame_to, frame_from, ros::Time(0), transform);
                    tf::transformTFToEigen(transform, cfg.body_to_vicon);
                } catch (tf::TransformException ex) {
                    ROS_ERROR("%s",ex.what());
                }

            }
        }

    } else {
        cfg.body_to_vicon.setIdentity();
    }

    if(!nh_.getParam(prefix + "r_xyz", cfg.r_vicon_xyz)){
        cfg.r_vicon_xyz = 0;
        ROS_WARN("Couldn't get param \"r_xyz\". Setting to zero.");
    }

    if(!nh_.getParam(prefix + "r_chi", cfg.r_vicon_chi)){
        cfg.r_vicon_chi = 0;
        ROS_WARN("Couldn't get param \"r_chi\". Setting to zero.");
    }
    vicon_module_.reset(new ViconModule(cfg));
}

RBISUpdateInterface* ViconHandlerROS::processMessage(const geometry_msgs::TransformStamped *msg,
                                                     StateEstimator *est)
{
    rigidTransformFromROS(*msg, vicon_transf_);
    return vicon_module_->processMessage(&vicon_transf_,est);
}

bool ViconHandlerROS::processMessageInit(const geometry_msgs::TransformStamped *msg,
                                         const std::map<std::string, bool> &sensor_initialized,
                                         const RBIS &default_state,
                                         const RBIM &default_cov,
                                         RBIS &init_state,
                                         RBIM &init_cov){
    rigidTransformFromROS(*msg, vicon_transf_);
    return vicon_module_->processMessageInit(&vicon_transf_,
                                             sensor_initialized,
                                             default_state,
                                             default_cov,
                                             init_state,
                                             init_cov);
}
} // namespace pronto
