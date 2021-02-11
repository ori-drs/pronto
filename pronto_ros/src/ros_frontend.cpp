#include "pronto_ros/ros_frontend.hpp"

namespace pronto {
using SensorId = pronto::ROSFrontEnd::SensorId;

ROSFrontEnd::ROSFrontEnd(ros::NodeHandle& nh, bool verbose) :
    nh_(nh), verbose_(verbose)
{
    bool publish_pose;
    // looking for relative param names, the nodehandle namespace will be added
    // automatically, e.g. publish_pose -> /state_estimator_pronto/publish_pose
    if(nh_.getParam("publish_pose", publish_pose)){
        if(publish_pose){
            std::string pose_frame_id = "odom";
            if(!nh_.getParam("pose_frame_id", pose_frame_id)){
                ROS_WARN_STREAM("Couldn't get param \"pose_frame_id\". Setting default to: \"" << pose_frame_id <<"\"");
            }
            std::string pose_topic = "POSE_BODY";
            if(nh_.getParam("pose_topic", pose_topic)){
                pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 200);
                pose_msg_.header.frame_id = pose_frame_id;
            }
            std::string twist_topic = "TWIST_BODY";
            std::string twist_frame_id = "base";
            if(!nh_.getParam("twist_frame_id", twist_frame_id)){
                ROS_WARN_STREAM("Couldn't get param \"twist_frame_id\". Setting default to: \"" << twist_frame_id <<"\"");
            }
            if(nh_.getParam("twist_topic", twist_topic)){
                twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>(twist_topic, 200);
                twist_msg_.header.frame_id = twist_frame_id;
            }
            // try to
            if(!nh_.getParam("publish_tf", publish_tf_)){
                ROS_WARN("Couldn't get param \"publish_tf\". Not publishing TF.");
            }
            if(publish_tf_){
                std::string tf_child_frame_id = "base";
                if(!nh_.getParam("tf_child_frame_id", tf_child_frame_id)){
                    ROS_WARN("Couldn't get param \"tf_frame_id\". Setting default to \"base\".");
                }
                tf_pose_.frame_id_ = pose_frame_id;
                // NOTE implicitly assuming the twist frame id is the base frame
                tf_pose_.child_frame_id_ = tf_child_frame_id;
                ROS_INFO_STREAM("Publishing TF with frame_id: \"" << tf_pose_.frame_id_ << "\" and child_frame_id: \"" << tf_pose_.child_frame_id_ << "\"");
            }
        }

    }

    double history_span = 1e6; // keep 1 second as default
    if(!nh_.getParam("utime_history_span", history_span)){
     ROS_WARN_STREAM("Couldn't get param \"utime_history_span\". Setting default to \"" << history_span << "\"");
    }
    history_span_ = history_span;
    initializeState();
    initializeCovariance();
    if(verbose_){
        ROS_INFO_STREAM("Frontend constructed.");
    }

    aicp_path_publisher = nh_.advertise<nav_msgs::Path>("/aicp/relative_path",100);
}

void ROSFrontEnd::initializeState()
{
    // setting the initial values from the state
    std::vector<double> init_velocity = std::vector<double>(3,0.0);
    if(!nh_.getParam("x0/velocity", init_velocity)){
        ROS_WARN_STREAM("Couldn't get default velocity. Setting to zero.");
    }
    default_state.velocity() = Eigen::Map<Eigen::Vector3d>(init_velocity.data());

    // setting the initial values from the state
    std::vector<double> init_position  = std::vector<double>(3,0.0);
    if(!nh_.getParam("x0/position", init_position)){
        ROS_WARN_STREAM("Couldn't get default position. Setting to zero.");
    }
    default_state.position() = Eigen::Map<Eigen::Vector3d>(init_position.data());

    std::vector<double> init_omega = std::vector<double>(3,0.0);
    if(!nh_.getParam("x0/angular_velocity", init_omega)){
        ROS_WARN_STREAM("Couldn't get default ang velocity. Setting to zero.");
    }
    default_state.angularVelocity() = Eigen::Map<Eigen::Vector3d>(init_omega.data());

    std::vector<double> init_orient = std::vector<double>(3,0.0);
    if(!nh_.getParam("x0/rpy", init_orient)){
        ROS_WARN_STREAM("Couldn't get default ang velocity. Setting to zero.");
        default_state.orientation() = Eigen::Quaterniond::Identity();
    }
    default_state.orientation() = rotation::setQuatEulerAngles(Eigen::Map<Eigen::Vector3d>(init_orient.data()));
    init_state = default_state;
    head_state = default_state;
    if(verbose_){
        ROS_INFO_STREAM("Filter Initial State initialized.");
    }
}


void ROSFrontEnd::initializeCovariance(){
    default_cov = RBIM::Zero();

    double sigma_Delta_xy_init = 0;
    if(!nh_.getParam("sigma0/Delta_xy", sigma_Delta_xy_init)){
        ROS_WARN_STREAM("Couldn't get param sigma0/Delta_xy. Setting to zero.");
    }


    double sigma_Delta_z_init = 0;
    if(!nh_.getParam("sigma0/Delta_z", sigma_Delta_z_init)){
        ROS_WARN_STREAM("Couldn't get param sigma0/Delta_z. Setting to zero.");
    }

    double sigma_chi_xy_init = 0;
    if(!nh_.getParam("sigma0/chi_xy", sigma_chi_xy_init)){
        ROS_WARN_STREAM("Couldn't get param sigma0/chi_xy. Setting to zero.");
    }
    sigma_chi_xy_init *= M_PI / 180.0; // convert to radians

    double sigma_chi_z_init = 0;
    if(!nh_.getParam("sigma0/chi_z", sigma_chi_z_init)){
        ROS_WARN_STREAM("Couldn't get param sigma0/chi_z. Setting to zero.");
    }
    sigma_chi_z_init *= M_PI / 180.0; // convert to radians

    double sigma_vb_init = 0;
    if(!nh_.getParam("sigma0/vb", sigma_vb_init)){
        ROS_WARN_STREAM("Couldn't get param sigma0/vb. Setting to zero.");
    }

    double sigma_gyro_bias_init = 0;
    if(!nh_.getParam("sigma0/gyro_bias", sigma_gyro_bias_init)){
        ROS_WARN_STREAM("Couldn't get param sigma0/gyro_bias. Setting to zero.");
    }
    sigma_gyro_bias_init *= M_PI / 180.0; // convert to radians

    double sigma_accelerometer_bias_init = 0;
    if(!nh_.getParam("sigma0/accel_bias", sigma_accelerometer_bias_init)){
        ROS_WARN_STREAM("Couldn't get param sigma0/accel_bias. Setting to zero.");
    }

    Eigen::Vector3d xyz_cov_diag =
        Eigen::Array3d(sigma_Delta_xy_init, sigma_Delta_xy_init, sigma_Delta_z_init).square();

    Eigen::Vector3d init_chi_cov_diag = Eigen::Array3d(sigma_chi_xy_init, sigma_chi_xy_init, sigma_chi_z_init).square();

    //set all the sub-blocks of the covariance matrix
    default_cov.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind) = std::pow(sigma_vb_init,2) * Eigen::Matrix3d::Identity();
    default_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = init_chi_cov_diag.asDiagonal();
    default_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = xyz_cov_diag.asDiagonal();
    default_cov.block<3, 3>(RBIS::gyro_bias_ind, RBIS::gyro_bias_ind) = Eigen::Matrix3d::Identity()
        * std::pow(sigma_gyro_bias_init,2);
    default_cov.block<3, 3>(RBIS::accel_bias_ind, RBIS::accel_bias_ind) = Eigen::Matrix3d::Identity()
        * std::pow(sigma_accelerometer_bias_init,2);

    init_cov = default_cov;
    head_cov = default_cov;
    if(verbose_){
        ROS_INFO_STREAM("Filter Covariance initialized.");
    }
}

ROSFrontEnd::~ROSFrontEnd()
{
}

bool ROSFrontEnd::initializeFilter()
{
    // if the modules are not ready we return false
    if(!areModulesInitialized())
    {
        return false;
    }
    // if the filter is already initialized we quietly return
    if(isFilterInitialized()){
        return true;
    }
    state_est_.reset(new StateEstimator(new RBISResetUpdate(init_state,
                                                               init_cov,
                                                               RBISUpdateInterface::reset,
                                                               init_state.utime),
                                           history_span_));

    filter_initialized_ = true;
    if(verbose_){
        ROS_INFO_STREAM("Filter initialized.");
    }
    return true;
}

bool ROSFrontEnd::areModulesInitialized(){
    std::map<SensorId,bool>::iterator it = initialized_list_.begin();
    for(; it != initialized_list_.end(); ++it){
        if(!it->second){
            return false;
        }
    }
    return true;
}

bool ROSFrontEnd::isFilterInitialized(){
    return filter_initialized_;
}

}
