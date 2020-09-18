#include "pronto_ros/ins_ros_handler.hpp"
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_listener.h>

#include "pronto_ros/pronto_ros_conversions.hpp"


namespace pronto {

InsHandlerROS::InsHandlerROS(ros::NodeHandle &nh) : nh_(nh)
{
  tf2_ros::Buffer tf_imu_to_body_buffer_;
  tf2_ros::TransformListener tf_imu_to_body_listener_(tf_imu_to_body_buffer_);

    std::string ins_param_prefix = "ins/";
    std::string imu_frame = "imu";

    nh_.getParam(ins_param_prefix + "frame", imu_frame);
    std::string base_frame = "base";
    Eigen::Affine3d ins_to_body;
    while(nh_.ok()){
        try{
        geometry_msgs::TransformStamped temp_transform;
              temp_transform = tf_imu_to_body_buffer_.lookupTransform(imu_frame, base_frame,
                                       ros::Time(0));

              tf::transformMsgToEigen(temp_transform.transform, ins_to_body);
              break;
            }
            catch (tf2::TransformException ex){
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
            }
    }

    InsConfig cfg;

    if(!nh_.getParam(ins_param_prefix + "num_to_init", cfg.num_to_init)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace() << "/" << ins_param_prefix
                        << "num_to_init\". Using default: "
                        << cfg.num_to_init);
    } else {
        ROS_INFO_STREAM("num_to_init: " << cfg.num_to_init);
    }

    if(!nh_.getParam(ins_param_prefix + "accel_bias_update_online", cfg.accel_bias_update_online)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "accel_bias_update_online\". Using default: "
                        << cfg.accel_bias_update_online);
    }
    if(!nh_.getParam(ins_param_prefix + "gyro_bias_update_online", cfg.gyro_bias_update_online)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "gyro_bias_update_online\". Using default: "
                        << cfg.gyro_bias_update_online);
    }
    if(!nh_.getParam(ins_param_prefix + "accel_bias_recalc_at_start", cfg.accel_bias_recalc_at_start)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "accel_bias_recalc_at_start\". Using default: "
                        << cfg.accel_bias_recalc_at_start);

    }
    if(!nh_.getParam(ins_param_prefix + "gyro_bias_recalc_at_start", cfg.gyro_bias_recalc_at_start)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "gyro_bias_recalc_at_start\". Using default: "
                        << cfg.gyro_bias_recalc_at_start);
    }
    if(!nh_.getParam(ins_param_prefix + "timestep_dt", cfg.dt)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "timestep_dt\". Using default: "
                        << cfg.dt);
    }
    std::vector<double> accel_bias_initial_v;
    std::vector<double> gyro_bias_initial_v;
    if(!nh_.getParam(ins_param_prefix + "accel_bias_initial", accel_bias_initial_v)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "accel_bias_initial\". Using default: "
                        << cfg.accel_bias_initial.transpose());
    } else {
        cfg.accel_bias_initial = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(accel_bias_initial_v.data());
        ROS_INFO_STREAM("accel_bias_initial: " << cfg.accel_bias_initial.transpose());
    }


    if(!nh_.getParam(ins_param_prefix + "gyro_bias_initial", gyro_bias_initial_v)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "gyro_bias_initial\". Using default: "
                        << cfg.gyro_bias_initial.transpose());
    } else {
        cfg.gyro_bias_initial = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(gyro_bias_initial_v.data());
        ROS_INFO_STREAM("gyro_bias_initial: " << cfg.gyro_bias_initial.transpose());
    }


    if(!nh_.getParam(ins_param_prefix + "max_initial_gyro_bias", cfg.max_initial_gyro_bias)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "max_initial_gyro_bias\". Using default: "
                        << cfg.max_initial_gyro_bias);
    }

    if(!nh_.getParam(ins_param_prefix + "topic", imu_topic_)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "topic\". Using default: "
                        << imu_topic_);
    }
    int downsample_factor = downsample_factor_;
    if(!nh_.getParam(ins_param_prefix + "downsample_factor", downsample_factor)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "downsample_factor\". Using default: "
                        << downsample_factor);
    } else {
        downsample_factor_ = downsample_factor;
    }


    if(!nh_.getParam(ins_param_prefix + "roll_forward_on_receive", roll_forward_on_receive_)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "roll_forward_on_receive\". Using default: "
                        << roll_forward_on_receive_);
    }
    int utime_offset = utime_offset_;
    if(!nh_.getParam(ins_param_prefix + "utime_offset", utime_offset)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "utime_offset\". Using default: "
                        << utime_offset);
    } else {
        utime_offset_ = utime_offset;
    }

    double std_accel = 0;
    double std_gyro = 0;
    double std_gyro_bias = 0;
    double std_accel_bias = 0;

    if(!nh_.getParam(ins_param_prefix + "q_gyro", std_gyro)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "q_gyro\". Using default: "
                        << std_gyro);
    }
    if(!nh_.getParam(ins_param_prefix + "q_gyro_bias", std_gyro_bias)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "q_gyro_bias\". Using default: "
                        << std_gyro_bias);
    }
    if(!nh_.getParam(ins_param_prefix + "q_accel", std_accel)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "q_accel\". Using default: "
                        << std_accel);
    }
    if(!nh_.getParam(ins_param_prefix + "q_accel_bias", std_accel_bias)){
        ROS_WARN_STREAM("Couldn't get param \""
                        << nh_.getNamespace()  << "/" << ins_param_prefix
                        << "q_accel_bias\". Using default: "
                        << std_accel_bias);
    }

    cfg.cov_accel = std::pow(std_accel, 2);
    cfg.cov_gyro = std::pow(std_gyro * M_PI / 180.0, 2);
    cfg.cov_accel_bias = std::pow(std_accel_bias, 2);
    cfg.cov_gyro_bias = std::pow(std_gyro_bias * M_PI / 180.0,2);

    ins_module_ = InsModule(cfg, ins_to_body);
}

RBISUpdateInterface* InsHandlerROS::processMessage(const sensor_msgs::Imu *imu_msg, StateEstimator *est){
    // keep one every downsample_factor messages
    if(counter++ % downsample_factor_ != 0){
        return nullptr;
    }
    msgToImuMeasurement(*imu_msg, imu_meas_, utime_offset_);
    return ins_module_.processMessage(&imu_meas_, est);
}

bool InsHandlerROS::processMessageInit(const sensor_msgs::Imu *imu_msg,
                        const std::map<std::string, bool> &sensor_initialized,
                        const RBIS &default_state,
                        const RBIM &default_cov,
                        RBIS &init_state,
                        RBIM &init_cov){
    msgToImuMeasurement(*imu_msg, imu_meas_);
    return ins_module_.processMessageInit(&imu_meas_,
                                          sensor_initialized,
                                          default_state,
                                          default_cov,
                                          init_state,
                                          init_cov);

}
} // namespace pronto
