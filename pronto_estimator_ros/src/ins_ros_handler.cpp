#include "pronto_estimator_ros/ins_ros_handler.hpp"
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
namespace MavStateEst {

InsHandlerROS::InsHandlerROS(ros::NodeHandle &nh) : nh_(nh)
{

    tf::TransformListener tf_imu_to_body_listener_;
    tf::StampedTransform tf_imu_to_body_;

    std::string ins_param_prefix = "ins/";
    std::string imu_frame = "/imu";

    nh_.getParam(ins_param_prefix + "frame", imu_frame);
    std::string base_frame = "/base";
    Eigen::Affine3d ins_to_body;
    while(nh_.ok()){
        try{
              tf_imu_to_body_listener_.lookupTransform(imu_frame, base_frame,
                                       ros::Time(0), tf_imu_to_body_);
              geometry_msgs::Transform temp_transform;
              tf::transformTFToMsg(tf_imu_to_body_,temp_transform);
              tf::transformMsgToEigen(temp_transform, ins_to_body);
              break;
            }
            catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
            }
    }

    InsConfig cfg;

    nh_.getParam(ins_param_prefix + "num_to_init", cfg.num_to_init);
    nh_.getParam(ins_param_prefix + "accel_bias_update_online", cfg.accel_bias_update_online);
    nh_.getParam(ins_param_prefix + "gyro_bias_update_online", cfg.gyro_bias_update_online);
    nh_.getParam(ins_param_prefix + "accel_bias_recalc_at_start", cfg.accel_bias_recalc_at_start);
    nh_.getParam(ins_param_prefix + "gyro_bias_recalc_at_start", cfg.gyro_bias_recalc_at_start);
    nh_.getParam(ins_param_prefix + "timestep_dt", cfg.dt);
    std::vector<double> accel_bias_initial_v;
    std::vector<double> gyro_bias_initial_v;
    nh_.getParam(ins_param_prefix + "accel_bias_initial", accel_bias_initial_v);
    nh_.getParam(ins_param_prefix + "gyro_bias_initial", gyro_bias_initial_v);
    cfg.accel_bias_initial = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(accel_bias_initial_v.data());
    cfg.gyro_bias_initial = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(gyro_bias_initial_v.data());
    nh_.getParam(ins_param_prefix + "max_initial_gyro_bias", cfg.max_initial_gyro_bias);
    nh_.getParam(ins_param_prefix + "topic", imu_topic_);
    int downsample_factor = downsample_factor_;
    nh_.getParam(ins_param_prefix + "downsample_factor", downsample_factor);
    downsample_factor_ = downsample_factor;

    nh_.getParam(ins_param_prefix + "roll_forward_on_receive", roll_forward_on_receive_);
    int utime_offset = utime_offset_;
    nh_.getParam(ins_param_prefix + "utime_offset", utime_offset);
    utime_offset_ = utime_offset;

    double std_accel;
    double std_gyro;
    double std_gyro_bias;
    double std_accel_bias;

    nh_.getParam(ins_param_prefix + "q_gyro", std_gyro);
    nh_.getParam(ins_param_prefix + "q_gyro_bias", std_gyro_bias);
    nh_.getParam(ins_param_prefix + "q_accel", std_accel);
    nh_.getParam(ins_param_prefix + "q_accel_bias", std_accel_bias);

    cfg.cov_accel = std::pow(std_accel, 2);
    cfg.cov_gyro = std::pow(std_gyro * M_PI / 180.0, 2);
    cfg.cov_accel_bias = std::pow(std_accel_bias, 2);
    cfg.cov_gyro_bias = std::pow(std_gyro_bias * M_PI / 180.0,2);

    ins_module_ = InsModule(cfg, ins_to_body);
}

RBISUpdateInterface* InsHandlerROS::processMessage(const sensor_msgs::Imu *imu_msg, MavStateEstimator *est){
    // keep one every downsample_factor messages
    if(counter++ % downsample_factor_ != 0){
        return NULL;
    }
    msgToImuMeasurement(*imu_msg, imu_meas_);
    return ins_module_.processMessage(&imu_meas_, est);
}

void InsHandlerROS::msgToImuMeasurement(const sensor_msgs::Imu &imu_msg, ImuMeasurement &imu_meas){
    // convert the ROS message into our internal format
    tf::vectorMsgToEigen(imu_msg.linear_acceleration,imu_meas.acceleration);
    tf::quaternionMsgToEigen(imu_msg.orientation, imu_meas.orientation);
    tf::vectorMsgToEigen(imu_msg.angular_velocity, imu_meas.omega);
    imu_meas.utime = imu_msg.header.stamp.toNSec() / 1000 + utime_offset_;
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
} // namespace MavStateEst
