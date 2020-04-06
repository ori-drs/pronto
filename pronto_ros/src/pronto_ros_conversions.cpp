#include "pronto_ros/pronto_ros_conversions.hpp"
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/JointState.h>

namespace pronto {

void gpsDataFromROS(const pronto_msgs::GPSData &ros_msg,
                    GPSMeasurement &msg)
{
  msg.elev = ros_msg.elev;
  msg.gps_lock = ros_msg.gps_lock;
  msg.gps_time = ros_msg.gps_time;
  msg.heading = ros_msg.heading;
  msg.horizontal_accuracy = ros_msg.horizontal_accuracy;
  msg.latitude = ros_msg.latitude;
  msg.longitude = ros_msg.longitude;
  msg.numSatellites = ros_msg.num_satellites;
  msg.speed = ros_msg.speed;
  msg.utime = ros_msg.utime;
  msg.vertical_accuracy = ros_msg.vertical_accuracy;
  msg.xyz_pos = Eigen::Map<const Eigen::Vector3d>(ros_msg.xyz_pos.data());
}

void indexMeasurementFromROS(const pronto_msgs::IndexedMeasurement &ros_msg,
                             IndexedMeasurement &msg)
{
  // check that the size of z_indices == z_effective and R_effective is its square
  if(ros_msg.R_effective.size() != ros_msg.z_effective.size() * ros_msg.z_indices.size()){
    return;
  }

  // TODO check that the data are rowmajor
  msg.R_effective = Eigen::Map<const Eigen::MatrixXd>(ros_msg.R_effective.data(),
                                                      ros_msg.z_effective.size(),
                                                      ros_msg.z_effective.size());
  msg.state_utime = ros_msg.state_utime;
  msg.utime = ros_msg.utime;
  msg.z_effective = Eigen::Map<const Eigen::VectorXd>(ros_msg.z_effective.data(),
                                                      ros_msg.z_effective.size());
  msg.z_indices = Eigen::Map<const Eigen::VectorXi>(ros_msg.z_indices.data(),
                                                    ros_msg.z_indices.size());
}

void filterStateFromROS(const pronto_msgs::FilterState &ros_msg,
                        FilterState &msg)
{
  if(ros_msg.cov.size() != std::pow(ros_msg.state.size(),2))
  {
    throw std::logic_error("Covariance matrix of size " +
                           std::to_string(ros_msg.cov.size()) +
                           + ". " + std::to_string(std::pow(ros_msg.state.size(),2)) + " expected.");
    return;
  }
  Eigen::Quaterniond init_quat;
  tf::Quaternion q;
  tf::quaternionMsgToTF(ros_msg.quat, q);
  tf::quaternionTFToEigen(q, init_quat);
  Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> cov_map(ros_msg.cov.data(),
                                                                                                   ros_msg.state.size(),
                                                                                                   ros_msg.state.size());
  msg.cov = cov_map;
  msg.quat = init_quat;
  msg.state = Eigen::Map<const Eigen::VectorXd>(ros_msg.state.data(), ros_msg.state.size());
  msg.utime = ros_msg.header.stamp.toNSec() / 1000;
}

void msgToImuMeasurement(const sensor_msgs::Imu &imu_msg,
                         ImuMeasurement &imu_meas,
                         const int64_t& utime_offset)
{
  // convert the ROS message into our internal format
  tf::vectorMsgToEigen(imu_msg.linear_acceleration,imu_meas.acceleration);
  tf::quaternionMsgToEigen(imu_msg.orientation, imu_meas.orientation);
  tf::vectorMsgToEigen(imu_msg.angular_velocity, imu_meas.omega);
  imu_meas.utime = imu_msg.header.stamp.toNSec() / 1000 + utime_offset;
}

void poseMsgFromROS(const geometry_msgs::PoseWithCovarianceStamped &msg,
                    PoseMeasurement &pose_meas)
{
  // covariance is not implemented yet,
  //  we just forward to non covariance overload
  geometry_msgs::PoseStamped p;
  p.pose = msg.pose.pose;
  p.header = msg.header;
  poseMsgFromROS(p, pose_meas);
}
void poseMsgFromROS(const geometry_msgs::PoseStamped &msg,
                    PoseMeasurement &pose_meas)
{
  pose_meas.orientation = Orientation(msg.pose.orientation.w,
                                      msg.pose.orientation.x,
                                      msg.pose.orientation.y,
                                      msg.pose.orientation.z);
  pose_meas.pos << msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z;
  pose_meas.utime = (uint64_t) msg.header.stamp.toNSec() / 1000;
}

void poseMeasurementFromROS(const nav_msgs::Odometry &ros_msg,
                            PoseMeasurement &msg)
{
  msg.utime = ros_msg.header.stamp.toNSec() / 1000;

  msg.angular_vel << ros_msg.twist.twist.angular.x,
      ros_msg.twist.twist.angular.y,
      ros_msg.twist.twist.angular.z;

  msg.linear_vel << ros_msg.twist.twist.linear.x,
      ros_msg.twist.twist.linear.y,
      ros_msg.twist.twist.linear.z;

  msg.pos << ros_msg.pose.pose.position.x,
      ros_msg.pose.pose.position.y,
      ros_msg.pose.pose.position.z;
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(ros_msg.pose.pose.orientation, tf_q);
  tf::quaternionTFToEigen(tf_q, msg.orientation);
}

void poseMeasurementFromROS(const geometry_msgs::Pose& ros_msg,
                            PoseMeasurement &msg) {

}

void rigidTransformFromROS(const geometry_msgs::TransformStamped &msg,
                           RigidTransform &transf)
{
  tf::Transform temp_tf_transf_;
  tf::transformMsgToTF(msg.transform, temp_tf_transf_);
  tf::transformTFToEigen(temp_tf_transf_, transf.transform);
  transf.utime = msg.header.stamp.toNSec() / 1000; // from nanosec to microsec
}

void jointStateFromROS(const sensor_msgs::JointState &ros_msg, JointState &msg){
  // it is caller's responsibility to check that both joint states have same
  // size
  msg.utime = ros_msg.header.stamp.toNSec() / 1000;
  msg.joint_position = std::move(ros_msg.position);
  msg.joint_velocity = std::move(ros_msg.velocity);
  msg.joint_effort = std::move(ros_msg.effort);
  msg.joint_name = std::move(ros_msg.name);
}

void visualOdometryFromROS(const pronto_msgs::VisualOdometryUpdate& ros_msg,
                           VisualOdometryUpdate& msg){
  msg.curr_utime = ros_msg.curr_timestamp.toNSec() / 1000;
  msg.status = ros_msg.estimate_status;
  msg.prev_utime = ros_msg.prev_timestamp.toNSec() / 1000;
  tf::transformMsgToEigen(ros_msg.relative_transform, msg.relative_pose);
  msg.pose_covariance = Eigen::Map<const PoseCovariance, Eigen::Unaligned>(ros_msg.covariance.data(),6,6);
}

void lidarOdometryFromROS(const pronto_msgs::LidarOdometryUpdate &ros_msg,
                          LidarOdometryUpdate &msg)
{
  msg.curr_utime = ros_msg.curr_timestamp.toNSec() / 1000;
  msg.prev_utime = ros_msg.prev_timestamp.toNSec() / 1000;
  tf::transformMsgToEigen(ros_msg.relative_transform, msg.relative_pose);
  msg.pose_covariance = Eigen::Map<const PoseCovariance, Eigen::Unaligned>(ros_msg.covariance.data(),6,6);
}

void forceTorqueFromROS(const pronto_msgs::BipedForceTorqueSensors& ros_msg,
                        pronto::ForceTorqueSensorArray& msg) {
  msg.num_sensors = 4;
  msg.names = {"l_foot", "r_foot", "l_hand", "r_hand"};
  msg.utime = ros_msg.header.stamp.toNSec() * 1e-3;
  msg.sensors.resize(msg.num_sensors);
  pronto::ForceTorqueSensor ft;
  ft.utime = msg.utime;
  ft.force[0] = ros_msg.l_foot.force.x;
  ft.force[1] = ros_msg.l_foot.force.y;
  ft.force[2] = ros_msg.l_foot.force.z;
  ft.moment[0] = ros_msg.l_foot.torque.x;
  ft.moment[1] = ros_msg.l_foot.torque.y;
  ft.moment[2] = ros_msg.l_foot.torque.z;
  msg.sensors[0] = ft;

  ft.force[0] = ros_msg.r_foot.force.x;
  ft.force[1] = ros_msg.r_foot.force.y;
  ft.force[2] = ros_msg.r_foot.force.z;
  ft.moment[0] = ros_msg.r_foot.torque.x;
  ft.moment[1] = ros_msg.r_foot.torque.y;
  ft.moment[2] = ros_msg.r_foot.torque.z;
  msg.sensors[1] = ft;

  ft.force[0] = ros_msg.l_hand.force.x;
  ft.force[1] = ros_msg.l_hand.force.y;
  ft.force[2] = ros_msg.l_hand.force.z;
  ft.moment[0] = ros_msg.l_hand.torque.x;
  ft.moment[1] = ros_msg.l_hand.torque.y;
  ft.moment[2] = ros_msg.l_hand.torque.z;
  msg.sensors[2] = ft;

  ft.force[0] = ros_msg.r_hand.force.x;
  ft.force[1] = ros_msg.r_hand.force.y;
  ft.force[2] = ros_msg.r_hand.force.z;
  ft.moment[0] = ros_msg.r_hand.torque.x;
  ft.moment[1] = ros_msg.r_hand.torque.y;
  ft.moment[2] = ros_msg.r_hand.torque.z;
  msg.sensors[3] = ft;
}

}
