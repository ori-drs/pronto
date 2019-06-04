#pragma once
#include <pronto_core/definitions.hpp>

#include <pronto_msgs/GPSData.h>
#include <pronto_msgs/IndexedMeasurement.h>
#include <pronto_msgs/FilterState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

namespace pronto {

void gpsDataFromROS(const pronto_msgs::GPSData& ros_msg,
                    GPSMeasurement& msg);

void indexMeasurementFromROS(const pronto_msgs::IndexedMeasurement& ros_msg,
                             IndexedMeasurement& msg);

void filterStateFromROS(const pronto_msgs::FilterState& ros_msg,
                        FilterState& msg);

void msgToImuMeasurement(const sensor_msgs::Imu& imu_msg,
                         ImuMeasurement& imu_meas,
                         const int64_t &utime_offset = 0);

void poseMsgFromROS(const geometry_msgs::PoseWithCovarianceStamped &msg,
                    PoseMeasurement &pose_meas);

void poseMeasurementFromROS(const nav_msgs::Odometry& ros_msg,
                            PoseMeasurement& msg);

void rigidTransformFromROS(const geometry_msgs::TransformStamped& msg,
                           RigidTransform& transf);

}
