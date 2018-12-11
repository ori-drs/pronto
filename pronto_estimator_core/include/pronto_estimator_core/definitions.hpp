#pragma once
#include <cstdint>
#include <Eigen/Dense>

namespace MavStateEst {

typedef Eigen::Vector3d Position;
typedef Eigen::Quaterniond Orientation;
typedef Eigen::Vector3d LinearVelocity;
typedef Eigen::Vector3d AngularVelocity;
typedef Eigen::Vector3d ProperAcceleration;
typedef Eigen::Vector3d Acceleration;
typedef Eigen::Affine3d Transform;
typedef Eigen::Matrix<double,6,6> PoseCovariance;
struct ImuMeasurement {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    uint64_t utime;
    AngularVelocity omega;
    ProperAcceleration acceleration;
    Orientation orientation;
};

// this is how it is defined in LCM, but a pose should really be just
// position + orientation
struct PoseMeasurement {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    uint64_t utime;
    Position pos;
    LinearVelocity linear_vel;
    Orientation orientation;
    AngularVelocity angular_vel;
    Acceleration accel;
};

struct VisualOdometryUpdate {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    enum Status{NO_DATA = 0,
                ESTIMATE_VALID = 1,
                ESTIMATE_INSUFFICIENT_FEATURES = 2,
                ESTIMATE_DEGENERATE = 3,
                ESTIMATE_REPROJECTION_ERROR = 4};
public:
    uint64_t curr_utime;
    uint64_t prev_utime;
    Transform relative_pose; // relative pose between prev_utime and curr_utime
    Transform prev_pose; // pose of the robot in world frame at time prev_utime
    PoseCovariance pose_covariance;
    uint8_t status;
};

struct RigidTransform {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    uint64_t utime;
    Transform transform;
};
}
