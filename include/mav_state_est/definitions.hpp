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
}
