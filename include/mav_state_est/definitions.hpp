#pragma once
#include <cstdint>
#include <Eigen/Dense>

namespace MavStateEst {

typedef Eigen::Vector3d AngularVelocity;
typedef Eigen::Vector3d ProperAcceleration;
typedef Eigen::Quaterniond Quaternion;

struct ImuMeasurement {
    uint64_t utime;
    AngularVelocity omega;
    ProperAcceleration acceleration;
    Quaternion orientation;
};
}
