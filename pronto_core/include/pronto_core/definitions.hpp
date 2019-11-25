#pragma once
#include <cstdint>
#include <Eigen/Dense>
#include <vector>

namespace pronto {

typedef Eigen::Vector3d Position;
typedef Eigen::Quaterniond Orientation;
typedef Eigen::Vector3d LinearVelocity;
typedef Eigen::Vector3d AngularVelocity;
typedef Eigen::Vector3d ProperAcceleration;
typedef Eigen::Vector3d Acceleration;
typedef Eigen::Isometry3d Transform;
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

struct LidarOdometryUpdate {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    enum Status{NO_DATA = 0,
                REGISTRATION_VALID = 1,
                REGISTRATION_FAILURE = 2};
public:
    uint64_t curr_utime;
    uint64_t prev_utime;
    Transform relative_pose; // relative pose between prev_utime and curr_utime
    Transform prev_pose; // pose of the robot in world frame at time prev_utime
    PoseCovariance pose_covariance;
    uint8_t status = REGISTRATION_VALID; // assuming ok unless otherwise
};

struct RigidTransform {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    uint64_t utime;
    Transform transform;
};

struct IndexedMeasurement {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    uint64_t utime;
    uint64_t state_utime;
    Eigen::VectorXd z_effective;
    Eigen::VectorXi z_indices;
    Eigen::MatrixXd R_effective;
};

struct FilterState {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    uint64_t utime;
    Orientation quat;
    Eigen::VectorXd state;
    Eigen::MatrixXd cov;
};

struct GPSMeasurement {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    uint64_t    utime;
    int32_t    gps_lock;
    double     longitude;
    double     latitude;
    double     elev;
    double     horizontal_accuracy;
    double     vertical_accuracy;
    uint32_t    numSatellites;
    double     speed;
    double     heading;
    Position   xyz_pos;
    double     gps_time;
};

struct JointState {
    inline explicit JointState(const int N_DOF = 12) {
        joint_position.resize(N_DOF);
        joint_velocity.resize(N_DOF);
        joint_acceleration.resize(N_DOF);
        joint_effort.resize(N_DOF);
        joint_name.resize(N_DOF);

    }
  uint64_t utime;
  std::vector<double> joint_position;
  std::vector<double> joint_velocity;
  std::vector<double> joint_effort;
  std::vector<double> joint_acceleration;
  std::vector<std::string> joint_name;
};

struct ForceTorqueSensor {
    // default constructor
    ForceTorqueSensor()  = default;
    // copy constructor
    ForceTorqueSensor(const ForceTorqueSensor& s){
        this->utime = s.utime;
        memcpy(this->force, s.force, 3*sizeof(double));
        memcpy(this->moment, s.moment, 3*sizeof(double));
    }
    uint64_t utime;
    double force[3];
    double moment[3];
};

struct ForceTorqueSensorArray {
    uint64_t    utime;
    uint32_t    num_sensors;
    std::vector<std::string> names;
    std::vector<ForceTorqueSensor> sensors;
};
}
