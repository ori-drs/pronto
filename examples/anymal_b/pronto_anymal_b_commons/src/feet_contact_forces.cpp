#include "pronto_anymal_b_commons/feet_contact_forces.hpp"
#include <pronto_quadruped_commons/geometry/rotations.h>
#include <pronto_quadruped_commons/joint_id_tricks.h>

using namespace iit;

namespace pronto {
namespace anymal {

bool FeetContactForces::getFootGRF(const JointState &q,
                                      const JointState &qd,
                                      const JointState &tau,
                                      const Quaterniond &orient,
                                      const LegID &leg,
                                      Vector3d &foot_grf,
                                      const JointState &qdd,
                                      const Vector3d &xd,
                                      const Vector3d &xdd,
                                      const Vector3d &omega,
                                      const Vector3d &omegad) {

    Eigen::Matrix3d foot_jacobian = feet_jacs_.getFootJacobian(q, leg);

    rbd::Vector6D gravity_world = rbd::Vector6D::Zero();
    gravity_world(rbd::LZ) = -rbd::g;

    rbd::Vector6D gravity_base = rbd::Vector6D::Zero();
    rbd::Vector6D base_acceleration = rbd::Vector6D::Zero();
    rbd::VelocityVector base_twist = rbd::Vector6D::Zero();

    Eigen::Matrix3d R = pronto::commons::quatToRotMat(orient);

    gravity_base.segment(rbd::LX, 3) = R * gravity_world.segment(rbd::LX, 3);

    base_acceleration.segment(rbd::LX, 3) = xdd; //this is the absolute accel of the trunk without the gravity!!!
    base_acceleration.segment(rbd::AX, 3) = omegad;

    base_twist.segment(rbd::AX, 3) = omega;
    base_twist.segment(rbd::LX, 3) = xd;

    rbd::ForceVector h_base;
    JointState  h_joints;

    // Update the Joint Space Inertia Matrix with latest encoder values
    jsim_(q);


    // If we set all accelerations to zero, we basically compute the h_base and
    // h_joints components of the dynamics equation
    inverse_dynamics_.id_fully_actuated(h_base,
                                        h_joints,
                                        gravity_base,
                                        base_twist,
                                        base_acceleration,
                                        q,
                                        qd,
                                        qdd);

    // 3 joints of the LF leg, on a 3x1 vector;
    Eigen::Vector3d tau_leg = quadruped::getLegJointState(LegID(leg), tau);
    Eigen::Vector3d h_leg = quadruped::getLegJointState(LegID(leg), h_joints);
    Eigen::Matrix3d M_leg = jsim_.getFixedBaseBlock().block<3, 3>(leg * 3, leg * 3);
    Eigen::Matrix<double, 6, 3> F_leg = jsim_.getF().block<6, 3>(0, leg * 3);
    Eigen::Vector3d qdd_leg = qdd.block<3, 1>(leg * 3, 0);

    //N.B the terms - F_leg.transpose() * base_acceleration - M_leg * qdd_leg are already incorporated in h_joints

    foot_grf = -(foot_jacobian.transpose()).inverse() *
                (tau_leg - h_leg  );

    return foot_grf.allFinite();
}


}
}
