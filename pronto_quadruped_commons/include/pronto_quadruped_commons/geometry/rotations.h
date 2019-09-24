/*
 * Copyright (c) 2015-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * Authors: Marco Frigerio, Michele Focchi, Marco Camurri
 *
 * This file is part of pronto_quadruped_commons, a library for
 * algebra, kinematics and dynamics for quadruped robots.
 * This library is a fork of iit_commons.
 * For more information see:
 * https://github.com/iit-DLSLab/iit_commons
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once

#include <Eigen/Dense>
#include <pronto_quadruped_commons/geometry/algebra.h>

typedef typename Eigen::Quaterniond Quaternion;

namespace pronto {
namespace commons {

/**
 * @brief converts a quaternion \f$\mathbf{q}\f$ into a rotation matrix
 * \f$R_q(\mathbf{q})\f$. The rotation matrix maps a vector
 * \f$\mathbf{z}\in\mathbb{R}^{3\times1}\f$ (expressed in global coordinates)
 * into a vector \f$ \mathbf{z}' \in \mathbb{R}^{3\times1}\f$ (expressed in
 * local coordinates), such that \f$ \mathbf{z}' = R_q(\mathbf{q})\mathbf{z}\f$.
 *
 * @param[in] q structure containing the quaternion
 * @return the 3 by 3 rotation matrix \f$R_q(\mathbf{q})\f$
 * @remark the function uses the formula (125) from <a href="https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf">"Representing Attitude: Euler
 *  Angles, Unit Quaternions, and Rotation Vectors"</a> by James Diebel.
 * @date July 2005
 */
inline Eigen::Matrix3d  quatToRotMat(const Eigen::Quaterniond & q) {
    Eigen::Matrix3d R;
    R(0, 0) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.x() * q.x());
    R(1, 1) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.y() * q.y());
    R(2, 2) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.z() * q.z());
    R(0, 1) = 2.0 * (q.x() * q.y() + q.w() * q.z());
    R(0, 2) = 2.0 * (q.x() * q.z() - q.w() * q.y());
    R(1, 0) = 2.0 * (q.x() * q.y() - q.w() * q.z());
    R(1, 2) = 2.0 * (q.y() * q.z() + q.w() * q.x());
    R(2, 0) = 2.0 * (q.x() * q.z() + q.w() * q.y());
    R(2, 1) = 2.0 * (q.y() * q.z() - q.w() * q.x());

    return R;
}

/**
 * @brief Function to compute the rotation matrix which expresses a vector of
 * the fixed frame A into the rotated frame B according to the ZYX convention
 * (subsequent rotation) considering right hand coordinate systems (counter
 * clockwise convention)
 * \f[
 * {}_B R_A = \begin{bmatrix}
 * \cos(\psi)\cos(\theta) & \cos(\theta)\sin(\psi) & -\sin(\theta) \\
 * \cos(\psi)\sin(\phi)\sin(\theta) - \cos(\phi)\sin(\psi) & \cos(\phi)\cos(\psi) + \sin(\phi)\sin(\psi)\sin(\theta) & \cos(\theta)\sin(\phi) \\
 * \sin(\phi)\sin(\psi) + \cos(\phi)\cos(\psi)\sin(\theta) & \cos(\phi)\sin(\psi)\sin(\theta) - \cos(\psi)\sin(\phi) & \cos(\phi)\cos(\theta)
 * \end{bmatrix}
 * \f]
 * the transpose of this matrix has as director cosines (columns) the axis of
 * the rotated frame expressed in the fixed frame which will be multiplied for
 * the component of the vector in the rotated frame B to get the components in
 * the fixed frame A
 * @param[in] rpy vector containing roll \f$ \phi\f$, pitch \f$ \theta \f$ and yaw \f$\psi\f$
 * @return the matrix \f${}_B R_A\f$
 *
*/
Eigen::Matrix3d inline rpyToRot(const Eigen::Vector3d & rpy){

    Eigen::Matrix3d Rx, Ry, Rz;
    double roll, pitch, yaw;

    roll = rpy(0);
    pitch = rpy(1);
    yaw = rpy(2);

    Rx <<	1   ,    0     	  ,  	  0,
            0   ,    cos(roll) ,  sin(roll),
            0   ,    -sin(roll),  cos(roll);


    Ry << cos(pitch) 	,	 0  ,   -sin(pitch),
            0       ,    1  ,   0,
            sin(pitch) 	,	0   ,  cos(pitch);

    Rz << cos(yaw)  ,  sin(yaw) ,		0,
            -sin(yaw) ,  cos(yaw) ,  		0,
            0      ,     0     ,       1;


    return Rx*Ry*Rz;

}

/**
 * @brief the dual of rpyToRot()
 * @param R matrix \f$ {}_B R_A\f$ which maps a vector\f${}_A\mathbf{v}\f$
 *  (expressed in a fixed frame A) to a vector \f${}_B\mathbf{v}\f$ expressed
 *  in a (rotated) frame B such that \f${}_A\mathbf{v} = {}_B R_A {}_B\mathbf{v} \f$
 * @return a set of Euler angles (according to ZYX convention) representing the
 * orientation of frame B
 * @sa rpyToRot()
 */
Eigen::Vector3d inline rotTorpy(Eigen::Matrix3d R)
{
    Eigen::Vector3d rpy;
    rpy(0) = atan2f((float) R(1,2), (float) R(2,2));
    rpy(1) = -asinf((float) R(0,2));
    rpy(2) = atan2f((float) R(0,1), (float) R(0,0));

    return rpy;
}

/** \brief Function to compute the linear tranformation matrix between euler
 * rates (in ZYX convention) and omega vector, where omega is expressed in world
 * coordinates to get the component expressed in the world ortogonal frame.
 *
 * I need to multiply  the components of the vector of euler rate which is
 * expressed the rpy (non orthogonal) by the roll pitch yaw axis expressed in
 * the world frame I need to express the yaw/pitch/roll axis in world frame,
 * since we do first the rotation in the z axis, wz = yaw_d therefore
 * z = z' =[0;0;1] then we rotate about pitch so we have component for this
 * rotation in wy and -wx (y'= [-sin(yaw; cos(yaw) ;0]) if we consider roll
 * after the pitch we will have the roll axis after yaw and pitch rotation to
 * be x'' = cos(pitch)*x' -sin(pitch)*[0;0;1] where x' = [cos(yaw);sin(yaw);0]
*/
Eigen::Matrix3d inline rpyToEarInv(const Eigen::Vector3d & rpy){

    Eigen::Matrix3d EarInv;
    double pitch = rpy(1);
    double yaw = rpy(2);

    EarInv << cos(pitch)*cos(yaw), -sin(yaw), 0,
            cos(pitch)*sin(yaw),   cos(yaw),    0,
            -sin(pitch),         0,    1;

    return EarInv;
}

/**
 * @brief rpyToEar Function to compute the linear tranformation matrix between
 * euler rates (in ZYX convention) and omega vector where omega is expressed
 * in base coordinates (is R*EarInv)
 * @param rpy
 * @return
 */
Eigen::Matrix3d inline  rpyToEar(const Eigen::Vector3d & rpy){

    Eigen::Matrix3d Ear;
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    Ear<< 1,         0,         -sin(pitch),
            0,  cos(roll), cos(pitch)*sin(roll),
            0, -sin(roll), cos(pitch)*cos(roll);


    return Ear;
}

/**
 * @brief dervative of rpyToEarInv
 * @param rpy
 * @param rpyd
 * @return
 * @sa rpyToEarInv()
 */
Eigen::Matrix3d inline rpyToEarInv_dot(const Eigen::Vector3d & rpy, const Eigen::Vector3d & rpyd){

    Eigen::Matrix3d EarInv_dot;
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);
    double rolld = rpyd(0);
    double pitchd = rpyd(1);
    double yawd = rpyd(2);

    EarInv_dot << - cos(yaw)*sin(pitch)*pitchd - cos(pitch)*sin(yaw)*yawd, - cos(yaw)*yawd, 0,
            cos(yaw)*cos(pitch)*yawd - sin(yaw)*sin(pitch)*pitchd, -sin(yaw)*yawd, 0,
            -cos(pitch)*pitchd,  0, 0;

    return EarInv_dot;
}

/**
 * @brief twovecToRot find the rotation matrix \f$ R \f$ that rotates \f$ \mathbf{v}_1\f$
 * into \f$ v_2\f$ so that \f$ v_2 = R \mathbf{v}_1\f$
 * @param v1 first vector
 * @param v2 second vector
 * @return the rotation matrix \f$ R \f$
 * @remark the matrix orientation of the rotated frame is the transposed of the
 *  rotation matrix which rotates the vector in the fixed frame to align with
 *  the rotated frame
 */
Eigen::Matrix3d inline twovecToRot(const Eigen::Vector3d & v1,
                                   const Eigen::Vector3d & v2)
{
    Eigen::Vector3d axis;
    Eigen::Matrix3d base;
    Eigen::Matrix3d coordChangeMat;

    //check if there is a singulatiry in the cross product
    if (v1 == v2) {

        return Eigen::Matrix3d::Identity();
    }
    else {
        //do cross product to find the axis
        axis=v1.cross(v2);
        axis.normalize(); //it is fundamental to normalize it

        //build the new basis formed by the 3 vectors

        base.col(0) = v1.normalized();
        base.col(1) = v2.normalized();
        base.col(2) = axis;

        //rotation is like a change of coord between v1 and v2 leaving the axis unchanged

        coordChangeMat << 0, 1, 0,
                1, 0, 0,
                0, 0, 1;
        //the rotation matrix is in the original space

        return base * coordChangeMat * base.inverse();
    }
}
/**
 * @brief rpyToquat
 * @param[in] rpy
 * @return
 * @remark using ZYX convention for rpy
 * (see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
 */
Quaternion inline rpyToquat(const Eigen::Vector3d& rpy) {
    Quaternion q;
    double roll, pitch, yaw;

    roll = rpy(0);
    pitch = rpy(1);
    yaw = rpy(2);

    q.w() = cos(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0)+sin(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
    q.x() = sin(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0)-cos(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
    q.y() = cos(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0)+sin(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0);
    q.z() = cos(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0)-sin(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0);


    return q;
}

/**
 * @brief quatToOmega computes the angular velocity (in body coordinates)
 *  from quaternion position:
 * \f$\mathbf{q}\f$ and derivatives \f$\dot{\mathbf{q}}\f$
 * \f[
 * {}_B\boldsymbol{\omega} = W'(\mathbf{q})\dot{\mathbf{q}}
 * \f]
 * @param[in] q
 * @param[in] qd
 * @return the angular velocity (in body coordinates)
 * @date July 2005
 */
Eigen::Vector3d inline quatToOmega(Quaternion q, Quaternion qd)
{

    Eigen::Matrix<double,3,4> W;
    Eigen::Vector4d qd_vector;

    qd_vector << qd.w(), qd.x(), qd.y(), qd.z();

    W(0,0) = -q.x();
    W(0,1) = q.w();
    W(0,2) = q.z();
    W(0,3) = -q.y();

    W(1,0) = -q.y();
    W(1,1) = -q.z();
    W(1,2) = q.w();
    W(1,3) = q.x();

    W(2,0) = -q.z();
    W(2,1) = q.y();
    W(2,2) = -q.x();
    W(2,3) = q.w();


    return 2.0*W*qd_vector;

}

/**
 * @brief rotMatToQuat
 * @param R
 * @return
 * @remark using formula from attitude document the input matrix.
 * Should be \f${}_aR_w\f$, where \f$a\f$ is the rotated frame
 */
inline Quaternion  rotMatToQuat(const Eigen::Matrix3d & R) {
    Quaternion quat;

    ////compute the trace of R
    //double T = 1 + R(0,0) + R(1,1)  + R(2,2);
    ////If the trace of the matrix is greater than zero, then
    ////perform an "instant" calculation.
    //
    //if (T>1e-03) {
    //	quat.w() = 0.5* sqrt(T);
    //	quat.x() = 0.5*(R(1,2) - R(1,2))/sqrt(T);
    //	quat.y() = 0.5*(R(2,0) - R(2,0))/sqrt(T);
    //	quat.z() = 0.5*(R(0,1) - R(0,1))/sqrt(T);
    //} else {
    //	std::cout<<"cannot compute quaternion";
    //}

    double tr = R.trace();
    double sqrt_tr;

    if (tr > 1e-06)
    {
        sqrt_tr = sqrt(R.trace() + 1);
        quat.w() = 0.5*sqrt_tr;
        quat.x() = (R(1,2) - R(2,1))/(2.0*sqrt_tr);
        quat.y() = (R(2,0) - R(0,2))/(2.0*sqrt_tr);
        quat.z() = (R(0,1) - R(1,0))/(2.0*sqrt_tr);
    }
    else  if ((R(1,1) > R(0,0)) && (R(1,1) > R(2,2)))
    {
        // max value at R(1,1)
        sqrt_tr = sqrt(R(1,1) - R(0,0) - R(2,2) + 1.0 );

        quat.y() = 0.5*sqrt_tr;

        if ( sqrt_tr > 1e-06 ) sqrt_tr = 0.5/sqrt_tr;


        quat.w() = (R(2, 0) - R(0, 2))*sqrt_tr;
        quat.x() = (R(0, 1) + R(1, 0))*sqrt_tr;
        quat.z() = (R(1, 2) + R(2, 1))*sqrt_tr;

    }
    else if (R(2,2) > R(0,0))
    {
        // max value at R(2,2)
        sqrt_tr = sqrt(R(2,2) - R(0,0) - R(1,1) + 1.0 );

        quat.z() = 0.5*sqrt_tr;

        if ( sqrt_tr > 1e-06 ) sqrt_tr = 0.5/sqrt_tr;

        quat.w() = (R(0, 1) - R(1, 0))*sqrt_tr;
        quat.x() = (R(2, 0) + R(0, 2))*sqrt_tr;
        quat.y() = (R(1, 2) + R(2, 1))*sqrt_tr;
    }
    else {
        // max value at dcm(0,0)
        sqrt_tr = sqrt(R(0,0) - R(1,1) - R(2,2) + 1.0 );

        quat.x() = 0.5*sqrt_tr;

        if ( sqrt_tr > 1e-06 ) sqrt_tr = 0.5/sqrt_tr;

        quat.w() = (R(1, 2) - R(2, 1))*sqrt_tr;
        quat.y() = (R(0, 1) + R(1, 0))*sqrt_tr;
        quat.z() = (R(2, 0) + R(0, 2))*sqrt_tr;
    }

    return quat;

}

/**
 * @brief rotMatToRotVec
 * @param R
 * @return
 */
inline Eigen::Vector3d rotMatToRotVec(const Eigen::Matrix3d & R)
{
    //this formula depends on how you define R
    Eigen::Vector3d axis;
    Eigen::Matrix3d Rskew;
    double angle;


    //angle = acos( 0.5*( R(0,0)+ R(1,1)+ R(2,2)-1));
    //axis(0) =  R(1,2)-R(2,1);
    //axis(1) =  R(2,0) -R(0,2);
    //axis(2) =  R(0,1) - R(1,0);
    //axis *=0.5/sin(angle);
    //
    //return  angle*axis.normalized();

    double c = 0.5*( R(0,0)+ R(1,1)+ R(2,2)-1);
    Eigen::Vector3d w = - skew_simToVec(R);
    double s = w.norm(); //w = sin(theta)*axis

    if (fabs(s) <= 1e-10)
        return Eigen::Vector3d::Zero();

    else {
        angle = atan2(s,c);
        axis = w/s;
        return angle*axis;}

    /*
     // a more robust implementation (from Roy):
    double c = 0.5*( R(0,0)+ R(1,1)+ R(2,2)-1);
    Eigen::Vector3d w = - skew_simToVec(R);
    double s = w.norm();
    angle = atan2(s,c);

    if (s == 0)
        return Eigen::Vector3d::Zero();

    else {
        if  (angle< 0.9*M_PI){ //			% a somewhat arbitrary threshold
            axis = w/s;
            return angle*axis;}
        else
        {				//% extract v*v' component from R and
            Rskew = R - c * Eigen::Matrix3d::Identity();// pick biggest column (best chance
            Rskew = Rskew + Rskew.transpose();				// to get sign right)

            if ((Rskew(0,0) >= Rskew(1,1)) && (Rskew(0,0) >= Rskew(2,2)))
            {
                if (w(0) >= 0)
                    axis = R.col(0);
                else
                    axis = -R.col(0);
            }

            else
            {
                if (Rskew(1,1) >= Rskew(2,2))
                {
                    if (w(1) >= 0)
                        axis = R.col(1);
                    else
                        axis = R.col(1);
                }
                else
                {
                    if  (w(2) >= 0)
                        axis = R.col(2);
                    else
                        axis = -R.col(2);
                }
            }

            return  angle*axis.normalized();
        }


    }*/
}

/**
 * @brief computeOrientError
 * @param des_orient
 * @param actual_orient
 * @return
 * @sa computeOrientError(const Eigen::Matrix3d & Rdes, const Eigen::Matrix3d & Ract)
 */
Eigen::Vector3d  inline  computeOrientError(const Eigen::Vector3d & des_orient,
                                            const Eigen::Vector3d & actual_orient)
{

    Eigen::Matrix3d Ract = commons::rpyToRot(actual_orient);
    Eigen::Matrix3d Rdes = commons::rpyToRot(des_orient);
    Eigen::Vector3d err;

    //compute the rotation matrix that represent the relative orientation of
    // des_orient w.r.t actual_orient
    Eigen::Matrix3d Re = Rdes*Ract.transpose();
    //compute the angle-axis representation
    err = rotMatToRotVec(Re);

    return err;
}

/**
 * @brief compute orientation error based on rotation matrix.
 * the rotation matrix should be defined \f$ {}_{\mathrm{act}}R_w {}_{\mathrm{des}}R_w\f$
 * @param Rdes
 * @param Ract
 * @return
 */
Eigen::Vector3d  inline  computeOrientError(const Eigen::Matrix3d & Rdes, const Eigen::Matrix3d & Ract)
{

    Eigen::Vector3d err;

    // compute the rotation matrix that represent the relative orientation
    // of des_orient w.r.t actual_orient
    Eigen::Matrix3d Re = Rdes*Ract.transpose();
    //compute the angle-axis representation
    err = rotMatToRotVec(Re);

    return err;
}

/**
 * @brief compute orientation error based on quaternions
 * @param q_des
 * @param q
 * @return
 */
inline Quaternion computeOrientError(const Quaternion& q_des,
                                     const Quaternion& q)
{

    Quaternion q_e;
    int sign_flip = 1;

    // to the test to take the shortest path in the quaternion
    if (q.dot(q_des) < 0) {
        sign_flip = -1;
    }
    //compute angular error qe = qd x q1_inv
    q_e.w() = (sign_flip*q.w())*q_des.w() + (sign_flip*q.vec()).transpose()*q_des.vec();

    //compute rotation axis
    q_e.vec() = (sign_flip*q.w())*q_des.vec() - q_des.w()*q.vec() - (sign_flip*q.vec()).cross(q_des.vec());


    return q_e;

}

/**
 * @brief Computes the right-hand multiplication matrix from a given quaternion
 * @return right-hand multiplication matrix q2 * q1 = Qr(q1)*q2 (Qnot conjugate quaternion matrix)
 * @param[in]	q	quaternion
 * @sa quatL()
 */
inline Eigen::Matrix<double,4,4> quatR(const Quaternion& q) {

    Eigen::Matrix<double,4,4> M;

    M.setIdentity();
    M*=q.w();

    M(1,0) = q.x();
    M(2,0) = q.y();
    M(3,0) = q.z();
    M(0,1) = -q.x();
    M(0,2) = -q.y();
    M(0,3) = -q.z();

    M(1,2) = -q.z();
    M(1,3) = q.y();
    M(2,1) = q.z();
    M(2,3) = -q.x();
    M(3,1) = -q.y();
    M(3,2) = q.x();
    return M;

}

/**
 * @brief Computes the left-hand multiplication matrix from a given quaternion
 * @return 	left-hand multiplication matrix q2 * q1 = Ql(q2)*q1 NB Q(q_inv) = Q(q)^T
 * @param[in]	q	quaternion
 * @sa quatR()
 */
inline Eigen::Matrix<double,4,4> quatL(const Quaternion & q){
    Eigen::Matrix<double,4,4> M;

    M.setIdentity();
    M*=q.w();

    M(1,0) = q.x();
    M(2,0) = q.y();
    M(3,0) = q.z();
    M(0,1) = -q.x();
    M(0,2) = -q.y();
    M(0,3) = -q.z();


    M(1,2) = q.z();
    M(1,3) = -q.y();
    M(2,1) = -q.z();
    M(2,3) = q.x();
    M(3,1) = q.y();
    M(3,2) = -q.x();
    return M;
}



/**
 * @brief Converts a quaternion into a rotation vector
 * @param[in] q quaternion
 * @return 	the corresponding rotation vector
 */
inline Eigen::Vector3d quatToRotVec(const Quaternion & q){
    Eigen::Vector3d v;
    const double c = q.w();
    v = q.vec();
    const double s = v.norm();
    if(fabs(s) >= 1e-10){
        const double a = 2*atan2(s,c);
        return v*a/s;
    } else {
        return v*2;
    }
}

/**
 * @brief Converts a quaternion to a rpy ZYX convention
 * @param[in] q quaternion
 * @return corresponding rpy vector
 */
inline Eigen::Vector3d quatToRPY(const Quaternion & q){
    Eigen::Vector3d rpy;
    rpy = rotTorpy(quatToRotMat(q));
    return rpy;
}

/**
 * Converts a rotation vector to a quaternion
 * @param[in]	v	rotation vector
 * @return 	corresponding quaternion
 */
inline Quaternion rotVecToQuat(const Eigen::Vector3d& v){
    Quaternion q;
    const double a = v.norm();
    q.w() = cos(a/2);
    if(a >= 1e-10){
        q.vec() = sin(a/2)/a*v;
    } else {
        q.vec() = v/2;
    }
    q.normalize();
    return q;
}

/**
 * @brief rotVecToRotMat computes the rotation matrix associated to the rot_vector.
 * the rotation matrix maps a vector in a frame which is rotated about the
 * rot_vector wrt the original frame if you want to rotate the vector in the
 * original frame you should multiply by the transpose of this matrix
 * @param v
 * @return
 */
inline Eigen::Matrix3d rotVecToRotMat(const Eigen::Vector3d& v){

    Eigen::Matrix3d R;

    double angle = v.norm();
    if (fabs(angle) >= 1e-10) {
        Eigen::Vector3d axis = v.normalized();
        //using rodriguez formula Matrix3f R; R = AngleAxisf(0.33*M_PI, Vector3f::UnitZ());
        R = cos(angle) *Eigen::Matrix3d::Identity() + (1-cos(angle))*axis*axis.transpose() - sin(angle)*skew_sim(axis);
    } else {
        R.setIdentity();
    }
    return R;
}

/**
 * @brief computes the rotation matrix of a rotation expressed with the
 * angle-axis representation
 * @param angle
 * @param axis
 * @return the rotation matrix if the absolute value of the angle
 * is \f$\ge 1\times10^{-10}\f$, an identity otherwise.
 */
inline Eigen::Matrix3d rotVecToRotMat(const double& angle,
                                      const Eigen::Vector3d&  axis)
{

    if (fabs(angle) >= 1e-10){
        return rotVecToRotMat(angle*axis.normalized());
    }

    return Eigen::Matrix3d::Identity();
}

}
}
