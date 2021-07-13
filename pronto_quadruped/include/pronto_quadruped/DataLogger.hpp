/* Copyright (c) 2015-2019
 * Istituto Italiano di Tecnologia (IIT), University of Oxford
 * All rights reserved.
 *
 * Author: Marco Camurri (mcamurri@robots.ox.ac.uk)
 *
 * This file is part of pronto_quadruped,
 * a library for leg odometry on quadruped robots.
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

#include <fstream>
#include <Eigen/Dense>
#include <pronto_quadruped_commons/leg_data_map.h>

namespace pronto {
/**
 * @brief The DataLogger class stores on plain text ASCII files the content of
 * debug variables like poses, velocities, and general-purpose signals.
 * These files can be used to create a dataset, or to automatically process
 * these variables with matlab scripts.
 * @warning If the user calls different overloads of the addSample() call,
 * the class will not complain and write the lines accordingly, but probably
 * the file will not be correctly readable, because of the different types and
 * sizes of information in each line.
 */
class DataLogger {
public:
    template <class T>
    using LegDataMap = pronto::quadruped::LegDataMap<T>;
public:
    /**
     * @brief DataLogger constructor. Takes a path as optional input. If no path
     * is given, then the file chosen is $HOME/dls_logs/samples.txt
     * @param path the absolute path of a textfile. If the path does not exist,
     * the class will try to handle this situation by creating the missing
     * folders. If the file exists, it warns the user and overwrite.
     */
    DataLogger(std::string path  = "samples.txt");
    ~DataLogger();


    /**
     * @brief write a line on the file containing a time and a vector
     * @param[in] time an absolute time in seconds
     * @param[in] tx x component of the vector
     * @param[in] ty y component of the vector
     * @param[in] tz z component of the vector
     */
    void addSample(double time,
                   double tx,
                   double ty,
                   double tz);

    /**
     * @brief writes a four-dimentional vector on file
     * @param[in] time an absolute time in seconds
     * @param[in] v1 first component of the vector
     * @param[in] v2 second component of the vector
     * @param[in] v3 third component of the vector
     * @param[in] v4 fourth component of the vector
     */
    void addSample(double time,
                   double v1,
                   double v2,
                   double v3,
                   double v4);

    /**
     * @brief writes a three-dimentional vector for each leg on file, plus time
     * for a total of 13 components. First it writes time, and then the three
     * components of the first leg, then of the second legs, etc. The legs
     * follows the same convetion of pronto::quadruped::LegDataMap
     * @param[in] time an absolute time in seconds
     * @param[in] leg_vectors a data structure containing a three-dimensional vector
     * per each leg
     */
    void addSample(double time, LegDataMap<Eigen::Vector3d> leg_vectors);


    /**
     * @brief writes a time and the content of a Eigen::Vector3d vector on file
     * @param time an absolute time, in seconds
     * @param vec the vector to be written
     */
    void addSample(const double& time,
                   const Eigen::Vector3d& vec);

    /**
     * @brief writes a time and pose. First it writes the time, then the three
     * components of the absolute positions, then the quaternion representing
     * the attitude of the rigid body. The first quaternion component to be
     * written is W, then X, Y and Z.
     * @param[in] time an absolute time, in seconds
     * @param[in] position absolute position of the rigid body
     * @param[in] orientation orientation of the rigid body
     */
    void addSample(const double& time,
                   const Eigen::Vector3d& position,
                   const Eigen::Quaterniond& orientation);

    /**
     * @brief writes the velocity of a rigid body at a specific time.
     * @param time an absolute time (in seconds)
     * @param velocity body velocity espressed in the body frame
     * @param omega angular velocity of the body, in the body frame
     */
    void addSample(const double& time,
                   const Eigen::Vector3d& velocity,
                   const Eigen::Vector3d& omega);

    /**
     * @brief writes the velocity of a rigid body at a specific time.
     * @param time an absolute time (in seconds)
     * @param velocity body velocity espressed in the body frame
     * @param omega angular velocity of the body, in the body frame
     */
    void addSampleCSV(const double& time,
                      const Eigen::Vector3d& velocity,
                      const Eigen::Vector3d& omega);

    /**
     * @brief writes the pose of a rigid body at a specific time.
     * @param time an absolute time (in seconds)
     * @param x x-component of the body position
     * @param y y-component of the body position
     * @param z z-component of the body position
     * @param roll roll Euler angle of the rigid body (ZYX convention)
     * @param pitch pitch Euler angle of the rigid body (ZYX convention)
     * @param yaw yaw Euler angle of the rigid body (ZYX convention)
     */
    void addSample(double time,
                   double x,
                   double y,
                   double z,
                   double roll,
                   double pitch,
                   double yaw);

    /**
     * @brief writes the pose of a rigid body at specific time.
     * @param time an absolute time (in seconds)
     * @param tx x-component of the body position
     * @param ty y-component of the body position
     * @param tz z-component of the body position
     * @param rx x-component of the quaternion representing the body orientation
     * @param ry y-component of the quaternion representing the body orientation
     * @param rz z-component of the quaternion representing the body orientation
     * @param rw w-component of the quaternion representing the body orientation
     */
    void addSample(double time,
                   double tx,
                   double ty,
                   double tz,
                   double rx,
                   double ry,
                   double rz,
                   double rw);

    /**
     * @brief addSampleFreiburg adds samples in the freiburg file format
     * (see https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)
     * time tx ty tz qx qy qz qw
     * @param[in] time gives the number of seconds since the Unix epoch
     * @param[in] translation give the position of the optical center of the color
     * camera with respect to the world origin as defined by the motion capture
     * system
     * @param[in] rotation gives the orientation of the optical center of the color
     * camera in form of a unit quaternion with respect to the world origin
     * as defined by the motion capture system
     */
    void addSampleFreiburg(const double& time,
                           const Eigen::Vector3d& translation,
                           const Eigen::Quaterniond& rotation);

    /**
     * @brief writes a simple scalar value at specific time
     * @param[in] time an absolute time (in seconds)
     * @param[in] value a scalar value
     */
    void addSample(double time,
                   double value);

    void addSample(const double& time,
                   const Eigen::Affine3d& transf);

    /**
     * @brief addSampleCSV stores a pose in CSV format with following convention
     * time , x , y , z , qx, qy, qz, qw
     * time is in seconds from epoch, xyz are in meters
     * @param[in] time time since the epoch in seconds
     * @param[in] position position of the robot in meters
     * @param[in] orientation
     */
    void addSampleCSV(const double& time,
                      const Eigen::Vector3d& position,
                      const Eigen::Quaterniond& orientation);

    /**
     * @brief tells the class whether we should start from zero (this subtracts
     * the very first time given by a addSample() call from the subsequent
     * times of the other calls)
     * @param[in] start_from_zero if true, it uses the first time as reference
     * and subtracts it from the following ones. If false, it does nothing.
     */
    void setStartFromZero(bool start_from_zero);
    /**
     * @brief resets the first timestamp to use as reference for the following
     * ones. This automatically calls setStartFromZero() with true.
     * @param time an absolute time (in seconds)
     * @sa setStartFromZero()
     */
    void setFirstTime(double time);


private:
    std::ofstream ofs;
    bool is_first_time;
    double first_time;
    bool start_from_zero;
};

} // namespace pronto
