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

#include "pronto_quadruped/DataLogger.hpp"
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <sstream>
#include <Eigen/Dense>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <boost/filesystem.hpp>
#include <pronto_quadruped_commons/geometry/rotations.h>

using namespace std;

namespace pronto {

DataLogger::DataLogger(std::string path) :
    is_first_time(true),
    first_time(0.0),
    start_from_zero(false) {

    stringstream ss;
    char* homepath = getenv("HOME");
    std::cout << "[ DataLogger ] Request to create " << path << std::endl;
    if(homepath == NULL) {
        std::cout << "[ DataLogger ] WARNING: getenv() does not work.";
        std::cout << "Using getpwuid() to get $HOME." << std::endl;
        struct passwd *pw = getpwuid(getuid());
        homepath = pw->pw_dir;
    }
    ss << homepath << "/dls_logs/";
    boost::system::error_code ec;
    // create directory does not treat existing folder as error. The function
    // returns false because no directories are created, but the code should be
    // success. To be sure, we catch also file_exists code.
    if(!boost::filesystem::create_directory(ss.str(),ec)){
        if(ec.value() != boost::system::errc::file_exists &&
                ec.value() != boost::system::errc::success){
            // in case of other errors, we just print the message and return
            std::cerr << "Impossible to create log directory: " << ec.message() << std::endl;
            return;
        } else if(ec.value() == boost::system::errc::file_exists){
            std::cout << "WARNING: " << ec.message() << std::endl;
        }
    }
    ss << path;
    cout << "[ DataLogger ] Setting path to: " << ss.str().c_str() << endl;
    cout << "[ DataLogger ] Start from zero: " << (start_from_zero ? "true" : "false") << endl;
    cout << "[ DataLogger ] Is first time: " << (is_first_time ? "true" : "false") << endl;
    ofs.open(ss.str().c_str());
}

/**
 * @brief DataLogger::~DataLogger destructor takes care of the closing of the
 * file descriptor
 */
DataLogger::~DataLogger() {
    ofs.close();
}

void DataLogger::setFirstTime(double time) {
    first_time = time;
    start_from_zero = true;
    is_first_time = false;
}

void DataLogger::setStartFromZero(bool use_first_time) {
    this->start_from_zero = use_first_time;
}

void DataLogger::addSample(double time,
                           double tx,
                           double ty,
                           double tz) {
    ofs << setprecision(14)
        << time  - first_time << "\t "
        << tx << "\t "
        << ty << "\t "
        << tz << std::endl;
}

void DataLogger::addSample(double time,
                           double v1,
                           double v2,
                           double v3,
                           double v4) {
    ofs << setprecision(14)
        << time  - first_time << "\t "
        << v1 << "\t "
        << v2 << "\t "
        << v3 << "\t "
        << v4 << std::endl;
}

void DataLogger::addSample(const double &time, const Eigen::Vector3d &vec) {
    ofs << setprecision(14)
        << time - first_time << "\t "
        << vec(0) << "\t "
        << vec(1) << "\t "
        << vec(2) << std::endl;
}

void DataLogger::addSample(const double& time,
                           const Eigen::Vector3d& position,
                           const Eigen::Quaterniond& orientation) {
    if(start_from_zero && is_first_time) {
        first_time = time;
        is_first_time = false;
    }

    Eigen::Vector3d rpy = pronto::commons::quatToRPY(orientation);

    ofs << setprecision(14)
        << time - first_time << "\t"
        << position(0) << "\t"
        << position(1) << "\t"
        << position(2) << "\t"
        << rpy(0) * 180.0 / M_PI << "\t"
        << rpy(1) * 180.0 / M_PI << "\t"
        << rpy(2) * 180.0 / M_PI << "\t" << std::endl;
}

void DataLogger::addSample(const double& time,
                           const Eigen::Affine3d& transf) {
    addSample(time, transf.translation(), Eigen::Quaterniond(transf.rotation()));
}

void DataLogger::addSampleCSV(const double& time,
                              const Eigen::Vector3d& position,
                              const Eigen::Quaterniond& orientation)
{
    if(start_from_zero && is_first_time) {
        first_time = time;
        is_first_time = false;
    }
    ofs << setprecision(14)
        << time - first_time << ","
        << position(0) << ","
        << position(1) << ","
        << position(2) << ","
        << orientation.x() << ","
        << orientation.y() << ","
        << orientation.z() << ","
        << orientation.w() << std::endl;
}

void DataLogger::addSample(const double& time,
                           const Eigen::Vector3d& velocity,
                           const Eigen::Vector3d& omega){
    if(start_from_zero && is_first_time) {
        first_time = time;
        is_first_time = false;
    }

    ofs << setprecision(14)
        << time - first_time << "\t"
        << velocity(0) << "\t"
        << velocity(1) << "\t"
        << velocity(2) << "\t"
        << omega(0) << "\t"
        << omega(1) << "\t"
        << omega(2) << std::endl;
}

void DataLogger::addSampleCSV(const double& time,
                              const Eigen::Vector3d& velocity,
                              const Eigen::Vector3d& omega){
    if(start_from_zero && is_first_time) {
        first_time = time;
        is_first_time = false;
    }

    ofs << setprecision(14)
        << time - first_time << ","
        << velocity(0) << ","
        << velocity(1) << ","
        << velocity(2) << ","
        << omega(0) << ","
        << omega(1) << ","
        << omega(2) << std::endl;
}

void DataLogger::addSample(double time, LegDataMap<Eigen::Vector3d> leg_vectors) {
    if(start_from_zero && is_first_time) {
        first_time = time;
        is_first_time = false;
    }

    ofs << setprecision(14)
        << time - first_time;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 3; j++) {
            ofs << "\t" << leg_vectors[i](j);
        }
    }
    ofs << std::endl;
}

void DataLogger::addSample(double time,
                           double x,
                           double y,
                           double z,
                           double roll,
                           double pitch,
                           double yaw) {
    ofs << setprecision(14)
        << time - first_time << " "
        << x << " "
        << y << " "
        << z << " "
        << roll << " "
        << pitch << " "
        << yaw << std::endl;
}

void DataLogger::addSample(double time,
                           double tx,
                           double ty,
                           double tz,
                           double rx,
                           double ry,
                           double rz,
                           double rw) {
    ofs << setprecision(14)
        << time - first_time << "\t "
        << tx << "\t "
        << ty << "\t "
        << tz << "\t "
        << rx << "\t "
        << ry << "\t "
        << rz << "\t "
        << rw << std::endl;
}

void DataLogger::addSampleFreiburg(const double &time,
                                   const Eigen::Vector3d &translation,
                                   const Eigen::Quaterniond &rotation){
    ofs << setprecision(14)
        << time - first_time << " "
        << translation(0) << " "
        << translation(1) << " "
        << translation(2) << " "
        << rotation.x() << " "
        << rotation.y() << " "
        << rotation.z() << " "
        << rotation.w() << std::endl;
}


void DataLogger::addSample(double time,
                           double value) {
    if(is_first_time) {
        first_time = time;
        is_first_time = false;
    }
    ofs << setprecision(14)
        << time - first_time << "\t "
        << value << std::endl;
}
} // namespace pronto
