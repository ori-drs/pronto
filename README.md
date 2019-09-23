![](pronto_core/doc/pronto_logotype_se.svg)


## Introduction

Pronto is an efficient, versatile and modular EKF state estimator for both
proprioceptive (inertial, kinematics) and exteroceptive (LIDAR, camera) sensor
fusion. It provided the state estimate that was used by MIT DRC team in the
DARPA Robotics Challenge to estimate the position and motion of the Boston
Dynamics Atlas robot.

It has since been adapted to estimate the motion of the NASA Valkyrie robot at
the University of Edinburgh, the HyQ quadruped robot at the Istituto Italiano di
Tecnologia, and the ANYmal quadruped robot at the University of Oxford.

Pronto has been used with a variety of inputs from sensors such as IMUs
(Microstrain, KVH, XSense), LIDAR (Hokuyo, Velodyne), cameras
(Carnegie Robotics Multisense SL, Intel RealSense) and joint
kinematics.

[![image](http://img.youtube.com/vi/V_DxB76MkE4/0.jpg)](https://www.youtube.com/watch?v=V_DxB76MkE4)

[Pronto youtube video](https://www.youtube.com/watch?v=V_DxB76MkE4)

### Software Overview

The algorithms (and their ROS wrappers) are written in C/C++ and organized as
`catkin` packages.
The repository consists of the following main modules:

- `pronto_core`: core libraries that implment the filter, the state and
basic measurement modules (e.g., IMU, pose update)
- `pronto_biped` leg odometry measurement modules for humanoid robots (tested
on Atlas and Valkyrie)
- `pronto_quadruped` leg odometry libraries for a quadruped robot (tested on
HyQ and ANYmal)
- `pronto_quadruped_commons` abstract interfaces to perform leg odometry on a
quadruped robot. This is a fork of the `iit_commons` package (see
[here](https://github.com/iit-DLSLab/iit_commons).
- `*_ros` ROS wrappers of the above modules


Building the Code
-----------------

These compile instructions were tested on a fresh Ubuntu 14.04 install,
but is likely to work on other versions of Linux and MacOS.

### Requirements:

Install these common system dependencies:

### Compiling

Check out the source code using git:

    git clone https://github.com/ipab-slmc/pronto-distro.git
    cd pronto-distro
    git submodule update --init --recursive

Then start compiling:

    make

The compile time is about 4 mins.

### Using with your own robot

To use the estimator on your robot, you simply need
to provide the required inputs to our system:

-   IMU measurements of type ins\_t.lcm (ROS: sensor\_msgs/Imu)
    \* Also support the KVH 1750 IMU which is in the Atlas
-   Joint States of type joint\_states\_t.lcm (ROS:
    sensor\_msgs/JointState)
-   Force Torque sensor of type six\_axis\_force\_torque\_array\_t.lcm
    (ROS: geometry\_msgs/WrenchStamped)

Pronto will output:

-   /state_estimator_pronto/pose - the position and orientation of the robot's
    base link (in world coordinates)
-  /state_estimator_pronto/twist - the linear/angular velocity of the robot's base link (in base coordinates)

#### Using the estimator with a third party controller

We have successfully used Pronto with 4 other bipeds (including NASA
Valkyrie) and a quadruped. If you are interested in using the estimator
with your own controller, please get in touch.

At MIT and Edinburgh we use Pronto as our 333Hz Drake controller in a
high-rate control loop. Latency and relability have allowed us to
demonstrate challenging locomotion using the Atlas robot.

### Micro Aerial Vehicles

Pronto was originally developed for Micro Aerial Vehicle state
estimation.

[![image](http://img.youtube.com/vi/kYs215TgI7c/0.jpg)](https://www.youtube.com/watch?v=kYs215TgI7c)

[Micro aerial vehicle estimation using
Pronto](https://www.youtube.com/watch?v=kYs215TgI7c)

Log files demonstrating flight with Quadrotators and Fixed-wing RC
Planes can be provided on request.

Supported sensor of interest to aerial flight:

-   GPS - x, y, z
-   Vicon - x, y, z and orientation
-   Laser Scanmatcher - x, y, z and yaw or velocity and yaw rate
-   Optical Flow - velocity, yaw rate (downward facing camera)
-   Airspeed - forward velocity
-   Altimeter - z
-   Sideslip - lateral velocity

And example configuration for these sensors is in
docs/aerial\_sensors\_example.cfg

Publications
------------

-   State Estimation for Aggressive Flight in GPS-Denied Environments
    Using Onboard Sensing, A. Bry, A. Bachrach, N. Roy, ICRA 2012.
-   Drift-Free Humanoid State Estimation fusing Kinematic, Inertial and
    LIDAR sensing, M. Fallon, M. Antone, N. Roy, S. Teller. Humanoids
    2014.

Credits
-------

Originally Developed by Adam Bry, Abe Bachrach and Nicholas Roy of the
[MIT Robust Robotics Group](http://groups.csail.mit.edu/rrg/).

Extended to support humanoid motion by Maurice Fallon with the help of
the [MIT DARPA Robotics Challenge Team](http://www.drc.mit.edu).

Extended to support quadruped robots and full ROS conversion by 
Marco Camurri 

Additional contributions from:

-   Andy Barry
-   Pat Marion

The License information is available in the LICENSE file attached to
this document.

Maurice Fallon, Feb 2015. <mfallon@robots.ox.ac.uk>
Marco Camurri, Sep 2019 <mcamurri@robots.ox.ac.uk>
