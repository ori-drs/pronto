![](pronto_core/doc/pronto_logotype_se.svg)

## Introduction
Pronto is an efficient, versatile and modular EKF state estimator for both
proprioceptive (inertial, kinematics) and exteroceptive (LIDAR, camera) sensor
fusion.  It has been used with a variety of inputs from sensors such as IMUs
(Microstrain, KVH, XSense), LIDAR (Hokuyo, Velodyne), cameras
(Carnegie Robotics Multisense SL, Intel RealSense) and joint
kinematics.

### Legged Robots
Pronto provided the state estimate that was used by MIT DRC team in the
DARPA Robotics Challenge to estimate the position and motion of the Boston
Dynamics Atlas robot.

[![image](http://img.youtube.com/vi/V_DxB76MkE4/0.jpg)](https://www.youtube.com/watch?v=V_DxB76MkE4)
[Pronto on Atlas](https://www.youtube.com/watch?v=V_DxB76MkE4)

Since then, it has been adapted to estimate the motion of the NASA Valkyrie robot at
the University of Edinburgh, the HyQ quadruped robot at the Istituto Italiano di
Tecnologia, and the ANYmal quadruped robot at the University of Oxford.

[![image](http://img.youtube.com/vi/39Y1Jx1DMO8/0.jpg)](https://www.youtube.com/watch?v=39Y1Jx1DMO8)
[Pronto on HyQ](https://www.youtube.com/watch?v=39Y1Jx1DMO8)

### Micro Aerial Vehicles
Pronto was originally developed for Micro Aerial Vehicle state
estimation. The modules specific to MAVs (e.g., altimeter, GPS) are not currently supported.

## Software Overview
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
[here](https://github.com/iit-DLSLab/iit_commons)).
- `*_ros` ROS wrappers of the above modules
- other support packages for filtering

## Dependencies
Pronto depends on Eigen, Boost, and on the catkin packages in the following repositories (likely to be removed in future)
- [common\_utils](https://github.com/ori-drs/common_utils)
- [kinematics\_utils](https://github.com/ori-drs/kinematic_utils)

## System Requirements
The target operating system is **Ubuntu 18.04** equipped with **ROS Melodic**.  
 Other versions of Ubuntu/ROS might work but they are **not** actively supported or tested.

## Building the Code
Pronto is organized as a collection of catkin packages. To build the code,
just run `catkin build` followed by the name of the packages you are 
interested to build.

## Robot Implementation Example
To learn how to use Pronto on your robot, you can have a look at [this repository](https://github.com/ori-drs/pronto_anymal_example), which contains a full implementation on the ANYmal quadruped robot. 
## Publications
If you use part of this work in academic context, please cite the appropriate
publication from the list below:

*S. Nobili, M. Camurri, V. Barasuol, M. Focchi, D.G. Caldwell, C. Semini, M. Fallon*  
**Heterogeneous Sensor Fusion for Accurate State Estimation of Dynamic Legged Robots**  
in Proceedings of Robotics: Science and Systems XIII, 2017 ([PDF](http://www.robots.ox.ac.uk/~mobile/drs/Papers/2017RSS_nobili.pdf)) **DOI:** [10.15607/RSS.2017.XIII.007](https://www.doi.org/10.15607/RSS.2017.XIII.007)

```
@inproceedings{nobili2017rss,
    author = {Simona Nobili AND Marco Camurri AND Victor Barasuol AND Michele Focchi AND Darwin Caldwell AND Claudio Semini AND Maurice Fallon}, 
    title = {{Heterogeneous Sensor Fusion for Accurate State Estimation of Dynamic Legged Robots}}, 
    booktitle = {Proceedings of Robotics: Science and Systems}, 
    year = {2017}, 
    address = {Cambridge, Massachusetts}, 
    month = {July}, 
    doi = {10.15607/RSS.2017.XIII.007} 
}
```

*M. Camurri, M. Fallon, S. Bazeille, A. Radulescu, V. Barasuol, D.G. Caldwell, C. Semini*  
**Probabilistic Contact Estimation and Impact Detection for State Estimation of Quadruped Robots**  
in IEEE Robotics and Automation Letters, vol. 2, no. 2, pp. 1023-1030, April 2017 ([PDF](https://iit-dlslab.github.io/papers/camurri17ral.pdf)) **DOI:** [10.1109/LRA.2017.2652491](https://www.doi.org/10.1109/LRA.2017.2652491)

```
@article{camurri2017ral,
      author={M. {Camurri} and M. {Fallon} and S. {Bazeille} and A. {Radulescu} and V. {Barasuol} and D. G. {Caldwell} and C. {Semini}},
      journal={IEEE Robotics and Automation Letters},
      title={{Probabilistic Contact Estimation and Impact Detection for State Estimation of Quadruped Robots}},
      year = {2017},
      volume = {2},
      number = {2},
      pages = {1023-1030},
      doi = {10.1109/LRA.2017.2652491},
      ISSN = {2377-3766},
      month = {April}}
```

*M. Fallon, M. Antone, N. Roy, S. Teller*  
**Drift-Free Humanoid State Estimation fusing Kinematic, Inertial and LIDAR sensing**  
2014 IEEE-RAS International Conference on Humanoid Robots ([PDF](https://www.research.ed.ac.uk/portal/files/18903340/14_fallon_humanoids.pdf)) **DOI:**[10.1109/HUMANOIDS.2014.7041346](https://www.doi.org/10.1109/HUMANOIDS.2014.7041346)

```
@inproceedings{fallon2014humanoids,
author={M. F. {Fallón} and M. {Antone} and N. {Roy} and S. {Teller}},
booktitle={2014 IEEE-RAS International Conference on Humanoid Robots},
title={Drift-free humanoid state estimation fusing kinematic, inertial and LIDAR sensing},
year={2014},
volume={},
number={},
pages={112-119},
doi={10.1109/HUMANOIDS.2014.7041346},
ISSN={},
month={Nov},}
```

*A. Bry, A. Bachrach, N. Roy*  
**State Estimation for Aggressive Flight in GPS-Denied Environments Using Onboard Sensing**  
2012 IEEE International Conference on Robotics and Automation ([PDF](https://dspace.mit.edu/bitstream/handle/1721.1/86237/icra12_aggressive_flight.pdf)) **DOI:**[10.1109/ICRA.2012.6225295](https://www.doi.org//10.1109/ICRA.2012.6225295)

```
@inproceedings{bry2012icra,
author={A. {Bry} and A. {Bachrach} and N. {Roy}},
booktitle={2012 IEEE International Conference on Robotics and Automation},
title={State estimation for aggressive flight in GPS-denied environments using onboard sensing},
year={2012},
volume={},
number={},
pages={1-8},
doi={10.1109/ICRA.2012.6225295},
ISSN={},
month={May},}
```

## Credits

Originally Developed by Adam Bry, Abe Bachrach and Nicholas Roy of the
[MIT Robust Robotics Group](http://groups.csail.mit.edu/rrg/) for Micro Aerial Vehicles.

Extended to support humanoid motion by Maurice Fallon with the help of
the [MIT DARPA Robotics Challenge Team](http://www.drc.mit.edu).

Support for quadruped robots, full ROS conversion and logo design by 
Marco Camurri  ([IIT Dynamic Legged System Lab](http://dls.iit.it) and [ORI Dynamic Robot Systems Group](https://ori.ox.ac.uk/labs/drs/))

Additional contributions from: Andy Barry, Pat Marion, Dehann Fourie, Marco Frigerio, Michele Focchi, Benoit Casseau.

## License
Pronto is released under the LGPL v2.1 license. Please see the LICENSE file attached to
this document for more information.

