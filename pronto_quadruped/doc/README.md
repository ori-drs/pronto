# pronto\_quadruped {#mainpage}
This repository contains the quadrupedal Leg Odometry libraries (both standalone and ROS wrappers) for Pronto.  
The repository contains three catkin packages:
- **pronto_quadruped_commons**
- **pronto_quadruped** 
- **pronto_quadruped_ros**.

## pronto\_quadruped\_commons
This package contains the minimal interfaces to perform kinematics and dynamics on a generic quadruped robot.
This code is a fork of [**iit_commons**](https://github.com/iit-DLSLab/iit_commons) and has been originally  
developed in IIT by the [Dynamic Legged Systems Lab](https://github.com/iit-DLSLab).

## pronto\_quadruped
This package contains the libraries required to compute Leg Odometry for a generic quadruped robot with 12 Degrees-of-Freedom.
Implementations for the Kinematics and the Dynamics according to the interfaces defined in the `pronto_quadruped_commons` package have to 
provided if you want to use these classes on a specific quadruped robot.

## pronto\_quadruped\_ros
This package is a ROS wrapper for `pronto_quadruped`. 
It converts the ROS messages for joint states and IMU into the internal
format used by `pronto_quadruped` and calls its methods.

## Dependencies
The `pronto_quadruped_ros` depends on:

- roscpp
- pronto\_core
- geometry\_msgs
- sensor\_msgs
- pronto\_core

## Publications
If you use this work in an academic context, please cite the following publication:<br/> 
*M. Camurri, M. Fallon, S. Bazeille, A. Radulescu, V. Barasuol, D.G. Caldwell, C. Semini*<br/>
**Probabilistic Contact Estimation and Impact Detection for State Estimation of Quadruped Robots**<br/>
in IEEE Robotics and Automation Letters, vol. 2, no. 2, pp. 1023-1030, April 2017 
([PDF](https://iit-dlslab.github.io/papers/camurri17ral.pdf))  
**DOI:** [10.1109/LRA.2017.2652491](https://www.doi.org/10.1109/LRA.2017.2652491)  

    @article{camurri2017ral,
      author={M. {Camurri} and M. {Fallon} and S. {Bazeille} and A. {Radulescu} and V. {Barasuol} and D. G. {Caldwell} and C. {Semini}},
      journal={IEEE Robotics and Automation Letters},
      title={{Probabilistic Contact Estimation and Impact Detection for State Estimation of Quadruped Robots}},
      year={2017},
      volume={2},
      number={2},
      pages={1023-1030},
      doi={10.1109/LRA.2017.2652491},
      ISSN={2377-3766},
      month={April}}
      
