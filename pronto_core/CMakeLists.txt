cmake_minimum_required(VERSION 2.8.3)

project(pronto_core)
add_compile_options(-std=c++11)
set(PRONTO_CORE_LIB ${PROJECT_NAME})

#required by the lcm-based state estimator library
find_package(catkin REQUIRED COMPONENTS stl_utils
                                        eigen_utils)

find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PRONTO_CORE_LIB}
               CATKIN_DEPENDS eigen_utils stl_utils
               DEPENDS EIGEN3)

# this library contains only computation, no dependency on LCM
add_library (${PRONTO_CORE_LIB} src/rbis.cpp
                                src/ins_module.cpp
                                src/scan_matcher_module.cpp
                                src/rbis_update_interface.cpp
                                src/state_est.cpp
                                src/update_history.cpp
                                src/visual_odometry_module.cpp
                                src/lidar_odometry_module.cpp
                                src/vicon_module.cpp
                                src/indexed_meas_module.cpp
                                src/init_message_module.cpp
                                src/gps_module.cpp
                                src/pose_meas_module.cpp)

target_link_libraries(${PRONTO_CORE_LIB} ${catkin_LIBRARIES})

catkin_add_gtest(ins_module_ballistic test/imu_module_test.cpp)
target_link_libraries(ins_module_ballistic ${catkin_LIBRARIES} ${PRONTO_CORE_LIB} pthread)


install(TARGETS ${TARGETS}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)
