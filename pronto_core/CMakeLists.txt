cmake_minimum_required(VERSION 3.0.2)

project(pronto_core)
set(CMAKE_CXX_STANDARD 14)
set(PRONTO_CORE_LIB ${PROJECT_NAME})

add_compile_options(-Wall)

find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PRONTO_CORE_LIB}               
               DEPENDS EIGEN3)

add_library(${PRONTO_CORE_LIB} src/rbis.cpp
                               src/rotations.cpp
                               src/rigidbody.cpp
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

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(ins_module_ballistic test/imu_module_test.cpp)
  target_link_libraries(ins_module_ballistic ${catkin_LIBRARIES} ${PRONTO_CORE_LIB} pthread)
endif()

install(TARGETS ${TARGETS} ${PRONTO_CORE_LIB}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")
