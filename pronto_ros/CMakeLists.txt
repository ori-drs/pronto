cmake_minimum_required(VERSION 3.0.2)
project(pronto_ros)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall)

find_package(catkin REQUIRED COMPONENTS pronto_core
                                        roscpp
                                        sensor_msgs
                                        eigen_conversions
                                        tf_conversions
                                        tf2_ros
                                        nav_msgs
                                        pronto_msgs
                                        geometry_msgs)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS pronto_core
                              roscpp
                              sensor_msgs
                              tf_conversions
                              tf2_ros
                              eigen_conversions
                              nav_msgs
                              pronto_msgs
                              geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/ins_ros_handler.cpp
                            src/vicon_ros_handler.cpp
                            src/gps_ros_handler.cpp
                            src/index_meas_ros_handler.cpp
                            src/init_message_ros_handler.cpp
                            src/scan_matcher_ros_handler.cpp
                            src/visual_odometry_ros_handler.cpp
                            src/lidar_odometry_ros_handler.cpp
                            src/ros_frontend.cpp
                            src/pose_msg_ros_handler.cpp
                            src/pronto_ros_conversions.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/pronto_ros_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(lidar_odometry_visualizer src/lidar_odom_viz.cpp)
target_link_libraries(lidar_odometry_visualizer ${catkin_LIBRARIES})
add_dependencies(lidar_odometry_visualizer ${catkin_EXPORTED_TARGETS})

# Node to republish pronto_msgs/JointStateWithAcceleration to sensor_msgs/JointState
add_executable(joint_state_with_acceleration_republisher src/joint_state_with_acceleration_republisher.cpp)
target_link_libraries(joint_state_with_acceleration_republisher ${pronto_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${catkin_LIBRARIES})
install(
  TARGETS joint_state_with_acceleration_republisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} lidar_odometry_visualizer ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pronto_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
