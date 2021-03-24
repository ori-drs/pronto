cmake_minimum_required(VERSION 3.0.2)
project(pronto_quadruped_ros)

set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        pronto_core
                                        pronto_ros
                                        pronto_quadruped
                                        geometry_msgs
                                        sensor_msgs
                                        pronto_msgs
                                        visualization_msgs
                                        eigen_conversions)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS pronto_core
                              pronto_quadruped
                              geometry_msgs
                              sensor_msgs
                              pronto_msgs
                              visualization_msgs
                              eigen_conversions)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/legodo_handler_ros.cpp
                            src/stance_estimator_ros.cpp
                            src/force_sensor_stance_estimator_ros.cpp
                            src/foot_sensor_stance_detector_ros.cpp
                            src/conversions.cpp
                            src/leg_odometer_ros.cpp
                            src/dynamic_stance_estimator_ros.cpp
                            src/bias_lock_handler_ros.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pronto_quadruped_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

