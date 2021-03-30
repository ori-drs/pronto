cmake_minimum_required(VERSION 3.0.2)
project(pronto_biped_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  pronto_biped_core
  pronto_ros
  sensor_msgs
  kdl_parser
  tf2_eigen
)

find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS pronto_biped_core sensor_msgs kdl_parser tf2_eigen
               DEPENDS EIGEN3)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/pronto_biped_ros.cpp
                            src/legodo_handler_ros.cpp
                            src/yaw_lock_handler_ros.cpp
                            src/treefksolverposfull_recursive.cpp
                            src/forward_kinematics_ros.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} # ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp")

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pronto_biped_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
