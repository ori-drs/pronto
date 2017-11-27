cmake_minimum_required(VERSION 2.8.3)

project(pronto_estimator_core)

find_package(catkin REQUIRED COMPONENTS
  pronto_lcmtypes
  bot_core_lcmtypes
  bot_core
  bot_frames
  stl_utils
  eigen_utils
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES mav-state-est estimate_tools
  CATKIN_DEPENDS bot_core bot_frames eigen_utils stl_utils
  DEPENDS Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_subdirectory(src/estimate_tools)

add_subdirectory(src/mav_state_est)

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})