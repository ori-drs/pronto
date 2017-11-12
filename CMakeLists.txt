cmake_minimum_required(VERSION 2.8.3)
project(pronto_estimator_core)

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES mav-state-est estimate_tools
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
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