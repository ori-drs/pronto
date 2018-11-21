cmake_minimum_required(VERSION 2.8.3)

project(pronto_estimator_core)

find_package(catkin REQUIRED COMPONENTS pronto_lcmtypes
                                        bot_core_lcmtypes
                                        bot_core
                                        bot_frames
                                        stl_utils
                                        eigen_utils)

find_package(Eigen3 REQUIRED)

set(STATE_EST_LIB mav-state-est)
set(ESTIMATE_TOOLS_LIB estimate_tools)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${STATE_EST_LIB} ${ESTIMATE_TOOLS_LIB}
               CATKIN_DEPENDS bot_core bot_frames eigen_utils stl_utils
                DEPENDS EIGEN3)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${ESTIMATE_TOOLS_LIB} src/estimate_tools/iir_notch.cpp
                                  src/estimate_tools/imu_stream.cpp)
add_dependencies(${ESTIMATE_TOOLS_LIB} ${catkin_EXPORTED_TARGETS})

add_library(${STATE_EST_LIB} src/mav_state_est/lcm_front_end.cpp
                             src/mav_state_est/mav_state_est.cpp
                             src/mav_state_est/rbis.cpp
                             src/mav_state_est/rbis_initializer.cpp
                             src/mav_state_est/rbis_update_interface.cpp
                             src/mav_state_est/gps_handler.cpp
                             src/mav_state_est/index_meas_handler.cpp
                             src/mav_state_est/ins_handler.cpp
                             src/mav_state_est/atlas_ins_handler.cpp
                             src/mav_state_est/scan_matcher_handler.cpp
                             src/mav_state_est/vicon_handler.cpp
                             src/mav_state_est/optical_flow_handler.cpp
                             src/mav_state_est/update_history.cpp)

add_dependencies(${STATE_EST_LIB} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${STATE_EST_LIB} ${catkin_LIBRARIES})


install(TARGETS ${TARGETS}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


# Mark cpp header files for installation
install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
