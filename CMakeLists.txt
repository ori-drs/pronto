cmake_minimum_required(VERSION 2.8.3)

project(pronto_estimator_core)
add_compile_options(-std=c++11)

#required by the lcm-based state estimator library
find_package(catkin REQUIRED COMPONENTS pronto_lcmtypes
                                        bot_core
                                        bot_frames)
include_directories(${catkin_INCLUDE_DIRS})

# required by core pronto library
find_package(stl_utils)
include_directories(${stl_utils_INCLUDE_DIRS})

find_package(eigen_utils)
include_directories(${eigen_utils_INCLUDE_DIRS})

# required by estimate filter tools
find_package(bot_core_lcmtypes)
include_directories(${bot_core_lcmtypes_INCLUDE_DIRS})

# required by everybody
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(PRONTO_CORE_LIB prontocore)
set(STATE_EST_LIB mav-state-est)
set(ESTIMATE_TOOLS_LIB estimate_tools)

include_directories(include)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PRONTO_CORE_LIB} ${STATE_EST_LIB} ${ESTIMATE_TOOLS_LIB}
               CATKIN_DEPENDS bot_core bot_frames eigen_utils stl_utils
               DEPENDS EIGEN3)

# this library contains only computation, no dependency on LCM
add_library (${PRONTO_CORE_LIB} src/mav_state_est/rbis.cpp
                                src/mav_state_est/ins_module.cpp
                                src/mav_state_est/rbis_update_interface.cpp
                                src/mav_state_est/mav_state_est.cpp
                                src/mav_state_est/update_history.cpp)
target_link_libraries(${PRONTO_CORE_LIB} ${stl_utils_LIBRRARIES}
                                         ${eigen_utils_LIBRARIES})

# this library mainly contains filters for the Atlas IMU
add_library(${ESTIMATE_TOOLS_LIB} src/estimate_tools/iir_notch.cpp
                                  src/estimate_tools/imu_stream.cpp)
add_dependencies(${ESTIMATE_TOOLS_LIB} ${bot_core_lcmtypes_EXPORTED_TARGETS})

# this library contains the handling of signals through LCM
# TODO gradually move the computation from handlers to lcm-independent modules
# in the PRONTO_CORE_LIB library
add_library(${STATE_EST_LIB} src/mav_state_est/lcm_front_end.cpp
                             src/mav_state_est/rbis_initializer.cpp
                             src/mav_state_est/gps_handler.cpp
                             src/mav_state_est/index_meas_handler.cpp
                             src/mav_state_est/ins_handler.cpp
                             src/mav_state_est/atlas_ins_handler.cpp
                             src/mav_state_est/scan_matcher_handler.cpp
                             src/mav_state_est/vicon_handler.cpp
                             src/mav_state_est/optical_flow_handler.cpp)

add_dependencies(${STATE_EST_LIB} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${STATE_EST_LIB} ${PRONTO_CORE_LIB} ${catkin_LIBRARIES})

catkin_add_gtest(bot_trans_test test/bot_trans_eigen_conversion_test.cpp)
target_link_libraries(bot_trans_test ${STATE_EST_LIB} ${catkin_LIBRARIES} pthread)


install(TARGETS ${TARGETS}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


# Mark cpp header files for installation
install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
