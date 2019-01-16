cmake_minimum_required(VERSION 2.8.3)
project(pronto_estimator_lcm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS bot_core
                                        bot_frames
                                        pronto_conversions
                                        pronto_core
                                        pronto_lcmtypes
                                        pronto_estimate_tools)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# TODO this should be PROJECT_NAME
set(STATE_EST_LIB mav-state-est)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${STATE_EST_LIB}
               CATKIN_DEPENDS bot_core
                              bot_frames
                              pronto_conversions
                              pronto_core
                              pronto_lcmtypes
                              pronto_estimate_tools)

include_directories(include ${catkin_INCLUDE_DIRS})

# this library contains the handling of signals through LCM
# TODO gradually move the computation from handlers to lcm-independent modules
# in the PRONTO_CORE_LIB library
add_library(${STATE_EST_LIB} src/lcm_front_end.cpp
                             src/rbis_initializer.cpp
                             src/gps_handler.cpp
                             src/index_meas_handler.cpp
                             src/ins_handler.cpp
                             src/init_message_handler.cpp
                             src/atlas_ins_handler.cpp
                             src/scan_matcher_handler.cpp
                             src/vicon_handler.cpp
                             src/optical_flow_handler.cpp
                             src/rbis_fovis_update.cpp
                             src/pose_meas_handler.cpp)

add_dependencies(${STATE_EST_LIB} ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(bot_trans_test test/bot_trans_eigen_conversion_test.cpp)
target_link_libraries(bot_trans_test ${STATE_EST_LIB} ${catkin_LIBRARIES} pthread)


install(TARGETS ${TARGETS}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)

