cmake_minimum_required(VERSION 2.8.3)
project(pronto_estimate_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS bot_core_lcmtypes)
find_package(Eigen3)

set(ESTIMATE_TOOLS_LIB estimate_tools)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${ESTIMATE_TOOLS_LIB}
               CATKIN_DEPENDS bot_core_lcmtypes
               DEPENDS EIGEN3)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# this library mainly contains filters for the Atlas IMU
add_library(${ESTIMATE_TOOLS_LIB} src/iir_notch.cpp
                                  src/imu_stream.cpp)

target_link_libraries(${ESTIMATE_TOOLS_LIB} ${catkin_LIBRARIES})

# the library depends on the generated LCM types from bot_core_lcmtypes
add_dependencies(${ESTIMATE_TOOLS_LIB} ${catkin_EXPORTED_TARGETS})


 install(TARGETS ${ESTIMATE_TOOLS_LIB} 
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

# TODO add tests
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pronto_estimate_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

