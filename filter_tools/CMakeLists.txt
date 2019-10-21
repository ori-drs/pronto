cmake_minimum_required(VERSION 2.8.3)
project(filter_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(FILTER_TOOLS_LIB ${PROJECT_NAME})
set(BACKLASH_FILTER_LIB backlash_filter)
set(KALMAN_FILTER_LIB kalman_filter)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${FILTER_TOOLS_LIB}
                         ${KALMAN_FILTER_LIB}
                         ${BACKLASH_FILTER_LIB}
               DEPENDS EIGEN3)

include_directories(include)

add_library(${PROJECT_NAME} src/Filter.cpp
                            src/HeavyLowPassFilter.cpp
                            src/SignalTap.cpp)

add_library(${KALMAN_FILTER_LIB} src/alpha_filter.cpp
                                 src/kalman_filter.cpp
                                 src/simple_kalman_filter.cpp)

add_library(${BACKLASH_FILTER_LIB} src/backlash_filter.cpp
                                   src/torque_adjustment.cpp)

target_link_libraries(${BACKLASH_FILTER_LIB} ${KALMAN_FILTER_LIB})

######################################################
#add_executable(test-kalman-filter src/kalman_filter_demo.cpp)
#target_link_libraries(test-kalman-filter ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(test-filter src/filtering_demo.cpp)
#target_link_libraries(test-filter ${catkin_LIBRARIES} backlash_filter_tools)


