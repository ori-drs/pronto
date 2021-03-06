cmake_minimum_required(VERSION 3.0.2)
project(pronto_utils)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(include)

set(FILTER_TOOLS_LIB ${PROJECT_NAME})
set(BACKLASH_FILTER_LIB backlash_filter)
set(KALMAN_FILTER_LIB kalman_filter)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${FILTER_TOOLS_LIB}
                         ${KALMAN_FILTER_LIB}
                         ${BACKLASH_FILTER_LIB}
               DEPENDS EIGEN3)

add_library(${PROJECT_NAME} src/Filter.cpp
                            src/HeavyLowPassFilter.cpp
                            #src/SignalTap.cpp
                            src/DataFileLogger.cpp
                            src/BipolarSchmittTrigger.cpp
                            src/MedianFilter.cpp
                            src/RateChange.cpp
                            src/TrapezoidalInt.cpp
                            src/DistributedDiff.cpp
                            src/NumericalDiff.cpp
                            src/ExpireTimer.cpp
                            src/SchmittTrigger.cpp
                            src/BipolarSchmittTrigger.cpp
                            src/Gaussian.cpp)

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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} ${KALMAN_FILTER_LIB} ${BACKLASH_FILTER_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


