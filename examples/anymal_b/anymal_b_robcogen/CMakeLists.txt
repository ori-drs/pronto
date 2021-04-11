cmake_minimum_required(VERSION 3.0.2)
project(anymal_b_robcogen)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS pronto_quadruped_commons)

set(LIB_NAME ${PROJECT_NAME})

find_package(Eigen3)

catkin_package(INCLUDE_DIRS include
               LIBRARIES anymal_b_robcogen
               CATKIN_DEPENDS pronto_quadruped_commons)

include_directories(include 
                    ${catkin_INCLUDE_DIRS})

include_directories(./include)
include_directories(./include/${PROJECT_NAME})

set(SOURCES ${CMAKE_SOURCE_DIR}/src/transforms.cpp
            ${CMAKE_SOURCE_DIR}/src/jacobians.cpp
            ${CMAKE_SOURCE_DIR}/src/jsim.cpp
            ${CMAKE_SOURCE_DIR}/src/inverse_dynamics.cpp
            ${CMAKE_SOURCE_DIR}/src/forward_dynamics.cpp
            ${CMAKE_SOURCE_DIR}/src/inertia_properties.cpp
            ${CMAKE_SOURCE_DIR}/src/miscellaneous.cpp)


catkin_package(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
               LIBRARIES ${LIB_NAME}
               DEPENDS EIGEN3)

# Add library
add_library(${LIB_NAME} SHARED ${SOURCES})

install(TARGETS ${LIB_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


