cmake_minimum_required(VERSION 3.10.2)
project(pronto_anymal_b_commons)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS pronto_quadruped_commons	                                	
                                        anymal_b_robcogen)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS pronto_quadruped_commons anymal_b_robcogen)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/forward_kinematics.cpp
                            src/feet_contact_forces.cpp
                            src/feet_jacobians.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")

