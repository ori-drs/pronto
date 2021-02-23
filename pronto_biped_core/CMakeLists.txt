cmake_minimum_required(VERSION 3.0.2)
project(pronto_biped_core)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS filter_tools
                                        pronto_core)


set(LEGODO_LIB_NAME pronto_biped_legodo)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${LEGODO_LIB_NAME}
               CATKIN_DEPENDS filter_tools
                              pronto_core)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${LEGODO_LIB_NAME} src/leg_estimate.cpp 
                               src/foot_contact_classify.cpp
                               src/FootContact.cpp
                               src/FootContactAlt.cpp
                               src/legodo_module.cpp
                               src/legodo_common.cpp
                               src/yawlock_module.cpp
                               src/yawlock_common.cpp)

add_dependencies(${LEGODO_LIB_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${LEGODO_LIB_NAME} ${catkin_LIBRARIES})

install(TARGETS ${LEGODO_LIB_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_install_python(PROGRAMS python/state-sync-simple.py
                               python/test_pose_command.py
                               python/send-a-pose.py
                               python/state-sync-simple-multisense-standalone.py
                               python/send-fake-imu.py
                               python/send-fake-imu-ros.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
