cmake_minimum_required(VERSION 3.0.2)
project(pronto_anymal_b)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        pronto_ros
                                        pronto_quadruped_ros
                                        pronto_anymal_b_commons
                                        sensor_msgs)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(NODE_NAME ${PROJECT_NAME}_node)

add_executable(${NODE_NAME} src/pronto_anymal_node.cpp)
target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})

install(TARGETS ${NODE_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/pose_to_tf.py 
                               scripts/publish_csv_as_path.py 
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

