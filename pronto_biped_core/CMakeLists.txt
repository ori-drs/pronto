cmake_minimum_required(VERSION 2.8.3)
project(pronto_biped_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS filter_tools
                                        pronto_math
                                        pronto_core)


set(LEGODO_LIB_NAME pronto_biped_legodo)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${LEGODO_LIB_NAME}
               CATKIN_DEPENDS filter_tools
                              pronto_math
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
