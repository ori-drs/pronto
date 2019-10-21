cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
project(pronto_math)

add_compile_options(-std=c++11)

find_package(catkin)

find_package(Eigen3)

catkin_package(INCLUDE_DIRS  include
               LIBRARIES pronto_math
               DEPENDS EIGEN3)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/pronto_math.cpp)

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
