cmake_minimum_required(VERSION 2.8.3)
project(pronto_quadruped)

set(CMAKE_CXX_STANDARD 14)

set(DEFAULT_BUILD "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
   message(STATUS "Setting build type to '${DEFAULT_BUILD}' as none was specified.")
   set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD} CACHE STRING "Choose the type of build." FORCE)
   # Set the possible values of build type for cmake-gui
   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug"
                                                        "Release"
                                                        "MinSizeRel"
                                                        "RelWithDebInfo")
endif()

set(BUILD_DOC false  CACHE BOOL "Build Doxygen documentation")

if(${BUILD_DOC})
    # check if Doxygen is installed
    find_package(Doxygen QUIET)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

        # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message(WARNING "Doxygen need to be installed to generate the Doxygen documentation")
    endif (DOXYGEN_FOUND)
endif()


# Including the directory containing the sources
include_directories(include)

find_package(catkin REQUIRED pronto_quadruped_commons pronto_core)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(LIBRARIES ${PROJECT_NAME} 
               INCLUDE_DIRS include
               CATKIN_DEPENDS pronto_quadruped_commons 
               DEPENDS EIGEN3)

add_library(${PROJECT_NAME} SHARED src/LegOdometer.cpp
                                   src/DynamicStanceEstimator.cpp
                                   src/ForceSensorStanceEstimator.cpp
                                   src/FootSensorStanceDetector.cpp
                                   src/StanceEstimator.cpp
                                   src/FlexEstimator.cpp
                                   src/DataLogger.cpp
                                   src/ImuBiasLock.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

