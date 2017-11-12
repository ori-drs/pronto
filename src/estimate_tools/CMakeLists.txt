set(LIBRARY_NAME estimate_tools)

add_library(${LIBRARY_NAME} iir_notch.cpp imu_stream.cpp)

pods_use_pkg_config_packages(${LIBRARY_NAME} lcmtypes_bot2-core eigen3)

add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})