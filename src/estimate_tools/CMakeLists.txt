set(LIBRARY_NAME estimate_tools)

add_library(${LIBRARY_NAME} iir_notch.cpp imu_stream.cpp)
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})