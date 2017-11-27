set(LIBRARY_NAME mav-state-est)

add_library(${LIBRARY_NAME} lcm_front_end.cpp
  mav_state_est.cpp
  rbis.cpp
  rbis_initializer.cpp
  rbis_update_interface.cpp
  sensor_handlers.cpp
  update_history.cpp)

include_directories(${catkin_INCLUDE_DIRS})
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})