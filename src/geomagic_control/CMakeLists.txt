cmake_minimum_required(VERSION 2.8.3)
project(geomagic_control)

find_package(catkin REQUIRED COMPONENTS 
 roscpp 
 std_msgs 
 geometry_msgs 
 genmsg 
 rosconsole 
 tf 
 urdf
 geomagic_description
)

add_message_files(FILES PhantomButtonEvent.msg OmniFeedback.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  geometry_msgs 
  genmsg 
  rosconsole 
  tf 
  urdf
  geomagic_description 
  sensor_msgs 
)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h HL/hl.h HDU/hdu.h)

if(NOT OPENHAPTICS_INCLUDE_DIR)
  set(OPENHAPTICS_MESSAGE "OPENHAPTICS was not found. If you do not have it you will not be able to use the Phantom Omni.")
  message(STATUS "${OPENHAPTICS_MESSAGE}") 
  return()
endif(NOT OPENHAPTICS_INCLUDE_DIR)
  
add_executable(set_torque src/set_torque.cpp)
add_dependencies(set_torque ${catkin_EXPORTED_TARGETS})
target_link_libraries(set_torque HD HDU rt ncurses ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/Node_tf.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



add_executable(torque_calc1 src/torque_calc1.cpp)
add_dependencies(torque_calc1 ${catkin_EXPORTED_TARGETS})
target_link_libraries(torque_calc1 HD HDU rt ncurses ${catkin_LIBRARIES})
