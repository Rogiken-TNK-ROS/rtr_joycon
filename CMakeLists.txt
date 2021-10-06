cmake_minimum_required(VERSION 3.0.2)
project(rtr_joycon)

add_compile_options(-std=c++17)

set(ROS_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  radial_menu_msgs
  trajectory_msgs
  radial_menu_model
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  moveit_visual_tools
  tf2
  )

find_package(catkin REQUIRED COMPONENTS
  ${ROS_DEPENDS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${ROS_DEPENDS}
)


###########
# Library #
###########
add_library(keymap_config SHARED
  include/rtr_joycon/keymap_config.h
  src/keymap_config.cpp
  )
add_dependencies(keymap_config ${catkin_EXPORTED_TARGETS})
target_link_libraries(keymap_config ${catkin_LIBRARIES})

install(TARGETS keymap_config
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )  

set(ORIGINAL_LIBS
  base_control
  tohoku_jog_control
  mani_jog_control
  tohoku_ik_control
  )
foreach(lib ${ORIGINAL_LIBS})
  add_library(${lib} SHARED
    include/rtr_joycon/${lib}.h
    src/${lib}.cpp
    )
  add_dependencies(${lib} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${lib} keymap_config ${catkin_LIBRARIES})
  
  install(TARGETS ${lib}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )  
endforeach()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rtr_joycon
  include/rtr_joycon/rtr_joycon.h
  src/rtr_joycon.cpp
  src/rtr_joycon_node.cpp
  )
target_link_libraries(rtr_joycon
  ${ORIGINAL_LIBS}
  ${catkin_LIBRARIES})

#############
## Install ##
#############

set(EXEC_FILES
  rtr_joycon
  )

# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(DIRECTORIES
  # ./config/
  ./launch/
  )
# Install all files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
