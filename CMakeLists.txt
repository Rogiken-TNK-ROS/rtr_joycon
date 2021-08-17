cmake_minimum_required(VERSION 3.0.2)
project(rtr_joycon)

add_compile_options(-std=c++17)

set(ROS_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  radial_menu_msgs
  trajectory_msgs
  radial_menu_model
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
set(ORIGINAL_LIBS
  tohoku_control
  )
foreach(lib ${ORIGINAL_LIBS})
  add_library(${lib} SHARED
    include/rtr_joycon/${lib}.h
    src/${lib}.cpp
    )
  add_dependencies(${lib} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${lib} ${catkin_LIBRARIES})
  
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
