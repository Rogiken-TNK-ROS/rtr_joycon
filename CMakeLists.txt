cmake_minimum_required(VERSION 2.8.3)
project(rtr_joycon)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  jog_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    std_msgs
    geometry_msgs
    jog_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rtr_joycon
  src/rtr_joycon.cpp
  src/rtr_joycon_state.cpp)
target_link_libraries(rtr_joycon
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
