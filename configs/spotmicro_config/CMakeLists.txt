cmake_minimum_required(VERSION 2.8.3)
project(spotmicro_config)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  i2cpwm_board
  joint_limits_interface
  roscpp
  tr1cpp
  transmission_interface
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    i2cpwm_board
    joint_limits_interface
    roscpp
    tr1cpp
    transmission_interface
    trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_library(hardware_interface src/hardware_interface.cpp)
add_dependencies(hardware_interface tr1cpp)
target_link_libraries(hardware_interface ${catkin_LIBRARIES} ${i2cpwm_board_LIBRARIES})

add_executable(hardware_interface_node src/hardware_interface_node.cpp)
add_dependencies(hardware_interface_node hardware_interface)

target_link_libraries(hardware_interface_node
	hardware_interface
	${catkin_LIBRARIES})
