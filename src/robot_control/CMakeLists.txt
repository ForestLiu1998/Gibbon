cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_control
  CATKIN_DEPENDS 
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
#  DEPENDS system_lib
)

include_directories( 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable( robot_test src/test.cpp )
target_link_libraries( robot_test ${catkin_LIBRARIES} )
add_dependencies( robot_test ${catkin_EXPORTED_TARGETS} )

# add_executable( pick_place src/pick_place.cpp )
# target_link_libraries( pick_place ${catkin_LIBRARIES} )
# add_dependencies( pick_place ${catkin_EXPORTED_TARGETS} )
