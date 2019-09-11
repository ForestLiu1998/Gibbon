cmake_minimum_required(VERSION 2.8.3)
project(robot_gazebo)

add_compile_options(-std=c++11)

find_package(gazebo REQUIRED)
find_package(ignition-msgs1 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package(
)

include_directories(
  plugins
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)



# add_library(gazebo_ros_lds SHARED plugins/gazebo_ros_lds.cpp)
# target_link_libraries(gazebo_ros_lds ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
# add_dependencies(gazebo_ros_lds ${catkin_EXPORTED_TARGETS})

# add_library(gazebo_ros_joint_state_publisher1 SHARED plugins/gazebo_ros_joint_state_publisher.cpp)
# target_link_libraries(gazebo_ros_joint_state_publisher1 ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
# add_dependencies(gazebo_ros_joint_state_publisher1 ${catkin_EXPORTED_TARGETS})

# add_library(gazebo_ros_depth_camera1 SHARED plugins/gazebo_ros_depth_camera.cpp)
# target_link_libraries(gazebo_ros_depth_camera1 ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
# add_dependencies(gazebo_ros_depth_camera1 ${catkin_EXPORTED_TARGETS})


# install(TARGETS gazebo_ros_lds gazebo_ros_joint_state_publisher1 gazebo_ros_depth_camera1
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


