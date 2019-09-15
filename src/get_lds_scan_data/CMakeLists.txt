cmake_minimum_required( VERSION 2.8.3 )
project( get_lds_scan_data )

add_compile_options( -std=c++11 )

find_package( catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  laser_geometry
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros laser_geometry tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_directories( ${PCL_LIBRARY_DIRS} )



add_executable( get_lds_data_rail src/get_lds_data_rail.cpp )
target_link_libraries( get_lds_data_rail ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

