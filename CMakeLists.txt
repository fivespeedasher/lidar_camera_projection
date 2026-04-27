cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_projection)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pcl_ros pcl_conversions tf2_eigen)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(${catkin_INCLUDE_DIRS}
  include
)

add_library(detections_loader src/detections_loader.cpp)
target_include_directories(detections_loader PUBLIC ${catkin_INCLUDE_DIRS})

add_executable(lidar_filter_node src/lidar_filter_node.cpp)
add_dependencies(lidar_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_filter_node
  detections_loader
  ${catkin_LIBRARIES}
)
