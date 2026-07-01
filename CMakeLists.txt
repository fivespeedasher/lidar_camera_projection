cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_projection)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs pcl_ros pcl_conversions tf2_eigen)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros pcl_conversions
)

include_directories(${catkin_INCLUDE_DIRS}
  include
)

add_library(detections_loader src/detections_loader.cpp)
target_include_directories(detections_loader PUBLIC ${catkin_INCLUDE_DIRS})

add_library(transform_utils src/transform_utils.cpp)
target_include_directories(transform_utils PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(transform_utils
  ${catkin_LIBRARIES}
)

add_library(point_lookup src/point_lookup.cpp)
target_include_directories(point_lookup PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(point_lookup
  ${catkin_LIBRARIES}
)

add_executable(lidar_filter_node src/lidar_filter_node.cpp)
add_dependencies(lidar_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_filter_node
  ${catkin_LIBRARIES}
)

add_executable(lidar_filter_pure_node src/lidar_filter_pure_node.cpp)
add_dependencies(lidar_filter_pure_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_filter_pure_node
  ${catkin_LIBRARIES}
)

add_executable(camera_frame_node src/camera_frame_node.cpp)
add_dependencies(camera_frame_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_frame_node
  transform_utils
  point_lookup
  detections_loader
  ${catkin_LIBRARIES}
)

add_executable(detections_publisher_node src/detections_publisher_node.cpp)
add_dependencies(detections_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(detections_publisher_node
  ${catkin_LIBRARIES}
)
