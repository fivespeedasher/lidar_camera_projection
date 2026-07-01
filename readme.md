# run sample
1. 播放录好的点云与相机数据：rosbag play -l *22.bag
1. 播放录好的检测数据，仅测试用
```
rosrun lidar_camera_projection detections_publisher_node
```
or
```
rosrun lidar_camera_projection detections_publisher_node \
    _detections_path:=/home/robot/projects/catkin_ws2/data/detections_pixel_new.json \
    _output_topic:=/detections/pixel \
    _detections_update_rate:=0.1
```
1. 启动过滤节点：roslaunch lidar_camera_projection lidar_filter.launch
1. 启动投影节点：roslaunch lidar_camera_projection lidar_camera_transform.launch
