# run sample
1. 播放录好的点云与相机数据：rosbag play -l *22.bag
2. 启动过滤节点：roslaunch lidar_camera_projection lidar_filter.launch
3. 启动投影节点：roslaunch lidar_camera_projection lidar_camera_transform.launch
