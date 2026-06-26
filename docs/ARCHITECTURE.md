# lidar_camera_projection 项目架构文档

## 1. 项目概述

**功能**: 接收 Livox 激光雷达点云数据，经过过滤（前向滤波、地面去除、体素降采样）后，利用标定参数将点云从雷达坐标系变换到相机坐标系，并投影到像素平面。同时支持 YOLO 目标检测结果从归一化坐标到像素坐标的批量转换。

**包名**: `lidar_camera_projection` v0.1.0  
**路径**: `/home/robot/projects/catkin_ws2/src/lidar_camera_projection`  
**构建系统**: Catkin (ROS1)  

---

## 2. 项目文件结构

```
lidar_camera_projection/
├── CMakeLists.txt                          # 构建配置
├── package.xml                             # 包元数据
├── include/lidar_camera_projection/
│   ├── transform_utils.hpp                 # TransformMatrixBuilder 类声明
│   ├── point_lookup.hpp                    # PointLookup 类声明 (2D 空间网格)
│   ├── detections.hpp                      # 检测结果数据结构
│   └── detections_loader.hpp               # 检测加载函数声明
├── src/
│   ├── lidar_filter_node.cpp               # 节点1: 雷达点云过滤
│   ├── camera_frame_node.cpp        # 节点2: 坐标系变换 & 像素投影 & 检测框查询
│   ├── transform_utils.cpp                 # TransformMatrixBuilder 实现
│   ├── point_lookup.cpp                    # PointLookup 实现
│   └── detections_loader.cpp              # 检测结果坐标转换实现
├── launch/
│   ├── lidar_filter.launch                 # 启动雷达过滤节点
│   └── lidar_camera_transform.launch       # 启动坐标变换节点
└── doc/                                    # 参考文档（不参与构建）
    ├── ARCHITECTURE.md
    ├── decisions.md
    ├── progress.md
    └── troubleshooting.md
```

---

## 3. 整体数据处理流水线

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          数据输入                                            │
│                                                                             │
│  /livox/lidar                calib.json              detections.json        │
│  (PointCloud2, 雷达帧)       (标定参数)              (YOLO 归一化检测)       │
└──────────┬──────────────────────┬───────────────────────┬───────────────────┘
           │                      │                       │
           ▼                      │                       ▼
┌──────────────────────┐          │          ┌──────────────────────────────┐
│  lidar_filter_node   │          │          │  lidar_filter_node           │
│                      │          │          │  (detectionsTimerCallback)   │
│  cloudCallback():    │          │          │                              │
│  1. x>0.1 前向滤波     │          │          │  loadDetectionsNormalized(): │
│  2. RANSAC 地面去除  │          │          │  读取 detections.json 计数   │
│  3. VoxelGrid 降采样 │          │          │  (无文件写入)                │
│                      │          │          │                              │
│  输出: /livox/filtered│         │          └──────────────────────────────┘
└──────────┬───────────┘          │
           │                      │
           ▼                      ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  camera_frame_node                                                │
│                                                                          │
│  cloudCallback():                                                        │
│  1. transformCloud()       雷达帧 → 相机世界坐标                         │
│     输出: /livox/filtered_camera_frame                                   │
│  2. transformCloudToPixel()  相机世界 → 像素坐标 (保留所有点, N in=N out) │
│     输出: /livox/filtered_pixel                                          │
│  3. PointLookup::build()  构建 2D 空间网格 (像素坐标)                    │
│  4. loadDetectionsNormalized()  读取 detections.json (归一化坐标)        │
│  5. PointLookup::queryBoxNormalized()  对每个检测框查找框内点           │
│     输出: /livox/inbox_voxel                                             │
└──────────────────────────────────────────────────────────────────────────┘
```


## 4. 可配置参数

### 4.1 lidar_filter_node
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `input_topic` | string | `/livox/lidar` | 输入点云主题 |
| `output_topic` | string | `/livox/filtered` | 输出点云主题 |
| `voxel_leaf_size` | double | `0.05` | 体素网格尺寸 (m) |
| `ransac_distance_threshold` | double | `0.05` | RANSAC 平面距离阈值 (m) |
| `ransac_max_iterations` | int | `100` | RANSAC 最大迭代次数 |
| `detections_input` | string | `.../data/detections.json` | 检测输入文件路径 |
| `img_width` | int | `1920` | 目标图像宽度 |
| `img_height` | int | `1080` | 目标图像高度 |
| `detections_update_rate` | double | `0.1` | 检测文件刷新周期 (s) |



### 4.2 camera_frame_node
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `input_topic` | string | `/livox/filtered` | 输入点云主题 |
| `output_topic` | string | `/livox/filtered_camera_frame` | 相机世界输出主题 |
| `calib_path` | string | `.../data/calib.json` | 标定文件路径 |
| `img_width` | int | `1280` | 图像宽度 |
| `img_height` | int | `720` | 图像高度 |
| `detections_path` | string | `.../data/detections.json` | 检测输入文件路径 (归一化坐标) |
| `cluster_tolerance` | double | `0.8` | 欧几里得聚类容差 |

## 5. 核心数学流程

**标定加载 (loadCalib)**:
```
calib.json
  ├── "results" → "T_lidar_camera": [tx, ty, tz, qx, qy, qz, qw]
  │   ├── Eigen::Quaterniond(qw, qx, qy, qz) → R (旋转矩阵 3×3)
  │   ├── T_lidar_camera_ = [R  t]    (相机→雷达)
  │   │                     [0  1]
  │   └── T_camera_lidar_ = T_lidar_camera_.inverse()   (雷达→相机)
  └── "camera" → "intrinsics": [fx, fy, cx, cy]
      └── 存入 fx_, fy_, cx_, cy_
```

**坐标系变换 (transformCloud)**:
```
对雷达帧中的每个点 P_lidar = (x, y, z):
  P_cam_homo = T_camera_lidar_ * [x, y, z, 1]^T
  输出: (P_cam_homo.x, P_cam_homo.y, P_cam_homo.z, intensity)
```

**像素投影 (transformCloudToPixel)**:
```
对相机世界坐标中的每个点 P_cam = (x, y, z):
  u = fx * x / z + cx
  v = fy * y / z + cy
  输出: (u, v, z, intensity)   // x=列坐标, y=行坐标, z=深度
  注意: 当前版本保留所有点 (不过滤深度或边界), N in = N out
```