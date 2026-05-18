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
│   ├── detections.hpp                      # 检测结果数据结构
│   └── detections_loader.hpp               # 检测加载函数声明
├── src/
│   ├── lidar_filter_node.cpp               # 节点1: 雷达点云过滤
│   ├── camera_frame_filter_node.cpp        # 节点2: 坐标系变换 & 像素投影
│   ├── transform_utils.cpp                 # TransformMatrixBuilder 实现
│   └── detections_loader.cpp              # 检测结果坐标转换实现
├── launch/
│   ├── lidar_filter.launch                 # 启动雷达过滤节点
│   └── lidar_camera_transform.launch       # 启动坐标变换节点
└── doc/                                    # 参考文档（不参与构建）
    ├── PLAN.md
    ├── DEVLOG.md
    └── LIDAR_CAMERA_PROJECTION.md
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
│  1. x>0 前向滤波     │          │          │  loadAndSaveDetections():    │
│  2. RANSAC 地面去除  │          │          │  读取归一化坐标 → 像素坐标   │
│  3. VoxelGrid 降采样 │          │          │  输出: detections_pixel.json │
│                      │          │          │                              │
│  输出: /livox/filtered│         │          └──────────────────────────────┘
└──────────┬───────────┘          │
           │                      │
           ▼                      ▼
┌──────────────────────────────────────────────────────────────────┐
│  camera_frame_filter_node                                        │
│                                                                  │
│  cloudCallback():                                                │
│  1. transformCloud()       雷达帧 → 相机世界坐标                  │
│     输出: /livox/filtered_camera_frame                           │
│  2. transformCloudToPixel()  相机世界 → 像素坐标                  │
│     输出: /livox/filtered_pixel                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 4. 节点详细设计

### 4.1 lidar_filter_node

| 属性 | 值 |
|------|-----|
| 源文件 | `src/lidar_filter_node.cpp` |
| 主类 | `LidarFilterNode` |
| 依赖库 | `detections_loader` |

#### ROS 接口

| 接口 | 主题/类型 | 默认值 | 说明 |
|------|----------|--------|------|
| Subscribe | `sensor_msgs::PointCloud2` | `/livox/lidar` | 原始雷达点云输入 |
| Publish | `sensor_msgs::PointCloud2` | `/livox/filtered` | 过滤后点云输出 |
| Timer | `ros::Timer` | 0.1s 周期 | 定时刷新检测文件 |

#### 可配置参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `input_topic` | string | `/livox/lidar` | 输入点云主题 |
| `output_topic` | string | `/livox/filtered` | 输出点云主题 |
| `voxel_leaf_size` | double | `0.05` | 体素网格尺寸 (m) |
| `ransac_distance_threshold` | double | `0.05` | RANSAC 平面距离阈值 (m) |
| `ransac_max_iterations` | int | `100` | RANSAC 最大迭代次数 |
| `detections_input` | string | `.../data/detections.json` | 检测输入文件路径 |
| `detections_output` | string | `.../data/detections_pixel.json` | 检测输出文件路径 |
| `img_width` | int | `1920` | 目标图像宽度 |
| `img_height` | int | `1080` | 目标图像高度 |
| `detections_update_rate` | double | `0.1` | 检测文件刷新周期 (s) |

#### cloudCallback 处理链

```
PointCloud2 (ROS)
    │ pcl::fromROSMsg()
    ▼
PointCloud<PointXYZI> (PCL)
    │ 遍历所有点, 保留 pt.x > 0.0f
    ▼
cloud_x  (前向过滤)
    │ pcl::SACSegmentation (SACMODEL_PLANE + SAC_RANSAC)
    │ pcl::ExtractIndices (setNegative(true), 移除外点=地面)
    ▼
cloud_no_ground  (去地面)
    │ pcl::VoxelGrid (leaf_size 可配置)
    ▼
cloud_voxel  (降采样)
    │ pcl::toROSMsg()
    ▼
PointCloud2 → /livox/filtered
```

#### detectionsTimerCallback 处理链

```
detections.json (归一化坐标)
    │ loadAndSaveDetections()
    ▼
detections_pixel.json (像素坐标)
    │ 计数 class_id 变化
    ▼
ROS_INFO 日志输出
```

---

### 4.2 camera_frame_filter_node

| 属性 | 值 |
|------|-----|
| 源文件 | `src/camera_frame_filter_node.cpp` |
| 主类 | `CameraFrameFilterNode` |
| 依赖库 | `transform_utils` |

#### ROS 接口

| 接口 | 主题/类型 | 默认值 | 说明 |
|------|----------|--------|------|
| Subscribe | `sensor_msgs::PointCloud2` | `/livox/filtered` | 过滤后点云输入 |
| Publish | `sensor_msgs::PointCloud2` | `/livox/filtered_camera_frame` | 相机世界坐标点云 |
| Publish | `sensor_msgs::PointCloud2` | `/livox/filtered_pixel` (硬编码) | 像素坐标点云 |

#### 可配置参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `input_topic` | string | `/livox/filtered` | 输入点云主题 |
| `output_topic` | string | `/livox/filtered_camera_frame` | 相机世界输出主题 |
| `calib_path` | string | `.../data/calib.json` | 标定文件路径 |
| `img_width` | int | `1280` | 图像宽度 |
| `img_height` | int | `720` | 图像高度 |

#### cloudCallback 处理链

```
PointCloud2 (ROS)
    │ pcl::fromROSMsg()
    ▼
PointCloud<PointXYZI> (雷达帧)
    │ TransformMatrixBuilder::transformCloud()
    │ 对每个点: P_cam = T_camera_lidar * [x,y,z,1]^T
    ▼
cloud_camera_world  (相机世界坐标)
    │ pcl::toROSMsg() → /livox/filtered_camera_frame
    │
    │ TransformMatrixBuilder::transformCloudToPixel()
    │ 对每个点: u = fx*x/z + cx, v = fy*y/z + cy
    │ 过滤: z<=0.05 或超出图像范围的点
    ▼
cloud_pixel  (像素坐标, x=u, y=v, z=深度)
    │ pcl::toROSMsg() → /livox/filtered_pixel
    ▼
```

---

## 5. 库模块设计

### 5.1 transform_utils

| 属性 | 值 |
|------|-----|
| 头文件 | `include/lidar_camera_projection/transform_utils.hpp` |
| 实现文件 | `src/transform_utils.cpp` |
| 类型 | 共享库 (`libtransform_utils.so`) |
| 链接依赖 | `${catkin_LIBRARIES}` |

#### 类型别名

```cpp
typedef pcl::PointXYZI PointT;  // 点类型: x, y, z, intensity
```

#### 类: TransformMatrixBuilder

负责标定参数加载、坐标系变换和像素投影。

##### 私有成员

| 成员 | 类型 | 说明 |
|------|------|------|
| `T_lidar_camera_` | `Eigen::Matrix4d` | 加载的变换矩阵: **相机→雷达** |
| `T_camera_lidar_` | `Eigen::Matrix4d` | 逆矩阵: **雷达→相机** (用于实际变换) |
| `fx_, fy_, cx_, cy_` | `float` | 相机内参 (针孔模型) |
| `img_width_, img_height_` | `int` | 图像尺寸 (像素裁剪边界) |

##### 公有方法

| 方法 | 返回 | 功能 |
|------|------|------|
| `loadCalib(calib_path, img_width, img_height)` | `bool` | 从 calib.json 加载外参和内参 |
| `transformCloud(cloud_in, cloud_camera_world)` | `void` | 雷达坐标系 → 相机世界坐标 |
| `transformCloudToPixel(cloud_camera_world, cloud_pixel)` | `void` | 相机世界坐标 → 像素坐标 |
| `getInverseMatrix()` | `const Matrix4d&` | 返回 `T_camera_lidar_` |

##### 核心数学流程

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
  若 z <= 0.05: 跳过 (点在相机后方或太近)
  u = fx * x / z + cx
  v = fy * y / z + cy
  若 u,v 超出 [0, width) × [0, height): 跳过
  输出: (u, v, z, intensity)   // x=列坐标, y=行坐标, z=深度
```

**注意**: 当前投影为纯针孔模型，未实现镜头畸变校正（Brown-Conrady k1,k2,p1,p2,k3）。calib.json 中的 `distortion_coeffs` 字段不会被读取。

##### 内部辅助函数（文件级静态函数）

| 函数 | 功能 |
|------|------|
| `getDouble(json, key)` | 从 JSON 字符串中按 key 提取 double 值 |
| `extractArray(json, key)` | 从 JSON 字符串中提取 `[...]` 数组子串 |
| `parseDoubleArray(arr_str)` | 将 `[1.0, 2.0, ...]` 字符串解析为 `vector<double>` |

---

### 5.2 detections_loader

| 属性 | 值 |
|------|-----|
| 头文件 | `include/lidar_camera_projection/detections_loader.hpp` |
| 实现文件 | `src/detections_loader.cpp` |
| 类型 | 共享库 (`libdetections_loader.so`) |
| 链接依赖 | `${catkin_INCLUDE_DIRS}` (仅头文件依赖) |

#### 数据结构 (detections.hpp)

```
BBox4Corners                     Detection
├── xmin, ymin (左上)            ├── class_id: int
├── xmax, ymax (右下)            ├── bbox_pixel: BBox4Corners
├── tl_x, tl_y (左上角点)        └── confidence: double
├── tr_x, tr_y (右上角点)
├── bl_x, bl_y (左下角点)        DetectionsOutput
├── br_x, br_y (右下角点)        ├── width: int
└── computeCorners()             ├── height: int
     由 min/max 推算四角         └── detections: vector<Detection>
```

#### 核心函数

```cpp
bool loadAndSaveDetections(input_path, output_path, img_width, img_height)
```

**处理流程**:

```
输入: detections.json (归一化坐标)
  {
    "resolution": {"width": W0, "height": H0},
    "detections": [
      {"class_id": N, "confidence": C,
       "bbox": {"x_center": Xc, "y_center": Yc, "width": Bw, "height": Bh}}
    ]
  }

转换公式 (归一化 → 像素):
  xmin = (x_center - width/2)  * img_width
  ymin = (y_center - height/2) * img_height
  xmax = (x_center + width/2)  * img_width
  ymax = (y_center + height/2) * img_height

输出: detections_pixel.json (像素坐标)
  {
    "resolution": {"width": img_width, "height": img_height},
    "detections": [
      {"class_id": N,
       "bbox_pixel": {"xmin":.., "ymin":.., "xmax":.., "ymax":..},
       "corners": {"tl": [x,y], "tr": [x,y], "bl": [x,y], "br": [x,y]},
       "confidence": C}
    ]
  }
```

**注意**: 该模块使用自定义的手写 JSON 解析器（非 nlohmann_json），只能解析特定格式的扁平整数/浮点字段。嵌套数组或非规则空白可能导致解析错误。

##### 内部辅助函数（文件级静态函数）

| 函数 | 功能 |
|------|------|
| `getDouble(json, key)` | 按 key 提取 double 值 |
| `getInt(json, key)` | 按 key 提取 int 值 |
| `extractObject(json, key)` | 提取 `{...}` JSON 对象子串（花括号计数） |
| `extractArray(json, key)` | 提取 `[...]` JSON 数组子串 |
| `splitObjects(array_str)` | 将数组字符串拆分为独立对象字符串列表 |

---

## 6. 文件间依赖关系图

```
┌─────────────────────────────────────────────────────────────────────┐
│  头文件层 (include/lidar_camera_projection/)                         │
│                                                                     │
│  detections.hpp ─── 定义 BBox4Corners, Detection, DetectionsOutput  │
│       │                                                             │
│       ├── detections_loader.hpp ─── 声明 loadAndSaveDetections()    │
│       │                                                             │
│       └── transform_utils.hpp ─── 声明 TransformMatrixBuilder       │
│                                     typedef PointT = PointXYZI      │
└─────────────────────────────────────────────────────────────────────┘
                      │                        │
          ┌───────────▼──────────┐  ┌──────────▼──────────────────┐
          │ src/detections_loader.cpp  │ src/transform_utils.cpp   │
          │  静态函数: getDouble,      │  静态函数: getDouble,      │
          │  getInt, extractObject,    │  extractArray,            │
          │  extractArray, splitObjects│  parseDoubleArray         │
          │                            │                           │
          │  库: libdetections_loader  │  库: libtransform_utils   │
          └───────────┬───────────────┘  └──────────┬──────────────┘
                      │                             │
          ┌───────────▼──────────────┐  ┌───────────▼─────────────────────┐
          │ src/lidar_filter_node.cpp│  │ src/camera_frame_filter_node.cpp│
          │                          │  │                                │
          │ class LidarFilterNode    │  │ class CameraFrameFilterNode    │
          │  - cloudCallback()       │  │  - cloudCallback()             │
          │  - detectionsTimerCb()   │  │                                │
          │                          │  │                                │
          │ 可执行文件:               │  │ 可执行文件:                     │
          │ lidar_filter_node        │  │ camera_frame_filter_node       │
          └──────────────────────────┘  └────────────────────────────────┘
```

---

## 7. CMake 构建目标

```
┌──────────────────────────────────────────────────────────────┐
│ find_package(catkin REQUIRED                                 │
│   COMPONENTS roscpp sensor_msgs pcl_ros pcl_conversions      │
│              tf2_eigen)   ← tf2_eigen 声明但未链接            │
│                                                              │
│ include_directories: ${catkin_INCLUDE_DIRS}, include/         │
└──────────────────────────────────────────────────────────────┘
         │
         ├── libdetections_loader.so
         │   └── src/detections_loader.cpp
         │
         ├── libtransform_utils.so
         │   └── src/transform_utils.cpp
         │   └── LINK: ${catkin_LIBRARIES}
         │
         ├── lidar_filter_node (可执行文件)
         │   └── src/lidar_filter_node.cpp
         │   └── LINK: detections_loader, ${catkin_LIBRARIES}
         │
         └── camera_frame_filter_node (可执行文件)
             └── src/camera_frame_filter_node.cpp
             └── LINK: transform_utils, ${catkin_LIBRARIES}
```

---

## 8. ROS 主题通信图

```
┌──────────────────┐
│  Livox 雷达驱动   │
│  (外部节点)       │
└────────┬─────────┘
         │ /livox/lidar (sensor_msgs::PointCloud2)
         ▼
┌─────────────────────┐
│  lidar_filter_node  │
│                     │
│  过滤管线:          │
│  x>0 → RANSAC →    │
│  VoxelGrid          │
│                     │
│  检测转换:          │
│  detections.json    │
│  → detections_pixel │
│  .json (文件IO)     │
└────────┬────────────┘
         │ /livox/filtered (sensor_msgs::PointCloud2)
         ▼
┌──────────────────────────────┐
│  camera_frame_filter_node    │
│                              │
│  calib.json ──► 外参+内参    │
│                              │
│  变换管线:                   │
│  雷达帧 → 相机世界 → 像素    │
│                              │
│  输出:                       │
│  /livox/filtered_camera_frame│
│  /livox/filtered_pixel       │
└──────────────────────────────┘
```

| 主题 | 方向 | 消息类型 | 发布者 | 订阅者 |
|------|------|----------|--------|--------|
| `/livox/lidar` | 外部→本系统 | `PointCloud2` | Livox 驱动 | `lidar_filter_node` |
| `/livox/filtered` | 内部 | `PointCloud2` | `lidar_filter_node` | `camera_frame_filter_node` |
| `/livox/filtered_camera_frame` | 输出 | `PointCloud2` | `camera_frame_filter_node` | 下游消费者 |
| `/livox/filtered_pixel` | 输出 | `PointCloud2` | `camera_frame_filter_node` | 下游消费者 |

**像素点云编码约定**: 在 `/livox/filtered_pixel` 中，PointXYZI 字段含义发生变化：
- `x` = u (像素列坐标)
- `y` = v (像素行坐标)  
- `z` = 深度 (相机坐标系下的 Z 值，单位: 米)
- `intensity` = 原始反射率 (保持不变)

---

## 9. 外部文件接口

| 文件 | 读写 | 使用者 | 格式 | 说明 |
|------|------|--------|------|------|
| `calib.json` | 读 | `camera_frame_filter_node` | JSON (TUM 格式) | 外参 T_lidar_camera + 内参 |
| `detections.json` | 读 | `lidar_filter_node` | JSON (YOLO 格式) | 归一化检测结果 |
| `detections_pixel.json` | 写 | `lidar_filter_node` | JSON (像素格式) | 转换后的检测结果 |

### calib.json 期望结构

```json
{
  "results": {
    "T_lidar_camera": [tx, ty, tz, qx, qy, qz, qw]
  },
  "camera": {
    "intrinsics": [fx, fy, cx, cy]
  }
}
```

- 平移 (tx, ty, tz) 单位: 米
- 四元数 (qw, qx, qy, qz) 为 Eigen 顺序 (w 在前)
- 变换语义: `T_lidar_camera` 将相机坐标系下的点变换到雷达坐标系

---

## 10. 启动方式

两个节点通常按序串联启动:

```bash
# 终端 1: 启动雷达过滤
roslaunch lidar_camera_projection lidar_filter.launch

# 终端 2: 启动坐标变换 (依赖 /livox/filtered 主题)
roslaunch lidar_camera_projection lidar_camera_transform.launch
```

或合并到上层 launch 文件中:

```xml
<include file="$(find lidar_camera_projection)/launch/lidar_filter.launch"/>
<include file="$(find lidar_camera_projection)/launch/lidar_camera_transform.launch"/>
```

---

## 11. 已知局限

1. **无畸变校正**: `transformCloudToPixel` 仅使用针孔模型 `u=fx*x/z+cx`，未读取 calib.json 中的 `distortion_coeffs` 字段
2. **硬编码主题**: `/livox/filtered_pixel` 主题名硬编码在 `camera_frame_filter_node.cpp:39`，不可通过参数配置
3. **手动 JSON 解析**: 两个 `.cpp` 文件中各有一份独立的 `getDouble` 实现（代码重复），且只支持特定 JSON 格式
4. **`tf2_eigen` 未使用**: CMakeLists.txt 和 package.xml 中声明了 `tf2_eigen` 依赖，但未链接到任何目标，也未在任何源文件中引用
5. **检测加载为文件轮询**: 通过 `ros::Timer` 定期读取磁盘文件，非实时 ROS 主题订阅，适合离线后处理场景

## 计划
- TODO： 将过滤简化
- TODO： 将点云字典化，方便提取