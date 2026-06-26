#ifndef LIDAR_CAMERA_PROJECTION_DETECTIONS_HPP
#define LIDAR_CAMERA_PROJECTION_DETECTIONS_HPP

#include <vector>
#include <string>

struct BBox4Corners {
  int xmin, ymin;
  int xmax, ymax;
  int tl_x, tl_y;   // top-left
  int tr_x, tr_y;   // top-right
  int bl_x, bl_y;   // bottom-left
  int br_x, br_y;   // bottom-right

  void computeCorners() {
    tl_x = xmin; tl_y = ymin;
    tr_x = xmax; tr_y = ymin;
    bl_x = xmin; bl_y = ymax;
    br_x = xmax; br_y = ymax;
  }
};

struct Detection {
  int class_id;
  BBox4Corners bbox_pixel;
  double confidence;
};

struct DetectionsOutput {
  int width;
  int height;
  std::vector<Detection> detections;
};

struct BBoxNormalized {
  double x_center;  // [0,1]
  double y_center;  // [0,1]
  double width;     // [0,1]
  double height;    // [0,1]
};

struct DetectionNormalized {
  int class_id;
  BBoxNormalized bbox;
  double confidence;
};

struct NormalizedDetectionsOutput {
  int width;
  int height;
  std::vector<DetectionNormalized> detections;
};

#endif  // LIDAR_CAMERA_PROJECTION_DETECTIONS_HPP
