#ifndef LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP
#define LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP

#include <string>
#include <vector>
#include "lidar_camera_projection/detections.hpp"

bool loadDetectionsNormalized(const std::string& path,
                               NormalizedDetectionsOutput& out);

// Load detections from new format: bbox is an array [xmin, ymin, xmax, ymax]
bool loadDetectionsPixelNew(const std::string& json_str, DetectionsOutput& out);

#endif  // LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP