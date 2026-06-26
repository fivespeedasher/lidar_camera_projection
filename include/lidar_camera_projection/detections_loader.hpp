#ifndef LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP
#define LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP

#include <string>
#include <vector>
#include "lidar_camera_projection/detections.hpp"

bool loadDetectionsNormalized(const std::string& path,
                               NormalizedDetectionsOutput& out);

#endif  // LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP