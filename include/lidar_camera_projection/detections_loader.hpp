#ifndef LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP
#define LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP

#include <string>
#include <vector>
#include "lidar_camera_projection/detections.hpp"

bool loadAndSaveDetections(const std::string& input_path,
                            const std::string& output_path,
                            int img_width,
                            int img_height);

bool loadDetectionsPixel(const std::string& path, DetectionsOutput& out);

#endif  // LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP