#ifndef LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP
#define LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP

#include <string>
#include <vector>

bool loadAndSaveDetections(const std::string& input_path,
                            const std::string& output_path,
                            int img_width,
                            int img_height);

#endif  // LIDAR_CAMERA_PROJECTION_DETECTIONS_LOADER_HPP