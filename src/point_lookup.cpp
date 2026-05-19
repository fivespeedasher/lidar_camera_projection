#include "lidar_camera_projection/point_lookup.hpp"
#include <algorithm>

void PointLookup::build(const pcl::PointCloud<PointT>::Ptr& cloud_pixel,
                        int img_width, int img_height, int cell_size) {
  cell_size_ = cell_size;
  img_width_ = img_width;
  grid_cols_ = (img_width + cell_size - 1) / cell_size;
  grid_rows_ = (img_height + cell_size - 1) / cell_size;
  grid_.assign(grid_cols_ * grid_rows_, Cell());

  for (size_t i = 0; i < cloud_pixel->points.size(); ++i) {
    int col = static_cast<int>(cloud_pixel->points[i].x) / cell_size;
    int row = static_cast<int>(cloud_pixel->points[i].y) / cell_size;
    if (col < 0 || col >= grid_cols_ || row < 0 || row >= grid_rows_) continue;
    grid_[row * grid_cols_ + col].indices.push_back(static_cast<uint32_t>(i));
  }
}

void PointLookup::queryBox(const pcl::PointCloud<PointT>::Ptr& cloud_pixel,
                           const pcl::PointCloud<PointT>::Ptr& cloud_voxel,
                           int xmin, int ymin, int xmax, int ymax,
                           pcl::PointCloud<PointT>::Ptr& out_cloud) const {
  out_cloud->clear();

  int col_start = std::max(0, xmin / cell_size_);
  int col_end   = std::min(grid_cols_ - 1, xmax / cell_size_);
  int row_start = std::max(0, ymin / cell_size_);
  int row_end   = std::min(grid_rows_ - 1, ymax / cell_size_);

  for (int row = row_start; row <= row_end; ++row) {
    for (int col = col_start; col <= col_end; ++col) {
      for (uint32_t idx : grid_[row * grid_cols_ + col].indices) {
        const auto& px = cloud_pixel->points[idx];
        if (px.x >= xmin && px.x <= xmax && px.y >= ymin && px.y <= ymax) {
          out_cloud->points.push_back(cloud_voxel->points[idx]);
        }
      }
    }
  }
}
