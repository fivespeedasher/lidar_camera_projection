#include "lidar_camera_projection/point_lookup.hpp"
#include <algorithm>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

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

void PointLookup::extractLargestCluster(
    const pcl::PointCloud<PointT>::Ptr& cloud_in,
    pcl::PointCloud<PointT>::Ptr& out_cloud,
    double tolerance) const {
  out_cloud->clear();
  if (cloud_in->empty()) return;

  pcl::search::KdTree<PointT>::Ptr tree(
    new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<PointT> extraction;
  extraction.setClusterTolerance(tolerance);
  extraction.setMinClusterSize(3);
  extraction.setMaxClusterSize(1e6);
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(cloud_in);
  extraction.extract(clusters);

  if (clusters.empty()) return;

  size_t best_idx = 0;
  size_t best_size = 0;
  for (size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].indices.size() > best_size) {
      best_size = clusters[i].indices.size();
      best_idx = i;
    }
  }

  out_cloud->points.clear();
  for (const auto idx : clusters[best_idx].indices) {
    out_cloud->points.push_back(cloud_in->points[idx]);
  }
}
