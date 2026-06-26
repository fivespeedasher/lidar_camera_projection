#ifndef LIDAR_CAMERA_PROJECTION_POINT_LOOKUP_HPP
#define LIDAR_CAMERA_PROJECTION_POINT_LOOKUP_HPP

#include <vector>
#include <cstdint>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZI PointT;

class PointLookup {
public:
  void build(const pcl::PointCloud<PointT>::Ptr& cloud_pixel,
             int img_width, int img_height, int cell_size = 10);

  /**
   * Query points inside a detection box given in pixel coordinates.
   */
  void queryBox(const pcl::PointCloud<PointT>::Ptr& cloud_pixel,
                const pcl::PointCloud<PointT>::Ptr& cloud_voxel,
                int xmin, int ymin, int xmax, int ymax,
                pcl::PointCloud<PointT>::Ptr& out_cloud) const;

  /**
   * Query points inside a detection box given in normalized [0,1] coordinates.
   * Internally scales by (img_width_, img_height_) to pixel coords.
   */
  void queryBoxNormalized(const pcl::PointCloud<PointT>::Ptr& cloud_pixel,
                          const pcl::PointCloud<PointT>::Ptr& cloud_voxel,
                          double x_center, double y_center,
                          double width, double height,
                          pcl::PointCloud<PointT>::Ptr& out_cloud) const;

  /** Extract clusters via Euclidean distance and return only the largest one. */
  void extractLargestCluster(const pcl::PointCloud<PointT>::Ptr& cloud_in,
                             pcl::PointCloud<PointT>::Ptr& out_cloud,
                             double tolerance) const;

  double cluster_tolerance_ = 0.8;

private:
  struct Cell {
    std::vector<uint32_t> indices;
  };
  std::vector<Cell> grid_;
  int cell_size_ = 10;
  int grid_cols_ = 0;
  int grid_rows_ = 0;
  int img_width_ = 0;
  int img_height_ = 0;
};

#endif  // LIDAR_CAMERA_PROJECTION_POINT_LOOKUP_HPP
