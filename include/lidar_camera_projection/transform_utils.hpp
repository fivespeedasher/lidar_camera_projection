#ifndef LIDAR_CAMERA_PROJECTION_TRANSFORM_UTILS_HPP
#define LIDAR_CAMERA_PROJECTION_TRANSFORM_UTILS_HPP

#include <string>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZI PointT;

class TransformMatrixBuilder {
public:
  bool loadCalib(const std::string& calib_path, int img_width, int img_height);
  void transformCloud(const pcl::PointCloud<PointT>::Ptr& cloud_in,
                      const pcl::PointCloud<PointT>::Ptr& cloud_camera_world);
  void transformCloudToPixel(const pcl::PointCloud<PointT>::Ptr& cloud_camera_world,
                             const pcl::PointCloud<PointT>::Ptr& cloud_pixel);

  const Eigen::Matrix4d& getInverseMatrix() const { return T_camera_lidar_; }

private:
  Eigen::Matrix4d T_lidar_camera_;   // loaded: [[R t][0 1]]  camera -> lidar
  Eigen::Matrix4d T_camera_lidar_;  // inverse  lidar -> camera

  float fx_ = 0.f, fy_ = 0.f, cx_ = 0.f, cy_ = 0.f;
  int img_width_ = 1280, img_height_ = 720;
};

#endif  // LIDAR_CAMERA_PROJECTION_TRANSFORM_UTILS_HPP
