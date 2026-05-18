#include "lidar_camera_projection/transform_utils.hpp"
#include <fstream>
#include <sstream>
#include <ros/ros.h>

static double getDouble(const std::string& json, const std::string& key) {
  size_t pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return 0.0;
  size_t colon = json.find(':', pos);
  size_t start = json.find_first_not_of(" \t\n", colon + 1);
  size_t end = json.find_first_of(",}\n", start);
  std::string val = json.substr(start, end - start);
  val.erase(remove_if(val.begin(), val.end(), ::isspace), val.end());
  return std::stod(val);
}

static std::string extractArray(const std::string& json, const std::string& key) {
  size_t start = json.find("\"" + key + "\"");
  if (start == std::string::npos) return "";
  size_t pos = json.find('[', start);
  if (pos == std::string::npos) return "";
  int brace_count = 0;
  std::string result;
  for (size_t i = pos; i < json.size(); ++i) {
    char c = json[i];
    result += c;
    if (c == '{') brace_count++;
    else if (c == '}') brace_count--;
    else if (c == ']' && brace_count == 0) return result;
  }
  return result;
}

static std::vector<double> parseDoubleArray(const std::string& arr_str) {
  std::vector<double> values;
  size_t start = arr_str.find('[');
  size_t end = arr_str.find(']');
  if (start == std::string::npos || end == std::string::npos) return values;
  std::string inner = arr_str.substr(start + 1, end - start - 1);
  std::stringstream ss(inner);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(remove_if(item.begin(), item.end(), ::isspace), item.end());
    if (!item.empty()) values.push_back(std::stod(item));
  }
  return values;
}

bool TransformMatrixBuilder::loadCalib(const std::string& calib_path, int img_width, int img_height) {
  img_width_ = img_width;
  img_height_ = img_height;
  std::ifstream ifs(calib_path);
  if (!ifs.is_open()) {
    ROS_ERROR("[Transform] Cannot open calib file: %s", calib_path.c_str());
    return false;
  }

  std::stringstream ss;
  ss << ifs.rdbuf();
  std::string json = ss.str();

  std::string results_obj;
  {
    size_t start = json.find("\"results\"");
    if (start == std::string::npos) return false;
    size_t brace_start = json.find('{', start);
    int bc = 0;
    for (size_t i = brace_start; i < json.size(); ++i) {
      if (json[i] == '{') bc++;
      else if (json[i] == '}') {
        bc--;
        if (bc == 0) {
          results_obj = json.substr(brace_start, i - brace_start + 1);
          break;
        }
      }
    }
  }

  std::string T_arr_str = extractArray(results_obj, "T_lidar_camera");
  std::vector<double> vals = parseDoubleArray(T_arr_str);
  if (vals.size() != 7) {
    ROS_ERROR("[Transform] T_lidar_camera must have 7 values (x,y,z,qx,qy,qz,qw), got %zu", vals.size());
    return false;
  }

  double x = vals[0], y = vals[1], z = vals[2];
  double qx = vals[3], qy = vals[4], qz = vals[5], qw = vals[6];

  Eigen::Quaterniond q(qw, qx, qy, qz);
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Vector3d t(x, y, z);

  T_lidar_camera_.setIdentity();
  T_lidar_camera_.block<3, 3>(0, 0) = R;
  T_lidar_camera_.block<3, 1>(0, 3) = t;

  T_camera_lidar_ = T_lidar_camera_.inverse();

  ROS_INFO("[Transform] T_lidar_camera loaded (qx=%.4f qy=%.4f qz=%.4f qw=%.4f)",
          qx, qy, qz, qw);
  ROS_INFO("[Transform] T_camera_lidar (inverse) built");

  // parse intrinsics from "camera" -> "intrinsics" [fx, fy, cx, cy]
  {
    size_t cam_start = json.find("\"camera\"");
    if (cam_start != std::string::npos) {
      size_t intr_start = json.find("\"intrinsics\"", cam_start);
      if (intr_start != std::string::npos) {
        size_t arr_start = json.find('[', intr_start);
        size_t arr_end = json.find(']', arr_start);
        if (arr_start != std::string::npos && arr_end != std::string::npos) {
          std::string inner = json.substr(arr_start + 1, arr_end - arr_start - 1);
          std::stringstream ss(inner);
          std::string item;
          std::vector<double> intr;
          while (std::getline(ss, item, ',')) {
            item.erase(remove_if(item.begin(), item.end(), ::isspace), item.end());
            if (!item.empty()) intr.push_back(std::stod(item));
          }
          if (intr.size() == 4) {
            fx_ = intr[0];
            fy_ = intr[1];
            cx_ = intr[2];
            cy_ = intr[3];
          }
        }
      }
    }
  }
  ROS_INFO("[Transform] intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx_, fy_, cx_, cy_);
  ROS_INFO("[Transform] img_size: %d x %d", img_width_, img_height_);
  return true;
}

void TransformMatrixBuilder::transformCloud(const pcl::PointCloud<PointT>::Ptr& cloud_in,
                                           const pcl::PointCloud<PointT>::Ptr& cloud_camera_world) {
  cloud_camera_world->header = cloud_in->header;
  cloud_camera_world->reserve(cloud_in->size());

  Eigen::Matrix4d T = T_camera_lidar_;
  for (const auto& pt : cloud_in->points) {
    Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
    Eigen::Vector4d p_t = T * p;
    PointT pt_out;
    pt_out.x = p_t.x();
    pt_out.y = p_t.y();
    pt_out.z = p_t.z();
    pt_out.intensity = pt.intensity;
    cloud_camera_world->push_back(pt_out);
  }
}

void TransformMatrixBuilder::transformCloudToPixel(
    const pcl::PointCloud<PointT>::Ptr& cloud_camera_world,
    const pcl::PointCloud<PointT>::Ptr& cloud_pixel) {
  cloud_pixel->header = cloud_camera_world->header;
  cloud_pixel->reserve(cloud_camera_world->size());

  for (const auto& pt : cloud_camera_world->points) {
    float z = pt.z;
    if (z <= 0.05f) continue;

    float inv_z = 1.0f / z;
    float u = fx_ * pt.x * inv_z + cx_;
    float v = fy_ * pt.y * inv_z + cy_;

    if (u < 0 || u >= img_width_)  continue;
    if (v < 0 || v >= img_height_) continue;

    PointT pt_pix;
    pt_pix.x = u;
    pt_pix.y = v;
    pt_pix.z = z;
    pt_pix.intensity = pt.intensity;
    cloud_pixel->push_back(pt_pix);
  }
}
