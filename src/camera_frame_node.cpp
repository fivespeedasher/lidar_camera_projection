#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include "lidar_camera_projection/transform_utils.hpp"
#include "lidar_camera_projection/point_lookup.hpp"
#include "lidar_camera_projection/detections_loader.hpp"

typedef pcl::PointXYZI PointT;

class CameraFrameFilterNode {
public:
  CameraFrameFilterNode()
    : nh_("~")
  {
    std::string input_topic, output_topic, calib_path;

    nh_.param<std::string>("input_topic", input_topic, "/livox/filtered");
    nh_.param<std::string>("output_topic", output_topic, "/livox/filtered_camera_frame");
    nh_.param<std::string>("calib_path", calib_path, "/home/robot/projects/catkin_ws2/data/calib.json");
    nh_.param<int>("img_width", img_width_, 1280);
    nh_.param<int>("img_height", img_height_, 720);
    nh_.param<std::string>("detections_path", detections_path_,
                           "/home/robot/projects/catkin_ws2/data/detections_pixel.json");

    input_topic_ = input_topic;
    output_topic_ = output_topic;
    calib_path_ = calib_path;

    if (!transform_builder_.loadCalib(calib_path_, img_width_, img_height_)) {
      ROS_FATAL("[CameraFrameFilter] Failed to load calibration!");
      return;
    }

    ROS_INFO("[CameraFrameFilter] input_topic: %s", input_topic_.c_str());
    ROS_INFO("[CameraFrameFilter] output_topic: %s", output_topic_.c_str());
    ROS_INFO("[CameraFrameFilter] calib_path: %s", calib_path_.c_str());
    ROS_INFO("[CameraFrameFilter] img_size: %d x %d", img_width_, img_height_);
    ROS_INFO("[CameraFrameFilter] detections_path: %s", detections_path_.c_str());

    sub_ = nh_.subscribe(input_topic_, 10, &CameraFrameFilterNode::cloudCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
    pub_pixel_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox/filtered_pixel", 10);
    pub_inbox_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox/inbox_voxel", 10);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud_in);

    pcl::PointCloud<PointT>::Ptr cloud_camera_world(new pcl::PointCloud<PointT>);
    transform_builder_.transformCloud(cloud_in, cloud_camera_world);

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_camera_world, msg_out);
    msg_out.header = msg->header;
    pub_.publish(msg_out);

    pcl::PointCloud<PointT>::Ptr cloud_pixel(new pcl::PointCloud<PointT>);
    transform_builder_.transformCloudToPixel(cloud_camera_world, cloud_pixel);

    sensor_msgs::PointCloud2 msg_pixel;
    pcl::toROSMsg(*cloud_pixel, msg_pixel);
    msg_pixel.header = msg->header;
    pub_pixel_.publish(msg_pixel);

    // Build 2D spatial grid and query detection boxes
    lookup_.build(cloud_pixel, img_width_, img_height_);

    DetectionsOutput detections;
    if (loadDetectionsPixel(detections_path_, detections)) {
      pcl::PointCloud<PointT>::Ptr inbox_all(new pcl::PointCloud<PointT>);
      for (const auto& det : detections.detections) {
        pcl::PointCloud<PointT>::Ptr inbox(new pcl::PointCloud<PointT>);
        lookup_.queryBox(cloud_pixel, cloud_in,
                         det.bbox_pixel.xmin, det.bbox_pixel.ymin,
                         det.bbox_pixel.xmax, det.bbox_pixel.ymax,
                         inbox);
        ROS_INFO("[CameraFrameFilter] class %d: %zu inbox points",
                 det.class_id, inbox->points.size());
        *inbox_all += *inbox;
      }
      ROS_INFO("[CameraFrameFilter] total inbox points: %zu", inbox_all->points.size());

      sensor_msgs::PointCloud2 msg_inbox;
      pcl::toROSMsg(*inbox_all, msg_inbox);
      msg_inbox.header = msg->header;
      pub_inbox_.publish(msg_inbox);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  TransformMatrixBuilder transform_builder_;
  PointLookup lookup_;

  std::string input_topic_;
  std::string output_topic_;
  std::string calib_path_;
  std::string detections_path_;
  int img_width_;
  int img_height_;
  ros::Publisher pub_pixel_;
  ros::Publisher pub_inbox_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_frame_node");
  CameraFrameFilterNode node;
  ros::spin();
  return 0;
}
