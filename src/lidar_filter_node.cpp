#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZI PointT;

class LidarFilterNode {
public:
  LidarFilterNode()
    : nh_("~")
  {
<<<<<<< HEAD
    std::string input_topic, output_topic, detections_input;
=======
    std::string input_topic, output_topic;
>>>>>>> 8256c72... change the detection to publish-subscribe mode
    double voxel_leaf_size;

    nh_.param<std::string>("input_topic", input_topic, "/livox/lidar");
    nh_.param<std::string>("output_topic", output_topic, "/livox/filtered");
    nh_.param<double>("voxel_leaf_size", voxel_leaf_size, 0.05);
    nh_.param<double>("ransac_distance_threshold", ransac_distance_threshold_, 0.05);
    nh_.param<int>("ransac_max_iterations", ransac_max_iterations_, 100);

<<<<<<< HEAD
    nh_.param<std::string>("detections_input", detections_input, "/home/robot/projects/catkin_ws2/data/detections.json");
    nh_.param<int>("img_width", img_width_, 1920);
    nh_.param<int>("img_height", img_height_, 1080);
    nh_.param<double>("detections_update_rate", detections_update_rate_, 0.1);

    input_topic_ = input_topic;
    output_topic_ = output_topic;
    voxel_leaf_size_ = voxel_leaf_size;
    detections_input_ = detections_input;
=======
    input_topic_ = input_topic;
    output_topic_ = output_topic;
    voxel_leaf_size_ = voxel_leaf_size;
>>>>>>> 8256c72... change the detection to publish-subscribe mode

    ROS_INFO("[LidarFilter] input_topic: %s", input_topic_.c_str());
    ROS_INFO("[LidarFilter] output_topic: %s", output_topic_.c_str());
    ROS_INFO("[LidarFilter] voxel_leaf_size: %.3f  ransac_dist: %.3f  ransac_iter: %d",
             voxel_leaf_size_, ransac_distance_threshold_, ransac_max_iterations_);

    sub_ = nh_.subscribe(input_topic_, 10, &LidarFilterNode::cloudCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
<<<<<<< HEAD

    detections_timer_ = nh_.createTimer(ros::Duration(detections_update_rate_),
                                        &LidarFilterNode::detectionsTimerCallback, this);
    detectionsTimerCallback(ros::TimerEvent());  // initial load
  }

  void detectionsTimerCallback(const ros::TimerEvent&)
  {
    static int last_count = -1;
    NormalizedDetectionsOutput detections;
    if (loadDetectionsNormalized(detections_input_, detections)) {
      int count = static_cast<int>(detections.detections.size());
      if (count != last_count) {
        last_count = count;
        ROS_INFO("[LidarFilter] detections: img %dx%d  count: %d",
                 img_width_, img_height_, count);
      }
    }
=======
>>>>>>> 8256c72... change the detection to publish-subscribe mode
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud_in);

    // Step 1: x > 0 filter
    pcl::PointCloud<PointT>::Ptr cloud_x(new pcl::PointCloud<PointT>);
    cloud_x->reserve(cloud_in->size());
    for (const auto& pt : cloud_in->points) {
      if (pt.x > 0.1f) {
        cloud_x->push_back(pt);
      }
    }

    // Step 2: RANSAC ground plane removal
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_distance_threshold_);
    seg.setMaxIterations(ransac_max_iterations_);
    seg.setInputCloud(cloud_x);
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<PointT>::Ptr cloud_no_ground(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_x);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_ground);

    // Step 3: voxel filter
    pcl::PointCloud<PointT>::Ptr cloud_voxel(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud_no_ground);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(*cloud_voxel);

    // Step 4: publish
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_voxel, msg_out);
    msg_out.header = msg->header;
    pub_.publish(msg_out);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  std::string input_topic_;
  std::string output_topic_;
  double voxel_leaf_size_;
  double ransac_distance_threshold_;
  int ransac_max_iterations_;
<<<<<<< HEAD

  std::string detections_input_;
  int img_width_;
  int img_height_;
  double detections_update_rate_;
=======
>>>>>>> 8256c72... change the detection to publish-subscribe mode
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_filter_node");
  LidarFilterNode node;
  ros::spin();
  return 0;
}
