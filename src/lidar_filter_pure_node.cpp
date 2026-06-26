#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZI PointT;

/**
 * Pure point-cloud filter node — no detections, no timers.
 *
 * Pipeline:
 *   1. Forward filter (x > 0.1)
 *   2. RANSAC ground-plane removal
 *   3. VoxelGrid downsampling
 *
 * Reads /livox/lidar → publishes /livox/filtered  (configurable).
 */
class LidarFilterPureNode {
public:
  LidarFilterPureNode()
    : nh_("~")
  {
    std::string input_topic, output_topic;

    nh_.param<std::string>("input_topic", input_topic, "/livox/lidar");
    nh_.param<std::string>("output_topic", output_topic, "/livox/filtered");
    nh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.1);
    nh_.param<double>("ransac_distance_threshold", ransac_distance_threshold_, 0.07);
    nh_.param<int>("ransac_max_iterations", ransac_max_iterations_, 50);

    ROS_INFO("[LidarFilterPure] input: %s → output: %s  voxel: %.3f  ransac_dist: %.3f  ransac_iter: %d",
             input_topic.c_str(), output_topic.c_str(),
             voxel_leaf_size_, ransac_distance_threshold_, ransac_max_iterations_);

    sub_ = nh_.subscribe(input_topic, 10, &LidarFilterPureNode::cloudCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud_in);

    // Step 1: x > 0 filter (remove points behind or too close to sensor)
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

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  double voxel_leaf_size_;
  double ransac_distance_threshold_;
  int ransac_max_iterations_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_filter_pure_node");
  LidarFilterPureNode node;
  ros::spin();
  return 0;
}
