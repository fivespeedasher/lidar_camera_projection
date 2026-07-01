#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <sstream>

class DetectionsPublisherNode {
public:
  DetectionsPublisherNode()
    : nh_("~")
  {
    std::string detections_path, output_topic;
    double update_rate;

    nh_.param<std::string>("detections_path", detections_path,
                           "/home/robot/projects/catkin_ws2/data/detections_pixel_new.json");
    nh_.param<std::string>("output_topic", output_topic, "/detections/pixel");
    nh_.param<double>("detections_update_rate", update_rate, 0.1);

    detections_path_ = detections_path;
    output_topic_ = output_topic;

    pub_ = nh_.advertise<std_msgs::String>(output_topic_, 10);

    timer_ = nh_.createTimer(ros::Duration(update_rate),
                             &DetectionsPublisherNode::timerCallback, this);

    ROS_INFO("[DetectionsPublisher] path: %s", detections_path_.c_str());
    ROS_INFO("[DetectionsPublisher] topic: %s  rate: %.2fs", output_topic_.c_str(), update_rate);

    // initial publish
    timerCallback(ros::TimerEvent());
  }

  void timerCallback(const ros::TimerEvent&)
  {
    std::ifstream ifs(detections_path_);
    if (!ifs.is_open()) return;

    std::stringstream ss;
    ss << ifs.rdbuf();
    std::string content = ss.str();

    // count detections for logging
    static int last_count = -1;
    size_t pos = 0, count = 0;
    while ((pos = content.find("\"class_id\"", pos)) != std::string::npos) {
      count++;
      pos++;
    }
    if (static_cast<int>(count) != last_count) {
      last_count = count;
      ROS_INFO("[DetectionsPublisher] detections count: %zu", count);
    }

    std_msgs::String msg;
    msg.data = content;
    pub_.publish(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Timer timer_;
  std::string detections_path_;
  std::string output_topic_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detections_publisher_node");
  DetectionsPublisherNode node;
  ros::spin();
  return 0;
}
