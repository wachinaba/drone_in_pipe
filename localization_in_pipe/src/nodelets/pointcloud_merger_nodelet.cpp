#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>

namespace localization_in_pipe
{
class pointcloud_merger_nodelet : public nodelet::Nodelet
{
public:
  pointcloud_merger_nodelet(){};
  virtual ~pointcloud_merger_nodelet(){};

private:
  void onInit() override
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();
    int num_sensors;

    pnh.param("num_sensors", num_sensors, 0);
    if (num_sensors <= 0)
    {
      ROS_FATAL_STREAM("Invalid number of sensors: " << num_sensors);
      return;
    }

    pnh.param<float>("delay_threshold", delay_threshold_, 0.1);

    // Subscribe to sensor topics
    for (int i = 0; i < num_sensors; i++)
    {
      std::string topic_name = "/input_" + std::to_string(i);
      sub_sensors_.push_back(nh.subscribe<sensor_msgs::PointCloud2>(
          topic_name, 1, boost::bind(&pointcloud_merger_nodelet::sensorCallback, this, i, _1)));
    }

    // Advertise merged pointcloud topic
    pub_merged_ = nh.advertise<sensor_msgs::PointCloud2>("/merged_pointcloud", 1);

    // Set up timer for merging pointclouds
    double publish_rate;
    pnh.param("publish_rate", publish_rate, 15.0);
    timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate), &pointcloud_merger_nodelet::mergePointClouds, this);

    // Resize sensor_pointclouds_ to num_sensors
    sensor_pointclouds_.resize(num_sensors);
  }

  void sensorCallback(int sensor_index, const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // Store received pointcloud in vector
    sensor_pointclouds_[sensor_index] = msg;
  }

  void mergePointClouds(const ros::TimerEvent& event)
  {
    // Check if all pointclouds have been received
    if (std::count_if(sensor_pointclouds_.begin(), sensor_pointclouds_.end(),
                      [](const auto& pointcloud) { return pointcloud != nullptr; }) != sensor_pointclouds_.size())
    {
      ROS_WARN_STREAM("Not all pointclouds received yet");
      return;
    }

    // すべてのポイントクラウドのタイムスタンプの範囲がdelay_threshold_以下であることを確認する
    // Check if all pointclouds have timestamps within delay_threshold_
    auto minmax_timestamp = std::minmax_element(sensor_pointclouds_.begin(), sensor_pointclouds_.end(),
                                                [](const auto& pointcloud1, const auto& pointcloud2) {
                                                  return pointcloud1->header.stamp < pointcloud2->header.stamp;
                                                });

    if (minmax_timestamp.first == sensor_pointclouds_.end() || minmax_timestamp.second == sensor_pointclouds_.end() ||
        *minmax_timestamp.first == nullptr || *minmax_timestamp.second == nullptr)
    {
      ROS_FATAL_STREAM("Invalid timestamps");
      return;
    }

    if ((*minmax_timestamp.second)->header.stamp.toSec() - (*minmax_timestamp.first)->header.stamp.toSec() >
        delay_threshold_)
    {
      ROS_FATAL_STREAM("Timestamps of pointclouds differ by more than " << delay_threshold_ << " seconds");
      return;
    }

    // Merge pointclouds
    sensor_msgs::PointCloud2 merged_pointcloud;
    for (const auto& pointcloud : sensor_pointclouds_)
    {
      merged_pointcloud.width += pointcloud->width;
      merged_pointcloud.data.insert(merged_pointcloud.data.end(), pointcloud->data.begin(), pointcloud->data.end());
    }
    merged_pointcloud.header = (*minmax_timestamp.first)->header;
    merged_pointcloud.height = 1;
    merged_pointcloud.row_step = merged_pointcloud.width * merged_pointcloud.point_step;

    merged_pointcloud.fields = (*minmax_timestamp.first)->fields;
    merged_pointcloud.is_bigendian = (*minmax_timestamp.first)->is_bigendian;
    merged_pointcloud.point_step = (*minmax_timestamp.first)->point_step;
    merged_pointcloud.is_dense = (*minmax_timestamp.first)->is_dense;

    // Publish merged pointcloud
    pub_merged_.publish(merged_pointcloud);
  }

  std::vector<ros::Subscriber> sub_sensors_;
  ros::Publisher pub_merged_;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> sensor_pointclouds_;
  ros::Timer timer_;
  float delay_threshold_;
};
}  // namespace localization_in_pipe

PLUGINLIB_EXPORT_CLASS(localization_in_pipe::pointcloud_merger_nodelet, nodelet::Nodelet)