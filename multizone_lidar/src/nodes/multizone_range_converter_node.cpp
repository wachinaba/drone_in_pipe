#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <multizone_lidar_msgs/MultizoneRange.h>

#include "multizone_lidar/multizone_lidar_converter.h"

class MultizoneRangeConverterNode
{
public:
  MultizoneRangeConverterNode();
  void multizoneRangeCallback(const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg);

private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Publisher pointcloud_pub;
  ros::Publisher range_pub;
  ros::Subscriber sub;
  std::string frame_id;
};

MultizoneRangeConverterNode::MultizoneRangeConverterNode() : nh(), pnh("~")
{
  pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
  range_pub = nh.advertise<sensor_msgs::Range>("range", 1);
  sub = nh.subscribe("multizone_range", 1, &MultizoneRangeConverterNode::multizoneRangeCallback, this);
  if (!pnh.getParam("frame_id", frame_id))
  {
    ROS_WARN("Failed to get param 'frame_id'. fallback to empty string.");
    frame_id = "";
  }
}

void MultizoneRangeConverterNode::multizoneRangeCallback(const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 pointcloud;
  pointcloud.header = msg->header;
  MultizoneLidarConverter::convertToPointCloud2(msg, pointcloud);

  sensor_msgs::Range range;
  range.header = msg->header;
  if (!frame_id.empty())
  {
    range.header.frame_id = frame_id;
  }

  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = msg->horizontal_fov;
  range.min_range = msg->min_range;
  range.max_range = msg->max_range;
  range.range = MultizoneLidarConverter::convertToRange(msg);

  pointcloud_pub.publish(pointcloud);
  range_pub.publish(range);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multizone_range_converter_node");
  MultizoneRangeConverterNode node;
  ros::spin();
  return 0;
}