#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "multizone_lidar/multizone_lidar_converter.h"

class Pointcloud2ToRangeNode
{
public:
  Pointcloud2ToRangeNode();
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
  ros::NodeHandle nh;
  ros::Publisher range_pub;
  ros::Subscriber sub;
};

Pointcloud2ToRangeNode::Pointcloud2ToRangeNode() : nh()
{
  range_pub = nh.advertise<sensor_msgs::Range>("range", 1);
  sub = nh.subscribe("pointcloud", 1, &Pointcloud2ToRangeNode::pointCloudCallback, this);
}

void Pointcloud2ToRangeNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::Range range;
  range.header = msg->header;
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = 0.1;
  range.min_range = 0.1;
  range.max_range = 4.0;
  range.range = MultizoneLidarConverter::convertToRange(msg);
  range_pub.publish(range);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud2_to_range_node");
  Pointcloud2ToRangeNode node;
  ros::spin();
  return 0;
}