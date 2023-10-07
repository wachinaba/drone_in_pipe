#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "multizone_lidar/multizone_lidar_converter.h"

class Pointcloud1ToRangeNode
{
public:
  Pointcloud1ToRangeNode();
  void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);

private:
  ros::NodeHandle nh;
  ros::Publisher range_pub;
  ros::Subscriber sub;
};

Pointcloud1ToRangeNode::Pointcloud1ToRangeNode() : nh()
{
  range_pub = nh.advertise<sensor_msgs::Range>("range", 1);
  sub = nh.subscribe("pointcloud", 1, &Pointcloud1ToRangeNode::pointCloudCallback, this);
}

void Pointcloud1ToRangeNode::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  sensor_msgs::Range range;
  range.header = msg->header;
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = 0.1;
  range.min_range = 0.1;
  range.max_range = 4.0;

  sensor_msgs::PointCloud2::Ptr msg_pointcloud2 = boost::make_shared<sensor_msgs::PointCloud2>();
  sensor_msgs::convertPointCloudToPointCloud2(*msg, *msg_pointcloud2);

  range.range = MultizoneLidarConverter::convertToRange(msg_pointcloud2);
  range_pub.publish(range);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud2_to_range_node");
  Pointcloud1ToRangeNode node;
  ros::spin();
  return 0;
}