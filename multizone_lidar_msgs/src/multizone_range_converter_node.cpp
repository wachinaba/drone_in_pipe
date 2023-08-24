#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <multizone_lidar_msgs/MultizoneRange.h>

class MultizoneRangeConverterNode
{
public:
  MultizoneRangeConverterNode();
  void multizone_range_cb(const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg);

  void convert_multizone_range_to_pointcloud2(const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg,
                                              sensor_msgs::PointCloud2& pointcloud);
  void convert_multizone_range_to_range(const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg,
                                        sensor_msgs::Range& range);

private:
  ros::NodeHandle nh;
  ros::Publisher pointcloud_pub;
  ros::Publisher range_pub;
  ros::Subscriber sub;
};

MultizoneRangeConverterNode::MultizoneRangeConverterNode()
{
  pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
  range_pub = nh.advertise<sensor_msgs::Range>("range", 1);
  sub = nh.subscribe("multizone_range", 1, &MultizoneRangeConverterNode::multizone_range_cb, this);
}

void MultizoneRangeConverterNode::multizone_range_cb(const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 pointcloud;
  sensor_msgs::Range range;
  convert_multizone_range_to_pointcloud2(msg, pointcloud);
  convert_multizone_range_to_range(msg, range);
  pointcloud_pub.publish(pointcloud);
  range_pub.publish(range);
}

void MultizoneRangeConverterNode::convert_multizone_range_to_pointcloud2(
    const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg, sensor_msgs::PointCloud2& pointcloud)
{
  pointcloud.header = msg->header;
  pointcloud.height = msg->vertical_samples;
  pointcloud.width = msg->horizontal_samples;
  pointcloud.is_bigendian = false;
  pointcloud.is_dense = false;
  pointcloud.point_step = 12;
  pointcloud.row_step = pointcloud.point_step * pointcloud.width;
  pointcloud.fields.resize(3);
  pointcloud.fields[0].name = "x";
  pointcloud.fields[0].offset = 0;
  pointcloud.fields[0].datatype = 7;
  pointcloud.fields[0].count = 1;
  pointcloud.fields[1].name = "y";
  pointcloud.fields[1].offset = 4;
  pointcloud.fields[1].datatype = 7;
  pointcloud.fields[1].count = 1;
  pointcloud.fields[2].name = "z";
  pointcloud.fields[2].offset = 8;
  pointcloud.fields[2].datatype = 7;
  pointcloud.fields[2].count = 1;

  pointcloud.data.resize(pointcloud.row_step * pointcloud.height);

  float horizontal_increment = msg->horizontal_fov / msg->horizontal_samples;
  float vertical_increment = msg->vertical_fov / msg->vertical_samples;

  float horizontal_angle = -(float)(msg->horizontal_samples - 1) / 2 * horizontal_increment;
  float vertical_angle = -(float)(msg->vertical_samples - 1) / 2 * vertical_increment;

  for (int i_v = 0; i_v < msg->vertical_samples; i_v++)
  {
    for (int i_h = 0; i_h < msg->horizontal_samples; i_h++)
    {
      float range = msg->ranges[i_v * msg->horizontal_samples + i_h];  // range in meters
      float x = range;
      float y = range * -tan(horizontal_angle);
      float z = range * -tan(vertical_angle);

      memcpy(&pointcloud.data[(i_v * pointcloud.width + i_h) * pointcloud.point_step], &x, 4);
      memcpy(&pointcloud.data[(i_v * pointcloud.width + i_h) * pointcloud.point_step + 4], &y, 4);
      memcpy(&pointcloud.data[(i_v * pointcloud.width + i_h) * pointcloud.point_step + 8], &z, 4);
      horizontal_angle += horizontal_increment;
    }
    vertical_angle += vertical_increment;
    horizontal_angle = -(float)(msg->horizontal_samples - 1) / 2 * horizontal_increment;
  }
}

void MultizoneRangeConverterNode::convert_multizone_range_to_range(
    const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg, sensor_msgs::Range& range)
{
  range.header = msg->header;
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = msg->horizontal_fov / msg->horizontal_samples;
  range.min_range = msg->min_range;
  range.max_range = msg->max_range;

  range.range = 0.0;

  // calculate range using center 2x2 pixels
  int center_horizontal_index = msg->horizontal_samples / 2 - 1;
  int center_vertical_index = msg->vertical_samples / 2 - 1;
  int center_index = center_vertical_index * msg->horizontal_samples + center_horizontal_index;
  range.range = msg->ranges[center_index] + msg->ranges[center_index + 1] +
                msg->ranges[center_index + msg->horizontal_samples] +
                msg->ranges[center_index + msg->horizontal_samples + 1];
  range.range /= 4;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multizone_range_to_pointcloud2");
  MultizoneRangeConverterNode multizone_range_to_pointcloud2;
  ros::spin();
}